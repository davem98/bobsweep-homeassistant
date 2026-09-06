"""DataUpdateCoordinator for the bObsweep (Tuya local protocol) integration.

**This coordinator listens; it does not merely poll.**

The robot volunteers almost everything interesting over its persistent socket
instead of answering questions about it. A passive `receive()` capture of a
real 22-minute job on the validation unit shows the shape of the traffic:

* partial pushes like `{"6": 74}` arrive whenever a value moves — a status
  query is never involved;
* DP 105 command-channel frames arrive roughly every half second while a job
  runs, each one a *different* frame (AI-object sightings, acks, map chatter);
* DP 104 path-trail JSON and DP 106 echoes arrive the same way;
* some frames are one-shot — the `0xAA 0x22` room-selection ack is sent once,
  in reply to an app action, and is simply gone if nobody is listening.

A `status()` poll returns one queued message. Polling every 15 s therefore
sampled at most one of the ~30 messages the robot sent in that window, and
could never see a one-shot frame at all. Worse, the per-message DP 105 value is
the *unit of meaning*: the merged snapshot only ever holds the newest frame, so
everything the poll didn't happen to catch was lost outright.

So the design here is:

* one dedicated **I/O thread owns the `tinytuya.Device`**. tinytuya is blocking
  and not thread-safe; no other thread — including the event loop — may touch
  the device object. Everything else goes through a command queue.
* that thread runs a `receive()` loop, does a full authoritative `status()`
  every `POLL_INTERVAL_SECONDS`, heartbeats an idle socket, and reconnects with
  backoff when the socket dies.
* every datapoint dict from either source is handed to the event loop with
  `loop.call_soon_threadsafe` and funnelled into the single `_ingest()` choke
  point, which merges, fans out to per-DP consumers, and publishes.

The coordinator also owns the room-awareness state for the entry: the persisted
zone set (`ZoneStore`), the position source (`PositionSource` — see
`position.py`) and the `RoomTracker` that turns fixes into a room. They live
here rather than on an entity because the capture services, the `current_room`
sensor and the vacuum segment API all need the same instance, and because room
tracking must keep running whether or not any of those exist.
"""

from __future__ import annotations

import asyncio
import importlib
import logging
import queue
import select
import threading
import time
from dataclasses import dataclass, field
from datetime import timedelta
from typing import Any

import tinytuya
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed

from .const import (
    CONF_AI_OBJECT_POSITION,
    CONF_DEVICE_ID,
    CONF_HOST,
    CONF_LOCAL_KEY,
    CONF_MODEL_FAMILY,
    CONF_PROTOCOL_VERSION,
    DEFAULT_AI_OBJECT_POSITION,
    DEFAULT_MODEL_FAMILY,
    DEFAULT_PROTOCOL_VERSION,
    DOMAIN,
    FamilySpec,
    resolve_family,
)
from .position import PathTrailTracker, PositionSource, create_position_source
from .rooms import RoomTracker
from .transport import AiObjectTracker, RoomSelectionTracker
from .zones import ZoneStore

_LOGGER = logging.getLogger(__name__)

# tinytuya refuses to decode a frame whose declared payload exceeds
# MAX_PAYLOAD_LENGTH, and the robot's map/path frames comfortably exceed the
# stock limit. The research scripts raise it the same way and for the same
# reason. It has to be set on BOTH modules: `tinytuya.core` is rebound in
# `tinytuya.__init__`, so `tinytuya.core.const` is not reachable by attribute
# walk and the message decoder holds its own reference.
MAX_PAYLOAD_LENGTH = 262144
for _module_name in ("tinytuya.core.const", "tinytuya.core.message_helper"):
    try:
        setattr(
            importlib.import_module(_module_name),
            "MAX_PAYLOAD_LENGTH",
            MAX_PAYLOAD_LENGTH,
        )
    except Exception as _err:  # noqa: BLE001 - a stubbed/renamed tinytuya is survivable
        _LOGGER.debug("bObsweep: could not raise %s payload limit: %s", _module_name, _err)

# How often the I/O thread forces a full authoritative status query. This is a
# *floor*: any status answered in the meantime (e.g. one HA asked for) resets
# the timer. It exists because `async_set_updated_data()` reschedules HA's own
# refresh timer, so on a chatty robot the HA-side interval below would in
# practice never elapse.
POLL_INTERVAL_SECONDS = 30
# HA's scheduled refresh. Same value; whichever fires first wins.
UPDATE_INTERVAL_SECONDS = 30
# Socket read timeout handed to tinytuya. Only reached when the robot has gone
# quiet, because we `select()` before reading (see `_pump`).
SOCKET_TIMEOUT_SECONDS = 5
# How long the thread waits for readability before checking its queue/timers.
# This, not the socket timeout, sets worst-case command latency.
SELECT_TIMEOUT_SECONDS = 0.5
# Tuya devices drop a socket that has been silent for ~30 s.
HEARTBEAT_IDLE_SECONDS = 10
RECONNECT_BACKOFF_START = 1.0
RECONNECT_BACKOFF_MAX = 30.0
# No successful read for this long => tell HA the coordinator is failing, even
# if nothing is currently asking it for data.
STALE_FAILURE_SECONDS = 60
# Don't emit the same connection warning every second while a robot is off.
ERROR_LOG_INTERVAL_SECONDS = 60
# Ceiling on an awaited device command. tinytuya retries a timed-out socket
# `socketRetryLimit` times, so an offline robot can keep `status()` busy for
# ~25 s; this is a backstop against hanging a config-entry setup forever.
COMMAND_TIMEOUT_SECONDS = 30
THREAD_JOIN_TIMEOUT_SECONDS = 5.0


class _TransportError(Exception):
    """A device call failed. `hard` means the socket should be torn down."""

    def __init__(self, message: str, *, hard: bool = False) -> None:
        super().__init__(message)
        self.hard = hard


@dataclass(slots=True)
class _Command:
    """One unit of work for the I/O thread, with the future to resolve."""

    kind: str  # "status" | "set" | "set_multi"
    args: tuple[Any, ...] = ()
    loop: asyncio.AbstractEventLoop | None = None
    future: asyncio.Future[Any] | None = field(default=None, repr=False)


class BobsweepCoordinator(DataUpdateCoordinator[dict[str, Any]]):
    """Listens to a bObsweep robot over the local Tuya protocol."""

    def __init__(self, hass: HomeAssistant, entry: ConfigEntry) -> None:
        """Set up the coordinator and the underlying tinytuya device."""
        super().__init__(
            hass,
            _LOGGER,
            name=DOMAIN,
            update_interval=timedelta(seconds=UPDATE_INTERVAL_SECONDS),
        )

        self.entry = entry

        data = entry.data
        self.device_id: str = data[CONF_DEVICE_ID]
        self.host: str = data[CONF_HOST]
        self.model_family: str = data.get(CONF_MODEL_FAMILY, DEFAULT_MODEL_FAMILY)
        # The DP table every platform reads its ids and vocabularies from.
        # Unknown/missing values resolve to SLAM so old entries keep working.
        self.spec: FamilySpec = resolve_family(self.model_family)

        local_key: str = data[CONF_LOCAL_KEY]
        protocol_version: str = data.get(
            CONF_PROTOCOL_VERSION, DEFAULT_PROTOCOL_VERSION
        )

        # Owned exclusively by the I/O thread once `async_setup()` starts it.
        # Constructing it here is safe (no socket is opened until first use).
        self.device = tinytuya.Device(
            self.device_id,
            self.host,
            local_key,
            version=float(protocol_version),
        )
        self.device.set_socketPersistent(True)
        self.device.set_socketTimeout(SOCKET_TIMEOUT_SECONDS)

        # --- room awareness -------------------------------------------------
        # Zones are persisted per config entry in `.storage/bobsweep_zones.*`,
        # NOT in entry options: options changes reload the entry, which would
        # tear down an in-flight zone capture. `async_setup()` loads them.
        self.zone_store = ZoneStore(hass, entry.entry_id)
        # Accumulates the robot's AI obstacle reports off DP 105. Owned here
        # rather than by either consumer because both the obstacle sensor and
        # the optional position source need the *same* instance, and because it
        # has to be fed per message: the merged DP snapshot keeps only the
        # newest DP 105 value, which is usually some other kind of frame.
        self.ai_objects: AiObjectTracker | None = (
            AiObjectTracker() if self.spec.dp_transportation is not None else None
        )
        # DP 104 path trail. Passive: the robot emits new points on its own
        # schedule (today only while the vendor app's map screen is open) and
        # nothing here ever writes the datapoint.
        self.trail: PathTrailTracker | None = (
            PathTrailTracker() if self.spec.dp_path_data is not None else None
        )
        # DP 105 room-selection ack (0xAA 0x22). Emitted once per room-targeted
        # job, never reported by a poll -- only observable by listening.
        self.room_selection: RoomSelectionTracker | None = (
            RoomSelectionTracker() if self.spec.dp_transportation is not None else None
        )
        # Opt-in, default off -- a coarse position source feeding room
        # classification can name rooms confidently and wrongly. See position.py.
        self.ai_object_position: bool = bool(
            entry.options.get(CONF_AI_OBJECT_POSITION, DEFAULT_AI_OBJECT_POSITION)
        )
        # The one swap point for "where is the robot": the passive trail first,
        # then (opt-in) the approximate AI-object source. See position.py.
        self.position_source: PositionSource = create_position_source(
            self.spec,
            ai_object_tracker=self.ai_objects,
            ai_object_position=self.ai_object_position,
            path_trail_tracker=self.trail,
        )
        self.rooms = RoomTracker(
            hass,
            zone_store=self.zone_store,
            position_source=self.position_source,
            status_dp=self.spec.dp_status,
            entry_id=entry.entry_id,
            device_id=self.device_id,
        )

        # The robot pushes PARTIAL updates (often just {"6": <battery>}).
        # Accumulate every datapoint ever seen so a partial push doesn't blank
        # out the rest of the state. Event-loop-only state.
        self._dps: dict[str, Any] = {}

        # --- I/O thread plumbing --------------------------------------------
        self._commands: queue.SimpleQueue[_Command] = queue.SimpleQueue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._stopping = False
        # Thread-only clocks (monotonic). Nothing else reads them.
        self._last_io = 0.0
        self._last_poll = 0.0
        self._last_success = 0.0
        self._last_error_log = 0.0
        self._failure_reported = False
        # Room updates are coalesced: at ~2 pushes/second we must not stack up
        # one `rooms.async_update()` task per message.
        self._pending_room_snapshot: dict[str, Any] | None = None
        self._room_task: asyncio.Task[None] | None = None

    # ------------------------------------------------------------------ setup

    async def async_setup(self) -> None:
        """Load persisted state and start the listener thread."""
        await self.zone_store.async_load()
        self._start_thread()

    def _start_thread(self) -> None:
        """Start the single thread that owns the device. Idempotent."""
        if self._thread is not None:
            return
        self._stop.clear()
        self._stopping = False
        self._last_success = time.monotonic()
        self._thread = threading.Thread(
            target=self._io_thread,
            name=f"bobsweep-{self.device_id[-6:]}",
            daemon=True,
        )
        self._thread.start()

    # --------------------------------------------------------------- shutdown

    async def async_shutdown(self) -> None:
        """Stop the listener thread, then let the base class tear down."""
        await self.async_stop()
        await super().async_shutdown()

    async def async_stop(self) -> None:
        """Stop the I/O thread and close the socket. Safe to call twice."""
        if self._stopping:
            return
        self._stopping = True
        self._stop.set()

        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            await self.hass.async_add_executor_job(
                thread.join, THREAD_JOIN_TIMEOUT_SECONDS
            )
            if thread.is_alive():
                # The thread is parked inside a tinytuya call that can outlast
                # our join budget (an offline robot retries for ~25 s). Closing
                # the socket is the only way to interrupt it. This is the one
                # place another thread touches the device, and it is safe
                # precisely because it only *destroys* state the I/O thread is
                # about to abandon anyway -- tinytuya's own close() swallows
                # everything and the thread treats a dead socket as a
                # reconnect, sees the stop flag, and exits.
                _LOGGER.debug("bObsweep: forcing socket close to unblock I/O thread")
                await self.hass.async_add_executor_job(self.device.close)
                await self.hass.async_add_executor_job(
                    thread.join, THREAD_JOIN_TIMEOUT_SECONDS
                )
                if thread.is_alive():
                    _LOGGER.warning("bObsweep: I/O thread did not exit within timeout")

        self._fail_pending_commands("bObsweep integration is shutting down")

        task = self._room_task
        self._room_task = None
        self._pending_room_snapshot = None
        if task is not None and not task.done():
            task.cancel()

    def _fail_pending_commands(self, reason: str) -> None:
        """Resolve every queued command with a failure. Any thread."""
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            self._resolve(command, error=UpdateFailed(reason))

    # --------------------------------------------------------- the I/O thread

    def _io_thread(self) -> None:
        """Own the device: receive pushes, run commands, reconnect. Thread."""
        backoff = RECONNECT_BACKOFF_START
        try:
            while not self._stop.is_set():
                try:
                    self._pump()
                except _TransportError as err:
                    self._handle_transport_error(err)
                    if err.hard:
                        self._close_socket()
                    if self._stop.wait(backoff):
                        break
                    backoff = min(backoff * 2, RECONNECT_BACKOFF_MAX)
                except Exception as err:  # noqa: BLE001 - the thread must not die
                    self._handle_transport_error(
                        _TransportError(f"unexpected I/O error: {err}", hard=True)
                    )
                    _LOGGER.debug("bObsweep: I/O thread exception", exc_info=True)
                    self._close_socket()
                    if self._stop.wait(backoff):
                        break
                    backoff = min(backoff * 2, RECONNECT_BACKOFF_MAX)
                else:
                    backoff = RECONNECT_BACKOFF_START
        finally:
            self._close_socket()

    def _pump(self) -> None:
        """One iteration: commands first, then timers, then one receive."""
        # 1. Commands. Draining these before blocking on the socket is what
        #    keeps a button press from waiting behind a quiet robot.
        while not self._stop.is_set():
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                break
            self._execute(command)

        if self._stop.is_set():
            return

        now = time.monotonic()

        # 2. Authoritative poll. Also serves as the (re)connect: status() opens
        #    the socket if it is closed, and gives us a complete snapshot.
        if self.device.socket is None or now - self._last_poll >= POLL_INTERVAL_SECONDS:
            self._poll_status()
            return

        # 3. Heartbeat an idle socket so the robot doesn't drop us.
        if now - self._last_io >= HEARTBEAT_IDLE_SECONDS:
            self._call(lambda: self.device.heartbeat(nowait=True))
            self._last_io = time.monotonic()

        # 4. Wait for a push. select() rather than a blocking receive() so the
        #    command queue is serviced within SELECT_TIMEOUT_SECONDS even when
        #    the robot is silent. tinytuya reads exactly one framed message and
        #    buffers nothing, so socket readability is an honest signal.
        sock = self.device.socket
        if sock is None:
            return
        try:
            readable, _, _ = select.select([sock], [], [], SELECT_TIMEOUT_SECONDS)
        except (OSError, ValueError) as err:
            raise _TransportError(f"socket select failed: {err}", hard=True) from err
        if not readable:
            self._check_stale()
            return

        message = self._call(self.device.receive)
        self._last_io = time.monotonic()
        if message is None:
            # Readable but nothing decodable: an empty ack, or the peer closed.
            return
        dps = self._extract_dps(message)
        if dps:
            self._note_success()
            self._publish(dps, source="push")

    # -- device calls (thread only) ------------------------------------------

    def _call(self, func: Any) -> Any:
        """Run one tinytuya call, normalising its two failure dialects."""
        try:
            result = func()
        except Exception as err:  # noqa: BLE001 - tinytuya raises bare Exception
            raise _TransportError(str(err) or type(err).__name__, hard=True) from err
        if isinstance(result, dict) and "Error" in result:
            # tinytuya reports connection loss as an error *dict*, not an
            # exception. It has already discarded the socket itself, so this is
            # not a "hard" error for us -- but it still counts as a failure.
            raise _TransportError(
                f"{result.get('Error')} ({result.get('Err')})", hard=False
            )
        return result

    def _poll_status(self) -> dict[str, Any]:
        """Full authoritative query. Returns the datapoints it ingested."""
        result = self._call(self.device.status)
        self._last_io = self._last_poll = time.monotonic()
        if not isinstance(result, dict):
            raise _TransportError(f"unexpected response: {result!r}")
        dps = self._extract_dps(result)
        if not dps:
            raise _TransportError(f"no 'dps' in response: {result!r}")
        self._note_success()
        self._publish(dps, source="poll")
        return dps

    @staticmethod
    def _extract_dps(message: Any) -> dict[str, Any]:
        """Pull a non-empty datapoint dict out of a tinytuya message."""
        if not isinstance(message, dict):
            return {}
        dps = message.get("dps")
        if isinstance(dps, dict) and dps:
            return dps
        return {}

    def _execute(self, command: _Command) -> None:
        """Run one queued command and resolve its future. Thread."""
        try:
            if command.kind == "status":
                result: Any = self._poll_status()
            elif command.kind == "set":
                dp, value = command.args
                result = self._write(lambda: self.device.set_value(dp, value, nowait=False))
            elif command.kind == "set_multi":
                (dps,) = command.args
                result = self._write_multiple(dps)
            else:  # pragma: no cover - programming error
                raise _TransportError(f"unknown command {command.kind!r}")
        except _TransportError as err:
            self._resolve(command, error=UpdateFailed(str(err)))
            # Surface it to the reconnect logic too: a command that failed
            # because the socket died must still trigger backoff.
            raise
        self._resolve(command, result=result)

    def _write(self, func: Any) -> Any:
        """Perform a write. `nowait=False` deliberately: see async_set_dp."""
        result = self._call(func)
        self._last_io = time.monotonic()
        # Writes are answered with the resulting datapoint state. Free data --
        # take it, and take it through the same choke point as everything else.
        dps = self._extract_dps(result)
        if dps:
            self._note_success()
            self._publish(dps, source="write")
        return result

    def _write_multiple(self, dps: dict[str, Any]) -> Any:
        """Set several datapoints, one frame if tinytuya supports it."""
        if hasattr(self.device, "set_multiple_values"):
            return self._write(lambda: self.device.set_multiple_values(dps, nowait=False))
        result: Any = None
        for dp, value in dps.items():
            result = self._write(
                lambda dp=dp, value=value: self.device.set_value(dp, value, nowait=False)
            )
        return result

    def _close_socket(self) -> None:
        """Drop the socket so the next call reconnects. Thread (see async_stop)."""
        try:
            self.device.close()
        except Exception:  # noqa: BLE001 - closing must never raise
            _LOGGER.debug("bObsweep: error closing socket", exc_info=True)

    # -- failure bookkeeping (thread only) -----------------------------------

    def _note_success(self) -> None:
        self._last_success = time.monotonic()
        self._failure_reported = False

    def _handle_transport_error(self, err: _TransportError) -> None:
        """Log (rate-limited) and, past the grace period, fail the coordinator."""
        now = time.monotonic()
        _LOGGER.debug("bObsweep: transport error: %s", err)
        if now - self._last_error_log >= ERROR_LOG_INTERVAL_SECONDS:
            self._last_error_log = now
            _LOGGER.warning(
                "bObsweep: lost contact with %s (%s); retrying", self.host, err
            )
        self._check_stale(str(err))

    def _check_stale(self, reason: str = "no data received") -> None:
        """Mark the coordinator failed once we've been dark for too long."""
        if self._failure_reported or self._stop.is_set():
            return
        if time.monotonic() - self._last_success < STALE_FAILURE_SECONDS:
            return
        self._failure_reported = True
        self._to_loop(
            self.async_set_update_error,
            UpdateFailed(
                f"No data from bObsweep at {self.host} for "
                f"{STALE_FAILURE_SECONDS}s: {reason}"
            ),
        )

    # -- crossing back to the event loop --------------------------------------

    def _to_loop(self, func: Any, *args: Any) -> None:
        """Schedule `func(*args)` on the event loop. Thread."""
        try:
            self.hass.loop.call_soon_threadsafe(func, *args)
        except RuntimeError:  # loop already closed during shutdown
            _LOGGER.debug("bObsweep: event loop gone, dropping %s", getattr(func, "__name__", func))

    def _publish(self, dps: dict[str, Any], *, source: str) -> None:
        """Hand one message's datapoints to the event loop. Thread."""
        self._to_loop(self._ingest_callback, dps, source)

    def _resolve(self, command: _Command, *, result: Any = None, error: Exception | None = None) -> None:
        """Complete a command's future on the loop that created it. Any thread."""
        future = command.future
        loop = command.loop
        if future is None or loop is None:
            return

        def _set() -> None:
            if future.done():  # the awaiter timed out or was cancelled
                return
            if error is not None:
                future.set_exception(error)
            else:
                future.set_result(result)

        try:
            loop.call_soon_threadsafe(_set)
        except RuntimeError:
            pass

    @callback
    def _ingest_callback(self, dps: dict[str, Any], source: str) -> None:
        """`call_soon_threadsafe` trampoline (it takes no keyword arguments)."""
        self._ingest(dps, source=source)

    # ------------------------------------------------------- the choke point

    @callback
    def _ingest(self, dps: dict[str, Any], *, source: str) -> None:
        """Absorb one message's datapoints. Event loop only.

        Every datapoint that reaches this integration — pushed, polled, or
        echoed back from a write — arrives here and nowhere else. Keep it that
        way: it is the only reason a new per-DP consumer is a three-line change
        instead of an audit of the whole module.
        """
        if not isinstance(dps, dict) or not dps:
            return

        # Partial-merge rule: a push of {"6": 90} must not blank anything else.
        self._dps.update(dps)

        # --- per-DP consumers -------------------------------------------
        # EXTENSION POINT. Each consumer is fed the value from THIS message
        # only, never the merged snapshot: the snapshot keeps just the newest
        # value per DP, so re-offering it would replay one frame forever and
        # hide every frame that arrived between two reads. Add new consumers
        # here, in this block, guarded by `is not None`.
        if self.ai_objects is not None and self.spec.dp_transportation is not None:
            value = dps.get(self.spec.dp_transportation)
            if value is not None:
                self.ai_objects.ingest(value)
        if self.trail is not None and self.spec.dp_path_data is not None:
            value = dps.get(self.spec.dp_path_data)
            if value is not None:
                self.trail.ingest(value)
        # The same DP 105 value goes to both trackers; each ignores frames it
        # does not own, so the order is irrelevant.
        if self.room_selection is not None and self.spec.dp_transportation is not None:
            value = dps.get(self.spec.dp_transportation)
            if value is not None:
                self.room_selection.ingest(value)
        # --- end per-DP consumers ---------------------------------------

        snapshot = dict(self._dps)
        # Room tracking is strictly downstream and never raises, but it *is*
        # async (a position source may have to ask the robot something). Never
        # await it from here: that would put a device round-trip in the middle
        # of the ingest path. Schedule it, and coalesce — at ~2 pushes/second
        # we want the newest snapshot processed once, not a queue of tasks.
        self._schedule_room_update(snapshot)
        self.async_set_updated_data(snapshot)

    @callback
    def _schedule_room_update(self, snapshot: dict[str, Any]) -> None:
        """Queue a room-tracking pass, collapsing bursts into the latest one."""
        self._pending_room_snapshot = snapshot
        if self._room_task is None or self._room_task.done():
            self._room_task = self.hass.async_create_task(self._async_drain_rooms())

    async def _async_drain_rooms(self) -> None:
        """Run room tracking until no newer snapshot is waiting."""
        while (snapshot := self._pending_room_snapshot) is not None:
            self._pending_room_snapshot = None
            await self.rooms.async_update(snapshot)

    # ------------------------------------------------------- the async surface

    async def _async_update_data(self) -> dict[str, Any]:
        """HA's scheduled refresh: ask the I/O thread for a full status.

        Kept as the coordinator's poll (rather than letting pushes be the only
        source) so `async_config_entry_first_refresh()` still fails setup with
        `ConfigEntryNotReady` when the robot is unreachable.
        """
        await self._async_command("status")
        # The thread published through `_ingest` before resolving the future,
        # so the merged snapshot is already current. Return it rather than the
        # raw reply: entities expect every DP ever seen, not this message's.
        if not self._dps:
            raise UpdateFailed("No datapoints received from bObsweep")
        return dict(self._dps)

    async def async_set_dp(self, dp: str, value: Any) -> None:
        """Set a single datapoint on the device and refresh state.

        Writes keep `nowait=False`. It costs one socket round trip, but it is
        the only way to learn that a command was rejected — and the reply
        carries the resulting datapoint state, which we ingest for free. A
        `nowait=True` write would return success unconditionally and make the
        vacuum entity's optimistic state a guess.
        """
        await self._async_command("set", dp, value)
        await self.async_request_refresh()

    async def async_set_dps(self, dps: dict[str, Any]) -> None:
        """Set multiple datapoints on the device and refresh state."""
        await self._async_command("set_multi", dps)
        await self.async_request_refresh()

    async def _async_command(self, kind: str, *args: Any) -> Any:
        """Queue work for the I/O thread and await its result."""
        if self._stopping or self._thread is None:
            raise UpdateFailed("bObsweep connection is not running")

        loop = asyncio.get_running_loop()
        future: asyncio.Future[Any] = loop.create_future()
        self._commands.put(_Command(kind=kind, args=args, loop=loop, future=future))
        try:
            return await asyncio.wait_for(future, COMMAND_TIMEOUT_SECONDS)
        except TimeoutError as err:
            raise UpdateFailed(
                f"Timed out waiting for bObsweep at {self.host} ({kind})"
            ) from err
