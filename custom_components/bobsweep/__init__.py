"""The bObsweep (local Tuya) integration."""

from __future__ import annotations

import logging

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import Platform
from homeassistant.core import HomeAssistant

from .coordinator import BobsweepCoordinator

_LOGGER = logging.getLogger(__name__)

PLATFORMS: list[Platform] = [
    Platform.VACUUM,
    Platform.SENSOR,
    Platform.BINARY_SENSOR,
    # SWITCH exists solely for the DP 128 camera control. The platform creates
    # nothing on families without that datapoint, so forwarding it
    # unconditionally costs a Vision/Random setup one no-op call.
    Platform.SWITCH,
    # Robot settings (water level, floor detection, mute, volume, ...). Each
    # entity is created only when the family has the datapoint AND the unit
    # actually reported it -- see select.py for the gating rule.
    Platform.SELECT,
    Platform.NUMBER,
]

type BobsweepConfigEntry = ConfigEntry[BobsweepCoordinator]


async def async_setup_entry(hass: HomeAssistant, entry: BobsweepConfigEntry) -> bool:
    """Set up bObsweep from a config entry."""
    coordinator = BobsweepCoordinator(hass, entry)
    # Zones must be loaded before the first refresh: that refresh runs the first
    # room-classification pass, and classifying against an empty zone set that
    # simply hasn't been read yet would report `unmapped` for no reason.
    # `async_setup()` also starts the listener thread that owns the socket.
    await coordinator.async_setup()
    # Registered before the first refresh can fail: the thread is already
    # running by now, and a ConfigEntryNotReady must not leak it.
    entry.async_on_unload(coordinator.async_stop)
    await coordinator.async_config_entry_first_refresh()

    entry.runtime_data = coordinator

    entry.async_on_unload(entry.add_update_listener(_async_update_listener))

    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)

    return True


async def async_unload_entry(hass: HomeAssistant, entry: BobsweepConfigEntry) -> bool:
    """Unload a bObsweep config entry."""
    unloaded = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    if unloaded:
        # Stop the listener thread and close the socket. Idempotent, and also
        # reached via `entry.async_on_unload`; doing it here as well means a
        # reload cannot start a second thread while the first is still on the
        # socket. Reloading must not leak threads.
        await entry.runtime_data.async_stop()
    return unloaded


async def _async_update_listener(hass: HomeAssistant, entry: BobsweepConfigEntry) -> None:
    """Reload the config entry when its options change."""
    await hass.config_entries.async_reload(entry.entry_id)
