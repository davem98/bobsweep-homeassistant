"""Assist intents for the vacuum controls Home Assistant has no intent for.

Home Assistant ships `HassVacuumStart`, `HassVacuumReturnToBase` and
`HassVacuumCleanArea` (checked against the 2026.9 source). It has no intent
that stops or pauses a vacuum, and none that reports what one is doing, so
"stop Rosie" through the built-in Assist matcher has nothing to resolve to.
This module registers three:

* `BobsweepVacuumStop`   -> `vacuum.stop`   (on this integration: halt where
  it is -- it does not dock; "send Rosie home" is the docking command)
* `BobsweepVacuumPause`  -> `vacuum.pause`
* `BobsweepVacuumStatus` -> a spoken answer: activity, current room when the
  robot's room is known, battery

The names are namespaced so a future built-in `HassVacuumStop` cannot collide
with them. The sentences that reach them live in
`custom_sentences/en/bobsweep.yaml`; without that file only an LLM-backed
conversation agent (which calls intents as tools) can use them.

Home Assistant picks this module up by itself: the `intent` component runs
`async_setup_intents(hass)` from the `intent.py` platform of every loaded
integration, custom ones included (`integration_platform`, which looks the
file up in the integration's top-level files).

Scope: the stop and pause handlers act on *any* exposed vacuum, not only
bObsweep entities. They target by entity name and aliases, honour Assist
exposure and the entity's advertised STOP / PAUSE feature, and only call the
standard vacuum services -- exactly what Home Assistant's own
`HassVacuumStart` does. Restricting them to this integration would make
"stop the vacuum" work for one brand and fail with a confusing "no device"
for another in the same house, for no safety gain.
"""

from __future__ import annotations

from typing import Any

import voluptuous as vol

from homeassistant.components.vacuum import (
    DOMAIN as VACUUM_DOMAIN,
    SERVICE_PAUSE,
    SERVICE_STOP,
    VacuumEntityFeature,
)
from homeassistant.core import HomeAssistant, State
from homeassistant.helpers import config_validation as cv
from homeassistant.helpers import entity_registry as er
from homeassistant.helpers import intent

from .const import DOMAIN

INTENT_VACUUM_STOP = "BobsweepVacuumStop"
INTENT_VACUUM_PAUSE = "BobsweepVacuumPause"
INTENT_VACUUM_STATUS = "BobsweepVacuumStatus"

# Spoken form of each vacuum activity (the entity's state string).
_ACTIVITY_SPEECH: dict[str, str] = {
    "cleaning": "cleaning",
    "docked": "docked",
    "idle": "idle",
    "paused": "paused",
    "returning": "heading back to its dock",
    "error": "reporting a problem",
}

# Room sensor values that are not a room name.
_NO_ROOM = {"unknown", "unavailable", "unmapped", "", "none"}


async def async_setup_intents(hass: HomeAssistant) -> None:
    """Register the stop, pause and status intents."""
    intent.async_register(
        hass,
        _VacuumServiceIntentHandler(
            INTENT_VACUUM_STOP,
            VACUUM_DOMAIN,
            SERVICE_STOP,
            description="Stops a vacuum where it is (does not send it to its dock)",
            required_domains={VACUUM_DOMAIN},
            platforms={VACUUM_DOMAIN},
            required_features=VacuumEntityFeature.STOP,
        ),
    )
    intent.async_register(
        hass,
        _VacuumServiceIntentHandler(
            INTENT_VACUUM_PAUSE,
            VACUUM_DOMAIN,
            SERVICE_PAUSE,
            description="Pauses a vacuum",
            required_domains={VACUUM_DOMAIN},
            platforms={VACUUM_DOMAIN},
            required_features=VacuumEntityFeature.PAUSE,
        ),
    )
    intent.async_register(hass, VacuumStatusIntentHandler())


def _nameless(slots: dict[str, Any]) -> bool:
    """True when the sentence named no vacuum, area or floor ("stop the vacuum")."""
    return not any(key in slots for key in ("name", "area", "floor"))


class _VacuumServiceIntentHandler(intent.ServiceIntentHandler):
    """`ServiceIntentHandler` that refuses to guess between several vacuums.

    The stock handler, given no name, acts on every exposed entity of the
    domain -- right for "turn off the lights", wrong for "stop the vacuum" in a
    house with two robots. A nameless request here must resolve to exactly one
    exposed vacuum (Home Assistant's `single_target` match, which also lets the
    satellite's own area break a tie); otherwise it fails with the standard
    "which one?" error and nothing moves. Named requests behave exactly like
    the stock handler.
    """

    async def async_handle(self, intent_obj: intent.Intent) -> intent.IntentResponse:
        """Handle the intent, single-target when no vacuum was named."""
        slots = self.async_validate_slots(intent_obj.slots)
        if not _nameless(slots):
            return await super().async_handle(intent_obj)

        constraints = intent.MatchTargetsConstraints(
            domains=self.required_domains,
            features=self.required_features,
            assistant=intent_obj.assistant,
            single_target=True,
        )
        preferences = intent.MatchTargetsPreferences(
            area_id=slots.get("preferred_area_id", {}).get("value"),
            floor_id=slots.get("preferred_floor_id", {}).get("value"),
        )
        result = intent.async_match_targets(intent_obj.hass, constraints, preferences)
        if not result.is_match:
            raise intent.MatchFailedError(
                result=result, constraints=constraints, preferences=preferences
            )
        intent_obj.slots = slots
        return await self.async_handle_states(intent_obj, result, constraints, preferences)


class VacuumStatusIntentHandler(intent.IntentHandler):
    """Answer "where is Rosie" / "what is Rosie doing".

    Speaks the vacuum's activity, the room it is in when this integration's
    current-room sensor knows it (any other vacuum simply has no room), and
    the battery from a battery sensor on the same device (or the legacy
    `battery_level` attribute).
    """

    intent_type = INTENT_VACUUM_STATUS
    platforms = {VACUUM_DOMAIN}
    description = "Reports what a vacuum is doing, where it is and its battery"

    @property
    def slot_schema(self) -> dict:
        """Return the slot schema."""
        return {
            vol.Optional("name"): cv.string,
            vol.Optional("preferred_area_id"): cv.string,
            vol.Optional("preferred_floor_id"): cv.string,
        }

    async def async_handle(self, intent_obj: intent.Intent) -> intent.IntentResponse:
        """Handle the intent."""
        hass = intent_obj.hass
        slots = self.async_validate_slots(intent_obj.slots)
        name: str | None = slots.get("name", {}).get("value")

        constraints = intent.MatchTargetsConstraints(
            name=name,
            domains={VACUUM_DOMAIN},
            assistant=intent_obj.assistant,
            single_target=True,
        )
        preferences = intent.MatchTargetsPreferences(
            area_id=slots.get("preferred_area_id", {}).get("value"),
            floor_id=slots.get("preferred_floor_id", {}).get("value"),
        )
        result = intent.async_match_targets(hass, constraints, preferences)
        if not result.is_match:
            raise intent.MatchFailedError(
                result=result, constraints=constraints, preferences=preferences
            )

        state = result.states[0]
        response = intent_obj.create_response()
        response.response_type = intent.IntentResponseType.QUERY_ANSWER
        response.async_set_results(
            success_results=[
                intent.IntentResponseTarget(
                    type=intent.IntentResponseTargetType.ENTITY,
                    name=state.name,
                    id=state.entity_id,
                )
            ]
        )
        response.async_set_states([state])
        response.async_set_speech(_status_speech(hass, state))
        return response


def _status_speech(hass: HomeAssistant, state: State) -> str:
    """Build "Rosie is cleaning, in Kitchen. Battery 80 percent."."""
    activity = _ACTIVITY_SPEECH.get(state.state)
    if activity is None:
        activity = "unavailable" if state.state == "unavailable" else state.state
    speech = f"{state.name} is {activity}"

    room, battery = _device_room_and_battery(hass, state)
    if room is not None:
        speech += f", in {room}"
    speech += "."
    if battery is not None:
        speech += f" Battery {battery} percent."
    return speech


def _device_room_and_battery(
    hass: HomeAssistant, state: State
) -> tuple[str | None, int | None]:
    """Find the current room and battery level from the vacuum's device."""
    room: str | None = None
    battery: int | None = _as_percent(state.attributes.get("battery_level"))

    registry = er.async_get(hass)
    entry = registry.async_get(state.entity_id)
    if entry is None or entry.device_id is None:
        return room, battery

    for sibling in er.async_entries_for_device(registry, entry.device_id):
        if sibling.domain != "sensor":
            continue
        sibling_state = hass.states.get(sibling.entity_id)
        if sibling_state is None:
            continue
        if (
            sibling.platform == DOMAIN
            and sibling.unique_id.endswith("_current_room")
            and str(sibling_state.state).lower() not in _NO_ROOM
        ):
            room = sibling_state.state
        elif battery is None and sibling_state.attributes.get("device_class") == "battery":
            battery = _as_percent(sibling_state.state)
    return room, battery


def _as_percent(value: Any) -> int | None:
    """Round a battery reading to a whole percent, or None if it isn't one."""
    try:
        return round(float(value))
    except (TypeError, ValueError):
        return None
