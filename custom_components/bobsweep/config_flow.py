"""Config flow for the bObsweep (local Tuya) integration."""

from __future__ import annotations

import logging
from typing import Any

import voluptuous as vol

from homeassistant.config_entries import (
    ConfigEntry,
    ConfigFlow,
    ConfigFlowResult,
    OptionsFlow,
)
from homeassistant.core import callback
from homeassistant.helpers.selector import (
    SelectSelector,
    SelectSelectorConfig,
    SelectSelectorMode,
)

from .const import (
    CONF_AI_OBJECT_POSITION,
    CONF_DEVICE_ID,
    CONF_HOST,
    CONF_LOCAL_KEY,
    CONF_MODEL_FAMILY,
    CONF_PROTOCOL_VERSION,
    DEFAULT_AI_OBJECT_POSITION,
    DEFAULT_MODEL_FAMILY,
    DEFAULT_NAME,
    DEFAULT_PROTOCOL_VERSION,
    DOMAIN,
    MODEL_FAMILIES,
)

_LOGGER = logging.getLogger(__name__)

CONF_NAME = "name"

PROTOCOL_VERSIONS = ["3.1", "3.2", "3.3", "3.4", "3.5"]

# The DP-table families from const.py, in registry order. Labels are supplied by
# the `model_family` selector block in strings.json / translations, so the model
# lists stay editable without touching code. `const.py` (MODEL_FAMILIES) holds
# the authoritative model -> family table.
MODEL_FAMILY_OPTIONS = list(MODEL_FAMILIES)

STEP_USER_DATA_SCHEMA = vol.Schema(
    {
        vol.Required(CONF_HOST): str,
        vol.Required(CONF_DEVICE_ID): str,
        vol.Required(CONF_LOCAL_KEY): str,
        vol.Required(
            CONF_PROTOCOL_VERSION, default=DEFAULT_PROTOCOL_VERSION
        ): SelectSelector(
            SelectSelectorConfig(
                options=PROTOCOL_VERSIONS,
                mode=SelectSelectorMode.DROPDOWN,
            )
        ),
        vol.Required(
            CONF_MODEL_FAMILY, default=DEFAULT_MODEL_FAMILY
        ): SelectSelector(
            SelectSelectorConfig(
                options=MODEL_FAMILY_OPTIONS,
                mode=SelectSelectorMode.DROPDOWN,
                translation_key="model_family",
            )
        ),
        vol.Optional(CONF_NAME, default=DEFAULT_NAME): str,
    }
)


def _validate_device(
    device_id: str, host: str, local_key: str, protocol_version: str
) -> dict[str, Any]:
    """Attempt a real local connection to the device (blocking, run in executor).

    Returns the status payload on success. Raises CannotConnect or InvalidAuth
    on failure.
    """
    import tinytuya  # lazy import: blocking / heavy, only needed here

    try:
        device = tinytuya.Device(
            device_id,
            host,
            local_key,
            version=float(protocol_version),
        )
        status = device.status()
    except Exception as err:  # noqa: BLE001 - tinytuya raises broad exceptions
        _LOGGER.debug("bobsweep validation raised an exception: %s", err)
        raise CannotConnect from err

    if not isinstance(status, dict) or "Error" in status:
        error_text = str(status.get("Error", status)) if isinstance(status, dict) else str(status)
        _LOGGER.debug("bobsweep validation returned an error payload: %s", error_text)
        lowered = error_text.lower()
        if "key" in lowered or "905" in lowered or "901" in lowered:
            raise InvalidAuth(error_text)
        raise CannotConnect(error_text)

    return status


# Options are runtime preferences, not connection details, so they live in
# `entry.options` and are edited after setup rather than during it. Today there
# is exactly one, and it is off by default on purpose -- see
# `BobsweepOptionsFlow`.
OPTIONS_SCHEMA = vol.Schema(
    {
        vol.Optional(
            CONF_AI_OBJECT_POSITION, default=DEFAULT_AI_OBJECT_POSITION
        ): bool,
    }
)


class BobsweepConfigFlow(ConfigFlow, domain=DOMAIN):
    """Handle a config flow for bObsweep (local Tuya)."""

    VERSION = 1

    @staticmethod
    @callback
    def async_get_options_flow(config_entry: ConfigEntry) -> OptionsFlow:
        """Return the options flow for an existing entry."""
        return BobsweepOptionsFlow()

    async def async_step_user(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Handle the initial (and only) step."""
        errors: dict[str, str] = {}

        if user_input is not None:
            host = user_input[CONF_HOST]
            device_id = user_input[CONF_DEVICE_ID]
            local_key = user_input[CONF_LOCAL_KEY]
            protocol_version = user_input[CONF_PROTOCOL_VERSION]
            model_family = user_input[CONF_MODEL_FAMILY]
            name = user_input.get(CONF_NAME) or DEFAULT_NAME

            await self.async_set_unique_id(device_id)
            self._abort_if_unique_id_configured()

            try:
                await self.hass.async_add_executor_job(
                    _validate_device, device_id, host, local_key, protocol_version
                )
            except InvalidAuth:
                errors["base"] = "invalid_auth"
            except CannotConnect:
                errors["base"] = "cannot_connect"
            except Exception:  # noqa: BLE001
                _LOGGER.exception("Unexpected exception during bobsweep validation")
                errors["base"] = "unknown"
            else:
                return self.async_create_entry(
                    title=name,
                    data={
                        CONF_HOST: host,
                        CONF_DEVICE_ID: device_id,
                        CONF_LOCAL_KEY: local_key,
                        CONF_PROTOCOL_VERSION: protocol_version,
                        CONF_MODEL_FAMILY: model_family,
                        CONF_NAME: name,
                    },
                )

        return self.async_show_form(
            step_id="user",
            data_schema=self.add_suggested_values_to_schema(
                STEP_USER_DATA_SCHEMA, user_input
            )
            if user_input is not None
            else STEP_USER_DATA_SCHEMA,
            errors=errors,
        )


class BobsweepOptionsFlow(OptionsFlow):
    """Runtime preferences for an already-configured robot.

    **Why the AI-object position source is an option and defaults to off.**

    The robot has no working position datapoint (DP 104 is dead -- see
    `position.py`). The one thing that does yield coordinates is its obstacle
    detector: a new detection is roughly where the robot was standing. That is
    real information, and it is also sparse, event-driven, offset by however far
    the camera sees, and produced only when the floor happens to have clutter on
    it.

    Feeding that into room classification is a *judgement call about acceptable
    error*, and it is not ours to make silently. `rooms.py` is built end to end
    around refusing to name a room it is not sure of -- hysteresis before
    confirming, suspension when the map frame drifts, `unknown` rather than a
    guess. Switching this on by default would spend that carefulness on a source
    that cannot support it, and the failure mode is the specific one the whole
    module exists to prevent: a confident, wrong room name flowing into an
    automation.

    So it is opt-in, and this is the natural place for it:

    * it is a per-robot preference someone can change and change back, without
      re-entering the local key;
    * `entry.options` already triggers a reload through the update listener in
      `__init__.py`, so the position source is rebuilt cleanly on toggle;
    * it stays out of the initial setup form, where a new user has no context to
      judge it and every extra field is friction on an install that is already
      hard enough.

    A YAML flag would not be discoverable, and a switch entity would be worse --
    it would let an automation change how position is derived, which belongs to
    the user.
    """

    async def async_step_init(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Show and save the options form."""
        if user_input is not None:
            return self.async_create_entry(data=user_input)

        return self.async_show_form(
            step_id="init",
            data_schema=self.add_suggested_values_to_schema(
                OPTIONS_SCHEMA, dict(self.config_entry.options)
            ),
        )


class CannotConnect(Exception):
    """Error indicating the device could not be reached."""


class InvalidAuth(Exception):
    """Error to indicate the local_key/device_id pair is invalid."""
