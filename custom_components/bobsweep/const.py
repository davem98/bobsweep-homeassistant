"""Constants and the per-family bObsweep datapoint contract.

bObsweep ships three mutually incompatible Tuya datapoint (DP) layouts, and the
vendor's own Android app switches between them by model. `BobModule.getDPIDTable()`
in `assets/index.android.bundle` maps every WiFi-capable model onto one of
`TUYA_SLAM`, `TUYA_VISION` or `TUYA_RANDOM`; this module mirrors those three
tables verbatim. Everything here — including the model -> family table and the
per-family error-bit lists — was transcribed key for key from the vendor
Android app's JavaScript bundle (`BobModule.getDPIDTable()` and the
`TUYA_SLAM` / `TUYA_VISION` / `TUYA_RANDOM` object literals).

The families differ in three ways that matter to the integration:

1. **DP ids move.** Vision keeps work-mode on DP `2` and the error bitmask on
   DP `8` but relocates everything else into the `207`-`218` block. SLAM and
   Random share the low-DP layout for the commands they have in common.
2. **DPs disappear.** Vision has no brush/filter life, no consumable resets, no
   locate, no self-empty and no mop/vacuum attachment DPs. Random additionally
   has no cleaning-area or cleaning-record DP, and collapses SLAM's separate mop
   (118) / vacuum (119) attachment DPs into one dustbin+water-tank DP (103).
   A DP a family does not have is `None` here, never a guessed id — the
   platforms skip creating an entity rather than publishing a permanently
   unknown one.
3. **The enum vocabularies differ for the same concept.** Fan speed is
   `gentle/normal/strong/closed` on SLAM and `gentle/normal/strong` on Random but
   `quiet/standard/strong` on Vision; "stopped" is `standby` on SLAM/Random and
   `idle` on Vision. The vocabularies are therefore kept strictly per family and
   never merged.

Everything is bundled into a frozen `FamilySpec`, registered in `FAMILIES` under
the config-entry's `model_family` value. `resolve_family()` falls back to the
SLAM spec for a missing or unrecognised value so config entries written before
the family selector existed keep working unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

DOMAIN = "bobsweep"
DEFAULT_NAME = "bObsweep"

# --- config-entry keys -------------------------------------------------------
CONF_DEVICE_ID = "device_id"
CONF_LOCAL_KEY = "local_key"
CONF_HOST = "host"
CONF_PROTOCOL_VERSION = "protocol_version"  # tinytuya: "3.3" / "3.4" / "3.5"
CONF_MODEL_FAMILY = "model_family"          # "slam" | "vision" | "random"

# --- config-entry OPTIONS (entry.options, not entry.data) --------------------
# Opt-in only. See `position.py`: the AI-object stream is a sparse, approximate
# sighting feed, not localisation, and feeding it to the room classifier can
# produce confidently-wrong room names. Off unless the user opts in.
CONF_AI_OBJECT_POSITION = "ai_object_position"
DEFAULT_AI_OBJECT_POSITION = False

DEFAULT_PROTOCOL_VERSION = "3.3"
DEFAULT_MODEL_FAMILY = "slam"

FAMILY_SLAM = "slam"
FAMILY_VISION = "vision"
FAMILY_RANDOM = "random"

MODEL_FAMILIES = (FAMILY_SLAM, FAMILY_VISION, FAMILY_RANDOM)


def _frozen(mapping: Mapping[str, str]) -> Mapping[str, str]:
    """Return a read-only view of a vocabulary so a spec can't be mutated."""
    return MappingProxyType(dict(mapping))


@dataclass(frozen=True, kw_only=True)
class FamilySpec:
    """One bObsweep DP table: ids, enum vocabularies and HA state groupings.

    Every `dp_*` field is either the Tuya datapoint id as the app spells it (a
    string, because tinytuya keys the `dps` dict by string) or `None` when the
    family genuinely has no such datapoint. Consumers must treat `None` as
    "capability absent", not "unknown yet".
    """

    key: str

    # --- datapoint ids -------------------------------------------------------
    dp_power: str | None            # COMMAND_ENABLE (bool: start/stop)
    dp_mode: str                    # COMMAND_WORK_MODE (enum, see work_mode)
    dp_direction: str | None        # COMMAND_CLEAN_DIRECTION (manual jog)
    dp_status: str                  # COMMAND_STATUS (enum, read-only)
    dp_battery: str | None          # COMMAND_BATTERY_PERCENTAGE (int %)
    dp_side_brush_life: str | None
    dp_roll_brush_life: str | None
    dp_filter_life: str | None
    dp_reset_side_brush: str | None
    dp_reset_roll_brush: str | None
    dp_reset_filter: str | None
    dp_locate: str | None           # COMMAND_SEEK_ROBOT (bool: play a sound)
    dp_fan: str | None              # COMMAND_CLEANING_POWER (enum)
    dp_clean_record: str | None     # cleaning log/record blob
    dp_clean_area: str | None       # m^2 of the last clean
    dp_clean_time: str | None       # minutes
    dp_error: str | None            # COMMAND_ERROR_REPORTED1 (bitmask/fault)
    dp_error2: str | None = None    # COMMAND_ERROR_REPORTED2 (SLAM only)
    dp_error3: str | None = None    # COMMAND_FAULT_BITS (SLAM only, DP 131,
                                    # byte array / hex string, not an integer)
    dp_dustbin_empty_interval: str | None = None
    dp_dustbin_empty_switch: str | None = None   # auto self-empty enable (bool)
    dp_reset_main_brush_dirty: str | None = None
    dp_versions: str | None = None
    dp_status_mop: str | None = None             # SLAM mop attachment
    dp_status_vacuum: str | None = None          # SLAM vacuum attachment
    dp_dustbin_empty_duration: str | None = None
    dp_water_control: str | None = None          # DP 20 (SLAM has it too, 4 levels; unmodelled)
    dp_status_dustbin_watertank: str | None = None  # Random (DP 103)
    dp_mute: str | None = None                   # DP 215 Vision (SLAM: 107, unmodelled)
    dp_position: str | None = None               # Vision only (DP 216)
    # DP 104 COMMAND_PATH_DATA: the *candidate* source of live robot position.
    # UNVERIFIED -- a live probe got the request echoed straight back with zero
    # points while the robot was docked. The id is recorded here because that is
    # where DP ids belong; nothing may assume it works. Every interpretation of
    # it is confined to `position.py`, which is the one place to change once a
    # probe settles what actually carries position.
    dp_path_data: str | None = None              # SLAM only (DP 104)
    # DP 105 COMMAND_TRANSPORTATION: the map/geometry channel. *Reading* it is
    # verified -- a captured `eRestrictedToApp` frame decoded cleanly into five
    # no-go rectangles as signed int16 map-cell corner pairs. *Writing* it is
    # not; the vacuum segment-clean path names this DP in its error message and
    # deliberately sends nothing.
    dp_transportation: str | None = None         # SLAM only (DP 105)
    # DP 128 COMMAND_CAMERA: a plain boolean that turns the robot's on-board
    # object-detection camera on and off. VERIFIED against the live robot --
    # read path (`isObjectDetection`), write path (`publishDps({128: bool})`)
    # and an app UI toggle all agree, and the reference unit reports True. This is
    # the only user-facing control over the camera on the whole device.
    dp_camera: str | None = None                 # SLAM only (DP 128)
    dp_language: str | None = None               # Vision only (DP 217)
    dp_cliff_sensor: str | None = None           # DP 218 Vision (SLAM: 111, unmodelled)

    # --- settings DPs, added 2026-09-05 (live raw-status read, SLAM unit) ------
    # Same absence rule as everything else here: `None` means the *family* has
    # no such datapoint per the vendor tables. A family having the DP does not
    # mean a given unit emits it -- see the platform-level gating in select.py.
    dp_water_level: str | None = None            # COMMAND_WATER_CONTROL (DP 20)
    dp_floor_type_detection: str | None = None   # COMMAND_FLOOR_TYPE_DETECTION (112)
    dp_self_empty_power: str | None = None       # COMMAND_SELF_EMPTY_POWER (124)
    dp_mop_maintenance_strategy: str | None = None  # COMMAND_MOP_MAINTAINENCE_STRATEGY (134)
    dp_extending_arms: str | None = None         # COMMAND_EXTENDING_ARMS (130)
    dp_mop_dry_duration: str | None = None       # COMMAND_MOP_DRY_DURATION (132)
    dp_mop_wash_temperature: str | None = None   # COMMAND_MOP_WASH_TEMPERATURE (133)
    dp_dock_task_self_empty: str | None = None   # COMMAND_DOCK_TASK_SELF_EMPTY (135)
    dp_mute_switch: str | None = None            # COMMAND_MUTE as a plain bool switch
                                                  # (SLAM: 107, Vision: 215). Distinct
                                                  # from `dp_mute`/`_muted` above, which
                                                  # is the existing Vision-only sensor
                                                  # field and is left untouched.
    dp_cliff_sensor_switch: str | None = None    # COMMAND_CLIFF_SENSOR as a switch
                                                  # (SLAM only, DP 111, string 'on'/'off').
                                                  # Distinct from `dp_cliff_sensor` above
                                                  # (Vision's DP 218, unmodelled sensor).
    # No new field for "auto empty": it is the existing `dp_dustbin_empty_switch`
    # (SLAM DP 115) above, exposed as a switch. Reusing it rather than adding a
    # duplicate field.
    dp_quick_clean_use_global_vacuum: str | None = None  # DP 122
    dp_volume: str | None = None                 # COMMAND_VOLUME (DP 108, int 0-100)

    # --- enum vocabularies (NAME -> Tuya enum payload) -----------------------
    work_mode: Mapping[str, str]
    clean_direction: Mapping[str, str]
    status: Mapping[str, str]
    fan_speed: Mapping[str, str]

    # --- settings enum vocabularies (added 2026-09-05) -----------------------
    water_level: Mapping[str, str] | None = None
    floor_type_detection: Mapping[str, str] | None = None
    self_empty_power: Mapping[str, str] | None = None
    mop_maintenance_strategy: Mapping[str, str] | None = None
    extending_arms: Mapping[str, str] | None = None
    mop_dry_duration: Mapping[str, str] | None = None
    mop_wash_temperature: Mapping[str, str] | None = None
    dock_task_self_empty: Mapping[str, str] | None = None

    # --- HA-facing derivations ----------------------------------------------
    # HA `fan_speed_list`: the fan enum values a user may pick, excluding any
    # "off" value (SLAM's CLOSED), which HA models as stopping instead.
    fan_speeds: tuple[str, ...]

    # work_mode NAME keys used by the standard vacuum commands. None means the
    # family has no equivalent and the feature must not be advertised.
    mode_auto: str                  # what `vacuum.start` writes
    mode_charge: str                # what `vacuum.return_to_base` writes

    # Raw dp_status values grouped into HA VacuumActivity buckets. Anything not
    # listed falls through to VacuumActivity.IDLE.
    status_cleaning: frozenset[str]
    status_returning: frozenset[str]
    status_docked: frozenset[str]

    # --- optional / family-specific extras -----------------------------------
    water_control: Mapping[str, str] | None = None
    dustbin_watertank: Mapping[str, str] | None = None
    mode_spot: str | None = None    # what `vacuum.clean_spot` writes
    mode_stop: str | None = None    # explicit standby mode, if the family has one
    # Dedicated command DPs (SLAM only), verbatim from the vendor app's own
    # command functions: `startDocking()` writes COMMAND_START_STOP_DOCKING=true
    # (false cancels a return), `pauseRobot()` writes COMMAND_PAUSE, and
    # `sendStop()` clears COMMAND_ENABLE while cleaning or clears
    # START_STOP_DOCKING while returning. Writing the work mode alone does NOT
    # redirect a running job (verified on hardware 2026-09-05).
    dp_docking: str | None = None   # COMMAND_START_STOP_DOCKING
    dp_pause: str | None = None     # COMMAND_PAUSE
    status_paused: frozenset[str] = field(default_factory=frozenset)
    status_error: frozenset[str] = field(default_factory=frozenset)

    # --- fault-name tables (bit index -> app trouble key) --------------------
    # Transcribed from the app's ERROR_TOPIC_LIST1 / ERROR_TOPIC_LIST2 /
    # cTroubleBitKeys arrays in the vendor app's JavaScript bundle. The index into
    # each tuple *is* the bit index — never reorder or dedupe these, the
    # duplicate entries (SLAM bits 14/20, 15/16; Random bits 5/6 and 7-10) are
    # in the vendor arrays exactly as written here.
    fault_bits: tuple[str, ...] = ()        # dp_error  (DP 18 / DP 8), integer
    fault_bits2: tuple[str, ...] = ()       # dp_error2 (DP 113), integer
    fault_bits_raw: tuple[str, ...] = ()    # dp_error3 (DP 131), packed bytes

    # The app carries a `hasLeftRightTroubleReversed(model)` helper that swaps
    # indices 2 and 3 of the DP 18 trouble list for some models — on SLAM that
    # is `TroubleWheelRight` <-> `TroubleWheelLeft`. UNVERIFIED: whether the
    # reference unit (an UltraVision Pet Combo, or any other model) is in that
    # set is not known, and the DP tables do not say. Defaulting to False = "not
    # reversed" means the tables are used exactly as transcribed. If a real
    # robot ever reports the wrong wheel, flip this to True for that family;
    # the only consequence is which of those two wheel faults is named.
    left_right_faults_reversed: bool = False


# --- fault tables ------------------------------------------------------------
# Two app-level constants shared by every family.

# The app's own SKIP_TROUBLES: informational notes, not faults. They appear as
# bits in the DP 113 / DP 131 tables (indices 4 and 5) and must never make the
# robot look broken.
SKIP_TROUBLES: frozenset[str] = frozenset({"NoteMopWaterLow", "NoteMoppingIsOff"})

# Table entries that name a healthy state rather than a fault: Random's bit 0.
NON_FAULT_KEYS: frozenset[str] = frozenset({"Normal"})

# TUYA_SLAM ERROR_TOPIC_LIST1 - DP 18 (30 entries). Bits 14/20 and 15/16 really
# do repeat the same key in the vendor array.
SLAM_FAULT_BITS: tuple[str, ...] = (
    "TroubleSideBrush",              # 0
    "TroubleMainBrush",              # 1
    "TroubleWheelRight",             # 2  (swapped with 3 when reversed)
    "TroubleWheelLeft",              # 3
    "TroubleDustbinMop",             # 4
    "TroubleEdgeSensors",            # 5
    "TroubleBumper",                 # 6
    "TroubleElectronics",            # 7
    "TroubleBatteryLow",             # 8
    "TroubleBatteryOutOfCharge",     # 9
    "TroubleUserInterface",          # 10
    "TroubleBobStuck",               # 11
    "TroubleChargingStation",        # 12
    "TroubleLocalization",           # 13
    "TroubleNavigation",             # 14
    "TroubleSettingsDiscrepancy",    # 15
    "TroubleSettingsDiscrepancy",    # 16
    "TroublePositioning",            # 17
    "TroubleLidarSensor",            # 18
    "TroubleNoBatteryFound",         # 19
    "TroubleNavigation",             # 20
    "TroubleLidarBumper",            # 21
    "TroubleOverheated",             # 22
    "TroubleBattery",                # 23
    "TroubleWallSensor",             # 24
    "TroubleWallSensorDirty",        # 25
    "TroubleMopNotConnected",        # 26
    "TroubleWaterDispenserNotWorking",  # 27
    "TroubleCameraSwIncompatible",   # 28
    "TroubleCameraSwNotToDate",      # 29
)

# TUYA_SLAM ERROR_TOPIC_LIST2 - DP 113 (30 entries). Identical, name for name
# and index for index, to the first 30 entries of cTroubleBitKeys below.
SLAM_FAULT_BITS2: tuple[str, ...] = (
    "TroubleCameraNotDetected",           # 0
    "TroubleCameraSensorNotResponsive",   # 1
    "TroubleCameraElectronics",           # 2
    "TroubleWaterDispenser",              # 3
    "NoteMopWaterLow",                    # 4  (note, not a fault)
    "NoteMoppingIsOff",                   # 5  (note, not a fault)
    "TroubleStationMotor",                # 6
    "TroubleStationSensor",               # 7
    "TroubleCongestion",                  # 8
    "TroubleDustBag",                     # 9
    "TroubleDustBagFull",                 # 10
    "TroubleStationLid",                  # 11
    "TroubleEdgeSensors",                 # 12
    "TroubleSideBrush",                   # 13
    "TroubleMainBrush",                   # 14
    "TroubleVacuumMotorJam",              # 15
    "TroubleVacuumFanMotor",              # 16
    "TroubleFloorSense",                  # 17
    "TroubleMopNotConnectedLeft",         # 18
    "TroubleMopNotConnectedRight",        # 19
    "TroubleMopJamLeft",                  # 20
    "TroubleMopJamRight",                 # 21
    "TroubleMopElectronicLeft",           # 22
    "TroubleMopElectronicRight",          # 23
    "TroubleMopSensorLeft",               # 24
    "TroubleMopSensorRight",              # 25
    "TroubleMopArmJam",                   # 26
    "TroubleMopArmElectronic",            # 27
    "TroubleMopArmSensor",                # 28
    "TroubleBrushArmElectronic",          # 29
)

# cTroubleBitKeys - DP 131 (40 entries). Bits 0-29 are SLAM_FAULT_BITS2
# verbatim; bits 30-39 are the ten station/water faults that exist on no other
# datapoint. Read LSB-first across bytes: bitIndex = byteIndex * 8 + bit.
SLAM_FAULT_BITS_RAW: tuple[str, ...] = SLAM_FAULT_BITS2 + (
    "TroubleCleanWaterLow",               # 30
    "TroubleDirtyWaterFull",              # 31
    "TroubleTrayCantDrain",               # 32
    "TroubleStationOverheat",             # 33
    "TroubleDirtyWaterContainerPump",     # 34
    "TroubleCleanWaterContainerPump",     # 35
    "TroubleMopDryerTrouble",             # 36
    "TroubleCleanSolutionPump",           # 37
    "TroubleCleanWaterContainerValve",    # 38
    "TroubleMopAttachmentNotDetected",    # 39
)

# TUYA_VISION ERROR_TOPIC_LIST1 - DP 8 (15 entries). Vision's left/right entries
# are at indices 0/1, not 2/3.
VISION_FAULT_BITS: tuple[str, ...] = (
    "TroubleWheelRight",       # 0
    "TroubleWheelLeft",        # 1
    "TroubleSideBrush",        # 2
    "TroubleVacuumMotor",      # 3
    "TroubleMainBrush",        # 4
    "TroubleBumperRight",      # 5
    "TroubleBumperLeft",       # 6
    "TroubleDustbin",          # 7
    "TroubleEdgeSensorRight",  # 8
    "TroubleEdgeSensorFront",  # 9
    "TroubleEdgeSensorLeft",   # 10
    "LowBattery",              # 11  (no Trouble prefix in the vendor array)
    "TroubleMop",              # 12
    "TroublePowerError",       # 13
    "TroubleBobStuck",         # 14
)

# TUYA_RANDOM ERROR_TOPIC_LIST1 - DP 18 (15 entries). Bit 0 is 'Normal', a
# healthy state rather than a fault; the repeated bumper/edge-sensor entries are
# in the vendor array as written.
RANDOM_FAULT_BITS: tuple[str, ...] = (
    "Normal",              # 0  (not a fault - see NON_FAULT_KEYS)
    "TroubleWheel",        # 1
    "TroubleSideBrush",    # 2
    "TroubleMainBrush",    # 3
    "TroubleCharging",     # 4
    "TroubleBumper",       # 5
    "TroubleBumper",       # 6
    "TroubleEdgeSensors",  # 7
    "TroubleEdgeSensors",  # 8
    "TroubleEdgeSensors",  # 9
    "TroubleEdgeSensors",  # 10
    "TroubleVacuumMotor",  # 11
    "TroubleBattery",      # 12
    "TroubleWheelSensor",  # 13
    "TroubleNavigation",   # 14
)


# --- TUYA_SLAM ---------------------------------------------------------------
# The flagship LiDAR/SLAM units (UltraVision, Dustin, Austin, Phoenix, ...).
# This is the live, field-verified table: do not change any id or enum string.
SLAM = FamilySpec(
    key=FAMILY_SLAM,
    dp_power="2",
    dp_mode="3",
    dp_direction="4",
    dp_status="5",
    dp_battery="6",
    dp_side_brush_life="7",
    dp_roll_brush_life="8",
    dp_filter_life="9",
    dp_reset_side_brush="10",
    dp_reset_roll_brush="11",
    dp_reset_filter="12",
    dp_locate="13",
    dp_fan="14",
    dp_clean_record="15",
    dp_clean_area="16",
    dp_clean_time="17",
    dp_error="18",
    dp_error2="113",
    dp_error3="131",
    fault_bits=SLAM_FAULT_BITS,
    fault_bits2=SLAM_FAULT_BITS2,
    fault_bits_raw=SLAM_FAULT_BITS_RAW,
    dp_dustbin_empty_interval="114",
    dp_dustbin_empty_switch="115",
    dp_reset_main_brush_dirty="116",
    dp_versions="117",
    dp_status_mop="118",
    dp_status_vacuum="119",
    dp_dustbin_empty_duration="121",
    # Position/geometry channel ids. Neither is used to command the robot; see
    # the field comments above and `position.py`.
    dp_path_data="104",
    dp_transportation="105",
    dp_camera="128",
    # Settings DPs, added 2026-09-05. See the DP-MAPS-EXTRACTED.md TUYA_SLAM
    # section: 130/132/133/135 are tabled by the vendor for this family but
    # were absent from a live raw-status read of the reference unit -- see the
    # presence-gating rule in select.py for why that is not a contradiction.
    dp_water_level="20",
    dp_floor_type_detection="112",
    dp_self_empty_power="124",
    dp_extending_arms="130",
    dp_mop_dry_duration="132",
    dp_mop_wash_temperature="133",
    dp_mop_maintenance_strategy="134",
    dp_dock_task_self_empty="135",
    dp_mute_switch="107",
    dp_cliff_sensor_switch="111",
    dp_quick_clean_use_global_vacuum="122",
    dp_volume="108",
    water_level=_frozen(
        {"CLOSED": "closed", "LOW": "low", "MIDDLE": "middle", "HIGH": "high"}
    ),
    floor_type_detection=_frozen(
        {
            "ON": "on",
            "OFF": "off",
            "ON_WITH_CARPET_BOOST": "on_with_carpet_boost",
            "ON_AVOID_NOMOP": "on_avoid_nomop",
        }
    ),
    self_empty_power=_frozen({"STRONG": "strong", "NORMAL": "normal"}),
    mop_maintenance_strategy=_frozen(
        {
            "HEAVY": "heavy",
            "MEDIUM": "medium",
            "LIGHT": "light",
            "MANUAL": "manual",
            "MAX": "max",
        }
    ),
    extending_arms=_frozen(
        {
            "OFF": "off",
            "BRUSH_SIDE_AND_MOP": "brush_side_and_mop",
            "BRUSH_SIDE": "brush_side",
            "MOP": "mop",
        }
    ),
    mop_dry_duration=_frozen(
        {"DEFAULT": "default", "QUIET": "quiet", "MANUAL": "manual"}
    ),
    mop_wash_temperature=_frozen(
        {"ROOM": "room", "HEATED_WASH": "heated_wash", "HEATED_MOP": "heated_mop"}
    ),
    dock_task_self_empty=_frozen(
        {
            "OFF": "off",
            "MOP_WASH": "mop_wash",
            "MOP_DRY": "mop_dry",
            "DUSTBIN_EMPTY": "dustbin_empty",
        }
    ),
    work_mode=_frozen(
        {
            "AUTO_CLEANING": "smart",
            "CHARGE": "chargego",
            "ZONE": "zone",
            "POSE": "pose",
            "PART": "part",
            "FOLLOW_WALL": "wallfollow",
            "SELECTROOM": "selectroom",
            "QUICKMAP": "quick_map",
            "VAC_ONLY": "vac_only",
        }
    ),
    clean_direction=_frozen(
        {
            "FORWARD": "forward",
            "BACKWARD": "backward",
            "TURN_LEFT": "turn_left",
            "TURN_RIGHT": "turn_right",
            "STOP": "stop",
        }
    ),
    status=_frozen(
        {
            "STOP": "standby",
            "GO": "smart",
            "ZONE_CLEAN": "zone_clean",
            "PART_CLEAN": "part_clean",
            "CLEANING": "cleaning",
            "PAUSE": "paused",
            "GOTO_POS": "goto_pos",
            "POS_ARRIVED": "pos_arrived",
            "POS_UNARRIVE": "pos_unarrive",
            "DOCKING": "goto_charge",
            "CHARGING": "charging",
            "CHARGE_FINISH": "charge_done",
            "SLEEP": "sleep",
            "WALL_CLEAN": "wall_clean",
            "SELECTROOM": "select_room",
            "MAP_HOUSEKEEPING": "map_housekeeping",
            "MAP_OPERATION": "map_operation",
            "RELOCALIZING": "relocalizing",
            "SELF_EMPTY_INPROGRESS": "dustbin_emptying",
            "QUICK_MAP": "quick_map",
            "POWER_OFF": "power_off",
            "RE_DOCK": "redock",
            "VAC_ONLY": "vac_only",
            "MOP_WASH": "mop_wash",
            "MOP_WATER_REFILL": "mop_water_refill",
            "CHARGING_AND_MOP_DRY": "charging_and_mop_dry",
            "CHARGE_DONE_AND_MOP_DRY": "charge_done_and_mop_dry",
            "MOP_WASH_REFRESH": "mop_wash_refresh",
            "GOTO_CHARGE_MAINT": "goto_charge_maint",
        }
    ),
    fan_speed=_frozen(
        {
            "QUIET": "gentle",
            "STANDARD": "normal",
            "STRONG": "strong",
            "CLOSED": "closed",
        }
    ),
    fan_speeds=("gentle", "normal", "strong"),
    mode_auto="AUTO_CLEANING",
    mode_charge="CHARGE",
    mode_spot="PART",
    # SLAM has no standalone standby work mode; stopping clears COMMAND_ENABLE.
    mode_stop=None,
    status_cleaning=frozenset(
        {
            "smart", "cleaning", "zone_clean", "part_clean", "wall_clean",
            "select_room", "quick_map", "vac_only", "goto_pos", "map_operation",
        }
    ),
    status_returning=frozenset({"goto_charge", "redock", "goto_charge_maint"}),
    # Only charge states prove the robot is on the dock. `standby` and `sleep`
    # are where it lands after a stop or a failed return anywhere on the floor
    # (observed repeatedly in the validation unit's status history); a real
    # docking always ends in `charging`.
    status_docked=frozenset(
        {
            "charging", "charge_done",
            "charging_and_mop_dry", "charge_done_and_mop_dry",
        }
    ),
    status_paused=frozenset({"paused"}),
    dp_docking="102",
    dp_pause="101",
    # everything else (dustbin_emptying, mop_wash, relocalizing, ...) -> idle
)


# --- TUYA_VISION -------------------------------------------------------------
# Bob PetHair Vision / Vision Plus. Camera-navigated, no LiDAR, no dock features.
# Work mode stays on DP 2 and the error bitmask on DP 8 while everything else
# lives in the 207-218 block — that layout is verbatim from the app, not a slip.
VISION = FamilySpec(
    key=FAMILY_VISION,
    dp_power="207",
    dp_mode="2",
    dp_direction="208",
    dp_status="209",
    dp_battery="210",
    # Vision reports no consumable life and exposes no consumable resets.
    dp_side_brush_life=None,
    dp_roll_brush_life=None,
    dp_filter_life=None,
    dp_reset_side_brush=None,
    dp_reset_roll_brush=None,
    dp_reset_filter=None,
    dp_locate=None,                 # no COMMAND_SEEK_ROBOT
    dp_fan="211",
    dp_clean_record="212",          # COMMAND_CLEAN_LOG
    dp_clean_area="213",            # COMMAND_CLEAN_AREA
    dp_clean_time="214",            # COMMAND_CLEAN_DURATION
    dp_error="8",
    dp_error2=None,                 # single error bitmask only
    dp_error3=None,                 # no COMMAND_FAULT_BITS on this family
    fault_bits=VISION_FAULT_BITS,
    dp_mute="215",
    dp_position="216",
    dp_language="217",
    dp_cliff_sensor="218",
    # Settings DPs, added 2026-09-05. Vision has only the mute switch among
    # this batch; every other settings DP above is None for this family.
    dp_mute_switch="215",
    work_mode=_frozen(
        {
            "AUTO_CLEANING": "smart",
            "CHARGE": "chargego",
            "STOP": "standby",
            "FOLLOW": "follow",
            "MANUAL": "manual",
            "SPOT": "spot",
            "SINGLE": "single",
        }
    ),
    clean_direction=_frozen(
        {
            # 'foward' is the vendor's own typo in the app bundle. The device
            # expects it spelled that way — do not "correct" it.
            "FORWARD": "foward",
            "BACKWARD": "backward",
            "TURN_LEFT": "turn_left",
            "TURN_RIGHT": "turn_right",
            "STOP": "stop",
        }
    ),
    status=_frozen(
        {
            "STOP": "idle",
            "GO": "smart_clean",
            "FOLLOW_WALL": "follow_wall",
            "SPOT": "spot",
            "DOCKING": "docking",
            "CHARGING": "charging",
            "CHARGE_FINISH": "charge_finish",
            "MANUAL": "manual",
            "CLEAN_FINISH": "clean_finish",
            "SINGLE": "single",
            "PAUSE": "pause",
        }
    ),
    fan_speed=_frozen(
        {
            "QUIET": "quiet",
            "STANDARD": "standard",
            "STRONG": "strong",
        }
    ),
    # Vision has no CLOSED fan value, so every enum value is user-selectable.
    fan_speeds=("quiet", "standard", "strong"),
    mode_auto="AUTO_CLEANING",
    mode_charge="CHARGE",
    mode_spot="SPOT",
    mode_stop="STOP",               # Vision can be told to stand by explicitly
    status_cleaning=frozenset(
        {
            "smart_clean", "follow_wall", "spot", "single",
            # 'manual' is the user jogging the robot with the direction pad; the
            # brushes run, so HA-wise it is closer to CLEANING than IDLE.
            "manual",
        }
    ),
    status_returning=frozenset({"docking"}),
    # Unlike SLAM, Vision's STOP value ('idle') does not imply the robot is on
    # the dock — it is reported wherever the robot is simply stopped. Only the
    # charge states are treated as DOCKED here.
    status_docked=frozenset({"charging", "charge_finish"}),
    status_paused=frozenset({"pause"}),
    # 'clean_finish' means the run completed; the robot is stationary and not
    # necessarily docked, so it falls through to IDLE.
)


# --- TUYA_RANDOM -------------------------------------------------------------
# bObsweep Leaf / Charlotte. Bump-and-turn navigation, no mapping, but with a
# real water-control DP that the other two families lack.
RANDOM = FamilySpec(
    key=FAMILY_RANDOM,
    dp_power="2",
    dp_mode="3",
    dp_direction="4",
    dp_status="5",
    dp_battery="6",
    dp_side_brush_life=None,
    dp_roll_brush_life=None,
    dp_filter_life=None,
    dp_reset_side_brush=None,
    dp_reset_roll_brush=None,
    dp_reset_filter=None,
    dp_locate=None,                 # no COMMAND_SEEK_ROBOT
    dp_fan="14",
    dp_clean_record="102",          # COMMAND_LOG
    dp_clean_area=None,             # no cleaning-area DP
    dp_clean_time="17",
    dp_error="18",
    dp_error2=None,
    dp_error3=None,                 # no COMMAND_FAULT_BITS on this family
    fault_bits=RANDOM_FAULT_BITS,
    dp_water_control="20",
    # One combined attachment DP replaces SLAM's separate mop (118)/vacuum (119).
    dp_status_dustbin_watertank="103",
    # Settings DPs, added 2026-09-05. Random shares DP 20 with SLAM but only
    # a three-value enum (no MIDDLE) -- see DP-MAPS-EXTRACTED.md TUYA_RANDOM.
    # Every other settings DP above is None for this family.
    dp_water_level="20",
    water_level=_frozen({"CLOSED": "closed", "LOW": "low", "HIGH": "high"}),
    work_mode=_frozen(
        {
            "CHARGE": "chargego",
            "WALL_FOLLOW": "wall_follow",
            "STAND_BY": "standby",
            "SPIRAL": "spiral",
            "RANDOM": "random",
            "PARTIAL_BOW": "partial_bow",
        }
    ),
    clean_direction=_frozen(
        {
            "FORWARD": "forward",
            "BACKWARD": "backward",
            "TURN_LEFT": "turn_left",
            "TURN_RIGHT": "turn_right",
            "STOP": "stop",
        }
    ),
    status=_frozen(
        {
            "STOP": "standby",
            "GO": "smart_clean",
            "WALL_CLEAN": "wall_clean",
            "SPOT_CLEAN": "spot_clean",
            "ROOM_CLEAN": "room_clean",
            "DOCKING": "goto_charge",
            "CHARGING": "charging",
            "CHARGE_FINISH": "charge_done",
            "CLEANING": "cleaning",
            "SLEEP": "sleep",
            "IN_TROUBLE": "in_trouble",
        }
    ),
    fan_speed=_frozen(
        {
            "QUIET": "gentle",
            "STANDARD": "normal",
            "STRONG": "strong",
        }
    ),
    # Random shares SLAM's fan vocabulary minus CLOSED.
    fan_speeds=("gentle", "normal", "strong"),
    water_control=_frozen({"CLOSED": "closed", "LOW": "low", "HIGH": "high"}),
    dustbin_watertank=_frozen(
        {
            "FAN_IN": "fan_in",
            "FAN_TANK_IN": "fan_tank_in",
            "FAN_TANK_NONE": "fan_tank_none",
        }
    ),
    # A random-pattern robot has no "smart" mode; RANDOM is its normal full run.
    mode_auto="RANDOM",
    mode_charge="CHARGE",
    # No true spot mode exists; SPIRAL is this class of robot's spot clean and is
    # what the app surfaces behind its spot button.
    mode_spot="SPIRAL",
    mode_stop="STAND_BY",
    status_cleaning=frozenset(
        {"smart_clean", "wall_clean", "spot_clean", "room_clean", "cleaning"}
    ),
    status_returning=frozenset({"goto_charge"}),
    # Matches SLAM's precedent: 'standby'/'sleep' are the parked states for this
    # class of robot, which parks on the dock.
    status_docked=frozenset({"charging", "charge_done", "standby", "sleep"}),
    # Random has no pause status at all.
    status_paused=frozenset(),
    # Random is the only family with an explicit fault status value.
    status_error=frozenset({"in_trouble"}),
)


FAMILIES: dict[str, FamilySpec] = {
    FAMILY_SLAM: SLAM,
    FAMILY_VISION: VISION,
    FAMILY_RANDOM: RANDOM,
}


def resolve_family(model_family: str | None) -> FamilySpec:
    """Return the FamilySpec for a config entry's `model_family` value.

    Falls back to SLAM for a missing or unrecognised value so config entries
    written before the selector existed (and any future typo) keep working
    exactly as they do today rather than failing setup.
    """
    if model_family is None:
        return SLAM
    return FAMILIES.get(model_family, SLAM)
