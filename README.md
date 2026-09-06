# bObsweep (local) — Home Assistant integration

Unofficial, **local-only** control of bObsweep robot vacuums from Home
Assistant. No cloud, no bObsweep account needed at runtime — the integration
talks straight to the robot over the Tuya LAN protocol via
[`tinytuya`](https://github.com/jasonacox/tinytuya).

**Unofficial. Not affiliated with, endorsed by, or supported by bObsweep or
Tuya.** See [Disclaimer](#disclaimer).

## The honest barrier — read this before installing

This is not a two-click install. Home Assistant's own local Tuya integrations
need three things per device: a **host** (LAN IP), a **`device_id`**, and a
**`local_key`**. bObsweep's app gives you no way to read that `local_key` —
it's cloud-locked and, at rest on the phone, sits in an Android-Keystore
encrypted store that can't be decrypted offline.

The only path found so far is to run the bObsweep app in a **rooted Android
environment** and pull the key out of the app's live memory with a dynamic
instrumentation tool. That means getting a rooted, ARM-capable Android
environment set up, signing into the app there, and hooking the running
process — a real afternoon of work the first time, not a quick setup wizard.

**See [`RUNBOOK.md`](RUNBOOK.md)** for what's actually involved — it orients
you on the approach rather than walking through exact commands; the details
depend on the tooling you choose. Do that first; only come back here once you
have the three values in hand. The key is static per-pairing, so this is a
one-time cost — after that, control is fully local and keeps working even if
bObsweep's cloud changes or the app is pulled.

## What you need to configure it

| Value | What it is | Where it comes from |
|---|---|---|
| `host` | The vacuum's LAN IP | Scan for the Tuya LAN protocol port; see `RUNBOOK.md`. **DHCP-reserve it** — see [Troubleshooting](#troubleshooting). |
| `device_id` | The Tuya device id | Extracted alongside `local_key` — see `RUNBOOK.md`. |
| `local_key` | The 16-character per-device Tuya secret | Extracted from the app's live memory — see `RUNBOOK.md`. Never share or commit this. |
| `protocol_version` | Tuya LAN protocol version | Try `3.3` first; if you get `cannot_connect`, try `3.4`, then `3.5`. |
| `model_family` | Which datapoint map to use | `slam`, `vision`, or `random` — see the table below. |

## Supported models

The vacuum's Tuya datapoint (DP) layout depends on which internal "family" it
belongs to. The mapping below is transcribed verbatim from the bObsweep app's
own model tables (the vendor Android app's JavaScript bundle).

| Family | Models | Validation status |
|---|---|---|
| `slam` | bObsweep SLAM, Austin, Dustin, Dustin Plus, Dustin Combo, Appetite, Phoenix, Archer, Orbi, Bio, Maxim, UltraVision, UltraVision Pet, UltraVision Pet Combo | **Validated on real hardware** (an UltraVision Pet Combo, running live in HA). |
| `vision` | Bob PetHair Vision, Bob PetHair Vision Plus | Derived from the app's own DP tables. **Not yet hardware-validated.** |
| `random` | bObsweep Leaf, Charlotte | Derived from the app's own DP tables. **Not yet hardware-validated.** |

**Not supported, and never will be:** Bob PetHair, Bob PetHair Plus, Bob
Standard, Bob Pro, bObi Pet, bObi Classic. These are pre-WiFi / IR-only units
with no Tuya datapoints at all (`isWiFiControllable` is false for them in the
app) — there is nothing for this integration to talk to.

If you run a `vision` or `random` unit, please open an issue with what worked
and what didn't — that's how those families get promoted to validated.

## Entities

One vacuum entity plus sensors, switches and settings, all backed by one
coordinator. The coordinator keeps a persistent local socket open and
**listens** on it: the robot pushes most of its state unprompted (battery,
status, command-channel frames several times a second during a job), and a
full status poll every 30 seconds backs that up. Every entity is resolved against the datapoint table for your
configured `model_family` — if a family has no datapoint for a given reading
(e.g. Vision has no brush/filter-life DPs), that entity is simply not created
rather than showing up permanently unknown.

- **`vacuum.<name>`** — `StateVacuumEntity`. Supports start, pause, stop,
  return-to-base, fan speed (vocabulary and available speeds vary per family —
  e.g. `gentle`/`normal`/`strong` on SLAM and Random, `quiet`/`standard`/
  `strong` on Vision), locate (SLAM only — Vision/Random have no locate DP),
  spot clean, and a raw `send_command` passthrough. Exposes extra attributes
  including clean area/time, brush/filter life where the family reports them,
  and error state.
- **Sensors** (diagnostic unless noted): side brush life (%), rolling brush
  life (%), filter life (%), battery (%), last clean area (m², not marked
  diagnostic), last clean time (min, not marked diagnostic), status (raw DP
  string), **fault** (the current fault by name, `none` when healthy), and
  **current room** (see [Room awareness](#room-awareness)) — plus
  family-specific extras: **dustbin / water tank** status (Random only) and
  **cliff sensor** (Vision only, raw value).

  **Detected obstacles** (SLAM only) is the count of objects the robot's camera
  has found during the current job, with the list on an `objects` attribute as
  `{x, y, class}` — classes are the vendor's own (`wire`, `shoes`, `socks`,
  `toys`, `chair`, `table`, `trash_can`, `potted_plant`, `bowl`, `key`, `other`,
  `unknown`). Coordinates are raw map cells in the same frame as the robot's
  no-go zones. The list accumulates over a run rather than describing what is in
  front of the robot right now, and the robot refines an object's coordinates
  as it gets a better look, so treat the positions as approximate. The sensor
  reads *unknown* until the robot sends its first report; `0` is a real value
  meaning "nothing found yet".

  **Path trail** (SLAM only, diagnostic) is the robot's own point count for the
  current cleaning path, with the last known point and the path id as
  attributes. It fills in only while something is driving the robot's map
  session — today that means the vendor app's map screen being open (see
  [Room awareness](#room-awareness)).

  **Selected rooms** (SLAM only, diagnostic) records the room ids the robot
  acknowledged when a room-targeted clean was started from the app. The robot
  emits this once per job and never reports it on a poll, which is why the
  coordinator listens. The payload layout is not fully pinned down yet; the
  raw bytes and the decoder's guess at the layout are on the attributes. Where
  a room's name is known it is shown alongside the id — see
  [Room names](#room-names).

  **Saved maps**, **Schedules** and **Mop cloth dirt** (SLAM only, diagnostic)
  come from the robot's own read-only queries rather than from datapoints. The
  integration asks once at startup, and again whenever you call
  `bobsweep.refresh_robot_info`. Saved maps lists each stored floor map's id
  and name; Schedules lists the robot's stored cleaning schedules with their
  weekdays, times, target room ids, vacuum power, mop intensity and names; Mop
  cloth dirt is a percentage. All three read *unknown* until the robot answers.
- **Binary sensors**: self-emptying (SLAM only), charging, docked, problem
  (with error attributes), mopping, vacuuming. Each is only created when the
  configured family actually has the underlying datapoint(s) or status value
  it depends on.
- **Settings** (configuration category; each created only when the family has
  the datapoint *and* your unit actually reports it, because the vendor's
  tables describe the product line, not every firmware): selects for **water
  level** (SLAM and Random, with the family's own option list), **floor type
  detection**, **self-empty power**, **mop maintenance strategy**, and, on
  units that report them, **extending arms**, **mop dry duration**, **mop wash
  temperature** and **dock task self-empty**; switches for **mute** (SLAM and
  Vision), **cliff sensor**, **auto-empty** and **quick clean uses global
  vacuum settings**; and a **volume** slider.
- **Switch**: **Camera obstacle detection** (SLAM only) — turns the robot's
  on-board object-detection camera on and off (DP 128). This is the only
  user-facing control over the camera on the device, and it works entirely
  locally. bObsweep states the images are processed on the robot and never
  uploaded; that is the vendor's claim about their firmware, not something this
  integration can verify, which is rather the point of exposing the switch.
  Turning it off also stops the *Detected obstacles* reports, which come from
  the same detector.

## Room awareness

The integration can group the robot's position into rooms you teach it, and
report the current one on `sensor.<name>_current_room`. The zone capture and
management services are listed below.

The honest caveat: **this robot only reports where it is while the vendor
app's map screen is open.** The path-trail datapoint (DP 104) carries real
coordinates — `{"cmd":102,...,"point":[[x,y],...]}` — and the integration
harvests every point it sees and uses the newest one as the robot's position
for up to 90 seconds. But the robot streams those points only while the app's
map session is driving it; requests issued by this integration go unanswered
(so far — see below). In practice that means:

- **Teaching zones works.** Open the app's map screen, drive or send the robot
  around a room, and run the capture services: positions flow.
- **Unattended cleans have no position.** With the app closed, `current_room`
  reads *unknown* — the truthful answer, and deliberately not the same state
  as `unmapped` (which means the position is known and is in no room you have
  taught). The `last_known_room` attribute keeps the last confident answer.

There is one opt-in approximation, off by default, under the integration's
**Configure** button:

- **Estimate position from obstacle detections.** New obstacle detections happen
  roughly where the robot is standing, so they can stand in for a position feed.
  They are sparse (a handful per clean, none at all in a tidy room),
  event-driven, and offset by however far ahead the camera saw. Room changes
  still need several agreeing readings before they are confirmed, but with
  evidence this thin the sensor can name the wrong room. Enable it if a rough
  answer beats no answer for you; don't build anything that matters on it.

  The *Detected obstacles* sensor works either way — this setting only controls
  whether obstacle positions are also used to guess the room.

## Room names

The robot will not tell you what a room is called if you ask it — the vendor
app has no such command, and the map data it sends carries no names. But it
does hand over its **stored cleaning schedules**, and each one carries both the
room ids it targets and the name its owner typed. A schedule for a single room
is therefore the robot naming that room.

The integration reads those schedules at startup and derives an id → name map
from the single-room entries only. A schedule covering two rooms ("hall and
kitchen" for ids 3 and 2) names a *pair*, and which half is which is genuinely
unknowable from the data, so those are deliberately not split.

Anything left unnamed you can name yourself with `bobsweep.set_room_name`, and
undo with `bobsweep.clear_room_name`; your names are stored by this
integration, survive restarts, and win over the derived ones. The result
appears as a `room_names` attribute on the Schedules and Selected rooms
sensors.

## Services

Registered as entity services on the `vacuum` domain (`integration: bobsweep`):

| Service | Purpose |
|---|---|
| `bobsweep.refresh_robot_info` | Re-read the robot's saved maps, schedules, room names and mop-cloth status. Read-only; done once automatically at startup. |
| `bobsweep.set_room_name` | Give a room id a name of your own, overriding anything derived from the robot's schedules. See [Room names](#room-names). |
| `bobsweep.clear_room_name` | Forget a name you set, so the robot-derived one shows again. |
| `bobsweep.set_mode` | Write a raw Tuya work-mode value directly (zone clean, follow-wall, select-room, quick-map, vacuum-only, etc.) — reaches modes the standard vacuum start/pause/stop controls don't expose. |
| `bobsweep.empty_dustbin` | Trigger the auto-empty dock. |
| `bobsweep.set_dp` | Advanced/debug: write an arbitrary raw Tuya datapoint by id. Intended for development and troubleshooting, not routine use. |
| `bobsweep.capture_point` | Record the robot's current position into a scratch buffer under a label. Fails with an explanation when position is unavailable. |
| `bobsweep.start_zone_capture` / `bobsweep.stop_zone_capture` | Drive the robot around a room while recording its positions, then save the sampled footprint as a named zone. |
| `bobsweep.delete_zone` | Delete a stored zone by name. |
| `bobsweep.export_zones` / `bobsweep.import_zones` | Back up or restore the whole zone document (returns/accepts service response data). |

The zone services all depend on the robot reporting a position, which today
means having the vendor app's map screen open while you capture — see
[Room awareness](#room-awareness).

Note on the standard commands: on the SLAM family they mirror what the vendor
app sends. Return-to-base writes the dedicated docking datapoint (DP 102),
which works mid-clean — writing the `chargego` work mode alone does not
redirect a running job. Stop clears the enable switch while cleaning and
cancels the docking datapoint while returning. Pause writes the dedicated
pause datapoint (DP 101), transcribed from the app but not yet exercised on
hardware — reports welcome. Vision and Random use their explicit standby work
mode and have no separate pause.

The raw `bobsweep.set_dp` service keeps value types: `true`/`false` become
booleans and digit strings become integers, because the robot silently ignores
a boolean datapoint written as the string `"True"`.

## Installation

### HACS (custom repository)

1. HACS → the "..." menu (top right) → **Custom repositories**.
2. Add this repo's URL, category **Integration**.
3. Install **bObsweep (local)** from HACS, then restart Home Assistant.

### Manual

1. Copy `custom_components/bobsweep/` into your `config/custom_components/`
   directory (so you end up with `config/custom_components/bobsweep/...`).
2. Restart Home Assistant.

### Either way, then:

Settings → Devices & Services → **Add Integration** → **bObsweep (local)** →
enter `host`, `device_id`, `local_key`, `protocol_version`, `model_family`.

## Troubleshooting

- **`cannot_connect` during setup.** Almost always the wrong
  `protocol_version`. Try `3.3`, `3.4`, `3.5` in turn — the UltraVision Pet
  Combo used for validation needed `3.4`, not the more common `3.3` default.
- **Sensors intermittently go blank / battery updates but nothing else does.**
  Expected and handled: the robot pushes *partial* datapoint updates over its
  persistent socket (e.g. just `{"6": <battery>}`). The coordinator merges
  every datapoint it has ever seen rather than replacing on each poll — if you
  see this behavior in a debug log it isn't a bug, but if entities never
  populate after the first partial push, open an issue.
- **The integration stops working after your router hands out a new IP.**
  Give the vacuum a **DHCP reservation** (a fixed IP tied to its MAC) in your
  router — the local Tuya connection has no discovery/re-resolution, it's a
  fixed `host` in the config entry.

## Disclaimer

This is an **unofficial, independent interoperability project**, built by
reverse-engineering the bObsweep Android app to talk to hardware the author
owns. It is **not affiliated with, endorsed by, or supported by bObsweep or
Tuya Inc.** "bObsweep" and "Tuya" are trademarks of their respective owners,
referenced here only to describe compatibility. No bObsweep or Tuya code,
binaries, or brand assets are included in this repository — only original
integration code and a datapoint map transcribed as functional, non-creative
facts. Provided **as-is**, for personal use with hardware you own; use at your
own risk. See [`LICENSE`](LICENSE) for the full terms.

## How it works

The datapoint tables backing each model family were transcribed verbatim from
the vendor app's bundle; the full tables are not published in this repo.
`RUNBOOK.md` orients you on the approach that actually works, above.
