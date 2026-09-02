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

One vacuum entity plus a set of diagnostic sensors, all backed by the same
coordinator (polls the device every 15 seconds over a persistent local
socket). Every entity is resolved against the datapoint table for your
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
  family-specific extras: **water control** and **dustbin / water tank** status
  (currently modelled for Random only), **cliff sensor** (currently modelled for
  Vision only, raw value).

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

  Note: the SLAM family also has water-control (DP 20, with a fourth `middle`
  level) and mute/volume/cliff-sensor datapoints that this integration does not
  yet model. (The full per-family datapoint tables are not published in this
  repo.)
- **Binary sensors**: self-emptying (SLAM only), charging, docked, problem
  (with error attributes), mopping, vacuuming, muted (Vision only). Each is
  only created when the configured family actually has the underlying
  datapoint(s) or status value it depends on.
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

The honest caveat: **this robot does not reliably report where it is, yet.**
The datapoint that looks like a position trail (DP 104) *does* carry one — the
robot returns `{"cmd":102,...,"point":[[x,y],...]}` with real coordinates — but
so far it only does so while the vendor app's map screen is open and driving
the request loop. Requests issued by this integration go unanswered, so the
trail cannot be relied on as a live position source. By default `current_room`
therefore reads *unknown* — the truthful answer, and deliberately not the same
state as `unmapped` (which means the position is known and is in no room you
have taught).

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

## Services

Registered as entity services on the `vacuum` domain (`integration: bobsweep`):

| Service | Purpose |
|---|---|
| `bobsweep.set_mode` | Write a raw Tuya work-mode value directly (zone clean, follow-wall, select-room, quick-map, vacuum-only, etc.) — reaches modes the standard vacuum start/pause/stop controls don't expose. |
| `bobsweep.empty_dustbin` | Trigger the auto-empty dock. |
| `bobsweep.set_dp` | Advanced/debug: write an arbitrary raw Tuya datapoint by id. Intended for development and troubleshooting, not routine use. |
| `bobsweep.capture_point` | Record the robot's current position into a scratch buffer under a label. Fails with an explanation when position is unavailable. |
| `bobsweep.start_zone_capture` / `bobsweep.stop_zone_capture` | Drive the robot around a room while recording its positions, then save the sampled footprint as a named zone. |
| `bobsweep.delete_zone` | Delete a stored zone by name. |
| `bobsweep.export_zones` / `bobsweep.import_zones` | Back up or restore the whole zone document (returns/accepts service response data). |

The zone services all depend on the robot being able to report a position, so
they are only useful with the opt-in source above enabled — see
[Room awareness](#room-awareness).

Note on `async_pause`/`async_stop`: SLAM-family robots have no dedicated pause
datapoint; both are currently implemented as toggling the enable switch off.
This works but may need field-tuning against your specific firmware — reports
welcome.

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
