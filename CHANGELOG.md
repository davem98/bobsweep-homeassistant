# Changelog

All notable changes to this integration. Versions follow
[semantic versioning](https://semver.org/); dates are ISO-8601.

Everything here was developed against, and validated on, a single real robot —
an **UltraVision Pet Combo** (`slam` family). The `vision` and `random`
families are derived from the vendor app's own datapoint tables and remain
**unvalidated on hardware**; reports from owners of those models are welcome.

## [0.4.0] — 2026-09-06

### Added
- **`bobsweep.clear_room_name`** — removes a name you set, so any name implied
  by the robot's own schedules becomes visible again. Setting a name was
  previously a one-way door.
- Schedules now report the **weekdays** they run on and the **vacuum power** and
  **mop intensity** they use, alongside the raw values.

### Fixed
- **Room id 0 could not be named.** The service schema required an id of 1 or
  more, but 0 is a real room — the reference robot's own schedule store names
  room 0 "classroom".
- **The declared minimum Home Assistant version was wrong.** `hacs.json` said
  2024.8.0 while the code needs `VacuumActivity`, which arrived in 2025.1. An
  install on an older core would have been allowed and would have failed at
  import.

### Protocol notes
- The schedule weekday bitmask is **bit 0 = Monday** through bit 6 = Sunday.
  Measured, not assumed: a schedule saved for Wednesday alone came back as
  `0x04`. This **contradicts** the vendor app's own JavaScript weekday enum
  (Sunday = 0), which belongs to an older per-day schedule the robot does not
  use here — the reason it had to be measured.
- Two of the four trailing bytes on a schedule entry are now identified: byte 1
  is the vacuum power and byte 2 the mop intensity, on the app's own scales.
  Bytes 0 and 3 remain unknown and the field is still exposed as raw hex.

## [0.3.0] — 2026-09-06

### Added
- **Saved maps, schedules and mop-cloth dirt** are read from the robot using
  the vendor app's own read-only queries, once at startup and on demand via the
  new **`bobsweep.refresh_robot_info`** service.
- **Room names.** The robot has no "what is this room called" command, but its
  stored schedules carry the room ids they target *and* the name their owner
  typed, so a single-room schedule names that room. Names are derived from
  those, and **`bobsweep.set_room_name`** persists your own, which win over the
  derived ones. A schedule covering two rooms names a pair and is deliberately
  not split.
- New diagnostic sensors: saved maps, schedules, mop cloth dirt.

### Changed
- The read-only Random water-control sensor was removed; the writable water
  level select supersedes it.

### Security
- Every write to the transparent command datapoint is checked against an
  allowlist of the vendor app's own read-only queries immediately before it
  goes out. A guessed frame on that datapoint once started an unwanted job.

## [0.2.0] — 2026-09-05

### Added
- **The coordinator now listens** rather than only polling. It owns the device
  on one I/O thread with a continuous receive loop, a heartbeat, an
  authoritative poll every 30 s, a command queue, reconnect with backoff and
  clean shutdown. Polling every 15 s sampled at most one of roughly 30 messages
  the robot pushes per window, and could never observe a one-shot frame.
- **Passive path trail** (DP 104) harvested into a real position source, used
  for room classification while it is fresh.
- **Selected rooms**, tracked from the robot's one-shot room-clean
  acknowledgement, which a poll can never see.
- **Settings entities**, each created only when the family has the datapoint
  *and* the unit actually reports it: water level, floor type detection,
  self-empty power, mop maintenance strategy, extending arms, mop dry duration,
  mop wash temperature, dock task self-empty, mute, cliff sensor, auto-empty,
  quick-clean-uses-global-vacuum, and a volume slider.

### Fixed
- **Return to base did nothing during a clean.** It wrote the charge work mode;
  the vendor app writes a dedicated docking datapoint, which works mid-job.
- **Stop and pause** now mirror the app: stop clears the enable switch while
  cleaning and cancels docking while returning; pause uses the dedicated pause
  datapoint.
- **`bobsweep.set_dp` stringified booleans.** The robot silently ignores a
  boolean datapoint written as `"True"`. Values now keep their types.
- **`standby` was treated as docked.** It is where the robot sits after a stop
  anywhere on the floor; only charge states mean docked.

## [0.1.0] — 2026-09-01

Initial release: local control over the Tuya LAN protocol with no cloud, three
model families behind a per-family datapoint table, named fault decoding, the
DP 105 command-channel codec, detected obstacles, the camera switch, and the
zone/room-awareness layer.
