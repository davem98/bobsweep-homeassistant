# Changelog

All notable changes to this integration. Versions follow
[semantic versioning](https://semver.org/); dates are ISO-8601.

Everything here was developed against, and validated on, a single real robot —
an **UltraVision Pet Combo** (`slam` family). The `vision` and `random`
families are derived from the vendor app's own datapoint tables and remain
**unvalidated on hardware**; reports from owners of those models are welcome.

## [Unreleased]

### Added
- **Room-targeted cleaning.** `bobsweep.clean_rooms` starts a clean of one or
  more of the robot's rooms by name or id, sending the vendor app's own
  room-clean command — one frame on the command channel, confirmed on
  hardware. On Home Assistant 2026.3+ the same rooms are exposed as vacuum
  segments, so `vacuum.clean_area` and the "clean the kitchen" voice intent
  work once segments are mapped to areas in the vacuum entity's settings.
- **Stuck alerts with a best-effort location.** A new
  `binary_sensor.<name>_stuck` turns on when a fault meaning "cannot move on
  its own" appears (`bob_stuck`, wheel, bumper, cliff-sensor and boxed-in
  faults), with the rooms the job was told to clean, the last known position
  (source, exactness and age), the taught zone it falls in, the nearest
  reported obstacle and a human `message` as attributes. A persistent
  notification is raised and dismissed with it, `sensor.<name>_last_stuck`
  keeps the most recent event, and `bobsweep_stuck` /
  `bobsweep_stuck_cleared` fire on the event bus. The robot sends no position
  with the fault, so the location comes from the path trail and is reported
  as *unknown* when no map session was open.
- **Obstacle insights.** Every genuinely-new camera sighting is now recorded
  — class, position, zone, job rooms, job id — in a bounded, persisted
  history (`bobsweep_obstacles.<entry_id>`). `sensor.<name>_last_obstacle`
  shows the newest; `sensor.<name>_obstacle_insights` counts this job's
  obstacles and carries per-class counts (this job and the last 30 days),
  hotspots where things keep turning up across jobs, and per-job summaries.
  `bobsweep_obstacle_detected` fires per sighting;
  `bobsweep.clear_obstacle_history` wipes the history. SLAM only.
- Cleaning-job boundaries are inferred from the status datapoint (parked ->
  cleaning opens a job; pauses, mop washes and relocalisation do not split
  one), and a room selection is attached to a job only when its
  acknowledgement arrived for that job.

### Changed
- Vacuum segments are now the robot's own rooms. They were previously the
  zones taught through this integration, which could be listed but never
  cleaned because no zone-clean command had been verified.

### Fixed
- **The room-selection payload was misread.** Its layout is
  `[sweep count, room count, room ids…]`, settled against the app's code and
  a two-room job on hardware; the decoder had treated the leading byte as the
  room count, so the one-room sample looked like "one room, one pass" and any
  multi-room acknowledgement read as unknown. The `passes` attribute of the
  Selected rooms sensor is gone; `sweeps` replaces it.

## [0.4.1] — 2026-09-13

### Fixed
- **Deleting the integration left two files behind.** The saved zones and room
  names for a config entry live in Home Assistant's storage under keys derived
  from the entry id; nothing removed them when the entry was deleted. They are
  now removed with it.
- **The manifest declared `local_polling`.** The coordinator has listened on a
  persistent socket since 0.2.0; the integration is `local_push`.

### Changed
- The coordinator now passes its config entry to Home Assistant's coordinator
  base class, ahead of Home Assistant requiring it.
- A retired internal flag that once gated the path-trail source was removed;
  the finding it recorded now lives in a comment.

## [0.4.0] — 2026-09-06

### Added
- **`bobsweep.clear_room_name`** — removes a name you set, so any name implied
  by the robot's own schedules becomes visible again. Setting a name was
  previously a one-way door.
- Schedules now report the **weekdays** they run on and the **vacuum power** and
  **mop intensity** they use, alongside the raw values.

### Fixed
- **Room id 0 could not be named.** The service schema required an id of 1 or
  more, but 0 is a valid room id, which left that room permanently unnameable.
- **The declared minimum Home Assistant version was wrong.** `hacs.json` said
  2024.8.0 while the code needs `VacuumActivity`, which arrived in 2025.1. An
  install on an older core would have been allowed and would have failed at
  import.

### Protocol notes
- The schedule weekday bitmask is **bit 0 = Monday** through bit 6 = Sunday,
  determined on hardware. This differs from the weekday numbering the vendor
  app uses elsewhere, so it cannot safely be inferred — anyone implementing
  against this frame should measure it rather than assume.
- Two of the four trailing bytes on a schedule entry are now identified: byte 1
  is the vacuum power and byte 2 the mop intensity. Bytes 0 and 3 remain
  unknown, so the field is still exposed as raw hex as well.

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
  goes out. That same datapoint also carries destructive commands — erasing the
  saved map, starting a job — so an unrecognised frame is never sent.

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
