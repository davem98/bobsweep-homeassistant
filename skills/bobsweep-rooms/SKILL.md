---
name: bobsweep-rooms
description: Set up rooms, map zones and voice control for the bObsweep Home Assistant integration — learn the robot's room ids, name them, calibrate a screenshot of the vendor app's map into zones, map segments to areas, and make "clean the kitchen" work. Use when a user asks to configure rooms, "which room is which", room mapping, zones, current-room reporting, or voice/Assist control for their bObsweep vacuum.
---

# bObsweep rooms, zones and voice

You are helping a user finish the part of the bObsweep integration setup
that needs a human in the loop. The full playbook is `docs/ROOMS.md` in this
repository; this skill tells you how to drive it. Work with the user's Home
Assistant through whatever access you have (REST API with a long-lived
token, the UI via the user, or `ha` over SSH). Never contact the robot
directly.

## What you can rely on (measured on hardware)

- The robot numbers rooms; it never sends names. `sensor.<name>_schedules`
  carries each schedule's room ids; single-room schedules named after a
  room name that room automatically.
- The robot acknowledges room cleans it is *commanded* to do (via
  `bobsweep.clean_rooms`) but sends no acknowledgement for its own
  scheduled cleans.
- The robot reports its **no-go rectangles** in map cells
  (`no_go_zones` attribute on `sensor.<name>_current_room`). The vendor app
  draws the same rectangles as hatched boxes on its map. That is the
  calibration for turning a screenshot into zones.
- Position with the app closed is only available approximately, from
  obstacle sightings; exact position needs the app's map screen open.

## Procedure

1. **Name.** Confirm the vacuum entity is named what the user says aloud
   ("Rosie"). If not, have them rename the device (Settings → Devices →
   rename, accept entity-id rename).
2. **Room ids.** Read `sensor.<name>_schedules` and `room_names`. For ids
   still unnamed, ask the user for screenshots of the app's *Cleaning
   History* entries (each shows where the robot ended and its path) and
   match them to the schedule that started the run by time of day. Set
   names with `bobsweep.set_room_name` using the user's own words. Do not
   guess a name; ask.
3. **Zones.** Ask for a screenshot of the app's Map screen. If the robot
   reports fewer than three no-go rectangles, ask the user to add small
   ones spread across the map first (they can delete them later). Run
   `python tools/map_zones.py --screenshot ... --no-go ... --names ...
   --room-ids ... --out zones.json --overlay overlay.png`, show the
   overlay to the user, and only after they confirm the outlines look
   right call `bobsweep.import_zones` with `replace: true`. Verify
   `zone_count` on `sensor.<name>_current_room`.
4. **Position.** Have the user enable *Estimate position from obstacle
   detections* in the integration's Configure dialog.
5. **Voice.** Have the user map segments to areas in the vacuum entity's
   settings and expose the entity under Settings → Voice assistants
   (registry writes need an admin session; a non-admin token cannot do
   this). For "tell Rosie to clean the …" phrasings, and for stop / pause /
   resume / "send Rosie home" / "what is Rosie doing", install
   `custom_sentences/en/bobsweep.yaml` into their config and reload. The
   stop, pause and status intents are registered by the integration, so
   they need it loaded (restart after installing or updating it). An
   LLM-backed agent needs no sentence file, only the exposed entity and an
   alias for the spoken name.
6. **Test** with `bobsweep.clean_rooms` on one room, then `vacuum.stop`
   and `vacuum.return_to_base`. Warn the user before starting the robot.

## Privacy

The user's room names, map screenshot, zone coordinates and addresses are
theirs. Keep them in their Home Assistant and their own files; never write
them into this repository, an issue, or a commit.
