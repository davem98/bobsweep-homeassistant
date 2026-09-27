# Rooms, maps and voice — the setup guide

This is the part of the integration that cannot be automatic, written down
so you (or an assistant working for you) can do it in one sitting. Read it
after the integration is installed and the vacuum entity exists.

Where the robot is, what its rooms are called, and where its rooms *are* on
the map are three different questions, and the robot answers only the middle
one, indirectly. This guide gets you all three.

## 0. Name the robot what you say out loud

The config flow has a **Name** field. Put the robot's name there — the one
you use when you talk about it ("Rosie", "Rosie", "the vacuum") — because
Home Assistant's voice intents match on the entity's name. An existing
entry can be renamed: Settings → Devices & services → the bObsweep device →
pencil icon → rename, and accept the offer to rename the entity ids too.

## 1. Learn the room ids

The robot numbers its rooms (0, 1, 2 …) and never sends a name table. Two
sources give you the numbers:

- **Schedules.** If you have created cleaning schedules in the vendor app,
  `sensor.<name>_schedules` lists each one with its target room ids. A
  schedule that targets one room, named after that room, names it: the
  integration derives those automatically (see `room_names` on the sensor).
- **Cleaning summaries.** The app's Cleaning History shows, for each run,
  where the robot ended and the path it took. Cross-reference a run against
  the schedule that started it (same time of day) and you know which colour
  region the room ids in that schedule are. Two runs usually settle
  everything.

Then name what is still unnamed:

```yaml
action: bobsweep.set_room_name
target: { entity_id: vacuum.rosie }
data: { room_id: 2, name: "Kitchen" }
```

Use the words you would say. `sensor.<name>_selected_rooms` shows the
effective id → name map. From here on `bobsweep.clean_rooms` works by name:

```yaml
action: bobsweep.clean_rooms
target: { entity_id: vacuum.rosie }
data: { rooms: "Kitchen, Hallway" }
```

## 2. Turn the app's map into zones (room geometry)

The robot streams the floor map only to the vendor app, over the cloud. But
the app draws it, and the robot *does* state one piece of geometry in its
own coordinates: the **no-go rectangles**, which the app draws as hatched
boxes on that same map. Those boxes are enough to calibrate a screenshot.

1. In the app, draw at least **three no-go zones** if you have none (small
   ones in corners you never want cleaned are fine; you can delete them
   afterwards). Spread them across the map — the further apart, the better
   the fit.
2. Take a screenshot of the app's **Map** screen. Any phone size works.
3. Copy the rectangles the robot reports: the `no_go_zones` attribute of
   `sensor.<name>_current_room` (Developer tools → States). They are read
   at startup; `bobsweep.refresh_robot_info` re-reads them.
4. Run the tool (needs `pip install pillow numpy`, no Home Assistant needed):

   ```
   python tools/map_zones.py --screenshot map.png --no-go no_go.json \
       --names "Kitchen=peach,Hallway=pink,Office=pale green" \
       --room-ids "Kitchen=2,Hallway=3,Office=9" \
       --out zones.json --overlay overlay.png
   ```

   Check `overlay.png`: every room outlined, the re-projected rectangles
   sitting on the hatched boxes. The printed residuals should be a few tens
   of map cells at most.
5. Import: `bobsweep.import_zones` with `data:` set to the file's contents
   and `replace: true`. `sensor.<name>_current_room` reports `zone_count`.

From then on every position the integration gets — path-trail points while
the app's map is open, obstacle sightings any time — becomes a room name:
`current_room`, obstacle records, and the *where* in a stuck alert.

Turn on **Estimate position from obstacle detections** (the integration's
Configure dialog) so sightings are used for the room; it is off by default
because it is approximate, and the zones are what make it useful.

## 3. Voice: "clean the kitchen"

Home Assistant 2026.3+ exposes each named room as a vacuum **segment**.

1. Open the vacuum entity's settings (Settings → Devices & services →
   Entities → `vacuum.<name>` → ⚙) and map each segment onto a Home
   Assistant **area**. Create areas for rooms that have none; add aliases
   for the other words you use ("den", "study").
2. Settings → Voice assistants → **Expose** the vacuum entity. An
   unexposed vacuum silently ignores every voice command.
3. Say it. The built-in `HassVacuumCleanArea` intent already understands
   "clean the kitchen" / "vacuum the office". With the robot named Rosie,
   "Rosie, clean the kitchen" resolves the entity by name.

For the more natural "tell Rosie to go clean up the family room", drop
`custom_sentences/en/bobsweep.yaml` from this repository into your Home
Assistant `config/custom_sentences/en/` folder and reload — it adds those
phrasings to the same intent. If your assistant is an LLM-backed
conversation agent, it needs neither: it reads the exposed entity's name and
the areas, and phrases itself.

## 4. What the integration still cannot do

- Give a live position with the app closed. The trail feed needs a cloud map
  session; the obstacle-sighting estimate is the honest fallback.
- Tell you a room's *name* — only its number, until you name it.
- Know a room the robot has never mentioned (no schedule, no acknowledged
  clean, no name). `set_room_name` with its id adds it.
