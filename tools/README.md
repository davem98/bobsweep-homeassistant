# tools/

Helper scripts that run outside Home Assistant. They need only Python 3.12+
plus what each one lists; nothing here is loaded by the integration.

## map_zones.py -- room zones from a screenshot of the app's map

The integration's `current_room` sensor needs each room as a polygon in the
robot's own map-cell coordinates, and the robot never sends its map. What it
does send is its **no-go rectangles**, in map cells, and the vendor app draws
those same rectangles on top of the room-coloured map. So a screenshot of the
app's Map screen plus the no-go rectangles is enough to work out how screen
pixels map to cells, and from that, every room's outline.

You need at least two no-go zones drawn in the app (three or more gives a much
better fit; spread them across the map). They can be temporary: draw them,
take the screenshot, read the rectangles out of Home Assistant, then delete
them again if you like.

### Requirements

```
pip install pillow numpy
```

### Usage

```
python tools/map_zones.py --screenshot map.png --no-go no_go.json \
    --names Kitchen=teal,Hallway=grey --out zones.json --overlay overlay.png
```

| Flag | Meaning |
| --- | --- |
| `--screenshot PATH` | A screenshot of the app's Map screen. A full-phone screenshot is fine; the header, toolbars and background are ignored. |
| `--no-go PATH-or-JSON` | The robot's no-go rectangles in map cells: a JSON list of rectangles, each four `[x, y]` corners. A file path, or the JSON pasted inline. |
| `--names NAME=COLOUR,...` | Rename zones by the colour word the tool gave them, e.g. `Kitchen=peach,Hallway=pink`. Run once without it to see the words. |
| `--room-ids NAME=ID,...` | Attach the robot's own room ids to zones by (final) name, e.g. `Kitchen=3`. Optional. |
| `--out PATH` | Where to write the zone document (default `zones.json`). |
| `--overlay PATH` | Where to write the check image (default `overlay.png`). |
| `--min-area-px N` | Ignore room fragments smaller than N pixels (default 0.02% of the image). |
| `--dark-mode` / `--light-mode` | Override the theme auto-detection. |
| `--dump-detected` | Also print the red rectangles found on screen, in pixels, for debugging. |

The tool prints the fitted transform, the residual of every matched rectangle
in cells, and the zone list with vertex counts and areas. It exits non-zero
with a plain reason if it cannot find red rectangles, cannot match at least
two of them, or finds no rooms. A warning is printed if fewer than three
rectangles matched or any rectangle edge is more than 40 cells off; residuals
of 10-30 cells are normal (the app draws rounded, bordered rectangles, and at
phone resolution one pixel is about three cells).

**Always look at the overlay** before importing: room outlines in black,
the robot's rectangles re-projected in yellow (they should sit on the red
hatches), no-mop frames in blue, and the dock in magenta.

### Getting the no-go rectangles from Home Assistant

Open **Developer tools > States**, find `sensor.<name>_current_room`, and copy
the `no_go_zones` attribute. It is a list of rectangles, each four `[x, y]`
corners in map cells. Save it as a file (`no_go.json`) or paste it straight
into `--no-go`:

```
python tools/map_zones.py --screenshot map.png \
    --no-go '[[[100,-200],[400,-200],[400,-700],[100,-700]], ...]'
```

Take the screenshot with the same map loaded that the rectangles belong to,
with the no-go zones visible.

### Importing the result

Call the `bobsweep.import_zones` service with the file's contents as `data`
(Developer tools > Actions, YAML mode):

```yaml
action: bobsweep.import_zones
target:
  entity_id: vacuum.<name>
data:
  replace: true
  data: <paste the contents of zones.json here>
```

Then check `sensor.<name>_current_room`: its `zone_count` attribute should
equal the number of zones in the file. `bobsweep.export_zones` returns the
stored document if you want to keep a copy or hand-edit names.

### How it works (and what can go wrong)

1. The background is the most common colour; full-width uniform bars and
   anything without room colours are dropped, and the map is what remains.
2. The translucent red hatch of each no-go rectangle is found and boxed.
3. Each on-screen box is paired with each robot rectangle of similar aspect
   ratio; the pairing most other rectangles agree with wins, and a
   least-squares fit over the matched corners gives the scale and offset.
   The app draws the map rotated 180 degrees (map +x is screen-left, +y is
   screen-down); the tool tries every axis orientation and warns if the best
   one is not that.
4. Pixels are snapped to the app's flat room colours, with the hatch overlay
   un-blended underneath the rectangles. Speckles, icons and hatch remnants
   are filled from the surrounding room; enclosed fragments are absorbed.
5. Each room's contour is traced, simplified to 12-40 vertices and written
   as integer cells. The dock icon, if found, becomes `dock_ref`.

The room colours it knows are the ones the app has been seen to use. A
colour it does not know is still picked up when it covers real area and is
named by a generic colour word, but check the overlay. Rooms the app draws in
the same colour come out as `Teal`, `Teal 2`, ... and can be renamed with
`--names`. Dark mode is what this was developed against; light mode is
handled by the same rules but has had less testing.
