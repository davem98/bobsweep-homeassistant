# RUNBOOK — get a bObsweep vacuum into Home Assistant

This is deliberately not a step-by-step guide to extracting the local key —
it orients a capable person on what's needed and why it's hard, not a recipe
to copy-paste.

## Result

Local HA control via the `bobsweep` custom integration + `tinytuya`, using three
values: the vacuum's LAN **IP** (host), its Tuya **device_id**, and its
**local_key**. The key is static per pairing — extract it once, enter it in
HA, done.

## Why it's not a one-liner

- bObsweep is Tuya white-label (a "Thing" SDK, React Native app).
- Login is via **bObsweep's own vendor backend**, not Tuya directly — so the
  usual Tuya-cloud key-pullers can't authenticate.
- The `local_key` at rest is in **Android-Keystore-encrypted storage** and
  can't be decrypted offline.

So the key has to come out of a **logged-in app instance's live memory**: run
the vendor app in a rooted Android environment, and hook the Tuya SDK's
device object in memory once the app has loaded the paired robot. This
generally requires an ARM-capable rooted Android environment able to run the
app's native libraries, and a dynamic instrumentation tool that can attach to
the running process and read its object graph.

## Finding + pinning the LAN IP

- The robot answers the Tuya LAN protocol on its usual port; scanning the LAN
  for that (or power-cycling the vacuum to see which host drops) will locate
  it.
- Give it a **DHCP reservation** so the IP doesn't move once you've paired it
  to HA.

## Configure HA

- Copy `custom_components/bobsweep/` into HA `config/custom_components/`, restart.
- Settings → Devices & Services → Add → **bObsweep (local)**:
  host, device_id, local_key, protocol (**3.4** for the UltraVision Pet Combo;
  try 3.3/3.5 if `cannot_connect`), and **model family**.
- **Picking the model family**: bObsweep uses three incompatible Tuya datapoint
  tables and the app switches between them by model, so the wrong pick leaves
  entities blank and commands ignored. Choose by your robot's model name:
  - **slam** (LiDAR/SLAM) — bObsweep SLAM, Austin, Dustin / Dustin Plus / Dustin
    Combo, Appetite, Phoenix, Archer, Orbi, Bio, Maxim, UltraVision /
    UltraVision Pet / UltraVision Pet Combo. **(The reference unit is an UltraVision Pet Combo →
    `slam`.)**
  - **vision** — Bob PetHair Vision, Bob PetHair Vision Plus.
  - **random** — bObsweep Leaf, Charlotte.

  The model → family table above was transcribed verbatim from the vendor app's
  JavaScript bundle; the full per-family DP maps behind it are not published in
  this repo. Pre-WiFi units
  (Bob PetHair, Bob PetHair Plus, Bob Standard, Bob Pro, bObi Pet, bObi Classic)
  have no Tuya datapoints at all and cannot be used with this integration.

  What the choice changes: **vision** has no brush/filter-life sensors, no
  locate, no self-empty and no mopping sensor, and uses `quiet/standard/strong`
  fan speeds instead of `gentle/normal/strong`; **random** has no clean-area
  sensor, no locate and no self-empty, but adds Water control and Dustbin /
  water tank sensors. Entities a family cannot report are not created at all
  rather than sitting permanently unknown, and features it cannot perform are
  not advertised on the vacuum entity.
- If you picked wrong, delete the config entry and re-add it — `model_family` is
  read at setup and selects the whole DP map. An entry saved before the selector
  existed (no `model_family` at all) falls back to `slam`.

## Verifying / debugging DPs

Query the device directly (from a LAN host, once you have the key):
```python
import tinytuya
d = tinytuya.Device(DEVICE_ID, IP, LOCAL_KEY, version=3.4)
print(d.status())   # shows the live datapoint map
```
The robot pushes **partial** updates over a persistent socket, so the coordinator
**merges** DPs (see `coordinator.py`) rather than replacing — otherwise a
battery-only push blanks every other sensor.
