# Working on this repository with an AI agent

This file is for coding/assistant agents (Claude Code, Codex, Cursor, …)
that a user points at this repository. Humans: see `README.md`.

## What this is

A Home Assistant custom integration (`custom_components/bobsweep/`) that
controls bObsweep robot vacuums over the Tuya LAN protocol, with no cloud.
Logic that does not need Home Assistant lives in HA-free modules
(`transport.py`, `position.py`, `zones.py`, `geometry.py`, `stuck.py`,
`insights.py`, `faults.py`) so it can be exercised offline.

## Helping a user set it up

The setup that cannot be automatic — room ids, room geometry, voice — is
written as a step-by-step playbook in `docs/ROOMS.md`, and packaged as an
agent skill in `skills/bobsweep-rooms/SKILL.md` (copy or symlink that folder
into `.claude/skills/` to have Claude Code load it automatically). Follow that
playbook rather than improvising: it encodes what was measured on real
hardware (which frames exist, which do not, what the app's map screenshot
can and cannot yield).

Useful tool: `tools/map_zones.py` turns a screenshot of the vendor app's map
into a zone document, calibrated on the robot's own no-go rectangles.

## Rules that matter

- **DP 105 is a transparent command channel.** The only frames the
  integration writes to it are the vendor app's read-only getters
  (`transport.GETTER_FRAMES`) and the one room-clean command
  (`encode_room_clean_frame`). Never add a DP 105 write without a
  byte-for-byte source in the vendor app; the same datapoint carries map
  delete and room merge.
- Every wire-format claim in docstrings says whether it was measured on
  hardware or inferred. Keep that distinction when editing.
- Do not put a user's real room names, map coordinates, IP addresses,
  device ids or keys into code, docs, tests or commit messages. Use
  invented examples.
- Offline checks live outside the public tree (the maintainer's
  `research/verify_*.py`); at minimum run `python -m py_compile` on every
  touched module.
