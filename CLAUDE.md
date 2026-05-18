# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
Robotic facade element production for 9 student groups. ABB Gofa CRB 15000 + Güdel Track.
Pipeline: Grasshopper (Design) → JSON Export → Python + compas_rrc → OmniCore Controller.
RAPID project name on the controller: `Facade FS26` (also referenced as `PROJECT_NAME` in `globals.py`).

Based on the Swissbau26 project (`C:\Users\jurij\Documents\GitHub_HSLU\hslu_rrc_Swissbau26`), simplified for student use.

## Architecture

```
STUDENT_INPUT.md             # Detailed student input specification with images
README.md                    # Student-facing overview

design/
  hslu_rrc_tes-mini.ghx      # GH template: export, validation, IK visualization
  hslu_rrc_tes-mini.3dm      # Rhino file (gitignored, too large)
  gh_python/
    ExportFacade.py          # GH component: writes fab_data JSON + 3 STLs per element
    holzbedarf.py            # GH component: count elements per stock_category + total laufmeter

docs/
  images/                    # Documentation images

docker/
  REAL-docker-compose.yml    # ROS + ABB driver (real controller)
  VIRTUAL-docker-compose.yml # ROS + ABB driver (virtual controller)

robotstudio/
  BeamSimulator/             # RS 2025 SmartComponent: dynamic beam visualization
    BeamSimulator.csproj     # SDK-style C#/.NET Framework 4.8 project
    BeamSimulator.xml        # LibraryCompiler descriptor (signals + properties)
    BeamSimulatorCodeBehind.cs  # SmartComponent logic (Activate/Swap/Release/Reset)
    README.md                # Build + install + signal wiring

process/                     # (Work in Progress)
  production.py              # Main loop: Pick → Cut → Glue → Place
  globals.py                 # Speeds, tools, workobjects, frame dimensions
  joint_positions.py         # Taught joint targets per station

  _skills/                   # Low-level robot capabilities (DO NOT MODIFY for students)
    custom_motion.py         # Güdel track coordinated motion (MoveToJoints, MoveToRobtarget)
    fabdata.py               # JSON data loading (compas.json_load), v3 layer structure
    gripper.py               # Open/close via RAPID custom instructions
    CSS/                     # Cartesian Soft Servo: RAPID system module (RRC_CI_Rob.sys)
    GlueLine/                # All-in-one glue line execution in RAPID (avoids Python latency)
    GluePLC/                 # PLC safety handshake for glue system
    SimBeam/                 # Virtual-only: drives BeamSimulator SmartComponent via EIO
    SoftAct/                 # Compliant servo for soft gripping/pressing
    WoodStorage/             # Inventory mgmt (4 length categories: 400/550/750/1000, round-robin pick)

  stations/                  # Station implementations (DO NOT MODIFY for students)
    a_pick_station.py        # CSS grip from dynamic storage
    b_cut_station.py         # Dual 1D miter cuts (no Schifterschnitte)
    d_glue_station.py        # POS/NEG auto-dispatch, predefined glue path
    e_place_station.py       # Dynamic track offset, approach computed from place_position

  scripts/                   # Standalone helpers — run individually, not orchestrated
    test_connection.py       # Smoke test: ROS bridge + AbbClient handshake
    go_to_park.py            # Drive robot to jp_park (safe pose for refill access)
    get_frame.py             # Read current TCP frame (teaching aid for joint_positions.py)
    test_pick_positions.py   # Sanity-check wood_storage compartment frames
    wobj_test_ob_hslu_*.py   # Workobject calibration probes (cut, place)

  data/
    fab_data.json            # Student export (from GH)
    wood_storage.json        # Inventory state (persisted between runs, reloaded each pick)
    geometry/                # Runtime STL dump for BeamSimulator (gitignored)
```

## Student Input (GH)
Students provide per element in `ob_HSLU_Place` world coordinates:

| Index | Name | Type | Description |
|-------|------|------|-------------|
| 0 | Brep | Brep | Finished beam geometry (25x25mm, with miter cuts) |
| 1 | Centerline | Line | Center axis of the finished beam |
| 2 | Cut Plane A | Plane | Cut plane end A (Z outward, Y world-up) |
| 3 | Cut Plane B | Plane | Cut plane end B (Z outward, Y world-up) |

Glue planes are passed via a **separate** DataTree input on the export
component (`glue_planes_tree`) with the same `{layer;element}` path. Each
branch holds 0..N planes; list order = robot drive order.

GH template automatically computes: stock_category, place_position, robot frames in station workobjects.

Frame bounds: X = 0..2500mm, Y = -600..0mm. Origin = top-left of frame.

## Key Differences from Swissbau26
- No label station (c_lable_station removed)
- No Layer 2 / diagonal code (all diagonal logic removed from every station)
- 4 beam categories keyed by stock length (mm): "400", "550", "750", "1000"
- 25x25mm beams (not 40x40mm) → stack_offset_z = 25, grip load = 0.3kg
- Place station computes approach frames from place_position only (students don't provide pre-app, app, rot frames)
- Glue: student provides 0..N planes via separate DataTree, robot drives predefined path pattern at each
- JSON has 5 fields per element + variable-length glue_positions list
- Students provide raw geometry (Brep, Centerline, Planes), GH transforms to robot frames

## Data Format (tes-v4 SSOT)
Six top-level sections, written by `design/gh_python/ExportFacade.py`, read by
`process/_skills/fabdata.py` (fabrication) and `process/_skills/state.py` (state).

```jsonc
{
  "manifest":      { "manifest_id", "created_at", "pipeline_version", "schema_version": "tes-v4" },
  "configuration": { "project_name", "frame_size_mm", "beam_section_mm", "structural", "grid", "materials", "stock" },
  "design": {
    "layers": [{
      "id": 0,
      "elements": [{
        "id": 0,
        "stock_category": "400|550|750|1000",
        "centerline": Line,
        "finished_length_mm": 750.0
      }]
    }]
  },
  "process": {
    "cut_planes_world":  [{ "element_ref": "L0_E0", "plane_a": Frame, "plane_b": Frame }],
    "glue_planes_world": [{ "element_ref": "L0_E0", "planes": [Frame, ...] }]
  },
  "fabrication": {
    "target_cell": "hslu_rrc_tes-mini",
    "cell_config_ref": null,
    "layers": [{
      "id": 0,
      "elements": [{
        "id": 0,
        "stock_category": "400|550|750|1000",
        "place_position": Frame,
        "cut_position_a": Frame,
        "cut_position_b": Frame,
        "glue_positions": [Frame, ...]   // 0..N, [] = skip glue station
      }]
    }]
  },
  "state": {
    "run_id", "started_at", "last_updated_at",
    "elements": {
      "L0_E0": {
        "pick":  { "status": "pending|in_progress|done|failed", "at", "error" },
        "cut":   { ... },
        "glue":  { ... },
        "place": { ... }
      }
    },
    "errors": []
  }
}
```

**Schichten:** `design` = fertigungsneutral (Centerlines), `process` = anlagenneutral
(Welt-KS Cut/Glue Planes), `fabrication` = anlagenspezifisch (Workobject-Frames).
Anlagenwechsel HSLU→TUM regeneriert nur die `fabrication`-Section.

**State:** atomic save in dieselbe fab_data.json (tmp + os.replace). Resume-Prompt
am Start von production.py, Skip-If-Done pro Station.

**Frame-Format:** COMPAS `compas.geometry/Frame` mit point/xaxis/yaxis (kompakt als
1-Zeiler geschrieben via custom encoder, ~5400 Zeilen statt ~42000 bei 106 Elementen).

## compas_rrc Connection Pattern
```python
import compas_rrc as rrc
ros = rrc.RosClient()
ros.run()
r1 = rrc.AbbClient(ros, "/rob1")
r1.send(rrc.SetTool('t_HSLU_GripperZimmer'))
# ... robot commands ...
ros.close()
ros.terminate()
```

## Custom RAPID Instructions
| Instruction | Purpose |
|---|---|
| `r_Gudel_HSLU_MoveToJoints` | Coordinated 6-axis + track motion |
| `r_Gudel_HSLU_MoveTo` | Cartesian coordinated motion |
| `r_HSLU_GripperOpen/Close` | Pneumatic gripper |
| `r_HSLU_SawOn/Off` | Cutting station |
| `r_HSLU_GlueOn/Off` | Glue system PLC handshake |
| `r_RRC_CI_CSS` | Compliant servo (Define/On/Off) |
| `r_RRC_CI_GripLoad` | Workpiece load definition |
| `r_RRC_CI_GlueLine` | All-in-one glue line in RAPID |
| `r_RRC_CI_SoftAct/Deact` | Soft servo axis |
| `r_HSLU_SimBeam*` | Virtual-only: drive BeamSimulator SmartComponent (Activate/SwapCutA/SwapCutB/Release/Reset) |

## Workobjects
- `ob_HSLU_Pick_400` / `_550` / `_750` / `_1000` — Pick station, one wobj per stock length (defined in wood_storage.json)
- `ob_HSLU_Cut` — Cut station
- `ob_HSLU_Glue` — Glue station
- `ob_HSLU_Place` — Place station (= World coordinate system, OFFSET_TRACK = 593mm)

## Development Commands
```bash
# Docker (use REAL or VIRTUAL compose file)
cd docker && docker compose -f REAL-docker-compose.yml up -d
cd docker && docker compose -f REAL-docker-compose.yml down

# Single helper script (run from process/, not from scripts/)
cd process && python scripts/test_connection.py
cd process && python scripts/go_to_park.py

# Dry run (no robot motion) — edit production.py: main(dry_run=True)
# Production
cd process && python production.py
```

## Critical Safety Flag: `SIM_FAST` (production.py:52)
- `SIM_FAST = True` (current default) multiplies all TCP speeds by 4 and patches
  `globals.SPEED_*` **before** the station imports — required for usable RobotStudio
  simulation speed.
- **Must be set to `False` before any run on the real cell.** The boost also raises
  `MAX_TCP` and re-issues `rrc.SetMaxSpeed` to lift the controller cap, so the real
  robot would actually move at the boosted velocity if left enabled.
- Other runtime toggles in the same block: `DO_PICK/CUT/GLUE/PLACE` (skip a station
  but still run the transit motion), `CSS_ENABLED/SAW_ENABLED/GLUE_VALVE_ENABLED`
  (move without activating the tool — useful for path verification), `SIM_BEAMS`
  (drives the BeamSimulator SmartComponent; no-op on the real controller).

## Production Runtime (production.py)
Interactive flow (no CLI args):
1. Layer prompt (skipped if data has only one layer); element-range prompt accepts
   `Enter` (all), `5-10` (range incl.), or `12` (single).
2. `check_wood_storage()` shows demand vs capacity table, single Enter confirms
   "lager filled to recommendation", `n` opens per-category override.
3. Connect ROS, set tool, drive to `jp_home`, optional `sim_beam_reset`.
4. Loop: PICK → CUT → GLUE → PLACE per element. Before each PICK, `WoodStorage`
   is reloaded from disk; if the compartment is empty, `refill_lager()` parks the
   robot at `jp_park` and prompts the operator before retrying. Production does not
   abort on empty stock — it pauses.
5. Drive back to `jp_home`, close ROS.

Data integrity is enforced by the GH-side validate component (visual feedback in
the template) — there is no Python pre-flight validation step in `process/`.

## Validation
The only pre-flight validation lives in the **GH Template** (validate component):
visual feedback (green/orange/red) for missing data, bounds, cut angles. IK
visualization shows reachability. There is no Python-side validation —
`production.py` trusts the data exported by Grasshopper.

## Student Constraints
- Frame: X = 0..2500mm, Y = -600..0mm
- Max 2 layers
- Only 1D miter cuts (Gehrungsschnitte), no Schifterschnitte
- stock_category: automatically determined from centerline length
- 0..N glue planes per element (empty list = no gluing)
- Students must NOT modify `_skills/` or `stations/`

## Known TODOs
- Joint positions need teaching at the machine (search for `TODO: verify`)
- Wood storage base_frames + extax for the 4 new compartments (400/550/750/1000) need to be measured at the machine and entered in wood_storage.json
- Place station dynamic offset may need recalibration for facade frame
- process/ code is WIP — stations need testing on real hardware
