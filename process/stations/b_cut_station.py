# b_cut_station.py
"""Station B: Cut beam to length with circular saw.

Each beam receives 2 cuts (cut_position_a and cut_position_b) from the
Grasshopper-exported fab_data. The saw is a fixed circular saw — the robot
moves the beam through the blade.

Only 1D miter cuts (Gehrungsschnitte) are supported on the facade project;
no Schifterschnitte (compound miter cuts).

The saw stays ON between cut A and cut B intentionally (faster cycle time,
blade is already spinning).

Motion sequence (Layer 0 & 1, horizontal beams only):
    - Robot arrives from pick station holding the beam
    - Track moves from pick position to cut position (EXTAX_CUT=500mm)
    - X-compensation is applied: when the track moves, the robot must
      compensate in X to keep the beam at the same world position
    - Two cuts are executed with approach -> cut -> retract sequences
    - Between cuts, the robot rotates to the second cut orientation
    - Optional SimBeam swap hooks fire when the robot reaches each cut frame

RAPID instructions used:
    - r_HSLU_SawOn / r_HSLU_SawOff: Control saw motor via digital outputs
"""

# ==============================
# Imports
# ==============================
import sys
import os
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import compas_rrc as rrc
from compas.geometry import Frame, Point

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.SimBeam import sim_swap_cut_a, sim_swap_cut_b
from _skills.WoodStorage.wood_storage import WoodStorage

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_WITH_MEMBER, SPEED_APPROACH, SPEED_CUT,
    W_OBJ_CUT,
)
from joint_positions import jp_cut

# Storage instance for looking up track position by beam size
storage = WoodStorage()

# Duration in seconds for coordinated (track + robot) moves
COORD_MOVE_TIME = 1

# Fixed track position for the cut station (mm)
EXTAX_CUT = 500


def _get_rotation_frame(current_point, cut_frame):
    """Create a rotation frame: keeps the robot's current XYZ position but
    adopts the orientation (xaxis/yaxis) of the target cut frame.

    Used for the first cut (A) where the robot needs to rotate the beam
    into the cut orientation while staying at the track-compensated position.
    """
    return Frame(
        current_point,
        cut_frame.xaxis,
        cut_frame.yaxis
    )


def _get_original_rotation_frame(cut_frame):
    """Create a safe rotation frame offset from the cut position.

    Used for the second cut (B) where the robot has already released
    from the first rotation frame and needs a new safe position to
    rotate into the second cut orientation.

    Offset: X-100, Y-150, Z+300 from cut_frame (above and behind the saw).
    """
    return Frame(
        Point(
            cut_frame.point.x - 100,
            cut_frame.point.y - 150,
            cut_frame.point.z + 300
        ),
        cut_frame.xaxis,
        cut_frame.yaxis
    )


def _do_cut_sequence(r1, cut_frame, rotation_point=None, *, dry_run=False,
                     saw_on=False, saw_off=False, skip_initial_move=False,
                     use_original_rotation=False, on_arrived=None):
    """Run the approach + cut + retract sequence for one cut frame.

    Args:
        cut_frame: Target cut position frame.
        rotation_point: Point for rotation frame (used if use_original_rotation=False).
        saw_on: If True, turn saw ON before cutting.
        saw_off: If True, turn saw OFF after cutting.
        skip_initial_move: If True, skip move to rotation_frame (already there).
        use_original_rotation: If True, use original offset-based rotation frame.
        on_arrived: Callable invoked the moment the robot reaches cut_frame
                    (used for SimBeam swap hooks).
    """
    if dry_run:
        print(f"  [CUT-SEQ] frame: X={cut_frame.point.x:.1f} Y={cut_frame.point.y:.1f} Z={cut_frame.point.z:.1f} | saw_on={saw_on} | saw_off={saw_off}")
        return

    # --- CUT SEQUENCE ---

    # Rotation frame: either from rotation_point or original offset calculation
    if use_original_rotation:
        rotation_frame = _get_original_rotation_frame(cut_frame)
    else:
        rotation_frame = _get_rotation_frame(rotation_point, cut_frame)

    # Move to rotation frame (skip if already there from coordinated move)
    if not skip_initial_move:
        r1.send(rrc.MoveToFrame(rotation_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.JOINT))

    # Create an offset approach point (80mm above cut)
    cut_approach = cut_frame.copy()
    cut_approach.point.z += 80
    r1.send(rrc.MoveToFrame(cut_approach, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    # --- ACTUAL CUTTING ---

    # Turn saw on (only if saw_on=True)
    if saw_on:
        r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SawOn", [], []))

    r1.send(rrc.MoveToFrame(cut_frame, SPEED_CUT, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Trigger swap/hook the moment the robot arrives at cut_frame
    if on_arrived is not None:
        on_arrived()

    # Retract slightly in -X
    cut_retract = cut_frame.copy()
    cut_retract.point.x -= 30
    r1.send(rrc.MoveToFrame(cut_retract, SPEED_CUT, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Turn saw off (only if saw_off=True)
    if saw_off:
        r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SawOff", [], []))

    # Retract to rotation frame
    r1.send(rrc.MoveToFrame(rotation_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.LINEAR))


def b_cut_station(r1, data, i, *, layer_idx=0, dry_run=False, saw_enabled=True, sim_beams=False):
    """Cut a beam at 2 positions (A and B).

    Both cuts are always executed (both ends of the beam). Compensates the X
    position for the track movement from the pick station to the cut station.

    Args:
        r1: AbbClient instance (or None for dry_run)
        data: Loaded fab_data dict
        i: Element index within the layer
        layer_idx: Layer index (0 or 1)
        dry_run: If True, prints planned moves without robot connection
        saw_enabled: If True, activates the saw motor via RAPID instruction.
                     If False, robot moves through cut path without cutting
                     (for position testing).
        sim_beams: If True, fires SimBeam swap hooks on cut_a/cut_b arrivals
                   (virtual controller only).
    """
    element = get_element(data, i, layer_idx=layer_idx)
    cut_a_frame = element["cut_position_a"]
    cut_b_frame = element["cut_position_b"]

    # Get beam size and track position from pick station. Missing/unknown
    # values surface as a "Unknown category" ValueError from storage.get_extax().
    stock_category = element.get("stock_category", "").strip('"').strip("'")
    pick_extax = storage.get_extax(stock_category)

    # When the track moves from pick position to cut position, the beam's
    # world X coordinate shifts by the same amount. We need to compensate
    # the robot's X position so the beam stays in the same place relative
    # to the saw work object.
    # Example: pick at extax=1000, cut at extax=500 -> delta=500mm
    track_delta = pick_extax - EXTAX_CUT

    if dry_run:
        print(f"[CUT] i={i} layer={layer_idx} | saw={saw_enabled}")
        print(f"  stock_category: {stock_category} | pick_extax: {pick_extax} | delta: {track_delta}")
        _do_cut_sequence(None, cut_a_frame, dry_run=True, saw_on=saw_enabled, saw_off=False)
        _do_cut_sequence(None, cut_b_frame, dry_run=True, saw_on=False, saw_off=saw_enabled)
        return

    # === TRANSITION: Switch to cut wobj and read current position ===
    r1.send(rrc.SetWorkObject(W_OBJ_CUT))

    # Read current frame in cut wobj coordinates (robot is at pick exit position)
    current_frame = r1.send_and_wait(rrc.GetFrame())
    print(f"Current position in cut wobj: {current_frame.point}")

    # Adjust X for track movement: when track moves from pick_extax to EXTAX_CUT,
    # the robot needs to compensate in X direction
    rotation_point = Point(
        current_frame.point.x - track_delta,
        current_frame.point.y,
        current_frame.point.z
    )
    print(f"Adjusted rotation point (delta={track_delta}): {rotation_point}")

    # === CUT A: Coordinated move with orientation change ===
    # Create start frame: adjusted position + cut_a orientation
    start_frame_a = Frame(rotation_point, cut_a_frame.xaxis, cut_a_frame.yaxis)

    r1.send_and_wait(cm.MoveToRobtarget(
        frame=start_frame_a,
        ext_axes=[EXTAX_CUT],
        time=COORD_MOVE_TIME,
        zone=rrc.Zone.Z1,
        motion_type=rrc.Motion.LINEAR
    ))
    print("At cut station (coordinated move).")

    # Swap hooks: fire the moment the robot arrives at the cut frame
    on_arrived_a = (lambda: sim_swap_cut_a(r1, dry_run=dry_run)) if sim_beams else None
    on_arrived_b = (lambda: sim_swap_cut_b(r1, dry_run=dry_run)) if sim_beams else None

    # Run cut A sequence (saw ON at start, stays on)
    _do_cut_sequence(r1, cut_a_frame, rotation_point,
                     saw_on=saw_enabled, saw_off=False, skip_initial_move=True,
                     on_arrived=on_arrived_a)

    # === CUT B: Use original rotation frame (saw stays on, OFF at end) ===
    _do_cut_sequence(r1, cut_b_frame,
                     saw_on=False, saw_off=saw_enabled,
                     skip_initial_move=False, use_original_rotation=True,
                     on_arrived=on_arrived_b)

    # Leave station
    r1.send_and_wait(rrc.MoveToJoints(jp_cut.robax, [], SPEED_WITH_MEMBER, rrc.Zone.Z30))
    print("Left cut station.")


if __name__ == "__main__":
    DATA = load_data()

    ros = rrc.RosClient()
    ros.run()

    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    r1.send(rrc.SetTool(TOOL_GRIPPER))

    b_cut_station(r1, DATA, i=0, layer_idx=0, dry_run=False, saw_enabled=False)

    print("Finished")
    ros.close()
    ros.terminate()
