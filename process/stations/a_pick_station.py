# a_pick_station.py
"""Station A: Pick beam from wood storage.

Retrieves a timber beam from the WoodStorage inventory system. The storage
has compartments for 4 stock lengths (400, 550, 750, 1000 mm), each at a
different track position. Beams are stacked vertically, so the pick frame
Z-offset depends on how many beams remain in the compartment.

Physical setup:
    - One wobj per category: ob_HSLU_Pick_400 / _550 / _750 / _1000
    - Track positions (extax) and base_frames are configured in
      process/data/wood_storage.json

Motion sequence:
    1. Open gripper
    2. Coordinated move (track + robot) to pre-approach position
    3. Linear approach: high (Z=200) -> low (Z+50) -> pick frame
    4. CSS (Cartesian Soft Servo) press 5mm down for secure grip on 25mm beams
    5. Close gripper, optionally activate SimBeam, define GripLoad
    6. Retract on offset path (+10mm X/Z to avoid scraping compartment edge)
    7. Exit to safe position

The retract path is offset by 10mm in X and Z from the approach path.
This prevents the beam from scraping against the storage compartment
edge during lift-out.
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
from compas.geometry import Frame

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.gripper import gripper_open, gripper_close
from _skills.SimBeam import sim_beam_activate
from _skills.WoodStorage.wood_storage import WoodStorage

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_NO_MEMBER, SPEED_WITH_MEMBER, SPEED_APPROACH, SPEED_PRECISE,
)
from joint_positions import jp_pick

# Duration in seconds for coordinated (track + robot) moves
COORD_MOVE_TIME = 1


def a_pick_station(r1, data, i, *, layer_idx=0, dry_run=False, css_enabled=True, sim_beams=False):
    """Pick a beam from wood storage.

    Determines the beam size from fab_data, looks up the pick frame
    from WoodStorage (which tracks inventory and returns the correct
    Z-offset for the top beam in the stack), then executes the
    approach-grip-retract sequence.

    Args:
        r1: AbbClient instance (or None for dry_run)
        data: Loaded fab_data dict (from fabdata.load_data())
        i: Element index within the layer
        layer_idx: Layer index (0 or 1)
        dry_run: If True, prints planned moves without robot connection
        css_enabled: If True, activates Cartesian Soft Servo (CSS) to
                     press 5mm into the beam for a secure grip. CSS makes
                     specific axes compliant so the gripper yields to the
                     beam surface instead of fighting it.
        sim_beams: If True, activates SimBeam geometry in RobotStudio
                   after the gripper closes (virtual controller only)
    """
    # Reload storage from JSON each time (important: counts may have
    # changed from refill_all() or a previous pick in the same run)
    storage = WoodStorage()

    # Get beam size from element. If the field is missing or invalid, let
    # storage.get_pick_frame() raise with a clear "Ungueltige Kategorie"
    # error rather than silently picking wrong stock.
    element = get_element(data, i, layer_idx=layer_idx)
    beam_size = element.get("beam_size", "").strip('"').strip("'")

    # Get pick frame from storage
    pick_frame, compartment_id, wobj, extax = storage.get_pick_frame(beam_size)

    # Define approach frames
    # 1. Pre-approach: safe position above storage (180 deg rotated)
    pre_approach = Frame([300, 380, 200], [-1, 0, 0], [0, 1, 0])

    # 2. Approach high: above pick position at Z=200
    approach_high = pick_frame.copy()
    approach_high.point.z = 200

    # 3. Approach low: 50mm above pick frame
    approach_low = pick_frame.copy()
    approach_low.point.z += 50

    # Retract frames (offset 10mm in X+ and Z+ to clear edge)
    retract_offset = 10
    retract_start = Frame(
        [pick_frame.point.x + retract_offset, pick_frame.point.y, pick_frame.point.z + retract_offset],
        pick_frame.xaxis, pick_frame.yaxis
    )
    retract_low = Frame(
        [approach_low.point.x + retract_offset, approach_low.point.y, approach_low.point.z + retract_offset],
        approach_low.xaxis, approach_low.yaxis
    )
    retract_high = Frame(
        [approach_high.point.x + retract_offset, approach_high.point.y, approach_high.point.z + retract_offset],
        approach_high.xaxis, approach_high.yaxis
    )

    if dry_run:
        print(f"[PICK] layer={layer_idx} i={i}")
        print(f"  beam_size: {beam_size}")
        print(f"  compartment: {compartment_id}")
        print(f"  wobj: {wobj}, extax: {extax}")
        print(f"  === APPROACH ===")
        print(f"  1. pre_approach:    X={pre_approach.point.x:.0f} Y={pre_approach.point.y:.0f} Z={pre_approach.point.z:.0f}")
        print(f"  2. approach_high:   X={approach_high.point.x:.0f} Y={approach_high.point.y:.0f} Z={approach_high.point.z:.0f}")
        print(f"  3. approach_low:    X={approach_low.point.x:.0f} Y={approach_low.point.y:.0f} Z={approach_low.point.z:.0f}")
        print(f"  4. pick_frame:      X={pick_frame.point.x:.0f} Y={pick_frame.point.y:.0f} Z={pick_frame.point.z:.0f}")
        print(f"  === RETRACT (+10mm X/Z) ===")
        print(f"  1. retract_start:   X={retract_start.point.x:.0f} Y={retract_start.point.y:.0f} Z={retract_start.point.z:.0f}")
        print(f"  2. retract_low:     X={retract_low.point.x:.0f} Y={retract_low.point.y:.0f} Z={retract_low.point.z:.0f}")
        print(f"  3. retract_high:    X={retract_high.point.x:.0f} Y={retract_high.point.y:.0f} Z={retract_high.point.z:.0f}")
        print(f"  4. pre_approach:    X={pre_approach.point.x:.0f} Y={pre_approach.point.y:.0f} Z={pre_approach.point.z:.0f}")
        return

    # Set work object first
    r1.send(rrc.SetWorkObject(wobj))

    # Open the gripper
    gripper_open(r1, dry_run=dry_run, wait=False)

    # === APPROACH ===

    # 1. Coordinated move to pre-approach with track
    r1.send_and_wait(cm.MoveToRobtarget(
        frame=pre_approach,
        ext_axes=[extax],
        time=COORD_MOVE_TIME,
        zone=rrc.Zone.Z50,
        motion_type=rrc.Motion.JOINT
    ))
    print(f"At pre-approach (extax={extax}).")

    # 2. Move to approach_high (Z=200)
    r1.send(rrc.MoveToFrame(approach_high, SPEED_NO_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    # 3. Linear down to approach_low (50mm above pick)
    r1.send(rrc.MoveToFrame(approach_low, SPEED_APPROACH, rrc.Zone.Z5, rrc.Motion.LINEAR))

    # 4. Linear down to pick_frame
    r1.send(rrc.MoveToFrame(pick_frame, SPEED_PRECISE, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # === CSS + GRIP ===
    # CSS (Cartesian Soft Servo) makes Y and Z axes soft so the gripper
    # conforms to the beam surface. The robot presses 5mm down into the
    # beam stack (smaller than Swissbau26's 10mm because the 25mm cross-section
    # would deform under the deeper press).

    if css_enabled:
        # Define CSS profile: Y/Z axes at 50% softness, 100mm allowed deviation
        r1.send_and_wait(rrc.CustomInstruction('r_RRC_CI_CSS', ['Define', 'CSS_YZ'], [50, 50, 100]))
        r1.send_and_wait(rrc.CustomInstruction('r_RRC_CI_CSS', ['On', 'AllowMove'], []))

        # Press 5mm down into the beam for secure contact (25mm beam section)
        pick_press = pick_frame.copy()
        pick_press.point.z -= 5
        r1.send(rrc.MoveToFrame(pick_press, SPEED_PRECISE, rrc.Zone.FINE, rrc.Motion.LINEAR))

    r1.send(rrc.WaitTime(0.5))

    # Close the gripper
    gripper_close(r1, dry_run=dry_run, wait=True)

    # Activate beam geometry in RobotStudio simulation (beam appears at TCP)
    if sim_beams:
        sim_beam_activate(r1, layer_idx, i, dry_run=dry_run)

    # Define and activate gripper load (tells the controller the mass/inertia
    # of the gripped beam so motion planning accounts for it)
    # Parameters: mass=0.3kg (smaller for 25mm beam), CoG=(0,0,0.01), inertia matrix
    r1.send(rrc.CustomInstruction('r_RRC_CI_GripLoad', ['Define'], [0.3, 0, 0, 0.01, 1, 0, 0, 0, 0, 0, 0]))
    r1.send(rrc.CustomInstruction('r_RRC_CI_GripLoad', ['On'], []))

    r1.send(rrc.WaitTime(0.5))

    # Decrement beam count in storage JSON (persists to disk)
    storage.take_beam(compartment_id)

    # === RETRACT (offset path to avoid scraping edge) ===

    # 1. Lift and shift: 10mm X+ and Z+ from current position
    r1.send(rrc.MoveToFrame(retract_start, SPEED_PRECISE, rrc.Zone.Z5, rrc.Motion.LINEAR))

    # 2. Linear up to retract_low
    r1.send(rrc.MoveToFrame(retract_low, SPEED_PRECISE, rrc.Zone.Z5, rrc.Motion.LINEAR))

    # 3. Linear up to retract_high
    r1.send(rrc.MoveToFrame(retract_high, SPEED_APPROACH, rrc.Zone.Z10, rrc.Motion.LINEAR))

    if css_enabled:
        r1.send_and_wait(rrc.CustomInstruction('r_RRC_CI_CSS', ['Off'], []))

    # 4. Exit to pre-approach offset (Y-150 to clear storage)
    exit_frame = pre_approach.copy()
    exit_frame.point.y -= 150
    r1.send_and_wait(rrc.MoveToFrame(exit_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.LINEAR))

    print(f"Left pick station. Took from {compartment_id}.")


if __name__ == "__main__":
    # Load fab data (standalone)
    DATA = load_data()

    # Show storage status
    storage = WoodStorage()
    storage.print_status()

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    # Set tool to gripper
    r1.send(rrc.SetTool(TOOL_GRIPPER))

    # Execute pick station
    a_pick_station(r1, DATA, i=0, layer_idx=0, dry_run=False)

    # Show updated status
    storage.print_status()

    print("Finished")

    ros.close()
    ros.terminate()
