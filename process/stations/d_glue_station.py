# d_glue_station.py
"""Station D: Apply adhesive to beam surfaces.

The glue system uses a Robatech hot-melt adhesive melter controlled via
a Beckhoff PLC. A safety handshake (glue_on/glue_off) ensures the safety
cell is closed before enabling the glue system.

Each beam has 0..N student-provided glue positions (element["glue_positions"])
on the top face where it will contact the beam above. The robot drives them
in list order. Empty list = element is skipped entirely (no station entry).

At each plane the robot drives a predefined glue path pattern: a zigzag
fill of N parallel lines covering a 15x15 mm active area (5 mm margin
on each side of the 25x25 mm beam top).

Glue application is executed entirely in RAPID (r_RRC_CI_GlueLine) to
avoid Python-ROS-RAPID latency — this ensures consistent glue bead quality.
"""

# ==============================
# Imports
# ==============================
import math
import sys
import os
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import compas_rrc as rrc
from compas.geometry import Frame

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.GlueLine.glue_line import glue_line
from _skills.GluePLC.glue_plc import glue_on, glue_off

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_WITH_MEMBER, SPEED_APPROACH, SPEED_GLUE, SPEED_NO_MEMBER,
    W_OBJ_GLUE,
)
from joint_positions import jp_glue, jp_glue_rot, jp_glue_transit

# Duration in seconds for coordinated (track + robot) moves
COORD_MOVE_TIME = 1


# ==============================================================================
# Helpers
# ==============================================================================

# Tolerance for treating two glue planes as identically oriented. If all three
# axes (X, Y, Z) of plane B are within this many degrees of plane A's axes,
# the robot can transit directly from A's retract to B's approach without
# returning to jp_glue_rot for a safe reorientation.
ORIENTATION_TOL_DEG = 2.0


def _computed_zaxis(frame):
    """Robust z-axis: x cross y. Used for dry_run debug output."""
    z = frame.xaxis.cross(frame.yaxis)
    if z.length > 1e-6:
        z.unitize()
    return z


def _orientations_match(f1, f2, tol_deg=ORIENTATION_TOL_DEG):
    """True if all three axes of f1 and f2 are parallel within tol_deg."""
    cos_tol = math.cos(math.radians(tol_deg))
    pairs = (
        (f1.xaxis, f2.xaxis),
        (f1.yaxis, f2.yaxis),
        (f1.zaxis, f2.zaxis),
    )
    return all(a1.dot(a2) >= cos_tol for a1, a2 in pairs)


def _glue_plan(glue_frames):
    """Yield (frame, tag, enter_via_rot, exit_via_rot) per glue plane.

    enter_via_rot: True if we need to come via jp_glue_rot to safely reorient
                   the TCP to this plane (first plane, or orientation changed).
    exit_via_rot:  True if we need to return to jp_glue_rot after this plane
                   (last plane, or next plane has different orientation).
    """
    n = len(glue_frames)
    prev = None
    for idx, frame in enumerate(glue_frames, start=1):
        is_last = (idx == n)
        nxt = glue_frames[idx] if not is_last else None
        enter_via_rot = (prev is None
                         or not _orientations_match(prev, frame))
        exit_via_rot = (is_last
                        or not _orientations_match(frame, nxt))
        yield frame, str(idx), enter_via_rot, exit_via_rot
        prev = frame


def _build_offset_frames(start_frame):
    """Build the intermediate helper frames around the glue start position.

    Returns:
        pre: Pre-approach frame (Z+300, X+300 from start) — unused at runtime,
             only printed in dry_run for context.
        app: Approach frame (Z+30, X+30 from start)
        ret: Retract frame (Z+30, X+30 from start) — same as app to minimise
             travel between consecutive direct-mode glue planes.
    """
    pre = start_frame.copy()
    pre.point.z += 300
    pre.point.x += 300

    app = start_frame.copy()
    app.point.z += 30
    app.point.x += 30

    ret = start_frame.copy()
    ret.point.z += 30
    ret.point.x += 30

    return pre, app, ret


def _run_glue_line(r1, glue_frame, *, glue_valve_enabled=True,
                   line_length=15, y_step=7.5, num_lines=3,
                   pulse_on_ms=20, pulse_off_ms=20, accel_dist=0):
    """Execute a zigzag fill of parallel glue lines around glue_frame.

    Lines run along the frame's local X. The pattern is centered on glue_frame:
      - Each line is ``line_length`` mm long, from -L/2 to +L/2 in X.
      - ``num_lines`` parallel lines are spaced ``y_step`` mm apart, symmetric
        about Y=0.
      - Direction alternates each line (zigzag); shifts in Y between lines run
        without glue.

    Defaults give a 15x15 mm zigzag fill (3 lines at Y=-7.5/0/+7.5, each 15 mm)
    on the 25x25 mm beam top, leaving a 5 mm margin on every side.

    Args:
        glue_frame: TCP frame at the centre of the active glue area (beam
                    centred under the nozzle).
        glue_valve_enabled: If True, pulses the glue valve during each line.
        line_length: Length of each line along local X in mm.
        y_step: Spacing between adjacent lines in mm.
        num_lines: Number of parallel lines.
        pulse_on_ms / pulse_off_ms / accel_dist: Pass-through to glue_line.
    """
    half_x = line_length / 2.0
    y_offsets = [(i - (num_lines - 1) / 2.0) * y_step for i in range(num_lines)]

    for idx, y_off in enumerate(y_offsets):
        going_neg = (idx % 2 == 0)
        x_offset = -line_length if going_neg else +line_length
        x_start = +half_x if going_neg else -half_x

        line_start = glue_frame.copy()
        line_start.point.x += x_start
        line_start.point.y += y_off

        # Position TCP at line start (no glue). FINE on the first line so the
        # bead begins from rest; Z1 on subsequent shifts to keep it smooth.
        zone_in = rrc.Zone.FINE if idx == 0 else rrc.Zone.Z1
        speed_in = SPEED_APPROACH if idx == 0 else SPEED_GLUE
        r1.send_and_wait(rrc.MoveToFrame(line_start, speed_in, zone_in, rrc.Motion.LINEAR))

        if glue_valve_enabled:
            glue_line(r1, line_start, x_offset=x_offset, speed=SPEED_GLUE,
                      pulse_on_ms=pulse_on_ms, pulse_off_ms=pulse_off_ms,
                      accel_dist=accel_dist)
        else:
            line_end = line_start.copy()
            line_end.point.x += x_offset
            r1.send_and_wait(rrc.MoveToFrame(line_end, SPEED_GLUE, rrc.Zone.FINE, rrc.Motion.LINEAR))


# ==============================================================================
# Glue sequence
# ==============================================================================

def _do_glue_sequence(r1, glue_frame, *, dry_run=False, tag="",
                      glue_valve_enabled=True,
                      enter_via_rot=True, exit_via_rot=True):
    """[reorient at jp_glue_rot] -> app -> glue line -> ret -> [jp_glue_rot].

    Args:
        r1: AbbClient instance
        glue_frame: Start frame for the glue line
        dry_run: If True, prints planned moves only
        tag: Label used in print statements ("1", "2", ...)
        glue_valve_enabled: If True, opens/closes glue valve during path
        enter_via_rot: True = robot is at jp_glue_rot, do TCP-reorient first
                       (Precondition: robot positioned at jp_glue_rot).
                       False = robot is at previous retract with matching
                       orientation, go straight to approach.
        exit_via_rot: True = return to jp_glue_rot after retract — safe for
                      reorientation or station exit.
                      False = stay at retract, ready for next plane with
                      same orientation.
    """
    pre, app, ret = _build_offset_frames(glue_frame)

    prefix = f"[GLUE {tag}] " if tag else "[GLUE] "

    if dry_run:
        z_calc = _computed_zaxis(glue_frame)
        entry = "rot+reorient" if enter_via_rot else "direct(ret->app)"
        exit_path = "->rot" if exit_via_rot else "stay@ret"
        print(f"{prefix}z_calc={z_calc} entry={entry} exit={exit_path}")
        if enter_via_rot:
            print(f"{prefix}  rot joints: {jp_glue_rot.robax} extax={jp_glue_rot.extax}")
        print(f"{prefix}  pre:   X={pre.point.x:.0f} Y={pre.point.y:.0f} Z={pre.point.z:.0f}")
        print(f"{prefix}  app:   X={app.point.x:.0f} Y={app.point.y:.0f} Z={app.point.z:.0f}")
        print(f"{prefix}  start: X={glue_frame.point.x:.0f} Y={glue_frame.point.y:.0f} Z={glue_frame.point.z:.0f}")
        print(f"{prefix}  ret:   X={ret.point.x:.0f} Y={ret.point.y:.0f} Z={ret.point.z:.0f}")
        return

    print(f"{prefix}Executing glue sequence "
          f"(entry={'rot' if enter_via_rot else 'direct'}, "
          f"exit={'rot' if exit_via_rot else 'ret'}).")

    if enter_via_rot:
        # Reorient TCP at jp_glue_rot's position to glue_frame's orientation BEFORE
        # translating. A combined rotate+translate move makes the held beam swing
        # during the approach, which can collide with the glue station; rotating in
        # place first means the subsequent move to `app` is pure translation.
        current_tcp = r1.send_and_wait(rrc.GetFrame())
        rot_oriented = Frame(current_tcp.point, glue_frame.xaxis, glue_frame.yaxis)
        r1.send_and_wait(rrc.MoveToFrame(rot_oriented, SPEED_APPROACH, rrc.Zone.FINE, rrc.Motion.LINEAR))

        # Coordinated move to approach: track shifts from jp_glue_rot.extax (1000)
        # to jp_glue.extax (500), robot reaches app simultaneously.
        r1.send_and_wait(cm.MoveToRobtarget(app, jp_glue.extax, 1, rrc.Zone.Z10, rrc.Motion.LINEAR))
    else:
        # Direct mode: previous plane's orientation matches and track is already
        # at jp_glue.extax — pure TCP linear at SPEED_WITH_MEMBER, no time-based
        # coord-move overhead. Pipelines into the previous plane's retract via Z10.
        r1.send_and_wait(rrc.MoveToFrame(app, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    # Execute zigzag glue pattern (handles its own move-to-line-start)
    _run_glue_line(r1, glue_frame, glue_valve_enabled=glue_valve_enabled)

    if exit_via_rot:
        # Pipeline: ret + jp_glue_rot can flow together; the wait is on rot.
        # Beam ends up high above the arm, ready for reorientation or station exit.
        r1.send(rrc.MoveToFrame(ret, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))
        r1.send_and_wait(cm.MoveToJoints(jp_glue_rot.robax, jp_glue_rot.extax,
                                         COORD_MOVE_TIME, rrc.Zone.FINE))
    else:
        # Stay near retract — next plane has matching orientation. `send`
        # (no wait) lets the next call's approach blend in via Z10 without
        # stopping at ret. Net effect: ret_prev -> app_next as one smooth
        # linear move at SPEED_WITH_MEMBER.
        r1.send(rrc.MoveToFrame(ret, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))


# ==============================================================================
# Main station entry point
# ==============================================================================

def d_glue_station(r1, data, i, *, layer_idx=0, dry_run=False, glue_valve_enabled=True):
    """Apply adhesive to beam at all glue positions in element["glue_positions"].

    The PLC safety handshake (glue_on/glue_off) brackets the entire operation —
    the glue system is only active while the robot is in the glue station.

    Args:
        r1: AbbClient instance (or None for dry_run)
        data: Loaded fab_data dict
        i: Element index within the layer
        layer_idx: Layer index (0 or 1)
        dry_run: If True, prints planned moves without robot connection
        glue_valve_enabled: If True, pulses the glue valve during motion.
                            If False, robot moves through the glue path
                            without dispensing (for position testing).
    """
    element = get_element(data, i, layer_idx=layer_idx)
    glue_frames = element.get("glue_positions", [])

    # Skip element entirely if no glue positions are defined — no station entry,
    # no PLC handshake. Saves the ~10s round-trip for elements that don't glue.
    if not glue_frames:
        print(f"[GLUE] Skipping element {i} - no glue positions defined")
        return

    if dry_run:
        print(f"[GLUE] i={i} layer={layer_idx}, n_planes={len(glue_frames)}")
        glue_on(r1, dry_run=True)
        for frame, tag, enter_via_rot, exit_via_rot in _glue_plan(glue_frames):
            _do_glue_sequence(r1, frame, dry_run=True, tag=tag,
                              glue_valve_enabled=glue_valve_enabled,
                              enter_via_rot=enter_via_rot,
                              exit_via_rot=exit_via_rot)
        glue_off(r1, dry_run=True)
        return

    # Move to glue station
    r1.send(
        cm.MoveToJoints(jp_glue.robax, jp_glue.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    # Transit waypoint on the way to jp_glue_rot. The direct path would sweep
    # the held beam through the robot arm.
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue_transit.robax, jp_glue_transit.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    # Position once at jp_glue_rot here so each _do_glue_sequence call can
    # skip the redundant entry move (each call leaves us at rot, the next reuses it).
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue_rot.robax, jp_glue_rot.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    print("At glue position.")

    r1.send(rrc.SetWorkObject(W_OBJ_GLUE))

    # PLC Safety Handshake: wait for glue system ready
    glue_on(r1)
    print("Glue system enabled.")

    for frame, tag, enter_via_rot, exit_via_rot in _glue_plan(glue_frames):
        _do_glue_sequence(r1, frame, tag=tag,
                          glue_valve_enabled=glue_valve_enabled,
                          enter_via_rot=enter_via_rot,
                          exit_via_rot=exit_via_rot)

    # Turn glue system off
    glue_off(r1)
    print("Glue system disabled.")

    # Leave station - transit first, then back to jp_glue
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue_transit.robax, jp_glue_transit.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue.robax, jp_glue.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    print("Left glue station.")


if __name__ == "__main__":
    DATA = load_data()

    ros = rrc.RosClient()
    ros.run()

    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    r1.send(rrc.SetTool(TOOL_GRIPPER))

    d_glue_station(r1, DATA, i=0, layer_idx=0, dry_run=False, glue_valve_enabled=False)

    print("Finished")
    ros.close()
    ros.terminate()
