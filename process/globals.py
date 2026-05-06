# globals.py
"""Global configuration for the Facade production cell.

All constants used across multiple stations are defined here.
Values must match the ABB controller configuration (tools, work objects,
speeds are defined in RAPID and referenced by name here).

Important: Tool and work object names are RAPID-side identifiers.
If they are renamed on the controller, they must be updated here too.
"""

# ==============================================================================
# Robot identity
# ==============================================================================
ROBOT_NAME = "/rob1"           # ROS namespace, must match docker-compose.yml
PROJECT_NAME = "Facade FS26"

# ==============================================================================
# Tools (defined in RAPID on the controller)
# ==============================================================================
TOOL_SPIKE = 't_HSLU_Spike'            # Spike/drill tool
TOOL_GRIPPER = 't_HSLU_GripperZimmer'  # Zimmer pneumatic gripper

# ==============================================================================
# Work objects (defined in RAPID on the controller)
# ==============================================================================
# Each station has its own work object with a coordinate system calibrated
# to the station's physical position. The pick station work objects are
# defined per beam size in wood_storage.json (not here).
W_OBJ_CUT = 'ob_HSLU_Cut'
W_OBJ_GLUE = 'ob_HSLU_Glue'
W_OBJ_PLACE = 'ob_HSLU_Place'

# The place work object origin is offset from the track zero by this amount.
# Used in e_place_station to calculate: trackpos = x - offset + this value
OFFSET_TRACK_W_OBJ_PLACE = 593  # mm

# ==============================================================================
# TCP speeds (mm/s)
# ==============================================================================
SPEED_GLUE = 1000         # Fast — glue valve pulsing compensates for speed
SPEED_NO_MEMBER = 500     # Moving without a beam (empty gripper)
SPEED_WITH_MEMBER = 300   # Moving with a beam in the gripper
SPEED_APPROACH = 100      # Final approach to target positions
SPEED_PRECISE = 25        # Pick/place fine positioning
SPEED_CUT = 25            # Moving through the saw blade

# ==============================================================================
# Motion controller settings
# ==============================================================================
ACC = 100       # Acceleration [%]
RAMP = 100      # Acceleration ramp [%]
OVERRIDE = 100  # Speed override [%] (100 = full speed)
MAX_TCP = 1000  # Maximum TCP speed [mm/s]

# ==============================================================================
# Facade-specific constants
# ==============================================================================
# Frame dimensions: bounds for student element placement.
# Origin (0,0) is top-left of the frame; valid X range 0..FRAME_LENGTH,
# valid Y range -FRAME_WIDTH..0.
FRAME_WIDTH = 600        # mm (Y-direction)
FRAME_LENGTH = 2500      # mm (X-direction)

# Beam cross-section: production beams are smaller than the base frame beams.
# These values affect stack offsets in wood_storage.json and gripper load.
BEAM_SECTION = 25        # mm (cross-section of production beams)
FRAME_SECTION = 40       # mm (cross-section of base frame beams)

