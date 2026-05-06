# production.py
"""Main production orchestrator for the Facade FS26 robot cell.

This script controls the full fabrication workflow for timber facade element
assembly. Each beam goes through 4 stations in sequence:

    1. PICK   — Retrieve beam from wood storage (WoodStorage inventory system)
    2. CUT    — Cut beam to length at circular saw station (2 cuts per beam)
    3. GLUE   — Apply adhesive with Robatech glue system (PLC-controlled)
    4. PLACE  — Place beam onto facade frame (track-compensated approach)

The fabrication data (positions, sizes, orientations) is exported from
Grasshopper as JSON (see design/gh_python/ExportFacade.py) and organized
in layers. Each layer contains multiple elements (beams).

Usage:
    1. Start Docker (cd docker && docker-compose -f REAL-docker-compose.yml up -d)
    2. Configure RUN CONFIG below
    3. Run: python production.py

Configuration (in this file):
    - Enable/disable individual stations with DO_PICK, DO_CUT, etc.
    - Hardware flags (CSS_ENABLED, SAW_ENABLED, GLUE_VALVE_ENABLED)
      allow testing motion paths without activating tools
    - SIM_BEAMS drives the BeamSimulator SmartComponent in RobotStudio
      (virtual controller only — has no effect on the real cell)

Runtime selection (interactive at startup):
    - Layer (0 or 1, only prompted if data has multiple layers)
    - Element range (Enter = alle, oder z.B. "5-10" / "12")
"""

# ==============================
# Imports
# ==============================
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# SIM_FAST must be evaluated BEFORE the station imports below, because the
# stations bind the SPEED_* values from globals at their own import time.
# Patch globals here so the subsequent `from stations import ...` picks up
# the boosted values.
#
# DO NOT enable on the real cell. Sim-only.
SIM_FAST = True
SIM_FAST_FACTOR = 8

if SIM_FAST:
    import globals as _g
    _scale_names = (
        "SPEED_GLUE", "SPEED_NO_MEMBER", "SPEED_WITH_MEMBER",
        "SPEED_APPROACH", "SPEED_PRECISE", "SPEED_CUT",
    )
    for _n in _scale_names:
        setattr(_g, _n, int(getattr(_g, _n) * SIM_FAST_FACTOR))
    _g.MAX_TCP = max(_g.MAX_TCP, _g.SPEED_NO_MEMBER + 100)
    print(f"[SIM_FAST] Alle Speeds x{SIM_FAST_FACTOR} - nur fuer Simulation!")
    del _g, _scale_names, _n

import compas_rrc as rrc
from prompt_toolkit import prompt as ptk_prompt

from _skills.fabdata import load_data, has_layers, get_layer_count, get_element_count, get_element
from _skills.SimBeam import sim_beam_reset
from _skills.WoodStorage.wood_storage import WoodStorage, VALID_CATEGORIES
from globals import ROBOT_NAME, TOOL_GRIPPER
from joint_positions import jp_home, jp_park
import _skills.custom_motion as cm

from stations import a_pick_station, b_cut_station, d_glue_station, e_place_station


# ==============================
# RUN CONFIG
# ==============================
# Station toggles — set to False to skip a station (robot still moves between stations)
DO_PICK  = True
DO_CUT   = True
DO_GLUE  = True
DO_PLACE = True

# Hardware toggles — False = dry-run motion without tool activation
CSS_ENABLED = True            # Cartesian Soft Servo for gentle gripping at pick
SAW_ENABLED = True            # Circular saw on/off during cut moves
GLUE_VALVE_ENABLED = True     # Glue valve pulsing during glue moves
SIM_BEAMS = True              # BeamSimulator SmartComponent (virtual controller only)

# Demo mode: skip all wood-storage operator prompts. Initial check auto-accepts
# the recommendation, mid-production refills auto-fill without parking. Lets a
# RobotStudio simulation run end-to-end without interaction. DO NOT enable on
# the real cell — the operator never gets a chance to actually load wood.
AUTO_REFILL_LAGER = True

# Production range (LAYER / START_I / N_RUNS) is asked interactively at runtime.
# Default = full layer 0; the operator can choose layer + element range.


def _compute_demand(data, production_plan):
    """Count required beams per category for a production plan.

    Args:
        data: Loaded fab_data dict
        production_plan: list of (layer_idx, element_idx) tuples

    Returns:
        (demand_dict, None, None) on success
        (None, bad_size, (layer, i)) if an element has unknown beam_size
    """
    demand = {cat: 0 for cat in VALID_CATEGORIES}
    for layer, i in production_plan:
        element = get_element(data, i, layer_idx=layer)
        beam_size = element.get("beam_size", "").strip('"').strip("'")
        if beam_size not in demand:
            return None, beam_size, (layer, i)
        demand[beam_size] += 1
    return demand, None, None


def _prompt_int(prompt_text, default, lo, hi):
    """Prompt for an integer in [lo, hi] with editable pre-fill.

    The default value is pre-filled in the input line; arrow keys, Backspace
    and editing work as expected. Press Enter to accept the (possibly edited)
    value. An empty input also returns the default.
    """
    while True:
        response = ptk_prompt(prompt_text, default=str(default)).strip()
        if not response:
            return default
        try:
            value = int(response)
        except ValueError:
            print("    Bitte Zahl eingeben.")
            continue
        if value < lo or value > hi:
            print(f"    Ungueltig ({lo}..{hi})")
            continue
        return value


def check_wood_storage(data, production_plan):
    """Pre-flight: show demand vs lager, single confirm or per-category override.

    Default flow (typical case):
        1. Compute demand per category from production_plan
        2. Display table with Bedarf / Kapazitaet / Empfehlung + refill warnings
        3. Single yes/no prompt
        4. On yes: apply recommendations
        5. On no: per-category override (with default = recommendation)

    Mid-production refills (when a lager runs empty during the loop) are
    handled separately by refill_lager().

    Args:
        data: Loaded fab_data dict
        production_plan: list of (layer_idx, element_idx) tuples

    Returns:
        True on success, False on bad data (unknown beam_size in design)
    """
    storage = WoodStorage()

    demand, bad_size, bad_loc = _compute_demand(data, production_plan)
    if demand is None:
        bad_layer, bad_i = bad_loc
        print(f"\n[FEHLER] Unbekannte beam_size: '{bad_size}' bei Layer {bad_layer}, Element {bad_i}")
        return False

    # Recommended counts (capped at lager capacity)
    recommendations = {
        cat: min(demand[cat], storage.get_capacity(cat))
        for cat in VALID_CATEGORIES
    }

    # ---- Summary table ----
    print("\n" + "=" * 60)
    print("HOLZLAGER START")
    print("=" * 60)
    print("\nAus dem Design berechnet:\n")
    print("  Kategorie   Bedarf   Lager max   Empfehlung")
    print("  " + "-" * 47)
    for cat in VALID_CATEGORIES:
        cap = storage.get_capacity(cat)
        rec = recommendations[cat]
        print(f"  {cat+' mm':<11} {demand[cat]:>6}   {cap:>9}   {rec:>10}")

    # Refill warnings (only for categories where demand exceeds capacity)
    warnings = []
    for cat in VALID_CATEGORIES:
        cap = storage.get_capacity(cat)
        if demand[cat] > cap:
            n_refills = (demand[cat] - 1) // cap
            warnings.append(f"  ! {cat} mm: {n_refills} Auffuellung(en) waehrend Produktion noetig")
    if warnings:
        print()
        for w in warnings:
            print(w)

    # ---- Auto mode for demos: accept recommendation without prompt ----
    if AUTO_REFILL_LAGER:
        for cat in VALID_CATEGORIES:
            storage.set_count(cat, recommendations[cat])
        print("\n[AUTO] Lager wie empfohlen konfiguriert (AUTO_REFILL_LAGER=True).")
        return True

    # ---- Single confirmation prompt ----
    print("\nHast du genau die Empfehlung ins Lager gelegt?")
    print("  [Enter] = ja, weiter mit Produktion")
    print("  [n]     = Werte einzeln korrigieren")

    response = ptk_prompt("> ").strip().lower()

    if response in ("", "j", "ja", "y", "yes"):
        for cat in VALID_CATEGORIES:
            storage.set_count(cat, recommendations[cat])
        print("\n[OK] Lager wie empfohlen konfiguriert.")
        return True

    # ---- Per-category override ----
    # Default is pre-filled and editable: arrow keys / Backspace work,
    # Enter accepts. Hit Enter to keep the recommendation unchanged.
    print("\nWieviele Stueck liegen im Lager? (Wert editierbar, dann Enter)\n")
    for cat in VALID_CATEGORIES:
        cap = storage.get_capacity(cat)
        if demand[cat] == 0:
            # Auto-skip categories that aren't needed
            storage.set_count(cat, 0)
            continue
        rec = recommendations[cat]
        count = _prompt_int(f"  {cat} mm (max {cap}): ", rec, 0, cap)
        storage.set_count(cat, count)

    print()
    storage.print_status()
    return True


def refill_lager(r1, storage, category, remaining_demand, *, dry_run=False):
    """Mid-production refill prompt for an empty lager category.

    Parks the robot at jp_park (track retracted, robot folded), then asks
    the operator how many beams were nachgefuellt. Updates storage counts
    via storage.set_count(category, n).

    Args:
        r1: AbbClient instance (or None for dry_run)
        storage: WoodStorage instance to update
        category: Empty category that triggered the refill
        remaining_demand: How many beams of this category are still needed
                          for the rest of the production run
        dry_run: If True, skips robot motion (the prompt still runs)
    """
    capacity = storage.get_capacity(category)
    recommended = min(remaining_demand, capacity)

    # Demo mode: no park move, no operator prompt — just refill to recommendation
    # so a RobotStudio simulation can run end-to-end without interaction.
    if AUTO_REFILL_LAGER:
        storage.set_count(category, recommended)
        print(f"\n[AUTO-REFILL] Lager '{category}' mm leer -> auf {recommended} Stueck gesetzt.\n")
        return

    if not dry_run and r1 is not None:
        r1.send_and_wait(cm.MoveToJoints(jp_park.robax, jp_park.extax, 1, rrc.Zone.Z50))

    print("\n" + "=" * 60)
    print(f"  LAGER {category} mm IST LEER - bitte nachfuellen")
    print("=" * 60)
    print(f"\n  Noch benoetigt fuer Rest der Produktion: {remaining_demand:>3} Stueck")
    print(f"  Maximal in dieses Lager:                 {capacity:>3} Stueck")
    print(f"  Empfehlung jetzt nachzufuellen:          {recommended:>3} Stueck")
    print(f"\n  (Roboter ist geparkt - du kannst sicher ans Lager.)")

    print()
    count = _prompt_int(
        "Anzahl nachgefuellt: ",
        recommended, 1, capacity,
    )
    storage.set_count(category, count)
    print(f"  [OK] Lager '{category}' auf {count} Stueck gesetzt.\n")


def _remaining_demand(data, production_plan, plan_idx, category):
    """Count beams of `category` still needed from plan_idx onwards (inclusive).

    Spans multiple layers if production_plan does — used by the mid-production
    refill prompt to tell the operator the total still-required count.
    """
    return sum(
        1 for layer, i in production_plan[plan_idx:]
        if get_element(data, i, layer_idx=layer)
               .get("beam_size", "").strip('"').strip("'") == category
    )


def _prompt_layer(n_layers):
    """Prompt for layer selection.

    Returns:
        int: a specific layer index
        None: produce all layers (only possible when n_layers > 1)

    Auto-returns 0 (no prompt) if data has only one layer.
    """
    if n_layers <= 1:
        return 0

    print("\nWelcher Layer?")
    print(f"  [Enter] = beide (alle Elemente)")
    for i in range(n_layers):
        print(f"  {i}       = nur Layer {i}")

    while True:
        response = ptk_prompt("> ", default="").strip()
        if not response:
            return None
        try:
            value = int(response)
        except ValueError:
            print("    Bitte Zahl eingeben oder Enter fuer beide.")
            continue
        if value < 0 or value >= n_layers:
            print(f"    Ungueltig (0..{n_layers-1})")
            continue
        return value


def _prompt_element_range(n_total):
    """Prompt for element range. Returns (start_i, n_runs).

    Empty input    -> alle Elemente (0, n_total)
    "X-Y"          -> Range X bis Y inkl. (X, Y-X+1)
    "X"            -> nur Element X (X, 1)
    """
    while True:
        response = ptk_prompt("> ", default="").strip()
        if not response:
            return 0, n_total
        try:
            if "-" in response:
                a, b = response.split("-", 1)
                start = int(a.strip())
                end = int(b.strip())
            else:
                start = int(response.strip())
                end = start
        except ValueError:
            print("    Ungueltig. Format: 'X-Y' oder 'X' (z.B. '5-10' oder '12')")
            continue

        if start < 0 or end >= n_total or start > end:
            print(f"    Ungueltig. Erlaubt: 0..{n_total-1}, start <= end")
            continue

        return start, end - start + 1


def main(*, dry_run=False):
    """Run the production loop.

    Workflow:
        1. Load fabrication data from JSON, run validation
        2. Prompt for layer + element range to produce
        3. Check wood storage inventory (interactive, skipped in dry_run)
        4. Connect to robot via ROS bridge, optionally reset SimBeam state
        5. Loop over elements: PICK -> CUT -> GLUE -> PLACE
        6. Move to safe end position, disconnect

    Args:
        dry_run: If True, simulates the entire workflow without robot
                 connection. All stations print their planned moves
                 instead of executing them. Useful for verifying
                 fab_data before running on the real robot.
    """

    # ==============================
    # 1. Load Data
    # ==============================
    print("\n" + "=" * 50)
    print("FACADE PRODUCTION")
    print("=" * 50)

    DATA = load_data()

    # ==============================
    # 2. Layer + Element Range Selection
    # ==============================
    if has_layers(DATA):
        n_layers = get_layer_count(DATA)
        print(f"Daten: {n_layers} Layer")
    else:
        n_layers = 1
        print("Daten im v2-Format (kein Layer)")

    layer_choice = _prompt_layer(n_layers)

    # Build production plan: list of (layer_idx, element_idx) tuples
    if layer_choice is None:
        # All layers, all elements
        production_plan = []
        for layer in range(n_layers):
            n = get_element_count(DATA, layer_idx=layer)
            production_plan.extend((layer, i) for i in range(n))
    else:
        # Single layer + range selection
        layer = layer_choice
        n_total = get_element_count(DATA, layer_idx=layer)
        print(f"\nLayer {layer} hat {n_total} Elemente (0..{n_total-1}).")
        print("Welche Elemente produzieren?")
        print("  Enter = alle")
        print("  Range = z.B. '5-10' oder einzelnes '12'")
        start_i, n_runs = _prompt_element_range(n_total)
        production_plan = [(layer, start_i + k) for k in range(n_runs)]

    if not production_plan:
        print("[FEHLER] Keine Elemente zum Produzieren!")
        return

    n_total_run = len(production_plan)

    # Plan summary by layer
    by_layer = {}
    for ly, i in production_plan:
        by_layer.setdefault(ly, []).append(i)
    print(f"\nProduziere {n_total_run} Element(e):")
    for ly in sorted(by_layer.keys()):
        elems = by_layer[ly]
        if len(elems) == 1:
            print(f"  Layer {ly}: Element {elems[0]}")
        else:
            print(f"  Layer {ly}: {len(elems)} Elemente ({elems[0]}..{elems[-1]})")

    # ==============================
    # 3. Check Wood Storage
    # ==============================
    if not dry_run:
        if not check_wood_storage(DATA, production_plan):
            return

    # ==============================
    # 4. Connect Robot
    # ==============================
    if dry_run:
        r1 = None
        ros = None
        print("\n*** DRY RUN MODE - keine Roboterbewegungen ***\n")
    else:
        ros = rrc.RosClient()
        ros.run()
        r1 = rrc.AbbClient(ros, ROBOT_NAME)
        print("Connected.")
        r1.send(rrc.SetTool(TOOL_GRIPPER))

        # Lift controller speed cap when running in SIM_FAST mode so the boosted
        # speeds aren't capped by the controller-side MAX_TCP default.
        if SIM_FAST:
            import globals as _g
            r1.send(rrc.SetMaxSpeed(100, _g.MAX_TCP))
            print(f"[SIM_FAST] Controller MaxSpeed: {_g.MAX_TCP} mm/s")

        # Move to home / safe pose before any production work
        r1.send_and_wait(cm.MoveToJoints(jp_home.robax, jp_home.extax, 1, rrc.Zone.Z50))

        # Clear any beams from previous run so the facade starts empty
        if SIM_BEAMS:
            sim_beam_reset(r1)

    # ==============================
    # 5. Production Loop
    # ==============================
    # Storage instance for the lager-empty fail-safe (separate from the one
    # the pick station maintains internally; we reload from disk before each
    # check so take_beam() decrements from inside a_pick_station are visible).
    storage_check = None if dry_run else WoodStorage()

    for plan_idx, (ly, i) in enumerate(production_plan):
        print(f"\n{'='*50}")
        print(f"  LAYER {ly} | ELEMENT {i} ({plan_idx+1}/{n_total_run})")
        print(f"{'='*50}")

        if DO_PICK:
            element = get_element(DATA, i, layer_idx=ly)
            beam_size = element.get("beam_size", "").strip('"').strip("'")

            # Fail-safe: if the lager for this category is empty, park robot
            # and prompt operator to refill before attempting the pick.
            if not dry_run:
                storage_check.reload()
                if not storage_check.has_beams(beam_size):
                    remaining = _remaining_demand(DATA, production_plan, plan_idx, beam_size)
                    refill_lager(r1, storage_check, beam_size, remaining)

            print("\n--- PICK ---")
            a_pick_station.a_pick_station(r1, DATA, i, layer_idx=ly, dry_run=dry_run, css_enabled=CSS_ENABLED, sim_beams=SIM_BEAMS)

        if DO_CUT:
            print("\n--- CUT ---")
            b_cut_station.b_cut_station(r1, DATA, i, layer_idx=ly, dry_run=dry_run, saw_enabled=SAW_ENABLED, sim_beams=SIM_BEAMS)

        if DO_GLUE:
            print("\n--- GLUE ---")
            d_glue_station.d_glue_station(r1, DATA, i, layer_idx=ly, dry_run=dry_run, glue_valve_enabled=GLUE_VALVE_ENABLED)

        if DO_PLACE:
            print("\n--- PLACE ---")
            e_place_station.e_place_station(r1, DATA, i, layer_idx=ly, dry_run=dry_run, sim_beams=SIM_BEAMS)

    # ==============================
    # 6. Cleanup
    # ==============================
    if not dry_run and r1 is not None:
        # Move to home / safe pose after all elements are placed
        r1.send_and_wait(cm.MoveToJoints(jp_home.robax, jp_home.extax, 1, rrc.Zone.Z50))

    if not dry_run and ros is not None:
        ros.close()
        ros.terminate()

    print(f"\n{'='*50}")
    print(f"  FERTIG - {n_total_run} Elemente produziert")
    print(f"{'='*50}")


if __name__ == "__main__":
    main(dry_run=False)  # Set True to simulate without robot connection
