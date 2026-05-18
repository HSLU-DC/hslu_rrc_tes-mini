# state.py
"""State-Section in fab_data.json verwalten (tes-v4 SSOT).

Schema (data["state"]):
    {
        "run_id": str | None,
        "started_at": ISO8601 str | None,
        "last_updated_at": ISO8601 str | None,
        "elements": {
            "L0_E0": {
                "pick":  {"status": "pending|in_progress|done|failed",
                          "at": ISO8601 str | None,
                          "error": str | None},
                "cut":   {...},
                "glue":  {...},
                "place": {...},
            },
            ...
        },
        "errors": [str, ...],
    }

Persistierung: atomic save via tmp + os.replace.
"""
import os
import uuid
import compas
from datetime import datetime
from pathlib import Path

from _skills.fabdata import PATH

ACTIONS = ("pick", "cut", "glue", "place")
STATUSES = ("pending", "in_progress", "done", "failed")


def element_ref(layer_idx, elem_idx):
    return "L{}_E{}".format(layer_idx, elem_idx)


def _now():
    return datetime.now().isoformat()


# ==============================================================================
# Persistierung
# ==============================================================================

def save_atomic(data, path=PATH):
    """Atomic write: tmp file + os.replace. Verhindert korrupte fab_data.json
    bei Crash waehrend des Schreibens."""
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    compas.json_dump(data, str(tmp), pretty=True)
    os.replace(str(tmp), str(path))


# ==============================================================================
# Run-Lifecycle
# ==============================================================================

def start_run(data, run_id=None):
    """Setzt state.run_id und state.started_at. Setzt nicht zurueck."""
    state = data["state"]
    state["run_id"] = run_id or "run-{}".format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
    state["started_at"] = _now()
    state["last_updated_at"] = state["started_at"]
    return state["run_id"]


def reset_state(data):
    """Setzt alle Element-Actions auf 'pending' zurueck. Nuetzlich nach
    Materialfehler oder fuer Re-Run desselben fab_data."""
    state = data["state"]
    for ref, actions in state["elements"].items():
        for action in ACTIONS:
            if action in actions:
                actions[action] = {"status": "pending", "at": None, "error": None}
    state["run_id"] = None
    state["started_at"] = None
    state["last_updated_at"] = _now()
    state["errors"] = []


# ==============================================================================
# Action-Updates
# ==============================================================================

def mark_action(data, ref, action, status, error=None):
    """Setzt status fuer ein Element/Action und aktualisiert last_updated_at.

    Mutiert data, persistiert NICHT - Aufrufer muss save_atomic() bauen.
    """
    if action not in ACTIONS:
        raise ValueError("Unbekannte action '{}'. Erlaubt: {}".format(action, ACTIONS))
    if status not in STATUSES:
        raise ValueError("Unbekannter status '{}'. Erlaubt: {}".format(status, STATUSES))

    state = data["state"]
    if ref not in state["elements"]:
        state["elements"][ref] = {a: {"status": "pending", "at": None, "error": None} for a in ACTIONS}

    state["elements"][ref][action] = {
        "status": status,
        "at": _now(),
        "error": error,
    }
    state["last_updated_at"] = _now()


def append_error(data, message):
    state = data["state"]
    state["errors"].append({"at": _now(), "message": message})
    state["last_updated_at"] = _now()


# ==============================================================================
# Queries
# ==============================================================================

def get_action_status(data, ref, action):
    state = data["state"]
    if ref not in state["elements"]:
        return "pending"
    return state["elements"][ref].get(action, {}).get("status", "pending")


def is_done(data, ref, action):
    return get_action_status(data, ref, action) == "done"


def all_actions_done(data, ref):
    return all(is_done(data, ref, a) for a in ACTIONS)


def find_resume_point(data, production_plan):
    """Erstes (layer_idx, elem_idx) aus production_plan, das noch eine
    nicht-done Action hat. None wenn alles fertig.

    production_plan: Liste von (layer_idx, elem_idx)-Tuples.
    """
    for layer_idx, elem_idx in production_plan:
        ref = element_ref(layer_idx, elem_idx)
        if not all_actions_done(data, ref):
            return (layer_idx, elem_idx)
    return None


def has_progress(data):
    """True wenn mind. eine Action != pending. Hinweis ob ein Resume sinnvoll ist."""
    for actions in data["state"]["elements"].values():
        for action in ACTIONS:
            if actions.get(action, {}).get("status") != "pending":
                return True
    return False


def progress_summary(data):
    """String fuer Konsolen-Ausgabe: 'X von Y Actions done, Z failed'."""
    state = data["state"]
    total = 0
    done = 0
    failed = 0
    for actions in state["elements"].values():
        for action in ACTIONS:
            total += 1
            status = actions.get(action, {}).get("status", "pending")
            if status == "done":
                done += 1
            elif status == "failed":
                failed += 1
    return "{}/{} actions done, {} failed".format(done, total, failed)
