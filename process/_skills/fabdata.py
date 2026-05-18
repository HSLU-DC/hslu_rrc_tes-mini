# fabdata.py
"""Fabrication data loading and access helpers (tes-v4 SSOT).

Schema: data["fabrication"]["layers"][layer_idx]["elements"][i].
Vorher v2/v3-Fallbacks entfernt - sauberer Schnitt nach Refactor.
"""
import compas
from pathlib import Path

# _skills/fabdata.py  ->  data/fab_data.json
PATH = Path(__file__).resolve().parents[1] / "data" / "fab_data.json"


def load_data(path=PATH):
    """Load fabrication data from JSON file (tes-v4 SSOT)."""
    return compas.json_load(str(path))


def _fabrication_layers(data):
    return data["fabrication"]["layers"]


# ==============================================================================
# Layer functions
# ==============================================================================

def get_layer_count(data):
    """Anzahl Layer in fabrication."""
    return len(_fabrication_layers(data))


def get_layer(data, layer_idx):
    """Layer-Dict mit "id" und "elements"."""
    return _fabrication_layers(data)[layer_idx]


def get_layer_elements(data, layer_idx):
    """Liste der Element-Dicts fuer einen Layer."""
    return get_layer(data, layer_idx)["elements"]


# ==============================================================================
# Element functions
# ==============================================================================

def get_element(data, i, layer_idx=0):
    """Element-Dict mit allen Fabrication-Frames (Workobject-Koordinaten)."""
    return _fabrication_layers(data)[layer_idx]["elements"][i]


def get_element_count(data, layer_idx=0):
    """Anzahl Elemente in einem Layer."""
    return len(_fabrication_layers(data)[layer_idx]["elements"])


def get_total_element_count(data):
    """Anzahl Elemente ueber alle Layer."""
    return sum(len(layer["elements"]) for layer in _fabrication_layers(data))
