# HSLU RRC Mini-TES — Robotische Vorfertigung einer Holz-Lehm-Decke

Robotische Vorfertigung der Holztragstruktur einer hybriden Timber-Earth-Slab-Decke (TES)
mit dem ABB Gofa CRB 15000 auf Güdel-Schiene. Dieses Repository enthält den Mini-TES-Demonstrator
der Bachelor-Thesis von Juri Jerg (Digital Construction, HSLU) — vom parametrischen Konfigurator
über ein gemeinsames Datenmodell bis zur Stationslogik und RobotStudio-Simulation.

<img src="docs/images/00_anlage.jpeg" width="600">

## Pipeline

```
Konfigurator (Design) → JSON-Datenmodell → Python (Roboter-Steuerung) → Roboter / RobotStudio
```

Der Konfigurator legt das Lattenlayout fest und exportiert ein gemeinsames Datenobjekt
(`process/data/fab_data.json`). Der gesamte nachgelagerte Prozess hängt allein an der Struktur
dieser Datei, nicht daran, wie sie zustande kommt — das ist die schema-getriebene Durchgängigkeit
der Pipeline.

**Stationen:** Pick → Cut → Glue → Place

| Station | Was passiert |
|---------|-------------|
| **Pick**  | Roboter holt eine Latte aus dem Pre-Cut-Lager |
| **Cut**   | Roboter fährt zur Säge, schneidet die Latte auf Länge (Gehrungsschnitte) |
| **Glue**  | Roboter führt die Latte an die stationäre Leimdüse (Variante A, Hot-Melt) |
| **Place** | Roboter platziert die Latte auf dem wachsenden Bauteil |

## Bauteil und Lager

| Parameter | Wert |
|-----------|------|
| Bauteilmass | 1000 × 600 mm |
| Lagen | 6 |
| Lattenquerschnitt | 25 × 25 mm |
| Latten (je nach Konfiguration) | rund 106 |
| Pre-Cut-Lagerkategorien | 400 / 550 / 750 / 1000 mm |
| Klebstoff-Topologie | Variante A: stationäre Düse, Hot-Melt |

## Datenmodell

Pro Element liefert der Konfigurator vier Geometrien im Haupt-DataTree mit `{Layer;Element}`-Struktur:

| Index | Name | Typ | Beschreibung |
|-------|------|-----|-------------|
| 0 | Brep | Brep | Fertige Lattengeometrie (25 × 25 mm, mit Gehrung) |
| 1 | Centerline | Line | Mittelachse der fertigen Latte |
| 2 | Cut Plane A | Plane | Schnittebene Ende A |
| 3 | Cut Plane B | Plane | Schnittebene Ende B |

Plus 0..N Leimebenen pro Element in einem separaten DataTree (gleicher `{Layer;Element}`-Pfad).
Reihenfolge im Branch = Anfahrtsreihenfolge, leerer Branch = keine Leimung. Alle Geometrien im
Weltkoordinatensystem von Rhino (`ob_HSLU_Place`). Automatisch abgeleitet werden `stock_category`
(aus der Centerline-Länge), `place_position` (aus der Centerline) und die Roboterposen
(Transformation in die jeweiligen Workobjects).

## Workflow

### 1. Design im Konfigurator
Lattenlayout im Grasshopper-Template (`design/hslu_rrc_tes-mini.ghx`) erzeugen, Roboterdarstellung
(Inverse Kinematics) visuell auf Erreichbarkeit prüfen und die Daten exportieren. Die Validierung
läuft direkt im Template (visuelles Feedback grün/orange/rot).

### 2. JSON prüfen
Die exportierte Datei liegt unter `process/data/fab_data.json`.

### 3. RobotStudio-Simulation (vor jedem realen Lauf)
Das Design lässt sich in ABB RobotStudio vollständig simulieren — mit dynamisch eingeblendeter
Lattengeometrie (`BeamSimulator`) und kontinuierlicher Kollisionsprüfung.

```bash
# Virtuellen Controller starten
cd docker
docker compose -f VIRTUAL-docker-compose.yml up -d

# Produktionsskript starten
cd ../process
python production.py
```

> **Tipp:** In den RobotStudio-Einstellungen die Simulationsgeschwindigkeit auf Maximum stellen,
> sonst läuft die Simulation in Echtzeit.

### 4. Produktion (echte Anlage)

> **Wichtig:** Vor dem Lauf an der echten Anlage in `production.py` **`SIM_FAST = False`** setzen
> (Default `True` für die RS-Simulation). `SIM_FAST = True` multipliziert alle Verfahrgeschwindigkeiten —
> auf der echten Anlage gefährlich.

```bash
# Docker für die echte Anlage starten
cd docker
docker compose -f REAL-docker-compose.yml up -d

# Produktion starten
cd ../process
python production.py
```

Beim Start fragt das Skript interaktiv nach Layer-Auswahl und Element-Bereich, zählt den Lattenbedarf
pro Lagerkategorie (400 / 550 / 750 / 1000 mm) und gleicht ihn mit `wood_storage.json` ab. Läuft ein
Bucket mid-production leer, pausiert der Roboter, statt abzubrechen — nach dem Nachlegen läuft die
Produktion weiter.

#### Toggle-Flags in `production.py`

```python
# Stationen ein/aus (Roboter fährt trotzdem die Wege zwischen den Stationen)
DO_PICK  = True
DO_CUT   = True
DO_GLUE  = True
DO_PLACE = True

# Werkzeuge ein/aus (False = Bewegung wird ausgeführt, Werkzeug bleibt inaktiv)
CSS_ENABLED        = True   # Cartesian Soft Servo am Pick (sanftes Greifen)
SAW_ENABLED        = True   # Säge beim Schneiden
GLUE_VALVE_ENABLED = True   # Leim-Ventil beim Leimen

# Simulation-only Flags (auf der echten Anlage egal / muss aus)
SIM_FAST  = False   # MUSS False auf echter Anlage! (True nur für RS-Sim, x4 Speed)
SIM_BEAMS = True    # BeamSimulator in RS — auf echter Anlage wirkungslos
```

## Requirements

**Design:** Rhino 8 mit Grasshopper, [Robot Components](https://github.com/RobotComponents/RobotComponents) v4.1.0 (GH-Plugin).

**Produktion** (am Anlagen-PC eingerichtet): Python 3.13 (Anaconda), [COMPAS](https://compas.dev/) v2.10,
[compas_rrc](https://github.com/compas-rrc/compas_rrc), [compas_fab](https://github.com/compas-rrc/compas_fab),
Docker Desktop (ROS + ABB Driver).

## Projektstruktur

```
hslu_rrc_tes-mini/
├── README.md
├── design/                  # Konfigurator: Rhino-/Grasshopper-Dateien
│   ├── hslu_rrc_tes-mini.ghx
│   └── gh_python/           # GH-Python-Skripte (Export, Holzbedarf)
├── process/                 # Roboter-Steuerung
│   ├── production.py         # Hauptskript
│   ├── globals.py            # Konfiguration
│   ├── joint_positions.py
│   ├── data/
│   │   ├── fab_data.json     # Export aus dem Konfigurator
│   │   └── wood_storage.json # Holzlager-Inventar
│   ├── _skills/              # Robot-Skills (Greifer, Bewegung, Leimlinie ...)
│   ├── stations/             # Station-Code (Pick, Cut, Glue, Place)
│   └── scripts/              # Hilfs- und Testskripte
├── robotstudio/             # RobotStudio: BeamSimulator-SmartComponent
├── docker/                  # ROS + ABB Driver (REAL / VIRTUAL)
└── docs/                    # Dokumentation und Bilder
```

## Schwesterprojekt: Pipeline-Generalität

Der Fertigungs-Stack (`process/`, `_skills/`, `stations/`) ist domänenagnostisch und wird mit dem
Schwesterprojekt **[hslu_rrc_facade](https://github.com/HSLU-DC/hslu_rrc_facade)** geteilt: Dort
erzeugen Studierende frei geformte Fassadenpaneele über einen anderen Konfigurator, speisen aber
dasselbe Datenmodell und durchlaufen denselben Stations- und Skill-Code. Beide Anwendungsfälle
unterscheiden sich nur im Design-Frontend — das ist der praktische Beleg der schema-getriebenen
Pipeline-Generalität.

## Troubleshooting

| Problem | Lösung |
|---------|--------|
| Roboter in GH zeigt unrealistische Pose | Geometrie anpassen, Position nicht erreichbar |
| GH-Validierung zeigt Fehler (orange/rot) | Daten korrigieren und neu exportieren |
| "Nicht genug Holz" | Holzlager auffüllen, das Skript fragt danach |
| Docker-Fehler | `docker compose down && docker compose up -d` |
| Roboter antwortet nicht | Controller eingeschaltet und im AUTO-Modus? |

## Kontakt

Juri Jerg — juri.jerg@hslu.ch
