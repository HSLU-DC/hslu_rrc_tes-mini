# HSLU RRC Facade - Parametrische Fassadenelemente

Robotische Produktion von parametrischen Fassadenelementen mit dem ABB Gofa CRB 15000.

> **Hinweis:** Der `process/` Ordner (Python/Roboter-Steuerung) ist aktuell noch **Work in Progress**. Der Grasshopper-Workflow und die Input-Spezifikation sind bereit.

<img src="docs/images/00_anlage.jpeg" width="600">

## Übersicht

Ihr designt in Grasshopper ein parametrisches Fassadenelement aus 25x25mm Holzlatten,
das robotisch auf einen Grundrahmen (600 x 2500mm, 40x40mm) platziert wird.

**Pipeline:** Grasshopper (Design) → JSON Export → Python (Roboter-Steuerung)

**Stationen:** Pick → Cut → Glue → Place

| Station | Was passiert |
|---------|-------------|
| **Pick** | Roboter holt Holz aus dem Lager |
| **Cut** | Roboter fährt zur Säge, schneidet beide Enden (Gehrungsschnitte) |
| **Glue** | Roboter fährt zur Leimstation, Leim wird aufgetragen |
| **Place** | Roboter platziert das Holz auf dem Rahmen |

## Was ihr liefern müsst

Ihr liefert pro Element **4 Geometrien** im Haupt-DataTree mit `{Layer;Element}` Struktur:

| Index | Name | Typ | Beschreibung |
|-------|------|-----|-------------|
| 0 | Brep | Brep | Fertige Balkengeometrie (25x25mm, mit Gehrung) |
| 1 | Centerline | Line | Mittelachse des fertigen Balkens |
| 2 | Cut Plane A | Plane | Schnittebene Ende A |
| 3 | Cut Plane B | Plane | Schnittebene Ende B |

Plus **0..N Leimebenen pro Element** in einem **separaten DataTree** (gleicher `{Layer;Element}` Pfad). Reihenfolge im Branch = Anfahrtsreihenfolge. Leerer Branch = keine Leimung für dieses Element.

Alles im **Weltkoordinatensystem** von Rhino (= `ob_HSLU_Place`). Origin liegt oben links am Rahmen.

Die detaillierte Spezifikation mit Bildern findet ihr unter **[`STUDENT_INPUT.md`](STUDENT_INPUT.md)**.

<img src="docs/images/01_rhino_gh_anlage.png" width="600">

Folgendes wird automatisch berechnet:
- **beam_size** (aus Centerline-Länge)
- **place_position** (aus Centerline)
- **Roboter-Positionen** (Transformation in die jeweiligen Workobjects)

## Constraints / Einschränkungen

| Constraint | Wert |
|-----------|------|
| Rahmengrösse | X: 0 - 2500mm, Y: 0 - -600mm |
| Balkenquerschnitt | 25 x 25mm |
| Maximale Anzahl Layer | 2 (Layer 0 und Layer 1) |
| Schnitttyp | Nur Gehrungsschnitte (1D), keine Schifterschnitte |
| Leimebenen pro Element | 0 - N (variable Anzahl) |
| Platzierungsreihenfolge | = Reihenfolge im DataTree |

## Workflow

<img src="docs/images/02_gh_script.png" width="600">

### 1. Design in Grasshopper

1. Öffne das GH-Template (`grasshopper/hslu_rrc_facade.gh`)
2. Verbinde eure Geometrie mit den Inputs (siehe [`STUDENT_INPUT.md`](STUDENT_INPUT.md))
3. Prüfe visuell die Roboterdarstellung (Inverse Kinematics) - sind die Positionen erreichbar?
4. Exportiere die Daten (Button `update = True`)

### 2. JSON prüfen

Die exportierte Datei liegt unter `process/data/fab_data.json`. Die Validierung läuft direkt im Grasshopper-Template (visuelles Feedback grün/orange/rot) — eine separate Python-Validierung gibt es nicht mehr.

### 3. RobotStudio-Simulation (empfohlen vor jedem realen Lauf)

Bevor ihr an die echte Anlage geht, könnt ihr euer Design in **ABB RobotStudio** vollständig simulieren — mit dynamisch eingeblendeter Balkengeometrie und kontinuierlicher Kollisionsprüfung. So findet ihr Probleme bevor sie an der Maschine auftreten.

<img src="docs/images/10_robotstudio_overview.png" width="600">

**Setup:**

1. **Pack'n'Go-Datei** in RobotStudio öffnen: `robotstudio/hslu_rrc_facade_v1.0.0.rspag`
2. **Virtuellen Docker-Container** starten:
   ```bash
   cd docker
   docker compose -f VIRTUAL-docker-compose.yml up -d
   ```
3. **Produktionsskript** wie üblich starten:
   ```bash
   cd ../process
   python production.py
   ```

> **Tipp:** In den RobotStudio-Einstellungen die **Simulationsgeschwindigkeit auf das Maximum** stellen — sonst läuft die Sim in Echtzeit und dauert genauso lange wie die echte Produktion.

<img src="docs/images/11_robotstudio_speed_settings.png" width="500">

**Was die Simulation neu kann:**

- **Dynamische Balkengeometrie** — beim JSON-Export aus Grasshopper wird zusätzlich die Geometrie pro Element exportiert. Der Export dauert dadurch **1-2 Sekunden länger**. In RobotStudio werden die Stäbe in der korrekten Position dargestellt und bewegen sich mit dem Greifer mit.
- **Kollisionserkennung** — während die Simulation läuft, wird kontinuierlich auf Kollisionen zwischen Roboter, Greifer, Balken und Anlagenteilen geprüft. Erkannte Kollisionen werden **visuell als Markups im 3D-View** angezeigt und zusätzlich **im Log-Fenster** ausgegeben.

<img src="docs/images/12_robotstudio_collision.png" width="600">

### 4. Produktion starten (echte Anlage)

> **Wichtig:** Vor dem Lauf an der echten Anlage in `production.py` **`SIM_FAST = False`** setzen (Default ist `True` für die RS-Simulation in Schritt 3). `SIM_FAST = True` multipliziert alle Verfahrgeschwindigkeiten — auf der echten Anlage gefährlich!

```bash
# 1. Docker starten (echte Anlage)
cd docker
docker compose -f REAL-docker-compose.yml up -d

# 2. Produktion starten
cd ../process
python production.py
```

#### Interaktiver Start-Dialog

Beim Start fragt das Skript nach was produziert werden soll.

**1. Layer-Auswahl** (nur wenn die Daten mehrere Layer enthalten):

```
Welcher Layer?
  [Enter] = beide (alle Elemente)
  0       = nur Layer 0
  1       = nur Layer 1
> _
```

**2. Element-Bereich:**

```
Layer 0 hat 12 Elemente (0..11).
> _
```

| Eingabe | Bedeutung |
|---|---|
| `Enter` | alle Elemente des Layers |
| `5-10` | Elemente 5 bis 10 (inklusive) |
| `12` | nur Element 12 |

**3. Lager-Check:** das Skript zählt automatisch wie viele Stäbe pro Kategorie (400 / 550 / 750 / 1000 mm) gebraucht werden und vergleicht mit dem Inventar in `wood_storage.json`. Ist nicht genug da, fragt es ab, wie viele Stäbe ihr nachgelegt habt.

> **Lager leer mid-production:** Wenn unterwegs ein Bucket leer wird, **bricht das Skript nicht ab**. Der Roboter pausiert, ihr legt Stäbe nach und gebt im Terminal die nachgelegte Anzahl ein — die Produktion läuft danach automatisch weiter.

#### Optionale Toggle-Flags in `production.py`

Falls ihr einzelne Stationen oder Werkzeug-Aktivierungen für Tests umgehen wollt, ändert die Flags am Anfang der Datei:

```python
# Stationen ein/aus (Roboter fährt trotzdem die Wege zwischen den Stationen)
DO_PICK  = True
DO_CUT   = True
DO_GLUE  = True
DO_PLACE = True

# Werkzeuge ein/aus (False = Bewegung wird ausgeführt, aber Werkzeug bleibt inaktiv)
CSS_ENABLED        = True   # Cartesian Soft Servo am Pick (sanftes Greifen)
SAW_ENABLED        = True   # Säge beim Schneiden
GLUE_VALVE_ENABLED = True   # Leim-Ventil beim Leimen

# Simulation-only Flags (auf der echten Anlage egal / muss aus)
SIM_FAST  = False   # MUSS False auf echter Anlage! (True nur für RS-Sim, x4 Speed)
SIM_BEAMS = True    # BeamSimulator in RS — auf echter Anlage wirkungslos
```

## Requirements

### Design (bei euch)

- Rhino 8
- Grasshopper (in Rhino 8 integriert)
- [Robot Components](https://github.com/RobotComponents/RobotComponents) **v4.1.0** (GH Plugin, via Package Manager installieren)

### Produktion (wird zur Verfügung gestellt)

> Der Produktions-PC/Laptop an der Anlage ist bereits vollständig eingerichtet.
> Ihr müsst diese Software **nicht** auf euren Rechnern installieren.

- Python 3.13 (Anaconda)
- [COMPAS](https://compas.dev/) v2.10
- [compas_rrc](https://github.com/compas-rrc/compas_rrc)
- [compas_fab](https://github.com/compas-rrc/compas_fab)
- Docker Desktop (ROS + ABB Driver)

## Projektstruktur

```
hslu_rrc_facade/
├── README.md                    # Diese Datei
├── STUDENT_INPUT.md             # Detaillierte Input-Spezifikation
├── docs/
│   └── images/                  # Bilder zur Dokumentation
├── docker/
│   └── docker-compose.yml       # ROS + ABB Driver
├── grasshopper/
│   ├── hslu_rrc_facade.gh       # GH Template
│   ├── export_fab_data.py       # GH Python Export-Script
│   └── validate_fab_data.py     # GH Python Validierung
├── process/                     # (Work in Progress)
│   ├── production.py            # Hauptskript
│   ├── globals.py               # Konfiguration
│   ├── data/
│   │   ├── fab_data.json        # Euer Export (aus GH)
│   │   └── wood_storage.json    # Holzlager-Inventar
│   ├── _skills/                 # Robot-Skills (NICHT verändern!)
│   └── stations/                # Station-Code (NICHT verändern!)
└── design/                      # Eure Design-Dateien
```

**Wichtig:** Dateien in `_skills/` und `stations/` bitte NICHT verändern!

## Troubleshooting

| Problem | Lösung |
|---------|--------|
| Roboter in GH zeigt unrealistische Pose | Geometrie anpassen, Position ist nicht erreichbar |
| GH-Validierung zeigt Fehler (orange/rot) | Daten in GH korrigieren und neu exportieren |
| "Nicht genug Holz" | Holzlager physisch auffüllen, Script fragt danach |
| Docker-Fehler | `docker compose down && docker compose up -d` |
| Roboter antwortet nicht | Prüfe ob Controller eingeschaltet und im AUTO-Modus |

## Kontakt

Bei Problemen: Juri - juri.jerg@hslu.ch kontaktieren.
