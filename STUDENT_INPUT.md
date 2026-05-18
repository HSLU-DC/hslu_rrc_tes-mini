# Fassadenelemente - Input-Spezifikation

## Koordinatensystem

Alle Geometrie wird im **Weltkoordinatensystem** von Rhino erstellt. Dieses entspricht dem Roboter-Workobject `ob_HSLU_Place`.

Der **Ursprung (0, 0, 0) liegt oben links** am Grundrahmen.

- **X-Achse:** entlang der langen Seite des Rahmens (0 bis 2500mm)
- **Y-Achse:** entlang der kurzen Seite des Rahmens (0 bis -600mm)
- **Z-Achse:** nach oben, weg vom Rahmen

<img src="docs/images/00_koordinatensystem.png" width="600">

---

## Datenstruktur

Die Daten werden als **Grasshopper DataTree** mit der Pfad-Struktur `{Layer;Element}` organisiert.

- **Layer 0:** Erste Lage direkt auf dem Grundrahmen
- **Layer 1:** Zweite Lage auf Layer 0 (optional)
- Maximal **2 Layer** erlaubt

Die **Reihenfolge der Elemente im Tree bestimmt die Reihenfolge**, in der der Roboter die Elemente platziert. Überlegt euch die Sequenz sorgfältig - ein Element kann nur dort platziert werden, wo es aufliegt.

Jeder Branch im Haupt-Tree enthält **4 Einträge**:

| Index | Name | Typ | Beschreibung |
|-------|------|-----|-------------|
| 0 | Brep | Brep | Fertige Balkengeometrie |
| 1 | Centerline | Line | Mittelachse des Balkens |
| 2 | Cut Plane A | Plane | Schnittebene Ende A |
| 3 | Cut Plane B | Plane | Schnittebene Ende B |

**Leimebenen werden in einem zweiten DataTree** mit gleichem `{Layer;Element}`-Pfad geliefert. Pro Branch könnt ihr **0 bis beliebig viele** Leimebenen reingeben — die Reihenfolge im Branch ist die Reihenfolge in der der Roboter sie anfährt. Leerer Branch = der Roboter überspringt die Leimstation für dieses Element.

<img src="docs/images/01_datatree.png" width="600">

---

## 0 - Brep (Balkengeometrie)

Die vollständige 3D-Geometrie des **fertigen Holzstabs** mit 25x25mm Querschnitt. Die Gehrungsschnitte an beiden Enden müssen bereits ausgeführt sein - das Brep zeigt den Balken so, wie er am Ende auf dem Rahmen liegt.

Das Brep wird **nicht an den Roboter übertragen**. Es dient zur Visualisierung, Kollisionsprüfung und als Grundlage für die automatische Berechnung der Roboterpositionen.

**Anforderungen:**
- Querschnitt: 25 x 25mm
- Geschlossenes Brep (Solid)
- Gehrungsschnitte an beiden Enden bereits modelliert

<img src="docs/images/02_brep.png" width="600">

---

## 1 - Centerline (Mittelachse)

Die Mittelachse des **fertigen** (zugeschnittenen) Balkens als Linie. Die Centerline verläuft exakt durch die Mitte des Balkenquerschnitts.

**Anforderungen:**
- **Startpunkt:** Mitte der Stirnfläche an Ende A
- **Endpunkt:** Mitte der Stirnfläche an Ende B
- Einfache Linie (Line), keine Kurve

Die Centerline bestimmt automatisch:
- Die **Platzierungsposition** (Mittelpunkt der Centerline)
- Die **Platzierungsrichtung** (Richtung der Centerline)
- Die **stock_category** (Länge der Centerline bestimmt aus welchem Lager das Holz geholt wird)

<img src="docs/images/03_centerline.png" width="600">

### Stock-Längen und Verschnitt

Im Lager liegen Stäbe in **vier festen Längen**: **400 / 550 / 750 / 1000 mm**.

Aus der Centerline-Länge wird automatisch der **nächst-grössere** Bucket gewählt; die Differenz wird in der Cut-Station weggeschnitten.

| Centerline-Länge | Stock-Länge (= stock_category) | Verschnitt |
|---|---|---|
| 0 - 400 mm   | 400 mm  | bis zu 400 mm |
| 401 - 550 mm | 550 mm  | bis zu 150 mm |
| 551 - 750 mm | 750 mm  | bis zu 200 mm |
| 751 - 1000 mm | 1000 mm | bis zu 250 mm |

**Maximale Centerline-Länge: 1000 mm** (längster verfügbarer Stock).

**Minimale Centerline-Länge:** abhängig von der Länge **und** vom Schnittwinkel zur Kreissäge — kurze Stäbe mit steilen Gehrungsschnitten können vom Greifer aus nicht erreichbar sein. **Prüft das in der GH-IK-Vorschau** für eure spezifischen Geometrien.

### Greifer-Position

Der Greifer fasst den Stab **immer mittig** (im Schwerpunkt). Diese Annahme ist im GH-Template fest hinterlegt — ihr müsst die Greifposition nicht selbst angeben.

---

## 2 - Cut Plane A (Schnittebene Ende A)

Die Schnittebene am **ersten Ende** des Balkens (Startpunkt der Centerline). Diese Plane definiert, wie die Säge das Holz schneidet.

**Position:**
- Mitte der Stirnfläche an Ende A (= Startpunkt der Centerline)

**Orientierung:**
- **Z-Achse:** zeigt **nach aussen**, weg vom Balken (= Normale der Schnittfläche)
- **Y-Achse:** zeigt **nach oben** (Welt-Z Richtung)
- **X-Achse:** ergibt sich automatisch

**Wichtig:**
- Nur **Gehrungsschnitte** (1D) sind erlaubt. Das bedeutet: die Schnittebene darf nur um die vertikale Achse gedreht sein. Eine Neigung der Schnittebene nach oben oder unten (Schifterschnitt / 2D-Schnitt) ist **nicht möglich** und wird von der Validierung abgelehnt.

<img src="docs/images/04_cut_plane_orientierung.png" width="600">

---

## 3 - Cut Plane B (Schnittebene Ende B)

Die Schnittebene am **zweiten Ende** des Balkens (Endpunkt der Centerline). Identische Konvention wie Cut Plane A.

**Position:**
- Mitte der Stirnfläche an Ende B (= Endpunkt der Centerline)

**Orientierung:**
- **Z-Achse:** zeigt **nach aussen**, weg vom Balken
- **Y-Achse:** zeigt **nach oben** (Welt-Z Richtung)
- **X-Achse:** ergibt sich automatisch


---

## 4 - Glue Plane A (Leimebene 1)

Die erste Leimebene definiert, wo der Roboter Leim aufträgt. Der Leim wird in der Leimstation auf die **Unterseite** des Balkens aufgetragen, bevor das Element platziert wird.

**Position:**
- Auf der **Unterseite** des Balkens (= Kontaktfläche zum Element darunter)
- In der **Mitte** der Leimfläche (Plane sitzt 12.5mm von allen Rändern entfernt)
- Die Leimfläche ist 25 x 25mm (= Querschnitt des Balkens)
- **Entlang des Balkens:** dort wo der Balken auf einem anderen Element oder dem Grundrahmen aufliegt — am Ende ODER weiter innen (siehe 90°-Regel unten)

**Orientierung:**
- **Z-Achse:** zeigt **nach unten** (Richtung Unterlage / Rahmen)
- **X-Achse:** zeigt **Richtung Mitte** des Elements (entlang der Centerline, zur Balkenmitte hin)
- **Y-Achse:** ergibt sich automatisch

**Wo soll die Leimebene sitzen?** Dort wo der Balken auf einem anderen Element oder dem Grundrahmen aufliegt. Das kann am Ende sein (typisch bei kreuzenden Elementen) oder weiter innen — wichtig ist die strukturelle Verbindung.

<img src="docs/images/05_glue_plane_orientierung.png" width="600">

### 90°-Regel bei innen liegender Leimebene

Liegt die Leimebene mehr als **100mm vom Balkenende** entfernt (entlang der Centerline gemessen), muss sie um **90° um die Plane-Z-Achse** gedreht werden. Hintergrund: hinter der Leimdüse steht der **Drucker**. In der normalen Orientierung würde der Stab über die Düse hinaus nach hinten ragen und mit dem Drucker kollidieren. Durch die 90°-Drehung wird der Stab seitlich statt längs zur Düse geführt — der Bereich hinter der Düse bleibt frei.

In **welche Richtung** (CW oder CCW um Z) gedreht werden muss, hängt davon ab, wie der gedrehte Stab in den verfügbaren Bauraum passt. Probiert beide Drehrichtungen aus und prüft in Grasshopper:

1. **Erreichbarkeit** der Roboterpose in der IK-Vorschau
2. **Kollisionsfreiheit mit der Einhausung** — die Einhausung ist im GH-Template bereits modelliert und visuell sichtbar

Die RobotStudio-Simulation (siehe README, Workflow-Schritt 3) prüft zusätzlich die gesamte Anlage auf Kollisionen (Roboter, Greifer, weitere Anlagenteile) — als Schluss-Verifikation vor dem realen Lauf.

> **Hinweis zur Orientierung nach der Drehung:** Nach der 90°-Drehung zeigt die Plane-X-Achse nicht mehr „Richtung Balkenmitte" wie unter § 4 oben beschrieben, sondern senkrecht zum Balken. Das ist in diesem Fall **erlaubt und gewollt** — die ursprüngliche X-Achsen-Konvention gilt nur für nicht gedrehte Planes (innerhalb 100 mm vom Balkenende).

<img src="docs/images/07_glue_plane_innen_90deg.png" width="600">

---

## Weitere Leimebenen (optional, beliebig viele)

Pro Element könnt ihr **0 bis beliebig viele** Leimebenen liefern. Identische Konvention wie Glue Plane A — inkl. der **90°-Regel bei innen liegender Plane** (>100mm vom Balkenende). Die Reihenfolge im Branch des Glue-Plane-DataTrees bestimmt die Anfahrtsreihenfolge.

**Wann brauche ich mehrere Leimebenen?**
- Wenn der Balken an **mehreren Stellen** auf anderen Elementen oder dem Grundrahmen aufliegt (typisch bei Layer 1, oder bei langen Balken die mehrfach verleimt werden)

**Wann reicht eine Leimebene oder gar keine?**
- Eine: einseitige Fixierung
- Keine: das Element wird ohne Leim gesetzt — der Roboter überspringt die ganze Leimstation

Entscheidung liegt bei euch - überlegt was strukturell sinnvoll ist.


---

## Zusammenfassung der Plane-Konventionen

| Plane | Z-Achse | Y-Achse | X-Achse |
|-------|---------|---------|---------|
| Cut Plane A | nach aussen (weg vom Balken) | nach oben (Welt-Z) | ergibt sich |
| Cut Plane B | nach aussen (weg vom Balken) | nach oben (Welt-Z) | ergibt sich |
| Glue Plane (alle) | nach unten | ergibt sich | Richtung Balkenmitte |

<img src="docs/images/06_uebersicht_alle_planes.png" width="600">

---

## Constraints / Einschränkungen

| Constraint | Wert |
|-----------|------|
| Rahmengrösse | X: 0 - 2500mm, Y: 0 - -600mm |
| Balkenquerschnitt | 25 x 25mm |
| Stock-Längen (stock_category) | 400 / 550 / 750 / 1000 mm (automatisch aus Centerline gewählt) |
| Max Centerline-Länge | 1000 mm |
| Min Centerline-Länge | abhängig von Schnittwinkel — in GH-IK-Vorschau prüfen |
| Maximale Anzahl Layer | 2 |
| Schnitttyp | Nur Gehrungsschnitte (1D) |
| Leimebenen pro Element | 0 - N (variable Anzahl, separater DataTree) |
| Leimebenen-Position | beliebig entlang Balken; ab 100mm vom Ende muss die Plane um 90° um Z gedreht werden (Drehrichtung kollisionsfrei zur Einhausung wählen — siehe § 4) |
| Platzierungsreihenfolge | = Reihenfolge im DataTree |
