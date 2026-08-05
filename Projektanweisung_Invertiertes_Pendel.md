# Projektanweisung: Invertiertes Pendel mit interaktiver Steuerung und automatisierten Reglern
 
## 1. Projektübergreifende Zielsetzung
 
Ziel des Projekts ist die Weiterentwicklung eines bereits bestehenden, formelbasierten OpenModelica-Modells eines invertierten Pendels zu einer vollständigen, spielerischen Simulationsumgebung. Das bestehende Modell wird durch eine Implementierung mit den Standard-Bibliotheken (insbesondere der Modelica Standard Library, z. B. MultiBody, Mechanics, Blocks) ersetzt. Anschließend wird die Anwendung um eine interaktive Steuerung, ein Punktesystem, mehrere automatisierte Regler und verschiedene Schwierigkeitsgrade erweitert. Das Projekt wird vollständig in einem GitHub-Repository versioniert und mündet in einer 20-seitigen schriftlichen Ausarbeitung sowie einer Abschlusspräsentation.
 
## 2. Ausgangslage
 
Repository: [scarrax/InvertedPendulum](https://github.com/scarrax/InvertedPendulum)
 
Aktueller Stand im Repository:
 
- `pendulum.mo`: formelbasiertes Modelica-Modell (Wagen + Stab), flache Bewegungsgleichungen, Eingang `tau`, Ausgänge `s`, `v`, `phi`, `vphi`.
- `export.mos`: OpenModelica-Skript, das aus `pendulum.mo` eine FMU (Co-Simulation, CVODE-Solver) baut.
- `pendulum_game_controlled.py`: Das eigentliche Spiel. Lädt die FMU über `fmpy` (`FMU2Slave`), simuliert in Echtzeit (dt = 0,02 s, 40 s Spieldauer) und visualisiert mit Pygame (Wagen, Pendel, Tachometer, Plots für φ(t) und τ(t)).
- `leaderboard.csv`: persistente Bestenliste (Datum, Zeit, Name, Score).
- `protokoll.md`: enthält bereits eine Lagrange-Herleitung der Bewegungsgleichungen, die Linearisierung um die aufrechte Lage sowie eine Einordnung des aktuellen Reglers. Wichtige Grundlage für Kapitel 2 und 5 der schriftlichen Ausarbeitung, nicht doppelt herleiten.
Steuerung und Spielablauf (Ist-Zustand):
 
- Pfeiltasten steuern die Wagenkraft (`tau = ±MAX_TAU`).
- Taste `H` schaltet zwischen manuellem und Auto-Modus um.
- Im Auto-Modus übernimmt `SimpleController` (PD-Regler auf Winkel und Winkelgeschwindigkeit) die Steuerung.
- Scoring: Bonus für geringe Winkelabweichung von der aufrechten Position, gestaffelt in drei Zonen (grobe Toleranz, 15°-Zone, 5°-Zone).
- Wichtige Konvention: Im Code ist φ = 0 die stabile, hängende Ruhelage und φ = π die instabile, aufrechte Zielposition. Bei jeder Erweiterung (Regler, Scoring, Schwierigkeitsgrade) auf diese Konvention achten.
- `Controller` ist bereits als abstrakte Basisklasse angelegt, `SimpleController` ist die einzige aktuelle Implementierung. Ein TODO im Code deutet weitere Regler an (z. B. LQR oder energiebasierter Swing-up-Controller).
Technologie-Stack (Python-Seite): `pygame` (Visualisierung, Tastatursteuerung), `fmpy` (FMU-Einbindung), `pandas` (Leaderboard).
 
## 3. Arbeitspakete
 
### AP1: Modellierung mit Standardbibliotheken
- Nachbildung der bisherigen, formelbasierten Dynamik aus `pendulum.mo` durch Komponenten aus den Modelica-Standardbibliotheken (naheliegender Ausgangspunkt laut Code-Kommentar: `Modelica.Mechanics.MultiBody.Examples.Elementary.Pendulum`).
- Validierung: Das neue Modell muss dieselben Signale (`s`, `v`, `phi`, `vphi` bei gegebenem `tau`) liefern wie das ursprüngliche, formelbasierte Modell (Vergleich anhand definierter Testszenarien).
- Der bestehende FMU-Export-Workflow (`export.mos`) muss mit dem neuen Modell weiterhin funktionieren, da `pendulum_game_controlled.py` die FMU über `fmpy` einliest.
- Dokumentation der verwendeten Komponenten und der Modellstruktur.
### AP2: Interaktive Steuerung und Punktesystem
- Bestehende Pygame-Steuerung (Pfeiltasten, Taste `H` für Auto-Modus) bleibt Grundlage, wird bei Bedarf erweitert.
- Punktesystem: Bestehende Zonen-Logik (grobe Toleranz, 15°-Zone, 5°-Zone um φ = π) prüfen, dokumentieren und bei Bedarf verfeinern.
- Leaderboard (`leaderboard.csv`, über `pandas`) bleibt bestehen, ggf. um Regler-Läufe oder Schwierigkeitsgrad als Spalte erweitern.
### AP3: Automatisierte Regler
- Bestehende `Controller`-Basisklasse in `pendulum_game_controlled.py` als Grundlage nutzen, `SimpleController` (PD-Regler auf φ und dessen Ableitung) bleibt als Referenz erhalten.
- Weitere Reglerklassen nach demselben Interface implementieren, zum Beispiel:
  - LQR mit vollem Zustand (s, v, θ, θ̇), um zusätzlich die Wagenposition zu regeln (im Protokoll bereits als sinnvolle Erweiterung genannt).
  - Energiebasierter Swing-up-Controller, relevant weil die Modell-Anfangsbedingung (φ ≈ 67,5°) nicht in der Nähe der aufrechten Lage liegt.
  - Optional: Pole Placement oder ein nichtlinearer bzw. lernbasierter Regler als weitere Erweiterung.
- Vergleich der Regler hinsichtlich Stabilität, Reaktionszeit und Robustheit gegenüber Störungen.
### AP4: Schwierigkeitsgrade
- Definition mehrerer Schwierigkeitsstufen, zum Beispiel durch Variation von:
  - Pendellänge oder Masseverteilung
  - Reibung
  - Störgrößen (z. B. zufällige Stöße)
  - Zulässiger Toleranzbereich für die Punktevergabe
- Jede Stufe sollte spürbar unterschiedliche Anforderungen an Spieler oder Regler stellen.
### AP5: Schriftliche Ausarbeitung (ca. 20 Seiten)
Empfohlene Gliederung:
1. Einleitung und Motivation
2. Theoretische Grundlagen (Mechanik des invertierten Pendels, Regelungstechnik)
3. Modellierung mit Standardbibliotheken (Vorgehen, Herausforderungen, Validierung)
4. Umsetzung der Spielmechanik und des Punktesystems
5. Reglerentwurf und Vergleich
6. Schwierigkeitsgrade und deren Auswirkung
7. Ergebnisse und Diskussion
8. Fazit und Ausblick
### AP6: Präsentation
- Aufbereitung der wichtigsten Ergebnisse in einer Präsentation.
- Live-Demo empfehlenswert, sofern zeitlich und technisch machbar.
## 4. Technische Anforderungen
 
- Modellierungsumgebung: OpenModelica (OMEdit), Nutzung der MultiBody-Bibliothek für die mechanische Struktur.
- Kopplung Modelica–Python: über FMU (Co-Simulation, CVODE), Export via `export.mos`, Einbindung via `fmpy` (`FMU2Slave`). Diese Schnittstelle bleibt bei der Umstellung auf Standardbibliotheken bestehen.
- Python-Seite: `pygame` (Visualisierung, Steuerung), `pandas` (Leaderboard), neue Regler als Unterklassen der bestehenden `Controller`-Basisklasse.
- Versionierung: GitHub-Repository ([scarrax/InvertedPendulum](https://github.com/scarrax/InvertedPendulum)) mit sinnvoller Branch- und Commit-Struktur.
- Reproduzierbarkeit: Alle Modelle und Regler müssen aus dem Repository heraus nachvollziehbar lauffähig sein (FMU muss vor Spielstart im Projektroot liegen).
## 5. Offene Punkte zur Klärung mit der betreuenden Person
 
- Sind bestimmte Reglertypen von der betreuenden Person explizit gewünscht oder vorgegeben?
- Gibt es Vorgaben zur Bewertung des Punktesystems (z. B. Formel, Gewichtung)?
- Soll die Nutzersteuerung mit den automatisierten Reglern direkt vergleichbar gemacht werden (z. B. gemeinsames Highscore-System)?
## 6. Meilensteine (Vorschlag)
 
| Meilenstein | Inhalt |
|---|---|
| M1 | Modell mit Standardbibliotheken fertiggestellt und validiert |
| M2 | Tastatursteuerung und Punktesystem funktionsfähig |
| M3 | Mindestens zwei Regler implementiert und verglichen |
| M4 | Schwierigkeitsgrade implementiert |
| M5 | Schriftliche Ausarbeitung fertiggestellt |
| M6 | Präsentation vorbereitet |
 
## 7. Deliverables
 
- GitHub-Repository mit vollständigem, dokumentiertem Code
- Schriftliche Ausarbeitung (ca. 20 Seiten)
- Präsentation (inkl. optionaler Live-Demo)