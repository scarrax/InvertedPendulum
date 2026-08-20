# Schriftliche Ausarbeitung -- LaTeX-Vorlage (AP5)

Struktur passend zur empfohlenen Gliederung aus `Projektanweisung_Invertiertes_Pendel.md`
(Abschnitt 3, AP5) und den Formatierungsvorgaben aus `Layout.pdf` (Fachbereich IuM).

## Schreiben & Kompilieren (lokal in VS Code)

Hauptdatei: `main.tex` (pdflatex + biber fuer das Literaturverzeichnis). Primaerer
Schreib-Workflow ist lokal in VS Code:

- Extension **LaTeX Workshop** (James-Yu.latex-workshop) ist installiert; sie kompiliert
  bei jedem Speichern automatisch neu (Standard-Recipe `latexmk`) und aktualisiert die
  PDF-Vorschau.
- Jede Kapiteldatei traegt am Anfang `% !TEX root = ../main.tex` -- dadurch weiss
  LaTeX Workshop auch beim direkten Bearbeiten einer Kapiteldatei, dass `main.tex`
  kompiliert werden muss.
- Alternative ohne die Extension, manuell im Terminal aus `report/`:
  ```
  latexmk -pdf main.tex          # einmalig
  latexmk -pdf -pvc main.tex     # kompiliert automatisch bei jedem Speichern
  ```
- PDF-Anzeige: entweder LaTeX Workshops eigene Vorschau, oder die Extension
  **vscode-pdf** (tomoki1207) fuer `main.pdf` direkt im Editor-Tab.

## Optional: in Overleaf verwenden

Falls doch mal in Overleaf statt lokal geschrieben werden soll (z. B. fuer die
Zusammenarbeit mit der betreuenden Person):

1. **GitHub-Import/-Sync** (benoetigt in der Regel einen bezahlten Overleaf-Plan mit
   GitHub-Integration): In Overleaf "New Project" -> "Import from GitHub" -> dieses
   Repository waehlen. Overleaf importiert das gesamte Repo als ein Projekt (inkl. Code,
   CSV, PDFs); unter den Overleaf-Projekteinstellungen `report/main.tex` als
   "Main document" setzen.

2. **Manueller Upload** (funktioniert immer, auch im Free-Plan): Den Ordner `report/` als
   ZIP herunterladen und in Overleaf ueber "New Project" -> "Upload Project" hochladen.

Lokale Aenderungen und Overleaf synchronisieren sich in keinem der beiden Faelle
automatisch -- nur einer der beiden Wege sollte fuer den eigentlichen Schreibprozess
genutzt werden, nicht parallel.

## Kapitelstruktur

Jede Datei unter `chapters/` beginnt mit einem Kommentar, welches bestehende Projektdokument
(Validierungsberichte, Specs, `protokoll.md`, `CLAUDE.md`) als Quelle fuer den jeweiligen
Abschnitt dient. Fliesstext ist bewusst nicht vorausgefuellt -- die `% TODO`-Kommentare
markieren, was noch zu schreiben ist.

## Layout-Vorgaben

Umgesetzt gemaess `Layout.pdf`: Blocksatz mit automatischer Silbentrennung, 1,5-zeilig,
Raender 2,5/2,5/2 cm, Palatino als Schrift, nummerierte und am Gleichheitszeichen
ausgerichtete Formeln, roemische Seitenzahlen vor Kapitel 1 und arabische ab Kapitel 1,
Kapitelnummerierung nur im Hauptteil (Frontmatter/Backmatter bleiben unnummeriert), lebende
Kolumnentitel.

Seitenlimit laut `MCE_Projektablauf (2).pdf`: ca. 20 Seiten Text (Tabellen, Abbildungen,
Verzeichnisse und Anhang zaehlen nicht dazu). Abgabe bis **11.09.2026** im ILIAS-Ordner
"Abgabe der Ausarbeitungen" und per E-Mail an die betreuende Person.
