# Arbeitsregeln

Dieses Projekt wird auf mehreren Computern bearbeitet.
Synchronisiere jede Änderungsaufgabe vor Beginn und nach Abschluss
mit dem Git-Remote origin.

## Vor Beginn
- Prüfe mit git status den aktuellen Branch und lokale Änderungen.
- Hole den aktuellen Remote-Stand mit git fetch origin.
- Aktualisiere den Arbeitsbranch bei sauberem Arbeitsverzeichnis
  mit git pull --ff-only.
- Auf master ist origin/master der gemeinsame Stand.
- Überschreibe oder verwerfe keine vorhandenen lokalen Änderungen.
- Falls lokale Änderungen oder auseinanderlaufende Branches die
  Synchronisierung verhindern, sichere die Arbeit und kläre das
  Problem, bevor du weitere Änderungen machst.
- Lies nach der Aktualisierung die AGENTS.md erneut.

## Nach Abschluss
- Führe die für die Änderungen erforderlichen Prüfungen aus.
- Committe alle zur Aufgabe gehörenden Änderungen mit einer
  aussagekräftigen Commit-Nachricht.
- Nimm keine fremden Änderungen, Zugangsdaten oder lokalen
  Build-Dateien in den Commit auf.
- Pushe die Commits auf den entsprechenden Branch bei origin.
  Commit und Push sind hierfür ausdrücklich autorisiert und
  sollen ohne zusätzliche Rückfrage erfolgen.
- Wird der Push wegen neuer Remote-Commits abgelehnt, hole diese
  ab und integriere sie ohne Änderungen zu verlieren.
  Prüfe das Ergebnis und pushe erneut.
- Verwende niemals Force-Push oder git reset --hard.
- Melde abschließend Branch, Commit und Erfolg des Pushs.
  Falls die Synchronisierung scheitert, benenne den Grund klar.
