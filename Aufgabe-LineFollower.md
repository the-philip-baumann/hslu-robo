# Duckiebot als "Line Follower"

Der mobile Roboter Duckiebot soll in diesem Laborversuch auf einer Strasse autonom fahren können. Dazu verwendet er als Sensor seine Kamera, erkennt die Strassenlinie und steuert entsprechend seine Räder an.

Wie ihr die Kamerabilder auslest und die Räder ansteuert, habt ihr im Unterricht kennengelernt. Schaut euch ansonsten nochmals die Python Dateien `camera_subscriber.py`, `wheel_command_publisher.py` oder auch `breitenberg.py` an.

Erstellt eure eigenen Python Scripts im selben `duckie_lab` Paket.

Für die Strasse könnt ihr die schwarzen Schaumstoffmatten benutzen. Einige Elemente haben schon Strassen eingezeichnet. Falls diese nicht ausreichen, könnt ihr weitere Elemente basteln. Eine Anleitung dazu findet ihr auf [duckietown.org](https://docs.duckietown.com/daffy/opmanual-duckietown/intro.html).

## Aufgabe 1: Einer Linie folgen

Euer Duckiebot soll einer Linie, welche auf dem Untergrund aufgezeichnet ist, folgen.

a) Erstellt einen Parcours mit den Schaumstoffmatten ohne Kreuzungen. Die Strassen sind mit zwei weissen Linien und einer gelben, gestrichelten Mittellinie definiert. Wählt eine Linie aus, welcher euer Roboter folgen soll.

b) Wählt eine mittlere Geschwindigkeit für den Duckiebot. Verarbeitet mit einer geeigneten Frequenz die Kamerabilder und steuert damit die Richtung des Roboters.


## Aufgabe 2: Bahngeschwindigkeit während der Bewegung ändern

Der Roboter soll je nach Krümmung der Kurve langsamer oder schneller fahren.

a) Modifiziert die Geschwindigkeit längs des Weges, so dass der Roboter in Bereichen geringer Krümmung schneller fährt als in Bereichen grosser Krümmung.

b) Optimiert euer Programm, so dass der Roboter die Kurven möglichst schnell durchfahren kann.

## Aufgabe 3: Der Strasse folgen

Falls euer Duckiebot bis jetzt die ausgewählte Linie mittig abfährt, soll er jetzt der Spur folgen.

a) Schreibt euer Programm so um, dass der Roboter auf seiner Spur bleibt, d.h., dass die weisse Seitenlinie links ist und die gelbe Mittellinie rechts.

b) Lasst den Roboter den Parcours in beide Drehrichtungen abfahren.


## Aufgabe 4: Flüssige Bewegungen

Betrachtet kritisch eure Lösungen zu den vorhergehenden Aufgaben. Bewegt sich der Roboter flüssig? Falls nicht, optimiert die Bewegungen. Überlegt euch, ob ihr einen Regler dazu verwenden könnt. Als Einstieg kann euch der Wikipedia Artikel über [Regler](https://de.wikipedia.org/wiki/Regler) dienen.


</br>
</br>

---


**Weiterführende Literatur**: Hertzberg et al. Mobile Roboter, Springer Verlag 

**Quelle**: "Thymio als Line Follower", Joachim Wirth, FS22