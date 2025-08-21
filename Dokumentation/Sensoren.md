# Doku Config File

**Datum:** 19. August 2025  
**Ersteller:** Miro Sieber

## Inhalt zu jedem Sensor:

- Typ
- Funktion
- ausleseintervall
- sonstiges


## Illuminanz Sensor
- Typ: OPT3004
- Funktion: Misst die Umgebungshelligkeit
- ausleseintervall: 500 ms
- sonstiges: passt die RGB-LED-Helligkeit an

## Temperatur Sensor
- Typ: MTS4Z
- Funktion: Misst die Temperatur (+- 0.1°C)
- ausleseintervall: 5s
- sonstiges: -

## Temperatur und Luftfeuchtigkeit Sensor
- Typ: AHT21
- Funktion: Misst Temperatur und Luftfeuchtigkeit (+- 0.5°C, +- 3%)
- ausleseintervall: 10s
- sonstiges: läuft im gleichen Task wie Luftqualität Sensor, um kompensationswerte zu übergeben

## Luftqualität Sensor
- Typ: ENS160
- Funktion: Misst die Luftqualität in eCO2, TVOC und Luftqualität- Index
- ausleseintervall: 10s
- sonstiges: interne Heizung die 1 Min vorlauf zeit benötigt

## Geräuschpegel Sensor
- Typ: dB Sensor
- Funktion: Misst den Geräuschpegel
- ausleseintervall: 100 ms
- sonstiges: -

## Präsenz Sensor
- Typ: ld2412
- Funktion: Erkennt Bewegung und Anwesenheit
- ausleseintervall: 100 ms
- sonstiges: -

## Externe Sensoren
- Kontakt Sensoren für Türen und Fenster
- Bluetooth Präsenz Sensor (nicht implementiert V8)

## Aktoren
- Relais 
- RGB LED
- Audio Gong
- Buzzer


## Intruder Alarm
Der Omnisense fungiert auch als Alarmanlage und kann bei unbefugtem Zutritt Alarm schlagen. Der Intruder Alarm kann über Zigbee aktiviert werden, er überwacht die Fensterkontakte und den Präsenzsensor. Bei Alarm wird der Lautsprecher oder der Buzzer aktiviert. Und der Alarm wird an den Coordinator gesendet. Der akustische Alarm kann auch direkt via Zigbee aktiviert werden.

## RGB LED
Die RGB LED zeigt mit 5 Farben an, welche Luftqualität herrscht. Die Farben sind wie folgt zugeordnet:
- Blau: sehr Gute Luftqualität
- Grün: Gute Luftqualität
- Gelb: Mässige Luftqualität
- Rot: Schlechte Luftqualität
- Violett: Sehr schlechte Luftqualität

Die LED-Helligkeit wird automatisch der Umgebungshelligkeit angepasst, sodass sie immer gut sichtbar ist und nachts ausgeschaltet wird.

Die Helligkeit kann via Zigbee feinjustiert werden.