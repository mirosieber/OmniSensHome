# Doku Softwarestruktur

**Datum:** 19. August 2025  
**Ersteller:** Miro Sieber

## Inhalt

- Task Übersicht
- Endpoint Übersicht

## Architektur

Die Software basiert auf dem Multitasking-Betriebssystem FreeRTOS und ist in mehrere Aufgaben (Tasks) unterteilt, die parallel ausgeführt werden. Die Tasks werden im Mainprogramm erstellt und laufen anschliessend unabhängig voneinander. Jenachdem welche Sensoren in der Config Datei aktiviert wurden, werden mer oder weniger Tasks erstellt. Hier eine übersicht über alle Tasks:

- MainTask:
  - Initialisiert das System und startet die Tasks.
  - Läuft immer, solange das Gerät eingeschaltet ist.
  - Überwacht den Zigbee-Factory reset Button.

- ZigbeeMain:
  - Hintergrundtask der Zigbee Bibliotheke.
  - Zigbee-Kommunikation.
  - OTA-Updates.
  - Routing.
  - ...

Folgende Tasks laufen nur wenn der entsprechende Sensor aktiviert ist:

- Lux Sensor update Task:
  - Initialisiert den OPT3004 Sensor.
  - Liest die Umgebungshelligkeit aus dem OPT3004 Sensor aus.
  - Übergibt die Daten an den Endpoint 2.
  - Passt die RGB-LED-Helligkeit anhand der Umgebungshelligkeit an.

- Temperatur Sensor update Task:
  - Initialisiert den MTS4Z Sensor.
  - Liest die Temperatur aus dem MTS4Z Sensor aus.
  - Übergibt die Daten an den Endpoint 3.

- Temperatur Luftfeuchtigkeit und Luftqualität Sensor update Task:
  - Initialisiert den ENS160 Sensor.
  - Initialisiert den AHT21 Sensor.
  - Liest die Temperatur und Luftfeuchtigkeit aus dem AHT21 Sensor aus.
  - Liest die Luftqualität aus dem ENS160 Sensor aus.
  - Übergibt die Temperatur- und Luftfeuchtigkeitsdaten an den Endpoint 4.
  - Übergibt den eCO2-Wert an den Endpoint 5, den TVOC-Wert an den Endpoint 16 und den Luftqualitätswert an den Endpoint 22.
  - Passt die RGB-LED-Farbe anhand der Luftqualität an.

- Geräuschpegel Sensor update Task:
  - Initialisiert den dB Sensor.
  - Liest den Geräuschpegel aus dem dB Sensor aus.
  - Übergibt die Daten an den Endpoint 6.

- Präsenz Sensor update Task:
  - Initialisiert den ld2412 Sensor.
  - Liest die Daten aus dem ld2412 Sensor aus.
  - Übergibt die Daten an den Endpoint 7.

- Für die UART kommunikation hat der Präsenzsensor drei zusätzliche Tasks:
  - UART event Task:
    - Wartet auf ein UART Ereignis.
    - Liest die Daten vom UART aus.
  - UART transmit Task:
    - Wartet auf ein UART Übertragungsereignis.
    - Sendet die Daten an den ld2412 Sensor.
 - UART receive Task:
   - Wartet auf ein UART Empfangsereignis.
   - Liest die Daten vom UART aus.

- contact Switch update Task:
  - Liest den Status der contact Switches aus.
  - Übergibt die Daten an die Endpoints 8 - 11.

- Relay control Task:
  - überprüft wie viele Relais, Switches und Buttons aktiv sind und ordnet diese einander der reihenach zu (wenn Button 2 aktiv und Relays 4, werden diese einander zugeordnet)
  - handelt sowohl Schalter als auch Taster als einfache Toggle-Impulse
  - Übergibt die Relays zustände an die Endpoints 12-15.

- Speaker Task:
  - Wartet bis der Intruder Alarm ausgelöst wird.
  - Aktiviert den Speaker und spielt den Alarmton (.wav) ab.

- Buzzer Task:
  - Wartet bis der Intruder Alarm ausgelöst wird.
  - Aktiviert den Buzzer und spielt den Alarmton (hard coded) ab.

## Endpoints

- Endpoint 1: Range Extender (weiterleiten von nachrichten an andere Sensoren)
- Endpoint 2: Lux Sensor
- Endpoint 3: Temperatur Sensor OTA update
- Endpoint 4: Temperatur und Luftfeuchtigkeit Sensor
- Endpoint 5: eCO2 Sensor
- Endpoint 6: Geräuschpegel (dB) Sensor
- Endpoint 7: Präsenz Sensor
- Endpoint 8: Contact Switch 1
- Endpoint 9: Contact Switch 2
- Endpoint 10: Contact Switch 3
- Endpoint 11: Contact Switch 4
- Endpoint 12: Relay 1
- Endpoint 13: Relay 2
- Endpoint 14: Relay 3
- Endpoint 15: Relay 4
- Endpoint 16: TVOC Sensor
- Endpoint 17: RGB LED
- Endpoint 18: Bluetooth aktivieren vom Präsenz Sensor
- Endpoint 19: Audio Gong Trigger
- Endpoint 20: Intruder Alarm Aktivieren
- Endpoint 21: Intruder erkannt rückmeldung
- Endpoint 22: Luftqualitätswert
- Endpoint 23: Reset Zigbee Device