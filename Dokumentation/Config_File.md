# Doku Config File

**Datum:** 19. August 2025  
**Ersteller:** Miro Sieber

## Inhalt

- wiso ein Configfile
- was wird im Configfile definiert
- wie wird das Configfile hochgeladen

## Vorteile des Configfiles
Durch das verwenden eines Configfiles wird es möglich auf alle OmniSense Geräte die gleiche Software zu laden. Im Configfile wird definiert, welche Sensoren bestückt sind und welche externen Komponenten verwendet werden. Das Configfile muss nur geändert werden, wenn sich an der Hardware etwas ändert. Es ist nur per USB möglich dieses File zu ändern.

## Das Configfile

Das Configfile ist eine zentrale Datei, die alle relevanten Informationen über die Hardwarekonfiguration eines OmniSense Geräts enthält. Es wird im JSON-Format erstellt und definiert die verwendeten Sensoren, Aktoren und deren Pinbelegung. Das Configfile ermöglicht eine einfache Anpassung der Software an verschiedene Hardwarekonfigurationen, ohne dass der Quellcode geändert werden muss. Die Struktur des Configfiles darf nicht angepasst werden, da sie sonst nicht mehr gelesen werden kann. Sie umfasst folgende Abschnitte:

- Device:

Hier wird definiert ob das gerät als "Router" oder als "Enddevice" arbeiten soll.

Router -> leitet nachrichten andere Geräte weiter, immer online, geeignet für Netzversorgte Geräte

Enddevice -> sendet Daten an den Router, kann in den Energiesparmodus gehen, geeignet für batteriebetriebene Geräte

lediglich informativ:
PowerSupply 
Description
HardwareVersion
SoftwareVersion
SerialNumber

werden auf zigbee2mqtt angezeigt:
Manufacturer
Modell

- FactoryResetButton
ermöglicht das zurücksetzen der Zigbee verbindung via Boot Knopf an hier definertem pin

Folgende Funktionen können bei bedarf aktiviert werden und die jeweiligen spezifischen einstellungen angepasst werden:
- WiFi
- RGB Led
- I2C schnittstelle (kann nicht deaktiviert werden)
- Speaker (verwendet zusammen mit dem Mikrofon die I2S Schnittstelle zwei der GPIO Pins sind gemeinsam)
- Buzzer
- Mikrofon(nicht implementiert (V8)) (um Sprache zu erkennen um lautstärke zu erkennen dB Sensor verwenden)
- Display (nicht implementiert (V8))

Die Sensoren sind in einem Array definiert und können bei Bedarf aktiviert oder deaktiviert werden. Die genaue Pinbelegung und Konfiguration der Sensoren erfolgt ebenfalls im Configfile.

Schalteingänge:

Die Schalteingänge sind ebenfalls in einem Array definiert. Es sind maximal 4 Schalteingänge möglich. Anhand des Namens wird der Typ des Schalteingangs bestimmt (möglich sind "Button", "Contact", "Switch").

Button -> Ein Taster, der bei Betätigung ein impuls Signal sendet.

Switch -> Ein Schalter, der bei Betätigung ein kontinuierliches Signal sendet. (zustand einer Lampe ändert bei jedem betätigen)

Contact -> Ein Kontakt, zur überwachung von Türen und Fenstern. Pullup aktiviert, muss auf GND gezogen werden wenn das Fenster geschlossen ist.

Relays: 

Die Relais sind in einem Array definiert und können bei Bedarf aktiviert oder deaktiviert werden. Es sind maximal 4 Relais möglich. 


## **Wichtig!**
Die Externen GPIO-Pins die für z.B. für Relays, Schalter, Buzzer oder Lautsprecher verwendet werden sind begrenzt, es sind Lediglich Pin 2, 3, 14 und 15 verfügbar. Es muss sichergestellt werden dass kein Pin doppelt belegt wird!

Weiteres: 
- Falls Komponenten wie z.B. das Mikrofon nicht verwendet werden, können die entsprechenden Pins für andere Funktionen genutzt werden.
- Pin 1 ist für den Interrupt des ENS160 vorgesehen, wird aber in der software nicht verwendet und kann deswegen anderweitig eingesetzt werden.

## Hochladen des Configfiles
Um das Configfile auf ein OmniSense Gerät hochzuladen, muss zunächst eine Verbindung über USB hergestellt werden. Anschliessend kann das Configfile mithilfe der [ESP32DataFlasher](https://github.com/mirosieber/ESP32DataFlasher) Software auf das Gerät übertragen werden. Es ist wichtig sicherzustellen, dass das Configfile im richtigen Format vorliegt und alle erforderlichen Informationen enthält, bevor der Upload gestartet wird. Falsche oder Doppelte Pinbelegungen können zu Problemen, Kurzschlüssen oder defekten Komponenten oder Pins führen!

Maximale Dateigrösse und Offset müssen mit der Partitionstabelle übereinstimmen.(Defaultwerte passen)