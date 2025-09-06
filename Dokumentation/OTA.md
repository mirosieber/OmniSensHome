# Doku Zigbee OTA

**Datum:** 04. September 2025  
**Ersteller:** Miro Sieber

## Inhalt:
- neue firmware erstellen und hochladen
- ESP32 seite
- Zigbee2Mqtt seite
- automatisches erstellen des Immages

# Firmware erstellen und hochladen
sobald alle änderungen per USB auf einem Bord getestet wurden und funktionieren, kann nachfolgender Anleitung eine neue Version erstellt werden und per OTA auf alle Boards geladen werden.

1. Im main file Code die OTA_UPGRADE_RUNNING_FILE_VERSION und die OTA_UPGRADE_DOWNLOADED_FILE_VERSION jeh um eins erhöhen (hex). Dies ist wichtig damit das system automatisch erkennt welche Geräte upgedatet werden müssen.
2. Code kompilieren (wichtig dass build file erstellt wird, diese wird als update verwendet. 
3. Testen per USB um sicherzustellen dass der code funktioniert.
4. Alle änderungen auf Git pushen.
5. Git erstellt alle nötigen files und änderungen. überprüfen ob der automatisch startende vorgang erfolgreich abschliesst und ein neuer Versionsrelease erstellt wird.
6. Im Z2M web UI alle Geräte nacheinander updaten. Es soll nie mehr als ein update aufs mal laufen.

## ESP32 seite
Auf dem ESP wird dem TemSensor Endpoint ein OTA client cluster hinzugefügt, denn dieser Endpoint ist bei allen configurationen immer vorhanden. Diesem werden die aktuellen Versionen der Hardware, running File version und downloaded file Version übergeben. Der rest leuft automatisch und im hintergrund der Zigbee Bibliotheke.

## Zigbee2Mqtt seite
Im Z2M web UI gibt es einen Tab OTA. Dort werden alle geräte angezeigt die OTA fähig sind. Um dort zu erscheinen muss in myCustome device ota auf true gesetzt werden.

und im configuration.yaml muss hinzugefügt werden:

` ota: `
`	update_check_interval: 2880 `
`zigbee_ota_override_index_location: https://raw.githubusercontent.com/mirosieber/OmniSensHome/refs/heads/main/my_index.json`

dieses index file muss unter inem öffentilchen link verfügbar sein und enthält informationen über das neuste verfügbare update.

[OTA updates | Zigbee2MQTT](https://www.zigbee2mqtt.io/guide/usage/ota_updates.html#local-ota-index-and-firmware-files)

# automatisches erstellen des Immages

sobald neue daten auf das Git repository gepusht werden, leuft folgender ablauf automatisch auf einem von github bereitgestellten ubuntu server ab:
1. repository wird auf dem server gecloned.
2. phyton und zigpy werden eingerichtet
3. im main file wird nach der Version gesucht und der wert von hex in dec umgewandelt
4. **Projekt wird nicht neu compiliert!** um sicherzustellen, dass sich das update in keinster weise von der per usb getesteten version unterscheidet wird das projekt nicht neu kompiliert, sondern einfach das vom pc hochgeladene build.bin verwendet. Deswegen ist es sehr wichtig das file vor dem push zu kompilieren.
5. Aus dem binary wird mit hilfe ines pyton tool ein .ota file erstellt.
6. ein index file wird erstellt, das informationen wie filename, version, grösse und release link der ota datei enthält.
7. Das neue OTA File wird als neues release unter dem namen der aktuellen Version veröffentlicht.
8. das indexfile wird zurück auf das repository gepusht, so dass es z2m dort finden kann um automatisch zu überprüfen für welche geräte updates verfügbar sind.