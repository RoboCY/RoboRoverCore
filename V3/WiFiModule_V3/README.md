# WiFiModule_V3

The firmware of the rover's WiFi module (ESP-01S). It makes the
`RoboRover-XXXX` WiFi network and serves the phone controller at
`http://192.168.4.1`. It is not a project of its own: the Kypruino side is
[11_WiFiDashboard_V3](../11_WiFiDashboard_V3/) or
[12_UnboxingDemo_V3](../12_UnboxingDemo_V3/), and both use this firmware.

The module comes already programmed. **Reflash it only if it is blank or was
overwritten** - for example when the phone finds no `RoboRover-XXXX` network,
or the page at `192.168.4.1` does not open.

## Reflashing

### 1. Set up the Arduino IDE (once)

1. *File → Preferences → Additional boards manager URLs*: add
   `https://arduino.esp8266.com/stable/package_esp8266com_index.json`
2. *Tools → Board → Boards Manager*: install **esp8266** by ESP8266 Community.
3. *Sketch → Include Library → Manage Libraries*: install **WebSockets** by
   Markus Sattler.

### 2. Board settings

| Setting (*Tools* menu) | Value |
|------------------------|-------|
| Board | Generic ESP8266 Module |
| Flash Size | 1MB (FS:64KB OTA:~470KB) |
| CPU Frequency | 160 MHz |
| Erase Flash | Only Sketch - keeps the WiFi and internet settings saved in the Network tab |

### 3. Connect and upload

1. Put the ESP-01S on a USB programmer for ESP-01 modules, in programming
   mode (GPIO0 held to GND while it powers up - on most programmers a
   PROG / UART switch set to PROG). It runs on 3.3 V only.
2. Choose the programmer's port under *Tools → Port*.
3. Open `WiFiModule_V3.ino` and click **Upload**.
4. Switch the programmer back to UART (or release GPIO0), put the module back
   on the rover and switch the rover on. `RoboRover-XXXX` appears within a few
   seconds.

## Changing the phone page

The page is `web/index.html`. The module serves a packed copy of it, so
after editing run

```
python tools/build_web.py
```

to regenerate `index_html.h`, then upload again. Keep `//` comments on their
own lines in `index.html`: the build removes them.

## Files

| File | What it is |
|------|------------|
| `WiFiModule_V3.ino` | The access point, web server, WebSocket and the serial link to the Kypruino |
| `Internet.ino` | *Dashboard + Internet* mode: joins a home WiFi, MQTT data link, internet remote control |
| `NetConfig.h` | Network settings kept in the module's flash |
| `MiniMqtt.h` | A small MQTT client |
| `web/index.html` | The phone page (source) |
| `index_html.h` | The phone page, packed by `tools/build_web.py` - do not edit |
