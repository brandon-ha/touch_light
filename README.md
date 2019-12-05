
# touch_light_nodemcu
Short demo:
[![Alt text](https://imgur.com/1dLWVOE.jpg)](https://youtu.be/Ediuyl3Jnzo)
## What is touch_light_nodemcu?
Heavily modified implementation of [pblesi's touch_light Particle project](https://github.com/pblesi/touch_light). The purpose of the original project is to synchronize WiFi-connected touch lights as a way to help connect individuals together from across the globe. I decided to dedicate my time into this because the NodeMCU can be found for significantly less than the Particle Photon used in pblesi's project (~$5 vs $20). Given the lower cost, I felt that this solution would encourage more people to experiment with touch lights.

### What's different?
- Rewritten for NodeMCU v2 Arduino microcontrollers (versus the much more expensive Particle Photon)
- Uses MQTT servers to connect to other lights remotely (versus the Particle servers)
- WiFi connection portal with MQTT configuration fields
- New color cycle LED animation 
- Tweaked sensitivity values
- Flash storage system implemented (used to store WiFi credentials and MQTT information)

### Flashing
Open sketch in Arduino IDE, install ESP8266 libraries under Board Manager, compile, write to Arduino.
