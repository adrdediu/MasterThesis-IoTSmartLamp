# Master Thesis-IoT Smart Lamp

IoT Smart Lamp implementation for Master Thesis in Information Technology 4 Telecommunications. This repository contains the embedded code running on a NodeMCU ESP8266, controlling an array of LEDs through shift registers, providing temperature and pressure and providing a RESTful API interface.

## Setup
1. Copy `config.h.example` to `config.h`
2. Edit `config.h` with your credentials:
   - Set `ALLOWED_IP` to your client IP address
   - Set `AUTH_TOKEN` to your secure token
3. Upload the code to your NodeMCU device

## Configuration
The device requires:
- WiFi credentials (configured via WiFiManager)
- Allowed IP address
- Authentication token

## API Endpoints
- `/api/status` - Get device status and sensor readings
- `/api/leds` - Control LED patterns
- `/api/reconfigure` - Reset WiFi settings
- `/api/save_state` - Save current LED state

## Required Libraries
See `libraries.txt` for the complete list of dependencies.
