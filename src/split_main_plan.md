# plan to split up main

- split main into:
  - configuration
  - sensors 
  - heater
  - web stuff
  - file stuff
  - safety
  - debugging stuff
 
## config.h:
  -  For all constants, pin definitions, and configuration parameters. (Should combine with current config file in root)
## sensors.h / sensors.cpp: 
  - Manages sensor reading, multiplexer control, and the SensorReading data structure and buffer.
## heater_control.h / heater_control.cpp: 
  - Handles everything related to the heater, relay, and PID controller.
## web_interface.h / web_interface.cpp: 
  - Manages the ESP8266WebServer, WebSocketsServer, all HTTP route handlers, WebSocket event handling, and serving the web page.
## file_operations.h / file_operations.cpp: 
  - Deals with LittleFS, data file creation, writing, and download.
## safety_system.h / safety_system.cpp: 
  - Contains the hardware timer-based safety checks and watchdog logic.
## debug_utils.h / debug_utils.cpp: 
  - For the printDebugStats function.
## main.cpp: 
  - Will be drastically slimmed down, containing setup() and loop(), orchestrating calls to the other modules.
  
## Globals and shared

Hardware objects like server, webSocket, and Ticker instances will be defined in their respective .cpp files and declared extern in their .h files.
Shared state variables (e.g., dataLoggingEnabled, the readings buffer) will also be homed in the most relevant .cpp file and declared extern in its .h file.

