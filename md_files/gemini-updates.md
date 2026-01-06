# Gemini Updates: SD Card Logging Implementation

This document outlines the changes made to the project to implement SD card logging functionality.

## Summary of Changes

The project has been updated to save sensor data to an SD card instead of the internal `LittleFS`. This prevents the internal storage from filling up and allows for easy retrieval of log files. The web interface has also been updated to list and download the log files from the SD card.

## File Changes

### `src/data_manager.h`

*   Included the `<SD.h>` library for SD card support.
*   Added function declarations for `initializeSDCard()` and `getNewLogFileName()`.
*   Added a `logFileName` character array to store the name of the current log file.

### `src/data_manager.cpp`

*   Implemented `initializeSDCard()` to initialize the SD card.
*   Implemented `getNewLogFileName()` to generate a new log file name (e.g., `data.log`, `data_1.log`, etc.) if a file with the same name already exists.
*   Modified `initializeDataFile()` to initialize the SD card and create a new log file with a header.
*   Modified `writeBufferToFile()` to write sensor data to the log file on the SD card.
*   Modified `clearDataFile()` to remove all `data_*.log` files from the SD card.

### `src/web_interface.h`

*   Added a function declaration for `handleListLogs()`.
*   Added a function declaration for `handleDeleteLog()`.

### `src/web_interface.cpp`

*   Added a new route `/logs` in `setupWebServer()` to handle listing log files.
*   Added a new route `/logs/delete` in `setupWebServer()` to handle deleting log files.
*   Implemented `handleListLogs()` to send a JSON array of log file names and sizes from the SD card.
*   Implemented `handleDeleteLog()` to delete a specific log file from the SD card.
*   Modified `handleDataDownload()` to accept a `file` query parameter and send the requested log file from the SD card.
*   Added error handling to `handleDataDownload()`, `handleClearData()`, and `handleListLogs()` to check for the presence of the SD card.

### `data/index.template.html`

*   Added a new "Log Files on SD Card" section to the web page.
*   This section includes a table to display the log file names, sizes, and a "Download" and "Delete" button for each file.
*   A "Refresh List" button was added to manually refresh the list of log files.
*   The `listLogFiles()` JavaScript function was added to fetch the list of log files from the `/logs` endpoint and populate the table.
*   The `deleteLogFile()` JavaScript function was added to send a delete request to the server.
*   The `window.onload` and `clearData()` functions were updated to call `listLogFiles()` to ensure the list is always up-to-date.
*   Added error handling to display a message if the SD card is not found.

## Hardware Requirements

An SD card module must be connected to the ESP8266 using the following SPI pins:

| SD Card Pin | ESP8266 Pin | NodeMCU Label |
| :--- | :--- | :--- |
| **CS** | GPIO15 | D8 |
| **MOSI** | GPIO13 | D7 |
| **MISO** | GPIO12 | D6 |
| **SCK** | GPIO14 | D5 |
| **VCC** | 3.3V | 3V3 |
| **GND** | GND | GND |

## Implemented Suggestions

*   **Error Handling:** Added more robust error handling for SD card operations. The web interface will now display an error message if the SD card is not found or fails to initialize.
*   **File Deletion:** Implemented the ability to delete individual log files from the web interface.
*   **File Size:** The size of each log file is now displayed in the web interface.

## Future Suggestions

*   **Pagination:** If a large number of log files are expected, consider adding pagination to the log file list.
*   **Configuration:** Allow the user to configure the logging interval and other parameters from the web interface.