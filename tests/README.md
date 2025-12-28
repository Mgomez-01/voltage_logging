# Frontend Testing Suite

This directory contains comprehensive tests for the ESP8266 Dual Sensor Logger web interface.

## Test Files

- `test_frontend.py` - Main frontend functionality tests using Playwright
- `test_websocket.py` - WebSocket connection and data flow tests
- `requirements.txt` - Python dependencies for running tests

## Test Coverage

### Basic Functionality (`test_frontend.py`)
- Page loading and initial state
- Button states and interactions
- Data logging controls (start/stop)
- Heater control (relay on/off, emergency stop)
- PID control (enable/disable, parameter updates)
- Temperature setting
- Data display and clearing
- Chart initialization
- Sensor reading updates

### WebSocket Testing (`test_websocket.py`)
- WebSocket connection establishment
- Message parsing and handling
- Fallback to HTTP polling when WebSocket fails
- Data flow through WebSocket messages
- Logging pause/resume affects message processing

## Setup and Installation

1. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Install Playwright browsers:
   ```bash
   playwright install
   ```

## Running Tests

### Run all tests:
```bash
pytest
```

### Run specific test file:
```bash
pytest test_frontend.py
pytest test_websocket.py
```

### Run with verbose output:
```bash
pytest -v
```

### Run specific test class or method:
```bash
pytest test_frontend.py::TestFrontendBasics::test_page_loads
pytest test_websocket.py::TestWebSocketConnection
```

## Test Architecture

### Mock Server
The tests use a built-in HTTP mock server that simulates the ESP8266 API endpoints:
- `/status` - Returns mock sensor data
- `/start`, `/stop` - Logging control
- `/relay/on`, `/relay/off` - Heater control
- `/pid/enable`, `/pid/disable` - PID control
- `/temp/set`, `/pid/params` - Parameter setting
- `/clear` - Data clearing

### WebSocket Mock Server
For WebSocket testing, a separate mock WebSocket server is used that:
- Accepts WebSocket connections on port 8081
- Broadcasts mock sensor data every 500ms
- Handles multiple concurrent connections
- Simulates real-time data flow

### Browser Automation
Tests use Playwright for browser automation:
- Headless Chromium browser for consistent testing
- Page fixtures for isolated test environments
- JavaScript injection for mocking fetch API
- Dialog handling for confirmation prompts

## Test Data

Mock sensor data includes:
- Voltage readings (0-1V range)
- Temperature readings (realistic ranges)
- Heater state (on/off)
- PID control status
- Target temperature settings
- PID output values

## Debugging Tests

### Enable headed browser mode:
Modify the browser fixture in test files:
```python
browser = p.chromium.launch(headless=False)
```

### Add debug output:
Use `page.screenshot()` to capture browser state:
```python
page.screenshot(path="debug.png")
```

### Console logging:
Access browser console logs:
```python
page.on("console", lambda msg: print(f"Console: {msg.text}"))
```

## Expected Test Results

All tests should pass when:
- The HTML file loads correctly
- JavaScript functions work as expected
- Mock server responses are handled properly
- WebSocket connections and fallbacks work
- UI updates reflect backend state changes
- User interactions trigger correct API calls

## Common Issues

1. **Playwright not installed**: Run `playwright install`
2. **Port conflicts**: Mock servers use ports 8080 and 8081
3. **Timing issues**: Increase wait timeouts if tests are flaky
4. **File paths**: Ensure `index.test.html` exists in parent directory

## Extending Tests

To add new tests:
1. Follow existing patterns in test classes
2. Use appropriate fixtures for browser/page setup
3. Mock external dependencies (API calls, WebSocket)
4. Test both success and error scenarios
5. Verify UI state changes after actions