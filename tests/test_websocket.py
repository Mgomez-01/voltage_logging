import pytest
# import pytest_asyncio # No longer needed with pytest-anyio
import json
import asyncio
import threading
import time
from pathlib import Path
from playwright.async_api import async_playwright, Page, Browser # async_playwright for async tests
import websockets
try:
    from websockets.asyncio.server import serve
except ImportError:
    from websockets.server import serve
import anyio # For anyio.sleep if needed, though asyncio.sleep might work with asyncio backend

# Configure for pytest-anyio
pytestmark = pytest.mark.anyio

class MockWebSocketServer:
    """Mock WebSocket server for testing WebSocket functionality"""
    
    def __init__(self, port=8081):
        self.port = port
        self.server = None
        self.clients = set()
        self.running = False
        self.broadcast_task = None # To keep track of the broadcast task
        
    async def handler(self, websocket):
        """Handle WebSocket connections"""
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)
    
    async def broadcast_sensor_data(self):
        """Broadcast mock sensor data to all connected clients"""
        try:
            while self.running:
                if self.clients:
                    message = {
                        "type": "reading",
                        "timestamp": int(time.time() * 1000),
                        "voltage": 0.5 + (time.time() % 10) * 0.05,  # Varying voltage
                        "temperature": 20 + (time.time() % 20),       # Varying temperature
                        "heaterState": False,
                        "targetTemp": 25.0,
                        "pidEnabled": False,
                        "pidOutput": 0.0
                    }
                    
                    # Send to all connected clients
                    # Use a copy of the set for iteration if modifying it (though not strictly necessary here)
                    disconnected_clients = set()
                    for client in self.clients:
                        try:
                            await client.send(json.dumps(message))
                        except websockets.exceptions.ConnectionClosed:
                            # Mark client for removal, don't modify set while iterating
                            disconnected_clients.add(client)
                        except Exception: # Catch other potential send errors
                            disconnected_clients.add(client) 
                    
                    if disconnected_clients:
                        self.clients.difference_update(disconnected_clients)
                
                await asyncio.sleep(0.5)  # Send data every 500ms (asyncio.sleep is fine if anyio backend is asyncio)
        except asyncio.CancelledError:
            # Allow task to be cancelled cleanly
            pass
        finally:
            # Cleanup if needed when broadcast stops
            pass

    async def start(self):
        """Start the WebSocket server"""
        self.running = True
        self.server = await serve(self.handler, "localhost", self.port)
        
        # Start broadcasting in the background
        self.broadcast_task = asyncio.create_task(self.broadcast_sensor_data())
        
        return self.server
    
    async def stop(self):
        """Stop the WebSocket server"""
        self.running = False # Signal broadcast_sensor_data to stop
        if self.broadcast_task:
            self.broadcast_task.cancel()
            try:
                await self.broadcast_task
            except asyncio.CancelledError:
                pass # Expected
            self.broadcast_task = None
        
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        # Ensure clients are cleared or connections closed if necessary,
        # though server.close() should handle this.
        for client in list(self.clients): # Iterate over a copy
            try:
                await client.close(reason="Server shutting down")
            except websockets.exceptions.ConnectionClosed:
                pass # Already closed
            except Exception: # Other errors during client close
                pass 
        self.clients.clear()


@pytest.fixture(scope="session") # Changed from @pytest_asyncio.fixture
async def websocket_server():
    """Start a mock WebSocket server for testing"""
    server = MockWebSocketServer(8081) # Using a fixed port
    
    # The server needs to run in an asyncio event loop.
    # Since this fixture is async (run by pytest-anyio), we can start the server directly.
    # However, running it in a separate thread with its own loop is also a valid pattern
    # if we want to isolate it or if the main test loop is busy.
    # For simplicity with pytest-anyio, let's try to manage it within the main async context
    # if possible, or stick to the threaded model if it's more robust.
    # The provided threaded model is fine.

    server_loop = asyncio.new_event_loop()
    
    def run_server_in_thread():
        asyncio.set_event_loop(server_loop)
        try:
            server_loop.run_until_complete(server.start())
            # Keep the server running (serve() does this, task needs to be managed)
            # The broadcast_sensor_data loop is managed by server.running flag
            # The websockets.serve itself runs indefinitely until server.close() is called.
            # We need a way for loop.run_until_complete to not block indefinitely here,
            # or manage the server task properly.
            # A simpler way for the thread:
            async def maintain_server():
                 await server.start() # This starts the websocket server and broadcast task
                 while server.running: # Keep thread alive while server is meant to be running
                     await asyncio.sleep(0.1)
                 # If server.running becomes false externally, this loop ends, leading to thread exit
                 # but server.stop() still needs to be called.

            # Corrected thread target:
            server_loop.run_until_complete(maintain_server())

        finally:
            # Ensure loop closes if an error occurs during run_until_complete
            # server_loop.run_until_complete(server.stop()) # Stop server if thread ends prematurely
            server_loop.close()

    server_thread = threading.Thread(target=run_server_in_thread, daemon=True)
    server_thread.start()
    
    # Give the server time to start
    await asyncio.sleep(1) # asyncio.sleep should be fine with anyio's asyncio backend
    
    if not server.running or not server.server: # Check if server actually started
         pytest.fail("MockWebSocketServer failed to start")

    yield f"ws://localhost:{server.port}" # Use server.port in case it's dynamic in future
    
    # Stop the server
    # server.running = False # This is now handled within server.stop()
    if server_loop.is_running():
        # Schedule stop on the server's own event loop
        async def stop_server_on_its_loop():
            await server.stop()

        future = asyncio.run_coroutine_threadsafe(stop_server_on_its_loop(), server_loop)
        try:
            future.result(timeout=5) # Wait for stop to complete
        except TimeoutError:
            print("Warning: MockWebSocketServer.stop() timed out during teardown.")
        except Exception as e:
            print(f"Warning: Error during MockWebSocketServer.stop(): {e}")

    else: # Loop not running, server might not have started properly or already stopped
        # Attempt direct stop if possible, though it might not be clean
        if server.running: # Check if it thinks it's running
            # This case is tricky, as its loop is not running.
            # For simplicity, assume if loop isn't running, it's already dealt with.
            print("Warning: Server loop not running during teardown, force stopping may be needed or indicate issue.")


    # Ensure thread is joined
    server_thread.join(timeout=5)
    if server_thread.is_alive():
        print("Warning: WebSocket server thread did not exit cleanly.")


@pytest.fixture(scope="session") # Changed from @pytest_asyncio.fixture
async def browser():
    """Create a browser instance for testing"""
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)
        yield browser
        await browser.close()

@pytest.fixture # Changed from @pytest_asyncio.fixture
async def page(browser: Browser):
    """Create a new page for each test"""
    page = await browser.new_page()
    yield page
    await page.close()

@pytest.fixture
def test_html_path():
    """Get path to the test HTML file"""
    return Path(__file__).parent.parent / "index.test.html"

class TestWebSocketConnection:
    """Test WebSocket connection functionality"""
    
    async def test_websocket_connection_attempt(self, page: Page, test_html_path, websocket_server):
        """Test that the page attempts to connect to WebSocket"""
        await page.goto(f"file://{test_html_path.absolute()}")
        
        # Override the WebSocket URL to use our mock server
        # This JS injection logic seems fine.
        await page.evaluate(f"""
            // Override the WebSocket URL construction
            const originalInitWebSocket = window.initWebSocket;
            window.initWebSocket = function() {{
                connectionAttempts++;
                logDebug('WebSocket connection attempt #' + connectionAttempts);
                const wsUrl = '{websocket_server}';
                logDebug('Hostname: localhost, WebSocket URL: ' + wsUrl + ', Page URL: ' + window.location.href);
                try {{
                    ws = new WebSocket(wsUrl);
                    logDebug('WebSocket object created');
                }} catch(e) {{
                    logDebug('Error creating WebSocket: ' + e.message);
                    setTimeout(initWebSocket, 3000); // Original timeout
                    return;
                }}
                
                ws.onopen = function() {{ 
                    logDebug('WebSocket connected successfully!'); 
                    document.getElementById('status').textContent = 'WebSocket Connected (Dual Sensor)'; 
                    document.getElementById('status').className = 'status connected'; 
                }};
                
                ws.onclose = function(event) {{ 
                    logDebug('WebSocket closed - Code: ' + event.code + ', Reason: ' + event.reason + ', Clean: ' + event.wasClean); 
                    document.getElementById('status').textContent = 'Disconnected - Retrying...'; 
                    document.getElementById('status').className = 'status disconnected'; 
                    // setTimeout(initWebSocket, 2000); // Avoid re-triggering in test override
                }};
                
                ws.onerror = function(error) {{ 
                    logDebug('WebSocket error occurred: ' + JSON.stringify(error)); 
                    document.getElementById('status').textContent = 'WebSocket Error - Retrying...'; 
                    document.getElementById('status').className = 'status disconnected'; 
                }};
                
                ws.onmessage = function(event) {{ 
                    lastMessageTime = Date.now(); 
                    logDebug('WebSocket message received: ' + event.data); 
                    if (loggingEnabled) {{ 
                        try {{ 
                            const data = JSON.parse(event.data); 
                            if (data.type === 'reading') addSensorReading(data.timestamp, data.voltage, data.temperature, data); 
                        }} catch(e) {{ logDebug('Error parsing message: ' + e.message); }} 
                    }} 
                }};
            }};
            
            // Restart WebSocket connection with our mock server
            initWebSocket();
        """)
        
        # Wait for connection to establish
        await asyncio.sleep(2) # Give time for WebSocket handshake and onopen
        
        # Check that WebSocket connected
        status_element = page.locator('#status')
        await status_element.wait_for(timeout=3000) # Wait for element to be potentially updated
        status_text = await status_element.text_content()
        assert 'WebSocket Connected' in status_text
        assert 'status connected' in await status_element.get_attribute('class')
    
    async def test_websocket_message_handling(self, page: Page, test_html_path, websocket_server):
        """Test that WebSocket messages are handled correctly"""
        await page.goto(f"file://{test_html_path.absolute()}")
        
        # Connect to our mock WebSocket server (simplified from original test for clarity)
        await page.evaluate(f"""
            window.initWebSocket = function() {{ // Override to prevent retries messing with test
                const wsUrl = '{websocket_server}';
                ws = new WebSocket(wsUrl);
                
                ws.onopen = function() {{
                    document.getElementById('status').textContent = 'WebSocket Connected (Dual Sensor)'; 
                    document.getElementById('status').className = 'status connected';
                }};
                
                ws.onmessage = function(event) {{
                    lastMessageTime = Date.now(); 
                    logDebug('WebSocket message received (test_websocket_message_handling): ' + event.data); 
                    if (loggingEnabled) {{ // loggingEnabled is true by default in index.test.html
                        try {{
                            const data = JSON.parse(event.data); 
                            if (data.type === 'reading') addSensorReading(data.timestamp, data.voltage, data.temperature, data); 
                        }} catch(e) {{ logDebug('Error parsing message: ' + e.message); }} 
                    }} 
                }};
                ws.onerror = (err) => logDebug("WS Error in test: " + JSON.stringify(err));
                ws.onclose = (ev) => logDebug("WS Close in test: " + ev.code);

            }};
            initWebSocket();
        """)
        
        # Wait for WebSocket connection and first message (server sends every 0.5s)
        await asyncio.sleep(3) # Increased wait time
        
        # Check that sensor readings are being updated
        voltage_element = page.locator('#currentVoltage')
        temp_element = page.locator('#currentTemperature')
        sample_count_element = page.locator('#sampleCount')
        
        # Initial values should not be '--' anymore
        await voltage_element.wait_for(lambda el: el.text_content() != '--', timeout=5000)

        voltage_text = await voltage_element.text_content()
        temp_text = await temp_element.text_content()
        sample_count = await sample_count_element.text_content()
        
        assert voltage_text != '--'
        assert temp_text != '--'
        assert int(sample_count) > 0, f"Sample count was {sample_count}, expected > 0"
        
        # Wait for more messages and verify updates
        initial_sample_count = int(sample_count)
        await asyncio.sleep(2) # Server sends every 0.5s, so expect ~4 more messages
        
        new_sample_count = int(await sample_count_element.text_content())
        assert new_sample_count > initial_sample_count, f"New sample count {new_sample_count} not > initial {initial_sample_count}"
    
    async def test_websocket_fallback_to_polling(self, page: Page, test_html_path):
        """Test fallback to HTTP polling when WebSocket fails"""
        await page.goto(f"file://{test_html_path.absolute()}")
        
        # Force WebSocket connection to fail by using invalid URL
        # This JS logic seems fine.
        await page.evaluate("""
            maxRetries = 2; // Reduce retries for faster testing
            // connectionAttempts = 2; // Let initWebSocket increment it. Start from 0.
            connectionAttempts = 0;
            
            // Override initWebSocket to simulate failure
            const originalInitWebSocket = window.initWebSocket;
            window.initWebSocket = function() {
                if (connectionAttempts >= maxRetries) {
                    logDebug('Max WebSocket connection attempts reached, switching to polling mode');
                    document.getElementById('status').textContent = 'Using HTTP Polling (WebSocket failed)';
                    document.getElementById('status').className = 'status connected'; // Polling is also 'connected' state
                    startPolling(); // Original startPolling makes actual HTTP requests
                    return;
                }
                // Simulate failed connection by not actually connecting or calling ws.onopen
                connectionAttempts++;
                logDebug('Simulating WS fail, attempt: ' + connectionAttempts);
                // Call onclose/onerror to trigger retry/fallback logic from main page script
                if (ws && ws.onclose) { ws.onclose({code: 1006, reason: "Test fail", wasClean: false}); }
                else { setTimeout(initWebSocket, 100); } // If ws not created, retry via timeout
            };
            
            // Override startPolling to use mock data instead of actual HTTP requests
            // because we don't have the mock_server fixture here (it's sync).
            // If we needed real polling, this test would need a running HTTP server.
            window.startPolling = function() {
                logDebug('Starting HTTP polling mode (mocked for test)...');
                setInterval(function() {
                    // Mock polling response
                    const mockData = {
                        type: "reading", // Ensure type is included if addSensorReading expects it via JSON.parse
                        voltage: 0.6234,
                        temperature: 24.56,
                        timestamp: Date.now(),
                        heaterState: false,
                        targetTemp: 25.0,
                        pidEnabled: false,
                        pidOutput: 0.0
                    };
                    // Simulate the structure of data as if it came from server/JSON
                    addSensorReading(mockData.timestamp, mockData.voltage, mockData.temperature, mockData);
                }, 500); // Poll every 0.5s
            };
            
            // Trigger WebSocket initialization
            initWebSocket();
        """)
        
        # Wait for fallback to polling
        await asyncio.sleep(2) # Allow time for retries and switch to polling
        
        # Check that status shows polling mode
        status_element = page.locator('#status')
        await status_element.wait_for(timeout=3000)
        status_text = await status_element.text_content()

        assert 'HTTP Polling' in status_text
        assert 'WebSocket failed' in status_text
        
        # Wait a bit more and check that data is still being updated via polling
        sample_count_locator = page.locator('#sampleCount')
        initial_sample_count_text = await sample_count_locator.text_content()
        # Ensure some polling has occurred. If initial count is 0, wait for it to be > 0
        if initial_sample_count_text == '0':
             await sample_count_locator.wait_for(lambda el: el.text_content() != '0', timeout=3000)
        
        initial_sample_count = int(await sample_count_locator.text_content())
        assert initial_sample_count > 0, "Polling did not produce samples"

        await asyncio.sleep(1.5) # Wait for more polling updates
        
        voltage_text = await page.locator('#currentVoltage').text_content()
        temp_text = await page.locator('#currentTemperature').text_content()
        new_sample_count = int(await sample_count_locator.text_content())
        
        assert voltage_text != '--'
        assert temp_text != '--'
        assert new_sample_count > initial_sample_count, "Polling did not update sample count"


class TestWebSocketDataFlow:
    """Test data flow through WebSocket"""
    
    async def test_sensor_data_parsing(self, page: Page, test_html_path):
        """Test parsing of sensor data from WebSocket messages"""
        await page.goto(f"file://{test_html_path.absolute()}")
        
        # Simulate WebSocket message with specific data
        test_data = {
            "type": "reading",
            "timestamp": 1234567890123,
            "voltage": 0.7891,
            "temperature": 26.89,
            "heaterState": True,
            "targetTemp": 30.0,
            "pidEnabled": True,
            "pidOutput": 67.3
        }
        
        # Ensure logging is enabled for message processing (it is by default)
        await page.evaluate(f"""
            // Simulate receiving a WebSocket message via the ws.onmessage handler structure
            const mockEvent = {{
                data: '{json.dumps(test_data)}'
            }};
            
            // Manually trigger the equivalent of ws.onmessage's core logic
            // This assumes the global 'loggingEnabled' variable and 'addSensorReading' are accessible
            lastMessageTime = Date.now(); 
            logDebug('Simulated WebSocket message received: ' + mockEvent.data); 
            if (window.loggingEnabled) {{ // Access through window scope if needed
                try {{ 
                    const data = JSON.parse(mockEvent.data); 
                    if (data.type === 'reading') window.addSensorReading(data.timestamp, data.voltage, data.temperature, data); 
                }} catch(e) {{ logDebug('Error parsing message in test_sensor_data_parsing: ' + e.message); }} 
            }}
        """)
        
        # Verify that the data was parsed and displayed correctly
        await asyncio.sleep(0.1) # Brief pause for DOM updates

        assert await page.locator('#currentVoltage').text_content() == '0.7891'
        assert await page.locator('#currentTemperature').text_content() == '26.89'
        assert await page.locator('#currentTemp').text_content() == '26.89' # currentTemp is another display for temperature
        assert await page.locator('#relayStatus').text_content() == 'ON'
        assert await page.locator('#pidStatus').text_content() == 'ENABLED'
        # The HTML formats pidOutput to one decimal place: pidOutput.toFixed(1)
        assert await page.locator('#pidOutputValue').text_content() == '67.3' 
        assert await page.locator('#sampleCount').text_content() == '1'
    
    async def test_logging_pause_affects_websocket(self, page: Page, test_html_path):
        """Test that pausing logging affects WebSocket message processing"""
        await page.goto(f"file://{test_html_path.absolute()}")
        
        # Helper to simulate message processing (mimicking ws.onmessage)
        async def send_mock_message(p: Page, data_dict):
            await p.evaluate(f"""
                const eventData = '{json.dumps(data_dict)}';
                if (window.loggingEnabled) {{
                    try {{
                        const parsedData = JSON.parse(eventData);
                        if (parsedData.type === 'reading') {{
                            window.addSensorReading(parsedData.timestamp, parsedData.voltage, parsedData.temperature, parsedData);
                        }}
                    }} catch (e) {{
                        logDebug('Error in send_mock_message: ' + e.message);
                    }}
                }}
            """)
            await asyncio.sleep(0.1) # DOM update time

        # First, send a message while logging is enabled (default)
        await send_mock_message(page, {
            "type": "reading", "timestamp": Date.now(), "voltage": 0.1111, "temperature": 11.11,
            "heaterState": False, "targetTemp": 25.0, "pidEnabled": False, "pidOutput": 0.0
        })
        
        initial_sample_count = int(await page.locator('#sampleCount').text_content())
        assert initial_sample_count == 1, "Sample count should be 1 after first message"
        
        # Pause logging by clicking the toggle button
        await page.locator('#logToggle').click() # This calls toggleLogging()
        await asyncio.sleep(0.1) # DOM update time

        # Verify logging is paused
        log_toggle_btn = page.locator('#logToggle')
        assert 'Resume Logging' in await log_toggle_btn.text_content()
        assert await page.evaluate("window.loggingEnabled") is False # Check JS variable state
        
        # Send another message while logging is paused
        await send_mock_message(page, {
            "type": "reading", "timestamp": Date.now(), "voltage": 0.2222, "temperature": 22.22,
            "heaterState": False, "targetTemp": 25.0, "pidEnabled": False, "pidOutput": 0.0
        })
        
        # Sample count should not have increased
        paused_sample_count = int(await page.locator('#sampleCount').text_content())
        assert paused_sample_count == initial_sample_count, "Sample count should not change when logging paused"
        assert await page.locator('#currentVoltage').text_content() == '0.1111', "Voltage should not update when paused"
        
        # Resume logging
        await page.locator('#logToggle').click() # Calls toggleLogging() again
        await asyncio.sleep(0.1)
        assert await page.evaluate("window.loggingEnabled") is True # Check JS variable state
        
        # Send another message
        await send_mock_message(page, {
            "type": "reading", "timestamp": Date.now(), "voltage": 0.3333, "temperature": 33.33,
            "heaterState": False, "targetTemp": 25.0, "pidEnabled": False, "pidOutput": 0.0
        })
        
        # Sample count should have increased
        resumed_sample_count = int(await page.locator('#sampleCount').text_content())
        assert resumed_sample_count > paused_sample_count, "Sample count should increase after resuming"
        assert resumed_sample_count == initial_sample_count + 1
        assert await page.locator('#currentVoltage').text_content() == '0.3333', "Voltage should update after resuming"
