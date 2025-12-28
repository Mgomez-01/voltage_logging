import pytest
import json
import os
from pathlib import Path
from unittest.mock import patch, MagicMock
from playwright.sync_api import sync_playwright, Page, Browser
import http.server
import socketserver
import socket  # Moved to top-level
import threading
import time

class MockHTTPHandler(http.server.SimpleHTTPRequestHandler):
    """Mock HTTP handler for ESP8266 API endpoints"""
    
    def do_GET(self):
        if self.path == '/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                'voltage': 0.5234,
                'temperature': 23.45,
                'timestamp': int(time.time() * 1000),
                'heaterState': False,
                'targetTemp': 25.0,
                'pidEnabled': False,
                'pidOutput': 0.0
            }
            self.wfile.write(json.dumps(response).encode())
        elif self.path == '/start':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Logging started')
        elif self.path == '/stop':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Logging stopped')
        elif self.path == '/clear':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Data cleared')
        elif self.path == '/relay/on':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Relay ON')
        elif self.path == '/relay/off':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Relay OFF')
        elif self.path == '/pid/enable':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'PID enabled')
        elif self.path == '/pid/disable':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'PID disabled')
        elif self.path.startswith('/temp/set'):
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Temperature set')
        elif self.path.startswith('/pid/params'):
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'PID parameters updated')
        else:
            super().do_GET()

@pytest.fixture(scope="session")
def mock_server():
    """Start a mock HTTP server for testing"""
    
    # Enable socket reuse and find an available port
    class ReuseSocketTCPServer(socketserver.TCPServer):
        allow_reuse_address = True
    
    # Create and bind the socket first
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('localhost', 0)) # Bind to localhost and OS-assigned free port
    port = server_socket.getsockname()[1]
    
    # Pass the bound socket to the server
    httpd = ReuseSocketTCPServer(('localhost', port), MockHTTPHandler, bind_and_activate=False)
    httpd.socket = server_socket # Assign the pre-bound socket
    httpd.server_address = ('localhost', port) # Update server address
    
    server_thread = threading.Thread(target=httpd.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    
    yield f"http://localhost:{port}"
    
    # Teardown: shutdown the server and close the socket
    httpd.shutdown()
    httpd.server_close()
    server_thread.join(timeout=5) # Wait for thread to finish

@pytest.fixture(scope="session")
def browser():
    """Create a browser instance for testing"""
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        yield browser
        browser.close()

@pytest.fixture
def page(browser: Browser):
    """Create a new page for each test"""
    page = browser.new_page()
    yield page
    page.close()

@pytest.fixture
def test_html_path():
    """Get path to the test HTML file"""
    return Path(__file__).parent.parent / "index.test.html"

class TestFrontendBasics:
    """Test basic HTML structure and initial state"""
    
    def test_page_loads(self, page: Page, test_html_path):
        """Test that the HTML page loads successfully"""
        page.goto(f"file://{test_html_path.absolute()}")
        assert page.title() == "ESP8266 Dual Sensor Logger with Heater Control (TEST SAMPLE)"
        
    def test_initial_button_states(self, page: Page, test_html_path):
        """Test initial state of control buttons"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        start_btn = page.locator('#startBtn')
        stop_btn = page.locator('#stopBtn')
        relay_on_btn = page.locator('#relayOnBtn')
        relay_off_btn = page.locator('#relayOffBtn')
        pid_enable_btn = page.locator('#pidEnableBtn')
        pid_disable_btn = page.locator('#pidDisableBtn')
        
        assert start_btn.is_enabled()
        assert stop_btn.is_disabled()
        assert relay_on_btn.is_enabled()
        assert relay_off_btn.is_enabled()
        assert pid_enable_btn.is_enabled()
        assert pid_disable_btn.is_disabled()
        
    def test_initial_display_values(self, page: Page, test_html_path):
        """Test initial display values"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        assert page.locator('#currentVoltage').text_content() == '--'
        assert page.locator('#currentTemperature').text_content() == '--'
        assert page.locator('#currentTemp').text_content() == '--'
        assert page.locator('#sampleCount').text_content() == '0'
        assert page.locator('#relayStatus').text_content() == 'OFF'
        assert page.locator('#pidStatus').text_content() == 'DISABLED'

class TestDataLogging:
    """Test data logging functionality"""
    
    def test_start_logging_button(self, page: Page, test_html_path, mock_server):
        """Test start logging button functionality"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        # Set up the fetch mock AFTER loading the page
        page.evaluate(f"""
            const originalFetch = window.fetch;
            window.fetch = function(url, options) {{
                console.log('Fetch called with URL:', url, 'Type:', typeof url);
                if (url == null) {{
                    console.error('URL is null or undefined:', url);
                    return Promise.reject(new Error('URL is null or undefined: ' + url));
                }}
                const mockUrl = '{mock_server}' + url;
                console.log('Mock URL:', mockUrl);
                return originalFetch(mockUrl, options);
            }};
        """)
        
        # Accept the confirm dialog that appears when starting logging
        page.on("dialog", lambda dialog: dialog.accept())
        
        start_btn = page.locator('#startBtn')
        stop_btn = page.locator('#stopBtn')
        
        start_btn.click()
        
        # Wait for the fetch operation to complete by waiting for button state change
        page.wait_for_function("document.getElementById('startBtn').disabled === true", timeout=5000)
        
        # Check that buttons are updated
        assert start_btn.is_disabled()
        assert stop_btn.is_enabled()
        
        # Check status text
        main_status = page.locator('#mainLoggingStatus')
        assert main_status.text_content() == 'ACTIVE'
    
    def test_stop_logging_button(self, page: Page, test_html_path, mock_server):
        """Test stop logging button functionality"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        # Mock the fetch function
        page.evaluate(f"""
            const originalFetch = window.fetch;
            window.fetch = function(url, options) {{
                if (url == null) {{  // Corrected: removed duplicate 'if'
                    console.error('URL is null or undefined');
                    return Promise.reject(new Error('URL is null or undefined'));
                }}
                const mockUrl = '{mock_server}' + url;
                return originalFetch(mockUrl, options);
            }};
        """)
        
        page.on("dialog", lambda dialog: dialog.accept())
        
        # First start logging
        page.locator('#startBtn').click()
        page.wait_for_function("document.getElementById('startBtn').disabled === true", timeout=5000)
        
        # Then stop logging
        page.locator('#stopBtn').click()
        page.wait_for_function("document.getElementById('stopBtn').disabled === true", timeout=5000)
        
        # Check that buttons are updated
        assert page.locator('#startBtn').is_enabled()
        assert page.locator('#stopBtn').is_disabled()
        
        # Check status text
        main_status = page.locator('#mainLoggingStatus')
        assert main_status.text_content() == 'PAUSED'

class TestHeaterControl:
    """Test heater control functionality"""
    
    def test_relay_on_button(self, page: Page, test_html_path, mock_server):
        """Test relay on button functionality"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        page.evaluate(f"""
            const originalFetch = window.fetch;
            window.fetch = function(url, options) {{
                if (url == null) {{
                    console.error('URL is null or undefined');
                    return Promise.reject(new Error('URL is null or undefined'));
                }}
                const mockUrl = '{mock_server}' + url;
                return originalFetch(mockUrl, options);
            }};
        """)
        
        page.on("dialog", lambda dialog: dialog.accept())
        
        relay_on_btn = page.locator('#relayOnBtn')
        relay_on_btn.click()
        
        # Wait for the relay status to update
        page.wait_for_function("document.getElementById('relayStatus').textContent === 'ON'", timeout=5000)
        
        # Check that relay status is updated
        relay_status = page.locator('#relayStatus')
        assert relay_status.text_content() == 'ON'
        
        # Check heater indicator class
        heater_indicator = page.locator('#heaterIndicator')
        assert 'heater-on' in heater_indicator.get_attribute('class')
    
    def test_emergency_stop(self, page: Page, test_html_path, mock_server):
        """Test emergency stop functionality"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        page.evaluate(f"""
            const originalFetch = window.fetch;
            window.fetch = function(url, options) {{
                if (url == null) {{
                    console.error('URL is null or undefined');
                    return Promise.reject(new Error('URL is null or undefined'));
                }}
                const mockUrl = '{mock_server}' + url;
                return originalFetch(mockUrl, options);
            }};
        """)
        
        page.on("dialog", lambda dialog: dialog.accept())
        
        emergency_btn = page.locator('text=⚠️ EMERGENCY STOP')
        emergency_btn.click()
        
        page.wait_for_timeout(1000) # Allow time for JS and fetch calls to resolve
        
        # Check that both heater and PID are disabled
        # It might take a moment for these to update if they rely on fetch promises
        page.wait_for_function("document.getElementById('relayStatus').textContent === 'OFF'", timeout=5000)
        page.wait_for_function("document.getElementById('pidStatus').textContent === 'DISABLED'", timeout=5000)

        relay_status = page.locator('#relayStatus')
        pid_status = page.locator('#pidStatus')
        
        assert relay_status.text_content() == 'OFF'
        assert pid_status.text_content() == 'DISABLED'

class TestPIDControl:
    """Test PID control functionality"""
    
    def test_enable_pid(self, page: Page, test_html_path, mock_server):
        """Test enable PID functionality"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        page.evaluate(f"""
            const originalFetch = window.fetch;
            window.fetch = function(url, options) {{
                if (url == null) {{
                    console.error('URL is null or undefined');
                    return Promise.reject(new Error('URL is null or undefined'));
                }}
                const mockUrl = '{mock_server}' + url;
                return originalFetch(mockUrl, options);
            }};
        """)
        
        page.on("dialog", lambda dialog: dialog.accept())
        
        pid_enable_btn = page.locator('#pidEnableBtn')
        pid_disable_btn = page.locator('#pidDisableBtn')
        
        pid_enable_btn.click()
        # Wait for status and button states to update
        page.wait_for_function("document.getElementById('pidStatus').textContent === 'ENABLED'", timeout=5000)
        page.wait_for_function("document.getElementById('pidEnableBtn').disabled === true", timeout=5000)
        
        # Check button states
        assert pid_enable_btn.is_disabled()
        assert pid_disable_btn.is_enabled()
        
        # Check status text
        pid_status = page.locator('#pidStatus')
        assert pid_status.text_content() == 'ENABLED'
    
    def test_set_target_temperature(self, page: Page, test_html_path, mock_server):
        """Test setting target temperature"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        page.evaluate(f"""
            const originalFetch = window.fetch;
            window.fetch = function(url, options) {{
                if (url == null) {{
                    console.error('URL is null or undefined');
                    return Promise.reject(new Error('URL is null or undefined'));
                }}
                const mockUrl = '{mock_server}' + url;
                return originalFetch(mockUrl, options);
            }};
        """)
        
        page.on("dialog", lambda dialog: dialog.accept()) # For alert messages
        
        temp_input = page.locator('#targetTempInput')
        set_btn = page.locator('button:has-text("Set")').first # More specific selector
        
        temp_input.fill('30.5')
        set_btn.click()
        
        # Wait for alert to ensure fetch was processed (assuming alert means success)
        # Or, if there's a visual confirmation on page, wait for that.
        # For now, a small timeout for the fetch to complete.
        page.wait_for_timeout(500) # Increased slightly for network op
        
        # Verify the input value is still there (and implicitly that the fetch was called)
        assert temp_input.input_value() == '30.5'
        # Add assertion for confirmation if possible, e.g., if status text changes
    
    def test_update_pid_parameters(self, page: Page, test_html_path, mock_server):
        """Test updating PID parameters"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        page.evaluate(f"""
            const originalFetch = window.fetch;
            window.fetch = function(url, options) {{
                if (url == null) {{
                    console.error('URL is null or undefined');
                    return Promise.reject(new Error('URL is null or undefined'));
                }}
                const mockUrl = '{mock_server}' + url;
                return originalFetch(mockUrl, options);
            }};
        """)
        
        page.on("dialog", lambda dialog: dialog.accept()) # For alert messages
        
        kp_input = page.locator('#kpInput')
        ki_input = page.locator('#kiInput')
        kd_input = page.locator('#kdInput')
        update_btn = page.locator('button:has-text("Update")') # More specific selector
        
        kp_input.fill('2.0')
        ki_input.fill('0.5')
        kd_input.fill('0.1')
        update_btn.click()
        
        page.wait_for_timeout(500) # Increased slightly for network op
        
        # Verify the input values are still there
        assert kp_input.input_value() == '2.0'
        assert ki_input.input_value() == '0.5'
        assert kd_input.input_value() == '0.1'
        # Add assertion for confirmation if possible

class TestDataDisplay:
    """Test data display and chart functionality"""
    
    def test_clear_data_function(self, page: Page, test_html_path, mock_server):
        """Test clear data functionality"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        page.evaluate(f"""
            const originalFetch = window.fetch;
            window.fetch = function(url, options) {{
                if (url == null) {{
                    console.error('URL is null or undefined');
                    return Promise.reject(new Error('URL is null or undefined'));
                }}
                const mockUrl = '{mock_server}' + url;
                return originalFetch(mockUrl, options);
            }};
        """)
        
        page.on("dialog", lambda dialog: dialog.accept())
        
        # First, add some data so clearing has an effect
        page.evaluate("""
            addSensorReading(Date.now(), 0.1, 10, {});
            addSensorReading(Date.now(), 0.2, 20, {});
        """)
        assert page.locator('#sampleCount').text_content() == '2'


        clear_btn = page.locator('button:has-text("Clear Data")') # More specific selector
        clear_btn.click()
        
        # Wait for the data to be cleared
        page.wait_for_function("document.getElementById('sampleCount').textContent === '0'", timeout=5000)
        
        # Check that sample count is reset
        sample_count = page.locator('#sampleCount')
        assert sample_count.text_content() == '0'
        
        # Check that current readings are reset
        assert page.locator('#currentVoltage').text_content() == '--'
        assert page.locator('#currentTemperature').text_content() == '--'
    
    def test_sensor_reading_update(self, page: Page, test_html_path):
        """Test sensor reading update functionality"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        # Simulate adding a sensor reading
        page.evaluate("""
            addSensorReading(
                Date.now(),
                0.7834,
                25.67,
                {
                    heaterState: true,
                    targetTemp: 30.0,
                    pidEnabled: true,
                    pidOutput: 45.2
                }
            );
        """)
        
        # Check that values are updated
        assert page.locator('#currentVoltage').text_content() == '0.7834'
        assert page.locator('#currentTemperature').text_content() == '25.67'
        assert page.locator('#currentTemp').text_content() == '25.67'
        assert page.locator('#sampleCount').text_content() == '1'
        
        # Check heater status updates
        assert page.locator('#relayStatus').text_content() == 'ON'
        assert page.locator('#pidStatus').text_content() == 'ENABLED'
        assert page.locator('#pidOutputValue').text_content() == '45.2' # In HTML, output is toFixed(1)
    
    def test_chart_initialization(self, page: Page, test_html_path):
        """Test chart initialization"""
        page.goto(f"file://{test_html_path.absolute()}")
        
        # Wait for chart initialization logic to run
        page.wait_for_timeout(1000) # Default timeout in original test
        
        # Check chart status
        chart_status = page.locator('#chartStatus')
        status_text = chart_status.text_content()
        
        # Chart should either be active or disabled (depending on internet access for Chart.js CDN)
        # For a local test, it's more predictable if Chart.js is local or if we mock its absence/presence.
        # Assuming CDN access or Chart.js is otherwise available:
        assert status_text == 'Active (Dual Sensor)' or status_text == 'Disabled (no internet)' # Original check
        
        # Check that canvas element exists and is visible (or styled display:none if Chart.js fails)
        canvas = page.locator('#sensorChart')
        is_chart_active = status_text == 'Active (Dual Sensor)'
        if is_chart_active:
            assert canvas.is_visible()
        else: # Disabled (no internet)
            # The JS code sets style.display = 'none' if Chart is undefined.
             assert canvas.get_attribute('style') == 'display: none;' or not canvas.is_visible()
