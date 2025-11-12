#!/usr/bin/env python3
"""
RPLiDAR Deep Diagnostic Tool
Comprehensive logging and testing with maximum verbosity
"""

import serial
import time
import struct
import sys
import os

class RPLiDARDiagnostic:
    # RPLiDAR Commands
    SYNC_BYTE = 0xA5
    CMD_STOP = 0x25
    CMD_RESET = 0x40
    CMD_SCAN = 0x20
    CMD_FORCE_SCAN = 0x21
    CMD_GET_INFO = 0x50
    CMD_GET_HEALTH = 0x52
    CMD_GET_SAMPLERATE = 0x59

    def __init__(self, port='/dev/rplidar', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def log(self, msg, level="INFO"):
        """Print formatted log message"""
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] [{level:5s}] {msg}")

    def log_hex(self, data, prefix=""):
        """Log data in hex format"""
        if len(data) > 0:
            hex_str = ' '.join(f'{b:02X}' for b in data)
            self.log(f"{prefix}{hex_str}", "DATA")
        else:
            self.log(f"{prefix}(empty)", "DATA")

    def open_port(self):
        """Open serial port with detailed logging"""
        self.log("="*70, "INFO")
        self.log("RPLiDAR Deep Diagnostic - Maximum Verbosity", "INFO")
        self.log("="*70, "INFO")
        self.log(f"Port: {self.port}", "INFO")
        self.log(f"Baudrate: {self.baudrate}", "INFO")
        print()

        try:
            self.log("Opening serial port...", "INFO")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                rtscts=False,
                dsrdtr=False
            )

            self.log(f"✅ Port opened: {self.ser.is_open}", "OK")
            self.log(f"   Baudrate: {self.ser.baudrate}", "INFO")
            self.log(f"   Timeout: {self.ser.timeout}s", "INFO")
            self.log(f"   Parity: {self.ser.parity}", "INFO")
            self.log(f"   Stopbits: {self.ser.stopbits}", "INFO")
            self.log(f"   Bytesize: {self.ser.bytesize}", "INFO")
            print()
            return True

        except serial.SerialException as e:
            self.log(f"❌ Failed to open port: {e}", "ERROR")
            return False

    def check_control_signals(self):
        """Check and log serial control signals"""
        self.log("Checking serial control signals...", "INFO")
        try:
            cts = self.ser.cts if hasattr(self.ser, 'cts') else None
            dsr = self.ser.dsr if hasattr(self.ser, 'dsr') else None
            ri = self.ser.ri if hasattr(self.ser, 'ri') else None
            cd = self.ser.cd if hasattr(self.ser, 'cd') else None

            self.log(f"   CTS (Clear To Send): {cts}", "INFO")
            self.log(f"   DSR (Data Set Ready): {dsr}", "INFO")
            self.log(f"   RI  (Ring Indicator): {ri}", "INFO")
            self.log(f"   CD  (Carrier Detect): {cd}", "INFO")
            print()
        except Exception as e:
            self.log(f"   Could not read signals: {e}", "WARN")
            print()

    def test_buffer_status(self):
        """Check input/output buffer status"""
        try:
            in_waiting = self.ser.in_waiting
            out_waiting = self.ser.out_waiting if hasattr(self.ser, 'out_waiting') else 0

            self.log(f"Buffer status:", "INFO")
            self.log(f"   Input buffer: {in_waiting} bytes", "INFO")
            self.log(f"   Output buffer: {out_waiting} bytes", "INFO")
            print()
            return in_waiting
        except Exception as e:
            self.log(f"   Could not read buffers: {e}", "WARN")
            return 0

    def send_command(self, cmd, payload=None, label=""):
        """Send command with detailed logging"""
        self.log(f"{'─'*70}", "INFO")
        self.log(f"Sending command: {label}", "CMD")

        # Build command packet
        packet = bytes([self.SYNC_BYTE, cmd])
        if payload:
            packet += payload

        self.log_hex(packet, "TX: ")

        try:
            # Clear buffers first
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # Send command
            bytes_written = self.ser.write(packet)
            self.ser.flush()

            self.log(f"   Wrote {bytes_written} bytes", "INFO")
            time.sleep(0.1)  # Give device time to respond

            return True

        except serial.SerialTimeoutException:
            self.log("   ❌ Write timeout!", "ERROR")
            return False
        except Exception as e:
            self.log(f"   ❌ Write error: {e}", "ERROR")
            return False

    def read_response(self, expected_bytes=None, timeout=2.0, label=""):
        """Read response with detailed logging"""
        self.log(f"Reading response: {label}", "INFO")

        start_time = time.time()
        data = bytearray()

        try:
            # Check what's in buffer first
            in_waiting = self.ser.in_waiting
            self.log(f"   Bytes in buffer: {in_waiting}", "INFO")

            if expected_bytes:
                # Read exact number of bytes
                self.log(f"   Expecting: {expected_bytes} bytes", "INFO")
                data = self.ser.read(expected_bytes)
            else:
                # Read whatever is available
                while time.time() - start_time < timeout:
                    if self.ser.in_waiting > 0:
                        chunk = self.ser.read(self.ser.in_waiting)
                        data.extend(chunk)
                        time.sleep(0.05)
                    else:
                        if len(data) > 0:
                            break
                        time.sleep(0.01)

            elapsed = time.time() - start_time
            self.log(f"   Received: {len(data)} bytes in {elapsed:.3f}s", "INFO")

            if len(data) > 0:
                self.log_hex(data, "RX: ")
                return bytes(data)
            else:
                self.log("   ⚠️  No data received", "WARN")
                return bytes()

        except Exception as e:
            self.log(f"   ❌ Read error: {e}", "ERROR")
            return bytes()

    def motor_control(self, start=True):
        """Control motor via DTR with logging"""
        action = "START" if start else "STOP"
        self.log(f"{'─'*70}", "INFO")
        self.log(f"Motor Control: {action}", "CMD")

        try:
            self.ser.setDTR(not start)  # DTR=False means motor ON
            state = "LOW (ON)" if start else "HIGH (OFF)"
            self.log(f"   DTR set to: {state}", "INFO")
            time.sleep(0.5)

            # Check if motor control worked by trying to get a response
            self.test_buffer_status()

        except Exception as e:
            self.log(f"   ❌ DTR control error: {e}", "ERROR")

    def test_get_info(self):
        """Test GET_INFO command with full logging"""
        self.send_command(self.CMD_GET_INFO, label="GET_INFO (0x50)")

        # Read response descriptor (7 bytes)
        response = self.read_response(expected_bytes=27, timeout=2.0, label="Device Info")

        if len(response) >= 7:
            self.log("Parsing response descriptor:", "INFO")
            self.log(f"   Start Flag: 0x{response[0]:02X}{response[1]:02X}", "INFO")
            self.log(f"   Length: {struct.unpack('<I', response[2:6])[0]} bytes", "INFO")
            self.log(f"   Mode: 0x{response[6]:02X}", "INFO")

            if len(response) >= 27:
                model = response[7]
                fw_minor = response[8]
                fw_major = response[9]
                hardware = response[10]
                serial_num = response[11:27]

                self.log("Device Information:", "INFO")
                self.log(f"   Model: {model}", "INFO")
                self.log(f"   Firmware: {fw_major}.{fw_minor}", "INFO")
                self.log(f"   Hardware: {hardware}", "INFO")
                self.log(f"   Serial: {serial_num.hex().upper()}", "INFO")
                return True

        return False

    def test_get_health(self):
        """Test GET_HEALTH command"""
        self.send_command(self.CMD_GET_HEALTH, label="GET_HEALTH (0x52)")

        response = self.read_response(expected_bytes=10, timeout=2.0, label="Health Status")

        if len(response) >= 10:
            self.log("Parsing health response:", "INFO")
            status = response[7]
            error_code = struct.unpack('<H', response[8:10])[0]

            status_str = {0: "GOOD", 1: "WARNING", 2: "ERROR"}.get(status, "UNKNOWN")

            self.log(f"   Status: {status} ({status_str})", "INFO")
            self.log(f"   Error Code: 0x{error_code:04X}", "INFO")

            return status == 0

        return False

    def test_reset(self):
        """Test RESET command"""
        self.send_command(self.CMD_RESET, label="RESET (0x40)")
        self.log("   Waiting for device reset...", "INFO")
        time.sleep(2)

        # Try to reconnect
        self.log("   Attempting to reopen port after reset...", "INFO")
        try:
            self.ser.close()
            time.sleep(0.5)
            self.ser.open()
            self.log("   ✅ Port reopened successfully", "OK")
            return True
        except Exception as e:
            self.log(f"   ❌ Failed to reopen: {e}", "ERROR")
            return False

    def run_full_diagnostic(self):
        """Run complete diagnostic suite"""
        if not self.open_port():
            return False

        try:
            # Step 1: Check control signals
            self.check_control_signals()

            # Step 2: Check initial buffer state
            self.test_buffer_status()

            # Step 3: STOP command
            self.send_command(self.CMD_STOP, label="STOP (0x25)")
            time.sleep(0.5)

            # Step 4: Start motor
            self.motor_control(start=True)
            self.log("⚠️  PHYSICAL CHECK: Is motor spinning? (wait 2 sec)", "CHECK")
            time.sleep(2)

            # Step 5: Try GET_INFO
            info_ok = self.test_get_info()
            print()

            # Step 6: Try GET_HEALTH
            health_ok = self.test_get_health()
            print()

            # Step 7: Try reading any stray data
            self.log("{'─'*70}", "INFO")
            self.log("Checking for any unexpected data in buffer...", "INFO")
            time.sleep(0.5)
            stray = self.read_response(timeout=1.0, label="Stray Data")
            print()

            # Summary
            self.log("="*70, "INFO")
            self.log("DIAGNOSTIC SUMMARY", "INFO")
            self.log("="*70, "INFO")
            self.log(f"Port Communication: {'✅ OK' if self.ser.is_open else '❌ FAILED'}", "INFO")
            self.log(f"GET_INFO Response: {'✅ OK' if info_ok else '❌ FAILED'}", "INFO")
            self.log(f"GET_HEALTH Response: {'✅ OK' if health_ok else '❌ FAILED'}", "INFO")
            print()

            if not info_ok and not health_ok:
                self.log("DIAGNOSIS: RPLiDAR not responding to commands", "ERROR")
                self.log("Possible causes:", "INFO")
                self.log("  1. Motor not receiving power (check USB power)", "INFO")
                self.log("  2. Faulty USB cable", "INFO")
                self.log("  3. Dead/damaged RPLiDAR unit", "INFO")
                self.log("  4. Wrong device (not actually RPLiDAR)", "INFO")

            # Keep motor running for observation
            self.log("", "INFO")
            self.log("Keeping motor ON for 10 seconds for observation...", "INFO")
            for i in range(10, 0, -1):
                print(f"\r   {i} seconds remaining... ", end='', flush=True)
                time.sleep(1)
            print()

            # Cleanup
            self.log("Stopping motor and closing port...", "INFO")
            self.motor_control(start=False)
            self.ser.close()
            self.log("✅ Diagnostic complete", "OK")

        except KeyboardInterrupt:
            self.log("\n⚠️  Interrupted by user", "WARN")
            if self.ser and self.ser.is_open:
                self.motor_control(start=False)
                self.ser.close()
        except Exception as e:
            self.log(f"❌ Diagnostic error: {e}", "ERROR")
            import traceback
            traceback.print_exc()
            if self.ser and self.ser.is_open:
                self.ser.close()

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/rplidar'
    diag = RPLiDARDiagnostic(port)
    diag.run_full_diagnostic()
