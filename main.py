import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import os
import sys
import re
import logging
from ctypes import *
from struct import pack
from typing import Optional, List, Dict, Any

# ===================== Constants =====================
# Buffer size limits
MAX_RECEIVE_BUFFER_SIZE = 1024
MAX_RESPONSE_BUFFER_SIZE = 512
MAX_LOG_LINES = 1000

# Timeouts (seconds)
DEFAULT_RESPONSE_TIMEOUT = 2.0
SILENCE_TIMEOUT = 0.2
THREAD_JOIN_TIMEOUT = 1.0
MAX_BUFFER_AGE = 2.0

# Protocol constants
FRAME_HEADER_CC = 0xCC
FRAME_FOOTER_FF = 0xFF
FRAME_HEADER_AA = 0xAA
FRAME_FOOTER_55 = 0x55
FRAME_FOOTER_CC = 0xCC

# Connection defaults
DEFAULT_STATION_ID = 5
DEFAULT_SERIAL_BAUD = 9600
DEFAULT_CAN_BAUDRATE = "500kbps"

# ===================== Logging Setup =====================
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('can_tool.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# ===================== SDK Implementation =====================
def crc16_modbus(data: bytes) -> bytes:
    """Calculate MODBUS CRC16 checksum"""
    if not data:
        return b'\x00\x00'
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return pack("<H", crc)


def build_modbus_frame(station_id: int, cmd_code: int, data_bytes: list) -> bytes:
    """Build standard MODBUS frame"""
    control_word = cmd_code | REQ_FLAG
    frame = bytes([station_id, control_word, len(data_bytes)] + data_bytes)
    crc = crc16_modbus(frame)
    return frame + crc

def parse_hex_input(hex_str: str) -> list:
    """
    Parse hex input string to byte list
    Auto-splits every 2 characters, ignores all separators
    Supports formats:
    - "0103 0000 0001" -> [01, 03, 00, 00, 00, 01]
    - "010300000001"   -> [01, 03, 00, 00, 00, 01]
    - "01 03 00 00 00 01" -> [01, 03, 00, 00, 00, 01]
    - "01,03,00,00,00,01" -> [01, 03, 00, 00, 00, 01]
    """
    hex_str = hex_str.strip()
    if not hex_str:
        return []

    # Remove all separators: spaces, commas, 0x prefixes
    hex_clean = hex_str.replace(" ", "").replace(",", "").replace("0x", "").replace("0X", "")

    # Check if all characters are valid hex
    if not hex_clean:
        return []

    # Check for invalid characters
    try:
        int(hex_clean, 16)
    except ValueError:
        # Find the invalid character
        for ch in hex_clean:
            if ch not in '0123456789abcdefABCDEF':
                raise ValueError(f"Invalid hex character: '{ch}'")

    # Must be even number of characters
    if len(hex_clean) % 2 != 0:
        raise ValueError(f"Invalid hex string length: {len(hex_clean)} characters (must be even)")

    # Split every 2 characters
    result = []
    for i in range(0, len(hex_clean), 2):
        byte_str = hex_clean[i:i+2]
        value = int(byte_str, 16)
        result.append(value)

    return result

# ===================== Main Program =====================
VCI_USB_CAN2 = 4


class VCI_CAN_OBJ(Structure):
    """CAN object structure for TX/RX"""
    _fields_ = [
        ("ID", c_uint), ("TimeStamp", c_uint), ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte), ("RemoteFlag", c_ubyte), ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte), ("Data", c_ubyte * 8), ("Reserved", c_ubyte * 3)
    ]


class CAN_GUI_Tool:
    def __init__(self, root):
        self.root = root
        self.root.title("CAN/MODBUS Tool")
        self.root.geometry("680x620")
        self.root.minsize(620, 560)

        # Set window icon
        try:
            icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logo.ico")
            if os.path.exists(icon_path):
                self.root.iconbitmap(icon_path)
        except Exception as e:
            logger.warning(f"Failed to set window icon: {e}")

        self.ser = None
        self.can_dll = None
        self.is_connected = False
        self.port_map = {}
        self.dev_type_map = {}
        self.station_id = 5  # Default node ID
        self.cur_dev_type = None  # Current device type

        # Receive thread related
        self.receive_thread = None
        self.receive_running = False
        self.receive_buffer = bytearray()  # Receive buffer
        self.response_buffer = bytearray()  # Buffer for accumulating command responses
        self.last_response_time = 0  # Track when last response was received
        self.waiting_for_response = False  # Flag to indicate we're waiting for a response
        self.response_timeout = DEFAULT_RESPONSE_TIMEOUT  # Timeout for waiting for response

        # Thread safety
        self.buffer_lock = threading.Lock()  # Lock for buffer access
        self.serial_lock = threading.Lock()  # Lock for serial port access

        # Connection state
        self.last_command_time = 0  # Time when last command was sent
        self.command_retry_count = 0  # Number of retries for current command

        self.baudrate_map = {
            "1Mbps": 0x01, "800kbps": 0x02, "500kbps": 0x03, "400kbps": 0x04,
            "250kbps": 0x05, "200kbps": 0x06, "125kbps": 0x07, "100kbps": 0x08,
            "50kbps": 0x09, "20kbps": 0x0A, "10kbps": 0x0B, "5kbps": 0x0C
        }
        self.serial_baud_list = ["4800", "9600", "19200", "38400", "57600", "115200"]
        self.can_baud_reg = {
            "1Mbps":   (0, 0x14),
            "800kbps": (0, 0x16),
            "500kbps": (0, 0x1C),
            "400kbps": (0, 0x1C),
            "250kbps": (1, 0x1C),
            "200kbps": (1, 0x1C),
            "125kbps": (3, 0x1C),
            "100kbps": (3, 0x1C),
        }

        self.setup_connect_frame()
        self.setup_send_frame()
        self.setup_display()

    def start_receive_thread(self):
        """Start receive thread"""
        if self.receive_running:
            logger.warning("Receive thread already running")
            return

        self.receive_running = True

        try:
            if self.cur_dev_type == "CANALYST2" and self.can_dll:
                # CAN device uses DLL
                self.receive_thread = threading.Thread(target=self._receive_can_frames, daemon=True, name="CAN-Receiver")
                self.log_print("[*] CAN receive thread started")
            elif self.ser and self.ser.is_open:
                # Serial device uses serial
                self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True, name="Serial-Receiver")
                self.log_print("[*] Serial receive thread started")
            else:
                self.receive_running = False
                logger.error("Cannot start receive thread: no device available")
                return

            self.receive_thread.start()
        except Exception as e:
            self.receive_running = False
            logger.error(f"Failed to start receive thread: {e}")
            self.log_print(f"[ERR] Failed to start receive thread: {e}")

    def stop_receive_thread(self):
        """Stop receive thread gracefully"""
        if not self.receive_running:
            return

        self.receive_running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=THREAD_JOIN_TIMEOUT)
            if self.receive_thread.is_alive():
                logger.warning("Receive thread did not stop gracefully")
            self.receive_thread = None
            self.log_print("[*] Receive thread stopped")

    def _receive_loop(self):
        """Serial receive loop - optimized for complete frame reception"""
        last_data_time = time.time()
        ccff_buffer = bytearray()  # Buffer for accumulating CC...FF frames

        while self.receive_running and self.ser:
            try:
                # Check if serial port is still open
                if not self.ser.is_open:
                    logger.warning("Serial port closed unexpectedly")
                    break

                if self.ser.in_waiting > 0:
                    # Read all available data with lock
                    with self.serial_lock:
                        data = self.ser.read(self.ser.in_waiting)

                    if not data:
                        time.sleep(0.001)
                        continue

                    last_data_time = time.time()

                    # Check if we're accumulating a CC...FF frame
                    if len(ccff_buffer) > 0 or (len(data) > 0 and data[0] == FRAME_HEADER_CC):
                        ccff_buffer.extend(data)

                        # Check if we have a complete CC...FF frame
                        if len(ccff_buffer) >= 5 and ccff_buffer[0] == FRAME_HEADER_CC:
                            # Look for FF
                            ff_pos = -1
                            for i in range(4, len(ccff_buffer)):
                                if ccff_buffer[i] == FRAME_FOOTER_FF:
                                    ff_pos = i
                                    break

                            if ff_pos != -1:
                                # Complete frame found
                                frame = bytes(ccff_buffer[:ff_pos + 1])
                                del ccff_buffer[:ff_pos + 1]
                                self._parse_uimessage_response(frame)
                            elif len(ccff_buffer) > MAX_RECEIVE_BUFFER_SIZE:
                                # Buffer too large, flush it
                                logger.warning(f"CCFF buffer overflow, flushing {len(ccff_buffer)} bytes")
                                ccff_buffer.clear()
                    else:
                        # Regular data, add to main buffer with lock
                        with self.buffer_lock:
                            self.receive_buffer.extend(data)
                            # Limit buffer size
                            if len(self.receive_buffer) > MAX_RECEIVE_BUFFER_SIZE:
                                logger.warning(f"Receive buffer overflow, truncating")
                                del self.receive_buffer[:len(self.receive_buffer) - MAX_RECEIVE_BUFFER_SIZE]
                        # Process buffer immediately
                        self._parse_receive_buffer()
                else:
                    # Check if buffer has old data that needs to be flushed
                    if len(self.receive_buffer) > 0 and (time.time() - last_data_time) > MAX_BUFFER_AGE:
                        self._flush_incomplete_response()

                    # Shorter sleep for more responsive reception
                    time.sleep(0.005)  # 5ms polling interval
            except serial.SerialException as e:
                if self.receive_running:
                    logger.error(f"Serial error: {e}")
                    self.log_print(f"[ERR] Serial error: {str(e)}")
                break
            except Exception as e:
                if self.receive_running:
                    logger.error(f"Receive loop error: {e}", exc_info=True)
                    self.log_print(f"[ERR] Serial receive error: {str(e)}")
                break

    def _parse_receive_buffer(self):
        """Parse received frames in buffer - optimized for string/hex responses"""
        while len(self.receive_buffer) >= 2:
            if self.cur_dev_type == "CANALYST2":
                # CAN device: parse AA...55 frames
                if not self._parse_can_frame():
                    break
            else:
                # Serial device: try to parse frames
                parsed = False

                # Try UIMessage frame (AA...CC, 16 bytes)
                if self._parse_uimessage_frame():
                    parsed = True
                    continue

                # Try CAN gateway frame (AA...55 format)
                if self._parse_can_frame():
                    parsed = True
                    continue

                # Try string/hex command response (variable length)
                if self._parse_response_frame():
                    parsed = True
                    continue

                # If nothing parsed, check if we have enough data or wait for more
                if not parsed:
                    # If buffer has data but can't parse, check for timeout
                    if len(self.receive_buffer) > 0:
                        # Try to find any valid frame start
                        has_frame_start = False
                        for i in range(len(self.receive_buffer)):
                            if self.receive_buffer[i] == 0xAA:
                                has_frame_start = True
                                # Remove garbage before frame start
                                if i > 0:
                                    del self.receive_buffer[:i]
                                break

                        if not has_frame_start:
                            # No frame start found, might be raw data or incomplete
                            # If buffer is getting large, flush it
                            if len(self.receive_buffer) > 128:
                                raw = bytes(self.receive_buffer[:])
                                self.receive_buffer.clear()
                                hex_str = ' '.join(f'{b:02X}' for b in raw)
                                self.log_print(f"[RX] Raw: {hex_str}")
                    break

    def _parse_can_frame(self) -> bool:
        """Parse CAN frame (AA...55 format) with proper error handling"""
        with self.buffer_lock:
            return self._parse_can_frame_impl()

    def _parse_can_frame_impl(self) -> bool:
        """Internal CAN frame parsing implementation"""
        # Find header 0xAA
        if len(self.receive_buffer) == 0:
            return False

        if self.receive_buffer[0] != FRAME_HEADER_AA:
            # Try to find AA in buffer
            aa_pos = -1
            for i in range(len(self.receive_buffer)):
                if self.receive_buffer[i] == FRAME_HEADER_AA:
                    aa_pos = i
                    break

            if aa_pos == -1:
                # No AA found, clear buffer if it's getting large
                if len(self.receive_buffer) > MAX_RECEIVE_BUFFER_SIZE // 2:
                    self._flush_incomplete_response()
                return False
            else:
                # Remove bytes before AA
                if aa_pos > 0:
                    del self.receive_buffer[:aa_pos]
                return False

        # Need at least 4 bytes
        if len(self.receive_buffer) < 4:
            return False

        # Get data length
        ctrl_byte = self.receive_buffer[1]
        data_len = ctrl_byte & 0x0F

        # Validate data length
        if data_len > 8:
            logger.warning(f"Invalid CAN data length: {data_len}")
            self.receive_buffer.pop(0)
            return False

        # Determine frame type
        is_extended = (ctrl_byte & 0x20) != 0
        if is_extended:
            frame_len = 2 + 1 + 4 + data_len + 1  # AA + ctrl + 4-byte ID + data + 55
        else:
            frame_len = 2 + 1 + 2 + data_len + 1  # AA + ctrl + 2-byte ID + data + 55

        if len(self.receive_buffer) < frame_len:
            return False

        # Check footer 0x55
        if self.receive_buffer[frame_len - 1] != FRAME_FOOTER_55:
            # Invalid frame, skip this AA
            logger.warning(f"Invalid CAN frame footer: 0x{self.receive_buffer[frame_len - 1]:02X}")
            self.receive_buffer.pop(0)
            return False

        # Extract complete frame
        frame = bytes(self.receive_buffer[:frame_len])
        del self.receive_buffer[:frame_len]

        # Display CAN frame
        self._display_can_frame(frame, is_extended)
        return True

    def _parse_uimessage_frame(self) -> bool:
        """Parse UIMessage frame (AA...CC format, 16 bytes)"""
        # Find header 0xAA
        if len(self.receive_buffer) == 0:
            return False

        if self.receive_buffer[0] != 0xAA:
            # Try to find AA in buffer
            aa_pos = -1
            for i in range(len(self.receive_buffer)):
                if self.receive_buffer[i] == 0xAA:
                    aa_pos = i
                    break

            if aa_pos == -1:
                # No AA found, clear buffer if it's getting large
                if len(self.receive_buffer) > 64:
                    self._flush_incomplete_response()
                return False
            else:
                # Remove bytes before AA
                if aa_pos > 0:
                    del self.receive_buffer[:aa_pos]
                return False

        # UIMessage fixed 16 bytes
        if len(self.receive_buffer) < 16:
            return False

        # Check footer 0xCC — 不匹配时不 pop，留给上层检测其他格式
        if self.receive_buffer[15] != 0xCC:
            # Not a UIMessage frame, might be CAN frame or other format
            return False

        # Extract complete frame
        frame = bytes(self.receive_buffer[:16])
        del self.receive_buffer[:16]

        # Display frame without CRC check
        self._display_uimessage_frame(frame)
        return True

    def _verify_uimessage_crc(self, frame: bytes) -> bool:
        """Verify UIMessage CRC16"""
        # CRC range: ID+CW+DL+d0-d7+Aux (bytes 1-11, excluding AA)
        crc_data = frame[1:12]
        expected_crc = crc16_modbus(crc_data)
        actual_crc = frame[12:14]
        return expected_crc == actual_crc

    def _parse_response_frame(self) -> bool:
        """Parse response frame for string/hex commands (variable length)"""
        if len(self.receive_buffer) < 2:
            return False

        # Look for frame start marker (0xAA)
        if self.receive_buffer[0] != 0xAA:
            # Not a valid frame start, check if it's raw data response
            if self._is_raw_data_response():
                return self._handle_raw_data_response()
            return False

        # Check if this is a UIMessage frame (AA...CC)
        if len(self.receive_buffer) >= 16 and self.receive_buffer[15] == 0xCC:
            return False  # Let _parse_uimessage_frame handle it

        # Check if this is a CAN gateway frame (AA...55)
        if self._has_can_frame_footer():
            return False  # Let _parse_can_frame handle it

        # Try to parse as variable-length response frame
        return self._parse_variable_length_response()

    def _is_raw_data_response(self) -> bool:
        """Check if buffer contains raw data response (no frame markers)"""
        # If we've been waiting and buffer has data without frame markers
        return len(self.receive_buffer) > 0 and self.receive_buffer[0] != 0xAA

    def _handle_raw_data_response(self) -> bool:
        """Handle raw data response without frame markers"""
        if len(self.receive_buffer) > 0:
            raw = bytes(self.receive_buffer[:])
            self.receive_buffer.clear()
            hex_str = ' '.join(f'{b:02X}' for b in raw)
            self.log_print(f"[RX] Response: {hex_str}")
            return True
        return False

    def _has_can_frame_footer(self) -> bool:
        """Check if buffer has CAN frame footer (0x55) at expected position"""
        if len(self.receive_buffer) < 4:
            return False

        ctrl_byte = self.receive_buffer[1]
        data_len = ctrl_byte & 0x0F
        is_extended = (ctrl_byte & 0x20) != 0

        if is_extended:
            frame_len = 2 + 1 + 4 + data_len + 1  # AA + ctrl + 4-byte ID + data + 55
        else:
            frame_len = 2 + 1 + 2 + data_len + 1  # AA + ctrl + 2-byte ID + data + 55

        if len(self.receive_buffer) >= frame_len:
            return self.receive_buffer[frame_len - 1] == 0x55

        return False

    def _parse_variable_length_response(self) -> bool:
        """Parse variable-length response frame"""
        if len(self.receive_buffer) < 3:
            return False

        # Check for MODBUS-like response format
        # Format: [StationID][ControlWord][DataLen][Data...][CRC16]
        station_id = self.receive_buffer[0]
        control_word = self.receive_buffer[1]
        data_len = self.receive_buffer[2]

        # Validate frame length
        if len(self.receive_buffer) < 3 + data_len + 2:  # +2 for CRC
            return False  # Wait for more data

        # Extract complete frame
        frame_len = 3 + data_len + 2
        frame = bytes(self.receive_buffer[:frame_len])
        del self.receive_buffer[:frame_len]

        # Parse and display
        self._display_response_frame(frame, station_id, control_word, data_len)
        return True

    def _display_response_frame(self, frame: bytes, station_id: int, control_word: int, data_len: int):
        """Display parsed response frame"""
        data_bytes = frame[3:3 + data_len]

        # Determine frame type (bit7: 0=ACK, 1=Request)
        is_request = (control_word & 0x80) != 0
        cmd_code = control_word & 0x7F

        # Find command name
        cmd_name = self._get_cmd_name(cmd_code)

        # Format data
        if data_len > 0:
            data_hex = ' '.join(f'{b:02X}' for b in data_bytes)
            data_decimal = self._format_data_decimal(data_bytes, data_len)
            if data_decimal:
                data_display = f"{data_hex} ({data_decimal})"
            else:
                data_display = data_hex
        else:
            data_display = "No data"

        # Show raw frame
        frame_hex = ' '.join(f'{b:02X}' for b in frame)
        self.log_print(f"[RX] Response: {frame_hex}")

        # Show parsed content
        frame_type = "Request" if is_request else "ACK"
        self.log_print(f"   -- Type:{frame_type} Node:{station_id} Cmd:{cmd_name}(0x{cmd_code:02X}) DataLen:{data_len} Data:{data_display}")

    def _verify_response_crc(self, frame: bytes) -> bool:
        """Verify CRC of response frame"""
        # CRC is at the last 2 bytes
        if len(frame) < 5:
            return True  # No CRC to verify

        # Data to verify: everything except the last 2 bytes (CRC)
        data_to_verify = frame[:-2]
        received_crc = frame[-2:]
        calculated_crc = crc16_modbus(data_to_verify)

        return received_crc == calculated_crc

    def _parse_modbus_response(self, frame: bytes):
        """Parse MODBUS response frame and extract data"""
        if len(frame) < 3:
            return None

        station_id = frame[0]
        control_word = frame[1]
        data_len = frame[2]
        data_bytes = frame[3:3 + data_len]

        return {
            'station_id': station_id,
            'control_word': control_word,
            'data_len': data_len,
            'data': data_bytes,
            'is_request': (control_word & 0x80) != 0,
            'cmd_code': control_word & 0x7F
        }

    def _decode_response_data(self, data: bytes, data_len: int) -> dict:
        """Decode response data based on length"""
        result = {}

        if data_len == 0:
            result['type'] = 'empty'
            result['value'] = None
        elif data_len == 1:
            result['type'] = 'uint8'
            result['value'] = data[0]
            result['hex'] = f'0x{data[0]:02X}'
        elif data_len == 2:
            value = data[0] | (data[1] << 8)
            result['type'] = 'uint16'
            result['value'] = value
            result['hex'] = f'0x{value:04X}'
        elif data_len == 3:
            value = data[0] | (data[1] << 8) | (data[2] << 16)
            if value >= 0x800000:
                value -= 0x1000000
            result['type'] = 'int24'
            result['value'] = value
            result['hex'] = f'0x{value & 0xFFFFFF:06X}'
        elif data_len >= 4:
            value = int.from_bytes(data[:4], byteorder='little', signed=True)
            result['type'] = 'int32'
            result['value'] = value
            result['hex'] = f'0x{value & 0xFFFFFFFF:08X}'

        return result

    def _is_response_complete(self) -> bool:
        """Check if we have a complete response in the buffer"""
        if len(self.receive_buffer) < 3:
            return False

        # Check if buffer has a complete MODBUS-like frame
        data_len = self.receive_buffer[2]
        expected_len = 3 + data_len + 2  # +2 for CRC
        return len(self.receive_buffer) >= expected_len

    def _is_response_buffer_complete(self) -> bool:
        """Check if response buffer has a complete frame"""
        if len(self.response_buffer) < 3:
            return False

        # Check if buffer has a complete MODBUS-like frame
        data_len = self.response_buffer[2]
        expected_len = 3 + data_len + 2  # +2 for CRC
        return len(self.response_buffer) >= expected_len

    def _accumulate_response_data(self, data: bytes) -> bool:
        """
        Accumulate response data until complete frame is received.
        Returns True if a complete frame is available.
        """
        self.response_buffer.extend(data)

        # Check if we have a complete frame
        if len(self.response_buffer) >= 3:
            data_len = self.response_buffer[2]
            expected_len = 3 + data_len + 2  # +2 for CRC
            if len(self.response_buffer) >= expected_len:
                return True

        return False

    def _handle_timeout_response(self):
        """Handle response timeout - flush any pending data"""
        if len(self.response_buffer) > 0:
            self.log_print("[WARN] Response timeout - flushing incomplete data")
            self._flush_incomplete_response()
            self.response_buffer.clear()

    def _flush_incomplete_response(self):
        """Flush incomplete response data from buffer"""
        if len(self.receive_buffer) > 0:
            raw = bytes(self.receive_buffer[:])
            self.receive_buffer.clear()
            hex_str = ' '.join(f'{b:02X}' for b in raw)
            self.log_print(f"[RX] Incomplete response flushed: {hex_str}")
        if len(self.response_buffer) > 0:
            raw = bytes(self.response_buffer[:])
            self.response_buffer.clear()
            hex_str = ' '.join(f'{b:02X}' for b in raw)
            self.log_print(f"[RX] Response buffer flushed: {hex_str}")

    def _clear_receive_buffer(self):
        """Clear receive buffer before sending new command"""
        if len(self.receive_buffer) > 0:
            self.receive_buffer.clear()
            self.log_print("[INFO] Receive buffer cleared")
        if len(self.response_buffer) > 0:
            self.response_buffer.clear()
            self.log_print("[INFO] Response buffer cleared")

    def _display_can_frame(self, frame: bytes, is_extended: bool):
        """Display received CAN frame"""
        ctrl_byte = frame[1]
        data_len = ctrl_byte & 0x0F
        is_remote = (ctrl_byte & 0x10) != 0

        if is_extended:
            # Extended: AA + ctrl + 4-byte ID + data + 55
            id_bytes = frame[2:6]
            can_id = (id_bytes[3] << 24) | (id_bytes[2] << 16) | (id_bytes[1] << 8) | id_bytes[0]
            data = frame[6:6 + data_len]
        else:
            # Standard: AA + ctrl + 2-byte ID + data + 55
            id_bytes = frame[2:4]
            can_id = (id_bytes[1] << 8) | id_bytes[0]
            data = frame[4:4 + data_len]

        id_hex = f"{can_id:08X}" if is_extended else f"{can_id:04X}"
        data_hex = ' '.join(f'{b:02X}' for b in data) if data_len > 0 else "None"

        frame_type = "Remote" if is_remote else "Data"
        frame_format = "Extended" if is_extended else "Standard"
        self.log_print(f"[RX] CAN RX [{frame_format}/{frame_type}] ID:{id_hex} Data:{data_hex}")

    def _receive_can_frames(self):
        """Receive CAN frames via DLL"""
        if not self.can_dll:
            return

        # Define VCI_Receive function prototype
        # VCI_Receive(DevType, DevIndex, CANIndex, pReceive, Len, WaitTime)
        try:
            VCI_Receive = self.can_dll.VCI_Receive
            VCI_Receive.argtypes = [c_int, c_int, c_int, POINTER(VCI_CAN_OBJ), c_int, c_int]
            VCI_Receive.restype = c_int

            # Prepare receive buffer
            receive_array = (VCI_CAN_OBJ * 10)()

            while self.receive_running:
                # Receive CAN frames, wait 100ms
                ret = VCI_Receive(VCI_USB_CAN2, 0, 0, receive_array, 10, 100)
                if ret > 0:
                    for i in range(ret):
                        obj = receive_array[i]
                        self._display_received_can_obj(obj)
                elif ret == 0:
                    # No data, continue waiting
                    pass
                else:
                    # Error
                    if self.receive_running:
                        self.log_print("[ERR] CAN receive error")
                    break
        except Exception as e:
            if self.receive_running:
                self.log_print(f"[ERR] CAN receive error: {str(e)}")

    def _display_received_can_obj(self, obj):
        """Display received CAN object"""
        can_id = obj.ID
        is_extended = obj.ExternFlag
        data_len = obj.DataLen
        is_remote = obj.RemoteFlag

        # Extract data
        data = bytes([obj.Data[i] for i in range(data_len)]) if data_len > 0 else b''

        id_hex = f"{can_id:08X}" if is_extended else f"{can_id:04X}"
        data_hex = ' '.join(f'{b:02X}' for b in data) if data_len > 0 else "None"

        frame_type = "Remote" if is_remote else "Data"
        frame_format = "Extended" if is_extended else "Standard"
        self.log_print(f"[RX] CAN RX [{frame_format}/{frame_type}] ID:{id_hex} Data:{data_hex}")

    def _display_uimessage_frame(self, frame: bytes):
        """Display received UIMessage frame"""
        station_id = frame[1]
        control_word = frame[2]
        data_len = frame[3]
        data_bytes = frame[4:12]  # 8-byte data area

        # Determine frame type (bit7: 0=ACK, 1=Request)
        is_request = (control_word & 0x80) != 0
        cmd_code = control_word & 0x7F

        # Find command name
        cmd_name = self._get_cmd_name(cmd_code)

        # Format data - show hex and decimal
        if data_len > 0:
            actual_data = data_bytes[:data_len]
            data_hex = ' '.join(f'{b:02X}' for b in actual_data)

            # Try to interpret as integers
            data_decimal = self._format_data_decimal(actual_data, data_len)
            if data_decimal:
                data_display = f"{data_hex} ({data_decimal})"
            else:
                data_display = data_hex
        else:
            data_display = "No data"

        # Show raw frame first
        frame_hex = ' '.join(f'{b:02X}' for b in frame)
        self.log_print(f"[RX] Serial RX: {frame_hex}")

        # Then show parsed content
        self.log_print(f"   -- Node ID:{station_id} Cmd:{cmd_name}(0x{cmd_code:02X}) DataLen:{data_len} Data:{data_display}")

    def _format_data_decimal(self, data: bytes, data_len: int) -> str:
        """Convert data bytes to decimal display"""
        if data_len == 0:
            return ""

        # Little-endian parsing
        if data_len == 1:
            return str(data[0])
        elif data_len == 2:
            value = data[0] | (data[1] << 8)
            return str(value)
        elif data_len == 3:
            value = data[0] | (data[1] << 8) | (data[2] << 16)
            # Handle signed numbers
            if value >= 0x800000:
                value -= 0x1000000
            return str(value)
        elif data_len >= 4:
            value = int.from_bytes(data[:4], byteorder='little', signed=True)
            return str(value)
        return ""

    def _get_cmd_name(self, cmd_code: int) -> str:
        """Get command name from control word"""
        CMD_NAMES = {
            0x01: "PP", 0x06: "IC", 0x07: "IE", 0x0B: "ML", 0x0C: "SN",
            0x0F: "ER", 0x10: "MT", 0x11: "MS", 0x15: "MO", 0x16: "BG",
            0x17: "ST", 0x18: "MF", 0x19: "AC", 0x1A: "DC", 0x1B: "SS",
            0x1C: "SD", 0x1D: "JV", 0x1E: "SP", 0x1F: "PR", 0x20: "PA",
            0x21: "OG", 0x2D: "BL", 0x2E: "DV", 0x34: "IL", 0x35: "TG",
            0x37: "DI", 0x3D: "QE", 0x5A: "RT", 0x7E: "SY",
        }
        return CMD_NAMES.get(cmd_code, f"0x{cmd_code:02X}")

    def scan_com_ports(self):
        display = ["CAN0"]
        self.port_map["CAN0"] = "CANAL0"
        self.dev_type_map["CAN0"] = "CANALYST2"
        for p in serial.tools.list_ports.comports():
            name = p.device
            desc = p.description.upper()
            # Simplified: show port and short identifier
            if "CH340" in desc or "CH341" in desc:
                show = f"{name} [USB-CAN]"
            else:
                show = name
            display.append(show)
            self.port_map[show] = name
            self.dev_type_map[show] = "USBCANA" if "CH340" in desc or "CH341" in desc else "OTHER"
        self.com_combobox["values"] = display

    def select_com(self, e):
        sel = self.com_var.get()
        self.real_com = self.port_map[sel]
        self.cur_dev_type = self.dev_type_map[sel]

    def setup_connect_frame(self):
        f = ttk.LabelFrame(self.root, text="Connection", padding=(8, 4))
        f.pack(padx=8, pady=3, fill=tk.X)
        f.columnconfigure(1, weight=1)
        f.columnconfigure(3, weight=1)
        ttk.Label(f, text="Device:").grid(row=0, column=0, padx=(0, 3), pady=2, sticky="w")
        self.com_var = tk.StringVar()
        self.com_combobox = ttk.Combobox(f, textvariable=self.com_var, width=12)
        self.com_combobox.grid(row=0, column=1, padx=(0, 6), pady=2, sticky="ew")
        self.com_combobox.bind("<<ComboboxSelected>>", self.select_com)
        ttk.Label(f, text="CAN Bit Rate:").grid(row=0, column=2, padx=(0, 6), pady=2, sticky="w")
        self.can_baud_var = tk.StringVar(value="500kbps")
        ttk.Combobox(f, textvariable=self.can_baud_var, values=list(self.baudrate_map.keys()), width=9).grid(row=0, column=3, padx=(0, 6), pady=2, sticky="ew")
        ttk.Label(f, text="Serial Baud Rate:").grid(row=0, column=4, padx=(0, 6), pady=2, sticky="w")
        self.ser_baud_var = tk.StringVar(value="9600")
        ttk.Combobox(f, textvariable=self.ser_baud_var, values=self.serial_baud_list, width=8).grid(row=0, column=5, padx=(0, 6), pady=2, sticky="ew")
        self.connect_btn = ttk.Button(f, text="Connect", command=self.toggle_connect, width=10)
        self.connect_btn.grid(row=0, column=6, padx=(0, 4), pady=2, sticky="e")

        self.scan_com_ports()


    def setup_send_frame(self):
        f = ttk.LabelFrame(self.root, text="Send", padding=(8, 4))
        f.pack(padx=8, pady=3, fill=tk.X)
        f.columnconfigure(2, weight=1)
        # CAN send row
        ttk.Label(f, text="Type:").grid(row=0, column=0, padx=(0, 4), pady=2, sticky="w")
        self.ft_var = tk.StringVar(value="Extended")
        ttk.Radiobutton(f, text="Standard", variable=self.ft_var, value="Standard").grid(row=0, column=1, padx=(0, 2), pady=2)
        ttk.Radiobutton(f, text="Extended", variable=self.ft_var, value="Extended").grid(row=0, column=2, padx=(0, 4), pady=2, sticky="w")
        ttk.Label(f, text="ID:").grid(row=0, column=3, padx=(0, 2), pady=2, sticky="w")
        self.id_var = tk.StringVar(value="04290095")
        ttk.Entry(f, textvariable=self.id_var, width=11).grid(row=0, column=4, padx=(0, 4), pady=2, sticky="ew")
        ttk.Label(f, text="Data:").grid(row=0, column=5, padx=(0, 2), pady=2, sticky="w")
        self.data_var = tk.StringVar(value="11 22 33 44")
        ttk.Entry(f, textvariable=self.data_var, width=16).grid(row=0, column=6, padx=(0, 4), pady=2, sticky="ew")
        self.send_can_btn = ttk.Button(f, text="Send CAN Command", command=self.send_can, state=tk.DISABLED, width=20)
        self.send_can_btn.grid(row=0, column=7, padx=(0, 4), pady=2)
        # String command row
        ttk.Label(f, text="String:").grid(row=1, column=0, padx=(0, 4), pady=2, sticky="w")
        self.string_var = tk.StringVar(value="SP1000;BG;")
        self.string_entry = ttk.Entry(f, textvariable=self.string_var)
        self.string_entry.grid(row=1, column=1, columnspan=4, padx=(0, 4), pady=2, sticky="ew")
        ttk.Label(f, text="Node:").grid(row=1, column=5, padx=(0, 2), pady=2, sticky="w")
        self.station_var = tk.StringVar(value="5")
        ttk.Entry(f, textvariable=self.station_var, width=16).grid(row=1, column=6, padx=(0, 4), pady=2)
        self.send_string_btn = ttk.Button(f, text="Send String Command", command=self.send_string_command, state=tk.DISABLED, width=20)
        self.send_string_btn.grid(row=1, column=7, padx=(0, 4), pady=2)
        # HEX data row
        ttk.Label(f, text="HEX:").grid(row=2, column=0, padx=(0, 4), pady=2, sticky="w")
        self.hex_var = tk.StringVar()
        self.hex_entry = ttk.Entry(f, textvariable=self.hex_var)
        self.hex_entry.grid(row=2, column=1, columnspan=4, padx=(0, 4), pady=2, sticky="ew")
        ttk.Label(f, text="Send:").grid(row=2, column=5, padx=(0, 2), pady=2, sticky="w")
        self.send_hex_btn = ttk.Button(f, text="Send", command=self.send_hex_data, state=tk.DISABLED, width=16)
        self.send_hex_btn.grid(row=2, column=6, padx=(0, 4), pady=2)
        self.send_hex_crc_btn = ttk.Button(f, text="Send with CRC", command=self.send_hex_with_crc, state=tk.DISABLED, width=20)
        self.send_hex_crc_btn.grid(row=2, column=7, columnspan=2, padx=(0, 4), pady=2)


    def setup_display(self):
        f = ttk.Frame(self.root, padding=0)
        f.pack(padx=8, pady=3, fill=tk.BOTH, expand=1)
        top_row = ttk.Frame(f)
        top_row.pack(fill=tk.X)
        ttk.Label(top_row, text="Logs").pack(side=tk.LEFT, padx=(2, 0))
        ttk.Button(top_row, text="Clear", command=self.clear_log, width=6).pack(side=tk.RIGHT, padx=(0, 2))
        self.log = scrolledtext.ScrolledText(f, font=("Consolas", 10), height=15)
        self.log.pack(fill=tk.BOTH, expand=1, pady=(2, 0))
        self.log.tag_config("time", foreground="gray")
        self.log.tag_config("ok", foreground="green")
        self.log.tag_config("error", foreground="red")
        self.log.tag_config("rx", foreground="blue")
        self.log.tag_config("info", foreground="orange")
        self.log.tag_config("normal", foreground="black")
        self.log.config(state=tk.DISABLED)


    def clear_log(self):
        """Clear log display"""
        try:
            self.log.config(state=tk.NORMAL)
            self.log.delete("1.0", tk.END)
            self.log.config(state=tk.DISABLED)
        except Exception as e:
            logger.error(f"Error clearing log: {e}")

    def log_print(self, msg):
        """Thread-safe log printing with line limit"""
        try:
            # Use after() for thread-safe GUI updates
            self.root.after(0, self._log_print_impl, msg)
        except Exception as e:
            logger.error(f"Error logging message: {e}")

    def _log_print_impl(self, msg):
        """Internal log printing implementation (must run on main thread)"""
        try:
            self.log.config(state=tk.NORMAL)

            # Add message
            self.log.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")

            # Limit log lines
            line_count = int(self.log.index('end-1c').split('.')[0])
            if line_count > MAX_LOG_LINES:
                self.log.delete("1.0", f"{line_count - MAX_LOG_LINES}.0")

            self.log.see(tk.END)
            self.log.config(state=tk.DISABLED)
        except Exception as e:
            logger.error(f"Error in log_print_impl: {e}")

    class VCI_CAN_INIT_CONFIG(Structure):
        _fields_ = [
            ("AccCode", c_uint), ("AccMask", c_uint), ("Reserved", c_uint),
            ("Filter", c_ubyte), ("Timing0", c_ubyte), ("Timing1", c_ubyte), ("Mode", c_ubyte)
        ]

    def send_string_command(self):
        """Send string command - standalone implementation"""
        try:
            cmd_str = self.string_var.get().strip()
            if not cmd_str:
                messagebox.showwarning("Hint", "Please enter string command, e.g. SP1000;PR-400;BG;")
                return

            # Check connection status
            if not self.is_connected:
                messagebox.showwarning("Hint", "Please connect serial port first")
                return

            # Get node ID
            try:
                station = int(self.station_var.get())
                if not (0 <= station <= 126):
                    raise ValueError()
            except ValueError:
                messagebox.showwarning("Hint", "Node ID must be 0-126")
                return

            # Clear any old response data from buffer before sending
            self._clear_receive_buffer()

            # Parse commands
            commands = self._parse_string_commands(cmd_str, station)
            if not commands:
                messagebox.showwarning("Hint", "No valid command")
                return

            # Show parse result
            self.log_print(f"[INFO] Parsed {len(commands)} command(s):")
            for i, cmd_info in enumerate(commands, 1):
                if cmd_info.get('passthrough'):
                    self.log_print(f"  {i}. {cmd_info['command']} (passthrough)")
                else:
                    param_str = f" (Param: {cmd_info['parameter']})" if cmd_info['parameter'] is not None else ""
                    self.log_print(f"  {i}. {cmd_info['command']}{param_str}")

            # Send commands
            for cmd_info in commands:
                # Passthrough commands: only supported in serial gateway mode
                if cmd_info.get('passthrough'):
                    if self.cur_dev_type == "CANALYST2" and self.can_dll:
                        self.log_print(f"[ERR] Passthrough not supported in CAN direct mode, skip: {cmd_info['command']}")
                    else:
                        raw_bytes = cmd_info['frame']
                        self.ser.write(raw_bytes)
                        frame_hex = raw_bytes.hex(' ').upper()
                        self.log_print(f"[OK] Passthrough sent: {cmd_info['command']} -> {frame_hex}")
                        # Wait for response
                        self._wait_for_string_response(timeout=1.0)
                    continue

                # Known commands (CANALYST2 uses MODBUS frames)
                if self.cur_dev_type == "CANALYST2":
                    frame = cmd_info['frame']
                    self.ser.write(frame)
                    frame_hex = frame.hex(' ').upper()
                    self.log_print(f"[OK] Sent: {cmd_info['command']} -> {frame_hex}")
                    # Wait for response
                    self._wait_for_string_response(timeout=1.0)
                else:
                    # Serial gateway mode - AA...55 frame format
                    modbus_frame = cmd_info['frame']
                    # Wrap as gateway frame: AA + MODBUS(without CRC) + CRC16 + 55
                    gateway_frame = self._build_gateway_frame(modbus_frame)
                    self.ser.write(gateway_frame)
                    frame_hex = gateway_frame.hex(' ').upper()
                    self.log_print(f"[OK] Sent: {cmd_info['command']} -> {frame_hex}")
                    # Wait for response
                    self._wait_for_string_response(timeout=1.0)

        except Exception as e:
            messagebox.showerror("Error", f"Send failed: {str(e)}")

    def _parse_string_commands(self, command_str: str, station_id: int) -> list:
        """Parse string commands - full implementation"""
        # Command code mapping - from Manual_UIM342_V4.10.pdf Section 5.0
        CMD_CODES = {
            # Protocol & System
            'PP': 0x01,   # Protocol Parameters (sub-index, 16-bit data)
            'IC': 0x06,   # Initial Configuration (sub-index, 16-bit data)
            'IE': 0x07,   # Information Enable (sub-index, 16-bit data)
            'ML': 0x0B,   # Get Model (no data)
            'SN': 0x0C,   # Get Serial Number (no data)
            'ER': 0x0F,   # Error Report (sub-index, 8-bit data)
            'QE': 0x3D,   # Quadrature Encoder (sub-index, 16-bit data)
            'SY': 0x7E,   # System Operation (sub-index, no data, no ACK)

            # Motor Driver
            'MT': 0x10,   # Motor Driver (sub-index, 16-bit data)
            'MO': 0x15,   # Motor On/Off (8-bit data)

            # Motion Control
            'BG': 0x16,   # Begin Motion (no data)
            'ST': 0x17,   # Stop Motion (no data)
            'MF': 0x18,   # Motion Parameter Frame (8-bit data)
            'AC': 0x19,   # Acceleration (32-bit data)
            'DC': 0x1A,   # Deceleration (32-bit data)
            'SS': 0x1B,   # Cut-in Speed (32-bit data)
            'SD': 0x1C,   # Stop Deceleration (32-bit data)
            'JV': 0x1D,   # Jog Velocity (32-bit signed data)
            'SP': 0x1E,   # PTP Speed (32-bit data)
            'PR': 0x1F,   # Position Relative (32-bit signed data)
            'PA': 0x20,   # Position Absolute (32-bit signed data)
            'OG': 0x21,   # Set Origin (no data)
            'BL': 0x2D,   # Backlash Compensation (16-bit data)
            'MS': 0x11,   # Motion Status (sub-index, 8-bit data)
            'DV': 0x2E,   # Desired Values (sub-index, 32-bit data)

            # Input/Output
            'IL': 0x34,   # Input Logic (sub-index, 16-bit data)
            'TG': 0x35,   # Trigger (sub-index, 16-bit data)
            'DI': 0x37,   # Digital I/O (special format)

            # Notification
            'RT': 0x5A,   # Real-Time Inform (auto-sent)
        }

        # 32-bit parameter commands (signed integer)
        CMD_32BIT = {'PA', 'PR', 'JV', 'SP', 'AC', 'DC', 'SS', 'SD', 'DV'}

        # 16-bit parameter commands (sub-index + 16-bit data, little-endian)
        CMD_16BIT = {'PP', 'IC', 'IE', 'QE', 'MT', 'BL', 'IL', 'TG'}

        # No-data commands
        CMD_NO_DATA = {'ML', 'SN', 'BG', 'ST', 'OG', 'SY', 'RT'}

        # Special format commands
        CMD_SPECIAL = {'DI', 'MF', 'MS', 'ER'}

        frames = []
        parts = [p.strip() for p in command_str.split(';') if p.strip()]

        for part in parts:
            # Match format: 2-3 letters + optional numeric param
            match = re.match(r'^([A-Z]{2,3})(-?\d+)?$', part.upper())
            if not match:
                continue

            cmd_name = match.group(1)
            param_str = match.group(2)

            # === NEW: Passthrough for unknown commands ===
            # If command is not in our known map, treat the raw ASCII + ';' as hex bytes
            # and send directly without MODBUS framing / Node association
            if cmd_name not in CMD_CODES:
                raw_text = part.upper() + ';'  # 补回被 split 吃掉的 ;
                raw_bytes = raw_text.encode('ascii')
                frames.append({
                    'command': raw_text,
                    'parameter': None,
                    'frame': bytes(raw_bytes),
                    'passthrough': True,
                })
                continue

            param = None
            if param_str:
                try:
                    param = int(param_str)
                except ValueError:
                    continue

            # Build data bytes
            data_bytes = []

            if cmd_name == 'DI':
                # DI command special handling (Manual Section 5.28)
                # DI; or DI0; - read digital I/O (no param)
                # DI1-DI255; - read specific channel (1 byte: channel)
                # DI256+; - control output (3 bytes: index_low, index_high, control_value)
                if param is None:
                    data_bytes = []
                elif param <= 255:
                    data_bytes = [param & 0xFF]
                else:
                    data_bytes = [param & 0xFF, (param >> 8) & 0xFF, 0x01]
            elif cmd_name == 'MF':
                # MF - Motion Parameter Frame (Manual Section 5.13)
                # Select motion param group: 0=normal, 2-7=input trigger
                if param is not None:
                    data_bytes = [param & 0xFF]
            elif cmd_name == 'MS':
                # MS[i] - Motion Status (Manual Section 5.24)
                # d0=0: get status, d0=1: get velocity and position, d0=0+d1=0: clear flags
                if param is not None:
                    data_bytes = [param & 0xFF]
            elif cmd_name == 'ER':
                # ER[i] - Error Report (Manual Section 5.6)
                # i=0: get latest error, i=1: clear latest error
                if param is not None:
                    data_bytes = [param & 0xFF]
            elif cmd_name in CMD_NO_DATA:
                # No-data commands
                data_bytes = []
            elif cmd_name in CMD_32BIT:
                # 32-bit signed integer - little-endian
                if param is not None:
                    data_bytes = list(int(param).to_bytes(4, byteorder='little', signed=True))
            elif cmd_name in CMD_16BIT:
                # 16-bit command: sub-index(d0) + 16-bit data(d2:d1 little-endian)
                # Format: [index, data_low, data_high]
                if param is not None:
                    data_bytes = [0x00, param & 0xFF, (param >> 8) & 0xFF]
            else:
                # Single byte param (e.g. MO)
                if param is not None:
                    data_bytes = [param & 0xFF]

            # Build MODBUS frame
            cmd_code = CMD_CODES[cmd_name]
            frame = self._build_modbus_frame(station_id, cmd_code, data_bytes)

            frames.append({
                'command': cmd_name,
                'parameter': param,
                'frame': frame,
            })

        return frames

    def _build_modbus_frame(self, station_id: int, cmd_code: int, data_bytes: list) -> bytes:
        """Build standard MODBUS frame"""
        # Control word: bit7=1 request ACK, bit6:0=function code
        control_word = cmd_code | 0x80

        # Build frame
        frame = bytes([station_id, control_word, len(data_bytes)] + data_bytes)

        # Calculate and append CRC16
        crc = crc16_modbus(frame)
        frame = frame + crc

        return frame

    def _build_gateway_frame(self, modbus_frame: bytes) -> bytes:
        """Build serial gateway UIMessage frame (16 bytes fixed)"""
        # Extract info from MODBUS frame
        station_id = modbus_frame[0]
        control_word = modbus_frame[1]
        data_len = modbus_frame[2]
        data_bytes = modbus_frame[3:3 + data_len]

        # Build 8-byte data area d0-d7 (zero-padded)
        padded_data = data_bytes + bytes(8 - len(data_bytes))

        # Frame (without CRC): AA + ID + CW + DL + d0-d7 + Aux(0x00)
        frame_without_crc = bytes([0xAA, station_id, control_word, data_len]) + padded_data + bytes([0x00])

        # Calculate CRC16 (range: ID+CW+DL+d0-d7+Aux, i.e. all bytes after AA)
        crc_data = frame_without_crc[1:]  # Remove AA
        crc = crc16_modbus(crc_data)

        # Complete frame: AA + ID + CW + DL + d0-d7 + Aux + CRC(R0,R1) + CC
        gateway_frame = frame_without_crc + crc + bytes([0xCC])

        return gateway_frame

    def send_hex_data(self):
        """Send HEX data (no CRC)"""
        try:
            hex_str = self.hex_var.get().strip()
            if not hex_str:
                messagebox.showwarning("Hint", "Please enter HEX data")
                return

            data_bytes = parse_hex_input(hex_str)
            if not data_bytes:
                messagebox.showwarning("Hint", "No valid HEX data")
                return

            # Clear any old response data from buffer before sending
            self._clear_receive_buffer()

            # Send raw bytes
            frame = bytes(data_bytes)
            if self.cur_dev_type == "CANALYST2":
                self.log_print("[ERR] CANALYST2 does not support HEX send")
                return

            self.ser.write(frame)
            self.log_print(f"[OK] HEX sent: {frame.hex(' ').upper()}")

            # Wait for response
            self._wait_for_string_response(timeout=1.0)

        except Exception as e:
            messagebox.showerror("Error", f"Send failed: {str(e)}")

    def send_hex_with_crc(self):
        """Send HEX data (with CRC)"""
        try:
            hex_str = self.hex_var.get().strip()
            if not hex_str:
                messagebox.showwarning("Hint", "Please enter HEX data")
                return

            data_bytes = parse_hex_input(hex_str)
            if not data_bytes:
                messagebox.showwarning("Hint", "No valid HEX data")
                return

            # Clear any old response data from buffer before sending
            self._clear_receive_buffer()

            # Calculate and append CRC
            frame = bytes(data_bytes)
            crc = crc16_modbus(frame)
            frame_with_crc = frame + crc

            if self.cur_dev_type == "CANALYST2":
                self.log_print("[ERR] CANALYST2 does not support HEX send")
                return

            self.ser.write(frame_with_crc)
            self.log_print(f"[OK] HEX sent (with CRC): {frame_with_crc.hex(' ').upper()}")

            # Wait for response
            self._wait_for_string_response(timeout=1.0)

        except Exception as e:
            messagebox.showerror("Error", f"Send failed: {str(e)}")

    def send_can(self):
        try:
            cid = int(self.id_var.get(), 16)
            data = [int(x, 16) for x in self.data_var.get().split()] if self.data_var.get() else []
            is_ext = 1 if self.ft_var.get() == "Extended" else 0
            # Format data as hex string
            data_hex = " ".join(f"{b:02X}" for b in data) if data else "None"

            # Clear any old response data from buffer before sending
            self._clear_receive_buffer()

            if self.cur_dev_type == "CANALYST2":
                obj = VCI_CAN_OBJ()
                obj.ID = cid
                obj.ExternFlag = is_ext
                obj.DataLen = len(data)
                for i in range(len(data)):
                    obj.Data[i] = data[i]
                self.can_dll.VCI_Transmit(VCI_USB_CAN2, 0, 0, byref(obj), 1)
                id_hex = f"{cid:08X}" if is_ext else f"{cid:04X}"
                self.log_print(f"[OK] CAN sent | ID: {id_hex} | Data: {data_hex}")
            else:
                frame = [0xAA, 0xC0 | (0x20 if is_ext else 0) | len(data)]
                if not is_ext:
                    frame += [cid & 0xFF, (cid >> 8) & 0xFF]
                else:
                    frame += [cid & 0xFF, (cid >> 8) & 0xFF, (cid >> 16) & 0xFF, (cid >> 24) & 0xFF]
                frame += data
                frame.append(0x55)
                self.ser.write(bytes(frame))
                id_hex = f"{cid:08X}" if is_ext else f"{cid:04X}"
                self.log_print(f"[OK] CAN sent | ID: {id_hex} | Data: {data_hex}")
                # Wait for response after sending
                self._wait_for_string_response(timeout=1.0)
        except Exception as e:
            messagebox.showerror("Error", f"Send failed: {str(e)}")
    
    def _wait_for_response(self, timeout=0.5):
        """Wait for response after sending a command (legacy method)"""
        self._wait_for_string_response(timeout=timeout)

    def _wait_for_string_response(self, timeout=2.0):
        """Wait for complete string/hex command response (CC...FF frame)"""
        self.waiting_for_response = True
        self.response_buffer.clear()
        start_time = time.time()
        last_data_time = start_time
        SILENCE_TIMEOUT = 0.2  # 200ms of silence means response is complete

        while time.time() - start_time < timeout:
            if self.ser and self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.response_buffer.extend(data)
                last_data_time = time.time()

                # Check if we have a complete CC...FF frame
                if self._has_complete_ccff_frame():
                    # Parse the complete frame
                    self._parse_ccff_frame()
                    break
            else:
                # No new data, check if we have a complete response
                if len(self.response_buffer) > 0:
                    # Check if we have a complete CC...FF frame
                    if self._has_complete_ccff_frame():
                        self._parse_ccff_frame()
                        break

                    # If we have data but no new data for a while, response might be complete
                    silence_duration = time.time() - last_data_time
                    if silence_duration > SILENCE_TIMEOUT:
                        # Response seems complete (no new data for SILENCE_TIMEOUT)
                        # Try to parse any remaining data
                        self._parse_response_buffer()
                        break

            time.sleep(0.005)  # 5ms polling

        self.waiting_for_response = False

        # Flush any remaining data if timeout
        self._handle_timeout_response()

    def _has_complete_ccff_frame(self) -> bool:
        """Check if response buffer contains a complete CC...FF frame"""
        if len(self.response_buffer) < 5:
            return False

        # Look for CC...FF pattern
        # Frame format: CC [data...] FF
        if self.response_buffer[0] == 0xCC:
            # Check if there's a FF at the end
            for i in range(4, len(self.response_buffer)):
                if self.response_buffer[i] == 0xFF:
                    return True
        return False

    def _parse_ccff_frame(self):
        """Parse CC...FF frame from response buffer"""
        if len(self.response_buffer) < 5:
            return

        # Find the FF position
        ff_pos = -1
        for i in range(4, len(self.response_buffer)):
            if self.response_buffer[i] == 0xFF:
                ff_pos = i
                break

        if ff_pos == -1:
            return

        # Extract the complete frame (CC...FF)
        frame = bytes(self.response_buffer[:ff_pos + 1])
        del self.response_buffer[:ff_pos + 1]

        # Parse the frame
        self._parse_uimessage_response(frame)

    def _parse_response_buffer(self):
        """Parse response buffer for complete frames"""
        while len(self.response_buffer) >= 3:
            # Check if we have a complete MODBUS-like frame
            data_len = self.response_buffer[2]

            # Validate data length (should be reasonable)
            if data_len > 128:
                # Invalid data length, skip this byte and try again
                self.response_buffer.pop(0)
                continue

            expected_len = 3 + data_len + 2  # +2 for CRC

            if len(self.response_buffer) < expected_len:
                break  # Wait for more data

            # Extract complete frame
            frame = bytes(self.response_buffer[:expected_len])
            del self.response_buffer[:expected_len]

            # Check if this is a UIMessage response (CC...FF format)
            # This takes priority over other parsing
            if len(frame) >= 5 and frame[0] == 0xCC and frame[-1] == 0xFF:
                # Parse as UIMessage response
                self._parse_uimessage_response(frame)
            # Check if this is a response to a known command (in string_commands.py)
            elif self._is_known_command_response(frame):
                # Parse and display with full parsing
                station_id = frame[0]
                control_word = frame[1]
                self._display_response_frame(frame, station_id, control_word, data_len)
            else:
                # Unknown command - just display raw data
                self._display_raw_response(frame)

            # Update last response time
            self.last_response_time = time.time()

    def _is_known_command_response(self, frame: bytes) -> bool:
        """Check if response is from a known command in string_commands.py"""
        if len(frame) < 3:
            return False

        # Get command code from control word (bit 6:0)
        control_word = frame[1]
        cmd_code = control_word & 0x7F

        # Known command codes from string_commands.py CMD_CODES
        KNOWN_CMD_CODES = {
            0x01,   # PP - Protocol Parameters
            0x06,   # IC - Initial Configuration
            0x07,   # IE - Information Enable
            0x0B,   # ML - Get Model
            0x0C,   # SN - Get Serial Number
            0x0F,   # ER - Error Report
            0x3D,   # QE - Quadrature Encoder
            0x7E,   # SY - System Operation
            0x10,   # MT - Motor Driver
            0x15,   # MO - Motor On/Off
            0x16,   # BG - Begin Motion
            0x17,   # ST - Stop Motion
            0x18,   # MF - Motion Parameter Frame
            0x19,   # AC - Acceleration
            0x1A,   # DC - Deceleration
            0x1B,   # SS - Cut-in Speed
            0x1C,   # SD - Stop Deceleration
            0x1D,   # JV - Jog Velocity
            0x1E,   # SP - PTP Speed
            0x1F,   # PR - Position Relative
            0x20,   # PA - Position Absolute
            0x21,   # OG - Set Origin
            0x2D,   # BL - Backlash Compensation
            0x11,   # MS - Motion Status
            0x2E,   # DV - Desired Values
            0x34,   # IL - Input Logic
            0x35,   # TG - Trigger
            0x37,   # DI - Digital I/O
            0x5A,   # RT - Real-Time Inform
        }

        return cmd_code in KNOWN_CMD_CODES

    def _display_raw_response(self, frame: bytes):
        """Display raw response for unknown commands (no parsing)"""
        # Show raw frame hex
        frame_hex = ' '.join(f'{b:02X}' for b in frame)
        self.log_print(f"[RX] Response (raw): {frame_hex}")

        # Also show as ASCII if printable
        try:
            ascii_str = frame.decode('ascii', errors='replace')
            # Check if mostly printable
            printable_count = sum(1 for c in ascii_str if c.isprintable() or c in '\r\n\t')
            if printable_count > len(ascii_str) * 0.7:
                self.log_print(f"   -- ASCII: {ascii_str}")
        except:
            pass

    def _parse_uimessage_response(self, frame: bytes) -> bool:
        """
        Parse UIMessage response frame (CC...FF format)
        Frame format: CC [header] [data_bytes...] FF
        - 7 bytes total: CC + 2 header + 3 data + FF -> 16-bit value
        - 9 bytes total: CC + 2 header + 5 data + FF -> 32-bit value
        Returns True if frame is valid and parsed
        """
        # Must start with CC and end with FF
        if len(frame) < 5 or frame[0] != 0xCC or frame[-1] != 0xFF:
            return False

        # Determine data length based on total frame length
        # Total length = CC(1) + header(2) + data + FF(1)
        # data_len = total_len - 4
        total_len = len(frame)
        data_len = total_len - 4  # Subtract CC, header (2 bytes), and FF

        # Extract header (2 bytes after CC) and data (bytes before FF)
        header = frame[1:3]
        data_bytes = frame[3:-1]

        # Verify data length matches
        if len(data_bytes) != data_len:
            return False

        # Format frame hex with parsed value in parentheses
        frame_hex = ' '.join(f'{b:02X}' for b in frame)

        if data_len == 3:
            # 3-byte data: 16-bit value
            value = self._parse_3byte_data(data_bytes)
            self.log_print(f"[RX] Response: {frame_hex} ({value})")
            return True
        elif data_len == 5:
            # 5-byte data: 32-bit value
            value = self._parse_5byte_data(data_bytes)
            self.log_print(f"[RX] Response: {frame_hex} ({value})")
            return True
        else:
            # Unknown data length, display raw
            self.log_print(f"[RX] Response: {frame_hex}")
            return True

    def _parse_3byte_data(self, data: bytes) -> int:
        """
        Parse 3-byte data using bit-field extraction (signed 16-bit)
        data[0]: bit1,bit0 -> result bit15,bit14
        data[1]: bit6~bit0 -> result bit13~bit7
        data[2]: bit6~bit0 -> result bit6~bit0
        bit7 in each byte is padding (ignored)
        """
        byte0, byte1, byte2 = data[0], data[1], data[2]

        # Extract valid bits (mask out bit7)
        b0_valid = byte0 & 0x03  # bit1,bit0
        b1_valid = byte1 & 0x7F  # bit6~bit0
        b2_valid = byte2 & 0x7F  # bit6~bit0

        # Combine into 16-bit value
        unsigned = (b0_valid << 14) | (b1_valid << 7) | b2_valid

        # Convert to signed 16-bit (two's complement)
        if unsigned >= 0x8000:
            return unsigned - 0x10000
        return unsigned

    def _parse_5byte_data(self, data: bytes) -> int:
        """
        Parse 5-byte data using bit-field extraction (signed 32-bit)
        Each byte's bit7 is padding (ignored)
        byte0 (bit3~bit0) -> D31~D28 (4 bits)
        byte1 (bit6~bit0) -> D27~D21 (7 bits)
        byte2 (bit6~bit0) -> D20~D14 (7 bits)
        byte3 (bit6~bit0) -> D13~D7 (7 bits)
        byte4 (bit6~bit0) -> D6~D0 (7 bits)
        Total: 4 + 7 + 7 + 7 + 7 = 32 bits
        """
        byte0, byte1, byte2, byte3, byte4 = data[0], data[1], data[2], data[3], data[4]

        # Extract valid bits (mask out bit7)
        b0_valid = byte0 & 0x0F  # bit3~bit0 (only 4 bits for MSB)
        b1_valid = byte1 & 0x7F  # bit6~bit0
        b2_valid = byte2 & 0x7F  # bit6~bit0
        b3_valid = byte3 & 0x7F  # bit6~bit0
        b4_valid = byte4 & 0x7F  # bit6~bit0

        # Combine into 32-bit value
        unsigned = (b0_valid << 28) | (b1_valid << 21) | (b2_valid << 14) | (b3_valid << 7) | b4_valid

        # Convert to signed 32-bit (two's complement)
        if unsigned >= 0x80000000:
            return unsigned - 0x100000000
        return unsigned

    def toggle_connect(self):
        """Toggle connection state with proper error handling"""
        if not self.is_connected:
            self._connect_device()
        else:
            self._disconnect_device()

    def _connect_device(self):
        """Connect to device with proper error handling"""
        try:
            # Validate inputs
            ser_baud = int(self.ser_baud_var.get())
            can_baud_str = self.can_baud_var.get()

            if ser_baud <= 0:
                raise ValueError("Invalid serial baud rate")

            if self.cur_dev_type == "CANALYST2":
                self._connect_can_device(can_baud_str)
            else:
                self._connect_serial_device(ser_baud)

            # Update UI state
            self.is_connected = True
            self.connect_btn.config(text="Disconnect")
            self._update_send_buttons_state(tk.NORMAL)

            # Start receive thread
            self.start_receive_thread()
            logger.info(f"Connected to {self.cur_dev_type}")

        except ValueError as e:
            logger.error(f"Invalid configuration: {e}")
            messagebox.showerror("Configuration Error", str(e))
        except Exception as e:
            logger.error(f"Connection failed: {e}", exc_info=True)
            messagebox.showerror("Connection Failed", str(e))
            self._cleanup_connection()

    def _connect_can_device(self, can_baud_str: str):
        """Connect to CAN device"""
        dll_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ControlCAN.dll")

        if not os.path.exists(dll_path):
            raise FileNotFoundError(f"CAN DLL not found: {dll_path}")

        self.can_dll = cdll.LoadLibrary(dll_path)

        # Open device
        open_ret = self.can_dll.VCI_OpenDevice(VCI_USB_CAN2, 0, 0)
        if open_ret != 1:
            self.can_dll = None
            raise RuntimeError(f"VCI_OpenDevice failed (ret={open_ret}), device may be occupied")

        # Initialize CAN
        cfg = self.VCI_CAN_INIT_CONFIG()
        cfg.AccCode = 0x80000000
        cfg.AccMask = 0xFFFFFFFF
        cfg.Filter = 0
        cfg.Mode = 0
        cfg.Timing0, cfg.Timing1 = self.can_baud_reg.get(can_baud_str, (0, 0x1C))

        init_ret = self.can_dll.VCI_InitCAN(VCI_USB_CAN2, 0, 0, byref(cfg))
        if init_ret != 1:
            self.can_dll.VCI_CloseDevice(VCI_USB_CAN2, 0)
            self.can_dll = None
            raise RuntimeError(f"VCI_InitCAN failed (ret={init_ret})")

        # Start CAN
        start_ret = self.can_dll.VCI_StartCAN(VCI_USB_CAN2, 0, 0)
        if start_ret != 1:
            self.can_dll.VCI_CloseDevice(VCI_USB_CAN2, 0)
            self.can_dll = None
            raise RuntimeError(f"VCI_StartCAN failed (ret={start_ret})")

        self.log_print(f"[OK] CAN0 connected ({can_baud_str})")

    def _connect_serial_device(self, ser_baud: int):
        """Connect to serial device"""
        if not self.real_com:
            raise ValueError("No serial port selected")

        self.ser = serial.Serial(
            self.real_com,
            ser_baud,
            timeout=0.1,
            write_timeout=0.1
        )
        self.log_print(f"[OK] {self.real_com} connected {ser_baud}")

    def _disconnect_device(self):
        """Disconnect from device with proper cleanup"""
        try:
            # Stop receive thread first
            self.stop_receive_thread()

            # Update UI state
            self.is_connected = False
            self.connect_btn.config(text="Connect")
            self._update_send_buttons_state(tk.DISABLED)

            # Close serial port
            if self.ser:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                except Exception as e:
                    logger.warning(f"Error closing serial port: {e}")
                finally:
                    self.ser = None

            # Close CAN device
            if self.can_dll:
                try:
                    self.can_dll.VCI_CloseDevice(VCI_USB_CAN2, 0)
                except Exception as e:
                    logger.warning(f"Error closing CAN device: {e}")
                finally:
                    self.can_dll = None

            # Clear buffers
            self._clear_all_buffers()
            self.log_print("[*] Disconnected")
            logger.info("Disconnected from device")

        except Exception as e:
            logger.error(f"Error during disconnect: {e}", exc_info=True)
            self.log_print(f"[ERR] Disconnect error: {str(e)}")

    def _cleanup_connection(self):
        """Cleanup connection resources after failed connect"""
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None

        if self.can_dll:
            try:
                self.can_dll.VCI_CloseDevice(VCI_USB_CAN2, 0)
            except:
                pass
            self.can_dll = None

    def _update_send_buttons_state(self, state: int):
        """Update send buttons state"""
        self.send_can_btn.config(state=state)
        self.send_string_btn.config(state=state)
        self.send_hex_btn.config(state=state)
        self.send_hex_crc_btn.config(state=state)

    def _clear_all_buffers(self):
        """Clear all buffers"""
        with self.buffer_lock:
            self.receive_buffer.clear()
        self.response_buffer.clear()

def on_closing(app):
    """Handle application closing"""
    logger.info("Application closing")
    app.stop_receive_thread()
    app._disconnect_device()
    app.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = CAN_GUI_Tool(root)
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(app))
    root.mainloop()
