# ////////////////////////////////////////////////////////////////////////////
# MIT License
#
# Copyright (c) [2022] UIROBOT
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Disclaimer: UIROBOT shall not be held responsible for any direct or indirect
# consequences resulting from the misuse of this software, including but not
# limited to damages caused by unauthorized purchases, improper configurations,
# or unintended usage. Users are solely responsible for ensuring the proper and
# safe application of this software in their respective environments.
# ////////////////////////////////////////////////////////////////////////////

"""
SDK functions for motor control and communication
"""

import asyncio
import time
import json
from typing import Optional, List, Dict, Any, Union, TYPE_CHECKING
from constants import (
    DEFAULT_TIMEOUT, BROADCAST_TIMEOUT, MAX_BROADCAST_RESPONSES,
    __IL, __ML, __IC, __DI, __MT, __MS, __AC, __DC, __SS, __SD,
    __MO, __JV, __BG, __ST, __OG, __PA, __PR, __SP,
    SCF_S1C_IDX, SCF_STL_IDX, SCF_TLC_IDX, MTS_BRK_IDX,
    # PRINT_MESSAGES is accessed via constants module
)
import constants
from utils import build_command_frame, bytes_to_hex_string, int32_signed_to_bytes, int32_to_bytes, Colors
from parsers import parse_il_response, parse_dio_port_response
from protocol import _process_received_data

if TYPE_CHECKING:
    from protocol import SerialProtocol

# Command flags
CMD_REQUEST_FLAG = 0x80  # Flag for GET/SET commands


# ==============================================================================
# SDK Client Class for Dependency Injection
# ==============================================================================

class SDKClient:
    """
    SDK Client class for managing serial transport and protocol instance.
    This class eliminates the need for global variables by using dependency injection.
    """
    
    def __init__(self, transport: asyncio.Transport, protocol_instance: 'SerialProtocol') -> None:  # type: ignore
        """
        Initialize SDK client with transport and protocol instance
        
        Args:
            transport: Serial transport object
            protocol_instance: Serial protocol instance
        """
        self.transport: asyncio.Transport = transport
        self.protocol_instance: 'SerialProtocol' = protocol_instance
    
    def _clear_response_queue(self) -> None:
        """Clear any existing responses in queue"""
        if self.protocol_instance is None:
            return
        
        while not self.protocol_instance.response_queue.empty():
            try:
                self.protocol_instance.response_queue.get_nowait()
            except asyncio.QueueEmpty:
                break
    
    def _print_command_info(self, description: str, send_data: bytes) -> None:
        """Print command information"""
        if not constants.PRINT_MESSAGES:
            return

        print("\n" + "=" * 60)
        print(Colors.green(description))
        print("=" * 60)
        print(Colors.green(f"Hexadecimal: {bytes_to_hex_string(send_data)}"))
    
    async def _send_command_and_wait_response(
        self, 
        send_data: bytes, 
        timeout: float, 
        max_responses: int = 1
    ) -> Union[Optional[Dict[str, Any]], Optional[List[Dict[str, Any]]]]:
        """
        Internal function to send command and wait for response(s)
        
        Args:
            send_data: Command frame bytes to send
            timeout: Timeout in seconds
            max_responses: Maximum number of responses to collect (1 for single, >1 for multiple)
            
        Returns:
            For max_responses=1: Parsed response dictionary or None
            For max_responses>1: List of parsed response dictionaries or None
        """
        if self.transport is None or self.protocol_instance is None:
            raise RuntimeError("Serial connection not established")
        
        self._clear_response_queue()
        
        # Send command
        send_time = time.perf_counter()
        self.transport.write(send_data)
        
        if max_responses == 1:
            # Single response
            try:
                response_frame = await asyncio.wait_for(
                    self.protocol_instance.response_queue.get(), 
                    timeout=timeout
                )
                return _process_received_data(response_frame, send_time)
            except asyncio.TimeoutError:
                print(Colors.red(f"X Timeout ({timeout}s) - No response received"))
                return None
        else:
            # Multiple responses
            responses = []
            start_time = time.perf_counter()
            
            while len(responses) < max_responses:
                remaining_time = timeout - (time.perf_counter() - start_time)
                if remaining_time <= 0:
                    break
                    
                try:
                    response_frame = await asyncio.wait_for(
                        self.protocol_instance.response_queue.get(), 
                        timeout=remaining_time
                    )
                    parsed = _process_received_data(response_frame, send_time)
                    if parsed:
                        responses.append(parsed)
                except asyncio.TimeoutError:
                    break
            
            if not responses:
                print(Colors.red(f"X Timeout ({timeout}s) - No responses received"))
                return None
            
            return responses
    
    async def send_and_receive(
        self, 
        send_data: bytes, 
        description: str, 
        timeout: Optional[float] = None
    ) -> Optional[Dict[str, Any]]:
        """
        Generic send and receive function for SDK commands
        
        Args:
            send_data: Command frame bytes to send
            description: Description for logging
            timeout: Timeout in seconds (default: DEFAULT_TIMEOUT)
            
        Returns:
            Parsed response dictionary or None if no response
        """
        if timeout is None:
            timeout = DEFAULT_TIMEOUT
        
        self._print_command_info(description, send_data)
        return await self._send_command_and_wait_response(send_data, timeout, max_responses=1)
    
    async def send_and_receive_multiple(
        self, 
        send_data: bytes, 
        description: str, 
        timeout: float = 1.0, 
        max_responses: int = 255
    ) -> Optional[List[Dict[str, Any]]]:
        """
        Generic send and receive function for SDK commands that expect multiple responses
        
        Args:
            send_data: Command frame bytes to send
            description: Description for logging
            timeout: Timeout in seconds
            max_responses: Maximum number of responses to collect
            
        Returns:
            List of parsed response dictionaries or None if no responses
        """
        self._print_command_info(description, send_data)
        return await self._send_command_and_wait_response(send_data, timeout, max_responses=max_responses)


# ==============================================================================
# Helper functions for parameter validation
# ==============================================================================

def _validate_station_id(station_id: int) -> None:
    """Validate station ID parameter"""
    if not (0 <= station_id <= 126):
        raise ValueError(f"Station ID must be 0-255, got {station_id}")


def _validate_byte_value(value: int, param_name: str) -> None:
    """Validate byte value (0-255)"""
    if not (0 <= value <= 255):
        raise ValueError(f"{param_name} must be 0-255, got {value}")


# ==============================================================================
# Helper functions for printing SDK responses
# ==============================================================================

def _print_sdk_response_basic(parsed: Optional[Dict[str, Any]], title: str, il_title_suffix: str = "") -> None:
    """
    Print basic SDK response information

    Args:
        parsed: Parsed response dictionary
        title: Title for the response section
        il_title_suffix: Optional suffix for IL Info title (e.g., " (Acknowledgment)")
    """
    if not constants.PRINT_MESSAGES:
        return

    print("\n" + "-" * 60)
    print(Colors.blue(f"{title}:"))
    print("-" * 60)

    if parsed is None:
        print(Colors.red("Return Value: None (No response received)"))
        return

    print(Colors.blue(f"Return Value: Success"))
    print(Colors.blue(f"  - Station ID: 0x{parsed['station_id']:02X} ({parsed['station_id']})"))

    cw_info = parsed.get('control_word_info', {})
    cw_name = cw_info.get('command_name', 'Unknown')
    cw_desc = cw_info.get('command_description', 'Unknown command')
    print(Colors.blue(f"  - Control Word: 0x{parsed['control_word']:02X} ({parsed['control_word']}) - {cw_name} ({cw_desc})"))
    print(Colors.blue(f"  - Data Length: {parsed['data_len']}"))
    print(Colors.blue(f"  - CRC Valid: {'OK Yes' if parsed['crc_valid'] else 'X No'}"))

    # If IL info is available, print it
    if 'il_info' in parsed:
        _print_il_info_detail(parsed['il_info'], il_title_suffix)
    elif parsed.get('data_len', 0) > 0:
        print(Colors.blue(f"  - Data Bytes: {bytes_to_hex_string(bytes(parsed['data_bytes']))}"))

    print("-" * 60)


def _print_il_info_detail(il_info: Dict[str, Any], title_suffix: str = "") -> None:
    """
    Print detailed IL (Input Logic) information

    Args:
        il_info: Dictionary containing parsed IL information
        title_suffix: Optional suffix for the IL Info title (e.g., " (Acknowledgment)")
    """
    if not constants.PRINT_MESSAGES:
        return

    print(Colors.blue(f"  - IL Info{title_suffix}:"))
    print(Colors.blue(f"    * Input Index: 0x{il_info['input_index']:02X} ({il_info['input_index']}) - {il_info['index_name']}"))

    if il_info['is_stall_trigger']:
        # Stall trigger case
        stall_action_info = il_info['stall_action_info']
        print(Colors.blue(f"    * Stall Action: 0x{il_info['stall_action']:02X} ({il_info['stall_action']}) - {stall_action_info['action_name']} ({stall_action_info['action_description']})"))
        if 'stall_fg_flag' in il_info:
            fg_status = "Enabled" if il_info['stall_fg_flag'] else "Disabled"
            print(Colors.blue(f"    * Stall FG Flag: {il_info['stall_fg_flag']} - Power-On Execution Flag ({fg_status})"))
        print(Colors.blue(f"    * Reserved: 0x{il_info['reserved']:02X} ({il_info['reserved']}) - Should be 0x00"))

    elif il_info['is_torque_limit_trigger']:
        # Torque limit trigger case
        torque_limit_action_info = il_info['torque_limit_action_info']
        print(Colors.blue(f"    * Torque Limit Action: 0x{il_info['torque_limit_action']:02X} ({il_info['torque_limit_action']}) - {torque_limit_action_info['action_name']} ({torque_limit_action_info['action_description']})"))
        if 'torque_limit_fg_flag' in il_info:
            fg_status = "Enabled" if il_info['torque_limit_fg_flag'] else "Disabled"
            print(Colors.blue(f"    * Torque Limit FG Flag: {il_info['torque_limit_fg_flag']} - Power-On Execution Flag ({fg_status})"))
        print(Colors.blue(f"    * Torque Limit Percent: {il_info['torque_limit_percent']}% (0x{il_info['torque_limit_percent']:02X}) - Torque Limit [10...300]%"))

    else:
        # Normal input trigger case
        falling_edge_info = il_info['falling_edge_info']
        rising_edge_info = il_info['rising_edge_info']
        print(Colors.blue(f"    * Falling Edge Action: 0x{il_info['falling_edge_action']:02X} ({il_info['falling_edge_action']}) - {falling_edge_info['action_name']} ({falling_edge_info['action_description']})"))
        if 'falling_edge_fg_flag' in il_info:
            fg_status = "Enabled" if il_info['falling_edge_fg_flag'] else "Disabled"
            print(Colors.blue(f"    * Falling Edge FG Flag: {il_info['falling_edge_fg_flag']} - Power-On Execution Flag ({fg_status})"))
        print(Colors.blue(f"    * Rising Edge Action: 0x{il_info['rising_edge_action']:02X} ({il_info['rising_edge_action']}) - {rising_edge_info['action_name']} ({rising_edge_info['action_description']})"))
        if 'rising_edge_fg_flag' in il_info:
            fg_status = "Enabled" if il_info['rising_edge_fg_flag'] else "Disabled"
            print(Colors.blue(f"    * Rising Edge FG Flag: {il_info['rising_edge_fg_flag']} - Power-On Execution Flag ({fg_status})"))


# Global SDK client instance (set via dependency injection)
_global_sdk_client: Optional['SDKClient'] = None


def set_sdk_client(client: 'SDKClient') -> None:
    """
    Set the global SDK client instance (dependency injection)
    
    Args:
        client: SDKClient instance
    """
    global _global_sdk_client
    _global_sdk_client = client


def get_sdk_client() -> 'SDKClient':
    """
    Get the global SDK client instance
    
    Returns:
        SDKClient instance
        
    Raises:
        RuntimeError: If SDK client is not set
    """
    if _global_sdk_client is None:
        raise RuntimeError("SDK client not initialized. Call set_sdk_client() first.")
    return _global_sdk_client


# ==============================================================================
# Generic SDK Command Builder (uses global SDK client)
# ==============================================================================

async def _execute_sdk_command(
    station_id: int,
    command_code: int,
    data_bytes: List[int],
    description: str,
    timeout: Optional[float] = None,
    max_responses: int = 1
) -> Union[Optional[Dict[str, Any]], Optional[List[Dict[str, Any]]]]:
    """
    Generic function to execute SDK commands
    
    Args:
        station_id: Station ID (0-255)
        command_code: Command code (e.g., __ML, __IC, etc.)
        data_bytes: Data bytes list
        description: Description for logging
        timeout: Timeout in seconds (default: DEFAULT_TIMEOUT)
        max_responses: Maximum number of responses (1 for single, >1 for multiple)
        
    Returns:
        Parsed response(s) or None
    """
    client = get_sdk_client()
    _validate_station_id(station_id)
    control_word = command_code | CMD_REQUEST_FLAG
    send_data = build_command_frame(station_id, control_word, len(data_bytes), data_bytes)
    
    if timeout is None:
        timeout = DEFAULT_TIMEOUT
    
    client._print_command_info(description, send_data)
    return await client._send_command_and_wait_response(send_data, timeout, max_responses=max_responses)


# ==============================================================================
# SDK Functions (backward compatible interface)
# ==============================================================================

async def SdkGetInputLogic(station_id: int, input_index: int) -> Optional[Dict[str, Any]]:
    """
    Encapsulate SdkGetInputLogic command with send and receive
    
    Args:
        station_id: Station ID (0-255)
        input_index: Input index (0-255), passed to d0 position
        Valid indices: 
        - SCF_S1C_IDX(0) to SCF_S8C_IDX(7): Input ports 1-8
        - SCF_P1C_IDX(8) to SCF_P8C_IDX(15): Input ports 9-16
        - SCF_STL_IDX(16): Stall trigger (Only for UIM342/341)
        - SCF_TLC_IDX(17): Torque limit trigger (Only for UIM720)
        
    Returns:
        Dictionary containing parsed response frame, or None if no response received
        If response is IL type, the dictionary will contain parsed IL information
    """
    _validate_byte_value(input_index, "input_index")
    description = f"Sending SdkGetInputLogic command (input_index={input_index}):"
    parsed = await _execute_sdk_command(station_id, __IL, [input_index], description)
    
    # If response is IL type, add parsed IL information
    if parsed and parsed.get('control_word_info', {}).get('is_input_logic'):
        if parsed['data_len'] == 3 and len(parsed['data_bytes']) == 3:
            try:
                il_info = parse_il_response(parsed['data_bytes'])
                parsed['il_info'] = il_info
            except ValueError:
                pass
    
    # Print return value
    _print_sdk_response_basic(parsed, "SdkGetInputLogic Return Value")

    return parsed


async def SdkSetInputLogic(
    station_id: int, 
    input_index: int, 
    falling_edge_action: int, 
    rising_edge_action: int
) -> Optional[Dict[str, Any]]:
    """
    Encapsulate SdkSetInputLogic command with send and receive
    
    Args:
        station_id: Station ID (0-255)
        input_index: Input index (0-255), passed to d0 position
        falling_edge_action: Action code on falling edge (Af) (0-255), passed to d1 position
        rising_edge_action: Action code on rising edge (Ar) (0-255), passed to d2 position
        
        Valid input indices: 
        - SCF_S1C_IDX(0) to SCF_S8C_IDX(7): Input ports 1-8
        - SCF_P1C_IDX(8) to SCF_P8C_IDX(15): Input ports 9-16
        - SCF_STL_IDX(16): Stall trigger (UIM342/341 only) - rising_edge_action should be 0x00
        - SCF_TLC_IDX(17): Torque limit trigger (UIM720 only) - rising_edge_action is torque limit percentage (10-300%)

        Valid action codes:
        - ILC_NOP_IDX(0x00): Disable / No Action
        - ILC_OFF_IDX(0x01): Driver OFF / Driver OFF
        - ILC_EST_IDX(0x02): Emergent Stop / Emergency Stop
        - ILC_DST_IDX(0x03): Decelerating Stop / Decelerating Stop
        - ILC_OPR_IDX(0x04): Origin + reverse PR / Set Origin + Reverse Position Relative
        - ILC_OES_IDX(0x05): Origin + EStop / Set Origin + Emergency Stop
        - ILC_ODS_IDX(0x06): Origin + DStop / Set Origin + Decelerating Stop
        - ILC_RJV_IDX(0x07): Reverse JV / Reverse Jog Velocity
        - ILC_SJV_IDX(0x08): Signed JV / Signed Jog Velocity
        - ILC_RPR_IDX(0x09): Reverse PR / Reverse Position Relative
        - ILC_SPR_IDX(0x0A): Signed PR / Signed Position Relative
        - ILC_SPA_IDX(0x0B): Signed PA / Signed Position Absolute
        - ILC_PVT_IDX(0x0F): Execute PVT / Execute PVT

    Returns:
        Dictionary containing parsed response frame, or None if no response received
        If response is IL type, the dictionary will contain parsed IL information
    """
    # Validate parameters
    _validate_station_id(station_id)
    _validate_byte_value(input_index, "input_index")
    _validate_byte_value(falling_edge_action, "falling_edge_action")
    _validate_byte_value(rising_edge_action, "rising_edge_action")
    
    # Special validation for stall trigger
    if input_index == SCF_STL_IDX and rising_edge_action != 0x00:
        print(Colors.yellow(f"WARNING Warning: For stall trigger (SCF_STL_IDX), rising_edge_action should be 0x00, got 0x{rising_edge_action:02X}"))
    
    # Special validation for torque limit trigger
    if input_index == SCF_TLC_IDX and not (10 <= rising_edge_action <= 300):
        print(Colors.yellow(f"WARNING Warning: For torque limit trigger (SCF_TLC_IDX), rising_edge_action should be torque percentage (10-300%), got {rising_edge_action}"))
    
    # Build command frame
    # IL SET command: Control Word = __IL | CMD_REQUEST_FLAG, DL = 3, d0 = input_index, d1 = falling_edge_action, d2 = rising_edge_action
    data_bytes = [input_index, falling_edge_action, rising_edge_action]  # d0, d1, d2
    description = f"Sending SdkSetInputLogic command (input_index={input_index}, falling_edge_action=0x{falling_edge_action:02X}, rising_edge_action=0x{rising_edge_action:02X}):"
    parsed = await _execute_sdk_command(station_id, __IL, data_bytes, description)
    
    # If response is IL type, add parsed IL information
    if parsed and parsed.get('control_word_info', {}).get('is_input_logic'):
        if parsed['data_len'] == 3 and len(parsed['data_bytes']) == 3:
            try:
                il_info = parse_il_response(parsed['data_bytes'])
                parsed['il_info'] = il_info
            except ValueError:
                pass
    
    # Print return value
    _print_sdk_response_basic(parsed, "SdkSetInputLogic Return Value", " (Acknowledgment)")

    return parsed


# Other SDK functions will be added later...
# This is just an example showing how to refactor SDK functions

# Add more SDK functions

async def SdkGetML(station_id: int) -> Union[Optional[Dict[str, Any]], Optional[List[Dict[str, Any]]]]:
    """Get Model, function module and firmware version"""
    description_base = "Sending SdkGetML command (Get the model, function module and firmware version)"
    
    if station_id == 0:
        # Broadcast to all stations
        description = f"{description_base} - Broadcast to all stations:"
        return await _execute_sdk_command(station_id, __ML, [], description, timeout=BROADCAST_TIMEOUT, max_responses=MAX_BROADCAST_RESPONSES)
    else:
        # Single station
        description = f"{description_base} - Station ID: {station_id}:"
        return await _execute_sdk_command(station_id, __ML, [], description)


async def SdkGetInitialConfig(station_id: int, command_index: int) -> Optional[Dict[str, Any]]:
    """Get Initial Configuration"""
    _validate_byte_value(command_index, "command_index")
    description = f"Sending SdkGetInitialConfig command (command_index={command_index}):"
    return await _execute_sdk_command(station_id, __IC, [command_index], description)


def _print_dio_port_info(dio_parsed: Dict[str, Any], structured_result: Dict[str, Any]) -> None:
    """
    Print DIO port information

    Args:
        dio_parsed: Full parsed DIO port response
        structured_result: Structured result with only IN1~IN3 and OP1
    """
    if not constants.PRINT_MESSAGES:
        return

    # Print structured result as JSON
    print(f"\n{'=' * 60}")
    print(Colors.blue("SdkGetDIOport Structured Result:"))
    print(f"{'=' * 60}")
    print(json.dumps(structured_result, indent=2, ensure_ascii=False))
    print(f"{'=' * 60}")

    # Print formatted display
    print(f"\n{Colors.blue('SdkGetDIOport Parsed Result:')}")
    print(f"{'=' * 60}")
    print(f"Input Port Status (d0 = {dio_parsed['d0_hex']}):")
    for input_name, input_info in dio_parsed['inputs'].items():
        # Only print IN1~IN3
        if input_name in ['IN1', 'IN2', 'IN3']:
            status_text = Colors.green(input_info['status']) if input_info['value'] == 1 else Colors.yellow(input_info['status'])
            print(f"  {input_name} (bit{input_info['bit_position']}): {status_text}")
    print(f"\nOutput Port Status (d1 = {dio_parsed['d1_hex']}):")
    for output_name, output_info in dio_parsed['outputs'].items():
        # Only print OP1
        if output_name == 'OP1':
            status_text = Colors.green(output_info['status']) if output_info['value'] == 1 else Colors.yellow(output_info['status'])
            print(f"  {output_name} (bit{output_info['bit_position']}): {status_text}")
    print(f"{'=' * 60}\n")


async def SdkGetDIOport(station_id: int, dio_index: int = 0) -> Optional[Dict[str, Any]]:
    """
    Get Digital I/O port status

    Args:
        station_id: Station ID (0-255)
        dio_index: DIO index (0-255), passed to d0 position for specific DIO operations

    Returns:
        Dictionary containing parsed response frame with additional 'dio_structured' field
        containing structured DIO port information (IN1~IN3 and OP1 only)
    """
    # DI command supports larger index values (e.g., DI257 is valid)
    if not (0 <= dio_index <= 1023):  # Allow up to 1023 for DI command
        raise ValueError(f"dio_index must be 0-1023, got {dio_index}")

    # Handle different DI operations based on parameter value
    if dio_index == 0:
        # DI0 or DI: Get general DIO status
        description = f"Sending SdkGetDIOport command (Get general DIO status):"
        data_bytes = [0]
    elif dio_index <= 255:
        # DI1-DI255: Get specific DIO channel info
        description = f"Sending SdkGetDIOport command (Get DIO channel {dio_index} info):"
        data_bytes = [dio_index]
    else:
        # DI256+: Control DIO outputs (e.g., DI257 for indicator light)
        description = f"Sending SdkGetDIOport command (Control DIO output {dio_index}):"
        # For control operations, we might need additional data bytes
        # Using the pattern: [index_low, index_high, control_value]
        data_bytes = [dio_index & 0xFF, (dio_index >> 8) & 0xFF, 0x01]  # 0x01 = turn on

    parsed = await _execute_sdk_command(station_id, __DI, data_bytes, description)
    
    # Parse DIO port response and add structured result
    if parsed and parsed.get('data_bytes') and len(parsed['data_bytes']) >= 2:
        try:
            dio_parsed = parse_dio_port_response(parsed['data_bytes'])
            
            # Build structured result with only IN1~IN3 and OP1
            structured_result = {
                'd0': dio_parsed['d0'],
                'd0_hex': dio_parsed['d0_hex'],
                'd1': dio_parsed['d1'],
                'd1_hex': dio_parsed['d1_hex'],
                'inputs': {
                    name: {
                        'bit_position': info['bit_position'],
                        'value': info['value'],
                        'status': info['status']
                    }
                    for name, info in dio_parsed['inputs'].items()
                    if name in ['IN1', 'IN2', 'IN3']
                },
                'outputs': {
                    name: {
                        'bit_position': info['bit_position'],
                        'value': info['value'],
                        'status': info['status']
                    }
                    for name, info in dio_parsed['outputs'].items()
                    if name == 'OP1'
                }
            }
            
            # Add structured result to parsed response
            parsed['dio_structured'] = structured_result
            
            # Print DIO port information
            _print_dio_port_info(dio_parsed, structured_result)
            
        except ValueError as e:
            print(Colors.red(f"Error parsing DIO port response: {e}"))
    
    return parsed


async def SdkGetMotorConfig(station_id: int, command_index: int) -> Optional[Dict[str, Any]]:
    """Get Motor Configuration"""
    _validate_byte_value(command_index, "command_index")
    description = f"Sending SdkGetMotorConfig command (command_index={command_index}):"
    return await _execute_sdk_command(station_id, __MT, [command_index], description)


async def SdkSetMotorConfig(station_id: int, command_index: int, value: int) -> Optional[Dict[str, Any]]:
    """Set Motor Configuration"""
    _validate_byte_value(command_index, "command_index")
    if not (0 <= value <= 0xFFFF):
        raise ValueError(f"Value must be 0-65535, got {value}")
    value_bytes = [value & 0xFF, (value >> 8) & 0xFF]  # 16-bit little-endian
    description = f"Sending SdkSetMotorConfig command (command_index={command_index}, value={value}):"
    return await _execute_sdk_command(station_id, __MT, [command_index] + value_bytes, description)


async def SdkGetMotionStatus(station_id: int, command_index: int) -> Optional[Dict[str, Any]]:
    """
    Get Motion Status.
    
    Args:
        station_id: Station ID (0-126)
        command_index: Query index
            - 0: Get Status Flags and Relative Position
            - 1: Get Current Speed and Absolute Position
    
    Returns:
        For command_index=0:
            - Status Flags (ms0, ms1): 16-bit status flags
            - Relative Position (PR0-PR3): 32-bit signed relative position
        For command_index=1:
            - Current Speed (sp0-sp2): 24-bit speed value (pps)
            - Absolute Position (PA0-PA3): 32-bit signed absolute position
    """
    _validate_byte_value(command_index, "command_index")
    data_bytes = [command_index]
    if command_index == 0:
        description = "Sending SdkGetMotionStatus command (command_index=0, Get Status Flags and Relative Position):"
    else:
        description = "Sending SdkGetMotionStatus command (command_index=1, Get Current Speed and Absolute Position):"
    return await _execute_sdk_command(station_id, __MS, data_bytes, description)


async def SdkGetAcceleration(station_id: int) -> Optional[Dict[str, Any]]:
    """Get Acceleration"""
    description = "Sending SdkGetAcceleration command (query current acceleration):"
    return await _execute_sdk_command(station_id, __AC, [], description)


async def SdkGetDeceleration(station_id: int) -> Optional[Dict[str, Any]]:
    """Get Deceleration"""
    description = "Sending SdkGetDeceleration command (query current deceleration):"
    return await _execute_sdk_command(station_id, __DC, [], description)


async def SdkGetCutInSpeed(station_id: int) -> Optional[Dict[str, Any]]:
    """Get Cut-In Speed (Start Speed)"""
    description = "Sending SdkGetCutInSpeed command (query current cut-in speed):"
    return await _execute_sdk_command(station_id, __SS, [], description)


async def SdkGetStopDeceleration(station_id: int) -> Optional[Dict[str, Any]]:
    """Get Stop Deceleration"""
    description = "Sending SdkGetStopDeceleration command (query current stop deceleration):"
    return await _execute_sdk_command(station_id, __SD, [], description)


async def SdkSetMotorOn(station_id: int, enable: int) -> Optional[Dict[str, Any]]:
    """Set Motor On/Off"""
    _validate_byte_value(enable, "enable")
    description = f"Sending SdkSetMotorOn command (enable={enable}):"
    return await _execute_sdk_command(station_id, __MO, [enable], description)


async def SdkSetJogMxn(station_id: int, speed_value: int) -> Optional[Dict[str, Any]]:
    """Set Jog Motion Speed"""
    speed_bytes = int32_signed_to_bytes(speed_value)
    description = f"Sending SdkSetJogMxn command (speed={speed_value}):"
    return await _execute_sdk_command(station_id, __JV, speed_bytes, description)


async def SdkSetBeginMxn(station_id: int) -> Optional[Dict[str, Any]]:
    """Set Begin Motion"""
    description = "Sending SdkSetBeginMxn command:"
    return await _execute_sdk_command(station_id, __BG, [], description)


async def SdkSetStopMxn(station_id: int) -> Optional[Dict[str, Any]]:
    """Set Stop Motion (Emergency Stop)"""
    description = "Sending SdkSetStopMxn command:"
    return await _execute_sdk_command(station_id, __ST, [], description)


async def SdkSetOrigin(station_id: int) -> Optional[Dict[str, Any]]:
    """Set Origin (Return to origin)"""
    description = "Sending SdkSetOrigin command:"
    return await _execute_sdk_command(station_id, __OG, [], description)


async def SdkSetPtpMxnA(station_id: int, pa_absolute: int) -> Optional[Dict[str, Any]]:
    """Set PTP Motion Absolute Position"""
    pos_bytes = int32_signed_to_bytes(pa_absolute)
    description = f"Sending SdkSetPtpMxnA command (pa_absolute={pa_absolute}):"
    return await _execute_sdk_command(station_id, __PA, pos_bytes, description)


async def SdkSetPtpMxnR(station_id: int, pr_relative: int) -> Optional[Dict[str, Any]]:
    """Set PTP Motion Relative Position"""
    pos_bytes = int32_signed_to_bytes(pr_relative)
    description = f"Sending SdkSetPtpMxnR command (pr_relative={pr_relative}):"
    return await _execute_sdk_command(station_id, __PR, pos_bytes, description)


async def SdkSetPtpSPD(station_id: int, speed: int) -> Optional[Dict[str, Any]]:
    """Set PTP Motion Speed"""
    speed_bytes = int32_to_bytes(speed)
    description = f"Sending SdkSetPtpSPD command (speed={speed}):"
    return await _execute_sdk_command(station_id, __SP, speed_bytes, description)


async def SdkGetPtpMxnA(station_id: int) -> Optional[Dict[str, Any]]:
    """Get PTP Motion Absolute Position"""
    description = "Sending SdkGetPtpMxnA command (query current absolute position):"
    return await _execute_sdk_command(station_id, __PA, [], description)