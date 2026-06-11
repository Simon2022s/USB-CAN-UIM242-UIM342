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
Parser functions for protocol response data and error handling
"""

from typing import Dict, Any, List, Callable, Optional, TypeVar, Coroutine
from functools import wraps
import asyncio

from constants import (
    # Command codes
    __ER, __ML, __MT, __IC, __IE, __MO, __BG, __ST,
    __JV, __SP, __PR, __PA, __OG, __AC, __DC, __SS, __SD,
    __DV, __DI, __RT, __MS, __IL,
    # Error codes
    ERR_INS_SYNT, ERR_INS_NUMB, ERR_INS_IDXR, ERR_SYS_STTM,
    ERR_MXN_DCSD, ERR_MXN_MRUN, ERR_MXN_MOFF, ERR_MXN_MTSD,
    ERR_MXN_BENA, ERR_MXN_BDIS, ERR_MXN_SPOG,
    ERR_PVT_RUNG, ERR_PVT_WPOV, ERR_PVT_IOFN, ERR_PVB_OVFL, ERR_SXP_BUSY,
    # Real-time notification codes
    RTCN_MXN_INP, RTCN_DIO_P1L, RTCN_DIO_P1H, RTCN_DIO_P2L, RTCN_DIO_P2H,
    RTCN_DIO_P3L, RTCN_DIO_P3H, RTCN_DIO_P4L, RTCN_DIO_P4H,
    # Input logic indices
    SCF_S1C_IDX, SCF_S2C_IDX, SCF_S3C_IDX,
    SCF_STL_IDX, SCF_TLC_IDX,
    # Input logic action codes
    ILC_NOP_IDX, ILC_OFF_IDX, ILC_EST_IDX, ILC_DST_IDX,
    ILC_OPR_IDX, ILC_OES_IDX, ILC_ODS_IDX, ILC_RJV_IDX,
    ILC_SJV_IDX, ILC_RPR_IDX, ILC_SPR_IDX, ILC_SPA_IDX, ILC_PVT_IDX,
    # IC configuration indices
    ICFG_AMO_IDX, ICFG_CCW_IDX, ICFG_UPG_IDX, ICFG_LCK_IDX,
    ICFG_ACM_IDX, ICFG_ABS_IDX, ICFG_QEM_IDX, ICFG_SLM_IDX,
    # Motor configuration indices
    MTS_MCS_IDX, MTS_CUR_IDX, MTS_PSV_IDX, MTS_ENA_IDX, MTS_BRK_IDX,
    # DVR indices
    DVR_MOD_IDX, DVR_CUR_IDX, DVR_SPD_IDX, DVR_PRM_IDX, DVR_PAM_IDX, DVR_TIS_IDX,
    # Gateway station ID
    GATEWAY_STATION_ID
)
from utils import bytes_to_int32, bytes_to_int32_signed, bytes_to_int24, Colors
from exceptions import NoResponseError, NoStationsError, NoDeviceStationsError, TargetStationNotFoundError

T = TypeVar('T')


def parse_error_code(error_code: int) -> Dict[str, Any]:
    """
    Parse Error Code from ER response d1 field
    
    Args:
        error_code: Error code value (0-255)
        
    Returns:
        Dictionary containing parsed results:
        - raw_value: Original error code value
        - error_name: Error code name (English)
        - error_description: Error description (English)
    """
    error_code_map = {
        ERR_INS_SYNT: ("ERR_INS_SYNT", "Instruction's Syntax is wrong."),
        ERR_INS_NUMB: ("ERR_INS_NUMB", "Instruction's Data are wrong."),
        ERR_INS_IDXR: ("ERR_INS_IDXR", "Instruction's Sub-Index is wrong."),
        ERR_SYS_STTM: ("ERR_SYS_STTM", "[TIME] Cannot change system time, while the motor is running."),
        ERR_MXN_DCSD: ("ERR_MXN_DCSD", "[MXN] Stop Decelleration (SD) is slower than the Decelleration(DC)."),
        ERR_MXN_MRUN: ("ERR_MXN_MRUN", "[MXN] Cannot change or query, while the motor is running."),
        ERR_MXN_MOFF: ("ERR_MXN_MOFF", "[MXN] Cannot BG, when the motor driver is OFF."),
        ERR_MXN_MTSD: ("ERR_MXN_MTSD", "[MXN] Cannot BG, when the motor is performing Emergency Stop."),
        ERR_MXN_BENA: ("ERR_MXN_BENA", "[MXN] Cannot BG, when the motor Brake is Locked."),
        ERR_MXN_BDIS: ("ERR_MXN_BDIS", "[MXN] Cannot turn off the motor driver, when the motor Brake is unlocked."),
        ERR_MXN_SPOG: ("ERR_MXN_SPOG", "[MXN] Cannot set origin (for ABS encoder only), when the motor is running."),
        ERR_PVT_RUNG: ("ERR_PVT_RUNG", "[PVT] Cannot set PV or MP[0], when the motor is running."),
        ERR_PVT_WPOV: ("ERR_PVT_WPOV", "[PVT] Index of QP/QV/QT exceeds MP[6]"),
        ERR_PVT_IOFN: ("ERR_PVT_IOFN", "[PVT] QA Mask not meeting I/O function requirements"),
        ERR_PVB_OVFL: ("ERR_PVB_OVFL", "[PVT] PVT buffer overflow"),
        ERR_SXP_BUSY: ("ERR_SXP_BUSY", "[PVT] Sx processing not complete, new parameters not accepted"),
    }
    
    if error_code in error_code_map:
        error_name, error_description = error_code_map[error_code]
    else:
        error_name = f"Unknown (0x{error_code:02X})"
        error_description = "Unknown error code"
    
    return {
        'raw_value': error_code,
        'error_name': error_name,
        'error_description': error_description
    }


def parse_control_word(control_word: int) -> Dict[str, Any]:
    """
    Parse Control Word from response frame
    
    Control Word structure (8 bits):
    - Bit 7 (MSB): Response flag bit
      * 0: Indicates this is a response to a command
      * 1: Indicates this is a command frame (used when sending)
    - Bit 6-0 (LSB 7 bits): Command code
    
    Args:
        control_word: Control Word value (0-255)
        
    Returns:
        Dictionary containing parsed results with command flags
    """
    # Extract MSB (Bit 7) - Response flag bit
    is_response = (control_word & 0x80) == 0  # If MSB is 0, it's a response
    
    # Extract lower 7 bits (Bit 6-0) - Command code
    command_code = control_word & 0x7F
    
    # Command code to name mapping
    command_map = {
        __ER: ("Error Report", "Error report/error notification"),
        __ML: ("Get Model", "Get the model, function module and firmware version"),
        __MT: ("Motor Configuration", "Motor driver configuration"),
        __IC: ("Power-Up Configuration", "Power-up configuration query"),
        __IE: ("Inform Enable", "Notification enable configuration (IN1/IN2/IN3/PTP finish)"),
        __MO: ("Motor On/Off", "Motor switch control"),
        __BG: ("Begin Motion", "Start motion"),
        __ST: ("Stop Motion", "Emergency stop (using _SD parameter)"),
        __JV: ("Jog Velocity", "Jog speed setting"),
        __SP: ("Speed", "Speed setting"),
        __PR: ("Position Relative", "Relative position"),
        __PA: ("Position Absolute", "Absolute position"),
        __OG: ("Origin", "Return to origin"),
        __AC: ("Acceleration", "Acceleration setting (Time:[10-60000]ms, Value:[10-1000000000]pps/s)"),
        __DC: ("Deceleration", "Deceleration setting (Time:[10-60000]ms, Value:[10-1000000000]pps/s)"),
        __SS: ("Start Speed", "Start Speed (Cut-In Speed) setting (v3:v0=[0...65535]pps)"),
        __SD: ("Stop Deceleration", "Stop Deceleration setting (d3:d0=[400...1,000,000,000]pps/s)"),
        __DV: ("Desire Value", "Desired value/target value"),
        __DI: ("Digital I/O", "Digital Signal Inputs and Outputs - DIO port status"),
        __RT: ("Real-Time Notification", "Real-time notification message"),
        __MS: ("Motion Status", "Motion Status & Displacement - Query motion status flags and relative position (index 0) or speed and absolute position (index 1)"),
        __IL: ("Input Logic", "Input Triggered Action Logic - Input trigger action logic configuration"),
    }
    
    # Get command name and description
    if command_code in command_map:
        command_name, command_description = command_map[command_code]
    else:
        command_name = f"Unknown (0x{command_code:02X})"
        command_description = "Unknown command"
    
    result = {
        'raw_value': control_word,
        'is_response': is_response,
        'command_code': command_code,
        'command_name': command_name,
        'command_description': command_description,
        # Command type flags
        'is_real_time_notification': command_code == __RT,
        'is_desire_value': command_code == __DV,
        'is_ic_configuration': command_code == __IC,
        'is_error_report': command_code == __ER,
        'is_get_model': command_code == __ML,
        'is_motor_configuration': command_code == __MT,
        'is_motion_status': command_code == __MS,
        'is_acceleration': command_code == __AC,
        'is_deceleration': command_code == __DC,
        'is_start_speed': command_code == __SS,
        'is_stop_deceleration': command_code == __SD,
        'is_input_logic': command_code == __IL,
    }
    
    return result


def parse_rtcn_d0_code(d0_code: int) -> Dict[str, Any]:
    """Parse Real-Time Notification d0 code"""
    rtcn_d0_map = {
        RTCN_MXN_INP: ("PTP Motion, In Position", "Point-to-point motion completed, in position"),
        RTCN_DIO_P1L: ("P1 Low Level", "DIO Port P1 changed to Low level"),
        RTCN_DIO_P1H: ("P1 High Level", "DIO Port P1 changed to High level"),
        RTCN_DIO_P2L: ("P2 Low Level", "DIO Port P2 changed to Low level"),
        RTCN_DIO_P2H: ("P2 High Level", "DIO Port P2 changed to High level"),
        RTCN_DIO_P3L: ("P3 Low Level", "DIO Port P3 changed to Low level"),
        RTCN_DIO_P3H: ("P3 High Level", "DIO Port P3 changed to High level"),
        RTCN_DIO_P4L: ("P4 Low Level", "DIO Port P4 changed to Low level"),
        RTCN_DIO_P4H: ("P4 High Level", "DIO Port P4 changed to High level"),
    }
    
    if d0_code in rtcn_d0_map:
        code_name, code_description = rtcn_d0_map[d0_code]
    else:
        code_name = f"Unknown (0x{d0_code:02X})"
        code_description = "Unknown real-time notification code"
    
    return {
        'raw_value': d0_code,
        'code_name': code_name,
        'code_description': code_description
    }


def parse_il_index(il_index: int) -> Dict[str, Any]:
    """Parse Input Logic (IL) response d0 index"""
    il_index_map = {
        SCF_S1C_IDX: ("Input# 1", "Input port 1 trigger action logic"),
        SCF_S2C_IDX: ("Input# 2", "Input port 2 trigger action logic"),
        SCF_S3C_IDX: ("Input# 3", "Input port 3 trigger action logic"),   
        SCF_STL_IDX: ("On Stall", "Stall trigger action logic (Only for UIM342/341)"),
        SCF_TLC_IDX: ("On TorqueLimit", "Torque limit trigger action logic (Only for UIM720)"),
    }
    
    if il_index in il_index_map:
        index_name, index_description = il_index_map[il_index]
    else:
        index_name = f"Unknown (0x{il_index:02X})"
        index_description = "Unknown IL response index"
    
    return {
        'raw_value': il_index,
        'index_name': index_name,
        'index_description': index_description
    }


def parse_il_action_code(action_code: int) -> Dict[str, Any]:
    """Parse Input Logic Action Code"""
    action_code_map = {
        ILC_NOP_IDX: ("Disable", "No Action"),
        ILC_OFF_IDX: ("Driver OFF", "Driver OFF"),
        ILC_EST_IDX: ("Emergent Stop", "Emergency Stop"),
        ILC_DST_IDX: ("Decelerating Stop", "Decelerating Stop"),
        ILC_OPR_IDX: ("Origin + reverse PR", "Set Origin + Reverse Position Relative"),
        ILC_OES_IDX: ("Origin + EStop", "Set Origin + Emergency Stop"),
        ILC_ODS_IDX: ("Origin + DStop", "Set Origin + Decelerating Stop"),
        ILC_RJV_IDX: ("Reverse JV", "Reverse Jog Velocity"),
        ILC_SJV_IDX: ("Signed JV", "Signed Jog Velocity"),
        ILC_RPR_IDX: ("Reverse PR", "Reverse Position Relative"),
        ILC_SPR_IDX: ("Signed PR", "Signed Position Relative"),
        ILC_SPA_IDX: ("Signed PA", "Signed Position Absolute"),
        ILC_PVT_IDX: ("Execute PVT", "Execute PVT"),
    }
    
    if action_code in action_code_map:
        action_name, action_description = action_code_map[action_code]
    else:
        action_name = f"Unknown (0x{action_code:02X})"
        action_description = "Unknown action code"
    
    return {
        'raw_value': action_code,
        'action_name': action_name,
        'action_description': action_description
    }


def parse_il_response(data_bytes: List[int]) -> Dict[str, Any]:
    """Parse Input Logic (IL) response data"""
    if len(data_bytes) < 3:
        raise ValueError(f"IL response needs at least 3 bytes, got {len(data_bytes)}")
    
    input_index = data_bytes[0]
    # Extract bit 7 (FG flag) and lower 7 bits (action code) from data_bytes[1]
    falling_edge_fg_flag = (data_bytes[1] & 0x80) >> 7  # Bit 7: FG flag for Power-On Execution
    falling_edge_action = data_bytes[1] & 0x7F  # Bits 0-6: Action code
    # Extract bit 7 (FG flag) and lower 7 bits (action code) from data_bytes[2]
    rising_edge_fg_flag = (data_bytes[2] & 0x80) >> 7  # Bit 7: FG flag for Power-On Execution
    rising_edge_action = data_bytes[2] & 0x7F  # Bits 0-6: Action code
    
    # Parse input index
    index_info = parse_il_index(input_index)
    
    # Check if this is a special trigger type
    is_stall_trigger = (input_index == SCF_STL_IDX)
    is_torque_limit_trigger = (input_index == SCF_TLC_IDX)
    
    result = {
        'input_index': input_index,
        'index_name': index_info['index_name'],
        'is_stall_trigger': is_stall_trigger,
        'is_torque_limit_trigger': is_torque_limit_trigger,
    }
    
    if is_stall_trigger:
        # Stall trigger: d1=stall_action, d2=reserved(0x00)
        result['stall_action'] = falling_edge_action
        result['stall_fg_flag'] = falling_edge_fg_flag  # FG flag for Power-On Execution
        result['reserved'] = rising_edge_action
        result['stall_action_info'] = parse_il_action_code(falling_edge_action)
    elif is_torque_limit_trigger:
        # Torque limit trigger: d1=torque_limit_action, d2=torque_limit_percent
        result['torque_limit_action'] = falling_edge_action
        result['torque_limit_fg_flag'] = falling_edge_fg_flag  # FG flag for Power-On Execution
        result['torque_limit_percent'] = rising_edge_action
        result['torque_limit_action_info'] = parse_il_action_code(falling_edge_action)
    else:
        # Normal input trigger: d1=falling_edge_action, d2=rising_edge_action
        result['falling_edge_action'] = falling_edge_action
        result['falling_edge_fg_flag'] = falling_edge_fg_flag  # FG flag for Power-On Execution
        result['rising_edge_action'] = rising_edge_action
        result['rising_edge_fg_flag'] = rising_edge_fg_flag  # FG flag for Power-On Execution
        result['falling_edge_info'] = parse_il_action_code(falling_edge_action)
        result['rising_edge_info'] = parse_il_action_code(rising_edge_action)
    
    return result


# Other parsing functions will be added later...
def parse_ic_index(ic_index: int) -> Dict[str, Any]:
    """Parse Power-Up Configuration (IC) response d0 index"""
    ic_index_map = {
        ICFG_AMO_IDX: ("PowerUp DRV-ON", "Power-up enable (0:Disable, 1:Enable)"),
        ICFG_CCW_IDX: ("Positive Direct", "Motor direction (0:CW, 1:CCW)"),
        ICFG_UPG_IDX: ("Exec, UPG", "UPG enable (0:Disable, 1:Enable)"),
        ICFG_LCK_IDX: ("Input Lock Sys", "Input lock system (0:Disable, 1:Enable)"),
        ICFG_ACM_IDX: ("Acc./Dec. Unit", "Acceleration/Deceleration unit (0:pps/ms, 1:ms)"),
        ICFG_ABS_IDX: ("Encoder Type", "Encoder type (0:Inc., 1:abs)"),
        ICFG_QEM_IDX: ("Closed-Loop Mode", "Closed-loop control mode (0:Disable, 1:Enable)"),
        ICFG_SLM_IDX: ("Software Limits", "Software limits"),
    }
    
    if ic_index in ic_index_map:
        index_name, index_description = ic_index_map[ic_index]
    else:
        index_name = f"Unknown (0x{ic_index:02X})"
        index_description = "Unknown IC response index"
    
    return {
        'raw_value': ic_index,
        'index_name': index_name,
        'index_description': index_description
    }


def parse_mt_index(mt_index: int) -> Dict[str, Any]:
    """Parse Motor Configuration (MT) response d0 index"""
    mt_index_map = {
        MTS_MCS_IDX: ("Micro-Step", "Micro-step subdivision (mm=[1,2,4,8,16,32,64,128])"),
        MTS_CUR_IDX: ("Current Run", "Working current (ii=[5...80] x0.1 Amp)"),
        MTS_PSV_IDX: ("Current Idle", "Idle current percentage (pp=[0...100]%)"),
        MTS_ENA_IDX: ("DRV-ON Delay", "Power-on enable delay (t1:t0=[0...60000]ms)"),
        MTS_BRK_IDX: ("Brake Lock", "Brake enable/release (ss=[0:Release; 1:Lock])"),
    }
    
    if mt_index in mt_index_map:
        index_name, index_description = mt_index_map[mt_index]
    else:
        index_name = f"Unknown (0x{mt_index:02X})"
        index_description = "Unknown MT response index"
    
    return {
        'raw_value': mt_index,
        'index_name': index_name,
        'index_description': index_description
    }


def parse_dio_port_response(data_bytes: List[int]) -> Dict[str, Any]:
    """
    Parse Digital I/O port (DI) response data
    
    Args:
        data_bytes: List of data bytes from DI response
                   - data_bytes[0] (d0): Input port status
                   - data_bytes[1] (d1): Output port status
    
    Returns:
        Dictionary containing parsed DIO port information:
        - d0: Raw value of input port byte
        - d1: Raw value of output port byte
        - inputs: Dictionary with input port status (IN1, IN2, IN3, ...)
        - outputs: Dictionary with output port status (OP1, OP2, ...)
    """
    if len(data_bytes) < 2:
        raise ValueError(f"DI response needs at least 2 bytes, got {len(data_bytes)}")
    
    d0 = data_bytes[0]  # Input port status
    d1 = data_bytes[1]  # Output port status
    
    # Parse input ports from d0
    # bit0 = IN1, bit1 = IN2, bit2 = IN3, etc.
    inputs = {}
    for bit_pos in range(8):
        bit_value = (d0 >> bit_pos) & 0x01
        input_name = f"IN{bit_pos + 1}"
        inputs[input_name] = {
            'bit_position': bit_pos,
            'value': bit_value,
            'status': 'HIGH' if bit_value == 1 else 'LOW'
        }
    
    # Parse output ports from d1
    # bit0 = OP1, bit1 = OP2, etc.
    outputs = {}
    for bit_pos in range(8):
        bit_value = (d1 >> bit_pos) & 0x01
        output_name = f"OP{bit_pos + 1}"
        outputs[output_name] = {
            'bit_position': bit_pos,
            'value': bit_value,
            'status': 'HIGH' if bit_value == 1 else 'LOW'
        }
    
    return {
        'd0': d0,
        'd1': d1,
        'd0_hex': f"0x{d0:02X}",
        'd1_hex': f"0x{d1:02X}",
        'inputs': inputs,
        'outputs': outputs
    }


def parse_motion_status_flags(status_flags: int) -> Dict[str, Any]:
    """
    Parse Status Flags from SdkGetMotionStatus command_index=0 (ms0 ms1, 16-bit).

    d0 (low byte, ms0):
        bit0~bit1: Mode - motion mode
        bit2: SON - motor driver
        bit3: IN1 - IN1 logic level
        bit4: IN2 - IN2 logic level
        bit5: IN3 - IN3 logic level
        bit6: OP1 - OP1 logic level
        bit7: n/a

    d1 (high byte, ms1):
        bit0: STOP - motor is in stationary
        bit1: PAIF - motor is in position
        bit2: n/a
        bit3: TLIF - motor stall is detected
        bit4: n/a
        bit5: LOCK - system is locked down
        bit6: n/a
        bit7: ERR - system error is detected

    Args:
        status_flags: 16-bit value (d0 + (d1 << 8))

    Returns:
        Dictionary with raw values (mode, SON, IN1, ...), hex strings (d0_hex, d1_hex,
        status_flags_hex), and parsed descriptions (*_desc): mode_desc, SON_desc (ON/OFF),
        IN1/IN2/IN3/OP1_desc (HIGH/LOW), STOP_desc (Stationary/Moving), PAIF_desc
        (In position/Not in position), TLIF_desc (Stall detected/OK), LOCK_desc
        (Locked/Unlocked), ERR_desc (Error/OK).
    """
    d0 = status_flags & 0xFF
    d1 = (status_flags >> 8) & 0xFF

    # d0: bit0~1 mode, bit2 SON, bit3 IN1, bit4 IN2, bit5 IN3, bit6 OP1, bit7 n/a
    mode = d0 & 0x03
    son = (d0 >> 2) & 0x01
    in1 = (d0 >> 3) & 0x01
    in2 = (d0 >> 4) & 0x01
    in3 = (d0 >> 5) & 0x01
    op1 = (d0 >> 6) & 0x01

    # d1: bit0 STOP, bit1 PAIF, bit2 n/a, bit3 TLIF, bit4 n/a, bit5 LOCK, bit6 n/a, bit7 ERR
    stop = (d1 >> 0) & 0x01
    paif = (d1 >> 1) & 0x01
    tlif = (d1 >> 3) & 0x01
    lock = (d1 >> 5) & 0x01
    err = (d1 >> 7) & 0x01

    # Parsed human-readable descriptions
    # d0: Mode (0-3, device-specific), SON/IN/OP: ON|OFF or HIGH|LOW
    mode_desc = str(mode)
    son_desc = "ON" if son else "OFF"       # motor driver
    in1_desc = "HIGH" if in1 else "LOW"     # IN1 logic level
    in2_desc = "HIGH" if in2 else "LOW"     # IN2 logic level
    in3_desc = "HIGH" if in3 else "LOW"     # IN3 logic level
    op1_desc = "HIGH" if op1 else "LOW"     # OP1 logic level
    # d1: STOP, PAIF, TLIF, LOCK, ERR
    stop_desc = "Stationary" if stop else "Moving"
    paif_desc = "In position" if paif else "Not in position"
    tlif_desc = "Stall detected" if tlif else "OK"
    lock_desc = "Locked" if lock else "Unlocked"
    err_desc = "Error" if err else "OK"

    return {
        'd0': d0,
        'd1': d1,
        'd0_hex': f"0x{d0:02X}",
        'd1_hex': f"0x{d1:02X}",
        'status_flags_hex': f"0x{status_flags:04X}",
        # d0 raw
        'mode': mode,
        'SON': son,
        'IN1': in1,
        'IN2': in2,
        'IN3': in3,
        'OP1': op1,
        # d0 parsed
        'mode_desc': mode_desc,
        'SON_desc': son_desc,
        'IN1_desc': in1_desc,
        'IN2_desc': in2_desc,
        'IN3_desc': in3_desc,
        'OP1_desc': op1_desc,
        # d1 raw
        'STOP': stop,
        'PAIF': paif,
        'TLIF': tlif,
        'LOCK': lock,
        'ERR': err,
        # d1 parsed
        'STOP_desc': stop_desc,
        'PAIF_desc': paif_desc,
        'TLIF_desc': tlif_desc,
        'LOCK_desc': lock_desc,
        'ERR_desc': err_desc,
    }


# ==============================================================================
# Error Handling and Response Printing Functions
# ==============================================================================

def _print_sdk_return_value(result: Any, function_name: str) -> None:
    """
    Print SDK function return value based on function name.
    Extracts and displays relevant data fields for each function type.
    
    Args:
        result: Response from SDK function (dict or list)
        function_name: Name of the SDK function
    """
    print("\n" + "=" * 60)
    print(Colors.blue(f"{function_name} Return Value:"))
    print("=" * 60)
    
    if isinstance(result, list):
        # Handle list responses (e.g., SdkGetML broadcast)
        print(f"List with {len(result)} items:")
        for i, item in enumerate(result):
            if isinstance(item, dict):
                print(f"\nItem {i + 1}:")
                _print_single_result(item, function_name)
        print("=" * 60)
        return
    
    if not isinstance(result, dict):
        print(str(result))
        print("=" * 60)
        return
    
    _print_single_result(result, function_name)
    print("=" * 60)


def _parse_error_code(error_code: int) -> str:
    """
    Parse error code (d1) from Error Report response.
    
    Args:
        error_code: Error code value from d1
        
    Returns:
        Error description string
    """
    error_map = {
        ERR_INS_SYNT: "ERR_INS_SYNT - Instruction's Syntax is wrong",
        ERR_INS_NUMB: "ERR_INS_NUMB - Instruction's Data are wrong",
        ERR_INS_IDXR: "ERR_INS_IDXR - Instruction's Sub-Index is wrong",
        ERR_SYS_STTM: "ERR_SYS_STTM - Cannot change system time while motor is running",
        ERR_MXN_DCSD: "ERR_MXN_DCSD - Stop Deceleration (SD) is slower than Deceleration (DC)",
        ERR_MXN_MRUN: "ERR_MXN_MRUN - Cannot change or query while motor is running",
        ERR_MXN_MOFF: "ERR_MXN_MOFF - Cannot BG when motor driver is OFF",
        ERR_MXN_MTSD: "ERR_MXN_MTSD - Cannot BG when motor is performing Emergency Stop",
        ERR_MXN_BENA: "ERR_MXN_BENA - Cannot BG when motor Brake is Locked",
        ERR_MXN_BDIS: "ERR_MXN_BDIS - Cannot turn off motor driver when Brake is unlocked",
        ERR_MXN_SPOG: "ERR_MXN_SPOG - Cannot set origin (ABS encoder) when motor is running",
        ERR_PVT_RUNG: "ERR_PVT_RUNG - Cannot set PV or MP[0] when motor is running",
        ERR_PVT_WPOV: "ERR_PVT_WPOV - Index of QP/QV/QT exceeds MP[6]",
        ERR_PVT_IOFN: "ERR_PVT_IOFN - QA Mask not meeting I/O function requirements",
        ERR_PVB_OVFL: "ERR_PVB_OVFL - PVT buffer overflow",
        ERR_SXP_BUSY: "ERR_SXP_BUSY - Sx processing not complete, new parameters not accepted",
    }
    return error_map.get(error_code, f"Unknown Error (0x{error_code:02X})")


def _print_error_report(data_bytes: List[int], data_len: int) -> None:
    """
    Parse and print Error Report (__ER) response data.
    
    Error Report format:
    - d0: Error index (0 = latest error)
    - d1: Error code
    - d2: Control word that caused the error
    - d3: Sub-Index or additional control word info
    - d4~d5: Factory use (reserved)
    
    Args:
        data_bytes: Data bytes from response
        data_len: Length of data bytes
    """
    if data_len < 4:
        print(Colors.red(f"  Error Report: Incomplete data (got {data_len} bytes, need at least 4)"))
        return
    
    d0 = data_bytes[0]  # Error index (0 = latest error)
    d1 = data_bytes[1]  # Error code
    d2 = data_bytes[2]  # Control word that caused the error
    d3 = data_bytes[3]  # Sub-Index or additional info
    
    # Parse error index
    error_index_desc = "Latest Error" if d0 == 0 else f"Error History [{d0}]"
    
    # Parse error code
    error_desc = _parse_error_code(d1)
    
    # Parse the control word that caused the error
    error_cw_info = parse_control_word(d2 | 0x80)  # Add 0x80 to treat as command
    error_cw_name = error_cw_info.get('command_name', 'Unknown')
    
    print(Colors.red(f"  [Error Report]"))
    print(Colors.red(f"    d0 - Error Index: {d0} ({error_index_desc})"))
    print(Colors.red(f"    d1 - Error Code: 0x{d1:02X} - {error_desc}"))
    print(Colors.red(f"    d2 - Error Source CW: 0x{d2:02X} - {error_cw_name}"))
    print(Colors.red(f"    d3 - Sub-Index/Info: 0x{d3:02X} ({d3})"))
    
    if data_len >= 6:
        d4 = data_bytes[4]
        d5 = data_bytes[5]
        print(Colors.yellow(f"    d4~d5 - Factory Use: 0x{d4:02X} 0x{d5:02X}"))


def _print_data_length_mismatch(result: Dict[str, Any], expected_len: int) -> None:
    """
    Print a unified message when response data length does not match expected.
    
    Args:
        result: Response dictionary from SDK function
        expected_len: Expected number of bytes
    """
    station_id = result.get('station_id', 'Unknown')
    data_len = result.get('data_len', 0)
    data_bytes = result.get('data_bytes', [])
    control_word = result.get('control_word', 0)
    crc_valid = result.get('crc_valid', False)
    
    # Parse control word to get command name and description
    cw_info = parse_control_word(control_word)
    cw_name = cw_info.get('command_name', 'Unknown')
    cw_desc = cw_info.get('command_description', '')
    is_error_report = cw_info.get('is_error_report', False)
    
    print(Colors.yellow(f"  Station ID: {station_id}"))
    print(Colors.yellow(f"  Warning: Unexpected response (expected {expected_len} bytes, got {data_len})"))
    print(Colors.yellow(f"  Control Word: 0x{control_word:02X} - {cw_name} ({cw_desc})"))
    print(Colors.yellow(f"  Data Length: {data_len}"))
    
    # If this is an Error Report, parse the error details
    if is_error_report and data_bytes and data_len >= 4:
        _print_error_report(data_bytes, data_len)
    elif data_bytes and data_len > 0:
        print(Colors.yellow(f"  Data Bytes: {[hex(b) for b in data_bytes[:data_len]]}"))
    
    print(Colors.yellow(f"  CRC Valid: {'OK Yes' if crc_valid else 'X No'}"))


def _print_single_result(result: Dict[str, Any], function_name: str) -> None:
    """
    Print single result dictionary based on function name.
    
    Args:
        result: Response dictionary from SDK function
        function_name: Name of the SDK function
    """
    station_id = result.get('station_id', 'Unknown')
    data_bytes = result.get('data_bytes', [])
    data_len = result.get('data_len', 0)
    
    # Extract function base name (remove parameters in parentheses)
    base_name = function_name.split('(')[0].strip()
    
    try:
        if base_name == "SdkGetInitialConfig":
            # ICFG_AMO_IDX or ICFG_ACM_IDX - returns configuration value
            # Response format: [d0=command_index, d1, d2] where d0 is command_index echo, d1 and d2 form a 16-bit int (little-endian)
            if data_len == 3 and len(data_bytes) == 3:
                d0 = data_bytes[0]  # Command index echo
                d1 = data_bytes[1]  # Low byte of configuration value
                d2 = data_bytes[2]  # High byte of configuration value
                config_value = d1 + (d2 << 8)  # Combine d1 and d2 to form 16-bit int (little-endian)
                config_name = "ICFG_AMO_IDX (Auto Motor On)" if "ICFG_AMO_IDX" in function_name else "ICFG_ACM_IDX (Auto Clear Motion)"
                print(Colors.blue(f"  Station ID: {station_id}"))
                print(Colors.blue(f"  Command Index (d0): {d0} (0x{d0:02X})"))
                print(Colors.blue(f"  {config_name} (d1+d2): {config_value} (0x{config_value:04X}, d1=0x{d1:02X}, d2=0x{d2:02X})"))
            else:
                _print_data_length_mismatch(result, 3)
                
        elif base_name == "SdkGetMotorConfig":
            # MTS_MCS_IDX, MTS_CUR_IDX, MTS_PSV_IDX, MTS_BRK_IDX - returns motor configuration value
            # Response format: [d0=command_index, d1, d2] where d0 is command_index echo, d1 and d2 form a 16-bit int (little-endian)
            if data_len == 3 and len(data_bytes) == 3:
                d0 = data_bytes[0]  # Command index echo
                d1 = data_bytes[1]  # Low byte of motor configuration value
                d2 = data_bytes[2]  # High byte of motor configuration value
                config_value = d1 + (d2 << 8)  # Combine d1 and d2 to form 16-bit int (little-endian)

                if "MTS_MCS_IDX" in function_name:
                    config_name = f"MTS_MCS_IDX (Micro-Step): {config_value} (Micro-step subdivision, mm=[1,2,4,8,16,32,64,128])"
                elif "MTS_CUR_IDX" in function_name:
                    current_amp = config_value * 0.1
                    config_name = f"MTS_CUR_IDX (Current Run): {config_value} (Working current: {current_amp:.1f} Amp, ii=[5...80] x0.1 Amp)"
                elif "MTS_PSV_IDX" in function_name:
                    config_name = f"MTS_PSV_IDX (Current Idle): {config_value} (Idle current percentage: {config_value}%, pp=[0...100]%)"
                elif "MTS_BRK_IDX" in function_name:
                    brake_status = "Release" if config_value == 0 else "Lock"
                    config_name = f"MTS_BRK_IDX (Brake Lock): {config_value} ({brake_status}, ss=[0:Release; 1:Lock])"
                else:
                    config_name = "Motor Config"
                print(Colors.blue(f"  Station ID: {station_id}"))
                print(Colors.blue(f"  Command Index (d0): {d0} (0x{d0:02X})"))
                print(Colors.blue(f"  {config_name} (config_value): {config_value} (d1=0x{d1:02X}, d2=0x{d2:02X})"))
            else:
                _print_data_length_mismatch(result, 3)
                
        elif base_name == "SdkSetMotorConfig":
            # SdkSetMotorConfig - parse response value
            # Response format: [d0=command_index, d1, d2] where d0 is command_index echo, d1 and d2 form a 16-bit int (little-endian)
            # Value: 0=Release, 1=Lock
            if data_len == 3 and len(data_bytes) == 3:
                d0 = data_bytes[0]  # Command index echo
                d1 = data_bytes[1]  # Low byte of value
                d2 = data_bytes[2]  # High byte of value
                value = d1 + (d2 << 8)  # Combine d1 and d2 to form 16-bit int (little-endian)
                
                # Determine config name based on function_name
                if "MTS_BRK_IDX" in function_name or "Brake" in function_name:
                    config_name = f"MTS_BRK_IDX (Brake Lock): {value} ( 0=Release, 1=Lock)"
                else:
                    config_name = f"Motor Config: {value}"
                
                print(Colors.green(f"  Station ID: {station_id}"))
                print(Colors.green(f"  Status: OK Success"))
                print(Colors.blue(f"  Command Index (d0): {d0} (0x{d0:02X})"))
                print(Colors.blue(f"  {config_name}"))
            else:
                _print_data_length_mismatch(result, 3)
                
        elif base_name == "SdkGetMotionStatus":
            # Returns different data based on command_index
            # Response format: [command_index] [data...]
            # command_index=0: Get Status Flags and Relative Position
            #   Response: [00] ms0 ms1 00 00 PR0 PR1 PR2 PR3 (8 bytes)
            # command_index=1: Get Current Speed and Absolute Position
            #   Response: [01] sp0 sp1 sp2 PA0 PA1 PA2 PA3 (8 bytes)
            if data_len == 8 and len(data_bytes) == 8:
                cmd_idx = data_bytes[0] if data_len > 0 else 0
                if cmd_idx == 0:
                    # command_index=0: Get Status Flags and Relative Position
                    # data_bytes: [00, ms0, ms1, 00, 00, PR0, PR1, PR2, PR3]
                    status_flags_low = data_bytes[1] + (data_bytes[2] << 8)  # ms0 ms1
                    rel_position = bytes_to_int32_signed(data_bytes, 4)  # PR0 PR1 PR2 PR3 (offset 4)
                    flags = parse_motion_status_flags(status_flags_low)
                    print(Colors.blue(f"  Station ID: {station_id}"))
                    print(Colors.blue(f"  Command Index: 0 (Get Status Flags and Relative Position)"))
                    print(Colors.blue(f"  Status Flags: {flags['status_flags_hex']} (d0={flags['d0_hex']}, d1={flags['d1_hex']})"))
                    print(Colors.blue(f"    d0: mode={flags['mode']} (0-3), SON={flags['SON']} ({flags['SON_desc']}), IN1={flags['IN1']} ({flags['IN1_desc']}), IN2={flags['IN2']} ({flags['IN2_desc']}), IN3={flags['IN3']} ({flags['IN3_desc']}), OP1={flags['OP1']} ({flags['OP1_desc']})"))
                    print(Colors.blue(f"    d1: STOP={flags['STOP']} ({flags['STOP_desc']}), PAIF={flags['PAIF']} ({flags['PAIF_desc']}), TLIF={flags['TLIF']} ({flags['TLIF_desc']}), LOCK={flags['LOCK']} ({flags['LOCK_desc']}), ERR={flags['ERR']} ({flags['ERR_desc']})"))
                    if flags['ERR']:
                        print(Colors.red(f"    [!!] System error is detected (ERR=1)"))
                    if flags['TLIF']:
                        print(Colors.yellow(f"    [!!] Motor stall is detected (TLIF=1)"))
                    print(Colors.blue(f"  Relative Position: {rel_position}"))
                elif cmd_idx == 1:
                    # command_index=1: Get Current Speed and Absolute Position
                    # data_bytes: [01, sp0, sp1, sp2, PA0, PA1, PA2, PA3]
                    speed = bytes_to_int24(data_bytes, 1)  # sp0 sp1 sp2 (offset 1)
                    abs_position = bytes_to_int32_signed(data_bytes, 4)  # PA0 PA1 PA2 PA3 (offset 4)
                    print(Colors.blue(f"  Station ID: {station_id}"))
                    print(Colors.blue(f"  Command Index: 1 (Get Current Speed and Absolute Position)"))
                    print(Colors.blue(f"  Current Speed: {speed} pps"))
                    print(Colors.blue(f"  Absolute Position: {abs_position}"))
                else:
                    print(Colors.yellow(f"  Station ID: {station_id} - Unknown command_index: {cmd_idx}"))
            else:
                _print_data_length_mismatch(result, 8)
                
        elif base_name in ["SdkGetAcceleration", "SdkGetDeceleration", "SdkGetStopDeceleration"]:
            # Returns 32-bit unsigned integer (4 bytes)
            if data_len == 4 and len(data_bytes) == 4:
                value = bytes_to_int32(data_bytes, 0)
                param_name = base_name.replace("SdkGet", "")
                print(Colors.blue(f"  Station ID: {station_id}"))
                print(Colors.blue(f"  {param_name}: {value}"))
            else:
                _print_data_length_mismatch(result, 4)
                
        elif base_name == "SdkGetCutInSpeed":
            # Returns 32-bit unsigned integer (4 bytes) - Start Speed
            if data_len == 4 and len(data_bytes) == 4:
                value = bytes_to_int32(data_bytes, 0)
                print(Colors.blue(f"  Station ID: {station_id}"))
                print(Colors.blue(f"  Cut-In Speed (Start Speed): {value} pps"))
            else:
                _print_data_length_mismatch(result, 4)
                
        elif base_name == "SdkGetPtpMxnA":
            # Returns 32-bit signed integer (4 bytes) - Absolute Position
            if data_len == 4 and len(data_bytes) == 4:
                position = bytes_to_int32_signed(data_bytes, 0)
                print(Colors.blue(f"  Station ID: {station_id}"))
                print(Colors.blue(f"  Absolute Position: {position}"))
            else:
                _print_data_length_mismatch(result, 4)
                
        elif base_name == "SdkGetDIOport":
            # DIO port - parse and print port status
            # Response format: [d0=Input port status, d1=Output port status]
            if data_len == 4 and len(data_bytes) == 4:
                try:
                    dio_parsed = parse_dio_port_response(data_bytes)
                    d0 = dio_parsed['d0']
                    d1 = dio_parsed['d1']
                    inputs = dio_parsed['inputs']
                    outputs = dio_parsed['outputs']
                    
                    print(Colors.blue(f"  Station ID: {station_id}"))
                    print(Colors.blue(f"  Input Port Status (d0): {dio_parsed['d0_hex']} ({d0})"))
                    # Only show IN1, IN2, IN3
                    for input_name in ['IN1', 'IN2', 'IN3']:
                        if input_name in inputs:
                            input_info = inputs[input_name]
                            status_color = Colors.green if input_info['value'] == 1 else Colors.yellow
                            print(Colors.blue(f"    {input_name} (bit{input_info['bit_position']}): {status_color(input_info['status'])}"))
                    
                    print(Colors.blue(f"  Output Port Status (d1): {dio_parsed['d1_hex']} ({d1})"))
                    # Only show OP1
                    if 'OP1' in outputs:
                        output_info = outputs['OP1']
                        status_color = Colors.green if output_info['value'] == 1 else Colors.yellow
                        print(Colors.blue(f"    OP1 (bit{output_info['bit_position']}): {status_color(output_info['status'])}"))
                except ValueError as e:
                    print(Colors.yellow(f"  Station ID: {station_id}"))
                    print(Colors.yellow(f"  Error parsing DIO port response: {e}"))
                    print(Colors.yellow(f"  Raw Data: d0=0x{data_bytes[0]:02X}, d1=0x{data_bytes[1]:02X}" if len(data_bytes) >= 2 else "  Raw Data: Incomplete"))
            else:
                _print_data_length_mismatch(result, 4)
                
        elif base_name == "SdkGetInputLogic":
            # Input Logic has il_info field with detailed information
            if 'il_info' in result:
                il_info = result['il_info']
                print(Colors.blue(f"  Station ID: {station_id}"))
                print(Colors.blue(f"  Input Index: {il_info.get('input_index', 'N/A')} - {il_info.get('index_name', 'N/A')}"))
                if not il_info.get('is_stall_trigger') and not il_info.get('is_torque_limit_trigger'):
                    falling_edge = il_info.get('falling_edge_info', {})
                    rising_edge = il_info.get('rising_edge_info', {})
                    print(Colors.blue(f"  Falling Edge: {falling_edge.get('action_name', 'N/A')}"))
                    print(Colors.blue(f"  Rising Edge: {rising_edge.get('action_name', 'N/A')}"))
            else:
                print(Colors.blue(f"  Station ID: {station_id}"))
                print(Colors.blue(f"  Input Logic: Available"))
                
        elif base_name == "SdkSetInputLogic":
            # SdkSetInputLogic - parse response using parse_il_response
            # Response format: [d0=input_index, d1=falling_edge_action, d2=rising_edge_action]
            if data_len == 3 and len(data_bytes) == 3:
                try:
                    il_info = parse_il_response(data_bytes[:data_len])
                    print(Colors.green(f"  Station ID: {station_id}"))
                    print(Colors.green(f"  Status: OK Success"))
                    print(Colors.blue(f"  Input Index: {il_info.get('input_index', 'N/A')} - {il_info.get('index_name', 'N/A')}"))
                    
                    if il_info.get('is_stall_trigger'):
                        # Stall trigger
                        stall_action_info = il_info.get('stall_action_info', {})
                        print(Colors.blue(f"  Stall Action: {stall_action_info.get('action_name', 'N/A')}"))
                        if il_info.get('stall_fg_flag'):
                            print(Colors.blue(f"  FG Flag: Enabled (Power-On Execution)"))
                    elif il_info.get('is_torque_limit_trigger'):
                        # Torque limit trigger
                        torque_limit_action_info = il_info.get('torque_limit_action_info', {})
                        print(Colors.blue(f"  Torque Limit Action: {torque_limit_action_info.get('action_name', 'N/A')}"))
                        print(Colors.blue(f"  Torque Limit Percent: {il_info.get('torque_limit_percent', 'N/A')}%"))
                        if il_info.get('torque_limit_fg_flag'):
                            print(Colors.blue(f"  FG Flag: Enabled (Power-On Execution)"))
                    else:
                        # Normal input trigger
                        falling_edge = il_info.get('falling_edge_info', {})
                        rising_edge = il_info.get('rising_edge_info', {})
                        print(Colors.blue(f"  Falling Edge: {falling_edge.get('action_name', 'N/A')}"))
                        if il_info.get('falling_edge_fg_flag'):
                            print(Colors.blue(f"    FG Flag: Enabled (Power-On Execution)"))
                        print(Colors.blue(f"  Rising Edge: {rising_edge.get('action_name', 'N/A')}"))
                        if il_info.get('rising_edge_fg_flag'):
                            print(Colors.blue(f"    FG Flag: Enabled (Power-On Execution)"))
                except ValueError as e:
                    print(Colors.yellow(f"  Station ID: {station_id}"))
                    print(Colors.yellow(f"  Error parsing Input Logic response: {e}"))
                    print(Colors.yellow(f"  Raw Data: {[hex(b) for b in data_bytes[:data_len]]}"))
            else:
                _print_data_length_mismatch(result, 3)
                
        elif base_name == "SdkSetJogMxn":
            # SdkSetJogMxn - parse response value
            # Response format: [d0=DVR_*_IDX, d1, d2, d3, d4] where d0 is index, d1~d4 form a 32-bit signed int (little-endian)
            if data_len == 5 and len(data_bytes) == 5:
                d0 = data_bytes[0]  # DVR index
                d1 = data_bytes[1]  # Low byte of speed value
                d2 = data_bytes[2]
                d3 = data_bytes[3]
                d4 = data_bytes[4]  # High byte of speed value
                speed_value = bytes_to_int32_signed(data_bytes, 1)  # Combine d1~d4 to form 32-bit signed int (little-endian)
                
                # Determine index name based on d0 value
                if d0 == DVR_MOD_IDX:
                    index_name = "DVR_MOD_IDX (Query Control Mode)"
                elif d0 == DVR_CUR_IDX:
                    index_name = "DVR_CUR_IDX (Query CUR desired value)"
                elif d0 == DVR_SPD_IDX:
                    index_name = "DVR_SPD_IDX (Query SP desired value)"
                elif d0 == DVR_PRM_IDX:
                    index_name = "DVR_PRM_IDX (Query PR desired value)"
                elif d0 == DVR_PAM_IDX:
                    index_name = "DVR_PAM_IDX (Query PA desired value)"
                elif d0 == DVR_TIS_IDX:
                    index_name = "DVR_TIS_IDX (Query TI desired value)"
                else:
                    index_name = f"Unknown Index ({d0})"
                
                print(Colors.green(f"  Station ID: {station_id}"))
                print(Colors.green(f"  Status: OK Success"))
                print(Colors.blue(f"  Index (d0): {d0} (0x{d0:02X}) - {index_name}"))
                print(Colors.blue(f"  Speed Value (d1~d4): {speed_value} pps"))
            else:
                _print_data_length_mismatch(result, 5)
                
        elif base_name == "SdkSetPtpMxnA":
            # SdkSetPtpMxnA - parse response value
            # Response format: [d0=DVR_PAM_IDX, d1, d2, d3, d4] where d0 is index, d1~d4 form a 32-bit signed int (little-endian)
            if data_len == 5 and len(data_bytes) == 5:
                d0 = data_bytes[0]  # DVR index
                pa_value = bytes_to_int32_signed(data_bytes, 1)  # Combine d1~d4 to form 32-bit signed int (little-endian)
                
                # Determine index name based on d0 value
                if d0 == DVR_MOD_IDX:
                    index_name = "DVR_MOD_IDX (Query Control Mode)"
                elif d0 == DVR_CUR_IDX:
                    index_name = "DVR_CUR_IDX (Query CUR desired value)"
                elif d0 == DVR_SPD_IDX:
                    index_name = "DVR_SPD_IDX (Query SP desired value)"
                elif d0 == DVR_PRM_IDX:
                    index_name = "DVR_PRM_IDX (Query PR desired value)"
                elif d0 == DVR_PAM_IDX:
                    index_name = "DVR_PAM_IDX (Query PA desired value)"
                elif d0 == DVR_TIS_IDX:
                    index_name = "DVR_TIS_IDX (Query TI desired value)"
                else:
                    index_name = f"Unknown Index ({d0})"
                
                print(Colors.green(f"  Station ID: {station_id}"))
                print(Colors.green(f"  Status: OK Success"))
                print(Colors.blue(f"  Index (d0): {d0} (0x{d0:02X}) - {index_name}"))
                print(Colors.blue(f"  Absolute Position Value (d1~d4): {pa_value}"))
            else:
                _print_data_length_mismatch(result, 5)
                
        elif base_name == "SdkSetPtpMxnR":
            # SdkSetPtpMxnR - parse response value
            # Response format: [d0=DVR_PRM_IDX, d1, d2, d3, d4] where d0 is index, d1~d4 form a 32-bit signed int (little-endian)
            if data_len == 5 and len(data_bytes) == 5:
                d0 = data_bytes[0]  # DVR index
                pr_value = bytes_to_int32_signed(data_bytes, 1)  # Combine d1~d4 to form 32-bit signed int (little-endian)
                
                # Determine index name based on d0 value
                if d0 == DVR_MOD_IDX:
                    index_name = "DVR_MOD_IDX (Query Control Mode)"
                elif d0 == DVR_CUR_IDX:
                    index_name = "DVR_CUR_IDX (Query CUR desired value)"
                elif d0 == DVR_SPD_IDX:
                    index_name = "DVR_SPD_IDX (Query SP desired value)"
                elif d0 == DVR_PRM_IDX:
                    index_name = "DVR_PRM_IDX (Query PR desired value)"
                elif d0 == DVR_PAM_IDX:
                    index_name = "DVR_PAM_IDX (Query PA desired value)"
                elif d0 == DVR_TIS_IDX:
                    index_name = "DVR_TIS_IDX (Query TI desired value)"
                else:
                    index_name = f"Unknown Index ({d0})"
                
                print(Colors.green(f"  Station ID: {station_id}"))
                print(Colors.green(f"  Status: OK Success"))
                print(Colors.blue(f"  Index (d0): {d0} (0x{d0:02X}) - {index_name}"))
                print(Colors.blue(f"  Relative Position Value (d1~d4): {pr_value}"))
            else:
                _print_data_length_mismatch(result, 5)
                
        elif base_name == "SdkSetPtpSPD":
            # SdkSetPtpSPD - parse response value
            # Response format: [d0=DVR_SPD_IDX, d1, d2, d3, d4] where d0 is index, d1~d4 form a 32-bit unsigned int (little-endian)
            if data_len == 5 and len(data_bytes) == 5:
                d0 = data_bytes[0]  # DVR index
                speed_value = bytes_to_int32(data_bytes, 1)  # Combine d1~d4 to form 32-bit unsigned int (little-endian)
                
                # Determine index name based on d0 value
                if d0 == DVR_MOD_IDX:
                    index_name = "DVR_MOD_IDX (Query Control Mode)"
                elif d0 == DVR_CUR_IDX:
                    index_name = "DVR_CUR_IDX (Query CUR desired value)"
                elif d0 == DVR_SPD_IDX:
                    index_name = "DVR_SPD_IDX (Query SP desired value)"
                elif d0 == DVR_PRM_IDX:
                    index_name = "DVR_PRM_IDX (Query PR desired value)"
                elif d0 == DVR_PAM_IDX:
                    index_name = "DVR_PAM_IDX (Query PA desired value)"
                elif d0 == DVR_TIS_IDX:
                    index_name = "DVR_TIS_IDX (Query TI desired value)"
                else:
                    index_name = f"Unknown Index ({d0})"
                
                print(Colors.green(f"  Station ID: {station_id}"))
                print(Colors.green(f"  Status: OK Success"))
                print(Colors.blue(f"  Index (d0): {d0} (0x{d0:02X}) - {index_name}"))
                print(Colors.blue(f"  Speed Value (d1~d4): {speed_value} pps"))
            else:
                _print_data_length_mismatch(result, 5)
                
        elif base_name.startswith("SdkSet"):
            # Set commands - just confirm success
            print(Colors.green(f"  Station ID: {station_id}"))
            print(Colors.green(f"  Status: OK Success"))
            if data_len > 0:
                print(Colors.blue(f"  Response Data: {[hex(b) for b in data_bytes[:data_len]]}"))
                
        else:
            # Unknown function - print basic info
            print(Colors.blue(f"  Station ID: {station_id}"))
            print(Colors.blue(f"  Control Word: 0x{result.get('control_word', 0):02X}"))
            print(Colors.blue(f"  Data Length: {data_len}"))
            if data_len > 0:
                print(Colors.blue(f"  Data Bytes: {[hex(b) for b in data_bytes[:data_len]]}"))
            print(Colors.blue(f"  CRC Valid: {'OK Yes' if result.get('crc_valid', False) else 'X No'}"))
            
    except Exception as e:
        # Fallback to basic printing if parsing fails
        print(Colors.yellow(f"  Station ID: {station_id}"))
        print(Colors.yellow(f"  Error parsing response: {e}"))
        print(Colors.yellow(f"  Raw Data: {data_bytes[:data_len] if data_len > 0 else 'No data'}"))


def check_sdk_response(result: Any, function_name: str) -> None:
    """
    Check SDK function response and raise exception if invalid.
    
    This function provides unified error checking for SDK responses.
    It checks for common error conditions and raises appropriate exceptions
    with consistent error messages.
    
    Args:
        result: Response from SDK function (can be None, dict, list, etc.)
        function_name: Name of the SDK function for error messages
        
    Raises:
        NoResponseError: If result is None (no response received)
        NoStationsError: If result is an empty list (no stations responded)
    """
    if result is None:
        print(Colors.red(f"\nX {function_name} failed - No response received"))
        print(Colors.red("No response from device."))
        raise NoResponseError(function_name)
    elif isinstance(result, list) and len(result) == 0:
        print(Colors.red(f"\nX {function_name} failed - No stations responded"))
        print(Colors.red("No stations available."))
        raise NoStationsError(function_name)


def get_device_station_ids(result_ml: Any) -> List[int]:
    """
    Extract device station IDs from SdkGetML response (excluding gateway).
    
    This function filters out the gateway station and returns a sorted list
    of device station IDs. The list is sorted to ensure consistent ordering
    when assigning STATION_ID_X and STATION_ID_Y.
    
    Args:
        result_ml: Response from SdkGetML (list of station responses or None)
    
    Returns:
        Sorted list of device station IDs (excluding gateway)
    
    Raises:
        NoResponseError: If no response received at all
        NoStationsError: If result is not a list or is empty
        NoDeviceStationsError: If only gateway responded (no device stations)
    """
    # Step 1: Use unified error checking for basic validation
    check_sdk_response(result_ml, "SdkGetML")
    
    # Step 2: Filter out gateway station - gateway is not a real device station
    device_stations = [resp for resp in result_ml if resp.get('station_id') != GATEWAY_STATION_ID]
    gateway_responses = [resp for resp in result_ml if resp.get('station_id') == GATEWAY_STATION_ID]
    
    # Step 3: Check if only gateway responded (no device stations)
    if len(device_stations) == 0:
        if len(gateway_responses) > 0:
            print(Colors.yellow(f"\nWARNING Gateway (Station ID {GATEWAY_STATION_ID}) is online, but no other device stations are online."))
            print(Colors.red("No device stations available."))
        else:
            print(Colors.red("\nX SdkGetML failed - No device stations responded"))
            print(Colors.red("No device stations available."))
        raise NoDeviceStationsError(has_gateway=(len(gateway_responses) > 0))
    
    # Step 4: Get and sort list of found device station IDs
    found_station_ids = [resp.get('station_id') for resp in device_stations]
    found_station_ids.sort()  # Sort to ensure consistent ordering
    
    return found_station_ids


def check_sdk_getml_response(result_ml: Any, target_station_id: int) -> None:
    """
    Check SdkGetML response and validate station availability.
    
    SdkGetML (Get Model List) is a broadcast command that queries all stations
    on the network. This function:
    1. Validates that responses were received (uses unified error checking)
    2. Filters out the gateway station (gateway is not a motor device)
    3. Verifies that device stations (motors) are available
    4. Checks that the target station ID exists in the available stations
    
    Gateway vs Device Stations:
    - Gateway (Station ID 0): The UIM2513 gateway device itself
    - Device Stations: Actual motor controllers (UIM342, etc.)
    - We need device stations to control motors, gateway alone is not sufficient
    
    Validation Steps:
    1. Use unified check_sdk_response for basic validation (None/empty list)
    2. Filter gateway from device stations
    3. Check if any device stations exist (gateway alone is not enough)
    4. Check if target station ID is in the available device stations
    
    Args:
        result_ml: Response from SdkGetML (list of station responses or None)
        target_station_id: The station ID we want to use (must be in available stations)
    
    Raises:
        NoResponseError: If no response received at all
        NoStationsError: If result is not a list or is empty
        NoDeviceStationsError: If only gateway responded (no device stations)
        TargetStationNotFoundError: If target_station_id not found in available stations
    """
    # Step 1: Use unified error checking for basic validation
    # This will raise NoResponseError or NoStationsError if needed
    check_sdk_response(result_ml, "SdkGetML")
    
    # Step 2: Filter out gateway station - gateway is not a real device station
    # Gateway (Station ID 0) is the UIM2513 gateway itself, not a motor controller
    # We need actual device stations (motors) to control
    device_stations = [resp for resp in result_ml if resp.get('station_id') != GATEWAY_STATION_ID]
    gateway_responses = [resp for resp in result_ml if resp.get('station_id') == GATEWAY_STATION_ID]
    
    # Step 3: Check if only gateway responded (no device stations)
    # Gateway alone is not sufficient - we need at least one device station (motor)
    if len(device_stations) == 0:
        if len(gateway_responses) > 0:
            print(Colors.yellow(f"\nWARNING Gateway (Station ID {GATEWAY_STATION_ID}) is online, but no other device stations are online."))
            print(Colors.red("No device stations available."))
        else:
            print(Colors.red("\nX SdkGetML failed - No device stations responded"))
            print(Colors.red("No device stations available."))
        raise NoDeviceStationsError(has_gateway=(len(gateway_responses) > 0))
    
    # Step 4: Get list of found device station IDs
    found_station_ids = [resp.get('station_id') for resp in device_stations]
    
    # Step 5: Check if target_station_id exists in found device stations
    # The target station must be one of the available device stations
    if target_station_id not in found_station_ids:
        print(Colors.red(f"\nX Target Station ID {target_station_id} (0x{target_station_id:02X}) is not found in available device stations."))
        print(Colors.yellow(f"Available device station IDs: {found_station_ids}"))
        if len(gateway_responses) > 0:
            print(Colors.blue(f"Gateway (Station ID {GATEWAY_STATION_ID}) is online"))
        print(Colors.red("Target station ID not available."))
        raise TargetStationNotFoundError(
            target_station_id, 
            found_station_ids, 
            has_gateway=(len(gateway_responses) > 0)
        )
    
    # Success: Found device stations (excluding gateway) and target station exists
    print(Colors.green(f"\nOK Gateway online, found {len(device_stations)} device station(s) (excluding gateway)"))
    if len(gateway_responses) > 0:
        print(Colors.blue(f"  - Gateway (Station ID {GATEWAY_STATION_ID}) is online"))
    for resp in device_stations:
        sid = resp.get('station_id', 'Unknown')
        marker = " ← Target" if sid == target_station_id else ""
        print(Colors.green(f"  - Device Station ID {sid} is online{marker}"))


async def execute_and_check_sdk(
    sdk_func: Callable[..., Coroutine[Any, Any, Any]], 
    function_name: str, 
    *args: Any, 
    **kwargs: Any
) -> Any:
    """
    Execute SDK function and check response, raise exception if invalid.
    
    This is the unified function for executing SDK commands with error checking.
    It should be used instead of calling SDK functions directly and then checking
    the response separately.
    
    Usage:
        result = await execute_and_check_sdk(
            SdkSetMotorOn, 
            "SdkSetMotorOn", 
            station_id, 
            1
        )
    
    Args:
        sdk_func: Async SDK function to execute
        function_name: Name of the function for error messages
        *args: Positional arguments to pass to SDK function
        **kwargs: Keyword arguments to pass to SDK function
        
    Returns:
        Response from SDK function (guaranteed to be valid, not None or empty list)
        
    Raises:
        NoResponseError: If no response received
        NoStationsError: If no stations responded
    """
    result = await sdk_func(*args, **kwargs)
    check_sdk_response(result, function_name)
    
    # Print return value based on function name
    _print_sdk_return_value(result, function_name)
    
    return result


def sdk_error_handler(
    function_name: Optional[str] = None,
    reraise: bool = True
) -> Callable:
    """
    Decorator for SDK functions to provide unified error handling.
    
    This decorator wraps SDK functions to automatically check responses
    and raise appropriate exceptions with consistent error messages.
    
    Args:
        function_name: Optional function name for error messages.
                     If not provided, uses the wrapped function's name.
        reraise: If True, re-raises exceptions after logging.
                If False, returns None on error.
    
    Returns:
        Decorator function
        
    Example:
        @sdk_error_handler(function_name="SdkSetMotorOn")
        async def SdkSetMotorOn(station_id: int, enable: int):
            # ... function implementation
            return result
    """
    def decorator(func: Callable[..., Coroutine[Any, Any, Any]]) -> Callable:
        @wraps(func)
        async def wrapper(*args: Any, **kwargs: Any) -> Any:
            name = function_name or func.__name__
            try:
                result = await func(*args, **kwargs)
                check_sdk_response(result, name)
                return result
            except (NoResponseError, NoStationsError) as e:
                if reraise:
                    raise
                return None
            except Exception as e:
                print(Colors.red(f"\nX {name} failed with unexpected error: {type(e).__name__}: {e}"))
                if reraise:
                    raise
                return None
        return wrapper
    return decorator