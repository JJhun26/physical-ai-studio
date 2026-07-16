import xmlrpc.client
import os
import socket
import hashlib
import time
from datetime import datetime
import logging
from functools import wraps
from logging.handlers import RotatingFileHandler
from queue import Queue
import threading
import struct
import sys
import ctypes
from ctypes import *

# from Cython.Compiler.Options import error_on_unknown_names

is_init =False
class ROBOT_AUX_STATE(Structure):
    _pack_ = 1
    _fields_ = [
        ("servoId", c_uint8),         # Servo drive ID number
        ("servoErrCode", c_int),     # Servo drive fault code
        ("servoState", c_int),       # Servo drive state
        ("servoPos", c_double),      # Servo current position
        ("servoVel", c_float),       # Servo current velocity
        ("servoTorque", c_float),    # Servo current torque
    ]

class EXT_AXIS_STATUS(Structure):
    _pack_ = 1
    _fields_ = [
        ("pos", c_double),        # Extended axis position
        ("vel", c_double),        # Extended axis velocity
        ("errorCode", c_int),     # Extended axis fault code
        ("ready", c_uint8),        # Servo ready
        ("inPos", c_uint8),        # Servo in position
        ("alarm", c_uint8),        # Servo alarm
        ("flerr", c_uint8),        # Following error
        ("nlimit", c_uint8),       # At negative limit
        ("pLimit", c_uint8),       # At positive limit
        ("mdbsOffLine", c_uint8),  # Drive 485 bus offline
        ("mdbsTimeout", c_uint8),  # Control card to control box 485 communication timeout
        ("homingStatus", c_uint8), # Extended axis homing status
    ]

class WELDING_BREAKOFF_STATE(Structure):
    _pack_ = 1
    _fields_ = [
        ("breakOffState", ctypes.c_uint8),        # Welding interruption state
        ("weldArcState", ctypes.c_uint8),        # Welding arc interruption state
    ]

"""   
@brief  Robot state feedback data packet
"""
class RobotStatePkg(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_head", ctypes.c_uint16),      # Frame header 0x5A5A
        ("frame_cnt", ctypes.c_uint8),         # Frame count
        ("data_len", ctypes.c_uint16),        # Data length
        ("program_state", ctypes.c_uint8),     # Program running state, 1-stopped; 2-running; 3-paused
        ("robot_state", ctypes.c_uint8),       # Robot motion state, 1-stopped; 2-running; 3-paused; 4-drag
        ("main_code", ctypes.c_int),          # Main fault code
        ("sub_code", ctypes.c_int),           # Sub fault code
        ("robot_mode", ctypes.c_uint8),        # Robot mode, 0-automatic mode; 1-manual mode
        ("jt_cur_pos", ctypes.c_double * 6),  # Robot current joint positions, assuming 6 joints
        ("tl_cur_pos", ctypes.c_double * 6),  # Tool current pose
        ("flange_cur_pos", ctypes.c_double * 6),  # End flange current pose
        ("actual_qd", ctypes.c_double * 6),  # Robot current joint velocities
        ("actual_qdd", ctypes.c_double * 6),  # Robot current joint accelerations
        ("target_TCP_CmpSpeed", ctypes.c_double * 2),  # Robot TCP composite command speed
        ("target_TCP_Speed", ctypes.c_double * 6),  # Robot TCP command speed
        ("actual_TCP_CmpSpeed", ctypes.c_double * 2),  # Robot TCP composite actual speed
        ("actual_TCP_Speed", ctypes.c_double * 6),  # Robot TCP actual speed
        ("jt_cur_tor", ctypes.c_double * 6),  # Current torque
        ("tool", ctypes.c_int),  # Tool number
        ("user", ctypes.c_int),  # Work object number
        ("cl_dgt_output_h", ctypes.c_uint8),  # Digital output 15-8
        ("cl_dgt_output_l", ctypes.c_uint8),  # Digital output 7-0
        ("tl_dgt_output_l", ctypes.c_uint8),  # Tool digital output 7-0 (only bit0-bit1 valid)
        ("cl_dgt_input_h", ctypes.c_uint8),  # Digital input 15-8
        ("cl_dgt_input_l", ctypes.c_uint8),  # Digital input 7-0
        ("tl_dgt_input_l", ctypes.c_uint8),  # Tool digital input 7-0 (only bit0-bit1 valid)
        ("cl_analog_input", ctypes.c_uint16 * 2),  # Control box analog input
        ("tl_anglog_input", ctypes.c_uint16),  # Tool analog input
        ("ft_sensor_raw_data", ctypes.c_double * 6),  # Force/torque sensor raw data
        ("ft_sensor_data", ctypes.c_double * 6),  # Force/torque sensor data
        ("ft_sensor_active", ctypes.c_uint8),  # Force/torque sensor active state, 0-reset, 1-active
        ("EmergencyStop", ctypes.c_uint8),  # Emergency stop flag
        ("motion_done", ctypes.c_int),  # In-position signal
        ("gripper_motiondone", ctypes.c_uint8),  # Gripper motion done signal
        ("mc_queue_len", ctypes.c_int),  # Motion queue length
        ("collisionState", ctypes.c_uint8),  # Collision detection, 1-collision; 0-no collision
        ("trajectory_pnum", ctypes.c_int),  # Trajectory point number
        ("safety_stop0_state", ctypes.c_uint8),  # Safety stop signal SI0
        ("safety_stop1_state", ctypes.c_uint8),  # Safety stop signal SI1
        ("gripper_fault_id", ctypes.c_uint8),  # Faulty gripper number
        ("gripper_fault", ctypes.c_uint16),  # Gripper fault
        ("gripper_active", ctypes.c_uint16),  # Gripper active state
        ("gripper_position", ctypes.c_uint8),  # Gripper position
        ("gripper_speed", ctypes.c_int8),  # Gripper speed
        ("gripper_current", ctypes.c_int8),  # Gripper current
        ("gripper_tmp", ctypes.c_int),  # Gripper temperature
        ("gripper_voltage", ctypes.c_int),  # Gripper voltage
        ("auxState", ROBOT_AUX_STATE),  # 485 extended axis state
        ("extAxisStatus", EXT_AXIS_STATUS*4),  # UDP extended axis state
        ("extDIState", ctypes.c_uint16*8),  # Extended DI input
        ("extDOState", ctypes.c_uint16*8),  # Extended DO output
        ("extAIState", ctypes.c_uint16*4),  # Extended AI input
        ("extAOState", ctypes.c_uint16*4),  # Extended AO output
        ("rbtEnableState", ctypes.c_int),  # Robot enable state
        ("jointDriverTorque", ctypes.c_double * 6),  # Joint drive current torque
        ("jointDriverTemperature", ctypes.c_double * 6),  # Joint drive current temperature
        ("year", ctypes.c_uint16),  # Year
        ("mouth", ctypes.c_uint8),  # Month
        ("day", ctypes.c_uint8),  # Day
        ("hour", ctypes.c_uint8),  # Hour
        ("minute", ctypes.c_uint8),  # Minute
        ("second", ctypes.c_uint8),  # Second
        ("millisecond", ctypes.c_uint16),  # Millisecond
        ("softwareUpgradeState", ctypes.c_int),  # Robot software upgrade state
        ("endLuaErrCode", ctypes.c_uint16),  # End LUA running state
        ("cl_analog_output", ctypes.c_uint16 * 2),  # Control box analog output
        ("tl_analog_output", ctypes.c_uint16),  # Tool analog output
        ("gripperRotNum", ctypes.c_float),  # Rotary gripper current rotation turns
        ("gripperRotSpeed", ctypes.c_uint8),  # Rotary gripper current rotation speed percentage
        ("gripperRotTorque", ctypes.c_uint8),  # Rotary gripper current rotation torque percentage
        ("weldingBreakOffState", WELDING_BREAKOFF_STATE), # Welding interruption state
        ("jt_tgt_tor", ctypes.c_double * 6),  # Joint command torque
        ("smartToolState", ctypes.c_int),  # SmartTool handle button state
        ("wideVoltageCtrlBoxTemp", ctypes.c_float),  # Wide-voltage control box temperature
        ("wideVoltageCtrlBoxFanCurrent", ctypes.c_uint16),  # Wide-voltage control box fan current (ma)
        ("toolCoord", ctypes.c_double * 6),  # Tool coordinate frame                                                                2025.09.17---3.8.6
        ("wobjCoord", ctypes.c_double * 6),  # Work object coordinate frame
        ("extoolCoord", ctypes.c_double * 6),  # External tool coordinate frame
        ("exAxisCoord", ctypes.c_double * 6),  # Extended axis coordinate frame
        ("load", ctypes.c_double),  # Load mass
        ("loadCog", ctypes.c_double * 3),  # Load center of gravity
        ("lastServoTarget", ctypes.c_double * 6),  # Last ServoJ target position in the queue                                          2025.10.15---3.8.7
        ("servoJCmdNum", ctypes.c_int),  # ServoJ command count
        ("check_sum", ctypes.c_uint16)]  # Checksum


class BufferedFileHandler(RotatingFileHandler):
    def __init__(self, filename, mode='a', maxBytes=0, backupCount=0, encoding=None, delay=False):
        super().__init__(filename, mode, maxBytes, backupCount, encoding, delay)
        self.buffer = []

    def emit(self, record):
        # log_entry = self.format(record)  # Format the log record
        # print(log_entry)  # Print the log entry
        if RPC.log_output_model == 2:
            RPC.queue.put(record)
        else:
            self.buffer.append(record)
            if len(self.buffer) >= 50:
                for r in self.buffer:
                    super().emit(r)
                self.buffer = []


class LogWriterThread(threading.Thread):
    def __init__(self, queue, log_handler):
        super().__init__()
        self.queue = queue
        self.log_handler = log_handler
        self.daemon = True

    def run(self):
        while True:
            record = self.queue.get()
            if record is None:
                break
            log_entry = self.log_handler.format(record)
            self.log_handler.stream.write(log_entry + self.log_handler.terminator)
            self.log_handler.flush()


def calculate_file_md5(file_path):
    if not os.path.exists(file_path):
        raise ValueError(f"{file_path} does not exist")
    md5 = hashlib.md5()
    with open(file_path, 'rb') as file:
        while chunk := file.read(8192):  # Read in 8KB chunks
            md5.update(chunk)
    return md5.hexdigest()


def xmlrpc_timeout(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if RPC.is_conect == False:
            return -4
        else:
            result = func(self, *args, **kwargs)
            return result

    return wrapper


class RobotError:
    ERR_SUCCESS = 0
    ERR_POINTTABLE_NOTFOUND = -7  # Uploaded file does not exist
    # ERR_SAVE_FILE_PATH_NOT_FOUND = -6  # Save file path does not exist
    ERR_NOT_FOUND_LUA_FILE = -5  # lua file does not exist
    ERR_RPC_ERROR = -4
    ERR_SOCKET_COM_FAILED = -2
    ERR_OTHER = -1
    ERROR_RECONN = -8
    ERR_SOCKET_RECV_FAILED=-16    #/* socket receive failed */
    ERR_SOCKET_SEND_FAILED=-15    #/* socket send failed */
    ERR_FILE_OPEN_FAILED=-14    #/* file open failed */
    ERR_FILE_TOO_LARGE=-13    #/* file size exceeds limit */
    ERR_UPLOAD_FILE_ERROR=-12    #/* upload file error */
    ERR_FILE_NAME=-11    #/* file name error */
    ERR_DOWN_LOAD_FILE_WRITE_FAILED=-10    #/* download file write failed */
    ERR_DOWN_LOAD_FILE_CHECK_FAILED=-9     #/* file download check failed */
    ERR_DOWN_LOAD_FILE_FAILED=-8     #/* file download failed */
    ERR_UPLOAD_FILE_NOT_FOUND=-7     #/* uploaded file exists */
    ERR_SAVE_FILE_PATH_NOT_FOUND=-6     #/* save file path does not exist */


class RPC():
    ip_address = "192.168.58.2"

    logger = None
    log_output_model = -1
    queue = Queue(maxsize=10000 * 1024)
    logging_thread = None
    is_conect = True
    ROBOT_REALTIME_PORT = 20004
    # BUFFER_SIZE = 1024 * 2
    BUFFER_SIZE = 1024 * 1024
    thread=  threading.Thread()
    SDK_state=True

    sock_cli_state_state = False
    closeRPC_state = False
    reconnect_lock = False
    reconnect_flag = False
    g_sock_com_err = RobotError.ERROR_RECONN


    def __init__(self, ip="192.168.58.2"):
        self.lock = threading.Lock()  # Add lock
        self.ip_address = ip
        link = 'http://' + self.ip_address + ":20003"
        self.robot = xmlrpc.client.ServerProxy(link)#xmlrpc connects to robot port 20003, used to send robot command data frames

        self.sock_cli_state = None
        self.robot_realstate_exit = False
        self.robot_state_pkg = RobotStatePkg#Robot state data

        self.stop_event = threading.Event()  # Stop event
        self.connect_to_robot()
        thread= threading.Thread(target=self.robot_state_routine_thread)#Create thread to loop and receive robot state data
        thread.daemon = True
        thread.start()
        time.sleep(1)
        print(self.robot)


        try:
            # Call the XML-RPC method
            socket.setdefaulttimeout(1)
            self.robot.GetControllerIP()
        except socket.timeout:
            print("XML-RPC connection timed out.")
            RPC.is_conect = False

        except socket.error as e:
            print("There may be a network fault, please check the network connection.")
            RPC.is_conect = False
        except Exception as e:
            print("An error occurred during XML-RPC call:", e)
            RPC.is_conect = False
        finally:
            # Restore default timeout
            self.robot = None
            socket.setdefaulttimeout(None)
            self.robot = xmlrpc.client.ServerProxy(link)

    def connect_to_robot(self):
        """Connect to the robot's real-time port"""
        # print("SDK connecting to robot")
        self.sock_cli_state = socket.socket(socket.AF_INET, socket.SOCK_STREAM)#Socket connects to robot port 20004, used to update robot state data in real time
        self.sock_cli_state.settimeout(0.3)  # Set timeout to 0.05 seconds
        try:
            self.sock_cli_state.connect((self.ip_address, self.ROBOT_REALTIME_PORT))
            self.sock_cli_state_state = True
        except socket.timeout:
            # print("Connection timed out, please check the network connection.")
            self.sock_cli_state_state = False
            return False
        except Exception as ex:
            self.sock_cli_state_state = False
            print("SDK failed to connect to robot real-time port", ex)
            return False
        return True

    def reconnect(self):
        """Auto reconnect"""
        max_retries = 1000
        retry_interval = 2  # 2 seconds
        # with self.lock:  # Acquire lock
        # RPC.is_conect = False
        # print("Disconnected")
        self.reconnect_flag = True
        for attempt in range(max_retries):
            # print(f"Attempting to reconnect, attempt {attempt + 1}")
            # print(f"Attempting to reconnect")
            # Ensure self.sock_cli_state is a new socket object
            if self.sock_cli_state:
                self.sock_cli_state.close()  # Close the old socket
                self.sock_cli_state = None  # Reset to None
            # Reinitialize the XML-RPC connection
            # self.robot = None
            # link = 'http://' + self.ip_address + ":20003"
            # self.robot = xmlrpc.client.ServerProxy(link)
            # Attempt to connect

            if self.connect_to_robot():
                # print("Reconnection successful")
                self.SDK_state = True
                self.reconnect_flag = False
                return True
                # Verify the XML-RPC connection
                # try:
                #     time.sleep(1)
                #     self.Mode(0)  # Call a simple XML-RPC method
                #     time.sleep(1)
                #     self.Mode(1)  # Call a simple XML-RPC method
                #     time.sleep(1)
                #     # self.robot.Mode(0)  # Call a simple XML-RPC method
                #     # time.sleep(1)
                #     # self.robot.Mode(1)  # Call a simple XML-RPC method
                #     # time.sleep(1)
                #     # self.Mode(0)  # Call a simple XML-RPC method
                #     print("XML-RPC connection verification succeeded")
                #     self.reconnect_flag = False
                #     # RPC.is_conect = True
                #     return True
                # except Exception as ex:
                #     print("XML-RPC connection verification failed:", ex)
                #     self.SDK_state = False
            else:
                # print(f"Reconnection failed, retrying after {retry_interval} seconds...")
                time.sleep(retry_interval)

        print("Maximum reconnection attempts reached, connection failed")
        self.SDK_state = False
        return False
        #
        # print("Automatic reconnection mechanism")
        # for i in range(1,6):
        #     print("---")
        #     time.sleep(2)
        #     try:
        #         self.sock_cli_state = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #         self.sock_cli_state.connect((self.ip_address, self.ROBOT_REALTIME_PORT))
        #         self.sock_cli_state_state = True
        #     except Exception as ex:
        #         self.sock_cli_state_state = False
        #         print("SDK failed to connect to robot real-time port", ex)
        #     if self.sock_cli_state_state:
        #         # self.sock_cli_state_state = True
        #         return


    def robot_state_routine_thread_old(self):
        """Thread routine for processing robot state data packets"""

        while not self.closeRPC_state:
            recvbuf = bytearray(self.BUFFER_SIZE)
            tmp_recvbuf = bytearray(self.BUFFER_SIZE)
            state_pkg = bytearray(self.BUFFER_SIZE)
            find_head_flag = False
            index = 0
            length = 0
            tmp_len = 0
            # if not self.sock_cli_state_state:
            #     if not self.connect_to_robot():
            #         return


            try:
                # while not self.robot_realstate_exit:
                while not self.robot_realstate_exit and not self.stop_event.is_set():
                    recvbyte = self.sock_cli_state.recv_into(recvbuf)
                    # timestamp_ms = int(datetime.now().timestamp() * 1000)
                    # print("Current timestamp (milliseconds):", timestamp_ms)
                    if recvbyte <= 0:
                        self.sock_cli_state.close()
                        print("Received robot state bytes -1")
                        # self.reconnect()
                        return
                    else:
                        if tmp_len > 0:
                            if tmp_len + recvbyte <= self.BUFFER_SIZE:
                                recvbuf = tmp_recvbuf[:tmp_len] + recvbuf[:recvbyte]
                                recvbyte += tmp_len
                                tmp_len = 0
                            else:
                                tmp_len = 0

                        for i in range(recvbyte):
                            if format(recvbuf[i], '02X') == "5A" and not find_head_flag:
                                if i + 4 < recvbyte:
                                    if format(recvbuf[i+1], '02X') == "5A":
                                        find_head_flag = True
                                        state_pkg[0] = recvbuf[i]
                                        index += 1
                                        length = length | recvbuf[i + 4]
                                        length = length << 8
                                        length = length | recvbuf[i + 3]
                                    else:
                                        continue
                                else:
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    break
                            elif find_head_flag and index < length + 5:
                                state_pkg[index] = recvbuf[i]
                                index += 1
                            elif find_head_flag and index >= length + 5:
                                if i + 1 < recvbyte:
                                    checksum = sum(state_pkg[:index])
                                    checkdata = 0
                                    checkdata = checkdata | recvbuf[i + 1]
                                    checkdata = checkdata << 8
                                    checkdata = checkdata | recvbuf[i]

                                    if checksum == checkdata:
                                        self.robot_state_pkg = RobotStatePkg.from_buffer_copy(recvbuf)
                                        find_head_flag = False
                                        index = 0
                                        length = 0
                                        i += 1
                                    else:
                                        # print(checksum,":",checkdata,"===========================")
                                        self.robot_state_pkg.jt_cur_pos[0] = 0
                                        self.robot_state_pkg.jt_cur_pos[1] = 0
                                        self.robot_state_pkg.jt_cur_pos[2] = 0
                                        find_head_flag = False
                                        index = 0
                                        length = 0
                                        i += 1
                                else:
                                    print("4.2")
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    break
                            else:
                                continue
            except Exception as ex:
                if not self.closeRPC_state:
                    self.sock_cli_state.close()
                    self.sock_cli_state_state = False
                    self.SDK_state=False
                    # self.reconnect()
                    # print("SDK failed to read robot real-time data", ex)
                    self.reconnect()

    def robot_state_routine_thread(self):
        """Thread routine for processing robot state data packets"""

        while not self.closeRPC_state:
            recvbuf = bytearray(self.BUFFER_SIZE)
            tmp_recvbuf = bytearray(self.BUFFER_SIZE)
            state_pkg = bytearray(self.BUFFER_SIZE)
            find_head_flag = False
            index = 0
            length = 0
            tmp_len = 0
            expected_length = self.BUFFER_SIZE  # Initial expected receive length

            try:
                while not self.robot_realstate_exit and not self.stop_event.is_set():
                    recvbyte = self.sock_cli_state.recv_into(recvbuf)
                    # print(f"Received robot state bytes {recvbyte}")
                    # print("Python struct size:", sizeof(self.robot_state_pkg))
                    if recvbyte <= 0:
                        self.sock_cli_state.close()
                        print("Received robot state bytes -1")
                        if not self.reconnect():
                            return
                        continue

                    # Process temporary buffer data
                    if tmp_len > 0:
                        if tmp_len + recvbyte <= self.BUFFER_SIZE:
                            recvbuf[:tmp_len + recvbyte] = tmp_recvbuf[:tmp_len] + recvbuf[:recvbyte]
                            recvbyte += tmp_len
                            tmp_len = 0
                        else:
                            tmp_len = 0

                    i = 0
                    while i < recvbyte:
                        # Find packet header
                        if format(recvbuf[i], '02X') == "5A" and not find_head_flag:
                            if i + 4 < recvbyte and format(recvbuf[i + 1], '02X') == "5A":
                                find_head_flag = True
                                state_pkg[0] = recvbuf[i]
                                index = 1
                                length = (recvbuf[i + 4] << 8) | recvbuf[i + 3]

                                #Check whether the length exceeds the expected value
                                if length + 7 > expected_length:
                                    expected_length = length + 7
                                    # Need to receive more data
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    find_head_flag = False
                                    break

                                i += 1
                            else:
                                i += 1
                                continue

                        # Packet header found, collect data
                        elif find_head_flag and index < length + 5:
                            if i >= recvbyte:
                                break

                            state_pkg[index] = recvbuf[i]
                            index += 1
                            i += 1

                        # Check checksum
                        elif find_head_flag and index >= length + 5:
                            if i + 1 < recvbyte:
                                checksum = sum(state_pkg[:index])
                                checkdata = (recvbuf[i + 1] << 8) | recvbuf[i]

                                if checksum == checkdata:
                                    self.robot_state_pkg = RobotStatePkg.from_buffer_copy(state_pkg[:sizeof(self.robot_state_pkg)])

                                    # print(f"@@@@@@{self.robot_state_pkg.toolCoord[0]}")
                                    find_head_flag = False
                                    index = 0
                                    length = 0
                                    expected_length = self.BUFFER_SIZE  # Reset expected length
                                    i += 2
                                else:
                                    # Checksum failure handling
                                    self.robot_state_pkg.jt_cur_pos[0] = 0
                                    self.robot_state_pkg.jt_cur_pos[1] = 0
                                    self.robot_state_pkg.jt_cur_pos[2] = 0
                                    find_head_flag = False
                                    index = 0
                                    length = 0
                                    i += 2
                            else:
                                # Insufficient data, save to temporary buffer
                                tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                tmp_len = recvbyte - i
                                break
                        else:
                            i += 1

            except Exception as ex:
                if not self.closeRPC_state:
                    self.sock_cli_state.close()
                    self.sock_cli_state_state = False
                    self.SDK_state = False
                    # print("SDK failed to read robot real-time data", ex)
                    self.reconnect()

    def robot_state_routine_thread_new(self):
        """Thread routine for processing robot state data packets"""

        while not self.closeRPC_state:
            # Use dynamic buffer size, initially self.BUFFER_SIZE
            current_buffer_size = self.BUFFER_SIZE
            recvbuf = bytearray(current_buffer_size)
            tmp_recvbuf = bytearray(current_buffer_size)
            state_pkg = bytearray(current_buffer_size)
            find_head_flag = False
            index = 0
            length = 0
            tmp_len = 0

            try:
                while not self.robot_realstate_exit and not self.stop_event.is_set():
                    recvbyte = self.sock_cli_state.recv_into(recvbuf)

                    if recvbyte <= 0:
                        self.sock_cli_state.close()
                        print("Received robot state bytes -1")
                        if not self.reconnect():
                            return
                        continue

                    # Process temporary buffer data
                    if tmp_len > 0:
                        if tmp_len + recvbyte <= current_buffer_size:
                            recvbuf[:tmp_len + recvbyte] = tmp_recvbuf[:tmp_len] + recvbuf[:recvbyte]
                            recvbyte += tmp_len
                            tmp_len = 0
                        else:
                            # Need to enlarge the receive buffer
                            new_buffer_size = tmp_len + recvbyte
                            new_recvbuf = bytearray(new_buffer_size)
                            new_recvbuf[:tmp_len] = tmp_recvbuf[:tmp_len]
                            new_recvbuf[tmp_len:tmp_len + recvbyte] = recvbuf[:recvbyte]
                            recvbuf = new_recvbuf
                            current_buffer_size = new_buffer_size
                            recvbyte += tmp_len
                            tmp_len = 0

                    i = 0
                    while i < recvbyte:
                        # Find packet header
                        if format(recvbuf[i], '02X') == "5A" and not find_head_flag:
                            if i + 4 < recvbyte and format(recvbuf[i + 1], '02X') == "5A":
                                find_head_flag = True
                                state_pkg[0] = recvbuf[i]
                                index = 1
                                length = (recvbuf[i + 4] << 8) | recvbuf[i + 3]

                                # Check whether the length exceeds the current buffer
                                if length + 7 > current_buffer_size:
                                    # Need to enlarge the buffer
                                    new_buffer_size = length + 7 + 100  # Add some extra space
                                    # Enlarge all related buffers
                                    new_recvbuf = bytearray(new_buffer_size)
                                    new_state_pkg = bytearray(new_buffer_size)
                                    new_tmp_recvbuf = bytearray(new_buffer_size)

                                    # Copy existing data
                                    if recvbyte - i > 0:
                                        new_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    tmp_recvbuf = new_tmp_recvbuf

                                    recvbuf = new_recvbuf
                                    state_pkg = new_state_pkg
                                    current_buffer_size = new_buffer_size

                                    find_head_flag = False
                                    break  # Break out of the inner loop and receive data again

                                i += 1
                            else:
                                i += 1
                                continue

                        # Packet header found, collect data
                        elif find_head_flag and index < length + 5:
                            if i >= recvbyte:
                                break

                            # Check whether the index is out of bounds
                            if index >= len(state_pkg):
                                # Need to enlarge the state_pkg buffer
                                new_size = index + 100
                                new_state_pkg = bytearray(new_size)
                                new_state_pkg[:len(state_pkg)] = state_pkg
                                state_pkg = new_state_pkg

                            state_pkg[index] = recvbuf[i]
                            index += 1
                            i += 1

                        # Check checksum
                        elif find_head_flag and index >= length + 5:
                            if i + 1 < recvbyte:
                                checksum = sum(state_pkg[:index])
                                checkdata = (recvbuf[i + 1] << 8) | recvbuf[i]

                                if checksum == checkdata:
                                    # Ensure there is enough data to parse
                                    if index >= 1184:  # According to the error message, at least 1184 bytes are required
                                        try:
                                            self.robot_state_pkg = RobotStatePkg.from_buffer_copy(state_pkg[:index])
                                            print(f"@@@@@@{self.robot_state_pkg.toolCoord[0]}")
                                        except Exception as e:
                                            print(f"Failed to parse data packet: {e}")
                                    else:
                                        print(f"Data packet size insufficient: {index} < 1184")

                                    find_head_flag = False
                                    index = 0
                                    length = 0
                                    i += 2
                                else:
                                    # Checksum failure handling
                                    print(f"Checksum failed: {checksum} != {checkdata}")
                                    self.robot_state_pkg.jt_cur_pos[0] = 0
                                    self.robot_state_pkg.jt_cur_pos[1] = 0
                                    self.robot_state_pkg.jt_cur_pos[2] = 0
                                    find_head_flag = False
                                    index = 0
                                    length = 0
                                    i += 2
                            else:
                                # Insufficient data, save to temporary buffer
                                if recvbyte - i > 0:
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                break
                        else:
                            i += 1

            except Exception as ex:
                if not self.closeRPC_state:
                    self.sock_cli_state.close()
                    self.sock_cli_state_state = False
                    self.SDK_state = False
                    # print("SDK failed to read robot real-time data", ex)
                    self.reconnect()

    def setup_logging(self, output_model=1, file_path="", file_num=5):
        """Used for processing logs"""
        self.logger = logging.getLogger("RPCLogger")
        log_level = logging.DEBUG
        log_handler = None

        if not file_path:
            current_datetime = datetime.now()
            formatted_date = current_datetime.strftime("%Y%m%d")
            file_name = "fairino_" + formatted_date + ".log"
            file_path = os.path.join(os.getcwd(), file_name)  # Use the current working directory if no path is provided
        else:
            file_path = os.path.abspath(file_path)  # Get the absolute path

        # Check whether the directory exists
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            # print(f"Error: The directory '{directory}' does not exist. Logging setup aborted.")
            return -1  # Return an error code if the directory does not exist

        if output_model == 0:
            RPC.log_output_model = 0
            log_handler = RotatingFileHandler(file_path, maxBytes=50 * 1024, backupCount=file_num)
        elif output_model == 1:
            RPC.log_output_model = 1
            log_handler = BufferedFileHandler(file_path, mode='a', maxBytes=50 * 1024, backupCount=file_num)
        elif output_model == 2:
            RPC.log_output_model = 2
            log_handler = BufferedFileHandler(file_path, mode='a', maxBytes=50 * 1024, backupCount=file_num)
            self.start_logging_thread(log_handler)

        formatter = logging.Formatter('[%(levelname)s] [%(asctime)s pid:%(process)d]  %(message)s')
        if log_handler:
            log_handler.setFormatter(formatter)
            self.logger.addHandler(log_handler)
        else:
            print("Error: Log handler not created. Logging setup aborted.")

        return 0  # Return a success code if logging is set up successfully

    def start_logging_thread(self, log_handler):
        """Create a thread for log storage"""
        logging_thread = LogWriterThread(RPC.queue, log_handler)
        RPC.logging_thread = logging_thread  # Store the reference to the logging thread
        logging_thread.start()

    def join_logging_thread(self):
        """Notify the logging thread to stop"""
        if RPC.logging_thread is not None:
            RPC.queue.put(None)  # Notify the logging thread to stop
            RPC.logging_thread.join()  # Wait for the logging thread to finish

    def __del__(self):
        """Garbage collector, similar to a destructor"""
        self.join_logging_thread()#Stop the logging thread

    def set_log_level(self, lvl):
        """Set the logging level"""
        levels = {1: logging.ERROR, 2: logging.WARNING, 3: logging.INFO, 4: logging.DEBUG}
        log_level = levels.get(lvl, logging.DEBUG)
        self.logger.setLevel(log_level)
        return log_level

    def log_call(func):
        """Log operation for recording function calls"""
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            args_str = ', '.join(map(repr, args))
            kwargs_str = ', '.join([f"{key}={value}" for key, value in kwargs.items()])
            if (kwargs_str) == "":
                call_message = f"Calling {func.__name__}" + f"({args_str}" + ")."
            else:
                call_message = f"Calling {func.__name__}" + f"({args_str}" + "," + f"{kwargs_str})."

            self.log_info(call_message)
            result = func(self, *args, **kwargs)
            if isinstance(result, (list, tuple)) and len(result) > 0:
                if result[0] == 0:
                    self.log_debug(f"{func.__name__} returned: {result}.")
                else:
                    self.log_error(f"{func.__name__} Error occurred. returned: {result}")
            else:
                if result == 0:
                    self.log_debug(f"{func.__name__} returned: {result}.")
                else:
                    self.log_error(f"{func.__name__} Error occurred. returned: {result}")

            return result

        return wrapper

    def log_debug(self, message):
        """Used for recording debug level logs"""
        if self.logger:
            self.logger.debug(message)

    def log_info(self, message):
        """Used for recording info level logs"""
        if self.logger:
            self.logger.info(message)

    def log_warning(self, message):
        """Used for recording warning level logs"""
        if self.logger:
            self.logger.warning(message)

    def log_error(self, message):
        """Used for recording errror level logs"""
        if self.logger:
            self.logger.error(message)

    def send_message(self, message):
        """Create a TCP connection to send a message"""
        # Create a TCP/IP socket
        sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        port = 8080  # Fixed port number is 8080
        try:
            # Connect to the server
            sock1.connect((self.ip_address, 8080))
            # Send data
            sock1.sendall(message.encode('utf-8'))

            response = sock1.recv(1024).decode('utf-8')

            value =response.split('III')
            if len(value) ==6:
                if value[4] == "1":
                    return 0
                else:
                    print("error happended",value[4])
                    return -1
            else:
                return -1
        except Exception as e:
            print(f'An error occurred: {e}')

        finally:
            sock1.close()

    """2024.12.23"""
    """   
       @brief Safety code acquisition
       @return Error code Success - 0, Failure - error code
    """

    def GetSafetyCode(self):
        if (self.robot_state_pkg.safety_stop0_state == 1) or (self.robot_state_pkg.safety_stop1_state == 1):
            return 99
        return 0
    """2024.12.23"""
    """   
    ***************************************************************************Robot Basics********************************************************************************************
    """

    """   
    @brief  Query SDK version number
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    @return Return value (returned on successful call) version SDK version number
    """

    @log_call
    @xmlrpc_timeout
    def GetSDKVersion(self):
        error = 0
        sdk = ["SDK:V2.2.1", "Robot:V3.9.1"]
        return error, sdk

    """   
    @brief  Query controller IP
    @param  [in] NULL
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) ip  controller IP
    """

    @log_call
    @xmlrpc_timeout
    def GetControllerIP(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetControllerIP()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,0

    """   
    @brief  Control robot manual/automatic mode switching
    @param  [in] Required parameter state: 0-automatic mode 1-manual mode
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def Mode(self, state):
        flag = True
        while self.reconnect_flag:
            time.sleep(0.1)

        state = int(state)
        flag = True
        while flag:
            try:
                error = self.robot.Mode(state)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Control the robot to enter or exit drag teaching mode
    @param  [in] Required parameter state: 0-exit drag teaching mode, 1-enter drag teaching mode
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def DragTeachSwitch(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)  # Force conversion to int type
        flag = True
        while flag:
            try:
                error = self.robot.DragTeachSwitch(state)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Query whether the robot is in drag teaching mode
    @param  [in] NULL
    @return Error code Success-0, Failure-error code
    @return Return value (returned on successful call) state 0-not in drag teaching mode, 1-in drag teaching mode
    """

    @log_call
    @xmlrpc_timeout
    def IsInDragTeach(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.IsInDragTeach()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Control robot enable or disable
    @param  [in] Required parameter state: 0-disable, 1-enable
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def RobotEnable(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)  # Force conversion to int type
        flag = True
        while flag:
            try:
                error = self.robot.RobotEnable(state)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    ***************************************************************************Robot Motion********************************************************************************************
    """

    """   
    @brief  jog jogging
    @param  [in] Required parameter ref: 0-joint jogging, 2-base coordinate frame jogging, 4-tool coordinate frame jogging, 8-work object coordinate frame jogging
    @param  [in] Required parameter nb: 1-joint 1 (or x axis), 2-joint 2 (or y axis), 3-joint 3 (or z axis), 4-joint 4 (or rotation about x axis), 5-joint 5 (or rotation about y axis), 6-joint 6 (or rotation about z axis)
    @param  [in] Required parameter dir: 0-negative direction, 1-positive direction
    @param  [in] Required parameter max_dis: maximum angle/distance per jog, unit deg or mm
    @param  [in] Default parameter vel: velocity percentage, [0~100] default 20
    @param  [in] Default parameter acc: acceleration percentage, [0~100] default 100
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def StartJOG(self, ref, nb, dir, max_dis, vel=20.0, acc=100.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        ref = int(ref)  # Force conversion to int type
        nb = int(nb)  # Force conversion to int type
        dir = int(dir)  # Force conversion to int type
        max_dis = float(max_dis)  # Force conversion to float type
        vel = float(vel)  # Force conversion to float type
        acc = float(acc)  # Force conversion to float type
        flag = True
        while flag:
            try:
                error = self.robot.StartJOG(ref, nb, dir, vel, acc, max_dis)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  jog jogging deceleration stop
    @param  [in] Required parameter: 1-joint jogging stop, 3-base coordinate frame jogging stop, 5-tool coordinate frame jogging stop, 9-work object coordinate frame jogging stop
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def StopJOG(self, ref):
        while self.reconnect_flag:
            time.sleep(0.1)
        ref = int(ref)  # Force conversion to int type
        flag = True
        while flag:
            try:
                error = self.robot.StopJOG(ref)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  jog jogging immediate stop
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ImmStopJOG(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ImmStopJOG()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Joint space motion (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter joint_pos: target joint position, unit [deg]
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Default parameter desc_pos: target Cartesian pose, unit [mm][deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls forward kinematics to solve for the return value
    @param  [in] Default parameter vel: velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc: acceleration percentage, [0~100] not open yet, default 0.0 
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter exaxis_pos: external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter blendT:[-1.0]-move to position (blocking), [0~500.0]-blend time (non-blocking), unit [ms] default -1.0
    @param  [in] Default parameter offset_flag:[0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos: pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveJ(self, joint_pos, tool, user, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
              exaxis_pos=[0.0, 0.0, 0.0, 0.0], blendT=-1.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        joint_pos = list(map(float, joint_pos))
        tool = int(tool)
        user = int(user)
        desc_pos = list(map(float, desc_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        exaxis_pos = list(map(float, exaxis_pos))
        blendT = float(blendT)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        if (desc_pos[0] == 0.0) and (desc_pos[1] == 0.0) and (desc_pos[2] == 0.0) and (desc_pos[3] == 0.0) and (
                desc_pos[4] == 0.0) and (desc_pos[5] == 0.0):  # If no parameter is entered, call forward kinematics to solve
            ret = self.robot.GetForwardKin(joint_pos)  # Forward kinematics solution
            if ret[0] == 0:
                desc_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveJ(joint_pos, desc_pos, tool, user, vel, acc, ovl, exaxis_pos, blendT, offset_flag,
                                 offset_pos)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Cartesian space linear motion (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter desc_pos: target Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Default parameter joint_pos: target joint position, unit [deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls inverse kinematics to solve for the return value
    @param  [in] Default parameter vel: velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc: acceleration percentage, [0~100] not open yet default 0.0
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter blendR:[-1.0]-move to position (blocking), [0~1000]-blend radius (non-blocking), unit [mm] default -1.0
    @param  [in] Default parameter blendMode transition mode; 0-inscribed transition; 1-corner transition
    @param  [in] Default parameter exaxis_pos: external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter search:[0]-no wire searching, [1]-wire searching
    @param  [in] Default parameter offset_flag:[0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos: pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter oacc acceleration scaling factor [0-100]/physical acceleration (mm/s2) default 100
    @param  [in] Default parameter config inverse solution joint space configuration, [-1]-solve with reference to the current joint position, [0~7]-solve based on a specific joint space configuration, default -1
    @param  [in] Default parameter velAccParamMode velocity/acceleration parameter mode; 0-percentage; 1-physical velocity (mm/s) acceleration (mm/s2) default 0
    @param  [in] Default parameter overSpeedStrategy  overspeed handling strategy, 0-strategy off; 1-standard; 2-report error and stop on overspeed; 3-adaptive deceleration, default 0
    @param  [in] Default parameter speedPercent  allowed deceleration threshold percentage [0-100], default 10%
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveL(self, desc_pos, tool, user, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
              blendR=-1.0, blendMode = 0,exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0, offset_flag=0,
              offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],oacc = 100.0,config=-1,velAccParamMode=0,overSpeedStrategy=0,speedPercent=10):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        blendMode = int(blendMode)
        exaxis_pos = list(map(float, exaxis_pos))
        search = int(search)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        oacc = float(oacc)
        config = int(config)
        velAccParamMode = int(velAccParamMode)
        overSpeedStrategy = int(overSpeedStrategy)
        speedPercent = int(speedPercent)
        if (overSpeedStrategy > 0):
            error = self.robot.JointOverSpeedProtectStart(overSpeedStrategy, speedPercent)
            if error!=0:
                return error
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # If no parameter is entered, call inverse kinematics to solve
            ret = self.robot.GetInverseKin(0, desc_pos, config)  # Inverse kinematics solution
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error1 = ret[0]
                return error1

        flag = True
        while flag:
            try:
                error1 = self.robot.MoveL([joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5], desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5], tool, user, vel, acc, ovl, blendR, blendMode, exaxis_pos[0],exaxis_pos[1],exaxis_pos[2],exaxis_pos[3], search,offset_flag, offset_pos[0],offset_pos[1],offset_pos[2],offset_pos[3],offset_pos[4],offset_pos[5],oacc,velAccParamMode])
                flag = False
            except socket.error as e:
                flag = True

        if (overSpeedStrategy > 0):
            error = self.robot.JointOverSpeedProtectEnd()
            if error!=0:
                return error

        return error1

    """   
    @brief  Cartesian space arc motion (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter desc_pos_p: waypoint Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool_p: waypoint tool number, [0~14]
    @param  [in] Required parameter user_p: waypoint work object number, [0~14]
    @param  [in] Required parameter desc_pos_t: target point Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool_t: tool number, [0~14]
    @param  [in] Required parameter user_t: work object number, [0~14]
    @param  [in] Default parameter joint_pos_p: waypoint joint position, unit [deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls inverse kinematics to solve for the return value
    @param  [in] Default parameter joint_pos_t: target point joint position, unit [deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls inverse kinematics to solve for the return value
    @param  [in] Default parameter vel_p: waypoint velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc_p: waypoint acceleration percentage, [0~100] not open yet, default 0.0
    @param  [in] Default parameter exaxis_pos_p: waypoint external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter offset_flag_p: whether the waypoint is offset [0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos_p: waypoint pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter vel_t: target point velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc_t: target point acceleration percentage, [0~100] not open yet default 0.0
    @param  [in] Default parameter exaxis_pos_t: target point external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter offset_flag_t: whether the target point is offset [0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos_t: target point pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter blendR:[-1.0]-move to position (blocking), [0~1000]-blend radius (non-blocking), unit [mm] default -1.0
    @param  [in] Default parameter oacc acceleration scaling factor [0-100]/physical acceleration (mm/s2) default 100
    @param  [in] Default parameter config inverse solution joint space configuration, [-1]-solve with reference to the current joint position, [0~7]-solve based on a specific joint space configuration, default -1
    @param  [in] Default parameter velAccParamMode velocity/acceleration parameter mode; 0-percentage; 1-physical velocity (mm/s) acceleration (mm/s2) default 0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveC(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              vel_p=20.0, acc_p=100.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], offset_flag_p=0,
              offset_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              vel_t=20.0, acc_t=100.0, exaxis_pos_t=[0.0, 0.0, 0.0, 0.0], offset_flag_t=0,
              offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ovl=100.0, blendR=-1.0,oacc=100.0,config=-1,velAccParamMode=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos_p = list(map(float, desc_pos_p))
        tool_p = int(tool_p)
        user_p = int(user_p)
        joint_pos_p = list(map(float, joint_pos_p))
        vel_p = float(vel_p)
        acc_p = float(acc_p)
        exaxis_pos_p = list(map(float, exaxis_pos_p))
        offset_flag_p = int(offset_flag_p)
        offset_pos_p = list(map(float, offset_pos_p))

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = int(tool_t)
        user_t = int(user_t)
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))
        offset_flag_t = int(offset_flag_t)
        offset_pos_t = list(map(float, offset_pos_t))

        ovl = float(ovl)
        blendR = float(blendR)
        oacc = float(oacc)
        config = int(config)
        velAccParamMode = int(velAccParamMode)

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # If no parameter is entered, call inverse kinematics to solve
            retp = self.robot.GetInverseKin(0, desc_pos_p, config)  # Inverse kinematics solution
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # If no parameter is input, call inverse kinematics solution
            rett = self.robot.GetInverseKin(0, desc_pos_t, config)  # Inverse kinematics solution
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveC([joint_pos_p[0],joint_pos_p[1],joint_pos_p[2],joint_pos_p[3],joint_pos_p[4],joint_pos_p[5], desc_pos_p[0],desc_pos_p[1],desc_pos_p[2],desc_pos_p[3],desc_pos_p[4],desc_pos_p[5], tool_p, user_p, vel_p, acc_p, exaxis_pos_p[0],exaxis_pos_p[1],exaxis_pos_p[2],exaxis_pos_p[3], offset_flag_p,
                                         offset_pos_p[0],offset_pos_p[1],offset_pos_p[2],offset_pos_p[3],offset_pos_p[4],offset_pos_p[5], joint_pos_t[0],joint_pos_t[1],joint_pos_t[2],joint_pos_t[3],joint_pos_t[4],joint_pos_t[5], desc_pos_t[0],desc_pos_t[1],desc_pos_t[2],desc_pos_t[3],desc_pos_t[4],desc_pos_t[5], tool_t, user_t, vel_t, acc_t, exaxis_pos_t[0],exaxis_pos_t[1],exaxis_pos_t[2],exaxis_pos_t[3],
                                         offset_flag_t, offset_pos_t[0],offset_pos_t[1],offset_pos_t[2],offset_pos_t[3],offset_pos_t[4],offset_pos_t[5], ovl, blendR,oacc,velAccParamMode])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Cartesian space full circle motion (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter desc_pos_p: waypoint Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool_p: waypoint tool number, [0~14]
    @param  [in] Required parameter user_p: waypoint work object number, [0~14]
    @param  [in] Required parameter desc_pos_t: target point Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool_t: tool number, [0~14]
    @param  [in] Required parameter user_t: work object number, [0~14]
    @param  [in] Default parameter joint_pos_p: waypoint joint position, unit [deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls inverse kinematics to solve for the return value
    @param  [in] Default parameter joint_pos_t: target point joint position, unit [deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls inverse kinematics to solve for the return value
    @param  [in] Default parameter vel_p: waypoint velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc_p: path point acceleration percentage, [0~100] not yet open, default 0.0
    @param  [in] Default parameter exaxis_pos_p: waypoint external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter vel_t: target point velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc_t: target point acceleration percentage, [0~100] not open yet default 0.0
    @param  [in] Default parameter exaxis_pos_t: target point external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter offset_flag: whether to offset [0]-no offset, [1]-offset in workpiece/base coordinate frame, [2]-offset in tool coordinate frame, default 0
    @param  [in] Default parameter offset_pos: pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter oacc: acceleration scaling factor [0-100]/physical acceleration (mm/s2), default: 100
    @param  [in] Default parameter blendR: -1: blocking; 0~1000: smoothing radius, default: -1
    @param  [in] Default parameter config inverse solution joint space configuration, [-1]-solve with reference to the current joint position, [0~7]-solve based on a specific joint space configuration, default -1
    @param  [in] Default parameter velAccParamMode velocity/acceleration parameter mode; 0-percentage; 1-physical velocity (mm/s) acceleration (mm/s2) default 0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def Circle(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               vel_p=20.0, acc_p=0.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=0.0,
               exaxis_pos_t=[0.0, 0.0, 0.0, 0.0],
               ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], oacc=100.0, blendR=-1,config=-1,velAccParamMode=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos_p = list(map(float, desc_pos_p))
        tool_p = int(tool_p)
        user_p = int(user_p)
        joint_pos_p = list(map(float, joint_pos_p))
        vel_p = float(vel_p)
        acc_p = float(acc_p)
        exaxis_pos_p = list(map(float, exaxis_pos_p))

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = int(tool_t)
        user_t = int(user_t)
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))

        ovl = float(ovl)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        oacc = float(oacc)
        blendR = float(blendR)
        config = int(config)
        velAccParamMode = int(velAccParamMode)

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # If no parameter is entered, call inverse kinematics to solve
            retp = self.robot.GetInverseKin(0, desc_pos_p, config)  # Inverse kinematics solution
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # If no parameter is input, call inverse kinematics solution
            rett = self.robot.GetInverseKin(0, desc_pos_t, config)  # Inverse kinematics solution
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error

        flag = True
        while flag:
            try:
                error = self.robot.Circle([joint_pos_p[0],joint_pos_p[1],joint_pos_p[2],joint_pos_p[3],joint_pos_p[4],joint_pos_p[5], desc_pos_p[0],desc_pos_p[1],desc_pos_p[2],desc_pos_p[3],desc_pos_p[4],desc_pos_p[5], tool_p, user_p, vel_p, acc_p, exaxis_pos_p[0],exaxis_pos_p[1],exaxis_pos_p[2],exaxis_pos_p[3],
                                           joint_pos_t[0],joint_pos_t[1],joint_pos_t[2],joint_pos_t[3],joint_pos_t[4],joint_pos_t[5], desc_pos_t[0],desc_pos_t[1],desc_pos_t[2],desc_pos_t[3],desc_pos_t[4],desc_pos_t[5],
                                          tool_t, user_t, vel_t, acc_t, exaxis_pos_t[0],exaxis_pos_t[1],exaxis_pos_t[2],exaxis_pos_t[3], ovl, offset_flag, offset_pos[0],offset_pos[1],offset_pos[2],offset_pos[3],offset_pos[4],offset_pos[5], oacc, blendR,velAccParamMode])
                flag = False
            except socket.error as e:
                flag = True
        return error

    # @log_call
    # @xmlrpc_timeout
    # def Circle(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #            joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #            vel_p=20.0, acc_p=0.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=0.0,
    #            exaxis_pos_t=[0.0, 0.0, 0.0, 0.0],
    #            ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
    #     while self.reconnect_flag:
    #         time.sleep(0.1)
    #     if self.GetSafetyCode() != 0:
    #         return self.GetSafetyCode()
    #     desc_pos_p = list(map(float, desc_pos_p))
    #     tool_p = float(int(tool_p))
    #     user_p = float(int(user_p))
    #     joint_pos_p = list(map(float, joint_pos_p))
    #     vel_p = float(vel_p)
    #     acc_p = float(acc_p)
    #     exaxis_pos_p = list(map(float, exaxis_pos_p))
    #
    #     desc_pos_t = list(map(float, desc_pos_t))
    #     tool_t = float(int(tool_t))
    #     user_t = float(int(user_t))
    #     joint_pos_t = list(map(float, joint_pos_t))
    #     vel_t = float(vel_t)
    #     acc_t = float(acc_t)
    #     exaxis_pos_t = list(map(float, exaxis_pos_t))
    #
    #     ovl = float(ovl)
    #     offset_flag = int(offset_flag)
    #     offset_pos = list(map(float, offset_pos))
    #
    #     if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
    #             and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # If no parameter is input, call inverse kinematics solution
    #         retp = self.robot.GetInverseKin(0, desc_pos_p, -1)  # Inverse kinematics solution
    #         if retp[0] == 0:
    #             joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
    #         else:
    #             error = retp[0]
    #             return error
    #
    #     if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
    #             and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # If no parameter is input, call inverse kinematics solution
    #         rett = self.robot.GetInverseKin(0, desc_pos_t, -1)  # Inverse kinematics solution
    #         if rett[0] == 0:
    #             joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
    #         else:
    #             error = rett[0]
    #             return error
    #
    #     flag = True
    #     while flag:
    #         try:
    #             error = self.robot.Circle(joint_pos_p, desc_pos_p, [tool_p, user_p, vel_p, acc_p], exaxis_pos_p,
    #                                       joint_pos_t,
    #                                       desc_pos_t,
    #                                       [tool_t, user_t, vel_t, acc_t], exaxis_pos_t, ovl, offset_flag, offset_pos)
    #             flag = False
    #         except socket.error as e:
    #             flag = True
    #     return error

    """   
    @brief  Cartesian space helical motion (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter desc_pos: target Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Required parameter param:[circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction, velAccMode]circle_num: number of helix turns, circle_angle: helix inclination angle,
    rad_init: helix initial radius, rad_add: radius increment, rotaxis_add: rotation axis direction increment, rot_direction: rotation direction, 0-clockwise, 1-counterclockwise, velAccMode velocity/acceleration parameter mode: 0-constant angular velocity, 1-constant linear velocity
    @param  [in] Default parameter joint_pos: target joint position, unit [deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls inverse kinematics to solve for the return value
    @param  [in] Default parameter vel: velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc: acceleration percentage, [0~100] default 100.0
    @param  [in] Default parameter exaxis_pos: external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter offset_flag:[0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos: pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter config inverse solution joint space configuration, [-1]-solve with reference to the current joint position, [0~7]-solve based on a specific joint space configuration, default -1
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def NewSpiral(self, desc_pos, tool, user, param, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0,
                  exaxis_pos=[0.0, 0.0, 0.0, 0.0],
                  ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        param[0] = int(param[0])
        param[1] = float(param[1])
        param[2] = float(param[2])
        param[3] = float(param[3])
        param[4] = float(param[4])
        param[5] = int(param[5])
        param[6] = int(param[6])
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        exaxis_pos = list(map(float, exaxis_pos))
        ovl = float(ovl)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        config = int(config)

        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # If no parameter is entered, call inverse kinematics to solve
            ret = self.robot.GetInverseKin(0, desc_pos, config)  # Inverse kinematics solution
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.NewSpiral([joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5],
                                              desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5],
                                              tool, user, vel, acc, exaxis_pos[0],exaxis_pos[1],exaxis_pos[2],exaxis_pos[3],
                                              ovl, offset_flag,
                                             offset_pos[0],offset_pos[1],offset_pos[2],offset_pos[3],offset_pos[4],offset_pos[5],
                                              float(param[0]),param[1],param[2],param[3],param[4],int(param[5]),param[6]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Servo motion start, used with ServoJ and ServoCart commands
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ServoMoveStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ServoMoveStart()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Servo motion end, used with ServoJ and ServoCart commands
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ServoMoveEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ServoMoveEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Joint space servo mode motion
    @param  [in] Required parameter joint_pos: target joint position, unit [deg]
    @param  [in] Required parameter axisPos  external axis position, unit mm
    @param  [in] Default parameter acc: acceleration, range [0~100], not yet open, default 0.0
    @param  [in] Default parameter vel: velocity, range [0~100], not yet open, default 0.0
    @param  [in] Default parameter cmdT: command issuing period, unit s, recommended range [0.001~0.0016], default 0.008
    @param  [in] Default parameter filterT: filter time, unit [s], not yet open, default 0.0
    @param  [in] Default parameter gain: proportional amplifier of the target position, not yet open, default 0.0
    @param  [in] Default parameter id: servoJ command ID, default 0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ServoJ(self, joint_pos,axisPos, acc=0.0, vel=0.0, cmdT=0.008, filterT=0.0, gain=0.0, id=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        joint_pos = list(map(float, joint_pos))
        axisPos = list(map(float, axisPos))
        acc = float(acc)
        vel = float(vel)
        cmdT = float(cmdT)
        filterT = float(filterT)
        gain = float(gain)
        id = int(id)
        flag = True
        while flag:
            try:
                error = self.robot.ServoJ(joint_pos,axisPos,acc, vel, cmdT, filterT, gain, id)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Cartesian space servo mode motion
    @param  [in] Required parameter mode:[0]-absolute motion (base coordinate frame), [1]-incremental motion (base coordinate frame), [2]-incremental motion (tool coordinate frame)
    @param  [in] Required parameter desc_pos: target Cartesian position / target Cartesian position increment
    @param  [in] Default parameter pos_gain: pose increment proportional coefficient, only effective in incremental motion, range [0~1], default [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    @param  [in] Default parameter acc: acceleration, range [0~100], not yet open, default 0.0
    @param  [in] Default parameter vel: velocity, range [0~100], not yet open, default 0.0
    @param  [in] Default parameter cmdT: command issuing period, unit s, recommended range [0.001~0.0016], default 0.008
    @param  [in] Default parameter filterT: filter time, unit [s], not yet open, default 0.0
    @param  [in] Default parameter gain: proportional amplifier of the target position, not yet open, default 0.0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ServoCart(self, mode, desc_pos, pos_gain=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], acc=0.0, vel=0.0, cmdT=0.008,
                  filterT=0.0, gain=0.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        mode = int(mode)
        desc_pos = list(map(float, desc_pos))
        pos_gain = list(map(float, pos_gain))
        acc = float(acc)
        vel = float(vel)
        cmdT = float(cmdT)
        filterT = float(filterT)
        gain = float(gain)
        flag = True
        while flag:
            try:
                error = self.robot.ServoCart(mode, desc_pos, pos_gain, acc, vel, cmdT, filterT, gain)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  Joint torque control start
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ServoJTStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ServoJTStart()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Joint torque control
    @param  [in] Required parameter torque j1~j6 joint torque, unit Nm
    @param  [in] Required parameter interval command period, unit s, range [0.001~0.008]
    @param  [in] Default parameter checkFlag detection strategy 0-no limit; 1-limit power; 2-limit velocity; 3-limit both power and velocity, default 0
    @param  [in] Default parameter jPowerLimit joint maximum power limit (W), default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter jVelLimit joint maximum velocity (°/s), default [0.0,0.0,0.0,0.0,0.0,0.0]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ServoJT(self, torque, interval, checkFlag=0, jPowerLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                jVelLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        torque = list(map(float, torque))
        interval = float(interval)
        checkFlag = int(checkFlag)
        jPowerLimit = list(map(float, jPowerLimit))
        jVelLimit = list(map(float, jVelLimit))
        flag = True
        while flag:
            try:
                error = self.robot.ServoJT(torque, interval, checkFlag, jPowerLimit, jVelLimit)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Joint torque control end
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ServoJTEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ServoJTEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Cartesian space point-to-point motion
    @param  [in] Required parameter desc_pos: target Cartesian position / target Cartesian position increment
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Default parameter vel: velocity, range [0~100], default 20.0
    @param  [in] Default parameter acc: acceleration, range [0~100], not yet open, default 0.0
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100, default 100.0
    @param  [in] Default parameter blendT:[-1.0]-motion in place (blocking), [0~500]-smoothing time (non-blocking), unit [ms] default -1.0
    @param  [in] Default parameter config: joint configuration, [-1]-solve with reference to current joint position, [0~7]-solve according to joint configuration, default -1
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveCart(self, desc_pos, tool, user, vel=20.0, acc=0.0, ovl=100.0, blendT=-1.0, config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendT = float(blendT)
        config = int(config)
        flag = True
        while flag:
            try:
                error = self.robot.MoveCart(desc_pos, tool, user, vel, acc, ovl, blendT, config)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Spline motion start
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SplineStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SplineStart()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Spline motion PTP (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter joint_pos: target joint position, unit [deg]
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Default parameter desc_pos: target Cartesian pose, unit [mm][deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls forward kinematics to solve for the return value
    @param  [in] Default parameter vel: velocity, range [0~100], default 20.0
    @param  [in] Default parameter acc: acceleration, range [0~100], default 100.0
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100, default 100.0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SplinePTP(self, joint_pos, tool, user, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=100.0, ovl=100.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        joint_pos = list(map(float, joint_pos))
        tool = int(tool)
        user = int(user)
        desc_pos = list(map(float, desc_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        if ((desc_pos[0] == 0.0) and (desc_pos[1] == 0.0) and (desc_pos[2] == 0.0) and (desc_pos[3] == 0.0)
                and (desc_pos[4] == 0.0) and (desc_pos[5] == 0.0)):  # If no parameter is input, call forward kinematics solution
            ret = self.robot.GetForwardKin(joint_pos)  # Forward kinematics solution
            if ret[0] == 0:
                desc_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.SplinePTP(joint_pos, desc_pos, tool, user, vel, acc, ovl)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Spline motion end
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SplineEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SplineEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  New spline motion start
    @param  [in] Required parameter type:0-arc transition, 1-given point path point
    @param  [in] Default parameter averageTime: global average connection time (ms), default 2000
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def NewSplineStart(self, type, averageTime=2000):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        averageTime = int(averageTime)
        flag = True
        while flag:
            try:
                error = self.robot.NewSplineStart(type, averageTime)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  New spline command point (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter desc_pos: target Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Required parameter lastFlag: whether it is the last point, 0-no, 1-yes
    @param  [in] Default parameter joint_pos: target joint position, unit [deg] default initial value [0.0,0.0,0.0,0.0,0.0,0.0], the default value calls inverse kinematics to solve for the return value
    @param  [in] Default parameter vel: velocity, range [0~100], not yet open, default 0.0
    @param  [in] Default parameter acc: acceleration, range [0~100], not yet open, default 0.0
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter blendR: [0~1000]-smoothing radius, unit [mm] default 0.0
    @param  [in] Default parameter config: inverse solution joint space configuration, [-1]-solve with reference to current joint position, [0~7]-solve according to specific joint space configuration, default -1
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def NewSplinePoint(self, desc_pos, tool, user, lastFlag, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=0.0,
                       acc=0.0, ovl=100.0, blendR=0.0, config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        lastFlag = int(lastFlag)
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        config = int(config)
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # If no parameter is entered, call inverse kinematics to solve
            ret = self.robot.GetInverseKin(0, desc_pos, config)  # Inverse kinematics solution
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.NewSplinePoint(joint_pos, desc_pos, tool, user, vel, acc, ovl, blendR, lastFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  New spline motion end
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def NewSplineEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.NewSplineEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Terminate motion
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def StopMotion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.StopMotion()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Pause motion
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def PauseMotion(self):
        # error = self.robot.PauseMotion()
        self.send_message("/f/bIII0III103III5IIIPAUSEIII/b/f")
        return 0

        # return error

    """   
    @brief  Resume motion
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ResumeMotion(self):
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        # error = self.robot.ResumeMotion()
        error = self.send_message("/f/bIII0III104III6IIIRESUMEIII/b/f")
        return error

    """   
    @brief  Point overall offset start
    @param  [in] Required parameter flag:0-offset in base or workpiece coordinate frame, 2-offset in tool coordinate frame
    @param  [in] Required parameter offset_pos: offset amount, unit [mm][°].
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def PointsOffsetEnable(self, flag, offset_pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        offset_pos = list(map(float, offset_pos))
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.PointsOffsetEnable(flag, offset_pos)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  Point overall offset end
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def PointsOffsetDisable(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.PointsOffsetDisable()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************Robot IO********************************************************************************************
    """

    """   
    @brief  Set control box digital output
    @param  [in] Required parameter id:io number, range [0~15]
    @param  [in] Required parameter status:0-off, 1-on
    @param  [in] Default parameter smooth:0-no smoothing, 1-smoothing, default 0
    @param  [in] Default parameter block:0-blocking, 1-non-blocking, default 0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetDO(self, id, status, smooth=0, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        status = int(status)
        smooth = int(smooth)
        block = int(block)
        flag = True
        while flag:
            try:
                error = self.robot.SetDO(id, status, smooth, block)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set tool digital output
    @param  [in] Required parameter id:io number, range [0~1]
    @param  [in] Required parameter status:0-off, 1-on
    @param  [in] Default parameter smooth:0-no smoothing, 1-smoothing, default 0
    @param  [in] Default parameter block:0-blocking, 1-non-blocking, default 0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetToolDO(self, id, status, smooth=0, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        status = int(status)
        smooth = int(smooth)
        block = int(block)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolDO(id, status, smooth, block)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set control box analog output
    @param  [in] Required parameter id:io number, range [0~1]
    @param  [in] Required parameter value: current or voltage value percentage, range [0~100%] corresponding to current value [0~20mA] or voltage [0~10V];
    @param  [in] Default parameter block:0-blocking, 1-non-blocking, default 0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAO(self, id, value, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        value = float(value)
        block = int(block)
        flag = True
        while flag:
            try:
                error = self.robot.SetAO(id, value * 40.95, block)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set tool analog output
    @param  [in] Required parameter id:io number, range [0]
    @param  [in] Required parameter value: current or voltage value percentage, range [0~100%] corresponding to current value [0~20mA] or voltage [0~10V];
    @param  [in] Default parameter block:0-blocking, 1-non-blocking, default 0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetToolAO(self, id, value, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        value = float(value)
        block = int(block)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolAO(id, value * 40.95, block)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get control box digital input
    @param  [in] Required parameter id:io number, range [0-15]
    @param  [in] Default parameter block:0-blocking, 1-non-blocking, default 0
    @return Error code Success-0  Failure-error code
    @return Return value (returned on successful call) di: 0-low level, 1-high level
    """

    @log_call
    # @xmlrpc_timeout
    def GetDI(self, id, block=0):
        id = int(id)
        block = int(block)
        # _error = self.robot.GetDI(id, block)
        # error = _error[0]
        # print(_error)
        # if _error[0] == 0:
        #     di = _error[1]
        #     return error, di
        # else:
        #     return error
        if 0 <= id < 8:
            level = (self.robot_state_pkg.cl_dgt_input_l & (0x01 << id)) >> id
            return 0, level
        elif 8 <= id < 16:
            id -= 8
            level = (self.robot_state_pkg.cl_dgt_input_h & (0x01 << id)) >> id
            return 0, level
        else:
            return -1,None


    """   
    @brief  Get tool digital input
    @param  [in] Required parameter id:io number, range [0~1]
    @param  [in] Default parameter block:0-blocking, 1-non-blocking, default 0
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) di: 0-low level, 1-high level
    """

    @log_call
    # @xmlrpc_timeout
    def GetToolDI(self, id, block=0):
        id = int(id)
        block = int(block)
        # _error = self.robot.GetToolDI(id, block)
        # error = _error[0]
        # if _error[0] == 0:
        #     di = _error[1]
        #     return error, di
        # else:
        #     return error
        if 0 <= id < 2:
            id+=1
            level = (self.robot_state_pkg.tl_dgt_input_l & (0x01 << id)) >> id
            return 0,level
        else:
            return -1,None

    """   
    @brief  Wait for control box digital input
    @param  [in] Required parameter id:io number, range [0~15]
    @param  [in] Required parameter status:0-off, 1-on
    @param  [in] Required parameter maxtime: maximum waiting time, unit [ms]
    @param  [in] Required parameter opt: timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout prompt and program continues execution, 2-wait indefinitely
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitDI(self, id, status, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitDI(id, status, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Wait for control box multiple digital inputs
    @param  [in] Required parameter mode 0-multiple AND, 1-multiple OR
    @param  [in] Required parameter id  io number, bit0~bit7 corresponds to DI0~DI7, bit8~bit15 corresponds to CI0~CI7
    @param  [in] Required parameter status:0-off, 1-on
    @param  [in] Required parameter maxtime: maximum waiting time, unit [ms]
    @param  [in] Required parameter opt: timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout prompt and program continues execution, 2-wait indefinitely
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitMultiDI(self, mode, id, status, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        id = int(id)
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitMultiDI(mode, id, status, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Wait for tool digital input
    @param  [in] Required parameter id:io number, range [0~1]
    @param  [in] Required parameter status:0-off, 1-on
    @param  [in] Required parameter maxtime: maximum waiting time, unit [ms]
    @param  [in] Required parameter opt: timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout prompt and program continues execution, 2-wait indefinitely
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitToolDI(self, id, status, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        id = id+1 #Controller internal 1 corresponds to di0, 2 corresponds to di1
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitToolDI(id, status, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get control box analog input
    @param  [in] Required parameter id:io number, range [0~1]
    @param  [in] Default parameter block:0-blocking, 1-non-blocking, default 0
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) value: input current or voltage value percentage, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    """

    @log_call
    @xmlrpc_timeout
    def GetAI(self, id, block=0):
        id = int(id)
        block = int(block)
        # _error = self.robot.GetAI(id, block)
        # error = _error[0]
        # if _error[0] == 0:
        #     value = _error[1]
        #     return error, value
        # else:
        #     return error
        if 0 <= id < 2:
            return 0,self.robot_state_pkg.cl_analog_input[id] / 40.95
        else:
            return -1


    """   
    @brief  Get tool analog input
    @param  [in] Required parameter id:io number, range [0]
    @param  [in] Default parameter block:0-blocking, 1-non-blocking, default 0
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) value: input current or voltage value percentage, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    """

    @log_call
    @xmlrpc_timeout
    def GetToolAI(self, id, block=0):
        id = int(id)
        block = int(block)
        # _error = self.robot.GetToolAI(id, block)
        # error = _error[0]
        # if _error[0] == 0:
        #     value = _error[1]
        #     return error, value
        # else:
        #     return error
        return 0, self.robot_state_pkg.tl_anglog_input / 40.95

    """   
    @brief  Get robot end point record button state
    @param  [in] NULL
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) button state, 0-pressed, 1-released
    """

    @log_call
    @xmlrpc_timeout
    def GetAxlePointRecordBtnState(self):
        # while self.reconnect_flag:
        #     time.sleep(0.1)
        # flag = True
        # while flag:
        #     try:
        #         _error = self.robot.GetAxlePointRecordBtnState()
        #         flag = False
        #     except socket.error as e:
        #         flag = True
        #
        # error = _error[0]
        # if _error[0] == 0:
        #     value = _error[1]
        #     return error, value
        # else:
        #     return error,None
        return 0,(self.robot_state_pkg.tl_dgt_input_l & 0x10) >> 4


    """   
    @brief  Get robot end DO output state
    @param  [in] NULL
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) do_state DO output state, do0~do1 corresponds to bit1~bit2, starting from bit0
    """

    @log_call
    @xmlrpc_timeout
    def GetToolDO(self):
        # _error = self.robot.GetToolDO()
        # error = _error[0]
        # if _error[0] == 0:
        #     value = _error[1]
        #     return error, value
        # else:
        #     return error
        return 0,self.robot_state_pkg.tl_dgt_output_l

    """   
    @brief  Get robot controller DO output state
    @param  [in] NULL
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) do_state_h DO output state, co0~co7 corresponds to bit0~bit7 do_state_l DO output state, do0~do7 corresponds to bit0~bit7
    """

    @log_call
    @xmlrpc_timeout
    def GetDO(self):
        # _error = self.robot.GetDO()
        # error = _error[0]
        # if _error[0] == 0:
        #     do_state_h = _error[1]
        #     do_state_l = _error[2]
        #     return error, [do_state_h, do_state_l]
        # else:
        #     return error
        return 0, [self.robot_state_pkg.cl_dgt_output_h,self.robot_state_pkg.cl_dgt_output_l]

    """   
    @brief  Wait for control box analog input
    @param  [in] Required parameter id:io number, range [0~1]
    @param  [in] Required parameter sign:0-greater than, 1-less than
    @param  [in] Required parameter value: input current or voltage value percentage, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    @param  [in] Required parameter maxtime: maximum waiting time, unit [ms]
    @param  [in] Required parameter opt: timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout prompt and program continues execution, 2-wait indefinitely
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitAI(self, id, sign, value, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        sign = int(sign)
        value = float(value)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitAI(id, sign, value*40.95, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Wait for tool analog input
    @param  [in] Required parameter id:io number, range [0]
    @param  [in] Required parameter sign:0-greater than, 1-less than
    @param  [in] Required parameter value: input current or voltage value percentage, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    @param  [in] Required parameter maxtime: maximum waiting time, unit [ms]
    @param  [in] Required parameter opt: timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout prompt and program continues execution, 2-wait indefinitely
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitToolAI(self, id, sign, value, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        sign = int(sign)
        value = float(value)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitToolAI(id, sign, value*40.95, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************Robot common settings********************************************************************************************
    """

    """   
    @brief  Set global velocity
    @param  [in] Required parameter vel  velocity percentage, range [0~100]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetSpeed(self, vel):
        while self.reconnect_flag:
            time.sleep(0.1)
        vel = int(vel)
        flag = True
        while flag:
            try:
                error = self.robot.SetSpeed(vel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set system variable
    @param  [in] Required parameter id: variable number, range [1~20]
    @param  [in] Required parameter value: variable value
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetSysVarValue(self, id, value):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        value = float(value)
        flag = True
        while flag:
            try:
                error = self.robot.SetSysVarValue(id, value)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set tool reference point - six-point method
    @param  [in] Required parameter point_num point number, range [1~6] 
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetToolPoint(self, point_num):
        while self.reconnect_flag:
            time.sleep(0.1)
        point_num = int(point_num)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolPoint(point_num)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Calculate tool coordinate frame - six-point method
    @param  [in] NULL
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) tcp_pose [x,y,z,rx,ry,rz] tool coordinate frame
    """

    @log_call
    @xmlrpc_timeout
    def ComputeTool(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeTool()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Set tool reference point - four-point method
    @param  [in] Required parameter point_num point number, range [1~4] 
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTcp4RefPoint(self, point_num):
        while self.reconnect_flag:
            time.sleep(0.1)
        point_num = int(point_num)
        flag = True
        while flag:
            try:
                error = self.robot.SetTcp4RefPoint(point_num)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Calculate tool coordinate frame - four-point method
    @param  [in] NULL
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) tcp_pose [x,y,z,rx,ry,rz]  tool coordinate frame
    """

    @log_call
    @xmlrpc_timeout
    def ComputeTcp4(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeTcp4()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Set tool coordinate frame
    @param  [in] Required parameter id: coordinate frame number, range [1~15]
    @param  [in] Required parameter t_coord:[x,y,z,rx,ry,rz]  tool center point pose relative to end flange center, unit [mm][°]
    @param  [in] Required parameter type:0-tool coordinate frame, 1-sensor coordinate frame
    @param  [in] Required parameter install: installation position, 0-robot end, 1-robot external
    @param  [in] Required parameter toolID: tool ID
    @param  [in] Required parameter loadNum: load number
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetToolCoord(self, id, t_coord, type, install, toolID, loadNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        t_coord = list(map(float, t_coord))
        type = int(type)
        install = int(install)
        toolID = int(toolID)
        loadNum = int(loadNum)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolCoord(id, t_coord, type, install, toolID, loadNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set tool coordinate frame list
    @param  [in] Required parameter id: coordinate frame number, range [1~15]
    @param  [in] Required parameter t_coord:[x,y,z,rx,ry,rz]  tool center point pose relative to end flange center, unit [mm][°]
    @param  [in] Required parameter type:0-tool coordinate frame, 1-sensor coordinate frame
    @param  [in] Required parameter install: installation position, 0-robot end, 1-robot external
    @param  [in] Required parameter loadNum: load number
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetToolList(self, id, t_coord, type, install , loadNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        t_coord = list(map(float, t_coord))
        type = int(type)
        install = int(install)
        loadNum = int(loadNum)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolList(id, t_coord, type, install, loadNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set external tool reference point - three-point method
    @param  [in] Required parameter point_num point number, range [1~3] 
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetExTCPPoint(self, point_num):
        while self.reconnect_flag:
            time.sleep(0.1)
        point_num = int(point_num)
        flag = True
        while flag:
            try:
                error = self.robot.SetExTCPPoint(point_num)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Calculate external tool coordinate frame - three-point method
    @param  [in] NULL
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) tcp_pose [x,y,z,rx,ry,rz] external tool coordinate frame
    """

    @log_call
    @xmlrpc_timeout
    def ComputeExTCF(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeExTCF()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Set external tool coordinate frame
    @param  [in] Required parameter id: coordinate frame number, range [0~14]
    @param  [in] Required parameter etcp: [x,y,z,rx,ry,rz] external tool coordinate frame, unit [mm][°]
    @param  [in] Required parameter etool: [x,y,z,rx,ry,rz] end tool coordinate frame, unit [mm][°]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetExToolCoord(self, id, etcp, etool):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        etcp = list(map(float, etcp))
        etool = list(map(float, etool))
        flag = True
        while flag:
            try:
                error = self.robot.SetExToolCoord(id, etcp, etool)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set external tool coordinate frame list
    @param  [in] Required parameter id: coordinate frame number, range [0~14]
    @param  [in] Required parameter etcp: [x,y,z,rx,ry,rz] external tool coordinate frame, unit [mm][°]
    @param  [in] Required parameter etool: [x,y,z,rx,ry,rz] end tool coordinate frame, unit [mm][°]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetExToolList(self, id, etcp, etool):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        etcp = list(map(float, etcp))
        etool = list(map(float, etool))
        flag = True
        while flag:
            try:
                error = self.robot.SetExToolList(id, etcp, etool)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set workpiece reference point - three-point method
    @param  [in] Required parameter point_num point number, range [1~3] 
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjCoordPoint(self, point_num):
        while self.reconnect_flag:
            time.sleep(0.1)
        point_num = int(point_num)
        flag = True
        while flag:
            try:
                error = self.robot.SetWObjCoordPoint(point_num)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Calculate work object coordinate frame
    @param  [in] method calculation method 0: origin-x axis-z axis  1: origin-x axis-xy plane
    @param  [in] refFrame reference coordinate frame
    @return Error code Success-0,  Failure-error code
    @return Return value (returned on successful call) wobj_pose [x,y,z,rx,ry,rz] work object coordinate frame
    """

    @log_call
    @xmlrpc_timeout
    def ComputeWObjCoord(self, method, refFrame):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        refFrame = int(refFrame)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeWObjCoord(method, refFrame)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Set work object coordinate frame
    @param  [in] Required parameter id: coordinate frame number, range [0~14]
    @param  [in] Required parameter coord: work object coordinate frame pose relative to end flange center, unit [mm][°]
    @param  [in] Required parameter refFrame: reference coordinate frame
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjCoord(self, id, coord, refFrame):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        coord = list(map(float, coord))
        refFrame = int(refFrame)
        flag = True
        while flag:
            try:
                error = self.robot.SetWObjCoord(id, coord, refFrame)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set work object coordinate frame list
    @param  [in] Required parameter id: coordinate frame number, range [0~14]
    @param  [in] Required parameter coord: work object coordinate frame pose relative to end flange center, unit [mm][°]
    @param  [in] Required parameter refFrame: reference coordinate frame
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjList(self, id, coord, refFrame):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        coord = list(map(float, coord))
        refFrame = int(refFrame)
        flag = True
        while flag:
            try:
                error = self.robot.SetWObjList(id, coord, refFrame)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set end load weight
    @param  [in] Required parameter loadNum load number
    @param  [in] Required parameter weight: unit [kg]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetLoadWeight(self, loadNum, weight):
        while self.reconnect_flag:
            time.sleep(0.1)
        loadNum = int(loadNum)
        weight = float(weight)
        flag = True
        while flag:
            try:
                error = self.robot.SetLoadWeight(loadNum,weight)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set robot installation method - fixed installation
    @param  [in] Required parameter method:0-upright mounting, 1-side mounting, 2-hanging mounting
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotInstallPos(self, method):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotInstallPos(method)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set robot installation angle
    @param  [in] Required parameter yangle: tilt angle
    @param  [in] Required parameter zangle: rotation angle
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotInstallAngle(self, yangle, zangle):
        while self.reconnect_flag:
            time.sleep(0.1)
        yangle = float(yangle)
        zangle = float(zangle)
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotInstallAngle(yangle, zangle)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set end-effector payload center of mass coordinates
    @param  [in] Required parameter x: center of mass coordinate, unit [mm]
    @param  [in] Required parameter y: center of mass coordinate, unit [mm]
    @param  [in] Required parameter z: center of mass coordinate, unit [mm]
    @param  [in] Default parameter loadNum payload number, default 0
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetLoadCoord(self, x, y, z, loadNum = 0):
        while self.reconnect_flag:
            time.sleep(0.1)
        x = float(x)
        y = float(y)
        z = float(z)
        loadNum = int(loadNum)
        flag = True
        while flag:
            try:
                error = self.robot.SetLoadCoord(x, y, z, loadNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Wait for the specified time
    @param  [in] Required parameter t_ms: unit [ms]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitMs(self, t_ms):
        while self.reconnect_flag:
            time.sleep(0.1)
        t_ms = int(t_ms)
        flag = True
        while flag:
            try:
                error = self.robot.WaitMs(t_ms)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************Robot safety settings********************************************************************************************
    """

    """   
    @brief  Set collision level
    @param  [in] Required parameter mode:0-level, 1-percentage
    @param  [in] Required parameter level=[j1,j2,j3,j4,j5,j6]: collision threshold; when mode=0, range: 1-10; correspondingly when mode=1, range 0-100%
    @param  [in] Required parameter config:0-do not update config file, 1-update config file
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAnticollision(self, mode, level, config):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        level = list(map(float, level))
        config = int(config)
        flag = True
        while flag:
            try:
                error = self.robot.SetAnticollision(mode, level, config)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set post-collision strategy
    @param  [in] Required parameter strategy: 0-report error and pause, 1-continue running, 2-report error and stop, 3-gravity torque mode, 4-oscillation response mode, 5-collision rebound mode
    @param  [in] Default parameter safeTime: safe stop time [1000-2000]ms, default: 1000
    @param  [in] Default parameter safeDistance: safe stop distance [1-150]mm, default: 100
    @param  [in] Default parameter safeVel: safe stop velocity [50-250]mm/s, default: 250
    @param  [in] Default parameter safetyMargin[6]: safety factor [1-10], default: [10,10,10,10,10,10]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetCollisionStrategy(self, strategy,safeTime=1000,safeDistance=100,safeVel=250,safetyMargin=[10,10,10,10,10,10]):
        while self.reconnect_flag:
            time.sleep(0.1)
        strategy = int(strategy)
        safeTime = int(safeTime)
        safeDistance = int(safeDistance)
        safeVel = int(safeVel)
        safetyMargin = list(map(int, safetyMargin))
        flag = True
        while flag:
            try:
                error = self.robot.SetCollisionStrategy(strategy,safeTime,safeDistance,safeVel,safetyMargin)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set positive limit
    @param  [in] Required parameter p_limit=[j1,j2,j3,j4,j5,j6]: six joint positions
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetLimitPositive(self, p_limit):
        while self.reconnect_flag:
            time.sleep(0.1)
        p_limit = list(map(float, p_limit))
        flag = True
        while flag:
            try:
                error = self.robot.SetLimitPositive(p_limit)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set negative limit
    @param  [in] Required parameter n_limit=[j1,j2,j3,j4,j5,j6]: six joint positions
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetLimitNegative(self, n_limit):
        while self.reconnect_flag:
            time.sleep(0.1)
        n_limit = list(map(float, n_limit))
        flag = True
        while flag:
            try:
                error = self.robot.SetLimitNegative(n_limit)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Clear error state; can only clear resettable errors
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ResetAllError(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ResetAllError()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Joint friction compensation switch
    @param  [in] Required parameter state: 0-off, 1-on
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FrictionCompensationOnOff(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)
        flag = True
        while flag:
            try:
                error = self.robot.FrictionCompensationOnOff(state)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set joint friction compensation coefficients - fixed mounting - upright
    @param  [in] Required parameter coeff=[j1,j2,j3,j4,j5,j6]: six joint compensation coefficients
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_level(self, coeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        coeff = list(map(float, coeff))
        flag = True
        while flag:
            try:
                error = self.robot.SetFrictionValue_level(coeff)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set joint friction compensation coefficients - fixed mounting - side mount
    @param  [in] Required parameter coeff=[j1,j2,j3,j4,j5,j6]: six joint compensation coefficients
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_wall(self, coeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        coeff = list(map(float, coeff))
        flag = True
        while flag:
            try:
                error = self.robot.SetFrictionValue_wall(coeff)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set joint friction compensation coefficients - fixed mounting - inverted
    @param  [in] Required parameter coeff=[j1,j2,j3,j4,j5,j6]: six joint compensation coefficients
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_ceiling(self, coeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        coeff = list(map(float, coeff))
        flag = True
        while flag:
            try:
                error = self.robot.SetFrictionValue_ceiling(coeff)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set joint friction compensation coefficients - free mounting
    @param  [in] Required parameter coeff=[j1,j2,j3,j4,j5,j6]: six joint compensation coefficients
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_freedom(self, coeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        coeff = list(map(float, coeff))
        flag = True
        while flag:
            try:
                error = self.robot.SetFrictionValue_freedom(coeff)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************Robot status query********************************************************************************************
    """

    """   
    @brief  Get robot mounting angle
    @param  [in] NULL
    @return error code success- 0,  failure-error code
    @return return value (returned on success) [yangle,zangle] yangle-tilt angle, zangle-rotation angle
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotInstallAngle(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotInstallAngle()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error,None

    """   
    @brief  Get system variable value
    @param  [in] id: system variable number, range [1~20]
    @return error code success- 0,  failure-error code
    @return return value (returned on success) var_value: system variable value
    """

    @log_call
    @xmlrpc_timeout
    def GetSysVarValue(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSysVarValue(id)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Get current joint position (angle)
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0,  failure-error code
    @return return value (returned on success) joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointPosDegree(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualJointPosDegree(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.jt_cur_pos[0],self.robot_state_pkg.jt_cur_pos[1],self.robot_state_pkg.jt_cur_pos[2],
                  self.robot_state_pkg.jt_cur_pos[3],self.robot_state_pkg.jt_cur_pos[4],self.robot_state_pkg.jt_cur_pos[5]]
    """   
    @brief  Get current joint position (radians)
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0,  failure-error code
    @return return value (returned on success) joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointPosRadian(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetActualJointPosRadian(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Get joint feedback velocity - deg/s
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0,  failure-error code
    @return return value (returned on success) speed=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointSpeedsDegree(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualJointSpeedsDegree(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.actual_qd[0],self.robot_state_pkg.actual_qd[1],self.robot_state_pkg.actual_qd[2],
                  self.robot_state_pkg.actual_qd[3],self.robot_state_pkg.actual_qd[4],self.robot_state_pkg.actual_qd[5]]

    """   
    @brief  Get joint feedback acceleration - deg/s^2
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0,  failure-error code
    @return return value (returned on success) acc=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointAccDegree(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualJointAccDegree(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.actual_qdd[0],self.robot_state_pkg.actual_qdd[1],self.robot_state_pkg.actual_qdd[2],
                  self.robot_state_pkg.actual_qdd[3],self.robot_state_pkg.actual_qdd[4],self.robot_state_pkg.actual_qdd[5]]

    """   
    @brief  Get TCP command resultant velocity
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0,  failure-error code
    @return return value (returned on success) [tcp_speed,ori_speed] tcp_speed linear resultant velocity ori_speed orientation resultant velocity 
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetTCPCompositeSpeed(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetTargetTCPCompositeSpeed(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.target_TCP_CmpSpeed[0],self.robot_state_pkg.target_TCP_CmpSpeed[1]]

    """   
    @brief  Get TCP feedback resultant velocity
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0,  failure-error code
    @return return value (returned on success) [tcp_speed,ori_speed] tcp_speed linear resultant velocity ori_speed orientation resultant velocity 
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPCompositeSpeed(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualTCPCompositeSpeed(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error
        return 0, [self.robot_state_pkg.actual_TCP_CmpSpeed[0], self.robot_state_pkg.actual_TCP_CmpSpeed[1]]

    """   
    @brief  Get TCP command velocity
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0,  failure-error code
    @return return value (returned on success) speed [x,y,z,rx,ry,rz] velocity mm/s
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetTCPSpeed(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetTargetTCPSpeed(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.target_TCP_Speed[0],self.robot_state_pkg.target_TCP_Speed[1],self.robot_state_pkg.target_TCP_Speed[2],
                  self.robot_state_pkg.target_TCP_Speed[3],self.robot_state_pkg.target_TCP_Speed[4],self.robot_state_pkg.target_TCP_Speed[5]]

    """   
    @brief  Get TCP feedback velocity
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0,  failure-error code
    @return return value (returned on success) speed [x,y,z,rx,ry,rz] velocity
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPSpeed(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualTCPSpeed(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.actual_TCP_Speed[0],self.robot_state_pkg.actual_TCP_Speed[1],self.robot_state_pkg.actual_TCP_Speed[2],
                  self.robot_state_pkg.actual_TCP_Speed[3],self.robot_state_pkg.actual_TCP_Speed[4],self.robot_state_pkg.actual_TCP_Speed[5]]

    """   
    @brief  Get current tool pose
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) tcp_pose=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPPose(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualTCPPose(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.tl_cur_pos[0],self.robot_state_pkg.tl_cur_pos[1],self.robot_state_pkg.tl_cur_pos[2],
                  self.robot_state_pkg.tl_cur_pos[3],self.robot_state_pkg.tl_cur_pos[4],self.robot_state_pkg.tl_cur_pos[5]]

    """   
    @brief  Get current tool coordinate frame number
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) tool_id: tool coordinate frame number
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPNum(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualTCPNum(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0,self.robot_state_pkg.tool

    """   
    @brief  Get current work object coordinate frame number 
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) wobj_id: work object coordinate frame number
    """

    @log_call
    @xmlrpc_timeout
    def GetActualWObjNum(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualWObjNum(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0, self.robot_state_pkg.user

    """   
    @brief  Get current end flange pose
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) flange_pose=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualToolFlangePose(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetActualToolFlangePose(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.flange_cur_pos[0],self.robot_state_pkg.flange_cur_pos[1],self.robot_state_pkg.flange_cur_pos[2],
                  self.robot_state_pkg.flange_cur_pos[3],self.robot_state_pkg.flange_cur_pos[4],self.robot_state_pkg.flange_cur_pos[5]]
    """   
    @brief  Inverse kinematics, solve joint position from Cartesian pose
    @param  [in] Required parameter type:0-absolute pose (base frame), 1-relative pose (base frame), 2-relative pose (tool frame)
    @param  [in] Required parameter desc_pose:[x,y,z,rx,ry,rz], tool pose, unit [mm][°]
    @param  [in] Default parameter config: joint configuration, [-1]-solve with reference to current joint position, [0~7]-solve according to joint configuration, default -1
    @return error code success- 0, failure-error code
    @return return value (returned on success) joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKin(self, type, desc_pos, config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        config = int(config)
        flag = True
        while flag:
            try:
                _error = self.robot.GetInverseKin(type, desc_pos, config)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Inverse kinematics, solve joint position from tool pose with reference to a specified joint position
    @param  [in] Required parameter type:0-absolute pose (base frame), 1-relative pose (base frame), 2-relative pose (tool frame)
    @param  [in] Required parameter desc_pose:[x,y,z,rx,ry,rz], tool pose, unit [mm][°]
    @param  [in] Required parameter joint_pos_ref: [j1,j2,j3,j4,j5,j6], joint reference position, unit [°]
    @return error code success- 0,joint_pos=[j1,j2,j3,j4,j5,j6] failure-error code
    @return return value (returned on success) joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKinRef(self, type, desc_pos, joint_pos_ref):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        joint_pos_ref = list(map(float, joint_pos_ref))
        flag = True
        while flag:
            try:
                _error = self.robot.GetInverseKinRef(type, desc_pos, joint_pos_ref)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Inverse kinematics, determine whether a joint position solution exists for a tool pose
    @param  [in] Required parameter type:0-absolute pose (base frame), 1-relative pose (base frame), 2-relative pose (tool frame)
    @param  [in] Required parameter desc_pose:[x,y,z,rx,ry,rz], tool pose, unit [mm][°]
    @param  [in] Required parameter joint_pos_ref: [j1,j2,j3,j4,j5,j6], joint reference position, unit [°]
    @return error code success- 0, failure-error code
    @return return value (returned on success) result: "True"-solution exists, "False"-no solution
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKinHasSolution(self, type, desc_pos, joint_pos_ref):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        joint_pos_ref = list(map(float, joint_pos_ref))
        flag = True
        while flag:
            try:
                _error = self.robot.GetInverseKinHasSolution(type, desc_pos, joint_pos_ref)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Forward kinematics, solve tool pose from joint position
    @param  [in] Required parameter joint_pos:[j1,j2,j3,j4,j5,j6]: joint position, unit [°]
    @return error code success- 0,  failure-error code
    @return return value (returned on success) desc_pos=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetForwardKin(self, joint_pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        joint_pos = list(map(float, joint_pos))
        flag = True
        while flag:
            try:
                _error = self.robot.GetForwardKin(joint_pos)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Get current joint torques
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) torques=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetJointTorques(self, flag=1):
        flag = int(flag)
        # _error = self.robot.GetJointTorques(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.jt_cur_tor[0],self.robot_state_pkg.jt_cur_tor[1],self.robot_state_pkg.jt_cur_tor[2],
                  self.robot_state_pkg.jt_cur_tor[3],self.robot_state_pkg.jt_cur_tor[4],self.robot_state_pkg.jt_cur_tor[5]]

    """   
    @brief  Get current payload mass
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) weight  unit [kg]
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetPayload(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetTargetPayload(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Get current payload center of mass
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) cog=[x,y,z]: center of mass coordinates, unit [mm]
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetPayloadCog(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetTargetPayloadCog(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3]]
        else:
            return error,None

    """   
    @brief  Get current tool coordinate frame
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) tcp_offset=[x,y,z,rx,ry,rz]: relative pose, unit [mm][°]
    """

    @log_call
    @xmlrpc_timeout
    def GetTCPOffset(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetTCPOffset(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Get current work object coordinate frame
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) wobj_offset=[x,y,z,rx,ry,rz]: relative pose, unit [mm][°]
    """

    @log_call
    @xmlrpc_timeout
    def GetWObjOffset(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetWObjOffset(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Get joint soft limit angles
    @param  [in] Default parameter flag: 0-blocking, 1-non-blocking, default 1
    @return error code success- 0, failure-error code
    @return return value (returned on success) [j1min,j1max,j2min,j2max,j3min,j3max,j4min,j4max,j5min,j5max,j6min,j6max]: axis 1~ axis 6 joint negative limit and positive limit, unit [mm]
    """

    @log_call
    @xmlrpc_timeout
    def GetJointSoftLimitDeg(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetJointSoftLimitDeg(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8],
                           _error[9], _error[10], _error[11], _error[12]]
        else:
            return error,None

    """   
    @brief  Get system time
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) t_ms: unit [ms]
    """

    @log_call
    @xmlrpc_timeout
    def GetSystemClock(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSystemClock()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Get robot current joint configuration
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) config: range [0~7]
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotCurJointsConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotCurJointsConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Get default velocity
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) vel: unit [mm/s]
    """

    @log_call
    @xmlrpc_timeout
    def GetDefaultTransVel(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetDefaultTransVel()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Query whether robot motion is complete
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) state:0-not complete, 1-complete
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotMotionDone(self):
        # _error = self.robot.GetRobotMotionDone()
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
            return 0,self.robot_state_pkg.motion_done
    """   
    @brief  Query robot error code
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) [maincode subcode] maincode main error code subcode sub error code
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotErrorCode(self):
        # _error = self.robot.GetRobotErrorCode()
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error
        return 0, [self.robot_state_pkg.main_code,self.robot_state_pkg.sub_code]

    """   
    @brief  Query robot teaching management waypoint data
    @param  [in] Required parameter name  waypoint name
    @return error code success- 0, failure-error code
    @return return value (returned on success) data waypoint data [x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool, wobj,speed,acc,e1,e2,e3,e4]
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotTeachingPoint(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotTeachingPoint(name)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            data =_error[1].split(',')
            if len(data)!= 20:
                self.log_error("get get Teaching Point size fail")
                return -1
            return error, [data[0],data[1], data[2], data[3], data[4], data[5], data[6],data[7],
                           data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15],
                           data[16], data[17], data[18], data[19] ]
        else:
            return error,None

    """   
    @brief  Query robot motion queue buffer length
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) len  buffer length
    """

    @log_call
    @xmlrpc_timeout
    def GetMotionQueueLength(self):
        # _error = self.robot.GetMotionQueueLength()
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0, self.robot_state_pkg.mc_queue_len

    """   
    @brief  Get robot emergency stop status
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) state emergency stop status, 0-not emergency stop, 1-emergency stop
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotEmergencyStopState(self):
        # _error = self.robot.GetRobotEmergencyStopState()
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0, self.robot_state_pkg.EmergencyStop

    """   
    @brief  Get safe stop signal
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) [si0_state,si1_state] si0_state safe stop signal SI0, 0-invalid, 1-valid si1_state safe stop signal SI1, 0-invalid, 1-valid
    """

    @log_call
    @xmlrpc_timeout
    def GetSafetyStopState(self):
        # _error = self.robot.GetSafetyStopState()
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error

        return 0, [self.robot_state_pkg.safety_stop0_state,self.robot_state_pkg.safety_stop1_state]

    """   
    @brief  Get communication status between SDK and robot
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return return value (returned on success) state communication status, 0-communication normal, 1-communication abnormal
    """

    @log_call
    @xmlrpc_timeout
    def GetSDKComState(self):
        # while self.reconnect_flag:
        #     time.sleep(0.1)
        # flag = True
        # while flag:
        #     try:
        #         _error = self.robot.GetSDKComState()
        #         flag = False
        #     except socket.error as e:
        #         flag = True
        #
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error,None
        g_sock_com_err = self.SDK_state
        if g_sock_com_err is True:
            return 0,0
        else:
            return 0,1


    """   
       @brief  Get SSH public key
       @param  [in] NULL
       @return error code success-0, failure-error code
       @return return value (returned on success) keygen public key
       """

    @log_call
    @xmlrpc_timeout
    def GetSSHKeygen(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSSHKeygen()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Send SCP command
    @param  [in] Required parameter mode 0-upload (host->controller), 1-download (controller->host)
    @param  [in] Required parameter sshname host username
    @param  [in] Required parameter sship host IP address
    @param  [in] Required parameter usr_file_url host file path
    @param  [in] Required parameter robot_file_url robot controller file path
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetSSHScpCmd(self, mode, sshname, sship, usr_file_url, robot_file_url):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        sshname = str(sshname)
        sship = str(sship)
        usr_file_url = str(usr_file_url)
        robot_file_url = str(robot_file_url)
        flag = True
        while flag:
            try:
                error = self.robot.SetSSHScpCmd(mode, sshname, sship, usr_file_url, robot_file_url)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Compute the MD5 value of the file at the specified path
    @param  [in] Required parameter file_path file path including file name, default Traj folder path is:"/fruser/traj/", e.g. "/fruser/traj/trajHelix_aima_1.txt"
    @return Error code Success-0  Failure-error code
    @return return value (returned on success) md5 file MD5 value
    """

    @log_call
    @xmlrpc_timeout
    def ComputeFileMD5(self, file_path):
        while self.reconnect_flag:
            time.sleep(0.1)
        file_path = str(file_path)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeFileMD5(file_path)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Get robot version information
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    @return return value (returned on success) robotModel robot model
    @return return value (returned on success) webVersion web version
    @return return value (returned on success) controllerVersion controller version
    """

    @log_call
    @xmlrpc_timeout
    def GetSoftwareVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSoftwareVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3]
        else:
            return error,None,None,None

    """   
    @brief  Get robot hardware version information
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    @return return value (returned on success) ctrlBoxBoardVersion control box version
    @return return value (returned on success) driver1Version 
    @return return value (returned on success) driver2Version 
    @return return value (returned on success) driver3Version
    @return return value (returned on success) driver4Version
    @return return value (returned on success) driver5Version
    @return return value (returned on success) driver6Version
    @return return value (returned on success) endBoardVersion
    """

    @log_call
    @xmlrpc_timeout
    def GetSlaveHardVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlaveHardVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error,None,None,None,None,None,None,None,None

    @log_call
    @xmlrpc_timeout
    def GetHardwareversion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlaveHardVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error, None, None, None, None, None, None, None, None

    """   
    @brief  Get robot firmware version information
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    @return return value (returned on success) ctrlBoxBoardVersion control box version
    @return return value (returned on success) driver1Version 
    @return return value (returned on success) driver2Version 
    @return return value (returned on success) driver3Version
    @return return value (returned on success) driver4Version
    @return return value (returned on success) driver5Version
    @return return value (returned on success) driver6Version
    @return return value (returned on success) endBoardVersion
    """

    @log_call
    @xmlrpc_timeout
    def GetSlaveFirmVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlaveFirmVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error,None,None,None,None,None,None,None,None

    @log_call
    @xmlrpc_timeout
    def GetFirmwareVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlaveFirmVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error, None, None, None, None, None, None, None, None

    """   
    @brief  Get DH compensation parameters
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    @return return value (returned on success) dhCompensation robot DH parameter compensation values (mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
    """

    @log_call
    @xmlrpc_timeout
    def GetDHCompensation(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetDHCompensation()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    ***************************************************************************Robot trajectory replay********************************************************************************************
    """

    """   
    @brief  Set trajectory recording parameters
    @param  [in] Required parameter name: trajectory name
    @param  [in] Required parameter period_ms: sampling period, fixed value, 2ms or 4ms or 8ms
    @param  [in] Default parameter type: data type, 1-joint position, default 1
    @param  [in] Default parameter di_choose: DI selection, bit0~bit7 correspond to control box DI0~DI7, bit8~bit9 correspond to end DI0~DI1, 0-not selected, 1-selected, default 0
    @param  [in] Default parameter do_choose: DO selection, bit0~bit7 correspond to control box DO0~DO7, bit8~bit9 correspond to end DO0~DO1, 0-not selected, 1-selected, default 0
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDParam(self, name, period_ms, type=1, di_choose=0, do_choose=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        period_ms = int(period_ms)
        type = int(type)
        di_choose = int(di_choose)
        do_choose = int(do_choose)
        flag = True
        while flag:
            try:
                error = self.robot.SetTPDParam(type, name, period_ms, di_choose, do_choose)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Start trajectory recording
    @param  [in] Required parameter name: trajectory name
    @param  [in] Required parameter period_ms: sampling period, fixed value, 2ms or 4ms or 8ms
    @param  [in] Default parameter type: data type, 1-joint position, default 1
    @param  [in] Default parameter di_choose: DI selection, bit0~bit7 correspond to control box DI0~DI7, bit8~bit9 correspond to end DI0~DI1, 0-not selected, 1-selected, default 0
    @param  [in] Default parameter do_choose: DO selection, bit0~bit7 correspond to control box DO0~DO7, bit8~bit9 correspond to end DO0~DO1, 0-not selected, 1-selected, default 0
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDStart(self, name, period_ms, type=1, di_choose=0, do_choose=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        period_ms = int(period_ms)
        type = int(type)
        di_choose = int(di_choose)
        do_choose = int(do_choose)
        flag = True
        while flag:
            try:
                error = self.robot.SetTPDStart(type, name, period_ms, di_choose, do_choose)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Stop trajectory recording
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetWebTPDStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SetWebTPDStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Delete trajectory recording
    @param  [in] Required parameter name: trajectory name
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDDelete(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                error = self.robot.SetTPDDelete(name)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Trajectory preload
    @param  [in] Required parameter name: trajectory name
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LoadTPD(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                error = self.robot.LoadTPD(name)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get trajectory start pose
    @param  [in] name trajectory file name, file extension not required
    @return error code success- 0, failure-error code
    @return return value (returned on success) desc_pose [x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetTPDStartPose(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                _error = self.robot.GetTPDStartPose(name)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Trajectory replay
    @param  [in] Required parameter name: trajectory name
    @param  [in] Required parameter blend: whether to blend, 0-no blending, 1-blending
    @param  [in] Required parameter ovl: velocity scaling factor, range [0~100]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveTPD(self, name, blend, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        name = str(name)
        blend = int(blend)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.MoveTPD(name, blend, ovl)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Trajectory preprocessing
    @param  [in] Required parameter name: trajectory name, e.g. /fruser/traj/trajHelix_aima_1.txt
    @param  [in] Required parameter ovl velocity scaling percentage, range [0~100]
    @param  [in] Default parameter opt 1-control point, default is 1
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LoadTrajectoryJ(self, name, ovl, opt=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        ovl = float(ovl)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.LoadTrajectoryJ(name, ovl, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Trajectory replay
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveTrajectoryJ(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.MoveTrajectoryJ()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get trajectory start pose
    @param  [in] Required parameter name: trajectory name
    @return error code success- 0, failure-error code
    @return return value (returned on success) desc_pose [x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetTrajectoryStartPose(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                _error = self.robot.GetTrajectoryStartPose(name)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Get trajectory point number
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) pnum
    """

    @log_call
    @xmlrpc_timeout
    def GetTrajectoryPointNum(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetTrajectoryPointNum()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Set the velocity during trajectory execution
    @param  [in] Required parameter ovl velocity scaling percentage, range [0~100]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJSpeed(self, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJSpeed(ovl)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the force and torque during trajectory execution
    @param  [in] Required parameter ft [fx,fy,fz,tx,ty,tz], units N and Nm
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceTorque(self, ft):
        while self.reconnect_flag:
            time.sleep(0.1)
        ft = list(map(float, ft))
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJForceTorque(ft)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the force along the x direction during trajectory execution
    @param  [in] Required parameter fx force along the x direction, unit N
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFx(self, fx):
        while self.reconnect_flag:
            time.sleep(0.1)
        fx = float(fx)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJForceFx(fx)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the force along the y direction during trajectory execution
    @param  [in] Required parameter fy force along the y direction, unit N
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFy(self, fy):
        while self.reconnect_flag:
            time.sleep(0.1)
        fy = float(fy)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJForceFy(fy)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the force along the z direction during trajectory execution
    @param  [in] Required parameter fz force along the z direction, unit N
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFz(self, fz):
        while self.reconnect_flag:
            time.sleep(0.1)
        fz = float(fz)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJForceFy(fz)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the torque about the x axis during trajectory execution
    @param  [in] Required parameter tx torque about the x axis, unit Nm
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTx(self, tx):
        while self.reconnect_flag:
            time.sleep(0.1)
        tx = float(tx)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJTorqueTx(tx)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the torque about the y axis during trajectory execution
    @param  [in] Required parameter ty torque about the y axis, unit Nm
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTy(self, ty):
        while self.reconnect_flag:
            time.sleep(0.1)
        ty = float(ty)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJTorqueTx(ty)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the torque about the z axis during trajectory execution
    @param  [in] Required parameter tz torque about the z axis, unit Nm
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTz(self, tz):
        while self.reconnect_flag:
            time.sleep(0.1)
        tz = float(tz)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJTorqueTx(tz)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************Robot WebAPP program usage********************************************************************************************
    """

    """   
    @brief  Set automatic loading of the default job program on startup
    @param  [in] Required parameter flag: 0-do not auto-load default program on startup, 1-auto-load default program on startup
    @param  [in] Required parameter program_name: job program name and path, e.g. "/fruser/movej.lua", where "/fruser/" is the fixed path
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LoadDefaultProgConfig(self, flag, program_name):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        program_name = str(program_name)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.LoadDefaultProgConfig(flag, program_name)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  Load the specified job program
    @param  [in] Required parameter program_name: job program name and path, e.g. "/fruser/movej.lua", where "/fruser/" is the fixed path
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ProgramLoad(self, program_name):
        while self.reconnect_flag:
            time.sleep(0.1)
        program_name = str(program_name)
        flag = True
        while flag:
            try:
                error = self.robot.ProgramLoad(program_name)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get the execution line number of the current robot job program
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) line_num
    """

    @log_call
    @xmlrpc_timeout
    def GetCurrentLine(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetCurrentLine()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Run the currently loaded job program
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ProgramRun(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.ProgramRun()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Pause the currently running job program
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ProgramPause(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ProgramPause()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Resume the currently paused job program
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ProgramResume(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.ProgramResume()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Terminate the currently running job program
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ProgramStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ProgramStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get the execution status of the robot job program
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) state:1-program stopped or no program running, 2-program running, 3-program paused
    """

    @log_call
    @xmlrpc_timeout
    def GetProgramState(self):
        # _error = self.robot.GetProgramState()
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0,self.robot_state_pkg.robot_state

    """   
    @brief  Get the name of the loaded job program
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) program_name
    """

    @log_call
    @xmlrpc_timeout
    def GetLoadedProgram(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetLoadedProgram()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    ***************************************************************************Robot peripherals********************************************************************************************
    """

    """   
    @brief  Get gripper configuration
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) [number,company,device,softversion] 
            number gripper number
            company gripper manufacturer, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Zhixing 
            device  device number, Robotiq(0-2F-85 series), Huiling(0-NK series,1-Z-EFG-100), Tianji(0-TEG-110), Dahuan(0-PGI-140), Zhixing(0-CTPM2F20)
            softvesion  software version number, not used for now, default is 0
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetGripperConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1] + 1, _error[2] + 1, _error[3], _error[4]]
        else:
            return error,None

    """   
    @brief  Activate gripper
    @param  [in] Required parameter index: gripper number
    @param  [in] Required parameter action:0-reset, 1-activate
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ActGripper(self, index, action):
        while self.reconnect_flag:
            time.sleep(0.1)
        index = int(index)
        action = int(action)
        flag = True
        while flag:
            try:
                error = self.robot.ActGripper(index, action)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Control gripper
    @param  [in] Required parameter index: gripper number
    @param  [in] Required parameter pos: position percentage, range [0~100]
    @param  [in] Required parameter vel: velocity percentage, range [0~100]
    @param  [in] Required parameter force: torque percentage, range [0~100]
    @param  [in] Required parameter maxtime: maximum wait time, range [0~30000], unit [ms]
    @param  [in] Required parameter block:0-blocking, 1-non-blocking
    @param  [in] Required parameter type gripper type, 0-parallel gripper; 1-rotary gripper
    @param  [in] Required parameter rotNum number of rotations
    @param  [in] Required parameter rotVel rotation velocity percentage [0-100]
    @param  [in] Required parameter rotTorque rotation torque percentage [0-100]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveGripper(self, index, pos, vel, force, maxtime, block, type, rotNum, rotVel, rotTorque):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        index = int(index)
        pos = int(pos)
        vel = int(vel)
        force = int(force)
        maxtime = int(maxtime)
        block = int(block)
        type = int(type)
        rotNum = float(rotNum)
        rotVel = int(rotVel)
        rotTorque = int(rotTorque)
        flag = True
        while flag:
            try:
                error = self.robot.MoveGripper(index, pos, vel, force, maxtime, block, type, rotNum, rotVel, rotTorque)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get gripper motion status
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) [fault,status]: gripper motion status, fault:0-no error, 1-error; status:0-motion not complete, 1-motion complete    
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperMotionDone(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetGripperMotionDone()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error,None

    """   
    @brief  Configure gripper
    @param  [in] Required parameter company: gripper manufacturer, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Zhixing
    @param  [in] Required parameter device: device number, Robotiq(0-2F-85 series), Huiling (0-NK series,1-Z-EFG-100), Tianji (0-TEG-110), Dahuan (0-PGI-140), Zhixing (0-CTPM2F20)
    @param  [in] Default parameter softversion: software version number, not used for now, default is 0
    @param  [in] Default parameter bus: device mounting position on the end bus, not used for now, default is 0;
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetGripperConfig(self, company, device, softversion=0, bus=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        company = int(company)
        device = int(device)
        softversion = int(softversion)
        bus = int(bus)
        flag = True
        while flag:
            try:
                error = self.robot.SetGripperConfig(company, device, softversion, bus)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Compute pre-grasp point - vision
    @param  [in] Required parameter desc_pos  grasp point Cartesian pose
    @param  [in] Required parameter zlength   z axis offset
    @param  [in] Required parameter zangle    rotation offset about the z axis
    @return error code success- 0, failure-error code
    @return Return value (returned on success) pre_pos  pre-grasp point Cartesian pose
    """

    @log_call
    @xmlrpc_timeout
    def ComputePrePick(self, desc_pos, zlength, zangle):
        while self.reconnect_flag:
            time.sleep(0.1)
        desc_pos = list(map(float, desc_pos))
        zlength = float(zlength)
        zangle = float(zangle)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputePrePick(desc_pos, zlength, zangle)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Compute retreat point - vision
    @param  [in] Required parameter desc_pos  grasp point Cartesian pose
    @param  [in] Required parameter zlength   z axis offset
    @param  [in] Required parameter zangle    rotation offset about the z axis
    @return error code success- 0, failure-error code
    @return Return value (returned on success) post_pos  retreat point Cartesian pose
    """

    @log_call
    @xmlrpc_timeout
    def ComputePostPick(self, desc_pos, zlength, zangle):
        while self.reconnect_flag:
            time.sleep(0.1)
        desc_pos = list(map(float, desc_pos))
        zlength = float(zlength)
        zangle = float(zangle)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputePostPick(desc_pos, zlength, zangle)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    ***************************************************************************Robot force control********************************************************************************************
    """

    """   
    @brief  Get force sensor configuration
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) [number,company,device,softversion,bus]
            number sensor number
            company  force sensor manufacturer, 17-Kunwei Technology, 19-Aerospace 11th Institute, 20-ATI sensor, 21-Zhongke Midian, 22-Weihang Minxin
            device  device number, Kunwei (0-KWR75B), Aerospace 11th Institute (0-MCS6A-200-4), ATI(0-AXIA80-M8), Zhongke Midian (0-MST2010), Weihang Minxin (0-WHC6L-YB10A)
            softvesion  software version number, not used for now, default is 0    
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.FT_GetConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1] + 1, _error[2] + 1, _error[3], _error[4]]
        else:
            return error,None

    """   
    @brief  Force sensor configuration
    @param  [in] Required parameter company: sensor manufacturer, 17-Kunwei Technology, 19-Aerospace 11th Institute, 20-ATI sensor, 21-Zhongke Midian, 22-Weihang Minxin, 23-NBIT, 24-Xinjingcheng(XJC), 26-NSR;
    @param  [in] Required parameter device: device number, Kunwei (0-KWR75B), Aerospace 11th Institute (0-MCS6A-200-4), ATI(0-AXIA80-M8), Zhongke Midian (0-MST2010), Weihang Minxin (0-WHC6L-YB10A), NBIT(0-XLH93003ACS), Xinjingcheng XJC(0-XJC-6F-D82), NSR(0-NSR-FTSensorA);
    @param  [in] Default parameter softversion: software version number, not used for now, default is 0
    @param  [in] Default parameter bus: device mounting position on the end bus, not used for now, default is 0;
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetConfig(self, company, device, softversion=0, bus=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        company = int(company)
        device = int(device)
        softversion = int(softversion)
        bus = int(bus)
        flag = True
        while flag:
            try:
                error = self.robot.FT_SetConfig(company, device, softversion, bus)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Force sensor activation
    @param  [in] Required parameter state: 0-reset, 1-activate
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_Activate(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)
        flag = True
        while flag:
            try:
                error = self.robot.FT_Activate(state)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Force sensor zero calibration
    @param  [in] Required parameter state: 0-remove zero point, 1-zero point correction
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetZero(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)
        flag = True
        while flag:
            try:
                error = self.robot.FT_SetZero(state)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set force sensor reference coordinate frame
    @param  [in] Required parameter ref: 0-tool coordinate frame, 1-base coordinate frame
    @param  [in] Default parameter coord: [x,y,z,rx,ry,rz] custom coordinate frame value, default [0,0,0,0,0,0]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetRCS(self, ref,coord=[0,0,0,0,0,0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        ref = int(ref)
        coord = list(map(float, coord))
        flag = True
        while flag:
            try:
                error = self.robot.FT_SetRCS(ref,coord)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Payload weight identification computation
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) weight-payload weight, unit kg
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdIdenCompute(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.FT_PdIdenCompute()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Payload weight identification recording
    @param  [in] Required parameter tool_id: sensor coordinate frame number, range [1~14]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdIdenRecord(self, tool_id):
        while self.reconnect_flag:
            time.sleep(0.1)
        tool_id = int(tool_id)
        flag = True
        while flag:
            try:
                error = self.robot.FT_PdIdenRecord(tool_id)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Payload center of mass identification computation
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) cog=[cogx,cogy,cogz] , payload center of mass, unit mm
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdCogIdenCompute(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.FT_PdCogIdenCompute()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3]]
        else:
            return error,None

    """   
    @brief  Payload center of mass identification recording
    @param  [in] Required parameter tool_id: sensor coordinate frame number, range [0~14]
    @param  [in] Required parameter index point number, range [1~3]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdCogIdenRecord(self, tool_id, index):
        while self.reconnect_flag:
            time.sleep(0.1)
        tool_id = int(tool_id)
        index = int(index)
        flag = True
        while flag:
            try:
                error = self.robot.FT_PdCogIdenRecord(tool_id, index)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get force/torque data in the reference coordinate frame
    @param  [in] Required parameter NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) data=[fx,fy,fz,tx,ty,tz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetForceTorqueRCS(self):
        # _error = self.robot.FT_GetForceTorqueRCS(0)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.ft_sensor_data[0],self.robot_state_pkg.ft_sensor_data[1],self.robot_state_pkg.ft_sensor_data[2],
                  self.robot_state_pkg.ft_sensor_data[3],self.robot_state_pkg.ft_sensor_data[4],self.robot_state_pkg.ft_sensor_data[5]]

    """   
    @brief  Get raw force/torque data of the force sensor
    @param  [in] Required parameter NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) data=[fx,fy,fz,tx,ty,tz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetForceTorqueOrigin(self):
        # _error = self.robot.FT_GetForceTorqueOrigin(0)
        # error = _error[0]
        # return error
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.ft_sensor_raw_data[0],self.robot_state_pkg.ft_sensor_raw_data[1],self.robot_state_pkg.ft_sensor_raw_data[2],
                  self.robot_state_pkg.ft_sensor_raw_data[3],self.robot_state_pkg.ft_sensor_raw_data[4],self.robot_state_pkg.ft_sensor_raw_data[5]]

    """   
    @brief  Collision guard
    @param  [in] Required parameter flag: 0-disable collision guard, 1-enable collision guard;
    @param  [in] Required parameter sensor_num: force sensor number
    @param  [in] Required parameter select: whether to detect collision on the six degrees of freedom [fx,fy,fz,mx,my,mz], 0-not effective, 1-effective
    @param  [in] Required parameter force_torque: collision detection force/torque, unit N or Nm
    @param  [in] Required parameter max_threshold: maximum threshold
    @param  [in] Required parameter min_threshold: minimum threshold
    Force/torque detection range:(force_torque-min_threshold,force_torque+max_threshold)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_Guard(self, flag, sensor_num, select, force_torque, max_threshold, min_threshold):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        sensor_num = int(sensor_num)
        select = list(map(int, select))
        force_torque = list(map(float, force_torque))
        max_threshold = list(map(float, max_threshold))
        min_threshold = list(map(float, min_threshold))
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.FT_Guard(flag, sensor_num, select, force_torque, max_threshold, min_threshold)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  Constant force control
    @param  [in] Required parameter flag: 0-disable collision guard, 1-enable collision guard;
    @param  [in] Required parameter sensor_id: force sensor number
    @param  [in] Required parameter select: [fx,fy,fz,mx,my,mz] whether to detect collision on the six degrees of freedom, 0-not effective, 1-effective
    @param  [in] Required parameter ft: [fx,fy,fz,mx,my,mz] collision detection force/torque, unit N or Nm
    @param  [in] Required parameter ft_pid: [f_p,f_i,f_d,m_p,m_i,m_d], force PID parameters, torque PID parameters
    @param  [in] Required parameter adj_sign: adaptive start/stop state, 0-disable, 1-enable
    @param  [in] Required parameter ILC_sign: ILC control start/stop state, 0-stop, 1-train, 2-operate
    @param  [in] Required parameter max_dis: maximum adjustment distance, unit mm
    @param  [in] Required parameter max_ang: maximum adjustment angle, unit deg
    @param  [in] Required parameter M mass parameter
    @param  [in] Required parameter B damping parameter
    @param  [in] Default parameter threshold rx, ry start threshold [0-10], default 0.2
    @param  [in] Default parameter adjustCoeff rx, ry torque adjustment coefficient [0-1], default 1
    @param  [in] Default parameter polishRadio: polishing disc radius, unit mm
    @param  [in] Default parameter filter_Sign filter enable flag 0-off; 1-on, default 0-off
    @param  [in] Default parameter posAdapt_sign pose compliance enable flag 0-off; 1-on, default 0-off
    @param  [in] Default parameter isNoBlock blocking flag, 0-blocking; 1-non-blocking, default 0-blocking
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_Control(self, flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M=None, B=None, threshold=[0.2,0.2], adjustCoeff=[1.0,1.0], polishRadio=0, filter_Sign=0, posAdapt_sign=0, isNoBlock=0):
        if M is None:
            M = [0, 0]
        if B is None:
            B = [0, 0]
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        sensor_id = int(sensor_id)
        select = list(map(int, select))
        ft = list(map(float, ft))
        ft_pid = list(map(float, ft_pid))
        adj_sign = int(adj_sign)
        ILC_sign = int(ILC_sign)
        max_dis = float(max_dis)
        max_ang = float(max_ang)


        M = list(map(float, M))
        B = list(map(float, B))
        threshold = list(map(float, threshold))
        adjustCoeff = list(map(float, adjustCoeff))
        polishRadio = float(polishRadio)
        filter_Sign = int(filter_Sign)
        posAdapt_sign = int(posAdapt_sign)
        isNoBlock = int(isNoBlock)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis,
                                              max_ang,polishRadio, filter_Sign, posAdapt_sign,[M[0],M[1],B[0],B[0],threshold[0],threshold[1],adjustCoeff[0],adjustCoeff[1]],isNoBlock)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  Helical exploration
    @param  [in] Required parameter rcs reference coordinate frame, 0-tool coordinate frame, 1-base coordinate frame
    @param  [in] Required parameter ft: force or torque threshold (0~100), unit N or Nm
    @param  [in] Default parameter dr: radial feed per turn, unit mm, default 0.7
    @param  [in] Default parameter max_t_ms: maximum exploration time, unit ms, default 60000
    @param  [in] Default parameter max_vel: maximum linear velocity, unit mm/s, default 5
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_SpiralSearch(self, rcs, ft, dr=0.7, max_t_ms=60000, max_vel=5):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        ft = float(ft)
        dr = float(dr)
        max_t_ms = float(max_t_ms)
        max_vel = float(max_vel)
        flag = True
        while flag:
            try:
                error = self.robot.FT_SpiralSearch(rcs, ft, dr, max_t_ms, max_vel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Rotary insertion
    @param  [in] Required parameter rcs reference coordinate frame, 0-tool coordinate frame, 1-base coordinate frame
    @param  [in] Required parameter ft: force or torque threshold (0~100), unit N or Nm
    @param  [in] Required parameter orn force/torque direction, 1-along the z axis direction, 2-about the z axis direction
    @param  [in] Default parameter angVelRot: rotation angular velocity, unit deg/s, default 3
    @param  [in] Default parameter angleMax: maximum rotation angle, unit deg, default 45
    @param  [in] Default parameter angAccmax: maximum rotation acceleration, unit deg/s^2, not used for now, default 0
    @param  [in] Default parameter rotorn: rotation direction, 1-clockwise, 2-counterclockwise, default 1
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_RotInsertion(self, rcs, ft, orn, angVelRot=3, angleMax=45, angAccmax=0, rotorn=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        ft = float(ft)
        orn = int(orn)
        angVelRot = float(angVelRot)
        angleMax = float(angleMax)
        angAccmax = float(angAccmax)
        rotorn = int(rotorn)
        flag = True
        while flag:
            try:
                error = self.robot.FT_RotInsertion(rcs, angVelRot, ft, angleMax, orn, angAccmax, rotorn)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Linear insertion
    @param  [in] Required parameter rcs reference coordinate frame, 0-tool coordinate frame, 1-base coordinate frame
    @param  [in] Required parameter ft: force or torque threshold (0~100), unit N or Nm
    @param  [in] Required parameter disMax: maximum insertion distance, unit mm
    @param  [in] Required parameter linorn: insertion direction:0-negative direction, 1-positive direction
    @param  [in] Default parameter lin_v: linear velocity, unit mm/s, default 1
    @param  [in] Default parameter lin_a: linear acceleration, unit mm/s^2, not used for now, default 1
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_LinInsertion(self, rcs, ft, disMax, linorn, lin_v=1.0, lin_a=1.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        ft = float(ft)
        disMax = float(disMax)
        linorn = int(linorn)
        lin_v = float(lin_v)
        lin_a = float(lin_a)
        flag = True
        while flag:
            try:
                error = self.robot.FT_LinInsertion(rcs, ft, lin_v, lin_a, disMax, linorn)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Compute middle plane position start
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_CalCenterStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.FT_CalCenterStart()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Compute middle plane position end
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) pos=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_CalCenterEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.FT_CalCenterEnd()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Surface positioning
    @param  [in] Required parameter rcs: reference coordinate frame, 0-tool coordinate frame, 1-base coordinate frame
    @param  [in] Required parameter dir: movement direction, 1-positive direction, 2-negative direction
    @param  [in] Required parameter axis: moving axis, 1-x, 2-y, 3-z
    @param  [in] Required parameter disMax: maximum exploration distance, unit mm
    @param  [in] Required parameter ft: action termination force threshold, unit N
    @param  [in] Default parameter lin_v: exploration linear velocity, unit mm/s, default 3
    @param  [in] Default parameter lin_a: exploration linear acceleration, unit mm/s^2, default 0
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_FindSurface(self, rcs, dir, axis, disMax, ft, lin_v=3.0, lin_a=0.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        dir = int(dir)
        axis = int(axis)
        ft = float(ft)
        lin_v = float(lin_v)
        lin_a = float(lin_a)
        flag = True
        while flag:
            try:
                error = self.robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, disMax, ft)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Disable compliance control
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_ComplianceStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.FT_ComplianceStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Enable compliance control
    @param  [in] Required parameter p: position adjustment coefficient or compliance coefficient
    @param  [in] Required parameter force: compliance enable force threshold, unit N
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FT_ComplianceStart(self, p, force):
        while self.reconnect_flag:
            time.sleep(0.1)
        p = float(p)
        force = float(force)
        flag = True
        while flag:
            try:
                error = self.robot.FT_ComplianceStart(p, force)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Load identification filter initialization
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyDynFilterInit(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LoadIdentifyDynFilterInit()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Load identification variable initialization
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyDynVarInit(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LoadIdentifyDynVarInit()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Load identification main program
    @param  [in] Required parameter joint_torque joint torque j1-j6
    @param  [in] Required parameter joint_pos joint position j1-j6
    @param  [in] Required parameter t sampling period
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyMain(self, joint_torque, joint_pos, t):
        while self.reconnect_flag:
            time.sleep(0.1)
        joint_torque = list(map(float, joint_torque))
        joint_pos = list(map(float, joint_pos))
        t = float(t)
        flag = True
        while flag:
            try:
                error = self.robot.LoadIdentifyMain(joint_torque, joint_pos, t)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get load identification result
    @param  [in] Required parameter gain gravity term coefficient double[6], centrifugal term coefficient double[6] 
    @return error code success- 0, failure-error code
    @return Return value (returned on success) weight load weight
    @return Return value (returned on success) cog load center of gravity [x,y,z]
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyGetResult(self, gain):
        while self.reconnect_flag:
            time.sleep(0.1)
        gain = list(map(float, gain))
        flag = True
        while flag:
            try:
                _error = self.robot.LoadIdentifyGetResult(gain)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], [_error[2], _error[3], _error[4]]
        else:
            return error,None,None

    """   
    ***************************************************************************Conveyor Belt Functions********************************************************************************************
    """

    """   
    @brief  Conveyor belt start, stop
    @param  [in] Required parameter status status, 1-start, 0-stop 
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorStartEnd(self, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorStartEnd(status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Record IO detection point
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointIORecord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorPointIORecord()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Record point A
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointARecord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorPointARecord()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Record reference point
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorRefPointRecord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorRefPointRecord()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Record point B
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointBRecord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorPointBRecord()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Conveyor belt workpiece IO detection
    @param  [in] Required parameter max_t maximum detection time, unit ms
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorIODetect(self, max_t):
        while self.reconnect_flag:
            time.sleep(0.1)
        max_t = int(max_t)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorIODetect(max_t)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get current object position
    @param  [in] Required parameter  mode 1-tracking grab 2-tracking motion 3-TPD tracking
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorGetTrackData(self, mode):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorGetTrackData(mode)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Conveyor belt tracking start
    @param  [in] Required parameter  status status, 1-start, 0-stop
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackStart(self, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorTrackStart(status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Conveyor belt tracking stop
    @param  [in] NULL
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorTrackEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Conveyor belt parameter configuration
    @param  [in] Required parameter  param = [encChannel,resolution,lead,wpAxis,vision,speedRadio]  encChannel encoder channel 1-2, resolution encoder resolution, number of pulses per encoder revolution,
    lead mechanical transmission ratio, conveyor belt movement distance per encoder revolution, wpAxis  work object coordinate frame number, select work object coordinate frame number for tracking motion function, set to 0 for tracking grab and TPD tracking, vision whether vision is configured  0 not configured  1 configured,
    speedRadio speed ratio  for conveyor belt tracking grab the range is (1-100)  set to 1 for tracking motion and TPD tracking
    @param  [in] Required parameter followType tracking motion type, 0-tracking motion; 1-chasing detection motion
    @param  [in] Default parameter startDis to be set for chasing detection grab, tracking start distance, -1: automatic calculation (automatically chase after the workpiece arrives below the robot), unit mm, default value 0
    @param  [in] Default parameter endDis to be set for chasing detection grab, tracking termination distance, unit mm, default value 100
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorSetParam(self, param, followType, startDis=0, endDis=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        param = list(map(float, param))
        followType = int(followType)
        startDis = int(startDis)
        endDis = int(endDis)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorSetParam(param, followType, startDis, endDis)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Conveyor belt grab point compensation
    @param  [in] Required parameter cmp compensation position [x,y,z]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorCatchPointComp(self, cmp):
        while self.reconnect_flag:
            time.sleep(0.1)
        cmp = list(map(float, cmp))
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorCatchPointComp(cmp)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Linear motion
    @param  [in] Required parameter  name cvrCatchPoint and cvrRaisePoint
    @param  [in] Required parameter tool tool number
    @param  [in] Required parameter wobj work object number
    @param  [in] Default parameter vel velocity default 20
    @param  [in] Default parameter acc acceleration default 100
    @param  [in] Default parameter ovl velocity scaling factor default 100
    @param  [in] Default parameter blendR:[-1.0]-move to position (blocking), [0~1000]-blend radius (non-blocking), unit [mm] default -1.0
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackMoveL(self, name, tool, wobj, vel=20, acc=100, ovl=100, blendR=-1.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        name = str(name)
        tool = int(tool)
        wobj = int(wobj)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorTrackMoveL(name, tool, wobj, vel, acc, ovl, blendR, 0, 0)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************Welding Functions********************************************************************************************
    """


    """   
    @brief  Welding start 
    @param  [in] Required parameter ioType io type 0-controller IO; 1-extended IO
    @param  [in] Required parameter arcNum welder configuration file number
    @param  [in] Required parameter timeout arc ignition timeout
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ARCStart(self, ioType, arcNum, timeout):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)
        flag = True
        while flag:
            try:
                error = self.robot.ARCStart(ioType, arcNum, timeout)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Welding end 
    @param  [in] Required parameter ioType io type 0-controller IO; 1-extended IO
    @param  [in] Required parameter arcNum welder configuration file number
    @param  [in] Required parameter timeout arc extinction timeout
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ARCEnd(self, ioType, arcNum, timeout):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)
        flag = True
        while flag:
            try:
                error = self.robot.ARCEnd(ioType, arcNum, timeout)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the correspondence between welding current and analog output 
    @param  [in] Required parameter currentMin welding current-analog output linear relationship left point current value (A)
    @param  [in] Required parameter currentMax welding current-analog output linear relationship right point current value (A)
    @param  [in] Required parameter outputVoltageMin welding current-analog output linear relationship left point analog output voltage value (V)
    @param  [in] Required parameter outputVoltageMax welding current-analog output linear relationship right point analog output voltage value (V)
    @param  [in] Required parameter AOIndex welding current analog output port
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrentRelation(self, currentMin, currentMax, outputVoltageMin, outputVoltageMax,AOIndex):
        while self.reconnect_flag:
            time.sleep(0.1)
        currentMin = float(currentMin)
        currentMax = float(currentMax)
        outputVoltageMin = float(outputVoltageMin)
        outputVoltageMax = float(outputVoltageMax)
        AOIndex =int(AOIndex)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCurrentRelation(currentMin, currentMax, outputVoltageMin, outputVoltageMax,AOIndex)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set the correspondence between welding voltage and analog output 
    @param  [in] Required parameter weldVoltageMin welding voltage-analog output linear relationship left point welding voltage value (A)
    @param  [in] Required parameter weldVoltageMax welding voltage-analog output linear relationship right point welding voltage value (A)
    @param  [in] Required parameter outputVoltageMin welding voltage-analog output linear relationship left point analog output voltage value (V)
    @param  [in] Required parameter outputVoltageMax welding voltage-analog output linear relationship right point analog output voltage value (V)    
    @param  [in] Required parameter AOIndex welding voltage analog output port
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltageRelation(self, weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax,AOIndex):
        while self.reconnect_flag:
            time.sleep(0.1)
        weldVoltageMin = float(weldVoltageMin)
        weldVoltageMax = float(weldVoltageMax)
        outputVoltageMin = float(outputVoltageMin)
        outputVoltageMax = float(outputVoltageMax)
        AOIndex =int(AOIndex)

        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetVoltageRelation(weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax,AOIndex)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get the correspondence between welding current and analog output 
    @return error code success- 0, failure-error code
    @return Return value (returned on success) currentMin welding current-analog output linear relationship left point current value (A)
    @return Return value (returned on success) currentMax welding current-analog output linear relationship right point current value (A)
    @return Return value (returned on success) outputVoltageMin welding current-analog output linear relationship left point analog output voltage value (V)
    @return Return value (returned on success) outputVoltageMax welding current-analog output linear relationship right point analog output voltage value (V)
    @return Return value (returned on success) AOIndex welding voltage and current analog output port
    """

    @log_call
    @xmlrpc_timeout
    def WeldingGetCurrentRelation(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        try:
            flag = True
            while flag:
                try:
                    _error = self.robot.WeldingGetCurrentRelation()
                    flag = False
                except socket.error as e:
                    flag = True

            error = _error[0]
            if error == 0:
                return error, _error[1], _error[2], _error[3], _error[4], _error[5]
            return _error,None,None,None,None,None
        except Exception as e:
            return RobotError.ERR_RPC_ERROR,None,None,None,None,None

    """   
    @brief  Get the correspondence between welding voltage and analog output 
    @return error code success- 0, failure-error code
    @return Return value (returned on success) weldVoltageMin welding voltage-analog output linear relationship left point welding voltage value (A)
    @return Return value (returned on success) weldVoltageMax welding voltage-analog output linear relationship right point welding voltage value (A)
    @return Return value (returned on success) outputVoltageMin welding voltage-analog output linear relationship left point analog output voltage value (V)
    @return Return value (returned on success) outputVoltageMax welding current-analog output linear relationship right point analog output voltage value (V)
    @return Return value (returned on success) AOIndex welding voltage analog output port
    """

    @log_call
    @xmlrpc_timeout
    def WeldingGetVoltageRelation(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        try:
            flag = True
            while flag:
                try:
                    _error = self.robot.WeldingGetVoltageRelation()
                    flag = False
                except socket.error as e:
                    flag = True

            error = _error[0]
            if error == 0:
                return error, _error[1], _error[2], _error[3], _error[4], _error[5]
            return _error,None,None,None,None,None
        except Exception as e:
            return RobotError.ERR_RPC_ERROR,None,None,None,None,None

    """   
    @brief  Set welding current 
    @param  [in] Required parameter ioType io type 0-controller IO; 1-extended IO
    @param  [in] Required parameter float current welding current value (A)
    @param  [in] Required parameter AOIndex welding current control box analog output port (0-1)
    @param  [in] Required parameter blend whether to smooth 0-no smoothing, 1-smoothing
    @return Error code success- 0, failure-
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrent(self, ioType, current, AOIndex,blend):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        current = float(current)
        AOIndex = int(AOIndex)
        blend = int(blend)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCurrent(ioType, current, AOIndex,blend)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set welding voltage 
    @param  [in] Required parameter ioType io type 0-controller IO; 1-extended IO
    @param  [in] Required parameter float voltage welding voltage value (A)
    @param  [in] Required parameter AOIndex welding voltage control box analog output port (0-1)
    @param  [in] Required parameter blend whether to smooth 0-no smoothing, 1-smoothing
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltage(self, ioType, voltage, AOIndex,blend):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        voltage = float(voltage)
        AOIndex = int(AOIndex)
        blend = int(blend)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetVoltage(ioType, voltage, AOIndex,blend)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set weaving parameters 
    @param  [in] Required parameter int weaveNum weaving parameter configuration number
    @param  [in] Required parameter int weaveType weaving type 0-planar triangular wave weaving; 1-vertical L-shaped triangular wave weaving; 2-clockwise circular weaving; 3-counterclockwise circular weaving; 4-planar sine wave weaving; 5-vertical L-shaped sine wave weaving; 6-vertical triangular wave weaving; 7-vertical sine wave weaving
    @param  [in] Required parameter float weaveFrequency weaving frequency (Hz)
    @param  [in] Required parameter int weaveIncStayTime wait mode 0-period does not include wait time; 1-period includes wait time
    @param  [in] Required parameter float weaveRange weaving amplitude (mm)
    @param  [in] Required parameter float weaveLeftRange vertical triangular weaving left chord length (mm)
    @param  [in] Required parameter float weaveRightRange vertical triangular weaving right chord length (mm)
    @param  [in] Required parameter int additionalStayTime vertical triangular weaving vertical triangle point stay time (ms)
    @param  [in] Required parameter int weaveLeftStayTime weaving left stay time (ms)
    @param  [in] Required parameter int weaveRightStayTime weaving right stay time (ms)
    @param  [in] Required parameter int weaveCircleRadio circular weaving-callback ratio (0-100%)
    @param  [in] Required parameter int weaveStationary weaving position wait, 0-position continues to move during wait time; 1-position stationary during wait time
    @param  [in] Required parameter float weaveYawAngle weaving direction azimuth angle (rotation around weaving Z axis), unit °, default 0
    @param  [in] Required parameter float weaveRotAngle weaving direction azimuth angle (rotation around weaving X axis), unit °, default 0
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WeaveSetPara(self, weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange,
                     weaveLeftRange, weaveRightRange, additionalStayTime, weaveLeftStayTime,
                     weaveRightStayTime, weaveCircleRadio, weaveStationary,weaveYawAngle=0,weaveRotAngle=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        weaveType = int(weaveType)
        weaveFrequency = float(weaveFrequency)
        weaveIncStayTime = int(weaveIncStayTime)
        weaveRange = float(weaveRange)
        weaveLeftRange = float(weaveLeftRange)
        weaveRightRange = float(weaveRightRange)
        additionalStayTime = int(additionalStayTime)
        weaveLeftStayTime = int(weaveLeftStayTime)
        weaveRightStayTime = int(weaveRightStayTime)
        weaveCircleRadio = int(weaveCircleRadio)
        weaveStationary = int(weaveStationary)
        weaveYawAngle = float(weaveYawAngle)
        weaveRotAngle = float(weaveRotAngle)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange,
                                                weaveLeftRange, weaveRightRange, additionalStayTime,
                                                weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio, weaveStationary,weaveYawAngle,weaveRotAngle)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set weaving parameters immediately 
    @param  [in] Required parameter int weaveNum weaving parameter configuration number
    @param  [in] Required parameter int weaveType weaving type 0-planar triangular wave weaving; 1-vertical L-shaped triangular wave weaving; 2-clockwise circular weaving; 3-counterclockwise circular weaving; 4-planar sine wave weaving; 5-vertical L-shaped sine wave weaving; 6-vertical triangular wave weaving; 7-
    @param  [in] Required parameter float weaveFrequency weaving frequency (Hz)
    @param  [in] Required parameter int weaveIncStayTime wait mode 0-period does not include wait time; 1-period includes wait time
    @param  [in] Required parameter float weaveRange weaving amplitude (mm)
    @param  [in] Required parameter int weaveLeftStayTime weaving left stay time (ms)
    @param  [in] Required parameter int weaveRightStayTime weaving right stay time (ms)
    @param  [in] Required parameter int weaveCircleRadio circular weaving-callback ratio (0-100%)
    @param  [in] Required parameter int weaveStationary weaving position wait, 0-position continues to move during wait time; 1-position stationary during wait time
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WeaveOnlineSetPara(self, weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftStayTime,
                           weaveRightStayTime, weaveCircleRadio, weaveStationary):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        weaveType = int(weaveType)
        weaveFrequency = float(weaveFrequency)
        weaveIncStayTime = int(weaveIncStayTime)
        weaveRange = float(weaveRange)
        weaveLeftStayTime = int(weaveLeftStayTime)
        weaveRightStayTime = int(weaveRightStayTime)
        weaveCircleRadio = int(weaveCircleRadio)
        weaveStationary = int(weaveStationary)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.WeaveOnlineSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange,
                                                          weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio,
                                                          weaveStationary)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  Weaving start 
    @param  [in] Required parameter int weaveNum weaving parameter configuration number
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WeaveStart(self, weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.WeaveStart(weaveNum)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  Weaving end 
    @param  [in] Required parameter int weaveNum weaving parameter configuration number
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WeaveEnd(self, weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.WeaveEnd(weaveNum)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  Forward wire feed 
    @param  [in] Required parameter int ioType io type  0-controller IO; 1-extended IO
    @param  [in] Required parameter int wireFeed wire feed control  0-stop wire feed; 1-wire feed
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetForwardWireFeed(self, ioType, wireFeed):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        wireFeed = int(wireFeed)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.SetForwardWireFeed(ioType, wireFeed)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  Reverse wire feed 
    @param  [in] Required parameter int ioType io type  0-controller IO; 1-extended IO
    @param  [in] Required parameter int wireFeed wire feed control  0-stop wire feed; 1-wire feed
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetReverseWireFeed(self, ioType, wireFeed):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        wireFeed = int(wireFeed)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.SetReverseWireFeed(ioType, wireFeed)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  Gas feed
    @param  [in] Required parameter int ioType io type  0-controller IO; 1-extended IO
    @param  [in] Required parameter int airControl gas feed control  0-stop gas feed; 1-gas feed
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAspirated(self, ioType, airControl):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        airControl = int(airControl)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.SetAspirated(ioType, airControl)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR


    """   
    @brief  Segmented welding get position and orientation
    @param  [in]Required parameter startPos=[x,y,z,rx,ry,rz] start point coordinates
    @param  [in]Required parameter endPos=[x,y,z,rx,ry,rz] end point coordinates
    @param  [in]Required parameter startDistance length from welding point to start point
    @return Error code success- 0, failure-error code    
    @return Return value (returned on success) weldPointDesc=[x,y,z,rx,ry,rz] Cartesian coordinate information of the welding point
    @return Return value (returned on success) weldPointJoint=[j1,j2,j3,j4,j5,j6] joint coordinate information of the welding point
    @return Return value (returned on success) tool tool number
    @return Return value (returned on success) user work object number
    """

    @log_call
    @xmlrpc_timeout
    def GetSegmentWeldPoint(self, startPos, endPos, startDistance):
        while self.reconnect_flag:
            time.sleep(0.1)
        startPos = list(map(float, startPos))
        endPos = list(map(float, endPos))
        startDistance = float(startDistance)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSegmentWeldPoint(startPos, endPos, startDistance)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            data = _error[1].split(',')
            if len(data) != 14:
                self.log_error("GetSegmentWeldPoint fail")
                return -1
            else:
                data = list(map(float,data))
                tool = int(data[12])
                work = int(data[13])
                return (error, [ data[0],data[1],data[3],data[4],data[4],data[5]],
                        [data[6],data[7],data[8],data[9],data[10],data[11]],tool, work)
        else:
            return error,None,None,None

    """   
    @brief  Segmented welding start
    @param  [in] Required parameter startDesePos: initial Cartesian pose, unit [mm][°]
    @param  [in] Required parameter endDesePos: target Cartesian pose, unit [mm][°]
    @param  [in] Required parameter startJPos: target joint position, unit [°]
    @param  [in] Required parameter endJPos: target joint position, unit [°] 
    @param  [in] Required parameter weldLength: welding length, unit [mm]
    @param  [in] Required parameter noWeldLength: non-welding length, unit [mm]    
    @param  [in] Required parameter weaveType weaving type 0-planar triangular wave weaving; 1-vertical L-shaped triangular wave weaving; 2-clockwise circular weaving; 3-counterclockwise circular weaving; 4-planar sine wave weaving; 5-vertical L-shaped sine wave weaving; 6-vertical triangular wave weaving; 7-vertical sine wave weaving
    @param  [in] Required parameter arcNum welder configuration file number
    @param  [in] Required parameter timeout arc extinction timeout
    @param  [in] Required parameter isWeave true-welding false-not welding
    @param  [in] Required parameter int weaveNum weaving parameter configuration number
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Default parameter vel: velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc: acceleration percentage, [0~100] not open yet default 0.0
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter blendR:[-1.0]-move to position (blocking), [0~1000]-blend radius (non-blocking), unit [mm] default -1.0
    @param  [in] Default parameter exaxis_pos: external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter search:[0]-no wire searching, [1]-wire searching
    @param  [in] Default parameter offset_flag:[0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos: pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SegmentWeldStart(self, startDesePos, endDesePos, startJPos, endJPos, weldLength, noWeldLength, weldIOType,
                         arcNum, weldTimeout, isWeave, weaveNum, tool, user,
                         vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0,
                         offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        startDesePos = list(map(float, startDesePos))
        endDesePos = list(map(float, endDesePos))
        startJPos = list(map(float, startJPos))
        endJPos = list(map(float, endJPos))

        weldLength = float(weldLength)
        noWeldLength = float(noWeldLength)
        weldIOType = int(weldIOType)
        arcNum = int(arcNum)
        weldTimeout = int(weldTimeout)
        isWeave = bool(isWeave)
        weaveNum = int(weaveNum)
        tool = int(tool)
        user = int(user)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        exaxis_pos = list(map(float, exaxis_pos))
        search = int(search)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        rtn = 0
        # Get the distance between the start point and end point and the direction cosine values
        # print("1",startDesePos,endDesePos)
        result = self.robot.GetSegWeldDisDir(startDesePos[0], startDesePos[1], startDesePos[2], endDesePos[0],
                                             endDesePos[1], endDesePos[2])
        # print("result",result)
        if result[0] != 0:
            return int(result[0])

        distance = result[1]
        endOffPos = list(offset_pos)

        rtn = self.robot.MoveJ(startJPos, startDesePos, tool, user, vel, acc, ovl, exaxis_pos, blendR, offset_flag,
                               offset_pos)
        # print("rtn1", rtn)
        if rtn != 0:
            return rtn

        weldNum = 0
        noWeldNum = 0
        i = 0
        while i < int(distance / (weldLength + noWeldLength)) * 2 + 2:
            if i % 2 == 0:
                weldNum += 1
                if weldNum * weldLength + noWeldNum * noWeldLength > distance:

                    rtn = self.robot.ARCStart(weldIOType, arcNum, weldTimeout)
                    # print("rtn2", rtn)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveStart(weaveNum)
                        if rtn != 0:
                            # print("rtn3", rtn)
                            return rtn

                    # getsegmentrtn = self.robot.GetSegmentWeldPoint(startDesePos,endDesePos,weldNum* weldLength + noWeldNum * noWeldLength)
                    # # print("getsegmentrtn", getsegmentrtn)
                    # # print(startDesePos, endDesePos, weldNum * weldLength + noWeldNum * noWeldLength)
                    # # print("weldNum", weldNum, "weldLength", weldLength)
                    # # print("noWeldNum", noWeldNum, "noWeldLength", noWeldLength)
                    # if getsegmentrtn[0] != 0  :
                    #     return getsegmentrtn[0]
                    # data = getsegmentrtn[1].split(',')
                    # data = list(map(float, data))
                    # if len(data) != 14:
                    #     self.log_error("GetSegmentWeldPoint fail")
                    #     return -1
                    # tmpJoint = [data[0],data[1],data[2],data[3],data[4],data[5]]
                    # tmpWeldDesc = [data[6],data[7],data[8],data[9],data[10],data[11]]
                    # tmpTool = int(data[12])
                    # tmpUser = int(data[13])
                    rtn = self.robot.MoveL(endJPos,endDesePos, tool, user, vel, acc, ovl, blendR,0, exaxis_pos,
                                           search, 0, endOffPos)
                    # print("rtn3", rtn,endJPos,endDesePos)
                    if rtn != 0:
                        self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                        if isWeave:
                            rtn = self.robot.WeaveEnd(weaveNum)
                            # print("rtn4", rtn)
                            if rtn != 0:
                                return rtn
                        return rtn
                    rtn = self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                    # print("rtn5", rtn)
                    if rtn != 0:
                        break
                    if isWeave:
                        rtn = self.robot.WeaveEnd(weaveNum)
                        # print("rtn6", rtn)
                        if rtn != 0:
                            break

                else:
                    rtn = self.robot.ARCStart(weldIOType, arcNum, weldTimeout)
                    # print("rtn7", rtn)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveStart(weaveNum)
                        # print("rtn8", rtn)
                        if rtn != 0:
                            return rtn

                    getsegmentrtn = self.robot.GetSegmentWeldPoint(startDesePos, endDesePos,
                                                                   weldNum * weldLength + noWeldNum * noWeldLength)
                    # print("rtn9", getsegmentrtn)
                    # print(startDesePos, endDesePos, weldNum * weldLength + noWeldNum * noWeldLength)
                    # print("weldNum", weldNum, "weldLength", weldLength)
                    # print("noWeldNum", noWeldNum, "noWeldLength", noWeldLength)
                    if getsegmentrtn[0] != 0:
                        return getsegmentrtn[0]
                    data = getsegmentrtn[1].split(',')
                    data = list(map(float, data))
                    if len(data) != 14:
                        self.log_error("GetSegmentWeldPoint fail")
                        return -1
                    tmpJoint = [data[0], data[1], data[2], data[3], data[4], data[5]]
                    tmpWeldDesc = [data[6], data[7], data[8], data[9], data[10], data[11]]
                    tmpTool = int(data[12])
                    tmpUser = int(data[13])
                    # print("tmpJoint",tmpJoint,tmpWeldDesc,tmpTool,tmpUser)
                    time.sleep(1)
                    nihao = self.robot.MoveL(tmpJoint, tmpWeldDesc, tmpTool, tmpUser, vel, acc, ovl, blendR,0, exaxis_pos,
                                           search, 0, endOffPos)
                    # print("rtn10nihao", nihao)
                    if nihao != 0:
                        self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                        if isWeave:
                            rtn = self.robot.WeaveEnd(weaveNum)
                            # print("rtn11", rtn)
                            if rtn != 0:
                                return rtn
                        return rtn
                    rtn = self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                    # print("rtn12", rtn)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveEnd(weaveNum)
                        # print("rtn13", rtn)
                        if rtn != 0:
                            return rtn
            else:
                noWeldNum += 1
                if weldNum * weldLength + noWeldNum * noWeldLength > distance:
                    # getsegmentrtn = self.robot.GetSegmentWeldPoint(startDesePos, endDesePos, weldNum* weldLength + noWeldNum * noWeldLength)
                    # # print("rtn14", getsegmentrtn)
                    # # print(startDesePos, endDesePos, weldNum * weldLength + noWeldNum * noWeldLength)
                    # # print("weldNum", weldNum, "weldLength", weldLength)
                    # # print("noWeldNum", noWeldNum, "noWeldLength", noWeldLength)
                    # if getsegmentrtn[0] != 0:
                    #     return getsegmentrtn[0]
                    # data = getsegmentrtn[1].split(',')
                    # data = list(map(float,data))
                    # if len(data) != 14:
                    #     self.log_error("GetSegmentWeldPoint fail")
                    #     return -1
                    # tmpJoint = [data[0], data[1], data[2], data[3], data[4], data[5]]
                    # tmpWeldDesc = [data[6], data[7], data[8], data[9], data[10], data[11]]
                    # tmpTool = int(data[12])
                    # tmpUser = int(data[13])
                    rtn = self.robot.MoveL(endJPos,endDesePos, tool, user, vel, acc, ovl, blendR,0, exaxis_pos,
                                           search, 0, endOffPos)
                    # print("rtn15", rtn,endJPos,endDesePos)
                    if rtn != 0:
                       return rtn
                    break
                else:
                    getsegmentrtn = self.robot.GetSegmentWeldPoint(startDesePos, endDesePos, weldNum* weldLength + noWeldNum * noWeldLength)
                    # print("rtn16", getsegmentrtn,startDesePos,endDesePos,weldNum* weldLength + noWeldNum * noWeldLength)

                    # print(startDesePos,endDesePos,weldNum* weldLength + noWeldNum * noWeldLength)
                    # print("weldNum",weldNum,"weldLength",weldLength)
                    # print("noWeldNum", noWeldNum, "noWeldLength", noWeldLength)
                    if getsegmentrtn[0] != 0:
                        return getsegmentrtn[0]
                    data = getsegmentrtn[1].split(',')
                    data = list(map(float, data))
                    if len(data) != 14:
                        self.log_error("GetSegmentWeldPoint fail")
                        return -1
                    tmpJoint = [data[0], data[1], data[2], data[3], data[4], data[5]]
                    tmpWeldDesc = [data[6], data[7], data[8], data[9], data[10], data[11]]
                    tmpTool = int(data[12])
                    tmpUser = int(data[13])
                    rtn = self.robot.MoveL(tmpJoint, tmpWeldDesc, tmpTool, tmpUser, vel, acc, ovl, blendR,0, exaxis_pos,
                                           search, 0, endOffPos)
                    # print("rtn17", rtn)
                    if rtn != 0:
                        return rtn
            i =i + 1
        return rtn

    """   
    @brief  Segmented welding termination
    @param  [in] Required parameter ioType: io type 0-controller IO; 1-extended IO
    @param  [in] Required parameter arcNum: welder configuration file number
    @param  [in] Required parameter timeout: arc extinction timeout
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SegmentWeldEnd(self, ioType, arcNum, timeout):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)

        flag = True
        while flag:
            try:
                rtn = self.robot.SegmentWeldEnd(ioType, arcNum, timeout)
                flag = False
            except socket.error as e:
                flag = True

        return rtn

    """   
    @brief  Initialize log parameters
    @param  [in]Default parameter output_model: output mode, 0-direct output; 1-buffered output; 2-asynchronous output, default 1
    @param  [in]Default parameter file_path: file save path + name, the name must be in the form of xxx.log, for example /home/fr/linux/fairino.log.
                    Defaults to the path where the executing program is located, default name fairino_ year+month+data.log (e.g.: fairino_2024_03_13.log);
    @param  [in]Default parameter file_num: number of files for rolling storage, 1~20, default value is 5. Single file limit 50M;
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LoggerInit(self, output_model=1, file_path="", file_num=5):
        return self.setup_logging(output_model, file_path, file_num)

    """   
    @brief  Set log filter level
    @param  [in] Default parameter lvl: filter level value, the smaller the value the less log output, 1-error, 2-warnning, 3-inform, 4-debug, default value is 1.
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetLoggerLevel(self, lvl=1):
        lvl=int(lvl)
        log_level = self.set_log_level(lvl)
        return 0

    """   
    @brief  Download waypoint table database
    @param  [in] pointTableName name of the waypoint table to download    pointTable1.db
    @param  [in] saveFilePath storage path for the downloaded waypoint table   C://test/
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def PointTableDownLoad(self, point_table_name, save_file_path):
        if not os.path.exists(save_file_path):
            return RobotError.ERR_SAVE_FILE_PATH_NOT_FOUND

        rtn = self.robot.PointTableDownload(point_table_name)
        if rtn == -1:
            return RobotError.ERR_POINTTABLE_NOTFOUND
        elif rtn != 0:
            return rtn
        port = 20011
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)
        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        total_buffer = bytearray(1024 * 1024 * 50)  # 50Mb
        total_size = 0
        recv_md5 = ""
        recv_size = 0
        find_head_flag = False
        while True:
            buffer = client.recv(1024)
            length = len(buffer)
            if length < 1:
                return RobotError.ERR_OTHER
            total_buffer[total_size:total_size + len(buffer)] = buffer

            total_size += len(buffer)
            if not find_head_flag and total_size > 4 and total_buffer[:4].decode('utf-8') == "/f/b":
                find_head_flag = True
            # After finding the file header, extract the file size and MD5 checksum. The file size information is located in bytes 5 to 12 of the total data, and the MD5 checksum information is located in bytes 13 to 44 of the total data.
            if find_head_flag and total_size > 12 + 32:
                recv_size = int(total_buffer[4:12].decode('utf-8'))
                recv_md5 = total_buffer[12:44].decode('utf-8')
            # Break out of the loop when the entire file is received
            if find_head_flag and total_size == recv_size:
                break
        if total_size == 0:
            return RobotError.ERR_OTHER
        file_buffer = total_buffer[12 + 32:total_size - 4]

        with open(os.path.join(save_file_path, point_table_name), 'wb') as file_writer:
            file_writer.write(file_buffer[:total_size - 16 - 32])

        check_md5 = calculate_file_md5(save_file_path + point_table_name)
        if check_md5 == recv_md5:
            client.send("SUCCESS".encode('utf-8'))
            return 0
        else:
            client.send("FAIL".encode('utf-8'))
            os.remove(os.path.join(save_file_path, point_table_name))
            return RobotError.ERR_OTHER

    """   
    @brief  Upload waypoint table database
    @param  [in] pointTableFilePath full path name for uploading the waypoint table   C://test/pointTable1.db
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def PointTableUpLoad(self, point_table_file_path):
        MAX_UPLOAD_FILE_SIZE = 2 * 1024 * 1024  # Maximum upload file is 2Mb
        # Check whether the file to upload exists
        if not os.path.exists(point_table_file_path):
            return RobotError.ERR_UPLOAD_FILE_NOT_FOUND

        file_info = os.path.getsize(point_table_file_path)
        total_size = file_info + 16 + 32
        if total_size > MAX_UPLOAD_FILE_SIZE:
            print("Files larger than 2 MB are not supported!")
            return -1

        point_table_name = os.path.basename(point_table_file_path)

        rtn = self.robot.PointTableUpload(point_table_name)
        if rtn != 0:
            return rtn

        port = 20010

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)

        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER

        client.settimeout(2)

        # client.receive_timeout = 2000
        # client.send_timeout = 2000

        send_md5 = calculate_file_md5(point_table_file_path)

        head_data = f"/f/b{total_size:08d}{send_md5}"
        num = client.send(head_data.encode('utf-8'))
        if num < 1:
            return RobotError.ERR_OTHER

        with open(point_table_file_path, 'rb') as fs:
            file_bytes = fs.read()

        num = client.send(file_bytes)
        if num < 1:
            return RobotError.ERR_OTHER
        end_data = "/b/f"
        num = client.send(end_data.encode('utf-8'))
        if num < 1:
            return RobotError.ERR_OTHER

        result_buf = client.recv(1024)
        if result_buf[:7].decode('utf-8') == "SUCCESS":
            return RobotError.ERR_SUCCESS
        else:
            return RobotError.ERR_OTHER

    """   
    @brief  Point table switch
    @param  [in] PointTableSwitch Name of the point table to switch to   "pointTable1.db", when the point table is empty, i.e. "", it means updating the lua program to the initial program without an applied point table
    @return Error code Success-0   Failure-error code 
    @return Error errorStr
    """

    @log_call
    @xmlrpc_timeout
    def PointTableSwitch(self, point_table_name):
        rtn = self.robot.PointTableSwitch(point_table_name)  # Switch point table
        if rtn != 0:
            if rtn == RobotError.ERR_POINTTABLE_NOTFOUND:
                error_str = "PointTable not Found!"
            else:
                error_str = "PointTable not Found!"
            return rtn, error_str
        return rtn

    """   
    @brief  Point table update lua file
    @param  [in] pointTableName Name of the point table to switch to   "pointTable1.db", when the point table is empty, i.e. "", it means updating the lua program to the initial program without an applied point table
    @param  [in] luaFileName Name of the lua file to update   "testPointTable.lua"
    @return Error code Success-0   Failure-error code 
    @return Error errorStr
    """

    @log_call
    @xmlrpc_timeout
    def PointTableUpdateLua(self, point_table_name, lua_file_name):
        try:

            rtn = self.robot.PointTableSwitch(point_table_name)  # Switch point table
            if rtn != 0:
                if rtn == RobotError.ERR_POINTTABLE_NOTFOUND:
                    error_str = "PointTable not Found!"
                else:
                    error_str = "PointTable not Found!"
                return rtn, error_str

            time.sleep(0.3)  # Add delay to ensure the backend has actually received the switched point table name after switching

            result = self.robot.PointTableUpdateLua(lua_file_name)
            error_str = result[1]
            if not error_str:
                error_str = "fail to update lua, please inspect pointtable"
            return result[0], error_str

        except Exception as e:
            return RobotError.ERR_RPC_ERROR, ""

    """   
    @brief  Download file
    @param  [in] fileType File type    0-lua file
    @param  [in] fileName File name    "test.lua"
    @param  [in] saveFilePath Save file path    "C://test/"
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def __FileDownLoad(self, fileType, fileName, saveFilePath):
        if not os.path.exists(saveFilePath):
            return RobotError.ERR_SAVE_FILE_PATH_NOT_FOUND
        rtn = self.robot.FileDownload(fileType, fileName)
        if rtn == -1:
            return RobotError.ERR_POINTTABLE_NOTFOUND
        elif rtn != 0:
            return rtn
        port = 20011
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)
        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        total_buffer = bytearray(1024 * 1024 * 50)  # 50Mb
        total_size = 0
        recv_md5 = ""
        recv_size = 0
        find_head_flag = False
        while True:
            buffer = client.recv(1024)
            length = len(buffer)
            if length < 1:
                return RobotError.ERR_OTHER
            total_buffer[total_size:total_size + len(buffer)] = buffer
            total_size += len(buffer)
            if not find_head_flag and total_size > 4 and total_buffer[:4].decode('utf-8') == "/f/b":
                find_head_flag = True
            # After finding the file header, extract the file size and MD5 checksum. The file size information is located in bytes 5 to 12 of the total data, and the MD5 checksum information is located in bytes 13 to 44 of the total data.
            # if find_head_flag and total_size > 12 + 32:
            if find_head_flag and total_size > 14 + 32:
                # recv_size = int(total_buffer[4:12].decode('utf-8'))
                recv_size = int(total_buffer[4:14].decode('utf-8'))
                # recv_md5 = total_buffer[12:44].decode('utf-8')
                recv_md5 = total_buffer[14:46].decode('utf-8')
            # Break out of the loop when the entire file is received
            if find_head_flag and total_size == recv_size:
                break
        if total_size == 0:
            return RobotError.ERR_OTHER
        # file_buffer = total_buffer[12 + 32:total_size - 4]
        file_buffer = total_buffer[14 + 32:total_size - 4]
        with open(os.path.join(saveFilePath, fileName), 'wb') as file_writer:
            # file_writer.write(file_buffer[:total_size - 16 - 32])
            file_writer.write(file_buffer[:total_size - 16 - 32 - 2])
        check_md5 = calculate_file_md5(saveFilePath + fileName)
        if check_md5 == recv_md5:
            client.send("SUCCESS".encode('utf-8'))
            return 0
        else:
            client.send("FAIL".encode('utf-8'))
            os.remove(os.path.join(saveFilePath, fileName))
            # return RobotError.ERR_OTHER
            return RobotError.ERR_DOWN_LOAD_FILE_FAILED

    """   
    @brief  Upload file
    @param  [in] fileType File type    0-lua file
    @param  [in] filePath Full path name of the file to upload    C://test/test.lua     
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def __FileUpLoad(self, fileType, filePath):

        if not os.path.exists(filePath):
            return RobotError.ERR_POINTTABLE_NOTFOUND

        MAX_UPLOAD_FILE_SIZE = 500 * 1024 * 1024;  # Maximum upload file size is 500Mb
        file_info = os.path.getsize(filePath)
        total_size = file_info + 46 + 4
        if total_size > MAX_UPLOAD_FILE_SIZE:
            print("Files larger than 500 MB are not supported!")
            return -1
        file_name = os.path.basename(filePath)
        rtn = self.robot.FileUpload(fileType, file_name)
        if rtn != 0:
            return rtn

        port = 20010

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(20)

        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        client.settimeout(20)

        send_md5 = calculate_file_md5(filePath)
        head_data = f"/f/b{total_size:10d}{send_md5}"
        num = client.send(head_data.encode('utf-8'))

        if num < 1:
            return RobotError.ERR_OTHER

        with open(filePath, "rb") as f:
            while True:
                data = f.read(2 * 1024 * 1024)
                if not data:  # If the end of the file is reached
                    end_data = "/b/f"
                    num = client.send(end_data.encode('utf-8'))  # Send the flag indicating file transfer complete
                    if num < 1:
                        return RobotError.ERR_OTHER
                    break  # Break out of the loop
                num = client.send(data)  # Send the read data to the client over the socket connection
                if num < 1:
                    return RobotError.ERR_OTHER
        time.sleep(0.5)
        result_buf = client.recv(1024)
        if result_buf[:7].decode('utf-8') == "SUCCESS":
            return RobotError.ERR_SUCCESS
        else:
            return RobotError.ERR_OTHER

    """   
    @brief  Delete file
    @param  [in] fileType File type    0-lua file
    @param  [in] fileName File name    "test.lua"
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def __FileDelete(self, fileType, fileName):
        rtn = self.robot.FileDelete(fileType, fileName)
        return rtn

    """   
    @brief  Download Lua file
    @param  [in] fileName Name of the lua file to download "test.lua"
    @param  [in] savePath Local path to save the file "D://Down/"
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LuaDownLoad(self, fileName, savePath):
        error = self.__FileDownLoad(0, fileName, savePath)
        return error

    """   
    @brief  Upload Lua file
    @param  [in] filePath Full path name of the file to upload   C://test/test.lua  
    @return Error code Success-0  Failure-error code
    """

    def LuaUpload(self, filePath):
        error = self.__FileUpLoad(0, filePath)
        if error == 0:
            file_name = os.path.basename(filePath)
            _error = self.robot.LuaUpLoadUpdate(file_name)
            tmp_error = _error[0]
            if tmp_error == 0:
                return tmp_error
            else:
                return tmp_error, _error[1]
        else:
            return error

    """   
    @brief  Delete Lua file
    @param  [in] fileName Name of the lua file to delete "test.lua"
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LuaDelete(self, fileName):
        error = self.__FileDelete(0, fileName)
        return error

    """   
    @brief  Get all current lua file names
    @return Error code Success-0  Failure-error code
    @return Return value (returned on successful call) lua_num Number of lua files
    @return Return value (returned on successful call) luaNames List of lua file names
    """

    @log_call
    @xmlrpc_timeout
    def GetLuaList(self):
        _error = self.robot.GetLuaList()
        # size = len(_error)
        error = _error[0]
        if _error[0] == 0:
            lua_num = _error[1]
            lua_name = _error[2].split(';')
            return error, lua_num, lua_name
        else:
            return error,None,None

    """   
    @brief  Set 485 extended axis parameters
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @param  [in] Required parameter int servoCompany Servo driver manufacturer, 1-Dynatec
    @param  [in] Required parameter int servoModel Servo driver model, 1-FD100-750C
    @param  [in] Required parameter int servoSoftVersion Servo driver software version, 1-V1.0
    @param  [in] Required parameter int servoResolution Encoder resolution
    @param  [in] Required parameter float axisMechTransRatio Mechanical transmission ratio  
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetParam(self, servoId, servoCompany, servoModel, servoSoftVersion, servoResolution,
                         axisMechTransRatio):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        servoCompany = int(servoCompany)
        servoModel = int(servoModel)
        servoSoftVersion = int(servoSoftVersion)
        servoResolution = int(servoResolution)
        axisMechTransRatio = float(axisMechTransRatio)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetParam(servoId, servoCompany, servoModel, servoSoftVersion, servoResolution,
                                                    axisMechTransRatio)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get 485 extended axis configuration parameters
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) int servoCompany Servo driver manufacturer, 1-Dynatec
    @return Return value (returned on successful call) servoModel Servo driver model, 1-FD100-750C 
    @return Return value (returned on successful call) servoSoftVersion Servo driver software version, 1-V1.0
    @return Return value (returned on successful call) int servoResolution Encoder resolution
    @return Return value (returned on successful call) float axisMechTransRatio Mechanical transmission ratio
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoGetParam(self, servoId):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        flag = True
        while flag:
            try:
                _error = self.robot.AuxServoGetParam(servoId)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5]
        else:
            return error,None,None,None,None,None

    """   
    @brief  Set 485 extended axis enable/disable
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @param  [in] Required parameter int status Enable state, 0-disable, 1-enable
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoEnable(self, servoId, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoEnable(servoId, status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set 485 extended axis control mode
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @param  [in] Required parameter mode Control mode, 0-position mode, 1-velocity mode
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetControlMode(self, servoId, mode):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        mode = int(mode)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetControlMode(servoId, mode)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set 485 extended axis target position (position mode)
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @param  [in] Required parameter float pos Target position, mm or °
    @param  [in] Required parameter float speed Target velocity, mm/s or °/s
    @param  [in] Required parameter acc Acceleration percentage [0-100] 
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetPos(self, servoId, pos, speed,acc=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        pos = float(pos)
        speed = float(speed)
        acc = float(acc)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetTargetPos(servoId, pos, speed,acc)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set 485 extended axis target velocity (velocity mode)
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @param  [in] Required parameter float speed Target velocity, mm/s or °/s
    @param  [in] Required parameter acc Acceleration percentage [0-100] 
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetSpeed(self, servoId, speed,acc):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        speed = float(speed)
        acc = float(acc)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetTargetSpeed(servoId, speed,acc)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set 485 extended axis target torque (torque mode)
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @param  [in] Required parameter float torque Target torque, Nm
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetTorque(self, servoId, torque):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        torque = float(torque)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetTargetTorque(servoId, torque)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set 485 extended axis homing
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @param  [in] Required parameter int mode Homing mode, 1-home at current position; 2-home at negative limit; 3-home at positive limit
    @param  [in] Required parameter float searchVel Homing velocity, mm/s or °/s
    @param  [in] Required parameter float latchVel Latch velocity, mm/s or °/s
    @param  [in] Required parameter acc Acceleration percentage [0-100] 
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoHoming(self, servoId, mode, searchVel, latchVel,acc=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        mode = int(mode)
        searchVel = float(searchVel)
        latchVel = float(latchVel)
        acc = float(acc)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoHoming(servoId, mode, searchVel, latchVel,acc)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Clear 485 extended axis error information
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoClearError(self, servoId):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoClearError(servoId)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get 485 extended axis servo status
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) servoErrCode Servo driver fault code
    @return Return value (returned on successful call) servoState Servo driver status bit0:0-not enabled; 1-enabled;  bit1:0-not moving; 1-moving; bit2:0-positive limit not triggered, 1-positive limit triggered; bit3:0-negative limit not triggered, 1-negative limit triggered;   bit4 0-positioning not complete; 1-positioning complete;  bit5:0-not homed; 1-homing complete
    @return Return value (returned on successful call) servoPos Servo current position mm or °
    @return Return value (returned on successful call) servoSpeed Servo current velocity mm/s or °/s
    @return Return value (returned on successful call) servoTorque Servo current torque Nm
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoGetStatus(self, servoId):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        flag = True
        while flag:
            try:
                _error = self.robot.AuxServoGetStatus(servoId)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5]
        else:
            return error,None,None,None,None,None

    """   
    @brief  Set 485 extended axis data axis number in status feedback
    @param  [in] Required parameter int servoId Servo driver ID, range [1-16], corresponds to slave ID 
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def AuxServosetStatusID(self, servoId):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetStatusID(servoId)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set robot peripheral protocol
    @param  [in] Required parameter int protocol Robot peripheral protocol number 4096-extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetExDevProtocol(self, protocol):
        while self.reconnect_flag:
            time.sleep(0.1)
        protocol = int(protocol)
        flag = True
        while flag:
            try:
                error = self.robot.SetExDevProtocol(protocol)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get robot peripheral protocol
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) int protocol Robot peripheral protocol number 4096-extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
    """

    @log_call
    @xmlrpc_timeout
    def GetExDevProtocol(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetExDevProtocol()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  Set robot acceleration
    @param [in]Required parameter acc Robot acceleration percentage
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetOaccScale(self, acc):
        while self.reconnect_flag:
            time.sleep(0.1)
        acc = float(acc)
        flag = True
        while flag:
            try:
                error = self.robot.SetOaccScale(acc)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Control box AO flying shot start
    @param [in]Required parameter AONum Control box AO number
    @param [in]Default parameter maxTCPSpeed Maximum TCP speed value [1-5000mm/s], default 1000
    @param [in]Default parameter maxAOPercent AO percentage corresponding to the maximum TCP speed value, default 100%
    @param [in]Required parameter zeroZoneCmp Dead zone compensation value AO percentage, integer, default 20%, range [0-100]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveAOStart(self, AONum, maxTCPSpeed=1000, maxAOPercent=100, zeroZoneCmp=20):
        while self.reconnect_flag:
            time.sleep(0.1)
        AONum = int(AONum)
        maxTCPSpeed = int(maxTCPSpeed)
        maxAOPercent = int(maxAOPercent)
        zeroZoneCmp = int(zeroZoneCmp)
        flag = True
        while flag:
            try:
                error = self.robot.MoveAOStart(AONum, maxTCPSpeed, maxAOPercent, zeroZoneCmp)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Control box AO flying shot stop
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveAOStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MoveAOStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  End AO flying shot start
    @param [in]Required parameter AONum End AO number
    @param [in]Required parameter maxTCPSpeed Maximum TCP speed value [1-5000mm/s], default 1000
    @param [in]Required parameter maxAOPercent AO percentage corresponding to the maximum TCP speed value, default 100%
    @param [in]Required parameter zeroZoneCmp Dead zone compensation value AO percentage, integer, default 20%, range [0-100]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveToolAOStart(self, AONum, maxTCPSpeed=1000, maxAOPercent=100, zeroZoneCmp=20):
        while self.reconnect_flag:
            time.sleep(0.1)
        AONum = int(AONum)
        maxTCPSpeed = int(maxTCPSpeed)
        maxAOPercent = int(maxAOPercent)
        zeroZoneCmp = int(zeroZoneCmp)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToolAOStart(AONum, maxTCPSpeed, maxAOPercent, zeroZoneCmp)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  End AO flying shot stop
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveToolAOStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToolAOStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis communication parameter configuration
    @param [in]Required parameter ip PLC IP address
    @param [in]Required parameter port	Port number
    @param [in]Required parameter period Communication cycle (ms, not yet available)
    @param [in]Required parameter lossPkgTime	Packet loss detection time (ms)
    @param [in]Required parameter lossPkgNum	Packet loss count
    @param [in]Required parameter disconnectTime	Communication disconnection confirmation duration
    @param [in]Required parameter reconnectEnable	Communication disconnection auto-reconnect enable 0-disable 1-enable
    @param [in]Required parameter reconnectPeriod	Reconnection cycle interval (ms)
    @param [in]Required parameter reconnectNum	Reconnection count
    @param [in]Required parameter selfConnect Whether to automatically establish connection on power-off restart; 0-do not establish connection; 1-establish connection
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevSetUDPComParam(self, ip, port, period, lossPkgTime, lossPkgNum, disconnectTime,
                             reconnectEnable, reconnectPeriod, reconnectNum,selfConnect):
        while self.reconnect_flag:
            time.sleep(0.1)
        ip = str(ip)
        port = int(port)
        period = int(period)
        period = 2  # Not yet available, must be 2
        lossPkgTime = int(lossPkgTime)
        lossPkgNum = int(lossPkgNum)
        disconnectTime = int(disconnectTime)
        reconnectEnable = int(reconnectEnable)
        reconnectPeriod = int(reconnectPeriod)
        reconnectNum = int(reconnectNum)
        selfConnect = int(selfConnect)

        flag = True
        while flag:
            try:
                error = self.robot.ExtDevSetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime,
                                                        reconnectEnable, reconnectPeriod, reconnectNum,selfConnect)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get UDP extended axis communication parameters
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) ip PLC IP address
    @return Return value (returned on successful call) port	Port number
    @return Return value (returned on successful call) period Communication cycle (ms, not yet available)
    @return Return value (returned on successful call) lossPkgTime	Packet loss detection time (ms)
    @return Return value (returned on successful call) lossPkgNum	Packet loss count
    @return Return value (returned on successful call) disconnectTime	Communication disconnection confirmation duration
    @return Return value (returned on successful call) reconnectEnable	Communication disconnection auto-reconnect enable 0-disable 1-enable
    @return Return value (returned on successful call) reconnectPeriod	Reconnection cycle interval (ms)
    @return Return value (returned on successful call) reconnectNum	Reconnection count
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevGetUDPComParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ExtDevGetUDPComParam()
                flag = False
            except socket.error as e:
                flag = True

        if _error[0] == 0:
            return _error[0], [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8],
                               _error[9]]
        else:
            return _error[0],None

    """   
    @brief  Load UDP communication
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevLoadUDPDriver(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtDevLoadUDPDriver()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Unload UDP communication
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevUnloadUDPDriver(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtDevUnloadUDPDriver()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Restore connection after UDP extended axis communication abnormal disconnection
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevUDPClientComReset(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtDevUDPClientComReset()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Close communication after UDP extended axis communication abnormal disconnection
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevUDPClientComClose(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtDevUDPClientComClose()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set extended robot position relative to extended axis
    @param [in]Required parameter installType 0-robot mounted on external axis, 1-robot mounted off the external axis
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotPosToAxis(self, installType):
        while self.reconnect_flag:
            time.sleep(0.1)
        installType = int(installType)
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotPosToAxis(installType)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set extended axis system DH parameter configuration
    @param [in]Required parameter axisConfig External axis configuration, 0-single DOF linear slide rail, 1-two DOF L-type positioner, 2-three DOF, 3-four DOF, 4-single DOF positioner
    @param [in]Required parameter  axisDHd1 External axis DH parameter d1 mm
    @param [in]Required parameter  axisDHd2 External axis DH parameter d2 mm
    @param [in]Required parameter  axisDHd3 External axis DH parameter d3 mm
    @param [in]Required parameter  axisDHd4 External axis DH parameter d4 mm
    @param [in]Required parameter  axisDHa1 External axis DH parameter a1 mm
    @param [in]Required parameter  axisDHa2 External axis DH parameter a2 mm
    @param [in]Required parameter  axisDHa3 External axis DH parameter a3 mm
    @param [in]Required parameter  axisDHa4 External axis DH parameter a4 mm
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAxisDHParaConfig(self, axisConfig, axisDHd1, axisDHd2, axisDHd3, axisDHd4, axisDHa1, axisDHa2, axisDHa3,
                            axisDHa4):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisConfig = int(axisConfig)
        axisDHd1 = float(axisDHd1)
        axisDHd2 = float(axisDHd2)
        axisDHd3 = float(axisDHd3)
        axisDHd4 = float(axisDHd4)
        axisDHa1 = float(axisDHa1)
        axisDHa2 = float(axisDHa2)
        axisDHa3 = float(axisDHa3)
        axisDHa4 = float(axisDHa4)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxisDHParaConfig(axisConfig, axisDHd1, axisDHd2, axisDHd3, axisDHd4, axisDHa1, axisDHa2,
                                                       axisDHa3, axisDHa4)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis parameter configuration
    @param [in]Required parameter axisId Axis number [1-4]
    @param [in]Required parameter axisType Extended axis type 0-translation; 1-rotation
    @param [in]Required parameter axisDirection Extended axis direction 0-forward; 1-reverse
    @param [in]Required parameter axisMax Extended axis maximum position mm
    @param [in]Required parameter axisMin Extended axis minimum position mm
    @param [in]Required parameter axisVel Velocity mm/s
    @param [in]Required parameter axisAcc Acceleration mm/s2
    @param [in]Required parameter axisLead Lead mm
    @param [in]Required parameter encResolution Encoder resolution
    @param [in]Required parameter axisOffect Weld seam start point extended axis offset
    @param [in]Required parameter axisCompany Driver manufacturer 1-Hechuan; 2-Inovance; 3-Panasonic
    @param [in]Required parameter axisModel Driver model 1-Hechuan-SV-XD3EA040L-E, 2-Hechuan-SV-X2EA150A-A, 1-Inovance-SV620PT5R4I, 1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG
    @param [in]Required parameter axisEncType Encoder type  0-incremental; 1-absolute
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisParamConfig(self, axisId, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc, axisLead,
                           encResolution, axisOffect, axisCompany, axisModel, axisEncType):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisId = int(axisId)
        axisType = int(axisType)
        axisDirection = int(axisDirection)
        axisMax = float(axisMax)
        axisMin = float(axisMin)
        axisVel = float(axisVel)
        axisAcc = float(axisAcc)
        axisLead = float(axisLead)
        encResolution = int(encResolution)
        axisOffect = float(axisOffect)
        axisCompany = int(axisCompany)
        axisModel = int(axisModel)
        axisEncType = int(axisEncType)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisParamConfig(axisId, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc,
                                                      axisLead, encResolution, axisOffect, axisCompany, axisModel, axisEncType)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get extended axis driver configuration information
    @param [in]Required parameter axisId Axis number [1-4]
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) axisCompany Driver manufacturer 1-Hechuan; 2-Inovance; 3-Panasonic
    @return Return value (returned on successful call) axisModel Driver model 1-Hechuan-SV-XD3EA040L-E, 2-Hechuan-SV-X2EA150A-A, 1-Inovance-SV620PT5R4I, 1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG
    @return Return value (returned on successful call) axisEncType Encoder type  0-incremental; 1-absolute
    """

    @log_call
    @xmlrpc_timeout
    def GetExAxisDriverConfig(self, axisId):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisId = int(axisId)
        flag = True
        while flag:
            try:
                error = self.robot.GetExAxisDriverConfig(axisId)
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], [error[1], error[2], error[3]]
        else:
            return error

    """   
    @brief  Set extended axis coordinate frame reference point - four-point method
    @param [in]Required parameter pointNum Point number [1-4]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSetRefPoint(self, pointNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointNum = int(pointNum)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisSetRefPoint(pointNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Calculate extended axis coordinate frame - four-point method
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) coord Coordinate frame value [x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisComputeECoordSys(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisComputeECoordSys()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set calibration reference point pose in the positioner end coordinate frame
    @param [in]Required parameter pos Pose value [x,y,z,rx,ry,rz]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetRefPointInExAxisEnd(self, pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        pos = list(map(float, pos))
        flag = True
        while flag:
            try:
                error = self.robot.SetRefPointInExAxisEnd(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Positioner coordinate frame reference point setting
    @param [in]Required parameter pointNum Point number [1-4]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def PositionorSetRefPoint(self, pointNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointNum = int(pointNum)
        flag = True
        while flag:
            try:
                error = self.robot.PositionorSetRefPoint(pointNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Positioner coordinate frame calculation - four-point method
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) coord Coordinate frame value [x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def PositionorComputeECoordSys(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.PositionorComputeECoordSys()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  Apply extended axis coordinate frame
    @param [in] Required parameter axisCoordNum coordinate frame number
    @param [in] Required parameter toolNum tool number
    @param [in] Required parameter coord coordinate frame value [x,y,z,rx,ry,rz]
    @param [in] Required parameter calibFlag calibration flag 0-no, 1-yes    
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisActiveECoordSys(self, axisCoordNum, toolNum, coord, calibFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisCoordNum = int(axisCoordNum)
        toolNum = int(toolNum)
        coord = list(map(float, coord))
        calibFlag = int(calibFlag)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisActiveECoordSys(axisCoordNum, toolNum, coord[0], coord[1], coord[2], coord[3],
                                                          coord[4], coord[5], calibFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis enable
    @param [in] Required parameter axisID axis number [1-4]
    @param [in] Required parameter status 0-disable; 1-enable
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisServoOn(self, axisID, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisID = int(axisID)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisServoOn(axisID, status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis homing
    @param [in] Required parameter axisID axis number [1-4]
    @param [in] Required parameter mode homing method 0-home at current position, 1-home at negative limit, 2-home at positive limit
    @param [in] Required parameter searchVel homing search velocity (mm/s)
    @param [in] Required parameter latchVel homing latch velocity (mm/s)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSetHoming(self, axisID, mode, searchVel, latchVel):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisID = int(axisID)
        mode = int(mode)
        searchVel = float(searchVel)
        latchVel = float(latchVel)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisSetHoming(axisID, mode, searchVel, latchVel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis jog start
    @param [in] Required parameter axisID axis number [1-4]
    @param [in] Required parameter direction rotation direction 0-reverse; 1-forward
    @param [in] Required parameter vel velocity (mm/s)
    @param [in] Required parameter acc (acceleration mm/s2)
    @param [in] Required parameter maxDistance maximum jog distance
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisStartJog(self, axisID, direction, vel, acc, maxDistance):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        axisID = int(axisID)
        direction = int(direction)
        vel = float(vel)
        acc = float(acc)
        maxDistance = float(maxDistance)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisStartJog(6, axisID, direction, vel, acc, maxDistance)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis jog stop
    @param [in] Required parameter axisID axis number [1-4]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisStopJog(self, axisID):
        axisID = int(axisID)
        error =self.send_message("/f/bIII19III240III14IIIStopExtAxisJogIII/b/f")
        # error = self.robot.ExtAxisStartJog(7, axisID, 0, 0.0, 0.0, 0.0)
        return error

    """   
    @brief  Set extended DO
    @param [in] Required parameter DONum DO number
    @param [in] Required parameter bOpen switch True-on, False-off
    @param [in] Required parameter smooth whether smooth True-yes, False-no
    @param [in] Required parameter block whether blocking True-yes, False-no
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAuxDO(self, DONum, bOpen, smooth, block):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        bOpen = bool(bOpen)
        smooth = bool(smooth)
        block = bool(block)
        open_flag = 1 if bOpen else 0
        smooth_flag = 1 if smooth else 0
        no_block_flag = 1 if block else 0
        print("open_flag",open_flag)
        print("smooth_flag", smooth_flag)
        print("no_block_flag", no_block_flag)
        flag = True
        while flag:
            try:
                error = self.robot.SetAuxDO(DONum, open_flag, smooth_flag, no_block_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set extended AO
    @param [in] Required parameter AONum AO number 
    @param [in] Required parameter value analog value [0-4095]
    @param [in] Required parameter block whether blocking True-yes, False-no
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAuxAO(self, AONum, value, block):
        while self.reconnect_flag:
            time.sleep(0.1)
        AONum = int(AONum)
        value = float(value)
        block = bool(block)
        no_block_flag = 0 if block else 1
        value =value
        flag = True
        while flag:
            try:
                error = self.robot.SetAuxAO(AONum, value, no_block_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set extended DI input filter time
    @param [in] Required parameter filterTime filter time (ms)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAuxDIFilterTime(self, filterTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        filterTime = int(filterTime)
        flag = True
        while flag:
            try:
                error = self.robot.SetAuxDIFilterTime(filterTime)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set extended AI input filter time
    @param [in] Required parameter AINum AI number
    @param [in] Required parameter filterTime filter time (ms)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAuxAIFilterTime(self, AINum,filterTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        AINum = int(AINum)
        filterTime = int(filterTime)
        flag = True
        while flag:
            try:
                error = self.robot.SetAuxAIFilterTime(AINum,filterTime)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Wait for extended DI input
    @param [in] Required parameter DINum DI number
    @param [in] Required parameter bOpen switch True-on, False-off
    @param [in] Required parameter time maximum wait time (ms)
    @param [in] Required parameter errorAlarm whether to continue motion True-yes, False-no
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitAuxDI(self, DINum, bOpen, time, errorAlarm):
        while self.reconnect_flag:
            time.sleep(0.1)
        DINum = int(DINum)
        bOpen = bool(bOpen)
        open_flag = 0 if bOpen else 1
        time = int(time)
        errorAlarm = bool(errorAlarm)
        errorAlarm_flag = 0 if errorAlarm else 1
        flag = True
        while flag:
            try:
                error = self.robot.WaitAuxDI(DINum, open_flag, time, errorAlarm_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Wait for extended AI input
    @param [in] Required parameter AINum AI number
    @param [in] Required parameter sign 0-greater than; 1-less than
    @param [in] Required parameter value AI value
    @param [in] Required parameter time maximum wait time (ms)
    @param [in] Required parameter errorAlarm whether to continue motion True-yes, False-no
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitAuxAI(self, AINum, sign, value, time, errorAlarm):
        while self.reconnect_flag:
            time.sleep(0.1)
        AINum = int(AINum)
        sign = int(sign)
        value = int(value)
        time = int(time)
        errorAlarm = bool(errorAlarm)
        errorAlarm_flag = 0 if errorAlarm else 1
        flag = True
        while flag:
            try:
                error = self.robot.WaitAuxAI(AINum, sign, value, time, errorAlarm_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get extended DI value
    @param [in] Required parameter DINum DI number
    @param [in] Required parameter isNoBlock whether blocking True-blocking false-non-blocking
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) isOpen 0-off; 1-on
    """

    @log_call
    @xmlrpc_timeout
    def GetAuxDI(self, DINum, isNoBlock):
        while self.reconnect_flag:
            time.sleep(0.1)
        DINum = int(DINum)
        isNoBlock = bool(isNoBlock)
        isNoBlock_flag = 0 if isNoBlock else 1
        flag = True
        while flag:
            try:
                error = self.robot.GetAuxDI(DINum, isNoBlock_flag)
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1]
        else:
            return error

    """   
    @brief  Get extended AI value
    @param [in] Required parameter AINum AI number
    @param [in] Required parameter isNoBlock whether blocking True-blocking False-non-blocking
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) value input value
    """

    @log_call
    @xmlrpc_timeout
    def GetAuxAI(self, AINum, isNoBlock):
        while self.reconnect_flag:
            time.sleep(0.1)
        AINum = int(AINum)
        isNoBlock = bool(isNoBlock)
        isNoBlock_flag = 0 if isNoBlock else 1
        flag = True
        while flag:
            try:
                error = self.robot.GetAuxAI(AINum, isNoBlock_flag)
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1]
        else:
            return error

    """   
    @brief  UDP extended axis motion
    @param [in] Required parameter pos target position axis 1 position ~ axis 4 position [exaxis[0],exaxis[1],exaxis[2],exaxis[3]]
    @param [in] Required parameter ovl velocity percentage
    @param [in] Required parameter blend blend parameter (mm or ms), -1 wait for motion to complete
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisMove(self, pos, ovl, blend=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        pos = list(map(float, pos))
        ovl = float(ovl)
        blend = float(blend)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisMoveJ(0, pos[0], pos[1], pos[2], pos[3], ovl, blend)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis synchronized motion with robot joint motion (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter joint_pos: target joint position, unit [deg]
    @param  [in] Required parameter desc_pos: target Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Required parameter exaxis_pos: external axis 1 position ~ external axis 4 position 
    @param  [in] Default parameter vel: velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc: acceleration percentage, [0~100] not open yet, default 0.0 
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0    
    @param  [in] Default parameter blendT:[-1.0]-move to position (blocking), [0~500.0]-blend time (non-blocking), unit [ms] default -1.0
    @param  [in] Default parameter offset_flag:[0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos: pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSyncMoveJ(self, joint_pos, tool, user, exaxis_pos, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
                         blendT=-1.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        joint_pos = list(map(float, joint_pos))
        tool = int(tool)
        user = int(user)
        desc_pos = list(map(float, desc_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        exaxis_pos = list(map(float, exaxis_pos))
        blendT = float(blendT)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        if (desc_pos[0] == 0.0) and (desc_pos[1] == 0.0) and (desc_pos[2] == 0.0) and (desc_pos[3] == 0.0) and (
                desc_pos[4] == 0.0) and (desc_pos[5] == 0.0):  # If no parameter is entered, call forward kinematics to solve
            ret = self.robot.GetForwardKin(joint_pos)  # Forward kinematics solution
            if ret[0] == 0:
                desc_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos[0], exaxis_pos[1], exaxis_pos[2], exaxis_pos[3], ovl, blendT)
        if error != 0:
            return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveJ(joint_pos, desc_pos, tool, user, vel, acc, ovl, exaxis_pos, blendT, offset_flag,
                                         offset_pos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis synchronized motion with robot linear motion (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter joint_pos: target joint position, unit [deg] 
    @param  [in] Required parameter desc_pos: target Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool: tool number, [0~14]
    @param  [in] Required parameter user: work object number, [0~14]
    @param  [in] Required parameter exaxis_pos: external axis 1 position ~ external axis 4 position 
    @param  [in] Default parameter vel: velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc: acceleration percentage, [0~100] not open yet default 0.0
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter blendR: [-1.0]-move to position (blocking), [0~1000]-blend radius (non-blocking), unit [mm] default -1.0    
    @param  [in] Default parameter search:[0]-no wire searching, [1]-wire searching
    @param  [in] Default parameter offset_flag:[0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos: pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter config inverse solution joint space configuration, [-1]-solve with reference to the current joint position, [0~7]-solve based on a specific joint space configuration, default -1
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSyncMoveL(self, desc_pos, tool, user, exaxis_pos, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
                         blendR=-1.0, search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        exaxis_pos = list(map(float, exaxis_pos))
        search = int(search)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        config = int(config)

        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # If no parameter is entered, call inverse kinematics to solve
            ret = self.robot.GetInverseKin(0, desc_pos, config)  # Inverse kinematics solution
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos[0], exaxis_pos[1], exaxis_pos[2], exaxis_pos[3], ovl, blendR)
        if error != 0:
            return error
        flag = True
        while flag:
            try:
                error = self.MoveL(desc_pos=desc_pos,tool= tool,user= user,vel= vel, acc=acc,ovl= ovl,blendR= blendR,blendMode=0,exaxis_pos= exaxis_pos,search= search,
                                         offset_flag=offset_flag,offset_pos= offset_pos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP extended axis synchronized motion with robot arc motion (automatic forward/inverse kinematics calculation)
    @param  [in] Required parameter joint_pos_p: waypoint joint position, unit [deg] 
    @param  [in] Required parameter desc_pos_p: waypoint Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool_p: waypoint tool number, [0~14]
    @param  [in] Required parameter user_p: waypoint work object number, [0~14]
    @param  [in] Required parameter exaxis_pos_p: waypoint external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    @param  [in] Required parameter joint_pos_t: target point joint position, unit [deg] 
    @param  [in] Required parameter desc_pos_t: target point Cartesian pose, unit [mm][deg]
    @param  [in] Required parameter tool_t: tool number, [0~14]
    @param  [in] Required parameter user_t: work object number, [0~14]
    @param  [in] Required parameter exaxis_pos_t: target point external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]    
    @param  [in] Default parameter vel_p: waypoint velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc_p: waypoint acceleration percentage, [0~100] not yet open, default 0.0    
    @param  [in] Default parameter offset_flag_p: whether the waypoint is offset [0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos_p: waypoint pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter vel_t: target point velocity percentage, [0~100] default 20.0
    @param  [in] Default parameter acc_t: target point acceleration percentage, [0~100] not open yet default 0.0
    @param  [in] Default parameter offset_flag_t: whether the target point is offset [0]-no offset, [1]-offset in work object/base coordinate frame, [2]-offset in tool coordinate frame default 0
    @param  [in] Default parameter offset_pos_t: target point pose offset, unit [mm][deg] default [0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] Default parameter ovl: velocity scaling factor, [0~100] default 100.0
    @param  [in] Default parameter blendR:[-1.0]-move to position (blocking), [0~1000]-blend radius (non-blocking), unit [mm] default -1.0
    @param  [in] Default parameter config inverse solution joint space configuration, [-1]-solve with reference to the current joint position, [0~7]-solve based on a specific joint space configuration, default -1
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSyncMoveC(self, desc_pos_p, tool_p, user_p, exaxis_pos_p, desc_pos_t, tool_t,
                         user_t, exaxis_pos_t,
                         joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         vel_p=20.0, acc_p=100.0, offset_flag_p=0,
                         offset_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         vel_t=20.0, acc_t=100.0, offset_flag_t=0,
                         offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         ovl=100.0, blendR=-1.0,config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos_p = list(map(float, desc_pos_p))
        tool_p = float(int(tool_p))
        user_p = float(int(user_p))
        joint_pos_p = list(map(float, joint_pos_p))
        vel_p = float(vel_p)
        acc_p = float(acc_p)
        exaxis_pos_p = list(map(float, exaxis_pos_p))
        offset_flag_p = int(offset_flag_p)
        offset_pos_p = list(map(float, offset_pos_p))

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = float(int(tool_t))
        user_t = float(int(user_t))
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))
        offset_flag_t = int(offset_flag_t)
        offset_pos_t = list(map(float, offset_pos_t))

        ovl = float(ovl)
        blendR = float(blendR)
        config = int(config)

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # If no parameter is entered, call inverse kinematics to solve
            retp = self.robot.GetInverseKin(0, desc_pos_p, config)  # Inverse kinematics solution
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # If no parameter is input, call inverse kinematics solution
            rett = self.robot.GetInverseKin(0, desc_pos_t, config)  # Inverse kinematics solution
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos_t[0], exaxis_pos_t[1], exaxis_pos_t[2], exaxis_pos_t[3], ovl, blendR)
        if error != 0:
            return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveC(joint_pos_p, desc_pos_p, [tool_p, user_p, vel_p, acc_p], exaxis_pos_p, offset_flag_p,
                                         offset_pos_p, joint_pos_t, desc_pos_t, [tool_t, user_t, vel_t, acc_t], exaxis_pos_t,
                                         offset_flag_t, offset_pos_t, ovl, blendR)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Wire seam search start
    @param [in] Required parameter refPos  1-reference point 2-contact point
    @param [in] Required parameter searchVel   search velocity %
    @param [in] Required parameter searchDis  search distance mm
    @param [in] Required parameter autoBackFlag automatic return flag, 0-no auto; -auto
    @param [in] Required parameter autoBackVel  automatic return velocity %
    @param [in] Required parameter autoBackDis  automatic return distance mm
    @param [in] Required parameter offectFlag  1-search with offset; 2-taught point search
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WireSearchStart(self, refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        refPos = int(refPos)
        searchVel = float(searchVel)
        searchDis = int(searchDis)
        autoBackFlag = int(autoBackFlag)
        autoBackVel = float(autoBackVel)
        autoBackDis = int(autoBackDis)
        offectFlag = int(offectFlag)
        flag = True
        while flag:
            try:
                error = self.robot.WireSearchStart(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Wire seam search end
    @param [in] Required parameter refPos  1-reference point 2-contact point
    @param [in] Required parameter searchVel   search velocity %
    @param [in] Required parameter searchDis  search distance mm
    @param [in] Required parameter autoBackFlag automatic return flag, 0-no auto; -auto
    @param [in] Required parameter autoBackVel  automatic return velocity %
    @param [in] Required parameter autoBackDis  automatic return distance mm
    @param [in] Required parameter offectFlag  1-search with offset; 2-taught point search
    @return error code success- 0, failure-error code
    """
    @log_call
    @xmlrpc_timeout
    def WireSearchEnd(self, refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        refPos = int(refPos)
        searchVel = float(searchVel)
        searchDis = int(searchDis)
        autoBackFlag = int(autoBackFlag)
        autoBackVel = float(autoBackVel)
        autoBackDis = int(autoBackDis)
        offectFlag = int(offectFlag)
        flag = True
        while flag:
            try:
                error = self.robot.WireSearchEnd(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Calculate wire seam search offset
    @param  [in] Required parameter seamType  weld seam type
    @param  [in] Required parameter method   calculation method
    @param  [in] Required parameter varNameRef reference points 1-6, "#" indicates no point variable
    @param  [in] Required parameter varNameRes contact points 1-6, "#" indicates no point variable
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) offectFlag 0-offset directly added to command point; 1-offset requires coordinate transformation of command point
    @return Return value (returned on successful call) offect offset pose [x, y, z, a, b, c]
    """
    @log_call
    @xmlrpc_timeout
    def GetWireSearchOffset(self, seamType, method,varNameRef,varNameRes):
        while self.reconnect_flag:
            time.sleep(0.1)
        seamType = int(seamType)
        method = int(method)
        if(len(varNameRes)!=6):
            return 4
        if(len(varNameRes)!=6):
            return 4
        varNameRef = list(map(str, varNameRef))
        varNameRes = list(map(str, varNameRes))

        flag = True
        while flag:
            try:
                _error = self.robot.GetWireSearchOffset(seamType, method, varNameRef[0], varNameRef[1], varNameRef[2], varNameRef[3], varNameRef[4], varNameRef[5],
                                                        varNameRes[0], varNameRes[1], varNameRes[2], varNameRes[3], varNameRes[4], varNameRes[5])
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], [_error[2], _error[3], _error[4], _error[5], _error[6], _error[7]]
        else:
            return error,None,None

    """   
    @brief  Wait for wire seam search completion
    @param  [in] Required parameter varName  contact point name "RES0" ~ "RES99"
    @return error code success- 0, failure-error code
    """
    @log_call
    @xmlrpc_timeout
    def WireSearchWait(self,varname):
        while self.reconnect_flag:
            time.sleep(0.1)
        varname=str(varname)
        flag = True
        while flag:
            try:
                error = self.robot.WireSearchWait(varname)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Write wire seam search contact point to database
    @param  [in] Required parameter varName  contact point name "RES0" ~ "RES99"
    @param  [in] Required parameter pos  contact point data [x, y, x, a, b, c]
    @return error code success- 0, failure-error code
    """
    @log_call
    @xmlrpc_timeout
    def SetPointToDatabase(self,varName,pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        varName = str(varName)
        pos = list(map(float,pos))

        flag = True
        while flag:
            try:
                error = self.robot.SetPointToDatabase(varName,pos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Arc tracking control
    @param  [in] Required parameter flag switch, 0-off; 1-on
    @param  [in] Required parameter delayTime lag time, unit ms
    @param  [in] Required parameter isLeftRight left-right deviation compensation 0-disable, 1-enable
    @param  [in] Required parameter klr left-right adjustment coefficient (sensitivity)
    @param  [in] Required parameter tStartLr left-right compensation start time cyc
    @param  [in] Required parameter stepMaxLr left-right maximum compensation per step mm
    @param  [in] Required parameter sumMaxLr left-right total maximum compensation mm
    @param  [in] Required parameter isUpLow up-down deviation compensation 0-disable, 1-enable
    @param  [in] Required parameter kud up-down adjustment coefficient (sensitivity)
    @param  [in] Required parameter tStartUd up-down compensation start time cyc
    @param  [in] Required parameter stepMaxUd up-down maximum compensation per step mm
    @param  [in] Required parameter sumMaxUd up-down total maximum compensation
    @param  [in] Required parameter axisSelect up-down coordinate frame selection, 0-swing; 1-tool; 2-base
    @param  [in] Required parameter referenceType up-down reference current setting method, 0-feedback; 1-constant
    @param  [in] Required parameter referSampleStartUd up-down reference current sampling start count (feedback), cyc
    @param  [in] Required parameter referSampleCountUd up-down reference current sampling loop count (feedback), cyc
    @param  [in] Required parameter referenceCurrent up-down reference current mA
    @param  [in] Required parameter offsetType offset tracking type, 0-no offset; 1-sampling; 2-percentage
    @param  [in] Required parameter offsetParameter offset parameter; sampling (offset sampling start time, samples one cycle by default); percentage (offset percentage (-100 ~ 100))
    @return error code success- 0, failure-error code
    """
    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceControl(self,flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd,
                            sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent, offsetType, offsetParameter):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        delaytime = float(delaytime)
        isLeftRight = int(isLeftRight)
        klr = float(klr)
        tStartLr = float(tStartLr)
        stepMaxLr = float(stepMaxLr)
        sumMaxLr = float(sumMaxLr)
        isUpLow = int(isUpLow)
        kud = float(kud)
        tStartUd = float(tStartUd)
        stepMaxUd = float(stepMaxUd)
        sumMaxUd = float(sumMaxUd)
        axisSelect = int(axisSelect)
        referenceType = int(referenceType)
        referSampleStartUd = float(referSampleStartUd)
        referSampleCountUd = float(referSampleCountUd)
        referenceCurrent = float(referenceCurrent)
        offsetType = int(offsetType)
        offsetParameter = int(offsetParameter)

        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.ArcWeldTraceControl(flag,delaytime, isLeftRight, [klr, tStartLr, stepMaxLr, sumMaxLr], isUpLow, [kud, tStartUd, stepMaxUd,
                                                       sumMaxUd], axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent, offsetType, offsetParameter)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  Arc tracking AI channel selection
    @param  [in] Required parameter channel arc tracking AI channel selection, [0-3]
    @return error code success- 0, failure-error code
    """
    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceExtAIChannelConfig(self,channel):
        while self.reconnect_flag:
            time.sleep(0.1)
        channel = int(channel)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceExtAIChannelConfig(channel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Force sensor assisted drag
    @param  [in] Required parameter status control state, 0-off; 1-on
    @param  [in] Required parameter asaptiveFlag adaptive enable flag, 0-off; 1-on
    @param  [in] Required parameter interfereDragFlag interference zone drag flag, 0-off; 1-on
    @param  [in] Required parameter ingularityConstraintsFlag singularity strategy, 0-avoid; 1-pass through
    @param  [in] Required parameter forceCollisionFlag robot collision detection flag during assisted drag; 0-off; 1-on
    @param  [in] Required parameter M=[m1,m2,m3,m4,m5,m6] inertia coefficients 
    @param  [in] Required parameter B=[b1,b2,b3,b4,b5,b6] damping coefficients
    @param  [in] Required parameter K=[k1,k2,k3,k4,k5,k6] stiffness coefficients
    @param  [in] Required parameter F=[f1,f2,f3,f4,f5,f6] drag six-dimensional force thresholds
    @param  [in] Required parameter Fmax maximum drag force limit
    @param  [in] Required parameter Vmax maximum joint velocity limit
    @return error code success- 0, failure-error code
    """
    @log_call
    @xmlrpc_timeout
    def EndForceDragControl(self, status, asaptiveFlag, interfereDragFlag, ingularityConstraintsFlag, forceCollisionFlag, M, B, K, F, Fmax, Vmax):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        asaptiveFlag = int(asaptiveFlag)
        interfereDragFlag = int(interfereDragFlag)
        ingularityConstraintsFlag = int(ingularityConstraintsFlag)
        M = list(map(float,M))
        B = list(map(float,B))
        K = list(map(float,K))
        F = list(map(float,F))
        Fmax = float(Fmax)
        Vmax = float(Vmax)
        forceCollisionFlag = int(forceCollisionFlag)
        flag = True
        while flag:
            try:
                error = self.robot.EndForceDragControl(status, asaptiveFlag, interfereDragFlag, ingularityConstraintsFlag,forceCollisionFlag, M, B, K, F, Fmax, Vmax)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Force sensor auto-enable after alarm clear
    @param  [in] Required parameter status control state, 0-off; 1-on
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetForceSensorDragAutoFlag(self, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.SetForceSensorDragAutoFlag(status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set six-dimensional force and joint impedance hybrid drag switch and parameters
    @param  [in] Required parameter status control state, 0-off; 1-on
    @param  [in] Required parameter impedanceFlag impedance enable flag, 0-off; 1-on
    @param  [in] Required parameter lamdeDain drag gain
    @param  [in] Required parameter KGain stiffness gain
    @param  [in] Required parameter BGain damping gain
    @param  [in] Required parameter dragMaxTcpVel maximum end-effector linear velocity limit during drag
    @param  [in] Required parameter dragMaxTcpOriVel maximum end-effector angular velocity limit during drag
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ForceAndJointImpedanceStartStop(self,status, impedanceFlag, lamdeDain, KGain, BGain,dragMaxTcpVel,dragMaxTcpOriVel):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        impedanceFlag = int(impedanceFlag)
        if((len(lamdeDain)!=6)or(len(KGain)!=6)or(len(BGain)!=6)):
            return 4
        lamdeDain = list(map(float,lamdeDain))
        KGain = list(map(float,KGain))
        BGain = list(map(float,BGain))
        dragMaxTcpVel = float(dragMaxTcpVel)
        dragMaxTcpOriVel = float(dragMaxTcpOriVel)
        flag = True
        while flag:
            try:
                error = self.robot.ForceAndJointImpedanceStartStop(status, impedanceFlag, lamdeDain, KGain, BGain,dragMaxTcpVel,dragMaxTcpOriVel)
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  Get force sensor drag switch status
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) dragState force sensor assisted drag control state, 0-off; 1-on
    @return Return value (returned on successful call) sixDimensionalDragState six-dimensional force assisted drag control state, 0-off; 1-on
    """

    @log_call
    @xmlrpc_timeout
    def GetForceAndTorqueDragState(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetForceAndTorqueDragState()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2]
        else:
            return error,None,None

    """   
    @brief  Set load weight under force sensor
    @param  [in] Required parameter weight load weight kg
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetForceSensorPayload(self,weight):
        while self.reconnect_flag:
            time.sleep(0.1)
        weight = float(weight)
        flag = True
        while flag:
            try:
                error = self.robot.SetForceSensorPayload(weight)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set load weight under force sensor
    @param  [in] Required parameter x load center of mass x mm 
    @param  [in] Required parameter y load center of mass y mm 
    @param  [in] Required parameter z load center of mass z mm 
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetForceSensorPayloadCog(self,x,y,z):
        while self.reconnect_flag:
            time.sleep(0.1)
        x = float(x)
        y = float(y)
        z = float(z)
        flag = True
        while flag:
            try:
                error = self.robot.SetForceSensorPayloadCog(x,y,z)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get load weight under force sensor
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) weight load weight kg
    """

    @log_call
    @xmlrpc_timeout
    def GetForceSensorPayload(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetForceSensorPayload()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None


    """   
    @brief  Get load center of mass under force sensor
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) x load center of mass x mm 
    @return Return value (returned on successful call) y load center of mass y mm 
    @return Return value (returned on successful call) z load center of mass z mm 
    """

    @log_call
    @xmlrpc_timeout
    def GetForceSensorPayloadCog(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetForceSensorPayloadCog()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2], _error[3]
        else:
            return error,None,None,None

    """   
    @brief  Force sensor automatic zero calibration
    @return error code success- 0, failure-error code
    @return Return value (returned on successful call) weight sensor mass kg 
    @return Return value (returned on successful call) pos=[x,y,z] sensor center of mass mm
    """
    @log_call
    @xmlrpc_timeout
    def ForceSensorAutoComputeLoad(self):
        rtn = self.ForceSensorSetSaveDataFlag(1)
        if rtn!=0:
            return rtn,None,None
        error =self.GetActualJointPosDegree()
        start_joint = error[1]
        error = self.GetActualJointPosDegree()
        if error[0]==0:
            joint =error[1]
            if joint[2]<0:
                joint[3] = joint[3] + 90
            else:
                joint[3] = joint[3] - 90
            rtn = self.MoveJ(joint,0,0,vel=10)
            if rtn!=0:
                return rtn,None,None
        else:
            return error,None,None

        rtn = self.ForceSensorSetSaveDataFlag(2)
        if rtn!=0:
            return rtn,None,None

        error = self.GetActualJointPosDegree()
        if error[0] == 0:
            joint = error[1]
            if joint[5] < 0:
                joint[5] = joint[5] + 90
            else:
                joint[5] = joint[5] - 90
            rtn = self.MoveJ(joint, 0, 0,vel=10)
            if rtn != 0:
                return rtn,None,None
        else:
            return error,None,None

        rtn = self.ForceSensorSetSaveDataFlag(3)
        if rtn!=0:
            return rtn,None,None

        _error = self.robot.ForceSensorComputeLoad()
        error = _error[0]
        self.MoveJ(start_joint,0,0,vel=10)
        if error == 0:
            return error, _error[1],[_error[2],_error[3],_error[4]]
        else:
            return error,None,None

    """   
    @brief  Sensor automatic zero calibration data recording
    @param  [in] Required parameter recordCount number of recorded data 1-3
    @return error code success- 0, failure-error code
    """
    @log_call
    @xmlrpc_timeout
    def ForceSensorSetSaveDataFlag(self,recordCount):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ForceSensorSetSaveDataFlag(recordCount)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Sensor automatic zero calibration calculation
    @return Error code success- 0, failure-error code    
    @return Return value (returned on successful call) weight sensor mass kg 
    @return Return value (returned on successful call) pos=[x,y,z] sensor center of mass mm
    """

    @log_call
    @xmlrpc_timeout
    def ForceSensorComputeLoad(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ForceSensorComputeLoad()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1],[_error[2],_error[3],_error[4]]
        else:
            return error,None,None

    """   
    @brief  End-effector sensor configuration
    @param  [in] Required parameter idCompany manufacturer, 18-JUNKONG; 25-HUIDE
    @param  [in] Required parameter idDevice type, 0-JUNKONG/RYR6T.V1.0
    @param  [in] Required parameter idSoftware software version, 0-J1.0/HuiDe1.0 (not yet open)
    @param  [in] Required parameter idBus mount position, 1-end port 1; 2-end port 2...8-end port 8 (not yet open)
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def AxleSensorConfig(self,idCompany, idDevice, idSoftware, idBus):
        while self.reconnect_flag:
            time.sleep(0.1)
        idCompany = int(idCompany)
        idDevice = int(idDevice)
        idSoftware = int(idSoftware)
        idBus = int(idBus)

        flag = True
        while flag:
            try:
                error = self.robot.AxleSensorConfig(idCompany, idDevice, idSoftware, idBus)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get end-effector sensor configuration    
    @return error code success- 0, failure-error code  
    @return Return value (returned on successful call) idCompany manufacturer, 18-JUNKONG; 25-HUIDE
    @return Return value (returned on success) idDevice type, 0-JUNKONG/RYR6T.V1.0
    """

    @log_call
    @xmlrpc_timeout
    def AxleSensorConfigGet(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.AxleSensorConfigGet()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2]
        else:
            return error,None,None

    """   
    @brief  End sensor activation
    @param  [in] Required parameter actFlag 0-reset; 1-activate
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def AxleSensorActivate(self,actFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        actFlag = int(actFlag)
        flag = True
        while flag:
            try:
                error = self.robot.AxleSensorActivate(actFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  End sensor register write
    @param  [in] Required parameter devAddr  Device address number 0-255
    @param  [in] Required parameter regHAddr Register address high 8 bits
    @param  [in] Required parameter regLAddr Register address low 8 bits
    @param  [in] Required parameter regNum  Number of registers 0-255
    @param  [in] Required parameter data1 Register write value 1
    @param  [in] Required parameter data2 Register write value 2
    @param  [in] Required parameter isNoBlock Whether blocking 0-blocking; 1-non-blocking
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def AxleSensorRegWrite(self,devAddr, regHAddr, regLAddr, regNum, data1, data2, isNoBlock):
        while self.reconnect_flag:
            time.sleep(0.1)
        devAddr = int(devAddr)
        regHAddr = int(regHAddr)
        regLAddr = int(regLAddr)
        regNum = int(regNum)
        data1 = int(data1)
        data2 = int(data2)
        isNoBlock = int(isNoBlock)
        flag = True
        while flag:
            try:
                error = self.robot.AxleSensorRegWrite(devAddr, regHAddr, regLAddr, regNum, data1, data2, isNoBlock)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set whether control box DO output resets after stop/pause
    @param  [in] Required parameter resetFlag  0-do not reset; 1-reset
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetCtlBoxDO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetCtlBoxDO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set whether control box AO output resets after stop/pause
    @param  [in] Required parameter resetFlag  0-do not reset; 1-reset
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetCtlBoxAO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetCtlBoxAO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set whether end tool DO output resets after stop/pause
    @param  [in] Required parameter resetFlag  0-do not reset; 1-reset
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetAxleDO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetAxleDO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set whether end tool AO output resets after stop/pause
    @param  [in] Required parameter resetFlag  0-do not reset; 1-reset
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetAxleAO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetAxleAO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set whether extended DO output resets after stop/pause
    @param  [in] Required parameter resetFlag  0-do not reset; 1-reset
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetExtDO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetExtDO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set whether extended AO output resets after stop/pause
    @param  [in] Required parameter resetFlag  0-do not reset; 1-reset
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetExtAO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetExtAO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set whether SmartTool output resets after stop/pause
    @param  [in] Required parameter resetFlag  0-do not reset; 1-reset
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetSmartToolDO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetSmartToolDO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  Simulated weave start
    @param  [in] Required parameter weaveNum  Weave parameter number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def WeaveStartSim(self,weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveStartSim(weaveNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Simulated weave end
    @param  [in] Required parameter weaveNum  Weave parameter number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def WeaveEndSim(self,weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveEndSim(weaveNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Start trajectory detection warning (no motion)
    @param  [in] Required parameter weaveNum  Weave parameter number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def WeaveInspectStart(self,weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveInspectStart(weaveNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  End trajectory detection warning (no motion)
    @param  [in] Required parameter weaveNum  Weave parameter number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def WeaveInspectEnd(self,weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveInspectEnd(weaveNum)
                flag = False
            except socket.error as e:
                flag = True

        return error



    """   
    @brief  Set welding process curve parameters
    @param  [in] Required parameter id Welding process number (1-99)
    @param  [in] Required parameter startCurrent Arc start current (A)
    @param  [in] Required parameter startVoltage Arc start voltage (V)
    @param  [in] Required parameter startTime Arc start time (ms)
    @param  [in] Required parameter weldCurrent Welding current (A)
    @param  [in] Required parameter weldVoltage Welding voltage (V)
    @param  [in] Required parameter endCurrent Arc end current (A)
    @param  [in] Required parameter endVoltage Arc end voltage (V)
    @param  [in] Required parameter endTime Arc end time (ms)
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetProcessParam(self, id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent,
                               endVoltage, endTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        startCurrent = float(startCurrent)
        startVoltage = float(startVoltage)
        startTime = float(startTime)
        weldCurrent = float(weldCurrent)
        weldVoltage = float(weldVoltage)
        endCurrent = float(endCurrent)
        endVoltage = float(endVoltage)
        endTime = float(endTime)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage,
                                                          endCurrent, endVoltage, endTime)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get welding process curve parameters    
    @param  [in] Required parameter id Welding process number (1-99)
    @return error code success- 0, failure-error code  
    @return Return value (returned on success) startCurrent Arc start current (A)
    @return Return value (returned on success) startVoltage Arc start voltage (V)
    @return Return value (returned on success) startTime Arc start time (ms)
    @return Return value (returned on success) weldCurrent Welding current (A)
    @return Return value (returned on success) weldVoltage Welding voltage (V)
    @return Return value (returned on success) endCurrent Arc end current (A)
    @return Return value (returned on success) endVoltage Arc end voltage (V)
    @return Return value (returned on success) endTime Arc end time (ms)
    """

    @log_call
    @xmlrpc_timeout
    def WeldingGetProcessParam(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.WeldingGetProcessParam(id)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error,None,None,None,None,None,None,None,None

    """   
    @brief  Extended IO - Configure welder gas detection signal
    @param  [in] Required parameter DONum  Gas detection signal extended DO number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetAirControlExtDoNum(self,DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetAirControlExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Extended IO - Configure welder arc start signal
    @param  [in] Required parameter DONum  Welder arc start signal extended DO number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetArcStartExtDoNum(self,DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetArcStartExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Extended IO - Configure welder reverse wire feed signal
    @param  [in] Required parameter DONum  Reverse wire feed signal extended DO number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetWireReverseFeedExtDoNum(self,DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWireReverseFeedExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Extended IO - Configure welder forward wire feed signal
    @param  [in] Required parameter DONum  Forward wire feed signal extended DO number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetWireForwardFeedExtDoNum(self,DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWireForwardFeedExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Extended IO - Configure welder arc start success signal
    @param  [in] Required parameter DINum  Arc start success signal extended DI number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetArcDoneExtDiNum(self,DINum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DINum = int(DINum)
        flag = True
        while flag:
            try:
                error = self.robot.SetArcDoneExtDiNum(DINum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Extended IO - Configure welder ready signal
    @param  [in] Required parameter DINum  Welder ready signal extended DI number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetWeldReadyExtDiNum(self,DINum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DINum = int(DINum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWeldReadyExtDiNum(DINum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Extended IO - Configure welding interruption recovery signal
    @param  [in] Required parameter reWeldDINum  Signal to resume welding after interruption, extended DI number
    @param  [in] Required parameter abortWeldDINum  Signal to exit welding after interruption, extended DI number
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetExtDIWeldBreakOffRecover(self,reWeldDINum, abortWeldDINum):
        while self.reconnect_flag:
            time.sleep(0.1)
        reWeldDINum = int(reWeldDINum)
        abortWeldDINum = int(abortWeldDINum)
        flag = True
        while flag:
            try:
                error = self.robot.SetExtDIWeldBreakOffRecover(reWeldDINum, abortWeldDINum)
                flag = False
            except socket.error as e:
                flag = True

        return error



    """   
    @brief  Set robot collision detection method
    @param  [in] Required parameter method Collision detection method: 0-current mode; 1-dual encoder; 2-current and dual encoder enabled simultaneously
    @param  [in] Required parameter thresholdMode Collision level threshold mode; 0-collision level fixed threshold mode; 1-custom collision detection threshold
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetCollisionDetectionMethod(self,method,thresholdMode):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        thresholdMode = int(thresholdMode)
        flag = True
        while flag:
            try:
                error = self.robot.SetCollisionDetectionMethod(method,thresholdMode)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set static collision detection start/stop
    @param  [in] Required parameter status 0-off; 1-on
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetStaticCollisionOnOff(self,status):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.SetStaticCollisionOnOff(status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Joint torque power detection
    @param  [in] Required parameter status 0-off; 1-on
    @param  [in] Required parameter power Set maximum power (W)
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetPowerLimit(self,status, power):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        power = float(power)
        flag = True
        while flag:
            try:
                error = self.robot.SetPowerLimit(status, power)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set robot 20004 port feedback period
    @param  [in] Required parameter period Robot 20004 port feedback period (ms)
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotRealtimeStateSamplePeriod(self,period):
        while self.reconnect_flag:
            time.sleep(0.1)
        period = int(period)
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotRealtimeStateSamplePeriod(period)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get robot 20004 port feedback period
    @param  [in]NULL
    @return Error code success- 0, failure-error code    
    @return Return value (returned on success) period Robot 20004 port feedback period (ms)
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotRealtimeStateSamplePeriod(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotRealtimeStateSamplePeriod()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None


    """   
    @brief  Get current torque of joint drives
    @param  [in] Required parameter NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) data=[j1,j2,j3,j4,j5,j6] Joint torque    [fx,fy,fz,tx,ty,tz]
    """
    @log_call
    @xmlrpc_timeout
    def GetJointDriverTorque(self):
        return 0,[self.robot_state_pkg.jointDriverTorque[0],self.robot_state_pkg.jointDriverTorque[1],self.robot_state_pkg.jointDriverTorque[2],
                  self.robot_state_pkg.jointDriverTorque[3],self.robot_state_pkg.jointDriverTorque[4],self.robot_state_pkg.jointDriverTorque[5]]


    """   
    @brief  Get current temperature of joint drives
    @param  [in] Required parameter NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) data=[t1,t2,t3,t4,t5,t6]
    """
    @log_call
    @xmlrpc_timeout
    def GetJointDriverTemperature (self):
        return 0,[self.robot_state_pkg.jointDriverTemperature [0],self.robot_state_pkg.jointDriverTemperature [1],self.robot_state_pkg.jointDriverTemperature[2],
                  self.robot_state_pkg.jointDriverTemperature [3],self.robot_state_pkg.jointDriverTemperature[4],self.robot_state_pkg.jointDriverTemperature[5]]



    """   
    @brief  Arc tracking + multi-layer multi-pass compensation enable
    @param  [in] Required parameter NULL
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceReplayStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceReplayStart()
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  Arc tracking + multi-layer multi-pass compensation disable
    @param  [in] Required parameter NULL
    @return Error code success- 0, failure-error code    
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceReplayEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceReplayEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Offset coordinate change - multi-layer multi-pass welding
    @param  [in] pointo Reference point Cartesian pose
    @param  [in] pointX Cartesian pose of reference point X-direction offset point
    @param  [in] pointZ Cartesian pose of reference point Z-direction offset point
    @param  [in] dx x-direction offset (mm)
    @param  [in] dz z-direction offset (mm)
    @param  [in] dry offset about y-axis (°)
    @return Error code success- 0, failure-error code    
    @return Return value (returned on success) offset Calculated offset result
    """

    @log_call
    @xmlrpc_timeout
    def MultilayerOffsetTrsfToBase(self,pointo,pointX,pointZ,dx,dz,dry):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointo =list(map(float,pointo))
        pointX = list(map(float, pointX))
        pointZ = list(map(float, pointZ))
        dx = float(dx)
        dz = float(dz)
        dry = float(dry)
        flag = True
        while flag:
            try:
                _error = self.robot.MultilayerOffsetTrsfToBase(pointo[0],pointo[1],pointo[2],
                                                               pointX[0],pointX[1],pointX[2],pointZ[0],pointZ[1],pointZ[2],dx,dz,dry)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None


    """   
    @brief  Enable specified orientation velocity
    @param  [in] Required parameter ratio Orientation velocity percentage [0-300]
    @return Error code success- 0, failure-error code    
    """
    @log_call
    @xmlrpc_timeout
    def AngularSpeedStart(self, ratio):
        while self.reconnect_flag:
            time.sleep(0.1)
        ratio = int(ratio)
        flag = True
        while flag:
            try:
                error = self.robot.AngularSpeedStart(ratio)
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  Disable specified orientation velocity
    @return Error code success- 0, failure-error code    
    """
    @log_call
    @xmlrpc_timeout
    def AngularSpeedEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.AngularSpeedEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  Robot software upgrade
    @param  [in] Required parameter  filePath Full path of software upgrade package
    @param  [in] Required parameter block Whether to block until upgrade completes true: blocking; false: non-blocking
    @return Error code success- 0, failure-error code    
    """
    @log_call
    @xmlrpc_timeout
    def SoftwareUpgrade(self,filePath, block):
        error = self.__FileUpLoad(1,filePath)

        print("__FileUpLoad", error)
        if 0==error:
            self.log_info("Software Upload success!")
            error =self.robot.SoftwareUpgrade()
            if 0!=error:
                return error
            if block:
                upgradeState = -1
                time.sleep(0.5)
                upgradeState = self.GetSoftwareUpgradeState()
                if upgradeState == 0:
                    self.log_error("software upgrade not start")
                    return -1
                while (upgradeState > 0 and upgradeState < 100):
                    time.sleep(0.5)
                    upgradeState = self.GetSoftwareUpgradeState()
                    # print("upgradeState",upgradeState,"%")
                if upgradeState == 100:
                    error = 0
                else:
                    error = upgradeState
            return error
        else:
            self.log_error("execute SoftwareUpgrade fail.")
            return error


    """   
    @brief  Get robot software upgrade status
    @return Error code Success- 0, Failure- error code   
    @return Return value (returned on success) state Robot software package upgrade status 0: idle or uploading upgrade package, 1~100: upgrade completion percentage, -1: software upgrade failed, -2: verification failed, -3: version verification failed, -4: decompression failed, -5: user configuration upgrade failed, -6: peripheral configuration upgrade failed, -7: extended axis configuration upgrade failed, -8: robot configuration upgrade failed, -9: DH parameter configuration upgrade failed
    """
    @log_call
    @xmlrpc_timeout
    def GetSoftwareUpgradeState(self):
        error = self.robot_state_pkg.softwareUpgradeState
        return error

    """   
    @brief  Set 485 extended axis motion acceleration/deceleration
    @param  [in] Required parameter  acc 485 extended axis motion acceleration
    @param  [in] Required parameter dec 485 extended axis motion deceleration
    @return Error code success- 0, failure-error code    
    """
    @log_call
    @xmlrpc_timeout
    def AuxServoSetAcc(self,acc,dec):
        while self.reconnect_flag:
            time.sleep(0.1)
        acc = float(acc)
        dec = float(dec)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetAcc(acc,dec)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set 485 extended axis emergency stop acceleration/deceleration
    @param  [in] Required parameter  acc 485 extended axis emergency stop acceleration
    @param  [in] Required parameter dec 485 extended axis emergency stop deceleration
    @return Error code success- 0, failure-error code    
    """
    @log_call
    @xmlrpc_timeout
    def AuxServoSetEmergencyStopAcc(self,acc,dec):
        while self.reconnect_flag:
            time.sleep(0.1)
        acc = float(acc)
        dec = float(dec)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetEmergencyStopAcc(acc,dec)
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  Get 485 extended axis emergency stop acceleration/deceleration
    @return Error code Success- 0, Failure- error code        
    @return Return value (returned on success) acc 485 extended axis emergency stop acceleration   
    @return Return value (returned on success) dec 485 extended axis emergency stop deceleration
    """
    @log_call
    @xmlrpc_timeout
    def AuxServoGetEmergencyStopAcc(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoGetEmergencyStopAcc()
                flag = False
            except socket.error as e:
                flag = True

        if error[0]==0:
            return error[0],error[1],error[2]
        else:
            return error

    """   
    @brief  Get 485 extended axis motion acceleration/deceleration
    @return Error code Success- 0, Failure- error code        
    @return Return value (returned on success) acc 485 extended axis motion acceleration 
    @return Return value (returned on success) dec 485 extended axis motion deceleration
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoGetAcc(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoGetAcc()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1], error[2]
        else:
            return error

    """   
    @brief  Get end communication parameters
    @return Error code Success- 0, Failure- error code        
    @return Return value (returned on success) baudRate Baud rate: supports 1-9600, 2-14400, 3-19200, 4-38400, 5-56000, 6-67600, 7-115200, 8-128000;
    @return Return value (returned on success) dataBit Data bits: data bits support (8,9), currently 8 is commonly used
    @return Return value (returned on success) stopBit Stop bits: 1-1, 2-0.5, 3-2, 4-1.5, currently 1 is commonly used
    @return Return value (returned on success) verify Parity bit: 0-None, 1-Odd, 2-Even, currently 0 is commonly used;
    @return Return value (returned on success) timeout Timeout: 1~1000ms, this value needs to be set to a reasonable time parameter in combination with the peripheral
    @return Return value (returned on success) timeoutTimes  Timeout count: 1~10, mainly for timeout retransmission, reducing occasional exceptions and improving user experience
    @return Return value (returned on success) period Periodic command time interval: 1~1000ms, mainly used for the time interval of each periodic command sent
    """

    @log_call
    @xmlrpc_timeout
    def GetAxleCommunicationParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleCommunicationParam()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1], error[2], error[3], error[4], error[5], error[6], error[7]
        else:
            return error

    """   
    @brief  Set end communication parameters
    @param  [in]  baudRate Baud rate: supports 1-9600, 2-14400, 3-19200, 4-38400, 5-56000, 6-67600, 7-115200, 8-128000;
    @param  [in]  dataBit Data bits: data bits support (8,9), currently 8 is commonly used
    @param  [in]  stopBit Stop bits: 1-1, 2-0.5, 3-2, 4-1.5, currently 1 is commonly used
    @param  [in]  verify Parity bit: 0-None, 1-Odd, 2-Even, currently 0 is commonly used;
    @param  [in]  timeout Timeout: 1~1000ms, this value needs to be set to a reasonable time parameter in combination with the peripheral
    @param  [in]  timeoutTimes  Timeout count: 1~10, mainly for timeout retransmission, reducing occasional exceptions and improving user experience
    @param  [in]  period Periodic command time interval: 1~1000ms, mainly used for the time interval of each periodic command sent
    @return Error code Success- 0, Failure- error code        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleCommunicationParam(self,baudRate,dataBit,stopBit,verify,timeout,timeoutTimes,period):
        while self.reconnect_flag:
            time.sleep(0.1)
        baudRate = int (baudRate)
        dataBit = int (dataBit)
        stopBit = int (stopBit)
        verify = int (verify)
        timeout = int (timeout)
        timeoutTimes = int (timeoutTimes)
        period = int(period)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleCommunicationParam(baudRate,dataBit,stopBit,verify,timeout,timeoutTimes,period)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set end file transfer type
    @param  [in] type 1-MCU upgrade file; 2-LUA file
    @return Error code Success- 0, Failure- error code        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleFileType(self,type):
        while self.reconnect_flag:
            time.sleep(0.1)
        type=int(type)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleFileType(type)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set enable end LUA execution
    @param  [in] enable 0-disable; 1-enable
    @return Error code Success- 0, Failure- error code        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleLuaEnable(self,enable):
        while self.reconnect_flag:
            time.sleep(0.1)
        enable=int(enable)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleLuaEnable(enable)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  End LUA file exception error recovery
    @param  [in] status 0-do not recover; 1-recover
    @return Error code Success- 0, Failure- error code        
    """
    @log_call
    @xmlrpc_timeout
    def SetRecoverAxleLuaErr(self,enable):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SetRecoverAxleLuaErr(enable)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get end LUA execution enable status
    @return Error code Success- 0, Failure- error code   
    @return Return value (returned on success) enable 0-disable; 1-enable
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleLuaEnableStatus(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleLuaEnableStatus()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1]
        else:
            return error

    """   
    @brief  Set end LUA end device enable type
    @param  [in] forceSensorEnable Force sensor enable status, 0-disable; 1-enable
    @param  [in] gripperEnable Gripper enable status, 0-disable; 1-enable
    @param  [in] IOEnable IO device enable status, 0-disable; 1-enable
    @return Error code Success- 0, Failure- error code        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleLuaEnableDeviceType(self,forceSensorEnable,gripperEnable,IOEnable):
        while self.reconnect_flag:
            time.sleep(0.1)
        forceSensorEnable = int(forceSensorEnable)
        gripperEnable = int(gripperEnable)
        IOEnable = int(IOEnable)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleLuaEnableDeviceType(forceSensorEnable,gripperEnable,IOEnable)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get end LUA end device enable type
    @return Error code Success- 0, Failure- error code   
    @return Return value (returned on success) forceSensorEnable Force sensor enable status, 0-disable; 1-enable
    @return Return value (returned on success) gripperEnable Gripper enable status, 0-disable; 1-enable
    @return Return value (returned on success) IOEnable IO device enable status, 0-disable; 1-enable
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleLuaEnableDeviceType(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleLuaEnableDeviceType()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1], error[2], error[3]
        else:
            return error

    """   
    @brief  Get currently configured end device
    @return Error code Success- 0, Failure- error code   
    @return Return value (returned on success) forceSensorEnable[8] Force sensor enable status, 0-disable; 1-enable
    @return Return value (returned on success) gripperEnable[8] Gripper enable status, 0-disable; 1-enable
    @return Return value (returned on success) IOEnable[8]  IO device enable status, 0-disable; 1-enable
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleLuaEnableDevice(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleLuaEnableDevice()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            par= error[1].split(',')
            if 24 != len(par):
                self.log_error("GetAxleLuaEnableDevice fail")
                return -1,None,None,None
            else:
                print(par)
                return (error[0], [par[0],par[1], par[2], par[3], par[4], par[5], par[6], par[7]],
                        [par[8], par[9], par[10], par[11], par[12], par[13], par[14], par[15]],
                        [par[16], par[17], par[18], par[19], par[20], par[21], par[22], par[23]])
        else:
            return error,None,None,None

    """   
    @brief  Set enable gripper action control function
    @param  [in] id Gripper device number
    @param  [in] func 0-gripper enable; 1-gripper initialization; 2-position setting; 3-velocity setting; 4-torque setting; 6-read gripper status; 7-read initialization status; 8-read fault code; 9-read position; 10-read velocity; 11-read torque, 12-15 reserved
    @return Error code Success- 0, Failure- error code        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleLuaGripperFunc(self,id,func):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        func = list(map(int, func))
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleLuaGripperFunc(id,func)
                flag = False
            except socket.error as e:
                flag = True

        # error = self.robot.SetAxleLuaGripperFunc(id,func)
        return error

    """   
    @brief  Get enable gripper action control function
    @param  [in] id Gripper device number
    @return Error code Success- 0, Failure- error code   
    @return Return value (returned on success) func 0-gripper enable; 1-gripper initialization; 2-position setting; 3-velocity setting; 4-torque setting; 6-read gripper status; 7-read initialization status; 8-read fault code; 9-read position; 10-read velocity; 11-read torque
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleLuaGripperFunc(self,id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id=int(id)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleLuaGripperFunc(id)
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            par = error[1].split(',')
            print(len(par))
            if 16 != len(par):
                self.log_error("GetAxleLuaEnableDevice fail")
                return -1
            else:
                return (error[0], [par[0],par[1],par[2], par[3], par[4], par[5], par[6], par[7], par[8],
                        par[9], par[10], par[11], par[12], par[13], par[14], par[15]])
        else:
            return error

    """   
    @brief  Set controller peripheral protocol LUA file name
    @param  [in]  id Protocol number
    @param  [in] name lua file name “CTRL_LUA_test.lua”
    @return Error code Success- 0, Failure- error code        
    """
    @log_call
    @xmlrpc_timeout
    def SetCtrlOpenLUAName(self,id,name):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        name = str(name)
        flag = True
        while flag:
            try:
                error = self.robot.SetCtrlOpenLUAName(id,name)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Get currently configured controller peripheral protocol LUA file name
    @return Error code Success-0, Fail-error code     
    @return Return value (returned on success) name[4] lua file name "CTRL_LUA_test.lua"
    """
    @log_call
    @xmlrpc_timeout
    def GetCtrlOpenLUAName(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetCtrlOpenLUAName()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            par = error[2].split(',')
            if 4 != sizeof(par):
                self.log_error("GetCtrlOpenLUAName fail")
                return -1
            else:
                return error[0], [error[1], error[2], error[3], error[4]]
        else:
            return error

    """   
    @brief  Load controller LUA protocol
    @param  [in] id Controller LUA protocol number
    @return Error code Success-0, Fail-error code     
    """

    @log_call
    @xmlrpc_timeout
    def LoadCtrlOpenLUA(self,id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                error = self.robot.LoadCtrlOpenLUA(id)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief   Unload controller LUA protocol
    @param  [in] id Controller LUA protocol number
    @return Error code Success-0, Fail-error code     
    """

    @log_call
    @xmlrpc_timeout
    def UnloadCtrlOpenLUA(self,id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                error = self.robot.UnloadCtrlOpenLUA(id)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Set controller LUA protocol error code
    @param  [in] id Controller LUA protocol number
    @return Error code Success-0, Fail-error code     
    """

    @log_call
    @xmlrpc_timeout
    def SetCtrlOpenLuaErrCode(self,id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                error = self.robot.SetCtrlOpenLuaErrCode(id)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Robot Ethercat slave file write
    @param  [in] type Slave file type, 1-upgrade slave file; 2-upgrade slave configuration file
    @param  [in] slaveID Slave number
    @param  [in] fileName Upload file name
    @return Error code Success-0, Fail-error code     
    """

    @log_call
    @xmlrpc_timeout
    def SlaveFileWrite(self,type,slaveID,fileName):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        slaveID = int(slaveID)
        fileName =str(fileName)
        flag = True
        while flag:
            try:
                error = self.robot.SlaveFileWrite(type,slaveID,fileName)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Robot Ethercat slave enters boot mode
    @return Error code Success-0, Fail-error code     
    """

    @log_call
    @xmlrpc_timeout
    def SetSysServoBootMode(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SetSysServoBootMode()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Upload end-effector Lua open protocol file
    @param  [in] filePath Local lua file path name ".../AXLE_LUA_End_DaHuan.lua"
    @return Error code Success-0, Fail-error code     
    """

    @log_call
    @xmlrpc_timeout
    def AxleLuaUpload(self,filePath):

        error = self.__FileUpLoad(10,filePath)
        file_name = "/tmp/" + os.path.basename(filePath)
        # file_name = os.path.basename(filePath)
        if 0!= error :
            return error
        else:
            rtn = self.SetAxleFileType(2)
            if(rtn!=0):
                return -1
            rtn = self.SetSysServoBootMode()
            if(rtn!=0):
                return -1
            rtn = self.SlaveFileWrite(1,7,file_name)
            if(rtn!=0):
                return -1
            return rtn


    """   
    ***************************************************************************New********************************************************************************************
    """

    """   
    @brief  Movable device enable
    @param  [in] enable Enable state, 0-disable, 1-enable
    @return Error code Success-0, Fail-error code     
    """

    @log_call
    @xmlrpc_timeout
    def TractorEnable(self, enable):
        while self.reconnect_flag:
            time.sleep(0.1)
        enable = int(enable)
        flag = True
        while flag:
            try:
                error = self.robot.TractorEnable(enable)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  Movable device homing
    @return Error code Success-0, Fail-error code     
    """

    @log_call
    @xmlrpc_timeout
    def TractorHoming(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.TractorHoming()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief Movable device linear motion
    @param [in] distance Linear motion distance (mm)
    @param [in] vel Linear motion velocity percentage (0-100)
    @return error code success- 0, failure-error code  
    """

    @log_call
    @xmlrpc_timeout
    def TractorMoveL(self,distance,vel):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        distance = float(distance)
        vel = float(vel)
        flag = True
        while flag:
            try:
                error = self.robot.TractorMoveL(distance,vel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief Movable device arc motion
    @param [in] radio Arc motion radius (mm)
    @param [in] angle Arc motion angle (deg)
    @param [in] vel Arc motion velocity percentage (0-100)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def TractorMoveC(self,radio, angle, vel):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        radio = float(radio)
        angle = float(angle)
        vel = float(vel)
        flag = True
        while flag:
            try:
                error = self.robot.TractorMoveC(radio, angle, vel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief Movable device stop motion
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout

    def TractorStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ProgramStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief Set welding wire seam-search extended IO port
    @param [in] searchDoneDINum Welding wire seam-search success DO port (0-127)
    @param [in] searchStartDONum Welding wire seam-search start/stop control DO port (0-127)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout

    def SetWireSearchExtDIONum(self,searchDoneDINum,searchStartDONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        searchDoneDINum = int(searchDoneDINum)
        searchStartDONum = int(searchStartDONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWireSearchExtDIONum(searchDoneDINum,searchStartDONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief Set welder control mode extended DO port
    @param [in] DONum Welder control mode DO port (0-127)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout

    def SetWeldMachineCtrlModeExtDoNum(self, DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWeldMachineCtrlModeExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief Set welder control mode
    @param [in] mode Welder control mode; 0-unified
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout

    def SetWeldMachineCtrlMode(self, mode):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        flag = True
        while flag:
            try:
                error = self.robot.SetWeldMachineCtrlMode(mode)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief Close RPC
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout

    def CloseRPC(self):
        # Set stop event to notify thread to stop
        self.stop_event.set()

        # If the thread is still running, wait for it to finish
        # if self.thread.is_alive():
        #     self.thread.join()

        # Clean up XML-RPC proxy
        if self.robot is not None:
            self.robot = None  # Set proxy to None, release resources
            self.sock_cli_state.close()
            self.sock_cli_state = None
            self.robot_state_pkg = None
            self.closeRPC_state = True
            # self.robot_realstate_exit = False

        # If the thread is still running, wait for it to finish
        if self.thread.is_alive():
            self.thread.join()


        print("RPC connection closed.")
        return

    """   
    @brief Record teach point
    @param [in] name Teach point name
    @param [in] update_allprogramfile Whether to overwrite 0-do not overwrite 1-overwrite
    @return error code success- 0, failure-error code
    """

    # @log_call
    # @xmlrpc_timeout
    #
    # def SavePoint(self,name,update_allprogramfile=0):
    #     name = str(name)
    #     update_allprogramfile = int(update_allprogramfile)
    #     error = self.robot.save_point(name,update_allprogramfile)
    #     return error

    """   
    @brief Start singularity pose protection
    @param [in] protectMode Singularity protection mode, 0: joint mode; 1-Cartesian mode
    @param [in] minShoulderPos Shoulder singularity adjustment range (mm), default 100.0
    @param [in] minElbowPos Elbow singularity adjustment range (mm), default 50.0
    @param [in] minWristPos Wrist singularity adjustment range (deg), default 10.0
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout

    def SingularAvoidStart(self, protectMode, minShoulderPos=100,minElbowPos=50,minWristPos=10):
        while self.reconnect_flag:
            time.sleep(0.1)
        protectMode = int(protectMode)
        minShoulderPos = float(minShoulderPos)
        minElbowPos = float(minElbowPos)
        minWristPos = float(minWristPos)
        flag = True
        while flag:
            try:
                error = self.robot.SingularAvoidStart(protectMode, minShoulderPos,minElbowPos,minWristPos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief Stop singularity pose protection
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout

    def SingularAvoidEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SingularAvoidEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
        @brief Get the number of rotations of the rotary gripper
        @return Error code Success-0, Fail-error code
        @return Return value (returned on success) fault 0-no error, 1-has error
        @return Return value (returned on success) num Number of rotations
    """

    @log_call
    @xmlrpc_timeout

    def GetGripperRotNum(self):
        return 0,self.robot_state_pkg.gripper_fault,self.robot_state_pkg.gripperRotNum

    """   
        @brief Get the rotation speed percentage of the rotary gripper
        @return Error code Success-0, Fail-error code
        @return Return value (returned on success) fault 0-no error, 1-has error
        @return Return value (returned on success) speed Rotation speed percentage
    """

    @log_call
    @xmlrpc_timeout

    def GetGripperRotSpeed(self):
        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripperRotSpeed

    """   
        @brief Get the rotation torque percentage of the rotary gripper
        @return Error code Success-0, Fail-error code
        @return Return value (returned on success) fault 0-no error, 1-has error
        @return Return value (returned on success) torque Rotation torque percentage
    """

    @log_call
    @xmlrpc_timeout

    def GetGripperRotTorque(self):
        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripperRotTorque

    """   
       @brief Start Ptp motion FIR filtering
       @param  [in] maxAcc Maximum acceleration limit (deg/s2)
       @param  [in] maxJek Unified joint jerk limit (deg/s3)
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def PtpFIRPlanningStart(self, maxAcc,maxJek):
        while self.reconnect_flag:
            time.sleep(0.1)
        maxAcc = float(maxAcc)
        maxJek = float(maxJek)
        flag = True
        while flag:
            try:
                error = self.robot.PtpFIRPlanningStart(maxAcc,maxJek)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
       @brief Close Ptp motion FIR filtering
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def PtpFIRPlanningEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.PtpFIRPlanningEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
       @brief Upload trajectory J file
       @param  [in] filePath Full path name of the trajectory file to upload   C://test/testJ.txt
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def TrajectoryJUpLoad(self,filePath):
        error = self.__FileUpLoad(20, filePath)
        return error

    """2024.12.16"""
    """   
       @brief Delete trajectory J file
       @param  [in] filePath Full path name of the trajectory file to delete   C://test/testJ.txt
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def TrajectoryJDelete(self, fileName):
        error = self.__FileDelete(20, fileName)
        return error

    """2024.12.18"""
    """   
       @brief Start LIN, ARC motion FIR filtering
       @param  [in] maxAccLin Linear acceleration limit (mm/s2)
       @param  [in] maxAccDeg Angular acceleration limit (deg/s2)
       @param  [in] maxJerkLin Linear jerk limit (mm/s3)
       @param  [in] maxJerkDeg Angular jerk limit (deg/s3)
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LinArcFIRPlanningStart(self, maxAccLin, maxAccDeg, maxJerkLin, maxJerkDeg):
        while self.reconnect_flag:
            time.sleep(0.1)
        maxAccLin = float(maxAccLin)
        maxAccDeg = float(maxAccDeg)
        maxJerkLin = float(maxJerkLin)
        maxJerkDeg = float(maxJerkDeg)
        flag = True
        while flag:
            try:
                error = self.robot.LinArcFIRPlanningStart(maxAccLin, maxAccDeg, maxJerkLin, maxJerkDeg)
                flag = False
            except socket.error as e:
                flag = True

        return  error

    """   
       @brief Close LIN, ARC motion FIR filtering
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LinArcFIRPlanningEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LinArcFIRPlanningEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.01.08"""
    """   
       @brief Tool coordinate frame transformation start
       @param  [in] toolNum Tool coordinate frame number [0-14]
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def ToolTrsfStart(self, toolNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        toolNum = int(toolNum)
        flag = True
        while flag:
            try:
                error = self.robot.ToolTrsfStart(toolNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
       @brief Tool coordinate frame transformation end
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def ToolTrsfEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ToolTrsfEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.01.08"""
    """3.7.8"""
    """
       @brief Calculate tool coordinate frame from point information
       @param  [in] method Calculation method; 0-four-point method; 1-six-point method
       @param  [in] pos Joint position group, array length is 4 for four-point method, array length is 6 for six-point method
       @return Error code Success - 0, Failure - error code
       @return Return value (returned on success) tcp_offset=[x,y,z,rx,ry,rz]: Tool coordinate frame calculated from point information, unit [mm][deg]
    """

    @log_call
    @xmlrpc_timeout

    def ComputeToolCoordWithPoints(self, method, pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        param = {}
        param[0] = pos[0]
        param[1] = pos[1]
        param[2] = pos[2]
        param[3] = pos[3]

        if method == 0:  # Four-point method
            param[4] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            param[5] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:  # Six-point method
            param[4] = pos[4]
            param[5] = pos[5]
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeToolCoordWithPoints(method, param[0], param[1], param[2], param[3], param[4], param[5])
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error,None

    """
       @brief Calculate work object coordinate frame from point information
       @param  [in] method Calculation method; 0: origin-x axis-z axis  1: origin-x axis-xy plane
       @param  [in] pos Three TCP position groups
       @param  [in] refFrame Reference coordinate frame
       @return Error code Success - 0, Failure - error code
       @return Return value (returned on success) wobj_offset=[x,y,z,rx,ry,rz]: Work object coordinate frame calculated from point information, unit [mm][deg]
    """

    @log_call
    @xmlrpc_timeout

    def ComputeWObjCoordWithPoints(self, method, pos, refFrame):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        param = {}
        param[0] = pos[0]
        param[1] = pos[1]
        param[2] = pos[2]
        refFrame = int(refFrame)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeWObjCoordWithPoints(method, param[0], param[1], param[2], refFrame)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error,None

    """
       @brief Set robot welding arc unexpected interruption detection parameters
       @param  [in] checkEnable Whether to enable detection; 0-disable; 1-enable
       @param  [in] arcInterruptTimeLength Arc interruption confirmation duration (ms)
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def WeldingSetCheckArcInterruptionParam(self, checkEnable, arcInterruptTimeLength):
        while self.reconnect_flag:
            time.sleep(0.1)
        checkEnable = int(checkEnable)
        arcInterruptTimeLength = int(arcInterruptTimeLength)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCheckArcInterruptionParam(checkEnable, arcInterruptTimeLength)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief Get robot welding arc unexpected interruption detection parameters
       @return Error code Success - 0, Failure - error code
       @return Return value (returned on success) checkEnable Whether to enable detection; 0-disable; 1-enable
       @return Return value (returned on success) arcInterruptTimeLength Arc interruption confirmation duration (ms)
    """

    @log_call
    @xmlrpc_timeout

    def WeldingGetCheckArcInterruptionParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.WeldingGetCheckArcInterruptionParam()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2]
        return error,None,None

    """
       @brief Set robot welding interruption recovery parameters
       @param  [in] enable Whether to enable welding interruption recovery
       @param  [in] length Weld seam overlap distance (mm)
       @param  [in] velocity Velocity percentage for robot returning to re-arc-start point (0-100)
       @param  [in] moveType Robot motion to re-arc-start point method; 0-LIN; 1-PTP
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def WeldingSetReWeldAfterBreakOffParam(self, enable, length, velocity, moveType):
        while self.reconnect_flag:
            time.sleep(0.1)
        enable = int(enable)
        length = float(length)
        velocity = float(velocity)
        moveType = int(moveType)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetReWeldAfterBreakOffParam(enable, length, velocity, moveType)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief Get robot welding interruption recovery parameters
       @return Error code Success - 0, Failure - error code
       @return Return value (returned on success) enable Whether to enable welding interruption recovery
       @return Return value (returned on success) length Weld seam overlap distance (mm)
       @return Return value (returned on success) velocity Velocity percentage for robot returning to re-arc-start point (0-100)
       @return Return value (returned on success) moveType Robot motion to re-arc-start point method; 0-LIN; 1-PTP
    """

    @log_call
    @xmlrpc_timeout

    def WeldingGetReWeldAfterBreakOffParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.WeldingGetReWeldAfterBreakOffParam()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2], _error[3], _error[4]
        return error,None,None,None,None

    """
       @brief Resume welding after robot welding interruption
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def WeldingStartReWeldAfterBreakOff(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingStartReWeldAfterBreakOff()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief Exit welding after robot welding interruption
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def WeldingAbortWeldAfterBreakOff(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingAbortWeldAfterBreakOff()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.01.09"""
    """
       @brief 
       @param  [in] status
       @param  [in] delayMode
       @param  [in] delayTime
       @param  [in] delayDisExAxisNum
       @param  [in] delayDis
       @param  [in] sensitivePara
       @param  [in] speed
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LaserSensorRecord(self, status, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, speed):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        delayMode = int(delayMode)
        delayTime = int(delayTime)
        delayDisExAxisNum = int(delayDisExAxisNum)
        delayDis = float(delayDis)
        sensitivePara = float(sensitivePara)
        speed = float(speed)
        flag = True
        while flag:
            try:
                error = self.robot.LaserSensorRecord(status, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, speed)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @param  [in] weldId
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingLaserOn(self, weldId):
        while self.reconnect_flag:
            time.sleep(0.1)
        weldId = int(weldId)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingLaserOn(weldId)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingLaserOff(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingLaserOff()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @param  [in] coordId
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingTrackOn(self, coordId):
        while self.reconnect_flag:
            time.sleep(0.1)
        coordId = int(coordId)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingTrackOn(coordId)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingTrackOff(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingTrackOff()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @param  [in] direction
       @param  [in] directionPoint
       @param  [in] vel
       @param  [in] distance
       @param  [in] timeout
       @param  [in] posSensorNum
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingSearchStart(self, direction, directionPoint, vel, distance, timeout, posSensorNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        direction = int(direction)
        directionPoint = list(map(float, directionPoint))
        vel = int(vel)
        distance = int(distance)
        timeout = int(timeout)
        posSensorNum = int(posSensorNum)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSearchStart(direction, directionPoint[0], directionPoint[1], directionPoint[2], vel, distance, timeout, posSensorNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief Laser seam-search end
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingSearchStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSearchStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.01.24"""
    """3.7.9"""
    """
       @brief Weave gradient start
       @param  [in] weaveChangeFlag Weave number 1-change weave parameters; 2-change weave parameters + welding speed
       @param  [in] weaveNum Weave number
       @param  [in] velStart Welding start speed, (cm/min)
       @param  [in] velEnd Welding end speed, (cm/min)
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def WeaveChangeStart(self, weaveChangeFlag, weaveNum, velStart, velEnd):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveChangeFlag = int(weaveChangeFlag)
        weaveNum = int(weaveNum)
        velStart = float(velStart)
        velEnd = float(velEnd)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveChangeStart(weaveChangeFlag, weaveNum, velStart, velEnd)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief Weave gradient end
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def WeaveChangeEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveChangeEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.02.20"""
    """3.8.0"""
    """
       @brief  Trajectory preprocessing (trajectory look-ahead)
       @param  [in] name  Trajectory file name
       @param  [in] mode Sampling mode, 0-no sampling; 1-equal data interval sampling; 2-equal error limit sampling
       @param  [in] errorLim Error limit, effective when using linear fitting
       @param  [in] type Smoothing method, 0-Bezier smoothing
       @param  [in] precision Smoothing precision, effective when using Bezier smoothing
       @param  [in] vamx Set maximum velocity, mm/s
       @param  [in] amax Set maximum acceleration, mm/s2
       @param  [in] jmax Set maximum jerk, mm/s3
       @param  [in] flag Constant-velocity look-ahead enable switch 0-disable; 1-enable
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def LoadTrajectoryLA(self, name, mode, errorLim, type, precision, vamx, amax, jmax, flag):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        name =str(name)
        mode = int(mode)
        errorLim = float(errorLim)
        type = int(type)
        precision = float(precision)
        vamx = float(vamx)
        amax = float(amax)
        jmax = float(jmax)
        flag = int(flag)

        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.LoadTrajectoryLA(name, mode, errorLim, type, precision, vamx, amax, jmax, flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """
       @brief Trajectory reproduction (trajectory look-ahead)
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def MoveTrajectoryLA(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.MoveTrajectoryLA()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.02.25"""
    """
      @brief  Custom collision detection threshold function start, set collision detection thresholds for joint end and TCP end
      @param  [in] flag 1-only joint detection enabled; 2-only TCP detection enabled; 3-joint and TCP detection enabled simultaneously
      @param  [in] jointDetectionThreshould Joint collision detection threshold j1-j6
      @param  [in] tcpDetectionThreshould TCP collision detection threshold, xyzabc
      @param  [in] block 0-non-blocking; 1-blocking
      @return  Error code Success-0, Fail-error code
    """

    @log_call
    @xmlrpc_timeout
    def CustomCollisionDetectionStart(self, flag, jointDetectionThreshould, tcpDetectionThreshould, block):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = int(flag)
        jointDetectionThreshould = list(map(float, jointDetectionThreshould))
        tcpDetectionThreshould = list(map(float, tcpDetectionThreshould))
        block = int(block)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.CustomCollisionDetectionStart(flag, jointDetectionThreshould, tcpDetectionThreshould, block)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """
       @brief Custom collision detection threshold function close
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout

    def CustomCollisionDetectionEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.CustomCollisionDetectionEnd()
                flag = False
            except socket.error as e:
                flag = True
        
        return error

    """2025.03.19"""
    """3.8.1"""
    """
       @brief Get robot state
       @return Error code Success - 0, Failure - error code
       @return Return value (returned on success) robot_state_pkg Robot state structure
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotRealTimeState(self):
        return 0,self.robot_state_pkg

    """   
    @brief  Stop motion
    @param  [in] NULL
    @return Error code Success-0  Failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def StopMove(self):
        error = self.send_message("/f/bIII0III102III4IIIStopIII/b/f")
        return error

    """2025.03.28"""
    """
       @brief Acceleration smoothing enable
       @param  [in] saveFlag Whether to save on power-off
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def AccSmoothStart(self, saveFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        saveFlag = bool(saveFlag)
        saveFlag_flag = 1 if saveFlag else 0
        saveFlag_flag = int(saveFlag_flag)
        flag = True
        while flag:
            try:
                error = self.robot.AccSmoothStart(saveFlag_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief Acceleration smoothing disable
       @param  [in] saveFlag Whether to save on power-off
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def AccSmoothEnd(self, saveFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        saveFlag = bool(saveFlag)
        saveFlag_flag = 1 if saveFlag else 0
        saveFlag_flag = int(saveFlag_flag)

        flag = True
        while flag:
            try:
                error = self.robot.AccSmoothEnd(saveFlag_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.04.03"""
    """
       @brief Controller log download
       @param  [in] savePath Save file path "D://zDown/"
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def RbLogDownload(self, savePath):
        try:
            error = self.robot.RbLogDownloadPrepare()
            if error == 0:
                savePath = str(savePath)
                fileName = "rblog.tar.gz"
                try:
                    error = self.__FileDownLoad(1, fileName, savePath)
                    return error
                except socket.error as e:
                    return RobotError.ERR_DOWN_LOAD_FILE_FAILED
            else:
                return error
        except socket.error as e:
            return RobotError.ERR_RPC_ERROR

    """
       @brief Download all data sources
       @param  [in] savePath Save file path "D://zDown/"
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def AllDataSourceDownload(self, savePath):
        try:
            error = self.robot.AllDataSourceDownloadPrepare()
            if error == 0:
                savePath = str(savePath)
                fileName = "alldatasource.tar.gz"
                try:
                    error = self.__FileDownLoad(2, fileName, savePath)
                    return error
                except socket.error as e:
                    return RobotError.ERR_DOWN_LOAD_FILE_FAILED
            else:
                return error
        except socket.error as e:
            return RobotError.ERR_RPC_ERROR

    """
       @brief Download data backup package
       @param  [in] savePath Save file path "D://zDown/"
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def DataPackageDownload(self, savePath):
        try:
            error = self.robot.DataPackageDownloadPrepare()
            if error == 0:
                savePath = str(savePath)
                fileName = "fr_user_data.tar.gz"
                try:
                    error = self.__FileDownLoad(3, fileName, savePath)
                    return error
                except socket.error as e:
                    return RobotError.ERR_DOWN_LOAD_FILE_FAILED
            else:
                return error
        except socket.error as e:
            return RobotError.ERR_RPC_ERROR

    """
       @brief Get control box SN code
       @return Error code Success - 0, Failure - error code
       @return Return value (returned on success) SNCode Control box SN code
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotSN(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotSN()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        return error,None

    """
       @brief Shut down robot operating system
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def ShutDownRobotOS(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.ShutDownRobotOS()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief Conveyor communication input detection
       @param  [in] timeout Wait timeout time ms
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorComDetect(self, timeout):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        timeout = int(timeout)
        flag = True
        while flag:
            try:
                # error = self.robot.ConveryComDetect(timeout)
                error = self.robot.ConveyorComDetect(timeout)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief Conveyor communication input detection trigger
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorComDetectTrigger(self):
        error = self.send_message("/f/bIII0III1149III25IIIConveyorComDetectTrigger()III/b/f")
        return error
        # while self.reconnect_flag:
        #     time.sleep(0.1)
        # if self.GetSafetyCode() != 0:
        #     return self.GetSafetyCode()
        #
        # flag = True
        # while flag:
        #     try:
        #         error = self.robot.ConveryComDetectTrigger()
        #         # error = self.robot.ConveyorComDetectTrigger()
        #         flag = False
        #     except socket.error as e:
        #         flag = True
        #
        # return error

    """2025.04.14"""
    """3.8.2"""
    """
       @brief Arc tracking welder current feedback AI channel selection
       @param  [in] channel Channel; 0-extended AI0; 1-extended AI1; 2-extended AI2; 3-extended AI3; 4-control box AI0; 5-control box AI1
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceAIChannelCurrent(self, channel):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        channel = int(channel)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceAIChannelCurrent(channel)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief Arc tracking welder voltage feedback AI channel selection
       @param  [in] channel Channel; 0-extended AI0; 1-extended AI1; 2-extended AI2; 3-extended AI3; 4-control box AI0; 5-control box AI1
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceAIChannelVoltage(self, channel):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        channel = int(channel)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceAIChannelVoltage(channel)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief Arc tracking welder current feedback conversion parameters
       @param  [in] AILow AI channel lower limit, default value 0V, range [0-10V]
       @param  [in] AIHigh AI channel upper limit, default value 10V, range [0-10V]
       @param  [in] currentLow Welder current value corresponding to AI channel lower limit, default value 0V, range [0-200V]
       @param  [in] currentHigh Welder current value corresponding to AI channel upper limit, default value 100V, range [0-200V]
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceCurrentPara(self, AILow=0, AIHigh=10, currentLow=0, currentHigh=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        AILow = float(AILow)
        AIHigh = float(AIHigh)
        currentLow = float(currentLow)
        currentHigh = float(currentHigh)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceCurrentPara(AILow, AIHigh, currentLow, currentHigh)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief Arc tracking welder voltage feedback conversion parameters
       @param  [in] AILow AI channel lower limit, default value 0V, range [0-10V]
       @param  [in] AIHigh AI channel upper limit, default value 10V, range [0-10V]
       @param  [in] voltageLow Welder voltage value corresponding to AI channel lower limit, default value 0V, range [0-200V]
       @param  [in] voltageHigh Welder voltage value corresponding to AI channel upper limit, default value 100V, range [0-200V]
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceVoltagePara(self, AILow=0, AIHigh=10, voltageLow=0, voltageHigh=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        AILow = float(AILow)
        AIHigh = float(AIHigh)
        voltageLow = float(voltageLow)
        voltageHigh = float(voltageHigh)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceVoltagePara(AILow, AIHigh, voltageLow, voltageHigh)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.04.16"""
    """
       @brief Set welding voltage gradient start
       @param  [in] IOType Control type; 0-control box IO; 1-digital communication protocol (UDP); 2-digital communication protocol (ModbusTCP)
       @param  [in] voltageStart Start welding voltage (V)
       @param  [in] voltageEnd End welding voltage (V)
       @param  [in] AOIndex Control box AO port number (0-1)
       @param  [in] blend Whether to smooth 0-not smooth; 1-smooth
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltageGradualChangeStart(self, IOType, voltageStart, voltageEnd, AOIndex, blend):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        IOType = int(IOType)
        voltageStart = float(voltageStart)
        voltageEnd = float(voltageEnd)
        AOIndex = int(AOIndex)
        blend = int(blend)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetVoltageGradualChangeStart(IOType, voltageStart, voltageEnd, AOIndex, blend)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief Set welding voltage gradient end
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltageGradualChangeEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetVoltageGradualChangeEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief Set welding current gradient start
       @param  [in] IOType Control type; 0-control box IO; 1-digital communication protocol (UDP); 2-digital communication protocol (ModbusTCP)
       @param  [in] currentStart Start welding current (A)
       @param  [in] currentEnd End welding current (A)
       @param  [in] AOIndex Control box AO port number (0-1)
       @param  [in] blend Whether to smooth 0-not smooth; 1-smooth
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrentGradualChangeStart(self, IOType, currentStart, currentEnd, AOIndex, blend):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        IOType = int(IOType)
        currentStart = float(currentStart)
        currentEnd = float(currentEnd)
        AOIndex = int(AOIndex)
        blend = int(blend)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCurrentGradualChangeStart(IOType, currentStart, currentEnd, AOIndex, blend)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief Set welding current gradient end
       @return Error code Success - 0, Failure - error code
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrentGradualChangeEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCurrentGradualChangeEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.04.27"""
    """
       @brief Get SmartTool button state
       @return Error code Success - 0, Failure - error code
       @return Return value (returned on success) state SmartTool handle button state; (bit0: 0-communication normal; 1-communication disconnected; bit1-undo operation; bit2-clear program; bit3-A key; bit4-B key; bit5-C key; bit6-D key; bit7-E key; bit8-IO key; bit9-manual/auto; bit10-start)
    """

    @log_call
    @xmlrpc_timeout
    def GetSmarttoolBtnState(self):
        return 0,self.robot_state_pkg.smartToolState

    """2025.05.08"""
    """
       @brief Get extended axis coordinate frame
       @return Error code Success - 0, Failure - error code
       @return Return value (returned on success) coord Extended axis coordinate frame
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisGetCoord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                _error = self.robot.ExtAxisGetCoord()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """2025.06.06"""
    """   
    @brief  Get gripper activation state
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) fault 0-no error, 1-error present
    @return Return value (returned on success) gripper_active bit0~bit15 correspond to gripper number 0~15, bit=0 not activated, bit=1 activated
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperActivateStatus(self):
        return 0, self.robot_state_pkg.gripper_fault,self.robot_state_pkg.gripper_active

    """   
    @brief  Get gripper position
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) fault 0-no error, 1-error present
    @return Return value (returned on success) position Position percentage, range 0~100% 
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurPosition(self):
        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_position

    """   
    @brief  Get gripper current
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) fault 0-no error, 1-error present
    @return Return value (returned on success) current Current percentage, range 0~100%
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurCurrent(self):
        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_current

    """   
    @brief  Get gripper voltage
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) fault 0-no error, 1-error present
    @return Return value (returned on success) voltage Voltage, unit 0.1V
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperVoltage(self):
        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_voltage

    """   
    @brief  Get gripper temperature
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) fault 0-no error, 1-error present
    @return Return value (returned on success) temp Temperature, unit ℃
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperTemp(self):
        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_tmp

    """   
    @brief  Get gripper speed
    @param  [in] NULL
    @return error code success- 0, failure-error code
    @return Return value (returned on success) fault 0-no error, 1-error present
    @return Return value (returned on success) speed Speed percentage, range 0~100%
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurSpeed(self):
        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_speed

    """2025.06.24"""
    """3.8.3"""
    """
    @brief Set wide-voltage control box temperature and fan speed monitoring parameters
    @param  [in] enable 0-disable monitoring; 1-enable monitoring
    @param  [in] period Monitoring period (s), range 1-100
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetWideBoxTempFanMonitorParam(self, enable, period):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        enable = int(enable)
        period = int(period)
        flag = True
        while flag:
            try:
                error = self.robot.SetWideBoxTempFanMonitorParam(enable, period)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Get wide-voltage control box temperature and fan speed monitoring parameters
    @return error code success- 0, failure-error code
    @return Return value (returned on success) enable 0-disable monitoring; 1-enable monitoring
    @return Return value (returned on success) period Monitoring period (s), range 1-100
    """

    @log_call
    @xmlrpc_timeout
    def GetWideBoxTempFanMonitorParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetWideBoxTempFanMonitorParam()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2]
        return error, None, None

    """2025.07.04"""
    """3.8.4"""

    """
    @brief Set focus calibration point
    @param  [in] pointNum Focus calibration point number 1-8
    @param  [in] point Calibration point coordinates
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetFocusCalibPoint(self, pointNum, point):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointNum = int(pointNum)
        point = list(map(float, point))
        flag = True
        while flag:
            try:
                error = self.robot.SetFocusCalibPoint(pointNum,point[0],point[1],point[2],point[3],point[4],point[5])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Calculate focus calibration result
    @param  [in] pointNum Number of calibration points
    @return error code success- 0, failure-error code
    @return Return value (returned on success) resultPos Calibration result XYZ
    @return Return value (returned on success) accuracy Calibration accuracy error
    """

    @log_call
    @xmlrpc_timeout
    def ComputeFocusCalib(self, pointNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointNum = int(pointNum)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeFocusCalib(pointNum)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3]], _error[4]
        return error, None, None

    """
    @brief Enable focus following
    @param  [in] kp Proportional parameter, default 50.0
    @param  [in] kpredict Feedforward parameter, default 19.0
    @param  [in] aMax Maximum angular acceleration limit, default 1440°/s^2
    @param  [in] vMax Maximum angular velocity limit, default 180°/s
    @param  [in] type Lock X-axis orientation (0-reference input vector; 1-horizontal; 2-vertical)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FocusStart(self, kp=50.0, kpredic=19.0, aMax=1440, vMax=180, type=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        kp = float(kp)
        kpredic = float(kpredic)
        aMax = float(aMax)
        vMax = float(vMax)
        type = int(type)
        flag = True
        while flag:
            try:
                error = self.robot.FocusStart(kp, kpredic, aMax, vMax, type)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Stop focus following
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FocusEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.FocusEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Set focus coordinates
    @param  [in] pos Focus coordinates XYZ
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetFocusPosition(self, pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        pos = list(map(float, pos))
        flag = True
        while flag:
            try:
                error = self.robot.SetFocusPosition(pos[0],pos[1],pos[2])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.07.08"""
    """
    @brief Set encoder upgrade
    @param  [in] path Local upgrade package full path (D://zUP/XXXXX.bin)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetEncoderUpgrade(self, path):
        while self.reconnect_flag:
            time.sleep(0.1)
        path = str(path)
        flag = True
        while flag:
            try:
                error = self.robot.SetEncoderUpgrade(path)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Set joint firmware upgrade
    @param  [in] type Upgrade file type; 1-upgrade firmware; 2-upgrade slave configuration file
    @param  [in] path Local upgrade package full path (D://zUP/XXXXX.bin)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetJointFirmwareUpgrade(self, type, path):
        type = int(type)
        path = str(path)
        errcode = self.__FileUpLoad(2, path)
        if errcode == 0:
            file_name = "/tmp/" + os.path.basename(path)
            for joint_num in range(1, 7):
                errcode = self.SlaveFileWrite(1, joint_num, file_name)
                if errcode != 0:
                    return errcode
        return errcode

    """
    @brief Set control box firmware upgrade
    @param  [in] type Upgrade file type; 1-upgrade firmware; 2-upgrade slave configuration file
    @param  [in] path Local upgrade package full path (D://zUP/XXXXX.bin)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetCtrlFirmwareUpgrade(self, type, path):
        type = int(type)
        path = str(path)
        errcode = self.__FileUpLoad(2, path)
        if errcode == 0:
            file_name = "/tmp/" + os.path.basename(path)
            errcode = self.SlaveFileWrite(type, 0, file_name)
            return errcode
        return errcode

    """
    @brief Set end firmware upgrade
    @param  [in] type Upgrade file type; 1-upgrade firmware; 2-upgrade slave configuration file
    @param  [in] path Local upgrade package full path (D://zUP/XXXXX.bin)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetEndFirmwareUpgrade(self, type, path):
        type = int(type)
        path = str(path)
        errcode = self.__FileUpLoad(2, path)
        if errcode == 0:
            file_name = "/tmp/" + os.path.basename(path)
            errcode = self.SlaveFileWrite(type, 7, file_name)
            return errcode
        return errcode

    """
    @brief Joint full parameter configuration file upgrade
    @param  [in] path Local upgrade package full path (D://zUP/XXXXX.bin)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def JointAllParamUpgrade(self, path):
        path = str(path)
        errcode = self.__FileUpLoad(5, path)
        if errcode == 0:
            error = self.robot.JointAllParamUpgrade()
            return error
        return errcode

    """2025.07.21"""
    """3.8.5"""
    """
    @brief Laser sensor record point
    @param  [in] coordID Laser sensor coordinate frame
    @return error code success- 0, failure-error code
    @return Return value (returned on success) joint Laser sensor recognized point joint position
    @return Return value (returned on success) desc Laser sensor recognized point Cartesian position
    @return Return value (returned on success) exaxis Laser sensor recognized point extended axis position
    """

    @log_call
    @xmlrpc_timeout
    def LaserRecordPoint(self, coordID):
        while self.reconnect_flag:
            time.sleep(0.1)
        coordID = int(coordID)
        flag = True
        while flag:
            try:
                _error = self.robot.LaserRecordPoint(coordID,0,100)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            param_str = str(_error[1])
            par_s = param_str.split(',')
            if len(par_s) != 16:
                return -1, None, None, None
            return (error, [float(par_s[0]),float(par_s[1]),float(par_s[2]),float(par_s[3]),float(par_s[4]),float(par_s[5])],
                    [float(par_s[6]),float(par_s[7]),float(par_s[8]),float(par_s[9]),float(par_s[10]),float(par_s[11])],
                    [float(par_s[12]),float(par_s[13]),float(par_s[14]),float(par_s[15])])
        return error, None, None, None

    """2025.08.07"""
    """
    @brief Set extended axis synchronous motion strategy with robot
    @param  [in] strategy Strategy; 0-robot as master; 1-extended axis synchronized with robot
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetExAxisRobotPlan(self, strategy):
        while self.reconnect_flag:
            time.sleep(0.1)
        strategy = int(strategy)
        flag = True
        while flag:
            try:
                error = self.robot.SetExAxisRobotPlan(strategy)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.08.12"""
    """
    @brief Get slave board parameters
    @return error code success- 0, failure-error code
    @return Return value (returned on success) type  0-Ethercat, 1-CClink, 3-Ethercat, 4-EIP
    @return Return value (returned on success) version  Protocol version
    @return Return value (returned on success) connState  0-not connected 1-connected
    """

    @log_call
    @xmlrpc_timeout
    def GetFieldBusConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetFieldBusConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[2], _error[3], _error[4]
        return error, None, None, None

    """
    @brief Write slave DO
    @param  [in] DOIndex  DO number
    @param  [in] wirteNum  Number to write
    @param  [in] status[8] Values to write, up to 8
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveWriteDO(self, DOIndex, wirteNum, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        DOIndex = int(DOIndex)
        wirteNum = int(wirteNum)
        status = list(map(int, status))
        flag = True
        while flag:
            try:
                error = self.robot.FieldBusSlaveWriteDO(DOIndex, wirteNum, status)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Write slave AO
    @param  [in] AOIndex  AO number
    @param  [in] wirteNum  Number to write
    @param  [in] status[8] Values to write, up to 8
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveWriteAO(self, AOIndex, wirteNum, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        AOIndex = int(AOIndex)
        wirteNum = int(wirteNum)
        status = list(map(int, status))
        flag = True
        while flag:
            try:
                error = self.robot.FieldBusSlaveWriteAO(AOIndex, wirteNum, status)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Read slave DI
    @param  [in] DOIndex  DI number
    @param  [in] readeNum  Number to read
    @return error code success- 0, failure-error code
    @return Return value (returned on success) status[8] Values read, up to 8
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveReadDI(self, DOIndex, readeNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DOIndex = int(DOIndex)
        readeNum = int(readeNum)
        flag = True
        while flag:
            try:
                _error = self.robot.FieldBusSlaveReadDI(DOIndex, readeNum)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1:readeNum+1]
        return error, None

    """
    @brief Read slave AI
    @param  [in] AOIndex  AI number
    @param  [in] readeNum  Number to read
    @return error code success- 0, failure-error code
    @return Return value (returned on success) status[8] Values read, up to 8
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveReadAI(self, AOIndex, readeNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        AOIndex = int(AOIndex)
        readeNum = int(readeNum)
        flag = True
        while flag:
            try:
                _error = self.robot.FieldBusSlaveReadAI(AOIndex, readeNum)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1:readeNum + 1]
        return error, None

    """
    @brief Wait for extended DI input
    @param  [in] DIIndex DI number
    @param  [in] status 0-low level; 1-high level
    @param  [in] waitMs Maximum wait time (ms)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveWaitDI(self, DIIndex, status, waitMs):
        while self.reconnect_flag:
            time.sleep(0.1)
        DIIndex = int(DIIndex)
        status = int(status)
        waitMs = int(waitMs)
        flag = True
        while flag:
            try:
                error = self.robot.FieldBusSlaveWaitDI(DIIndex, status, waitMs)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Wait for extended AI input
    @param  [in] AIIndex AI number
    @param  [in] waitType 0-greater than; 1-less than
    @param  [in] value AI value
    @param  [in] waitMs Maximum wait time (ms)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveWaitAI(self, AIIndex, waitType, value, waitMs):
        while self.reconnect_flag:
            time.sleep(0.1)
        AIIndex = int(AIIndex)
        waitType = int(waitType)
        value = float(value)
        waitMs = int(waitMs)
        flag = True
        while flag:
            try:
                error = self.robot.FieldBusSlaveWaitAI(AIIndex, waitType, value, waitMs)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Control array suction cup
    @param  [in] slaveID Slave number
    @param  [in] len Length
    @param  [in] ctrlValue Control value 1-suction at maximum vacuum 2-suction at set vacuum 3-stop suction
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetSuckerCtrl(self, slaveID, len, ctrlValue):
        while self.reconnect_flag:
            time.sleep(0.1)
        slaveID = int(slaveID)
        len = int(len)
        ctrlValue = list(map(int, ctrlValue))
        flag = True
        while flag:
            try:
                error = self.robot.SetSuckerCtrl(slaveID, len, ctrlValue)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Get array suction cup state
    @param  [in] slaveID Slave number
    @return error code success- 0, failure-error code
    @return Return value (returned on success) state Suction state 0-release object 1-workpiece detected and suction successful 2-object not sucked 3-object detached
    @return Return value (returned on success) pressValue Current vacuum, unit kpa
    @return Return value (returned on success) error Current error code of suction cup
    """

    @log_call
    @xmlrpc_timeout
    def GetSuckerState(self, slaveID):
        while self.reconnect_flag:
            time.sleep(0.1)
        slaveID = int(slaveID)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSuckerState(slaveID)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2], _error[3]
        return error, None, None, None

    """
    @brief Wait for suction cup state
    @param  [in] slaveID Slave number
    @param  [in] state Suction state 0-release object 1-workpiece detected and suction successful 2-object not sucked 3-object detached
    @param  [in] ms Maximum wait time
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def WaitSuckerState(self, slaveID, state, ms):
        while self.reconnect_flag:
            time.sleep(0.1)
        slaveID = int(slaveID)
        state = int(state)
        ms = int(ms)
        flag = True
        while flag:
            try:
                error = self.robot.WaitSuckerState(slaveID, state, ms)
                flag = False
            except socket.error as e:
                flag = True
        return error


    """
    @brief Upload open protocol Lua file
    @param  [in] filePath Local open protocol lua file path name
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def OpenLuaUpload(self, filePath):
        filePath =str(filePath)
        errcode = self.__FileUpLoad(11, filePath)
        if errcode == 0:
            pos = filePath.rfind('/')
            if pos == -1:
                return RobotError.ERR_FILE_NAME
            filename = filePath[pos + 1:]  # Get the file name part
            error = self.robot.CtrlOpenLuaUpLoadCheck(filename)
            return error
        return errcode

    """3.8.6"""
    """2025.09.03"""
    """
    @brief Set load force detection before drag start
    @param  [in] flag 0-off; 1-on
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetTorqueDetectionSwitch(self, flag):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.SetTorqueDetectionSwitch(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True
        return error

    # """2025.09.03"""
    # """
    # @brief Set load force detection before drag start
    # @param  [in] flag 0-off; 1-on
    # @return Error code Success- 0, Failure-error code
    # """
    #
    # @log_call
    # @xmlrpc_timeout
    # def SetTorqueDetectionSwitch(self, flag):
    #     while self.reconnect_flag:
    #         time.sleep(0.1)
    #     flag = int(flag)
    #     flag_tmp = True
    #     while flag_tmp:
    #         try:
    #             error = self.robot.SetTorqueDetectionSwitch(flag)
    #             flag_tmp = False
    #         except socket.error as e:
    #             flag_tmp = True
    #     return error

    """2025.09.05"""
    """
    @brief Laser peripheral open/close function
    @param  [in] OnOff 0-close 1-open
    @param  [in] weldId Weld seam ID default 0
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingLaserOnOff(self, OnOff, weldId=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        OnOff = int(OnOff)
        weldId = int(weldId)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingLaserOnOff(OnOff, weldId)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser tracking start/stop function
    @param  [in] OnOff 0-stop 1-start
    @param  [in] coordId Laser peripheral tool coordinate frame number
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingTrackOnOff(self, OnOff, coordId):
        while self.reconnect_flag:
            time.sleep(0.1)
        OnOff = int(OnOff)
        coordId = int(coordId)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingTrackOnOff(OnOff, coordId)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser seam search - fixed direction
    @param  [in] direction 0-x+ 1-x- 2-y+ 3-y- 4-z+ 5-z-
    @param  [in] vel Speed unit %
    @param  [in] distance Maximum search distance unit mm
    @param  [in] timeout Search timeout unit ms
    @param  [in] posSensorNum Laser calibrated tool coordinate number
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingSearchStart_xyz(self, direction, vel, distance, timeout, posSensorNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        direction = int(direction)
        vel = int(vel)
        distance = int(distance)
        timeout = int(timeout)
        posSensorNum = int(posSensorNum)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSearchStart_xyz(direction, vel, distance, timeout, posSensorNum)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser seam search - arbitrary direction
    @param  [in] directionPoint xyz of the search input point, [x,y,z]
    @param  [in] vel Speed unit %
    @param  [in] distance Maximum search distance unit mm
    @param  [in] timeout Search timeout unit ms
    @param  [in] posSensorNum Laser calibrated tool coordinate number
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingSearchStart_point(self, directionPoint, vel, distance, timeout, posSensorNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        directionPoint = list(map(float, directionPoint))
        vel = int(vel)
        distance = int(distance)
        timeout = int(timeout)
        posSensorNum = int(posSensorNum)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSearchStart_point(6, vel, distance, timeout, posSensorNum, directionPoint[0], directionPoint[1], directionPoint[2])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser IP configuration
    @param  [in] ip IP address of the laser peripheral
    @param  [in] port Port number of the laser peripheral
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingSensorConfig(self, ip, port):
        while self.reconnect_flag:
            time.sleep(0.1)
        ip =str (ip)
        port = int(port)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSensorConfig(ip, port)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser peripheral sampling period configuration
    @param  [in] period Laser peripheral sampling period unit ms
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingSensorSamplePeriod(self, period):
        while self.reconnect_flag:
            time.sleep(0.1)
        period = int(period)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSensorSamplePeriod(period)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser peripheral driver load
    @param  [in] type Protocol type of laser peripheral driver 101-Ruiniu 102-Chuangxiang 103-Quanshi 104-Tongzhou 105-Aotai
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LoadPosSensorDriver(self, type):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        flag = True
        while flag:
            try:
                error = self.robot.LoadPosSensorDriver(type)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser peripheral driver unload
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def UnLoadPosSensorDriver(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.UnLoadPosSensorDriver()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser weld seam trajectory recording
    @param  [in] status 0-stop recording 1-real-time tracking  2-start recording
    @param  [in] delayTime Delay time unit ms
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserSensorRecord1(self, status, delayTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        delayTime = int(delayTime)
        flag = True
        while flag:
            try:
                error = self.robot.LaserSensorRecord1(status, delayTime)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser weld seam trajectory reproduction
    @param  [in] delayTime Delay time unit ms
    @param  [in] speed Speed unit %
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserSensorReplay(self, delayTime, speed):
        while self.reconnect_flag:
            time.sleep(0.1)
        delayTime = int(delayTime)
        speed = float(speed)
        flag = True
        while flag:
            try:
                error = self.robot.LaserSensorReplay(3, delayTime, speed)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser tracking reproduction
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveLTR(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MoveLTR(0)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Laser weld seam trajectory reproduction
    @param  [in] delayMode Mode 0-delay time 1-delay distance
    @param  [in] delayTime Delay time unit ms
    @param  [in] delayDisExAxisNum Extended axis number
    @param  [in] delayDis Delay distance unit mm
    @param  [in] sensitivePara Compensation sensitivity coefficient
    @param  [in] speed Speed unit %
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def LaserSensorRecordandReplay(self, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, speed):
        while self.reconnect_flag:
            time.sleep(0.1)
        delayMode = int(delayMode)
        delayTime = int(delayTime)
        delayDisExAxisNum = int(delayDisExAxisNum)
        delayDis = float(delayDis)
        sensitivePara = float(sensitivePara)
        speed = float(speed)
        flag = True
        while flag:
            try:
                error = self.robot.LaserSensorRecordandReplay(4, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, speed)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Move to the start point of weld seam recording
    @param  [in] moveType 0-PTP 1-LIN
    @param  [in] ovl Speed unit %
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveToLaserRecordStart(self, moveType, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        moveType = int(moveType)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToLaserRecordStart(moveType, ovl)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Move to the end point of weld seam recording
    @param  [in] moveType 0-PTP 1-LIN
    @param  [in] ovl Speed unit %
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveToLaserRecordEnd(self, moveType, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        moveType = int(moveType)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToLaserRecordEnd(moveType, ovl)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Move to laser sensor search point
    @param  [in] moveFlag Motion type: 0-PTP; 1-LIN
    @param  [in] ovl Speed scaling factor, 0-100
    @param  [in] dataFlag Weld seam buffer data selection: 0-execute planning data; 1-execute recorded data
    @param  [in] plateType Plate type: 0-corrugated plate; 1-corrugated board; 2-fence plate; 3-oil drum; 4-corrugated shell steel
    @param  [in] trackOffectType Laser sensor offset type: 0-no offset; 1-base coordinate frame offset; 2-tool coordinate frame offset; 3-laser sensor raw data offset
    @param  [in] offset Offset amount
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveToLaserSeamPos(self, moveFlag, ovl, dataFlag, plateType, trackOffectType, offset):
        while self.reconnect_flag:
            time.sleep(0.1)
        moveFlag = int(moveFlag)
        ovl = float(ovl)
        plateType = int(plateType)
        trackOffectType = int(trackOffectType)
        offset = list(map(float, offset))
        flag = True
        while flag:
            try:
                error = self.robot.MoveToLaserSeamPos([moveFlag, ovl, dataFlag, plateType, trackOffectType, offset[0], offset[1],offset[2], offset[3], offset[4],offset[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Get laser sensor search point coordinate information
    @param [in] trackOffectType Laser sensor offset type: 0-no offset; 1-base coordinate frame offset; 2-tool coordinate frame offset; 3-laser sensor raw data offset
    @param [in] offset Offset amount
    @return error code success- 0, failure-error code
    @return Return value (returned on success) jPos Joint position [deg]
    @return Return value (returned on success) descPos Cartesian position [mm]
    @return Return value (returned on success) tool Tool coordinate frame
    @return Return value (returned on success) user Work object coordinate frame
    @return Return value (returned on success) exaxis Extended axis position [mm]
    """

    @log_call
    @xmlrpc_timeout
    def GetLaserSeamPos(self, trackOffectType, offset):
        while self.reconnect_flag:
            time.sleep(0.1)
        trackOffectType = int(trackOffectType)
        offset = list(map(float, offset))
        flag = True
        while flag:
            try:
                _error = self.robot.GetLaserSeamPos([trackOffectType, offset[0], offset[1], offset[2], offset[3],offset[4], offset[5]])
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            paramStr = str(_error[1])
            # print(f"{paramStr}\n")

            parS = paramStr.split(',')
            if len(parS) != 20:
                return -1, None, None, None, None, None
            return (error, [float(parS[0]),float(parS[1]),float(parS[2]),float(parS[3]),float(parS[4]),float(parS[5])],
                    [float(parS[6]),float(parS[7]),float(parS[8]),float(parS[9]),float(parS[10]),float(parS[11])],
                    int(parS[12]), int(parS[13]),
                    [float(parS[16]),float(parS[17]),float(parS[18]),float(parS[19])])
        return error, None, None, None, None, None

    """2025.09.12"""
    """
    @brief Impedance start/stop control
    @param [in] status 0: off; 1-on
    @param [in] workSpace 0-joint space; 1-Cartesian space
    @param [in] forceThreshold Trigger force threshold (N)
    @param [in] m Mass parameter
    @param [in] b Damping parameter
    @param [in] k Stiffness parameter
    @param [in] maxV Maximum linear velocity (mm/s)
    @param [in] maxVA Maximum linear acceleration (mm/s2)
    @param [in] maxW Maximum angular velocity (deg/s)
    @param [in] maxWA Maximum angular acceleration (deg/s2)
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def ImpedanceControlStartStop(self, status, workSpace, forceThreshold, m, b, k, maxV, maxVA, maxW, maxWA):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        workSpace = int(workSpace)
        forceThreshold = list(map(float, forceThreshold))
        m = list(map(float, m))
        b = list(map(float, b))
        k = list(map(float, k))
        maxV = float(maxV)
        maxVA = float(maxVA)
        maxW = float(maxW)
        maxWA = float(maxWA)
        flag = True
        while flag:
            try:
                error = self.robot.ImpedanceControlStartStop([status, workSpace,
                                                     forceThreshold[0], forceThreshold[1], forceThreshold[2], forceThreshold[3], forceThreshold[4], forceThreshold[5]
                                                     ,m[0], m[1], m[2], m[3], m[4], m[5]
                                                     ,b[0], b[1], b[2], b[3], b[4], b[5]
                                                     ,k[0], k[1], k[2], k[3], k[4], k[5]
                                                     ,maxV, maxVA, maxW, maxWA])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.09.17"""
    """
    @brief Get tool coordinate frame by number
    @param [in] id Tool coordinate frame number
    @return error code success- 0, failure-error code
    @return Return value (returned on success) coord Coordinate frame value
    """

    @log_call
    @xmlrpc_timeout
    def GetToolCoordWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetToolCoordWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """
    @brief Get work object coordinate frame by number
    @param [in] id Work object coordinate frame number
    @return error code success- 0, failure-error code
    @return Return value (returned on success) coord Coordinate frame value
    """

    @log_call
    @xmlrpc_timeout
    def GetWObjCoordWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetWObjCoordWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """
    @brief Get external tool coordinate frame by number
    @param [in] id External tool coordinate frame number
    @return error code success- 0, failure-error code
    @return Return value (returned on success) coord Coordinate frame value
    """

    @log_call
    @xmlrpc_timeout
    def GetExToolCoordWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetExToolCoordWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """
    @brief Get extended axis coordinate frame by number
    @param [in] id Extended axis coordinate frame number
    @return error code success- 0, failure-error code
    @return Return value (returned on success) coord Coordinate frame value
    """

    @log_call
    @xmlrpc_timeout
    def GetExAxisCoordWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetExAxisCoordWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """
    @brief Get payload mass and center of gravity by number
    @param [in] id Extended axis coordinate frame number
    @return error code success- 0, failure-error code
    @return Return value (returned on success) weight Payload mass
    @return Return value (returned on success) cog Payload center of gravity
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetPayloadWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetTargetPayloadWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, _error[1], [_error[2], _error[3], _error[4]]
        return error, None, None

    """
    @brief Get current tool coordinate frame
    @return error code success- 0, failure-error code
    @return Return value (returned on success) coord Coordinate frame value
    """

    @log_call
    @xmlrpc_timeout
    def GetCurToolCoord(self):
        return 0, [self.robot_state_pkg.toolCoord[0],
                   self.robot_state_pkg.toolCoord[1],
                   self.robot_state_pkg.toolCoord[2],
                   self.robot_state_pkg.toolCoord[3],
                   self.robot_state_pkg.toolCoord[4],
                   self.robot_state_pkg.toolCoord[5]]

    """
    @brief Get current work object coordinate frame
    @return error code success- 0, failure-error code
    @return Return value (returned on success) coord Coordinate frame value
    """

    @log_call
    @xmlrpc_timeout
    def GetCurWObjCoord(self):
        return 0, [self.robot_state_pkg.wobjCoord[0],
                   self.robot_state_pkg.wobjCoord[1],
                   self.robot_state_pkg.wobjCoord[2],
                   self.robot_state_pkg.wobjCoord[3],
                   self.robot_state_pkg.wobjCoord[4],
                   self.robot_state_pkg.wobjCoord[5]]

    """
    @brief Get current external tool coordinate frame
    @return error code success- 0, failure-error code
    @return Return value (returned on success) coord Coordinate frame value
    """

    @log_call
    @xmlrpc_timeout
    def GetCurExToolCoord(self):
        return 0, [self.robot_state_pkg.extoolCoord[0],
                   self.robot_state_pkg.extoolCoord[1],
                   self.robot_state_pkg.extoolCoord[2],
                   self.robot_state_pkg.extoolCoord[3],
                   self.robot_state_pkg.extoolCoord[4],
                   self.robot_state_pkg.extoolCoord[5]]

    """
    @brief Get current extended axis coordinate frame
    @return error code success- 0, failure-error code
    @return Return value (returned on success) coord Coordinate frame value
    """

    @log_call
    @xmlrpc_timeout
    def GetCurExAxisCoord(self):
        return 0, [self.robot_state_pkg.exAxisCoord[0],
                   self.robot_state_pkg.exAxisCoord[1],
                   self.robot_state_pkg.exAxisCoord[2],
                   self.robot_state_pkg.exAxisCoord[3],
                   self.robot_state_pkg.exAxisCoord[4],
                   self.robot_state_pkg.exAxisCoord[5]]

    """2025.09.18"""
    """
    @brief Set custom weaving parameters
    @param [in] id Custom weaving number: 0-2
    @param [in] pointNum Number of weaving points 0-10
    @param [in] point Moving endpoint data x,y,z
    @param [in] stayTime Weaving dwell time ms
    @param [in] frequency Weaving frequency Hz
    @param [in] incStayType Wait mode: 0-cycle excludes wait time; 1-cycle includes wait time
    @param [in] stationary Weaving position wait: 0-continue motion during wait time; 1-position stationary during wait time
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def CustomWeaveSetPara(self, id, pointNum, point, stayTime, frequency, incStayType, stationary):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        pointNum = int(pointNum)
        point = list(map(float, point))
        stayTime = list(map(float,stayTime))
        frequency = float(frequency)
        incStayType = int(incStayType)
        stationary = int(stationary)
        flag = True
        while flag:
            try:
                error = self.robot.CustomWeaveSetPara([id, pointNum,
                                                       point[0],point[1],point[2],point[3],point[4],point[5],point[6],point[7],point[8],point[9],
                                                       point[10],point[11],point[12],point[13],point[14],point[15],point[16],point[17],point[18],point[19],
                                                       point[20],point[21],point[22],point[23],point[24],point[25],point[26],point[27],point[28],point[29],
                                                       stayTime[0],stayTime[1],stayTime[2],stayTime[3],stayTime[4],stayTime[5],stayTime[6],stayTime[7],stayTime[8],stayTime[9],
                                                       frequency,incStayType,stationary])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Get custom weaving parameters
    @param [in] id Custom weaving number: 0-2
    @return error code success- 0, failure-error code
    @return Return value (returned on success) pointNum Number of weaving points 0-10
    @return Return value (returned on success) point Moving endpoint data x,y,z
    @return Return value (returned on success) stayTime Weaving dwell time ms
    @return Return value (returned on success) frequency Weaving frequency Hz
    @return Return value (returned on success) incStayType Wait mode: 0-cycle excludes wait time; 1-cycle includes wait time
    @return Return value (returned on success) stationary Weaving position wait: 0-continue motion during wait time; 1-position stationary during wait time
    """

    @log_call
    @xmlrpc_timeout
    def CustomWeaveGetPara(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.CustomWeaveGetPara(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            paramStr = str(_error[1])
            # print(f"{paramStr}\n")

            parS = paramStr.split(',')
            if len(parS) != 44:
                return -1, None, None, None, None, None, None
            return (error, int(parS[0]), [float(parS[1]),  float(parS[2]),  float(parS[3]),
                                          float(parS[4]),  float(parS[5]),  float(parS[6]),
                                          float(parS[7]),  float(parS[8]),  float(parS[9]),
                                          float(parS[10]), float(parS[11]), float(parS[12]),
                                          float(parS[13]), float(parS[14]), float(parS[15]),
                                          float(parS[16]), float(parS[17]), float(parS[18]),
                                          float(parS[19]), float(parS[20]), float(parS[21]),
                                          float(parS[22]), float(parS[23]), float(parS[24]),
                                          float(parS[25]), float(parS[26]), float(parS[27]),
                                          float(parS[28]), float(parS[29]), float(parS[30])],
                                          [float(parS[31]), float(parS[32]), float(parS[33]),
                                           float(parS[34]), float(parS[35]), float(parS[36]),
                                           float(parS[37]), float(parS[38]), float(parS[39]),
                                           float(parS[40])],
                                          float(parS[41]),int(parS[42]), int(parS[43]))
        return error, None, None, None, None, None, None

    """2025.09.19"""
    """
    @brief Robot operating system upgrade (LA control box)
    @param [in] filePath Full path of operating system upgrade package
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def KernelUpgrade(self, filePath):
        filePath = str(filePath)
        errcode = self.__FileUpLoad(6, filePath)
        if errcode == 0:
            try:
                result = self.robot.KernelUpgrade()
                if result is not None and hasattr(result, '__len__') and len(result) == 0:
                    # print("Warning: kernel upgrade call succeeded, but returned empty data")
                    return 0
                return result if result is not None else 0
            except xmlrpc.client.Fault as e:
                if "array has only 0 items" in str(e):
                    # print("Kernel upgrade command has been sent, but the server returned an empty response")
                    return 0
                else:
                    # print(f"Kernel upgrade failed: {e}")
                    return -1
        return errcode

    """
    @brief Get robot operating system upgrade result (LA control box)
    @return error code success- 0, failure-error code
    @return Return value (returned on success) result Upgrade result: 0: success; -1: failure
    """

    @log_call
    @xmlrpc_timeout
    def GetKernelUpgradeResult(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetKernelUpgradeResult()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, _error[1]
        return error, None

    """3.8.7"""
    """2025.10.11"""
    """
    @brief Enable joint torque sensor sensitivity calibration function
    @param [in] status 0-off; 1-on
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def JointSensitivityEnable(self, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.JointSensitivityEnable([status])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Get joint torque sensor sensitivity calibration result
    @return error code success- 0, failure-error code
    @return Return value (returned on success) calibResult j1~j6 joint sensitivity [0-1]
    @return Return value (returned on success) linearityn j1~j6 joint linearity [0-1]
    """

    @log_call
    @xmlrpc_timeout
    def JointSensitivityCalibration(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.JointSensitivityCalibration()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]], [_error[7], _error[8], _error[9], _error[10], _error[11], _error[12]]
        return error, None, None

    """
    @brief Joint torque sensor sensitivity data acquisition
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def JointSensitivityCollect(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.JointSensitivityCollect()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.10.15"""
    """
    @brief Clear motion command queue
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MotionQueueClear(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MotionQueueClear()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Get error frame counts of the robot's 8 slave ports
    @return error code success- 0, failure-error code
    @return Return value (returned on success) inRecvErr Input receive error frame count
    @return Return value (returned on success) inCRCErr Input CRC error frame count
    @return Return value (returned on success) inTransmitErr Input forwarding error frame count
    @return Return value (returned on success) inLinkErr Input link error frame count
    @return Return value (returned on success) outRecvErr Output receive error frame count
    @return Return value (returned on success) outCRCErr Output CRC error frame count
    @return Return value (returned on success) outTransmitErr Output forwarding error frame count
    @return Return value (returned on success) outLinkErr Output link error frame count
    """

    @log_call
    @xmlrpc_timeout
    def GetSlavePortErrCounter(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlavePortErrCounter()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            paramStr = str(_error[1])
            # print(f"{paramStr}\n")

            parS = paramStr.split(',')
            if len(parS) != 64:
                return -1, None, None, None, None, None, None, None, None
            return (error,
                    [int(parS[0]), int(parS[4]), int(parS[8]), int(parS[12]), int(parS[16]), int(parS[20]), int(parS[24]), int(parS[28])],
                    [int(parS[1]), int(parS[5]), int(parS[9]), int(parS[13]), int(parS[17]), int(parS[21]), int(parS[25]), int(parS[29])],
                    [int(parS[2]), int(parS[6]), int(parS[10]), int(parS[14]), int(parS[18]), int(parS[22]), int(parS[26]), int(parS[30])],
                    [int(parS[3]), int(parS[7]), int(parS[11]), int(parS[15]), int(parS[19]), int(parS[23]), int(parS[27]), int(parS[31])],
                    [int(parS[32]), int(parS[36]), int(parS[40]), int(parS[44]), int(parS[48]), int(parS[52]), int(parS[56]), int(parS[60])],
                    [int(parS[33]), int(parS[37]), int(parS[41]), int(parS[45]), int(parS[49]), int(parS[53]), int(parS[57]), int(parS[61])],
                    [int(parS[34]), int(parS[38]), int(parS[42]), int(parS[46]), int(parS[50]), int(parS[54]), int(parS[58]), int(parS[62])],
                    [int(parS[35]), int(parS[39]), int(parS[43]), int(parS[47]), int(parS[51]), int(parS[55]), int(parS[59]), int(parS[63])])
        return error, None, None, None, None, None, None, None, None

    """
    @brief Clear slave port error frames
    @param [in] slaveID Slave number 0~7
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SlavePortErrCounterClear(self, slaveID):
        while self.reconnect_flag:
            time.sleep(0.1)
        slaveID = int(slaveID)
        flag = True
        while flag:
            try:
                error = self.robot.SlavePortErrCounterClear(slaveID)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Set velocity feedforward coefficient for each axis
    @param [in] radio Velocity feedforward coefficient for each axis
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetVelFeedForwardRatio(self, radio):
        while self.reconnect_flag:
            time.sleep(0.1)
        radio = list(map(float,radio))
        flag = True
        while flag:
            try:
                error = self.robot.SetVelFeedForwardRatio([radio[0],radio[1],radio[2],radio[3],radio[4],radio[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Get velocity feedforward coefficient for each axis
    @return error code success- 0, failure-error code
    @return Return value (returned on success) radio Velocity feedforward coefficient for each axis
    """

    @log_call
    @xmlrpc_timeout
    def GetVelFeedForwardRatio(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetVelFeedForwardRatio()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3]), float(_error[4]), float(_error[5]), float(_error[6])]
        return error, None

    """
    @brief Generate robot MCU log
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def RobotMCULogCollect(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.RobotMCULogCollect()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """3.9.0"""
    """2025.11.03"""
    """
    @brief Move to intersection line start point
    @param  [in] Required parameter mainPoint Cartesian poses of the 6 teaching points of the main pipe
    @param  [in] Required parameter piecePoint Cartesian poses of the 6 teaching points of the branch pipe
    @param  [in] Required parameter tool Tool coordinate frame number
    @param  [in] Required parameter wobj Work object coordinate frame number
    @param  [in] Required parameter vel Velocity percentage
    @param  [in] Required parameter acc Acceleration percentage 
    @param  [in] Required parameter ovl Velocity scaling factor
    @param  [in] Required parameter oacc Acceleration scaling factor
    @param  [in] Required parameter moveType Motion type; 0-PTP; 1-LIN
    @param  [in] Default parameter mainExaxisPos Extended axis positions of the 6 teaching points of the main pipe, default [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] Default parameter pieceExaxisPos Extended axis positions of the 6 teaching points of the splicing pipe, default [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] Default parameter extAxisFlag Whether to enable extended axis; 0-disable; 1-enable
    @param  [in] Default parameter exaxisPos Start point extended axis position [0.0,0.0,0.0,0.0]
    @param  [in] Default parameter moveDirection Motion direction; 0-clockwise; 1-counterclockwise
    @param  [in] Default parameter offset Offset amount
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def MoveToIntersectLineStart(self, mainPoint, piecePoint, tool, wobj, vel, acc, ovl, oacc, moveType,mainExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],
                                 pieceExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],extAxisFlag=0,
                                 exaxisPos=[0.0,0.0,0.0,0.0],moveDirection=0,offset=[0.0,0.0,0.0,0.0,0.0,0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        mainPoint0 = list(map(float, mainPoint[0]))
        mainPoint1 = list(map(float, mainPoint[1]))
        mainPoint2 = list(map(float, mainPoint[2]))
        mainPoint3 = list(map(float, mainPoint[3]))
        mainPoint4 = list(map(float, mainPoint[4]))
        mainPoint5 = list(map(float, mainPoint[5]))
        mainExaxisPos0 = list(map(float, mainExaxisPos[0]))
        mainExaxisPos1 = list(map(float, mainExaxisPos[1]))
        mainExaxisPos2 = list(map(float, mainExaxisPos[2]))
        mainExaxisPos3 = list(map(float, mainExaxisPos[3]))
        mainExaxisPos4 = list(map(float, mainExaxisPos[4]))
        mainExaxisPos5 = list(map(float, mainExaxisPos[5]))
        piecePoint0 = list(map(float, piecePoint[0]))
        piecePoint1 = list(map(float, piecePoint[1]))
        piecePoint2 = list(map(float, piecePoint[2]))
        piecePoint3 = list(map(float, piecePoint[3]))
        piecePoint4 = list(map(float, piecePoint[4]))
        piecePoint5 = list(map(float, piecePoint[5]))
        pieceExaxisPos0 = list(map(float, pieceExaxisPos[0]))
        pieceExaxisPos1 = list(map(float, pieceExaxisPos[1]))
        pieceExaxisPos2 = list(map(float, pieceExaxisPos[2]))
        pieceExaxisPos3 = list(map(float, pieceExaxisPos[3]))
        pieceExaxisPos4 = list(map(float, pieceExaxisPos[4]))
        pieceExaxisPos5 = list(map(float, pieceExaxisPos[5]))
        extAxisFlag = int(extAxisFlag)
        exaxisPos = list(map(float, exaxisPos))

        tool = int(tool)
        wobj = int(wobj)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        oacc = float(oacc)
        moveType = int(moveType)
        moveDirection = int(moveDirection)
        offset = list(map(float, offset))
        flag = True
        while flag:
            try:
                error = self.robot.MoveToIntersectLineStart([mainPoint0[0], mainPoint0[1], mainPoint0[2], mainPoint0[3], mainPoint0[4], mainPoint0[5],
                                                             mainPoint1[0], mainPoint1[1], mainPoint1[2], mainPoint1[3], mainPoint1[4], mainPoint1[5],
                                                             mainPoint2[0], mainPoint2[1], mainPoint2[2], mainPoint2[3], mainPoint2[4], mainPoint2[5],
                                                             mainPoint3[0], mainPoint3[1], mainPoint3[2], mainPoint3[3], mainPoint3[4], mainPoint3[5],
                                                             mainPoint4[0], mainPoint4[1], mainPoint4[2], mainPoint4[3], mainPoint4[4], mainPoint4[5],
                                                             mainPoint5[0], mainPoint5[1], mainPoint5[2], mainPoint5[3], mainPoint5[4], mainPoint5[5],
                                                             mainExaxisPos0[0], mainExaxisPos0[1], mainExaxisPos0[2], mainExaxisPos0[3],
                                                             mainExaxisPos1[0], mainExaxisPos1[1], mainExaxisPos1[2], mainExaxisPos1[3],
                                                             mainExaxisPos2[0], mainExaxisPos2[1], mainExaxisPos2[2], mainExaxisPos2[3],
                                                             mainExaxisPos3[0], mainExaxisPos3[1], mainExaxisPos3[2], mainExaxisPos3[3],
                                                             mainExaxisPos4[0], mainExaxisPos4[1], mainExaxisPos4[2], mainExaxisPos4[3],
                                                             mainExaxisPos5[0], mainExaxisPos5[1], mainExaxisPos5[2], mainExaxisPos5[3],
                                                             piecePoint0[0],piecePoint0[1],piecePoint0[2],piecePoint0[3],piecePoint0[4],piecePoint0[5],
                                                             piecePoint1[0],piecePoint1[1],piecePoint1[2],piecePoint1[3],piecePoint1[4],piecePoint1[5],
                                                             piecePoint2[0],piecePoint2[1],piecePoint2[2],piecePoint2[3],piecePoint2[4],piecePoint2[5],
                                                             piecePoint3[0],piecePoint3[1],piecePoint3[2],piecePoint3[3],piecePoint3[4],piecePoint3[5],
                                                             piecePoint4[0],piecePoint4[1],piecePoint4[2],piecePoint4[3],piecePoint4[4],piecePoint4[5],
                                                             piecePoint5[0],piecePoint5[1],piecePoint5[2],piecePoint5[3],piecePoint5[4],piecePoint5[5],
                                                             pieceExaxisPos0[0], pieceExaxisPos0[1], pieceExaxisPos0[2], pieceExaxisPos0[3],
                                                             pieceExaxisPos1[0], pieceExaxisPos1[1], pieceExaxisPos1[2], pieceExaxisPos1[3],
                                                             pieceExaxisPos2[0], pieceExaxisPos2[1], pieceExaxisPos2[2], pieceExaxisPos2[3],
                                                             pieceExaxisPos3[0], pieceExaxisPos3[1], pieceExaxisPos3[2], pieceExaxisPos3[3],
                                                             pieceExaxisPos4[0], pieceExaxisPos4[1], pieceExaxisPos4[2], pieceExaxisPos4[3],
                                                             pieceExaxisPos5[0], pieceExaxisPos5[1], pieceExaxisPos5[2], pieceExaxisPos5[3],
                                                             extAxisFlag,
                                                             exaxisPos[0], exaxisPos[1], exaxisPos[2], exaxisPos[3],
                                                             tool, wobj, vel, acc, ovl, oacc, moveType,moveDirection,
                                                             offset[0],offset[1],offset[2],offset[3],offset[4],offset[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief Intersection line motion
    @param  [in] Required parameter mainPoint Cartesian poses of the 6 teaching points of the main pipe
    @param  [in] Required parameter piecePoint Cartesian poses of the 6 teaching points of the branch pipe
    @param  [in] Required parameter tool Tool coordinate frame number
    @param  [in] Default parameter wobj Work object coordinate frame number
    @param  [in] Default parameter vel Velocity percentage
    @param  [in] Default parameter acc Acceleration percentage 
    @param  [in] Default parameter ovl Velocity scaling factor
    @param  [in] Default parameter oacc Acceleration scaling factor
    @param  [in] Default parameter moveDirection Motion direction; 0-clockwise; 1-counterclockwise
    @param  [in] Default parameter mainExaxisPos Extended axis positions of the 6 teaching points of the main pipe, default [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] Default parameter pieceExaxisPos Extended axis positions of the 6 teaching points of the splicing pipe, default [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] Default parameter extAxisFlag Whether to enable extended axis; 0-disable; 1-enable
    @param  [in] Default parameter exaxisPos Start point extended axis position [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] Default parameter offset Offset amount
    @return error code success- 0, failure-error code
    """


    @log_call
    @xmlrpc_timeout
    def MoveIntersectLine(self, mainPoint, piecePoint, tool, wobj, vel, acc, ovl, oacc, moveDirection,mainExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],
                                 pieceExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],extAxisFlag=0,
                                 exaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],offset=[0.0,0.0,0.0,0.0,0.0,0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        mainPoint0 = list(map(float, mainPoint[0]))
        mainPoint1 = list(map(float, mainPoint[1]))
        mainPoint2 = list(map(float, mainPoint[2]))
        mainPoint3 = list(map(float, mainPoint[3]))
        mainPoint4 = list(map(float, mainPoint[4]))
        mainPoint5 = list(map(float, mainPoint[5]))
        mainExaxisPos0 = list(map(float, mainExaxisPos[0]))
        mainExaxisPos1 = list(map(float, mainExaxisPos[1]))
        mainExaxisPos2 = list(map(float, mainExaxisPos[2]))
        mainExaxisPos3 = list(map(float, mainExaxisPos[3]))
        mainExaxisPos4 = list(map(float, mainExaxisPos[4]))
        mainExaxisPos5 = list(map(float, mainExaxisPos[5]))
        piecePoint0 = list(map(float, piecePoint[0]))
        piecePoint1 = list(map(float, piecePoint[1]))
        piecePoint2 = list(map(float, piecePoint[2]))
        piecePoint3 = list(map(float, piecePoint[3]))
        piecePoint4 = list(map(float, piecePoint[4]))
        piecePoint5 = list(map(float, piecePoint[5]))
        pieceExaxisPos0 = list(map(float, pieceExaxisPos[0]))
        pieceExaxisPos1 = list(map(float, pieceExaxisPos[1]))
        pieceExaxisPos2 = list(map(float, pieceExaxisPos[2]))
        pieceExaxisPos3 = list(map(float, pieceExaxisPos[3]))
        pieceExaxisPos4 = list(map(float, pieceExaxisPos[4]))
        pieceExaxisPos5 = list(map(float, pieceExaxisPos[5]))
        extAxisFlag = int(extAxisFlag)
        exaxisPos0 = list(map(float, exaxisPos[0]))
        exaxisPos1 = list(map(float, exaxisPos[1]))
        exaxisPos2 = list(map(float, exaxisPos[2]))
        exaxisPos3 = list(map(float, exaxisPos[3]))
        tool = int(tool)
        wobj = int(wobj)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        oacc = float(oacc)
        moveDirection = int(moveDirection)
        offset = list(map(float, offset))
        flag = True
        while flag:
            try:
                error = self.robot.MoveIntersectLine(
                    [mainPoint0[0], mainPoint0[1], mainPoint0[2], mainPoint0[3], mainPoint0[4], mainPoint0[5],
                                                             mainPoint1[0], mainPoint1[1], mainPoint1[2], mainPoint1[3], mainPoint1[4], mainPoint1[5],
                                                             mainPoint2[0], mainPoint2[1], mainPoint2[2], mainPoint2[3], mainPoint2[4], mainPoint2[5],
                                                             mainPoint3[0], mainPoint3[1], mainPoint3[2], mainPoint3[3], mainPoint3[4], mainPoint3[5],
                                                             mainPoint4[0], mainPoint4[1], mainPoint4[2], mainPoint4[3], mainPoint4[4], mainPoint4[5],
                                                             mainPoint5[0], mainPoint5[1], mainPoint5[2], mainPoint5[3], mainPoint5[4], mainPoint5[5],
                                                             mainExaxisPos0[0], mainExaxisPos0[1], mainExaxisPos0[2], mainExaxisPos0[3],
                                                             mainExaxisPos1[0], mainExaxisPos1[1], mainExaxisPos1[2], mainExaxisPos1[3],
                                                             mainExaxisPos2[0], mainExaxisPos2[1], mainExaxisPos2[2], mainExaxisPos2[3],
                                                             mainExaxisPos3[0], mainExaxisPos3[1], mainExaxisPos3[2], mainExaxisPos3[3],
                                                             mainExaxisPos4[0], mainExaxisPos4[1], mainExaxisPos4[2], mainExaxisPos4[3],
                                                             mainExaxisPos5[0], mainExaxisPos5[1], mainExaxisPos5[2], mainExaxisPos5[3],
                                                             piecePoint0[0],piecePoint0[1],piecePoint0[2],piecePoint0[3],piecePoint0[4],piecePoint0[5],
                                                             piecePoint1[0],piecePoint1[1],piecePoint1[2],piecePoint1[3],piecePoint1[4],piecePoint1[5],
                                                             piecePoint2[0],piecePoint2[1],piecePoint2[2],piecePoint2[3],piecePoint2[4],piecePoint2[5],
                                                             piecePoint3[0],piecePoint3[1],piecePoint3[2],piecePoint3[3],piecePoint3[4],piecePoint3[5],
                                                             piecePoint4[0],piecePoint4[1],piecePoint4[2],piecePoint4[3],piecePoint4[4],piecePoint4[5],
                                                             piecePoint5[0],piecePoint5[1],piecePoint5[2],piecePoint5[3],piecePoint5[4],piecePoint5[5],
                                                             pieceExaxisPos0[0], pieceExaxisPos0[1], pieceExaxisPos0[2], pieceExaxisPos0[3],
                                                             pieceExaxisPos1[0], pieceExaxisPos1[1], pieceExaxisPos1[2], pieceExaxisPos1[3],
                                                             pieceExaxisPos2[0], pieceExaxisPos2[1], pieceExaxisPos2[2], pieceExaxisPos2[3],
                                                             pieceExaxisPos3[0], pieceExaxisPos3[1], pieceExaxisPos3[2], pieceExaxisPos3[3],
                                                             pieceExaxisPos4[0], pieceExaxisPos4[1], pieceExaxisPos4[2], pieceExaxisPos4[3],
                                                             pieceExaxisPos5[0], pieceExaxisPos5[1], pieceExaxisPos5[2], pieceExaxisPos5[3],
                                                             extAxisFlag,
                                                             exaxisPos0[0], exaxisPos0[1], exaxisPos0[2], exaxisPos0[3],
                                                             exaxisPos1[0], exaxisPos1[1], exaxisPos1[2], exaxisPos1[3],
                                                             exaxisPos2[0], exaxisPos2[1], exaxisPos2[2], exaxisPos2[3],
                                                             exaxisPos3[0], exaxisPos3[1], exaxisPos3[2], exaxisPos3[3],
                     tool, wobj, vel, acc, ovl, oacc, moveDirection,
                                                             offset[0],offset[1],offset[2],offset[3],offset[4],offset[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """3.9.0"""
    """2025.11.20"""
    """
    @brief Get joint torque sensor hysteresis error
    @return error code success- 0, failure-error code
    @return Return value (returned on success) hysteresisError j1~j6 joint hysteresis error
    """

    @log_call
    @xmlrpc_timeout
    def JointHysteresisError(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.JointHysteresisError()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3]), float(_error[4]), float(_error[5]), float(_error[6])]
        return error, None


    """
    @brief Get joint torque sensor repeatability
    @return error code success- 0, failure-error code
    @return Return value (returned on success) repeatability j1~j6 joint repeatability
    """

    @log_call
    @xmlrpc_timeout
    def JointRepeatability(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.JointRepeatability()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3]), float(_error[4]), float(_error[5]), float(_error[6])]
        return error, None

    """
    @brief Set joint force sensor parameters
    @param  [in] Required parameter M J1-J6 mass coefficients []
    @param  [in] Required parameter B J1-J6 damping coefficients []
    @param  [in] Required parameter K J1-J6 stiffness coefficients []
    @param  [in] Default parameter threshold Force control threshold, Nm
    @param  [in] Default parameter sensitivity Sensitivity, Nm/V, []
    @param  [in] Default parameter setZeroFlag Function enable flag; 0-off; 1-on; 2-record zero point at position 1; 3-record zero point at position 2
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SetAdmittanceParams(self, M, B, K, threshold, sensitivity, setZeroFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        M = list(map(float, M))
        B = list(map(float, B))
        K = list(map(float, K))
        threshold = list(map(float, threshold))
        sensitivity = list(map(float, sensitivity))
        setZeroFlag = int(setZeroFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetAdmittanceParams(
                    [M[0], M[1], M[2], M[3], M[4], M[5],
                     B[0], B[1], B[2], B[3], B[4], B[5],
                     K[0], K[1], K[2], K[3], K[4], K[5],
                     threshold[0], threshold[1], threshold[2], threshold[3], threshold[4], threshold[5],
                     sensitivity[0], sensitivity[1], sensitivity[2], sensitivity[3], sensitivity[4], sensitivity[5],
                     setZeroFlag])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """3.9.1"""
    """2025.12.01"""
    """
    @brief Enable torque compensation function and compensation coefficient
    @param  [in] Required parameter status Switch, 0-off; 1-on
    @param  [in] Required parameter torqueCoeff J1-J6 torque compensation coefficient [0-1]
    @return error code success- 0, failure-error code
    """

    @log_call
    @xmlrpc_timeout
    def SerCoderCompenParams(self, status, torqueCoeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        torqueCoeff = list(map(float, torqueCoeff))
        flag = True
        while flag:
            try:
                error = self.robot.SerCoderCompenParams([status,torqueCoeff[0],torqueCoeff[1],torqueCoeff[2],torqueCoeff[3],torqueCoeff[4],torqueCoeff[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error