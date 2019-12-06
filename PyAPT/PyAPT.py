'''APT Motor Controller for Thorlabs
Nicolas Abril - nicolas.abril@protonmail.ch

Adapted for Python 3 from:
https://github.com/mcleu/PyAPT

Changes include:
  * More exposed functions
  * Some custom exceptions
  * pep-8 style
  * Short names for functions
  * Enums
'''
import os
from ctypes import c_float, c_long, create_string_buffer, pointer, windll
from enum import Enum, Flag

DLL_PATH = '../APTDLLPack/DLL/x64/APT.dll'

class MotorConnectionError(Exception):
    '''Raised when a connection to a motor fails, or a motor is used without connecting'''

class DllNotFoundError(Exception):
    '''Raised when APT.dll is not found'''

class HWType(Enum):
    '''The different supported models of motors'''
    BSC001 = 11	# 1 Ch benchtop stepper driver
    BSC101 = 12	# 1 Ch benchtop stepper driver
    BSC002 = 13	# 2 Ch benchtop stepper driver
    BDC101 = 14	# 1 Ch benchtop DC servo driver
    SCC001 = 21	# 1 Ch stepper driver card (used within BSC102,103 units)
    DCC001 = 22	# 1 Ch DC servo driver card (used within BDC102,103 units)
    ODC001 = 24	# 1 Ch DC servo driver cube
    OST001 = 25	# 1 Ch stepper driver cube
    MST601 = 26	# 2 Ch modular stepper driver module
    TST001 = 29	# 1 Ch Stepper driver T-Cube
    TDC001 = 31	# 1 Ch DC servo driver T-Cube
    LTSXXX = 42	# LTS300/LTS150 Long Travel Integrated Driver/Stages
    L490MZ = 43	# L490MZ Integrated Driver/Labjack
    BBD10X = 44	# 1/2/3 Ch benchtop brushless DC servo driver

class MotorStatus(Flag):
    '''A status flag for the Motor.

    Has information about things like if the motor is moving and
    the motor limits.
    '''
    # CW hardware limit switch (0 - no contact, 1 - contact)
    CW_HW_LIMIT_SWITCH = 0x00000001
    # CCW hardware limit switch (0 - no contact, 1 - contact)
    CCW_HW_LIMIT_SWITCH = 0x00000002
    # CW software limit switch (0 - no contact, 1 - contact)
    # Not applicable to Part Number ODC001 and TDC001 controllers
    CW_SW_LIMIT_SWITCH = 0x00000004
    # Not applicable to Part Number ODC001 and TDC001 controllers
    # CCW software limit switch (0 - no contact, 1 - contact)
    CCW_SW_LIMIT_SWITCH = 0x00000008
    # Motor shaft moving clockwise (1 - moving, 0 - stationary)
    SHAFT_MOVING_CW = 0x00000010
    # Motor shaft moving counterclockwise (1 - moving, 0 - stationary)
    SHAFT_MOVING_CCW = 0x00000020
    # Shaft jogging clockwise (1 - moving, 0 - stationary)
    SHAFT_JOGGING_CW = 0x00000040
    # Shaft jogging counterclockwise (1 - moving, 0 - stationary)
    SHAFT_JOGGING_CCW = 0x00000080
    # Motor connected (1 - connected, 0 - not connected)
    # Not applicable to Part Number BMS001 and BMS002 controllers
    # Not applicable to Part Number ODC001 and TDC001 controllers
    CONNECTED = 0x00000100
    # Motor homing (1 - homing, 0 - not homing)
    HOMING = 0x00000200
    # Motor homed (1 - homed, 0 - not homed)
    HOMED = 0x00000400
    # For Future Use
    FUTURE_USE_1 = 0x00000800

    # The following are applicable only to the BBD10x series brushless DC controllers

    # Trajectory within tracking window (1 – within window, 0 – not within window)
    TRAJECTORY_IN_TRACKING_WNDW = 0x00001000
    # Axis within settled window (1 – settled within window, 0 – not settled within window)
    AXIS_IN_SETTLED_WNDW = 0x00002000
    # Axis exceeds position error limit (1 – limit exceeded, 0 – within limit)
    AXIS_EXCEEDS_POS_ERROR_LIMIT = 0x00004000
    # Set when position module instruction error exists (1 – instruction error exists, 0 – no error)
    POS_MODULE_INSTRUCTION_ERROR_EXISTS = 0x00008000
    # Interlock link missing in motor connector (1 – missing, 0 – present)
    INTERLOCK_LINK_MISSING = 0x00010000
    # Position module over temperature warning (1 – over temp, 0 – temp OK)
    POS_MODULE_OVER_TEMP_WARNING = 0x00020000
    # Position module bus voltage fault (1 – fault exists, 0 – OK)
    POS_MODULE_BUS_VOLTAGE_FAULT = 0x00040000
    # Axis commutation error (1 – error, 0 – OK)
    AXIS_COMMUTATION_ERROR = 0x00080000

    # Digital Input States are only applicable if associated digital input is fitted to controller

    # Digital input 1 state (1 - logic high, 0 - logic low)
    DIGITAL_INPUT_1 = 0x00100000
    # Digital input 2 state (1 - logic high, 0 - logic low)
    DIGITAL_INPUT_2 = 0x00200000
    # Digital input 3 state (1 - logic high, 0 - logic low)
    DIGITAL_INPUT_3 = 0x00400000
    # Digital input 4 state (1 - logic high, 0 - logic low)
    DIGITAL_INPUT_4 = 0x00800000
    # BBD10x Controllers: Axis phase current limit (1 – current limit exceeded, 0 – below limit)
    # Other Controllers: Digital input 5 state (1 - logic high, 0 - logic low)
    DIGITAL_INPUT_5 = 0x01000000
    # Digital input 6 state (1 - logic high, 0 - logic low)
    DIGITAL_INPUT_6 = 0x02000000

    # For Future Use
    FUTURE_USE_2 = 0x04000000
    # For Future Use
    FUTURE_USE_3 = 0x08000000
    # For Future Use
    FUTURE_USE_4 = 0x10000000
    # Active (1 – indicates unit is active, 0 – not active)
    ACTIVE = 0x20000000
    # For Future Use
    FUTURE_USE_5 = 0x40000000
    # Channel enabled (1 – enabled, 0 – disabled)
    ENABLED = 0x80000000

class APTMotor():
    '''APT Motor controller'''
    # TODO: Improve reporting on the return codes
    def __init__(self, serial_num=None, hw_type=HWType.TDC001, verbose=False, dllpath=DLL_PATH):
        self.verbose = verbose
        self.connected = False
        if not os.path.exists(dllpath):
            raise DllNotFoundError(f'{os.path.abspath(dllpath)} not found')
        self.aptdll = windll.LoadLibrary(dllpath)
        self.aptdll.APTInit()
        self.event_dlg_enabled = True
        self.aptdll.EnableEventDlg(True)
        self.hw_type = c_long(hw_type.value)
        self.bl_corr = 0.10  # 100um backlash correction
        if serial_num is not None:
            if self.verbose:
                print('Serial is', serial_num)
            self.serial_num = c_long(serial_num)
            self.init_hw_device()
        elif self.verbose:
            print('No serial, please set serial_num')

    def init_hw_device(self):
        '''Initialises the motor.
        You can only get the position of the motor and move the motor after it has been initialised.
        Once initialised, it will not respond to other objects trying to control it, until released.
        '''
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Initializing motor')
        ret = self.aptdll.InitHWDevice(self.serial_num)
        if ret != 0:
            raise MotorConnectionError(f'Unable to connect to motor {self.serial_num.value}')
        self.connected = True
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Connection was successfull')

    def apt_clean_up(self):
        '''Releases the APT object.
        Use when exiting the program.
        '''
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Cleaning up APT')
        self.aptdll.APTCleanUp()
        self.connected = False

    def enable_event_dlg(self, enable: bool):
        '''Enables or disables the Event Dialog message box'''
        ret = self.aptdll.EnableEventDlg(enable)
        self._check_return(ret)
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Enable Event Dialog = {enable}')
        self.event_dlg_enabled = enable

    def get_num_hw_units(self):
        '''Returns the number of HW units connected that are available to be interfaced'''
        num_units = c_long()
        self.aptdll.GetNumHWUnitsEx(self.hw_type, pointer(num_units))
        return num_units.value

    def get_serial_num_by_index(self, index):
        '''Returns the Serial Number of the specified index'''
        hw_serial_num = c_long()
        hw_index = c_long(index)
        self.aptdll.GetHWSerialNumEx(self.hw_type, hw_index, pointer(hw_serial_num))
        return hw_serial_num

    def set_serial_number(self, serial_num):
        '''Sets the Serial Number of the specified index'''
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Changing serial number to {serial_num.value}')
        self.serial_num = c_long(serial_num)
        return self.serial_num.value

    # Interfacing with the motor settings
    def get_hw_info(self):
        '''Returns the motor's model, software version and information about the hardware'''
        self._check_connected()
        model = create_string_buffer(255)
        sw_version = create_string_buffer(255)
        hw_notes = create_string_buffer(255)
        ret = self.aptdll.get_hw_info(self.serial_num, model, 255, sw_version, 255, hw_notes, 255)
        self._check_return(ret)
        return [model.value, sw_version.value, hw_notes.value]

    def identify(self):
        '''Causes the motor to blink the Active LED'''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Identifying')
        ret = self.aptdll.MOT_Identify(self.serial_num)
        self._check_return(ret)

    def get_status_flag(self):
        '''Returns the current status flag for the motor'''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Retrieving status flag')
        status_bits = c_long()
        ret = self.aptdll.MOT_GetStatusBits(self.serial_num, pointer(status_bits))
        self._check_return(ret)
        status = MotorStatus(status_bits.value)
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Status={status}')
        return status

    def get_stage_axis_info(self):
        '''Returns some the stage's min and max position, the unit they're in and the pitch'''
        self._check_connected()
        min_pos = c_float()
        max_pos = c_float()
        units = c_long()
        pitch = c_float()
        ret = self.aptdll.MOT_GetStageAxisInfo(self.serial_num,
                                               pointer(min_pos), pointer(max_pos),
                                               pointer(units), pointer(pitch))
        self._check_return(ret)
        return [min_pos.value, max_pos.value, units.value, pitch.value]

    def set_stage_axis_info(self, stage_axis_info):
        '''Returns the min/max position, the unit in mm, and the pitch'''
        self._check_connected()
        min_pos = c_float(stage_axis_info[0])
        max_pos = c_float(stage_axis_info[1])
        units = c_long(stage_axis_info[2])  # units of mm
        # Get different pitches of lead screw for moving stages for different stages.
        pitch = c_float(stage_axis_info[3])
        ret = self.aptdll.MOT_SetStageAxisInfo(self.serial_num, min_pos, max_pos, units, pitch)
        self._check_return(ret)

    def get_hw_limit_switches(self):
        '''Returns the action that the forward and reverse hardware limit switches make on contact.
        The action can be either Make, Break or Ignore.
        '''
        self._check_connected()
        reverse_limit_switch = c_long()
        forward_limit_switch = c_long()
        ret = self.aptdll.MOT_GetHWLimSwitches(self.serial_num,
                                               pointer(reverse_limit_switch),
                                               pointer(forward_limit_switch))
        self._check_return(ret)
        return [reverse_limit_switch.value, forward_limit_switch.value]

    def get_vel_params(self):
        '''Returns min velocity, acceleration and max velocity'''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Retrieving velocity parameters')
        min_vel = c_float()
        accel = c_float()
        max_vel = c_float()
        ret = self.aptdll.MOT_GetVelParams(self.serial_num,
                                           pointer(min_vel), pointer(accel), pointer(max_vel))
        self._check_return(ret)
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Velocity parameters are: '
                  f'min_vel={min_vel}, accel={accel}, max_vel={max_vel}')
        return [min_vel.value, accel.value, max_vel.value]

    def get_vel(self):
        '''Returns the maximum velocity'''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Retrieving maximum velocity')
        min_vel = c_float()
        accel = c_float()
        max_vel = c_float()
        ret = self.aptdll.MOT_GetVelParams(self.serial_num,
                                           pointer(min_vel), pointer(accel), pointer(max_vel))
        self._check_return(ret)
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Max velocity is max_vel{max_vel}')
        return max_vel

    def set_vel_params(self, min_vel, accel, max_vel):
        '''Sets the min/max velocities and the acceleration'''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Setting velocity parameters: '
                  f'min_vel={min_vel} accel={accel}, max_vel={max_vel}')
        min_vel = c_float(min_vel)
        accel = c_float(accel)
        max_vel = c_float(max_vel)
        ret = self.aptdll.MOT_SetVelParams(self.serial_num, min_vel, accel, max_vel)
        self._check_return(ret)

    def set_vel(self, max_vel):
        '''Sets the maximum velocity'''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Setting max velocity to {max_vel}')
        min_vel, accel, _ = self.get_vel_params()
        self.set_vel_params(min_vel, accel, max_vel)

    def get_vel_param_limits(self):
        '''Returns the max acceleration and velocity'''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - '
                  'Retrieving max acceleration and max velocity')
        max_accel = c_float()
        max_vel = c_float()
        ret = self.aptdll.MOT_GetVelParamLimits(self.serial_num,
                                                pointer(max_accel), pointer(max_vel))
        self._check_return(ret)
        return [max_accel.value, max_vel.value]

    # Controlling the motors
    # move = normal movement
    # movec = move with controlled velocity
    # move_bl = move with backlash correction
    #
    # rel = relative distance from current position.
    # abs = absolute position

    def get_pos(self):
        '''Obtain the current absolute position of the stage, as perceived by the motor'''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Retrieving position')
        pos = c_float()
        ret = self.aptdll.MOT_GetPosition(self.serial_num, pointer(pos))
        self._check_return(ret)
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Current position = {pos.value}')
        return pos.value

    def move_rel(self, rel_dist: float, wait: bool = True):
        '''
        Moves the motor a relative distance specified
            :param rel_dist: Relative position desired
        '''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Move relative')
        rel_dist = c_float(rel_dist)
        ret = self.aptdll.MOT_MoveRelativeEx(self.serial_num, rel_dist, wait)
        self._check_return(ret)

    def move_abs(self, abs_pos: float, wait: bool = True):
        '''
        Moves the motor to the Absolute position specified
            :param abs_pos: Position desired
        '''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Move absolute {abs_pos}')
        abs_pos = c_float(abs_pos)
        ret = self.aptdll.MOT_MoveAbsoluteEx(self.serial_num, abs_pos, wait)
        self._check_return(ret)

    def movec_rel(self, rel_dist: float, move_vel: float = 0.5, wait: bool = True):
        '''
        Moves the motor a relative distance specified at a controlled velocity
            :param rel_dist: Relative position desired
            :param move_vel: Motor velocity, mm/sec
        '''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - '
                  f'Move relative with controlled velocity {rel_dist} {move_vel}')
        # Save velocities to reset after move
        prev_vel = self.get_vel_param_limits()[1]
        # Set new desired max velocity
        self.set_vel(move_vel)
        self.move_rel(rel_dist, wait)
        self.set_vel(prev_vel)

    def movec_abs(self, abs_pos: float, move_vel: float = 0.5, wait: bool = True):
        '''
        Moves the motor to the Absolute position specified at a controlled velocity
            :param abs_pos: Position desired
            :param move_vel: Motor velocity in mm/sec
        '''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - '
                  f'Move absolute with controlled velocity {abs_pos} {move_vel}')
        # Save velocities to reset after move
        prev_vel = self.get_vel_param_limits()[1]
        # Set new desired max velocity
        self.set_vel(move_vel)
        self.move_abs(abs_pos, wait)
        self.set_vel(prev_vel)

    def move_bl_rel(self, rel_dist: float, wait: bool = True):
        '''
        Moves the motor a relative distance specified
            :param rel_dist: Relative position desired
        '''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - '
                  f'Move relative with bl correction {rel_dist}')
        self.move_rel(rel_dist - self.bl_corr, wait)
        self.move_rel(self.bl_corr, wait)

    def move_bl_abs(self, abs_pos: float, wait: bool = True):
        '''
        Moves the motor to the Absolute position specified
            :param abs_pos: Position desired
        '''
        self._check_connected()
        if self.verbose:
            print(f'Motor {self.serial_num.value} - '
                  f'Move absolute with bl correction {abs_pos}')
        if abs_pos < self.get_pos():
            if self.verbose:
                print('backlash move_abs', (abs_pos - self.bl_corr))
            self.move_abs(abs_pos-self.bl_corr, wait)
        self.move_abs(abs_pos, wait)

    def go_home(self, wait: bool = True):
        '''Moves the stage to home position and resets the perceived position'''
        self._check_connected()
        ret = self.aptdll.MOT_MoveHome(self.serial_num, wait)
        if self.verbose:
            print(f'Motor {self.serial_num.value} - Going home')
        self._check_return(ret)

    def stop(self):
        '''Stops the movement of the motor'''
        if self.connected:
            if self.verbose:
                print(f'Motor {self.serial_num.value} - Stopping')
            self.aptdll.MOT_StopProfiled(self.serial_num)

    def _check_connected(self):
        if not self.connected:
            raise MotorConnectionError('Motor is not connected. '
                                       'Use init_hw_device or create a new object.')

    def _check_return(self, ret_code):
        if ret_code != 0:
            self.stop()
            self.connected = False
