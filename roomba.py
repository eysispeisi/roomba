'''
https://www.irobotweb.com/~/media/MainSite/PDFs/About/STEM/Create/iRobot_Roomba_600_Open_Interface_Spec.pdf
'''

import serial
import time
import RPi.GPIO as gpio
import ctypes
import sys
from collections import OrderedDict


baudrates = [300,600,1200,2400,4800,9600,14400,19200,28800,38400,57600,115200]


device = "/dev/serial0"
default_baudrate = 115200

timeout = 1.0
baud_pin = 18
pulse_wake_up = 0

STREAM_INTERVAL = 0.015

###############################################################################
## Operational codes
##


MC_START = chr(128)
MC_RESET = chr(7)
MC_STOP = chr(173)
MC_BAUD = chr(129)

CC_SAFE = chr(131)
CC_FULL = chr(132)
CC_CLEAN = chr(135)
CC_MAX = chr(136)
CC_SPOT = chr(134)
CC_SEEK_DOCK = chr(143)
CC_POWER = chr(133)
CC_SCHEDULE = chr(167)
CC_SET_TIME = chr(168)

AC_DRIVE = chr(137)
AC_DRIVE_DIRECT = chr(145)
AC_DRIVE_PWM = chr(146)
AC_MOTORS = chr(138)
AC_PWM_MOTORS = chr(144)
AC_LEDS = chr(139)
AC_SCHEDULING_LEDS = chr(162)
AC_DIGIT_LEDS_RAW = chr(163)
AC_BUTTONS = chr(165)
AC_DIGIT_LEDS_ASCII = chr(164)
AC_SONG = chr(140)
AC_PLAY = chr(141)

IC_SENSORS = chr(142)
IC_QUERY_LIST = chr(149)
IC_STREAM = chr(148)
IC_PAUSE_RESUME_STREAM = chr(150)


###############################################################################
## Sensor Wrapper
##

SM_BUMPS_AND_WHEELDROPS = chr(7)
SM_WALL = chr(8)
SM_CLIFF_LEFT = chr(9)
SM_CLIFF_FRONT_LEFT = chr(10)
SM_CLIFF_FRONT_RIGHT = chr(11)
SM_CLIFF_RIGHT = chr(12)
SM_VIRTUAL_WALL = chr(13)
SM_OVERCURRENTS = chr(14)
SM_DIRT_DETECT = chr(15)
SM_IR_OPCODE = chr(17)
SM_BUTTONS = chr(18)
SM_DISTANCE = chr(19)
SM_ANGLE = chr(20)
SM_CHARGING_STATE = chr(21)
SM_VOLTAGE = chr(22)
SM_CURRENT = chr(23)
SM_TEMPERATURE = chr(24)
SM_BATTERY_CHARGE = chr(25)
SM_BATTERY_CAPACITY = chr(26)
SM_WALL_SIGNAL = chr(27)
SM_CLIFF_LEFT_SIGNAL = chr(28)
SM_CLIFF_FRONT_LEFT_SIGNAL = chr(29)
SM_CLIFF_FRONT_RIGHT_SIGNAL = chr(30)
SM_CLIFF_RIGHT_SIGNAL = chr(31)
SM_CHARGER_AVAILABLE = chr(34)
SM_OPEN_INTERFACE_MODE = chr(35)
SM_SONG_NUMBER = chr(36)
SM_SONG_PLAYING = chr(37)
SM_OI_STREAM_NUM_PACKETS = chr(38)
SM_VELOCITY = chr(39)
SM_RADIUS = chr(40)
SM_VELOCITY_RIGHT = chr(41)
SM_VELOCITY_LEFT = chr(42)
SM_ENCODER_COUNTS_LEFT = chr(43)
SM_ENCODER_COUNTS_RIGHT = chr(44)
SM_LIGHT_BUMPER = chr(45)
SM_LIGHT_BUMPER_LEFT = chr(46)
SM_LIGHT_BUMPER_FRONT_LEFT = chr(47)
SM_LIGHT_BUMPER_CENTER_LEFT = chr(48)
SM_LIGHT_BUMPER_CENTER_RIGHT = chr(49)
SM_LIGHT_BUMPER_FRONT_RIGHT = chr(50)
SM_LIGHT_BUMPER_RIGHT = chr(51)
SM_IR_OPCODE_LEFT = chr(52)
SM_IR_OPCODE_RIGHT = chr(53)
SM_LEFT_MOTOR_CURRENT = chr(54)
SM_RIGHT_MOTOR_CURRENT = chr(55)
SM_MAIN_BRUSH_CURRENT = chr(56)
SM_SIDE_BRUSH_CURRENT = chr(57)
SM_STASIS = chr(58)

TBL_BUMPS_AND_WHEEL_DROPS = [
    "Bump right?",
    "Bump left?",
    "Wheel drop right?",
    "Wheel drop left?",
    ]

TBL_WHEEL_OVERCURRENTS = [
    "Side brush",
    "",
    "Main brush",
    "Right wheel",
    "Left wheel",
    ]

TBL_BUTTONS = [
    "Clean",
    "Spot",
    "Dock",
    "Minute",
    "Hour",
    "Day",
    "Schedule",
    "Clock",
    ]
    
    
TBL_CHARGE_STATE = [
    "Not charging",
    "Reconditioning charging",
    "Full charging",
    "Trickle charging",
    "Waiting",
    "Charging fault condition",
    ]

TBL_CHARGING_SOURCES_AVAILABLE = [
    "Internal charger",
    "Home base",
    ]

TBL_OI_MODE = [
    "Off",
    "Passive",
    "Safe",
    "Full",
    ]

TBL_LIGHT_BUMPER = [
    "Light bumper left?",
    "Light bumper front left?",
    "Light bumper center left?",
    "Light bumper center right?",
    "Light bumper front right?",
    "Light bumper right?",
    ]

TBL_STASIS = [
    "Stasis toggling?",
    "Stasis disabled?",
    ]


class SensorPacket(object):
    name = ""
    id = 0
    data_bytes = 0
    def __init__(self):
        self.value = 0
    def compile_data(self, bytes):
        return [ord(b) if type(b) is str else b for b in bytes]
    def set(self, bytes):
        self.value = self.compile_data(bytes)
    def get(self):
        return self.value

class SensorPacket_1byte_unsigned(SensorPacket):
    data_bytes = 1
    def compile_data(self, bytes): 
        bytes = super(SensorPacket_1byte_unsigned, self).compile_data(bytes)
        return bytes[0]

class SensorPacket_1byte_signed(SensorPacket):
    data_bytes = 1
    def compile_data(self, bytes): 
        bytes = super(SensorPacket_1byte_signed, self).compile_data(bytes)
        b = bytes[0]
        mask = 1 << 7
        if b & mask:
            return -(b^0xff)
        return b

class SensorPacket_2byte_unsigned(SensorPacket):
    data_bytes = 2
    def compile_data(self, bytes): 
        bytes = super(SensorPacket_2byte_unsigned, self).compile_data(bytes)
        return bytes[0] << 8 | bytes[1]

class SensorPacket_2byte_signed(SensorPacket):
    data_bytes = 2
    def compile_data(self, bytes):
        bytes = super(SensorPacket_2byte_signed, self).compile_data(bytes)
        b = bytes[0] << 8 | bytes[1]
        mask = 1 << 15
        if b & mask:
            return -(b^0xffff)
        return b


class SensorPacket_BumpsAndWheelDrops(SensorPacket_1byte_unsigned):
    id = SM_BUMPS_AND_WHEELDROPS 
    name = "Bumps and wheel drops"
    def get(self):
        value = super(SensorPacket_BumpsAndWheelDrops, self).get()
        ret = []
        if value & 1:
            ret.append(TBL_BUMPS_AND_WHEEL_DROPS[0])
        if value & 2:
            ret.append(TBL_BUMPS_AND_WHEEL_DROPS[1])
        if value & 4:
            ret.append(TBL_BUMPS_AND_WHEEL_DROPS[2])
        if value & 8:
            ret.append(TBL_BUMPS_AND_WHEEL_DROPS[3])
        return ", ".join(ret)

        
class SensorPacket_Wall(SensorPacket_1byte_unsigned):
    id = SM_WALL 
    name = "Wall"
    def get(self):
        value = super(SensorPacket_Wall, self).get()
        return bool(value)

class SensorPacket_CliffLeft(SensorPacket_Wall):
    id = SM_CLIFF_LEFT 
    name = "Cliff left"

class SensorPacket_CliffFrontLeft(SensorPacket_Wall):
    id = SM_CLIFF_FRONT_LEFT 
    name = "Cliff Front Left"

class SensorPacket_CliffFrontRight(SensorPacket_Wall):
    id = SM_CLIFF_FRONT_RIGHT 
    name = "Cliff front right"

class SensorPacket_CliffRight(SensorPacket_Wall):
    id = SM_CLIFF_RIGHT 
    name = "Cliff right"

class SensorPacket_VirtualWall(SensorPacket_Wall):
    id = SM_VIRTUAL_WALL 
    name = "Virtual wall"

class SensorPacket_WheelOvercurrents(SensorPacket_1byte_unsigned):
    id = SM_OVERCURRENTS 
    name = "Wheel overcurrents"
    def get(self):
        value = super(SensorPacket_WheelOvercurrents, self).get()
        ret = []
        if value & 1:
            ret.append(TBL_WHEEL_OVERCURRENTS[0])
        if value & 4:
            ret.append(TBL_WHEEL_OVERCURRENTS[2])
        if value & 8:
            ret.append(TBL_WHEEL_OVERCURRENTS[3])
        if value & 16:
            ret.append(TBL_WHEEL_OVERCURRENTS[4])
        return ", ".join(ret)
            
class SensorPacket_DirtDetect(SensorPacket_1byte_unsigned):
    id = SM_DIRT_DETECT 
    name = "Dirt detect"

class SensorPacket_InfraredCharacterOmni(SensorPacket_1byte_unsigned):
    id = SM_IR_OPCODE 
    name = "Infrared character omni"

class SensorPacket_InfraredCharacterLeft(SensorPacket_1byte_unsigned):
    id = SM_IR_OPCODE_LEFT 
    name = "Infrared character left"

class SensorPacket_InfraredCharacterRight(SensorPacket_1byte_unsigned):
    id = SM_IR_OPCODE_RIGHT 
    name = "Infrared character right"

class SensorPacket_Buttons(SensorPacket_1byte_unsigned):
    id = SM_BUTTONS 
    name = "Buttons"
    def get(self):
        v = super(SensorPacket_Buttons, self).get()
        ret = []
        if v & 0b00000001:
            ret.append(TBL_BUTTONS[0])
        if v & 0b00000010:
            ret.append(TBL_BUTTONS[1])
        if v & 0b00000100:
            ret.append(TBL_BUTTONS[2])
        if v & 0b00001000:
            ret.append(TBL_BUTTONS[3])
        if v & 0b00010000:
            ret.append(TBL_BUTTONS[4])
        if v & 0b00100000:
            ret.append(TBL_BUTTONS[5])
        if v & 0b01000000:
            ret.append(TBL_BUTTONS[6])
        if v & 0b10000000:
            ret.append(TBL_BUTTONS[7])
        if ret:
            return ", ".join(ret)
        else:
            return "None"

class SensorPacket_Distance(SensorPacket_2byte_signed):
    id = SM_DISTANCE 
    name = "Distance"

class SensorPacket_Angle(SensorPacket_2byte_signed):
    id = SM_ANGLE 
    name = "Angle"

class SensorPacket_ChargingState(SensorPacket_1byte_unsigned):
    id = SM_CHARGING_STATE 
    name = "Charging state"
    def get(self):
        value = super(SensorPacket_ChargingState, self).get()
        return TBL_CHARGE_STATE[value]
        
class SensorPacket_Voltage(SensorPacket_2byte_unsigned):
    id = SM_VOLTAGE 
    name = "Voltage"
    def get(self): return "%.3fV" % (float(self.value)/1000,)

class SensorPacket_Current(SensorPacket_2byte_signed):
    id = SM_CURRENT 
    name = "Current"
    def get(self):
        return "%d mAh" % (self.value,)

class SensorPacket_Temperature(SensorPacket_1byte_signed):
    id = SM_TEMPERATURE 
    name = "Temperature"
    def get(self): return "%dC" % (self.value,)
    
class SensorPacket_BatteryCharge(SensorPacket_2byte_unsigned):
    id = SM_BATTERY_CHARGE 
    name = "Battery charge"
    def get(self): return "%d mAh" % (self.value,)

class SensorPacket_BatteryCapacity(SensorPacket_2byte_unsigned):
    id = SM_BATTERY_CAPACITY 
    name = "Battery capacity"
    def get(self): return "%d mAh" % (self.value,)

class SensorPacket_WallSignal(SensorPacket_2byte_unsigned):
    id = SM_WALL_SIGNAL 
    name = "Wall signal"

class SensorPacket_CliffLeftSignal(SensorPacket_2byte_unsigned):
    id = SM_CLIFF_LEFT_SIGNAL 
    name = "Cliff left signal"

class SensorPacket_CliffFrontLeftSignal(SensorPacket_2byte_unsigned):
    id = SM_CLIFF_FRONT_LEFT_SIGNAL 
    name = "Cliff front left signal"

class SensorPacket_CliffFrontRightSignal(SensorPacket_2byte_unsigned):
    id = SM_CLIFF_FRONT_RIGHT_SIGNAL 
    name = "Cliff front right signal"

class SensorPacket_CliffRightSignal(SensorPacket_2byte_unsigned):
    id = SM_CLIFF_RIGHT_SIGNAL 
    name = "Cliff right signal"

class SensorPacket_ChargingSourcesAvailable(SensorPacket_1byte_unsigned):
    id = SM_CHARGER_AVAILABLE 
    name = "Charging sources available"
    def get(self):
        value = super(SensorPacket_ChargingSourcesAvailable, self).get()
        return value
        ret = []
        if value & 1:
            ret.append(TBL_CHARGING_SOURCES_AVAILABLE[0])
        if value & 2:
            ret.append(TBL_CHARGING_SOURCES_AVAILABLE[1])
        if not ret:
            return "None"
        else:
            return ", ".join(ret)
        

class SensorPacket_OIMode(SensorPacket_1byte_unsigned):
    id = SM_OPEN_INTERFACE_MODE 
    name = "OI mode"
    def get(self):
        value = super(SensorPacket_OIMode, self).get()
        return TBL_OI_MODE[value]
    
class SensorPacket_SongNumber(SensorPacket_1byte_unsigned):
    id = SM_SONG_NUMBER 
    name = "Song number"

class SensorPacket_SongPlaying(SensorPacket_1byte_unsigned):
    id =  SM_SONG_PLAYING 
    name = "Song playing"

class SensorPacket_NumberOfStreamPackets(SensorPacket_1byte_unsigned):
    id =  SM_OI_STREAM_NUM_PACKETS 
    name = "Number of stream packets"

class SensorPacket_RequestedVelocity(SensorPacket_2byte_signed):
    id =  SM_VELOCITY 
    name = "Requested velocity"

class SensorPacket_RequestedRadius(SensorPacket_2byte_signed):
    id =  SM_RADIUS 
    name = "Requested radius"

class SensorPacket_RequestedRightVelocity(SensorPacket_2byte_signed):
    id =  SM_VELOCITY_RIGHT 
    name = "Requested right velocity"

class SensorPacket_RequestedLeftVelocity(SensorPacket_2byte_signed):
    id =  SM_VELOCITY_LEFT 
    name = "Requested left velocity"

class SensorPacket_LeftEncoderCounts(SensorPacket_2byte_signed):
    id =  SM_ENCODER_COUNTS_LEFT 
    name = "Left encoder counts"

class SensorPacket_RightEncoderCounts(SensorPacket_2byte_signed):
    id = SM_ENCODER_COUNTS_RIGHT 
    name = "Right encoder counts"

class SensorPacket_LightBumpSignal(SensorPacket_1byte_unsigned):
    id =  SM_LIGHT_BUMPER 
    name = "Light bump signal"
    def get(self):
        value = super(SensorPacket_LightBumpSignal, self).get()
        ret = []
        if value & 0b00000001:
            ret.append(TBL_LIGHT_BUMPER[0])
        if value & 0b00000010:
            ret.append(TBL_LIGHT_BUMPER[1])
        if value & 0b00000100:
            ret.append(TBL_LIGHT_BUMPER[2])
        if value & 0b00001000:
            ret.append(TBL_LIGHT_BUMPER[3])
        if value & 0b00010000:
            ret.append(TBL_LIGHT_BUMPER[4])
        if value & 0b00100000:
            ret.append(TBL_LIGHT_BUMPER[5])
        if not ret:
            return "None"
        else:
            return ", ".join(ret)

class SensorPacket_LightBumpLeftSignal(SensorPacket_2byte_unsigned):
    id = SM_LIGHT_BUMPER_LEFT 
    name = "Light bump left signal"

class SensorPacket_LightBumpFrontLeftSignal(SensorPacket_2byte_unsigned):
    id = SM_LIGHT_BUMPER_FRONT_LEFT 
    name = "Light bump front left signal"

class SensorPacket_LightBumpCenterLeftSignal(SensorPacket_2byte_unsigned):
    id = SM_LIGHT_BUMPER_CENTER_LEFT 
    name = "Light bump center left signal"

class SensorPacket_LightBumpCenterRightSignal(SensorPacket_2byte_unsigned):
    id = SM_LIGHT_BUMPER_CENTER_RIGHT 
    name = "Light bump center right signal"

class SensorPacket_LightBumpFrontRightSignal(SensorPacket_2byte_unsigned):
    id = SM_LIGHT_BUMPER_FRONT_RIGHT 
    name = "Light bump front right signal"

class SensorPacket_LightBumpRightSignal(SensorPacket_2byte_unsigned):
    id = SM_LIGHT_BUMPER_RIGHT 
    name = "Light bump right signal"

class SensorPacket_LeftMotorCurrent(SensorPacket_2byte_signed):
    id = SM_LEFT_MOTOR_CURRENT 
    name = "Left motor current"

class SensorPacket_RightMotorCurrent(SensorPacket_2byte_signed):
    id = SM_RIGHT_MOTOR_CURRENT 
    name = "Right motor current"

class SensorPacket_MainBrushMotorCurrent(SensorPacket_2byte_signed):
    id = SM_MAIN_BRUSH_CURRENT 
    name = "Main brush motor current"

class SensorPacket_SideBrushMotorCurrent(SensorPacket_2byte_signed):
    id = SM_SIDE_BRUSH_CURRENT 
    name = "Side brush motor current"

class SensorPacket_Stasis(SensorPacket_1byte_unsigned):
    id = SM_STASIS 
    name = "Stasis"
    def get(self):
        v = super(SensorPacket_Stasis, self).get()
        ret = []
        if v & 0b01:
            ret.append(TBL_STASIS[0])
        if v & 0b10:
            ret.append(TBL_STASIS[1])
        if ret:
            return ", ".join(ret)
        else:
            return "None"


SENSORS = [
    SensorPacket_BumpsAndWheelDrops,SensorPacket_Wall,SensorPacket_CliffLeft,SensorPacket_CliffFrontLeft,
    SensorPacket_CliffFrontRight,SensorPacket_CliffRight,SensorPacket_VirtualWall,SensorPacket_WheelOvercurrents,
    SensorPacket_DirtDetect,SensorPacket_InfraredCharacterOmni,SensorPacket_InfraredCharacterLeft,
    SensorPacket_InfraredCharacterRight,SensorPacket_Buttons,SensorPacket_Distance,SensorPacket_Angle,
    SensorPacket_ChargingState,SensorPacket_Voltage,SensorPacket_Current,SensorPacket_Temperature,
    SensorPacket_BatteryCharge,SensorPacket_BatteryCapacity,SensorPacket_WallSignal,SensorPacket_CliffLeftSignal,
    SensorPacket_CliffFrontLeftSignal,SensorPacket_CliffFrontRightSignal,SensorPacket_CliffRightSignal,
    SensorPacket_ChargingSourcesAvailable,SensorPacket_OIMode,SensorPacket_SongNumber,SensorPacket_SongPlaying,
    SensorPacket_NumberOfStreamPackets,SensorPacket_RequestedVelocity,SensorPacket_RequestedRadius,
    SensorPacket_RequestedRightVelocity,SensorPacket_RequestedLeftVelocity,SensorPacket_LeftEncoderCounts,
    SensorPacket_RightEncoderCounts,SensorPacket_LightBumpSignal,SensorPacket_LightBumpLeftSignal,
    SensorPacket_LightBumpFrontLeftSignal,SensorPacket_LightBumpCenterLeftSignal,SensorPacket_LightBumpCenterRightSignal,
    SensorPacket_LightBumpFrontRightSignal,SensorPacket_LightBumpRightSignal,SensorPacket_LeftMotorCurrent,
    SensorPacket_RightMotorCurrent,SensorPacket_MainBrushMotorCurrent,SensorPacket_SideBrushMotorCurrent,
    SensorPacket_Stasis,
]

SENSOR_MAP = {}
for sensor in SENSORS:
    SENSOR_MAP[sensor.id] = sensor

   
class SensorQuery:

    def __init__(self, query_type=IC_STREAM):
        self.packets = []
        self.query_type = query_type

        
    def add(self, *ids):
        for id in ids:
            self.packets.append( id )

    def get_request(self):
        msg = self.query_type + chr(len(self.packets))
        for id in self.packets:
            msg += id
        return msg

    def get_response_size(self):
        return 1+1+sum(SENSOR_MAP[id].data_bytes for id in self.packets)+1
        

    def parse_from_stream(self, stream):
        while 1:
            c = stream.read(1)
            if not c:
                return False
            if c == chr(19):
                break
        next_index = stream.pos

        np = stream.read(1)
        if not np:
            stream.seek(next_index)
            return False

        data = stream.read(ord(np))
        if len(data) != ord(np):
            stream.seek(next_index)
            return False
            
        crc = stream.read(1)
        if not crc or checksum(c+np+data+crc) != 0x00:
            stream.seek(next_index)
            return False

        packets = {}
        data = list(data)
        while len(data) > 1:
            try:
                if len(data) < 1:
                    break
                id = data.pop(0)
                sensor_type = SENSOR_MAP[id]
                n_bytes = sensor_type.data_bytes
               
                bytes = []
                for b in xrange(n_bytes):
                    bytes.append(data.pop(0))
                    
                p = sensor_type()
                p.set(bytes)
                packets[id] = p 
            except IndexError, e:
                sys.exc_clear()
                stream.seek(next_index)
                return False
        return packets

    def get_packets(self):
        ret = []
        for packet in self.packets.itervalues():
            ret.append( packet )
        return ret

    def is_data_too_large(self, packets):
        size = 3 # bits: header, count, crc
        for packet in packets:
            size += packet.data_bytes
        max = STREAM_INTERVAL / 10 * default_baudrate
        if size > max:
            return True
        return False



###############################################################################
## Utilities
##


def checksum(data):
    return (sum([ord(x) for x in data]) & 0xff)# ^ 0xff


###############################################################################
## Operetional Commands
##


def set_schedule():
    data = [
        0b00111110, # days [ reserved, sat, fri, thu, wed, tue, mon, sun ]
        0, 0,  # sunday, hour, minutes
        10, 0, # mon
        10, 0, # tue
        10, 0, # wed
        10, 0, # thu
        10, 0, # fri
        0, 0,  # sat
        ]
    return CC_SCHEDULE+"".join([chr(x) for x in data])


def update_clock():
    # day hour minute
    gmt = time.gmtime()
    day = gmt.tm_wday + 1 if gmt.tm_wday != 6 else 0
    hour = gmt.tm_hour
    minute = gmt.tm_min
    return "".join([CC_SET_TIME, chr(day), chr(hour), chr(minute)])


def drive(velocity, angle):
    '''
    velocity = -500 to +500
    angle = -2000 to +2000
    '''
    if not (-500 <= velocity <= 500):
        raise RuntimeError("velocity out of range")
    if not (-2000 <= angle <= 2000):
        raise RuntimeError("angle out of range")
    if velocity < 0:
        velocity += (2**16)&0xffff
    s_velocity = chr(velocity>>8&0xff) + chr(velocity&0xff)
    if angle < 0:
        angle += (2**16)&0xffff
    s_angle = (chr(angle>>8&0xff) + chr(angle&0xff))
    return AC_DRIVE + s_velocity + s_angle


def motors(*motors):
    '''
    args:
        0/1, # side brush
        0/1, # vaccum
        0/1, # main brush
        0/1, # Side brush clockwise
        0/1, # main brush direction
    '''
    b = 0
    for i, m in enumerate(motors):
        b += m<<i
    return AC_MOTORS + chr(b)

