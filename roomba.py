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


def format_cmd_rfnano(cmd, wake_up=0):
    global pulse_wake_up
    MASK_SIZE        = 0b00011111
    MASK_WAKE_UP     = 0b00100000
    RF_NANO_PAYLOAD = 32
    if type(cmd) is list:
        tmp = ""
        for c in cmd:
            if type(c) is int:
                tmp+= chr(c)
            elif type(c) is str:
                tmp += c
            else:
                raise RuntimeError("invalid type")
        cmd = tmp
    ibit = len(cmd)
    if wake_up:
        ibit |= MASK_WAKE_UP
    ret = chr(ibit)+cmd+chr(0)*(RF_NANO_PAYLOAD-len(cmd)-2)
    ret += chr(checksum(ret))
    return ret

    
def simple_command(cmd):
    return format_cmd_rfnano(MC_START+cmd+MC_STOP)


def checksum(data):
    return (sum([ord(x) for x in data]) & 0xff)# ^ 0xff


###############################################################################
## Operetional Commands
##

def test():
    print "test"
    messages = [SensorPacket_BumpsAndWheelDrops(),SensorPacket_Wall(),SensorPacket_CliffLeft(),SensorPacket_CliffFrontLeft(),
        SensorPacket_CliffFrontRight(),SensorPacket_CliffRight(),SensorPacket_VirtualWall(),SensorPacket_WheelOvercurrents(),
        SensorPacket_DirtDetect(),SensorPacket_InfraredCharacterOmni(),SensorPacket_InfraredCharacterLeft(),
        SensorPacket_InfraredCharacterRight(),SensorPacket_Buttons(),SensorPacket_Distance(),SensorPacket_Angle(),
        SensorPacket_ChargingState(),SensorPacket_Voltage(),SensorPacket_Current(),SensorPacket_Temperature(),
        SensorPacket_BatteryCharge(),SensorPacket_BatteryCapacity(),SensorPacket_WallSignal(),SensorPacket_CliffLeftSignal(),
        SensorPacket_CliffFrontLeftSignal(),SensorPacket_CliffFrontRightSignal(),SensorPacket_CliffRightSignal(),
        SensorPacket_ChargingSourcesAvailable(),SensorPacket_OIMode(),SensorPacket_SongNumber(),SensorPacket_SongPlaying(),
        SensorPacket_NumberOfStreamPackets(),SensorPacket_RequestedVelocity(),SensorPacket_RequestedRadius(),
        SensorPacket_RequestedRightVelocity(),SensorPacket_RequestedLeftVelocity(),SensorPacket_LeftEncoderCounts(),
        SensorPacket_RightEncoderCounts(),SensorPacket_LightBumpSignal(),SensorPacket_LightBumpLeftSignal(),
        SensorPacket_LightBumpFrontLeftSignal(),SensorPacket_LightBumpCenterLeftSignal(),SensorPacket_LightBumpCenterRightSignal(),
        SensorPacket_LightBumpFrontRightSignal(),SensorPacket_LightBumpRightSignal(),SensorPacket_LeftMotorCurrent(),
        SensorPacket_RightMotorCurrent(),SensorPacket_MainBrushMotorCurrent(),SensorPacket_SideBrushMotorCurrent(),
        SensorPacket_Stasis(),
        ]
    sm = SensorStream()
    for m in messages:
        sm.add(m)
    with get_socket() as socket:
        socket.write(MC_START)
        sm.set_stream(socket)
        sm.send_request()
        sm.read_response()
        for packet in sm.get_packets():
            print "%s: %s (%r)" % (packet.name, packet.get(), packet.value)
        sm.toggle_stream()

def check_battery():
    print "check battery"
    messages = [
        SensorPacket_ChargingState(),
        SensorPacket_Voltage(),
        SensorPacket_Current(),
        SensorPacket_Temperature(),
        SensorPacket_BatteryCharge(),
        SensorPacket_BatteryCapacity(),
        ]
    sm = SensorStream()
    for m in messages:
        sm.add(m)

    with get_socket() as socket:
        socket.write(MC_START)
        sm.set_stream(socket)
        sm.send_request()
        for n in xrange(3):
            sm.read_response()
            print "-"*20
            for packet in sm.get_packets():
                print "%s: %s (%r)" % (packet.name, packet.get(), packet.value)
            print "Battery state: %.1f%%" % (messages[4].value/float(messages[5].value)*100,)
            print
            # a clunky sleep can cause stream to go out of sync, todo calculate time elapsed to get correct sleep duration.
            time.sleep(STREAM_INTERVAL) 
        sm.toggle_stream()



def change_baudrate(baudrate):
    print "changing baudrate to ", baudrate
    '''
    time.sleep(2)
    gpio.setmode(gpio.BCM)
    gpio.setup(baud_pin, gpio.OUT)
    for x in xrange(3):
        gpio.output(baud_pin, gpio.HIGH)
        time.sleep(0.05)
        gpio.output(baud_pin, gpio.LOW)
    gpio.cleanup()
    '''
    with serial.Serial(device, baudrate=115200, timeout=timeout) as port:
        port.write(MC_START)
        time.sleep(0.015)
        port.write(MC_BAUD + chr(baudrates.index(baudrate)))
        port.flush()
    time.sleep(0.1)


def wake_up():
    '''
    uses gpio, physical connection to baude pin required
    Warning: calling this funtion repeatedly might cause a change to baude rate.
    '''
    print "wake from sleep"
    gpio.setmode(gpio.BCM)
    gpio.setup(baud_pin, gpio.OUT)
    gpio.output(baud_pin, gpio.LOW)
    gpio.cleanup()
    time.sleep(1)

def start():
    simple_command(MC_START)

def reset():
    print "resetting roomba"
    simple_command(MC_RESET)

def stop():
    print "stop"
    with get_socket() as port:
        port.write(MC_STOP)

def safe():
    with get_socket() as port:
        port.write(CC_SAFE)

def full():
    with get_socket() as port:
        port.write(CC_FULL)

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
    data = "".join([chr(x) for x in data])
    print("setting schedule to:", data)
    simple_command(CC_SCHEDULE+data)


def update_clock():
    # day hour minute
    gmt = time.gmtime()
    day = gmt.tm_wday + 1 if gmt.tm_wday != 6 else 0
    hour = gmt.tm_hour
    minute = gmt.tm_min
    cmd = "".join([CC_SET_TIME, chr(day), chr(hour), chr(minute)])
    print("set time to: %s day, %s hour, %s minute" % (day, hour, minute))
    simple_command(cmd)


def clean():
    simple_command(CC_CLEAN)


def dock():
    simple_command(CC_SEEK_DOCK)
        

def stop_stream():
    print "stop stream"
    simple_command(IC_PAUSE_RESUME_STREAM + chr(0))

def power_down():
    print "power down"
    return CC_POWER
    

def drive(velocity, angle):
    '''
    velocity = -500 to +500
    angle = -2000 to +2000
    '''
    if not (-500 <= velocity <= 500):
        raise RuntimeError("velocity out of range")
    if not (-2000 <= angle <= 2000):
        raise RuntimeError("angle out of range")
    cmd = MC_START
    #cmd += CC_FULL
    cmd += CC_SAFE
    if velocity < 0:
        velocity += (2**16)&0xffff
    s_velocity = chr(velocity>>8&0xff) + chr(velocity&0xff)
    if angle < 0:
        angle += (2**16)&0xffff
    s_angle = (chr(angle>>8&0xff) + chr(angle&0xff))
    return AC_DRIVE + s_velocity + s_angle
    return cmd


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

    
import select
import tty
import termios
def joy_ride():
    print "going for a joy ride"
    def isData():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
        
    import nrf24
    
    radio = nrf24.NRF24()
    radio.begin(0, 0, 25, None) #Set CE and IRQ pins
    w_pipe = [0x46, 0x47, 0x48, 0x49, 0x4a]
    radio.setPayloadSize(32)
    radio.setRetries(10,50)
    radio.setChannel(108)
    radio.openWritingPipe(w_pipe)
    radio.stopListening()
    radio.printDetails()
    wake_up = False
    if pulse_wake_up:
        wake_up = True
    radio.write(format_cmd_rfnano(MC_START, wake_up))
        
    v = 0
    a = 0
    do_clean = 1

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while 1:
            if isData():
                cmd = ""
                c = sys.stdin.read(1)
                if c == '\x1b':         # x1b is ESC
                    break
                elif c == 'w':
                    print 'forward'
                    v += 100
                    cmd = drive(v, a)
                elif c == 's':
                    print 'backward'
                    v -= 100
                    cmd = drive(v, a)
                elif c == 'a':
                    print 'left'
                    a += 100
                    cmd = drive(v, a)
                elif c == 'd':
                    print 'right'
                    a -= 100
                    cmd = drive(v, a)
                elif c == ' ':
                    if do_clean:
                        print 'clean'
                        cmd = motors(1, 1, 1, 1, 1)
                        do_clean = 0
                    else:
                        print 'no clean'
                        cmd = motors(0, 0, 0, 0, 0)
                        do_clean = 1
                if cmd:
                    cmd = format_cmd_rfnano(cmd)
                    print "cmd ", ["%x"%ord(c) for c in cmd]
                    t = time.time()
                    print radio.write(cmd)
                    print "%dms" % ((time.time()-t)*1000, )
        radio.write(format_cmd_rfnano(power_down()))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def sound():
    import nrf24
    
    t = 1
    
    radio = nrf24.NRF24()
    radio.begin(0, 0, 25, None) #Set CE and IRQ pins
    w_pipe = [0x46, 0x47, 0x48, 0x49, 0x4a]
    radio.setPayloadSize(32)
    radio.setRetries(10,50)
    radio.setChannel(108)
    radio.openWritingPipe(w_pipe)
    radio.stopListening()
    radio.printDetails()
    wake_up = False
    if pulse_wake_up:
        wake_up = True
        
    cmd = MC_START + CC_SAFE
    print radio.write(format_cmd_rfnano(cmd, wake_up))
    time.sleep(0.1)

    b = 40
    t = 8
    cmd = [ 
        AC_SONG, 0, None,
        b, t,
        b+12*2, t,
        b, t,
        b+12, t,
        b, t,
        b+12, t,
        b+12*2, t,
        ]
    cmd[2] = (len(cmd)-3)/2
    print "{",
    print ','.join(["0x%x" % ord(c) for c in format_cmd_rfnano(cmd)]),
    print "}"
    print radio.write(format_cmd_rfnano(cmd))
    
    cmd = [AC_PLAY, 0]
    print radio.write(format_cmd_rfnano(cmd))
    time.sleep(3)
    
    print radio.write(format_cmd_rfnano(CC_POWER))
    radio.end()

        
def main():
    #test_joyride()
    #reset()
    #check_battery()
    if len(sys.argv) > 1:
        print sys.argv
        if 'wakeup' in sys.argv:
            print "wake up"
            global pulse_wake_up
            pulse_wake_up = 1

        c = sys.argv[1]
        if c == '0':
            power_down()
        elif c == 'joyride':
            joy_ride()
        elif c == 'clean':
            clean()
        elif c == 'wakeup':
            start()
        elif c == 'sound':
            sound()
        
    
    #update_clock()
    #set_schedule()
    #test()
    #stop_stream()
    #dock()
    


if __name__ == '__main__':
    main()
