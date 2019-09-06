import RPi.GPIO as GPIO
import nrf24
import sys
import time
import StringIO

from roomba import *

MASK_WAKE_UP     = 0b00100000

w_pipe = [0x46, 0x47, 0x48, 0x49, 0x4a]
r_pipe = [0x41, 0x42, 0x43, 0x44, 0x45]
ce_pin = 25
irq_pin = None
retries = 10

radio = None

def SetupRadio():
    radio = nrf24.NRF24()
    radio.begin(0, 0, ce_pin, irq_pin) #Set CE and IRQ pins
    radio.setRetries(15,15)
    radio.setPayloadSize(32)
    radio.setChannel(108)
    radio.setDataRate(radio.BR_2MBPS)
    radio.enableDynamicPayloads()
    #radio.setAutoAck(True)
    #radio.setAutoAck(False)
    #radio.setCRCLength(radio.CRC_16)
    radio.setPALevel(nrf24.NRF24.PA_MAX)

    radio.openWritingPipe(r_pipe)
    radio.openReadingPipe(1, w_pipe)
    radio.startListening()
    radio.printDetails()
    return radio

def SensorMessages():
    sm = SensorQuery()
    sm.add( 
            #SM_CHARGING_STATE,
            SM_VOLTAGE,
            SM_CURRENT,
            SM_TEMPERATURE,
            SM_BATTERY_CHARGE,
            SM_BATTERY_CAPACITY,
            SM_OPEN_INTERFACE_MODE,
            #SM_VELOCITY,
            #SM_ANGLE,
            #SM_BUMPS_AND_WHEELDROPS,
            )
    radio_out( sm.get_request() )

    t = time.time()
    old_pos = 0
    msg_buff = StringIO.StringIO()
    lines = len(sm.packets)+3
    sys.stdout.write('\n' + '='*20 + '\n')
    sys.stdout.write('\n'*lines)
    def loop():
        try:
            if radio.available():
                ret = []
                radio.read(ret, radio.getDynamicPayloadSize()),
                old_pos = msg_buff.pos
                msg_buff.write(''.join(chr(c) for c in ret))
                msg_buff.seek(old_pos)
                available = msg_buff.len-msg_buff.pos
                if available >= sm.get_response_size():
                    packets = sm.parse_from_stream(msg_buff)
                    if packets:
                        sys.stdout.write( '\033[%dA' % (lines,) )
                        print time.time()-t
                        for id, packet in packets.iteritems():
                            print "%s: %s (%r)" % (packet.name, packet.get(), packet.value)
                        print "Battery state: %.1f%%" % (packets[SM_BATTERY_CHARGE].value/float(packets[SM_BATTERY_CAPACITY].value)*100,)
                        print
                sys.stdout.flush()
        except KeyboardInterrupt:
            pass
    return loop

def listen():
    radio.openWritingPipe(r_pipe)
    radio.openReadingPipe(1, w_pipe)
    radio.startListening()

def talk():
    radio.openWritingPipe(w_pipe)
    radio.openReadingPipe(1, r_pipe)
    radio.stopListening()

def checksum(data):
    return (sum([ord(x) for x in data]) & 0xff) ^ 0xff

def format_cmd_rfnano(cmd, wake_up=0):
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

def radio_out(cmd, wakeup=0, retries=1):
    out_cmd = format_cmd_rfnano(cmd, wakeup)
    #print "cmd:", [ord(c) for c in out_cmd], 
    talk()
    for n in xrange(retries):
        r = radio.write(out_cmd)
        if r:
            break
    listen()
    if not r:
        print "roomba not responding"
        exit()
    time.sleep(0.5)
    return r


def wakeup():
    radio_out(MC_START, 1)

def kill():
    radio_out(MC_START + CC_POWER)

def reset():
    radio_out(MC_START + MC_RESET)

def stop_stream():
    radio_out(MC_START + IC_PAUSE_RESUME_STREAM+chr(0))

def play_song():
    radio_out([MC_START, CC_SAFE])
    song = [140, 0, 7, 40, 8, 64, 8, 40, 8, 52, 8, 40, 8, 52, 8, 64, 8, 141, 0]
    radio_out(song)
    time.sleep(2)
    radio_out(MC_START)

def stop():
    radio_out([MC_START, MC_STOP])

def dock():
    radio_out([MC_START, CC_SEEK_DOCK])

import select
import tty
import termios
def joy_ride():
    print "going for a joy ride"
    def isData():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    radio_out([MC_START, CC_FULL], 1)
    loop = SensorMessages()
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
                    if v <= 400:
                        v += 100
                        cmd = drive(v, a)
                elif c == 's':
                    if -400 <= v:
                        v -= 100
                        cmd = drive(v, a)
                elif c == 'a':
                    a += 100
                    cmd = drive(v, a)
                elif c == 'd':
                    a -= 100
                    cmd = drive(v, a)
                elif c == ' ':
                    if do_clean:
                        cmd = motors(1, 1, 1, 1, 1)
                        do_clean = 0
                    else:
                        cmd = motors(0, 0, 0, 0, 0)
                        do_clean = 1
                if cmd:
                    radio_out(cmd)
                loop()
        radio_out([MC_START,])
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def test():
    radio_out([MC_START, ])
    radio_out([CC_SAFE,])

    velocity = 20
    angle = 1
    radio_out( drive(velocity, angle) )

    SensorMessages()
    radio_out(IC_PAUSE_RESUME_STREAM+chr(0))
    radio_out(MC_START)
    radio.end()


def update_clock():
    # day hour minute
    gmt = time.gmtime()
    day = gmt.tm_wday + 1 if gmt.tm_wday != 6 else 0
    hour = gmt.tm_hour
    minute = gmt.tm_min
    cmd = "".join([CC_SET_TIME, chr(day), chr(hour), chr(minute)])
    print("set time to: %s day, %s hour, %s minute" % (day, hour, minute))
    radio_out(cmd)


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
    radio_out(CC_SCHEDULE+data)    


def parse_args():
    if len(sys.argv) < 2:
        return
    c = sys.argv[1]
    if c == 'dock':
        dock()
    elif c == 'sync':
        update_clock()
        set_schedule()
        
        
    exit()


def main():
    global radio
    radio = SetupRadio()
    
    parse_args()
    #test()
    joy_ride()

if __name__ == '__main__':
    main()

