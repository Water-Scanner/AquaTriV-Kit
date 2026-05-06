# encoding=utf-8
"""
Water Linked DVL protocol parser
"""
from __future__ import print_function, division
import logging
import re
import time
import sys
import serial
import crcmod
import struct
from time import sleep

# Logger
log = logging.getLogger(__file__)

# Python2 detection
IS_PY2 = False
if sys.version_info < (3, 0):
    IS_PY2 = True

# Protocol definitions
SOP = ord('w')
EOP = ord('\n')
DIR_CMD = ord('c')
DIR_RESP = ord('r')
CHECKSUM = ord('*')

CMD_GET_VERSION = ord('v')
CMD_GET_PAYLOAD_SIZE = ord('n')
CMD_GET_BUFFER_LENGTH = ord('l')
CMD_GET_DIAGNOSTIC = ord('d')
CMD_GET_SETTINGS = ord('c')
CMD_SET_SETTINGS = ord('s')
CMD_QUEUE_PACKET = ord('q')
CMD_FLUSH = ord('f')
RESP_GOT_PACKET = ord('p')
VELOCITY_REPORT = ord('x')
ALL_VALID = [
    CMD_GET_VERSION,
    CMD_GET_PAYLOAD_SIZE,
    CMD_GET_BUFFER_LENGTH,
    CMD_GET_DIAGNOSTIC,
    CMD_GET_SETTINGS,
    CMD_SET_SETTINGS,
    CMD_QUEUE_PACKET,
    CMD_FLUSH,
    RESP_GOT_PACKET,
    VELOCITY_REPORT,
]




def is_checksum(ch):
    """ Is the given byte an checksum char """
    #print(type(ch), ch, chr(ch))
    #print("Checksum ", type(CHECKSUM), ord(CHECKSUM), CHECKSUM)
    if isinstance(ch, bytes):
        return ord(ch) == CHECKSUM
    return ch == CHECKSUM



class WlDVLGenericError(Exception):
    """ Generic error """


class WlProtocolParseError(WlDVLGenericError):
    """ Error parsing sentence """


class WlProtocolChecksumError(WlProtocolParseError):
    """ Sentence checksum is invalid """


class WlProtocolParser(object):
    """
    Water Linked DVL protocol parser
    """
    def __init__(self):
        self.crc_func = None
        self.crc_func = crcmod.predefined.mkPredefinedCrcFun("crc-8")

    @staticmethod
    def do_format_checksum(checksum):
        if IS_PY2:
            return bytes("*{:02x}".format(checksum))

        return bytes("*{:02x}".format(checksum), "ascii")

    def checksum_for_buffer(self, data):
        if IS_PY2:
            csum = self.crc_func(bytes(data))
        else:
            csum = self.crc_func(data)
        return self.do_format_checksum(csum)

    def parse(self, sentence, oym_cmd):
        sop = sentence[0]
        #print(sentence)
        if isinstance(sop, bytes):
            sop = ord(sop)
        if sop != SOP:
            return b'False'
            # This will swallow LF following a CR and garbage
            raise WlProtocolParseError("Missing SOP: Got {} Expected {}".format(sop, SOP))
        if len(sentence) < 3:  # Shortest possible command is 3, SOP+DIR+CMD
            raise WlProtocolParseError("Sentence is too short")

        direction = sentence[1]
        if isinstance(direction, bytes):
            direction = ord(direction)
        if direction not in [DIR_CMD, DIR_RESP]:
            raise WlProtocolParseError("Invalid direction {}: {}".format(direction, sentence))

        got_checksum = is_checksum(sentence[-3])
        csum = ""
        if got_checksum:
            csum = sentence[-3:]
            sentence = sentence[:-3]  # Remove checksum to ease further processing
            if csum != self.checksum_for_buffer(sentence):
                expect = self.checksum_for_buffer(sentence)
                raise WlProtocolChecksumError("Expected {} got {}".format(expect, csum))

        cmd = sentence[2]
        
        if isinstance(cmd, bytes):
            #print("Is cmd")
            cmd = ord(cmd)
        
        #if cmd in ALL_VALID:
        fragments = sentence.split(b',')
        options = None
        if len(fragments) > 1:
            options = fragments[1:]

        
        #print(cmd," ",self.doDict(cmd, direction, options))
        return self.doDict(oym_cmd, direction, options)
        #print("1111111111")
        #return None

    def doDict(self, oym_cmd, direction, options):
        #print("oym_cmd: ",oym_cmd)
        if oym_cmd == 'x':
            velocity_result = {
                "property":"velocity",
                "time_stamp": float(options[0].decode('utf-8')),
                "vx": float(options[1].decode('utf-8')),
                "vy": float(options[2].decode('utf-8')),
                "vz": float(options[3].decode('utf-8')),
                "fom": float(options[4].decode('utf-8')),
                "altitude": float(options[5].decode('utf-8')),
                "valid": True if options[6].decode('utf-8') == 'y' else False,
            }
            #print(velocity_result)
            return velocity_result
        elif oym_cmd == 'p':
            position_result = {
                "property":"position",
                "time_stamp":float(options[0].decode('utf-8')),
                "x":float(options[1].decode('utf-8')),
                "y":float(options[2].decode('utf-8')),
                "z":float(options[3].decode('utf-8')),
                "pos_std":float(options[4].decode('utf-8')),
                "roll":float(options[5].decode('utf-8')),
                "pitch":float(options[6].decode('utf-8')),
                "yaw":float(options[7].decode('utf-8')),
                "status":float(options[8].decode('utf-8'))
            }
            #print(position_result)
            return position_result

        else:
            return None


class WlDVLBase(object):
    """
    Water Linked DVL protocol parser base class
    """

    def __init__(self, iodev, debug=False):
        self._iodev = iodev
        self.parser = WlProtocolParser()

        self.payload_size = -1

        self._holdoff = 0
        self._buffer = bytearray()
        self.debug = debug

        self._rx_queue = list()

        self.oldString = ""

    # --------------------
    # Public API functions
    # --------------------
    
    def position_reset(self):
        #航迹推算清零
        result=self._iodev.write("wcr\n".encode("utf-8"))
        #sleep(0.5)


    def getData(self):
        oldString = self.oldString
        contain = 0
        raw_data = ""
        while contain == 0:
            raw_data = raw_data + self._iodev.read(1).decode("utf-8")
            if "\r\n" in raw_data:
                contain = 1
        raw_data = oldString + raw_data
        oldString = ""
        raw_data = raw_data.split("\r\n")
        oldString = raw_data[1]
        raw_data = raw_data[0] + "\r\n"
        self.oldString = oldString
        
        return raw_data

    def getPack(self):
        raw_data = 'False'
        while raw_data == 'False':
            try:
                raw_data = self._iodev.readline().decode("utf-8")
            except:
                raw_data == 'False'
                print("error,wait..")

        return raw_data

    def read(self):
        raw = ""
        nul_strip = ""
        while nul_strip == "":
            try:
                raw = ""
                raw = str(self.getPack().encode('utf-8'), 'utf-8')
                #print(raw)                
                try:
                    nul_strip = "w" + raw.split("w")[1]    #以w切分字符串，去除w
                    nul_strip = nul_strip.split("\r\n")[0]   #去除\r\n
                except Exception as err1:
                    pass
                    #print("Failed to fetch complete message: {}".format(err1))
                # if(len(nul_strip)>1):
                #     print(nul_strip[2])
                for c in nul_strip:
                    self._buffer.append(ord(c))
                #print(raw)
                #print(nul_strip)
                #print(self._buffer)
                if len(nul_strip) < 3:
                    return None
                if nul_strip != "":
                    packet = self.parser.parse(self._buffer,nul_strip[2])
                    #print(packet)
                    self._buffer = bytearray()
                    return packet
            except WlProtocolParseError as err:
                log.warning("Connect error: {}".format(err))

        self._buffer = bytearray()

        return None


class WlDVL(WlDVLBase):
    """
    Water Linked DVL protocol parser
    """

    def __init__(self, device, baudrate=115200, debug=False):
        try:
            self._serial = serial.Serial(device, baudrate)
            # #航迹推算清零
            # result=self._serial.write("wcr\n".encode("utf-8"))
            # sleep(0.5)

        except Exception as err:
            raise WlDVLGenericError("Error opening serial port {}".format(err))

        super(WlDVL, self).__init__(self._serial, debug=debug)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Water Linked DVL protocol parser test")
    parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='Serial port device')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    try:
        dvl = WlDVL(args.port, args.baud)
        print(f"Opened serial port {args.port} at {args.baud} baud.")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        exit(1)

    try:
        while True:
            try:
                data = dvl.read()
                if data:
                    print("\nReceived DVL Data:")
                    for k, v in data.items():
                        print(f"{k}: {v}")
            except Exception as e:
                print(f"Data conversion error: {e}")
            sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")