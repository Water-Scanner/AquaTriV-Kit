# encoding=utf-8
"""
Complete Water Linked DVL Protocol Parser with ROS Message Format Support
"""
from __future__ import print_function, division
import logging
import time
import sys
import serial
import crcmod
import struct
from time import sleep
import re

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

# Protocol command definitions
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
TRANSDUCER_REPORT = ord('t')
NAVIGATION_REPORT = ord('z')
BEAM_REPORT = ord('u')
POSITION_REPORT = ord('p')

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
    TRANSDUCER_REPORT,
    NAVIGATION_REPORT,
    BEAM_REPORT,
    POSITION_REPORT
]

def is_checksum(ch):
    """Check if the given byte is a checksum character"""
    if isinstance(ch, bytes):
        return ord(ch) == CHECKSUM
    return ch == CHECKSUM

class WlDVLGenericError(Exception):
    """Generic error"""

class WlProtocolParseError(WlDVLGenericError):
    """Error parsing sentence"""

class WlProtocolChecksumError(WlProtocolParseError):
    """Sentence checksum is invalid"""

class WlProtocolParser(object):
    """Complete DVL protocol parser with ROS message format support"""
    def __init__(self):
        self.crc_func = crcmod.predefined.mkPredefinedCrcFun("crc-8")
        self.beam_cache = {}
        self.last_beam_time = 0
        # 新增：组帧缓存
        self.frame_cache = {
            'velocity': None,
            'beams': {},
            'position': None
        }

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

    def clean_field(self, field):
        s = field.decode('utf-8').strip()
        if '*' in s:
            s = s.split('*')[0]
        s = s.replace('\r', '').replace('\n', '')
        # 只保留数字、负号、小数点
        match = re.match(r'^-?\d+(\.\d+)?', s)
        if match:
            return match.group(0)
        return s

    def parse(self, sentence, oym_cmd):
        """Parse a DVL sentence and return ROS-compatible data structure"""
        # Command validation
        if isinstance(oym_cmd, bytes):
            cmd = ord(oym_cmd)
        else:
            cmd = oym_cmd
            
        if isinstance(cmd, int):
            cmd_int = cmd
        else:
            cmd_int = ord(cmd)

        if cmd_int not in ALL_VALID:
            raise WlProtocolParseError(f"Invalid command: {cmd}")

        # Basic validation
        sop = sentence[0]
        if isinstance(sop, bytes):
            sop = ord(sop)
        if sop != SOP:
            return None
            
        if len(sentence) < 3:
            raise WlProtocolParseError("Sentence is too short")

        direction = sentence[1]
        if isinstance(direction, bytes):
            direction = ord(direction)
        if direction not in [DIR_CMD, DIR_RESP]:
            raise WlProtocolParseError("Invalid direction")

        # Checksum verification
        if is_checksum(sentence[-3]):
            csum = sentence[-3:]
            sentence = sentence[:-3]
            if csum != self.checksum_for_buffer(sentence):
                raise WlProtocolChecksumError("Checksum mismatch")

        cmd = sentence[2]
        if isinstance(cmd, bytes):
            cmd = ord(cmd)

        fragments = sentence.split(b',')
        options = fragments[1:] if len(fragments) > 1 else None

        # 关键修正：将cmd转为字符传递给doDict
        return self.doDict(chr(cmd), direction, options)

    def doDict(self, oym_cmd, direction, options):
        """Convert to ROS message format"""
        try:
            # velocity包
            if oym_cmd == 'x':
                velocity = {
                    'time': float(self.clean_field(options[0])),
                    'x': float(self.clean_field(options[1])),
                    'y': float(self.clean_field(options[2])),
                    'z': float(self.clean_field(options[3])),
                    'fom': float(self.clean_field(options[4])),
                    'altitude': float(self.clean_field(options[5])),
                    'velocity_valid': self.clean_field(options[6]) == 'y',
                }
                self.frame_cache['velocity'] = velocity
                return None
            # beam包
            elif oym_cmd == 'u':
                beam_id = int(self.clean_field(options[0]))
                beam = {
                    'id': beam_id,
                    'velocity': float(self.clean_field(options[1])),
                    'distance': float(self.clean_field(options[2])),
                    'rssi': float(self.clean_field(options[3])),
                    'nsd': float(self.clean_field(options[4])),
                    'valid': len(options) > 5 and self.clean_field(options[5]) == 'y'
                }
                self.frame_cache['beams'][beam_id] = beam
                return None
            # position包
            elif oym_cmd == 'p':
                # 严格按wrp格式解析
                position = {
                    'time_stamp': float(self.clean_field(options[0])),
                    'x': float(self.clean_field(options[1])),
                    'y': float(self.clean_field(options[2])),
                    'z': float(self.clean_field(options[3])),
                    'pos_std': float(self.clean_field(options[4])),
                    'roll': float(self.clean_field(options[5])),
                    'pitch': float(self.clean_field(options[6])),
                    'yaw': float(self.clean_field(options[7])),
                    'status': int(float(self.clean_field(options[8])))
                }
                self.frame_cache['position'] = position
                # 检查是否收齐一组
                if (self.frame_cache['velocity'] is not None and
                    len(self.frame_cache['beams']) == 4 and
                    self.frame_cache['position'] is not None):
                    now = time.time()
                    data = {
                        'header': {
                            'stamp': {'secs': int(now), 'nsecs': int((now-int(now))*1e9)},
                            'frame_id': 'dvl_link'
                        },
                        'time': self.frame_cache['velocity']['time'],
                        'velocity': {
                            'x': self.frame_cache['velocity']['x'],
                            'y': self.frame_cache['velocity']['y'],
                            'z': self.frame_cache['velocity']['z']
                        },
                        'fom': self.frame_cache['velocity']['fom'],
                        'altitude': self.frame_cache['velocity']['altitude'],
                        'beams': [self.frame_cache['beams'][i] for i in range(4)],
                        'velocity_valid': self.frame_cache['velocity']['velocity_valid'],
                        'status': self.frame_cache['position']['status'],
                        'form': '',
                        'position': self.frame_cache['position']
                    }
                    self.frame_cache = {'velocity': None, 'beams': {}, 'position': None}
                    return data
                else:
                    return None
            else:
                return None
        except Exception as e:
            log.error(f"Data conversion error: {e}")
            return None

    def _init_beam(self, beam_id):
        """Initialize a beam data structure"""
        return {
            'id': beam_id,
            'velocity': 0.0,
            'distance': 0.0,
            'rssi': 0.0,
            'nsd': 0.0,
            'valid': False
        }

    def _parse_velocity(self, msg, options):
        """Parse velocity data (wrx)"""
        msg['time'] = float(self.clean_field(options[0]))
        msg['velocity']['x'] = float(self.clean_field(options[1]))
        msg['velocity']['y'] = float(self.clean_field(options[2]))
        msg['velocity']['z'] = float(self.clean_field(options[3]))
        msg['fom'] = float(self.clean_field(options[4]))
        msg['altitude'] = float(self.clean_field(options[5]))
        msg['velocity_valid'] = self.clean_field(options[6]) == 'y'
        if len(options) > 7:
            msg['status'] = int(self.clean_field(options[7]))
        if len(options) > 8:
            msg['form'] = self.clean_field(options[8])

    def _parse_transducer(self, msg, options):
        """Parse transducer data (wrt)"""
        for i in range(4):
            if i < len(options):
                msg['beams'][i]['distance'] = float(self.clean_field(options[i]))
                msg['beams'][i]['valid'] = float(self.clean_field(options[i])) > 0

    def _parse_navigation(self, msg, options):
        """Parse navigation data (wrz)"""
        msg['velocity']['x'] = float(self.clean_field(options[0]))
        msg['velocity']['y'] = float(self.clean_field(options[1]))
        msg['velocity']['z'] = float(self.clean_field(options[2]))
        msg['velocity_valid'] = self.clean_field(options[3]) == 'y'
        msg['fom'] = float(self.clean_field(options[4]))
        msg['altitude'] = float(self.clean_field(options[5]))
        msg['status'] = int(self.clean_field(options[10]))

    def _parse_beam(self, msg, options):
        """Parse beam data (wru)"""
        beam_id = int(self.clean_field(options[0]))
        if 0 <= beam_id <= 3:
            msg['beams'][beam_id] = {
                'id': beam_id,
                'velocity': float(self.clean_field(options[1])),
                'distance': float(self.clean_field(options[2])),
                'rssi': float(self.clean_field(options[3])),
                'nsd': float(self.clean_field(options[4])),
                'valid': len(options) > 5 and self.clean_field(options[5]) == 'y'
            }

    def _parse_position(self, msg, options):
        """Parse position data (wrp)"""
        msg['time'] = float(self.clean_field(options[0]))
        msg['position'] = {
            'x': float(self.clean_field(options[1])),
            'y': float(self.clean_field(options[2])),
            'z': float(self.clean_field(options[3]))
        }
        msg['status'] = float(self.clean_field(options[8]))

class WlDVLBase(object):
    """Base class for DVL communication"""
    def __init__(self, iodev, debug=False):
        self._iodev = iodev
        self.parser = WlProtocolParser()
        self._buffer = bytearray()
        self.debug = debug
        self.oldString = ""

    def position_reset(self):
        """Reset position tracking"""
        self._iodev.write("wcr\n".encode("utf-8"))

    def getData(self):
        """Read data from the device with proper framing"""
        oldString = self.oldString
        raw_data = ""
        
        while '\n' not in raw_data:
            try:
                rec = self._iodev.read(1).decode("utf-8")
                if not rec:
                    raise Exception("No data received")
                raw_data += rec
            except Exception as e:
                log.error(f"Read error: {e}")
                sleep(0.1)
                continue
        
        raw_data = oldString + raw_data
        parts = raw_data.split('\n', 1)
        if len(parts) > 1:
            self.oldString = parts[1]
            return parts[0]
        return None

    def read(self):
        """Read and parse a complete message"""
        raw_data = self.getData()
        if not raw_data:
            return None
            
        try:
            # Extract message starting with 'w'
            if 'w' in raw_data:
                message_part = 'w' + raw_data.split('w', 1)[1]
                message = message_part.split('\r\n')[0]
                
                # Convert to bytes for parsing
                self._buffer = bytearray(message.encode('utf-8'))
                
                if len(message) >= 3:
                    return self.parser.parse(self._buffer, message[2])
        except Exception as e:
            log.warning(f"Parse error: {e}")
        
        return None

class WlDVL(WlDVLBase):
    """Serial port implementation of DVL interface"""
    def __init__(self, device, baudrate=115200, debug=False):
        try:
            self._serial = serial.Serial(device, baudrate, timeout=1)
        except Exception as err:
            raise WlDVLGenericError(f"Error opening serial port: {err}")
            
        super(WlDVL, self).__init__(self._serial, debug=debug)

# Example usage
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