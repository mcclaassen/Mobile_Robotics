#!/usr/bin/env python3


#use to transfer file to RPi
# $ sftp johndanger@raspberrypi.local
# $ put /Users/jdanger/Desktop/Mobile_Robotics/objs_LiDAR.py /home/johndanger/catkin_ws/src/hmwk1/scripts/objs_LiDAR.py
# $ put C:/Users/claassen/Desktop/Mobile_Robotics/objs_LiDAR.py /home/johndanger/catkin_ws/src/hmwk1/scripts/objs_LiDAR.py


import serial

class D300_LiDAR:
    port_by_connection_type = {
        'Mac' : '/dev/tty.usbserial-0001',
        'RPi' : '/dev/ttyUSB0',
        'Windows' : 'COM4',
    }
    #LD19/D300? Instructions:
    #   Uses DTOF (digital time of flight?)
    #   Can measure 4,500 times per second
    #   Measures distance and angle to form point cloud
    #   Stabilizes within 3 seconds of powering on
    #   Scans 2-D plane parallel to base (despite seeing lights pointing upwards)
    #   Can be speed controlled externally (rotation rate)
    #   Transmits data upon power-up, no commands necessary
    #   Data packet consists of:
    #       Header (1 byte): 0x54 (54 in hexidecimal)
    #       Packet info (1): 0x2C (packet type & # of measurement points (always 12)
    #       Speed (2): degrees/second
    #       Start Angle (2): 100ths of a degrees
    #       Data (3 x 12): each point has distance (2 bytes ) and intensity (1), distance in mm & intensity has no units
    #       End angle (2): 100ths of a degree
    #       Timestamp (2): ms, maxes out at 30,000 then starts over
    #       CRC check (1): verification of preceeding bytes in packet

    # So each packet is 1+1+2+2+36+2+2+1 = 47 bytes long

    def __init__(self, connection_type='', port=None):

        try:
            self.port = self.port_by_connection_type[connection_type]
        except KeyError:
            assert port is not None, 'Either pass connection type or custom port name'
            self.port = port

        self.conn_type = connection_type
        self.timeout_counter = 0
        self.scan_counter = 0
        self.buffer = b''

    def __enter__(self):
        #setup serial connection
        self.conn = serial.Serial(self.port)
        self._set_connection_params()
        return self.conn


    def __exit__(self, exception_type, exception_value, traceback):
        self.conn.close()


    def _set_connection_params(self):
        self.conn.baudrate = 230400
        self.conn.bytesize = serial.EIGHTBITS
        self.conn.stopbits = serial.STOPBITS_ONE
        self.conn.parity = serial.PARITY_NONE
        self.conn.timeout = 5

    @staticmethod
    def big_endian_at_loc(bytes_list, loc):
        return bytes_list[loc] + (bytes_list[loc+1] << 8)

    @staticmethod
    def little_endian_at_loc(self, bytes_list, loc):
        return bytes_list[loc+1] + (bytes_list[loc] << 8)

    def read_bytes(self, num_bytes=1, store=True):

        if not hasattr(self, 'conn'):
            raise RuntimeError('Serial connection not configured correctly')

        if store:
            self.buffer = self.conn.read(num_bytes)
        else:
            return self.conn.read(num_bytes)

        # Below is messy as starts to open up all kinds of connections
        # If want to run w/o a "with" statement, just call __enter__ manually
        # except (serial.PortNotOpenError, AttributeError):  #context manager generated port not open
        #     try:
        #         self.__enter__()
        #         self.buffer = self.conn.read(num_bytes)
        #         self.__exit__(None, None, None)
        #     except Exception:  #make sure connection closes even if exception in __enter/exit__ or .conn.read()
        #         self.conn.close()


    def read_packet(self, num_bytes_in_packet=47):

        #read full data packet
        #read() outputs byte string, this prints to std_out as a mix of ascii char ("1./&kls3" ect) and hex ("\x4f")
        #this makes it look like very weird hex values by it is really many char that are each selected individually
        #either a single ascii chr can be selected ("3" or "@") or a single hex, and selecting each returns a single int
        #so list gets all 47 int values from the byte string
        byteString = self.conn.read_until(b'\x54') #header will be at end
        packet = list(byteString)
        if len(packet) != num_bytes_in_packet or packet[0] != 0x2C:  #packet info starts list
            self.timeout_counter += 1
            self.buffer = b''
        else:
            self.buffer = packet


    def dep_skip_to_packet_start(self):

        #find start of a data packet
        self.read_bytes()
        while self.buffer != b'\x54':
            self.read_bytes()
        #throw away remainder of first data packet so next one will include header
        self.read_bytes(46, store=False)


    def process_packet(self, test_bytes=None):

        #get stored packet data or test data (for testing if works on specific bytes, allows instruction examples to be checked)
        packetValues = self.buffer if test_bytes is None else test_bytes

        #check header and packet info (these are constants given in lidar instructions)
        assert packetValues[-1] == 0x54 and packetValues[0] == 0x2C, f'header = {packetValues[-1]} and packet info = {packetValues[0]}'
        #check that packet info is as given in lidar instructions (packet type = 1, num of measurements = 12)
        # >> shifts binary 5 right (leaving left 3), & takes right 5 digits by boolean logic

        metaData = packetValues.pop(0)
        numMeasurements = metaData & 0b11111
        assert metaData >> 5 == 1 and numMeasurements == 12, 'wrong meta data'

        #2 byte values are made up of Least/Most Significant Byte (LSB/MSB) which represent a single 16-bit number
        #MSB needs to be added to front of LSB (by shifting 8 bits left) and adding the two (could also add MSB*256)
        #data is "big-endian", meaning the MSB is last in byte array
        speed = self.big_endian_at_loc(packetValues, 0)
        startAngle = self.big_endian_at_loc(packetValues, 2)
        endAngle = self.big_endian_at_loc(packetValues, 40)
        timestamp = self.big_endian_at_loc(packetValues, 42)
        crcCheck = packetValues[44]  #not sure how this works yet, below is simplified code from instructions

        #store packet info
        packetInfo = (speed, startAngle / 100, endAngle / 100, timestamp / 1000, crcCheck)  #units now degrees & s

        #distance is 2 bytes, like speed/angles, intensity is a single byte
        #distance units are mm and intensity has no units (should be ~200 normally)
        pointExtractRange = range(4, 4+numMeasurements*3, 3)
        packetData = [(self.big_endian_at_loc(packetValues, i), packetValues[i+2]) for i in pointExtractRange]


        return packetInfo, packetData



    def check_packet_checksum(self):
        pass
        # def CalCRC8(*p (8), len (8)):
        #     crc (8) = 0
        #     for (i (16) = 0; i < len; i++)
        #         crc = CrcTable[(crc ^ *p++) & 0xff]
        # return crc (8)