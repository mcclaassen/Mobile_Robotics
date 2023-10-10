import numpy as np
import serial
from matplotlib import pyplot as plt

# LD19 Instructions
# Uses DTOF (digital time of flight?)
# Can measure 4,500 times per second
# Measures distance and angle to form point cloud
# Stabilizes within 3 seconds of powering on
# Scans 2-D plane parallel to base (despite seeing lights pointing upwards)
# Can be speed controlled externally (rotation rate)
# Transmits measurements upon power-up, no commands necessary
# Measurement packet consists of:
#   Header (1 byte): 0x54 (54 in hexidecimal)
#   Measurement info (1): 0x2C (packet type & # of measurement points (always 12)
#   Speed (2): degrees/second
#   Start Angle (2): 100ths of a degrees
#   Data (3 x 12): each point has a 2 byte distance and a 1 byte intensity
#   End angle (2): 100ths of a degree
#   Timestamp (2): ms, maxes out at 30,000 then starts over
#   CRC check (1): verification of preceeding bytes in packet

# So each packet is 1+1+2+2+36+2+2+1 = 47 bytes long

# class LiDAR:
#
#     def __init__(self, port, connection_type=''):
#         self.port = port
#         self.conn_type = connection_type
#
#     #setup serial connection
#     def __enter__(self):
#
#         if self.conn_type == 'Macbook':
#             conn = serial.Serial(self.port)
#             conn.baudrate = 230400
#             conn.bytesize = serial.EIGHTBITS
#             conn.stopbits = serial.STOPBITS_ONE
#             conn.parity = serial.PARITY_NONE
#             conn.timeout = 5
#
#
#     def _uart_connection(self):
#
#
# LiDAR('/dev/tty.usbserial-0001')



with serial.Serial('/dev/tty.usbserial-0001') as serialConn:



    #find start of a measurement packet
    line = serialConn.read()
    while line != b'\x54':
        line = serialConn.read()

    #throw away remainder of first measurement packet for consistency (as it wont have header)
    remainderOfFirstPacket = serialConn.read(46)

    scanNum = 0
    pointsByScanNum = {}
    scanInfoByScanNum = {}
    while scanNum < 1000:

        #read full measurement packet
        #read() outputs byte string, this prints to std_out as a mix of ascii char ("1./&kls3" ect) and hex ("\x4f")
        #this makes it look like very weird hex values by it is really many char that are each selected individually
        #either a single ascii chr can be selected ("3" or "@") or a single hex, and selecting each returns a single int
        #so list gets all 47 int values from the byte string
        measurement = list(serialConn.read(47))

        #check header and measurement info (these are constants given in lidar instructions)
        assert measurement[0] == 0x54 and measurement[1] == 0x2C
        #check that measurement info is as given in lidar instructions (packet type = 1, number measurements = 12)
        # >> shifts binary 5 right (leaving left 3), & takes right 5 digits by boolean logic
        assert measurement[1] >> 5 == 1 and measurement[1] & 0b11111 == 12

        #2 byte values are made up of Least/Most Significant Byte (LSB/MSB) which represent a single 16-bit number
        #MSB needs to be added to front of LSB (by shifting 8 bits left) and adding the two (could also add MSB*256)
        #data is "big-endian", meaning the MSB is last in byte array
        speed = measurement[2] + (measurement[3] << 8)
        startAngle = measurement[4] + (measurement[5] << 8)
        endAngle = measurement[42] + (measurement[43] << 8)
        timestamp = measurement[44] + (measurement[45] << 8)
        scanInfoByScanNum[scanNum] = (speed, startAngle / 100, endAngle / 100, timestamp / 1000)  #units now degrees & s

        pointsByScanNum[scanNum] = []
        for i in range(12):
            #distance is 2 bytes, like speed/angles, intensity is a single byte
            pointsByScanNum[scanNum].append((
                    measurement[6+i] + (measurement[7+i] << 8), measurement[8+i]
            ))


        crcCheck = measurement[46]  #not sure how this works yet, below is simplified code from instructions
        # def CalCRC8(*p (8), len (8)):
        #     crc (8) = 0
        #     for (i (16) = 0; i < len; i++)
        #         crc = CrcTable[(crc ^ *p++) & 0xff]
        # return crc (8)

        scanNum += 1







# points will be plotted in cartesian space
allX, allY = [], []
for scanNum, scanInfo in scanInfoByScanNum.items():

    if scanNum == 0: #first scan cannot have angle traveled calculated for it
        continue

    #unpack scan info
    speed, startAngle, endAngle, timestamp = scanInfo

    #compare the angle travelled as calculated by the rotational speed and the measured angles
    elapsedTime = timestamp - scanInfoByScanNum[scanNum-1][3]  #timestamp of previous scan
    totalAngle_speed = elapsedTime * speed
    totalAngle_angle = endAngle - startAngle
    print(f'Angle diff: {round(totalAngle_speed - totalAngle_angle, 2)} degrees')

    #calculate angle travelled using total angle from two different methods
    angleBetweenPoints_speed = totalAngle_speed / len(pointsByScanNum[scanNum])
    angleBetweenPoints_angle = totalAngle_angle / len(pointsByScanNum[scanNum])
    for distance, intensity in pointsByScanNum[scanNum]:
        # pointAngle = startAngle + angleBetweenPoints_angle  #has shifted section around 0 degrees
        pointAngle = startAngle + angleBetweenPoints_speed  #seems to be similar but doesn't have shifted section
        #convert angle and distance into cartesian coordinates
        allX.append(distance * np.cos(np.deg2rad(pointAngle)))
        allY.append(distance * np.sin(np.deg2rad(pointAngle)))


plt.scatter(allX, allY, s=.1)
plt.show()











