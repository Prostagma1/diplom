import logging
import struct

import time

from .transmitter import Transmitter

class MultirotorControl:

    MSPCodes = {
        'MSP_API_VERSION':                1,
        'MSP_FC_VARIANT':                 2,
        'MSP_FC_VERSION':                 3,
        'MSP_BOARD_INFO':                 4,
        'MSP_BUILD_INFO':                 5,

        'MSP_NAME':                       10,
        'MSP_SET_NAME':                   11,

        'MSP_BATTERY_CONFIG':             32,
        'MSP_SET_BATTERY_CONFIG':         33,
        'MSP_MODE_RANGES':                34,
        'MSP_SET_MODE_RANGE':             35,
        'MSP_FEATURE_CONFIG':             36,
        'MSP_SET_FEATURE_CONFIG':         37,
        'MSP_BOARD_ALIGNMENT_CONFIG':     38,
        'MSP_SET_BOARD_ALIGNMENT_CONFIG': 39,
        'MSP_CURRENT_METER_CONFIG':       40,
        'MSP_SET_CURRENT_METER_CONFIG':   41,
        'MSP_MIXER_CONFIG':               42,
        'MSP_SET_MIXER_CONFIG':           43,
        'MSP_RX_CONFIG':                  44,
        'MSP_SET_RX_CONFIG':              45,
        'MSP_LED_COLORS':                 46,
        'MSP_SET_LED_COLORS':             47,
        'MSP_LED_STRIP_CONFIG':           48,
        'MSP_SET_LED_STRIP_CONFIG':       49,
        'MSP_RSSI_CONFIG':                50,
        'MSP_SET_RSSI_CONFIG':            51,
        'MSP_ADJUSTMENT_RANGES':          52,
        'MSP_SET_ADJUSTMENT_RANGE':       53,
        'MSP_CF_SERIAL_CONFIG':           54,
        'MSP_SET_CF_SERIAL_CONFIG':       55,
        'MSP_VOLTAGE_METER_CONFIG':       56,
        'MSP_SET_VOLTAGE_METER_CONFIG':   57,
        'MSP_SONAR':                      58,
        'MSP_PID_CONTROLLER':             59,
        'MSP_SET_PID_CONTROLLER':         60,
        'MSP_ARMING_CONFIG':              61,
        'MSP_SET_ARMING_CONFIG':          62,
        'MSP_RX_MAP':                     64,
        'MSP_SET_RX_MAP':                 65,
        'MSP_SET_REBOOT':                 68,
        'MSP_DATAFLASH_SUMMARY':          70,
        'MSP_DATAFLASH_READ':             71,
        'MSP_DATAFLASH_ERASE':            72,
        'MSP_LOOP_TIME':                  73,
        'MSP_SET_LOOP_TIME':              74,
        'MSP_FAILSAFE_CONFIG':            75,
        'MSP_SET_FAILSAFE_CONFIG':        76,
        'MSP_RXFAIL_CONFIG':              77,
        'MSP_SET_RXFAIL_CONFIG':          78,
        'MSP_SDCARD_SUMMARY':             79,
        'MSP_BLACKBOX_CONFIG':            80,
        'MSP_SET_BLACKBOX_CONFIG':        81,
        'MSP_TRANSPONDER_CONFIG':         82,
        'MSP_SET_TRANSPONDER_CONFIG':     83,
        'MSP_OSD_CONFIG':                 84,
        'MSP_SET_OSD_CONFIG':             85,
        'MSP_OSD_CHAR_READ':              86,
        'MSP_OSD_CHAR_WRITE':             87,
        'MSP_VTX_CONFIG':                 88,
        'MSP_SET_VTX_CONFIG':             89,
        'MSP_ADVANCED_CONFIG':            90,
        'MSP_SET_ADVANCED_CONFIG':        91,
        'MSP_FILTER_CONFIG':              92,
        'MSP_SET_FILTER_CONFIG':          93,
        'MSP_PID_ADVANCED':               94,
        'MSP_SET_PID_ADVANCED':           95,
        'MSP_SENSOR_CONFIG':              96,
        'MSP_SET_SENSOR_CONFIG':          97,
        'MSP_ARMING_DISABLE':             99,
        'MSP_STATUS':                     101,
        'MSP_RAW_IMU':                    102,
        'MSP_SERVO':                      103,
        'MSP_MOTOR':                      104,
        'MSP_RC':                         105,
        'MSP_RAW_GPS':                    106,
        'MSP_COMP_GPS':                   107,
        'MSP_ATTITUDE':                   108,
        'MSP_ALTITUDE':                   109,
        'MSP_ANALOG':                     110,
        'MSP_RC_TUNING':                  111,
        'MSP_PID':                        112,
        'MSP_BOXNAMES':                   116,
        'MSP_PIDNAMES':                   117,
        'MSP_BOXIDS':                     119,
        'MSP_SERVO_CONFIGURATIONS':       120,
        'MSP_MOTOR_3D_CONFIG':            124,
        'MSP_RC_DEADBAND':                125,
        'MSP_SENSOR_ALIGNMENT':           126,
        'MSP_LED_STRIP_MODECOLOR':        127,

        'MSP_VOLTAGE_METERS':             128,
        'MSP_CURRENT_METERS':             129,
        'MSP_BATTERY_STATE':              130,
        'MSP_MOTOR_CONFIG':               131,
        'MSP_GPS_CONFIG':                 132,
        'MSP_COMPASS_CONFIG':             133,
        'MSP_GPS_RESCUE':                 135,

        'MSP_STATUS_EX':                  150,

        'MSP_UID':                        160,
        'MSP_GPS_SV_INFO':                164,

        'MSP_GPSSTATISTICS':              166,

        'MSP_DISPLAYPORT':                182,

        'MSP_COPY_PROFILE':               183,

        'MSP_BEEPER_CONFIG':              184,
        'MSP_SET_BEEPER_CONFIG':          185,

        'MSP_SET_RAW_RC':                 200,
        'MSP_SET_PID':                    202,
        'MSP_SET_RC_TUNING':              204,
        'MSP_ACC_CALIBRATION':            205,
        'MSP_MAG_CALIBRATION':            206,
        'MSP_RESET_CONF':                 208,
        'MSP_SELECT_SETTING':             210,
        'MSP_SET_SERVO_CONFIGURATION':    212,
        'MSP_SET_MOTOR':                  214,
        'MSP_SET_MOTOR_3D_CONFIG':        217,
        'MSP_SET_RC_DEADBAND':            218,
        'MSP_SET_RESET_CURR_PID':         219,
        'MSP_SET_SENSOR_ALIGNMENT':       220,
        'MSP_SET_LED_STRIP_MODECOLOR':    221,
        'MSP_SET_MOTOR_CONFIG':           222,
        'MSP_SET_GPS_CONFIG':             223,
        'MSP_SET_COMPASS_CONFIG':         224,
        'MSP_SET_GPS_RESCUE':             225,

        'MSP_MODE_RANGES_EXTRA':          238,
        'MSP_SET_ACC_TRIM':               239,
        'MSP_ACC_TRIM':                   240,
        'MSP_SERVO_MIX_RULES':            241,
        'MSP_SET_RTC':                    246,

        'MSP_EEPROM_WRITE':               250,
        'MSP_DEBUG':                      254,

    }

    def __init__(self, transmitter: Transmitter):

        self.transmitter = transmitter

    
        self.SENSOR_DATA = {
            'gyroscope':                  [0, 0, 0],
            'accelerometer':              [0, 0, 0],
            'magnetometer':               [0, 0, 0],
            'altitude':                   0,
            'sonar':                      0,
            'kinematics':                 [0.0, 0.0, 0.0],
            'debug':                      [0, 0, 0, 0, 0, 0, 0, 0], # 8 values for special situations like MSP2_INAV_DEBUG
        }

        
        
    def __enter__(self):
            
        self.is_transmitter_open = self.connect()

        if self.is_transmitter_open is True:
            return self
            
        else:
            return 1
            

    def __exit__(self):
            
        if self.transmitter.is_connect is True:
            self.transmitter.disconnect()

            logging.info("Transmitter is closed")


    def connect(self, trials = 100, delay = 1) -> bool:

        for _ in range(trials):
            
            try:
                self.transmitter.connect()

                return True
            
            except:
                logging.error("Transmitter cant connect to serial/port")

            time.sleep(delay)

        return False
    
    def fast_read_imu(self):

        if self.send_RAW_msg(MultirotorControl.MSPCodes['MSP_RAW_IMU']):

            msg = self.receive_raw_msg(size = 24)[5:]
            converted_msg = struct.unpack('<9h' , msg[:-1])

            self.SENSOR_DATA['accelerometer'][0] = converted_msg[0]
            self.SENSOR_DATA['accelerometer'][1] = converted_msg[1]
            self.SENSOR_DATA['accelerometer'][2] = converted_msg[2]


            self.SENSOR_DATA['gyroscope'][0] = converted_msg[3]
            self.SENSOR_DATA['gyroscope'][1] = converted_msg[4]
            self.SENSOR_DATA['gyroscope'][2] = converted_msg[5]

            self.SENSOR_DATA['magnetometer'][0] = converted_msg[6]
            self.SENSOR_DATA['magnetometer'][1] = converted_msg[7]
            self.SENSOR_DATA['magnetometer'][2] = converted_msg[8]

    def fast_read_attitude(self):

        if self.send_RAW_msg(MultirotorControl.MSPCodes['MSP_ATTITUDE']):
            
            msg = self.receive_raw_msg(size = 12)[5:]
            converted_msg = struct.unpack('<3h' , msg[:-1])

            self.SENSOR_DATA['kinematics'][0] = converted_msg[0] / 10.0 # x
            self.SENSOR_DATA['kinematics'][1] = converted_msg[1] / 10.0 # y
            self.SENSOR_DATA['kinematics'][2] = converted_msg[2] # z


    def receive_raw_msg(self, size, timeout = 3):

        msg_header, msg =  self.transmitter.receive(size,timeout)

        return msg_header + msg

    @staticmethod
    def convert(val_list, n=16):
        if n == 16:
            return list(struct.pack('<' + 'H' * len(val_list), *val_list))
        buffer = bytearray()
        for val in val_list:
            for i in range(n // 8):
                buffer.append((int(val) >> (i * 8)) & 0xFF)
        return list(buffer)
    
    
    def reboot(self):
        logging.info("Reboot requested")
        return self.send_RAW_msg(MultirotorControl.MSPCodes['MSP_SET_REBOOT'], data=[])
    
    
    def send_RAW_MOTORS(self, data=[]):
        assert(type(data)==list)
        assert(len(data)==8)

        data = self.convert(data, 16) # any values bigger than 255 need to be converted.
                                    # RC and Motor commands go from 0 to 2000.

        return self.send_RAW_msg(MultirotorControl.MSPCodes['MSP_SET_MOTOR'], data)
    

    def send_RAW_RC(self, data=[]):
        data = self.convert(data, 16) # any values bigger than 255 need to be converted.
                                    # RC and Motor commands go from 0 to 2000.

        return self.send_RAW_msg(MultirotorControl.MSPCodes['MSP_SET_RAW_RC'], data, True, 3)
    

    def send_RAW_msg(self, code, data=[], blocking=True, timeout=-1):
        res = -1

        # Always reserve 6 bytes for protocol overhead
        # $ + M + < + data_length + msg_code + data + msg_crc
        len_data = len(data)
        size = len_data + 6
        checksum = 0

        bufView = bytearray([0]*size)

        bufView[0] = 36 #$
        bufView[1] = 77 #M
        bufView[2] = 60 #<
        bufView[3] = len_data
        bufView[4] = code

        checksum = bufView[3] ^ bufView[4]

        for i in range(len_data):
            bufView[i + 5] = data[i]
            checksum ^= bufView[i + 5]

        bufView[-1] = checksum

        
        res = self.transmitter.send(bufView, blocking,timeout)

        return res