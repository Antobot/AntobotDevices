# Python library for the SparkFun's line of u-Blox GPS units.
#
# SparkFun GPS-RTK NEO-M8P:
#   https://www.sparkfun.com/products/15005
# SparkFun GPS-RTK2 ZED-F9P:
#   https://www.sparkfun.com/products/15136
# SparkFun GPS ZOE-M8Q:
#   https://www.sparkfun.com/products/15193
# SparkFun GPS SAM-M8Q:
#   https://www.sparkfun.com/products/15210
# SparkFun GPS-RTK Dead Reckoning Phat ZED-F9R:
#   https://www.sparkfun.com/products/16475
# SparkFun GPS-RTK Dead Reckoning ZED-F9R:  
#   https://www.sparkfun.com/products/16344
# SparkFun GPS Dead Reckoning NEO-M9N:
#   https://www.sparkfun.com/products/15712
#
#------------------------------------------------------------------------
# Written by SparkFun Electronics, July 2020
#
# Do you like this library? Help suphard_port SparkFun. Buy a board!
#==================================================================================
# GNU GPL License 3.0
# Copyright (c) 2020 SparkFun Electronics
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# ==================================================================================
#
# The following code is a dependent on work by daymople and the awesome parsing
# capabilities of ubxtranslator: https://github.com/dalymople/ubxtranslator.
#
# pylint: disable=line-too-long, bad-whitespace, invalid-name, too-many-public-methods
#

import struct
import serial
import spidev
import datetime

from . import sparkfun_predefines as sp
from . import core

_DEFAULT_NAME = "Qwiic GPS"
_AVAILABLE_I2C_ADDRESS = [0x42]

class UbloxGps(object):
    """
    UbloxGps

    Initialize the library with the given port.

    :param hard_port:   The port to use to communicate with the module, this
                        can be a serial or SPI port. If no port is given, then the library
                        assumes serial at a 38400 baud rate.

    :return:            The UbloxGps object.
    :rtype:             Object
    """

    device_name = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    def __init__(self, hard_port = None):
        if hard_port is None:
            self.hard_port = serial.Serial("/dev/ttyUSB0/", 38700, timeout=1)
        elif type(hard_port) == spidev.SpiDev:
            sfeSpi = sfeSpiWrapper(hard_port)
            self.hard_port = sfeSpi
        else:
            self.hard_port = hard_port
        #print("self.hard_port in ublox_gps.py is:",self.hard_port)

        # Class message values
        self.ack_ms= {
            'ACK':0x01, 'NAK':0x00
        }
        self.cfg_ms= {
            'OTP':0x41,    'PIO':0x2c,      'PRT':0x00,     'PT2':0x59,     'RST':0x04,
            'SPT':0x64,    'USBTEST':0x58,  'VALDEL':0x8c,  'VALGET':0x8b,
            'VALSET':0x8a
        }
        self.esf_ms= {
            'ALG':0x14,       'INS':0x15,    'MEAS':0x02,  'RAW':0x03,
            'RESETALG':0x13,  'STATUS':0x10
        }
        self.inf_ms= {
            'DEBUG':0x04,  'ERROR':0x00,   'NOTICE':0x02,
            'TEST':0x03,   'WARNING':0x01
        }
        self.mga_ms= {
            'ACK':0x60,       'BDS_EPH':0x03,
            'BDS_ALM':0x03,   'BDS_HEALTH':0x03,      'BDS_UTC':0x03,
            'DBD_POLL':0x80,  'DBD_IO':0x80,          'GAL_EPH':0x02,
            'GAL_ALM':0x02,   'GAL_TIMEOFFSET':0x02,  'GAL_UTC':0x02
        }
        self.mon_ms= {
            'COMMS':0x36,  'GNSS':0x28,  'HW3':0x37,  'PATCH':0x27,
            'PIO':0x24,    'PT2':0x2b,   'RF':0x38,   'RXR':0x21,
            'SPT':0x2f
        }
        self.nav_ms= {
            'ATT':0x05,        'CLOCK':0x22,     'COV':0x36,
            'DOP':0x04,        'EELL':0x3d,      'EOE':0x61,        'GEOFENCE':0x39,
            'HPPOSECEF':0x13,  'HPPOSLLH':0x14,  'ORB':0x34,        'POSECEF':0x01,
            'POSLLH':0x02,     'PVT':0x07,       'RELPOSNED':0x3c,  'SAT':0x35,
            'SBAS':0x32,       'SIG':0x43,       'STATUS':0x03,     'TIMBDS':0x24,
            'TIMEGAL':0x25,    'TIMEGLO':0x23,   'TIMEGPS':0x20,    'TIMELS':0x25,
            'TIMEQZSS':0x27,   'TIMEUTC':0x21,   'VELECEF':0x11,    'VELNED':0x12
        }
        self.time_ms= {
            'TM2':0x03, 'TP':0x01, 'VRFY':0x06
        }

    def send_message(self, ubx_class, ubx_id, ubx_payload = None):
        """
        Sends a ublox message to the ublox module.

        :param ubx_class:   The ublox class with which to send or receive the
                            message to/from.
        :param ubx_id:      The message id under the ublox class with which
                            to send or receive the message to/from.
        :param ubx_payload: The payload to send to the class/id specified. If
                            none is given than a "poll request" is
                            initiated.
        :return: True on completion
        :rtype: boolean
        """

        SYNC_CHAR1 = 0xB5
        SYNC_CHAR2 = 0x62

        if ubx_payload == b'\x00' or ubx_payload is None:
            payload_length = 0
        elif type(ubx_payload) is not bytes and not bytearray:
            ubx_payload = bytes([ubx_payload])
            payload_length = len(ubx_payload)
        elif type(ubx_payload) is bytearray: 
            payload_length = len(ubx_payload)
            #print("payload length",payload_length)
        else:
            payload_length = len(ubx_payload)

        if type(ubx_payload) is not bytearray:
            if payload_length > 0:
                message = struct.pack('BBBBBB', SYNC_CHAR1, SYNC_CHAR2,
                                  ubx_class.id_, ubx_id, (payload_length & 0xFF),
                                  (payload_length >> 8)) + ubx_payload

            else:
                message = struct.pack('BBBBBB', SYNC_CHAR1, SYNC_CHAR2,
                                  ubx_class.id_, ubx_id, (payload_length & 0xFF),
                                  (payload_length >> 8))

        else:		                          
            message = struct.pack('BBBBBB', SYNC_CHAR1, SYNC_CHAR2,
		                          ubx_class.id_, ubx_id, 
		                          (payload_length & 0xFF),
		                          (payload_length >> 8)) + ubx_payload

        checksum = core.Parser._generate_fletcher_checksum(message[2:])
        #print("Message to write", message+checksum)
        self.hard_port.write(message + checksum)

        return True

    def ubx_get_val(self, key_id):
        """
        This function takes the given key id and breakes it into individual bytes
        which are then cocantenated together. This payload is then sent along
        with the CFG Class and VALGET Message ID to send_message(). Ublox
        Messages are then parsed for the requested values or a NAK signifying a
        problem.

        :return: The requested payload or a NAK on failure.
        :rtype: namedtuple
        """

        key_id_bytes = bytes([])
        if type(key_id) != bytes:
            while key_id > 0:
                key_id_bytes = key_id_bytes + bytes([(key_id & 0xFF)])
                key_id = key_id >> 8

        key_id_bytes = key_id_bytes[::-1]
        msg = self.send_message(sp.CFG_CLS, self.cfg_ms.get('VALGET'), key_id_bytes)
        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg
        
    def ubx_set_val(self, key_id,value):
        """
        This function takes the given key id and breakes it into individual bytes
        which are then cocantenated together. This payload is then sent along
        with the CFG Class and VALGET Message ID to send_message(). Ublox
        Messages are then parsed for the requested values or a NAK signifying a
        problem.

        :return: The requested payload or a NAK on failure.
        :rtype: namedtuple
        """
        key_id_bytes = bytes([])
        #value_id_bytes = bytes([])
        layer = 5 # layer both ram and flash
        layer = "0x{:04x}".format(layer) 
        reserved = "0x{:04x}".format(0)
        #value_id_bytes = bytes([])
        #print("key id before",hex(key_id))
        
        key_id = hex(key_id)
        
        if value == 0 or value ==1:
            value = "0x{:02x}".format(value) #hex(value)#padding zeros on the start of value to make it 2-digit hex
        
            #print("value id before",value,type(value))
            value_id_bytes = bytearray.fromhex(value[2::])
            #print("value id bytes",value_id_bytes,len(value_id_bytes))
        else:
            #print("value is not 0 or 1", value)
            #print("type of value in bytes",(value >> 8) & 0xff)
            value_id_bytes = bytearray(4)
            value_id_bytes[0] = value & 0xff
            
            value_id_bytes [1]= (value >> 8) & 0xff
            value_id_bytes [2]= (value >> 16) & 0xff
            value_id_bytes [3]= (value >> 24) & 0xff
            
            #print("value id bytes",value_id_bytes)
            
        
        #key_id_bytes = struct.pack('<Q',int(key_id,base=16))
        
        layer_id_bytes = bytearray.fromhex(layer[2::])
                       
        key_id_bytes = bytearray.fromhex(key_id[2::])[::-1] #making it little endian
        
        
        reserved_id_bytes = bytearray.fromhex(reserved[2::])
        
        #print("key id bytes",key_id_bytes,len(key_id_bytes),type(key_id_bytes))
        
        #print("layer id bytes",layer_id_bytes,len(layer_id_bytes))
        pl = layer_id_bytes+reserved_id_bytes+key_id_bytes+value_id_bytes
        #print("to write",pl)
        msg = self.send_message(sp.CFG_CLS, self.cfg_ms.get('VALSET'), pl) 
        #print("send message",msg)
        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def enable_UART1(self, enable):
        if enable is True:
            self.send_message(sp.CFG_CLS, self.cfg_ms.get('RST'), 0x00)

        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def geo_coords(self):
        """
        Sends a poll request for the NAV class with the PVT Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and PVT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('PVT')) # change this
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        s_payload = self.scale_NAV_PVT(payload)
        #print("PAyload of PVT",s_payload)
        return s_payload

    def hp_geo_coords(self):
        """
        Sends a poll request for the NAV class with the HPPOSLLH Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and HPPOSLLH Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('HPPOSLLH'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        s_payload = self.scale_NAV_HPPOSLLH(payload)
        return s_payload

    def velned(self):
        """
        Sends a poll request for the NAV class with the VELNED Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and VELNED Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('VELNED'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        s_payload = self.scale_NAV_VELNED(payload)
        return s_payload



    def date_time(self):
        """
        Sends a poll request for the NAV class with the PVT Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and PVT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('PVT'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        s_payload = self.scale_NAV_PVT(payload)
        print(s_payload)
        return s_payload

    def satellites(self):
        """
        Sends a poll request for the NAV class with the SAT Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and SAT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('SAT'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        #print("Payload of satellite",payload)
        #s_payload = self.scale_NAV_SAT(payload)
        #print("Scaled payload",s_payload)
        #return s_payload
        return payload

    def veh_attitude(self):
        """
        Sends a poll request for the NAV class with the ATT Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and ATT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('ATT'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        s_payload = self.scale_NAV_ATT(payload)
        return s_payload

    def stream_nmea(self,buff):
        """
        Reads directly from the module's data stream, by default this is NMEA
        data.

        :return: Returns NMEA data.
        :rtype: string
        """
        #return self.hard_port.read()
        #print("type of hard port:",self.hard_port)
        if isinstance(self.hard_port, sfeSpiWrapper):
            sentence=self.hard_port.readbuffer(buff)
        else:
            #sentence=self.hard_port.readline().decode('utf-8')
            sentence=self.readbuffer(buff)

        return sentence
        
        
    def readbuffer(self, buff):
        """
        Reads a byte or bytes of data from the SPI port. The bytes are
        converted to a bytes object before being returned.

        :return: The requested bytes
        :rtype: bytes
        
        buffer = bytearray()
        start_pattern=b"$"
        end_pattern = b"\r\n"
        count=0
        #print("in serial readbuffer function")
        #print("time before while",datetime.datetime.now())
        while (count<buff):
            data = self.hard_port.read(1)
            
            #
            buffer.extend(data)
            
            if (data == b"\n"):
                count =count+1
                #print(buffer)
        #print("time after while",datetime.datetime.now())
        #print("print buffer:")
        #print(buffer) 
        #print("end print")       
        start_idx = buffer.rfind(start_pattern)
        #print("start_idx:",start_idx)
        if start_idx != -1:  # Start pattern found
            end_idx = buffer.find(end_pattern, start_idx)
            if end_idx != -1:  # End pattern found
                sentence = buffer[start_idx:end_idx + len(end_pattern)]
                buffer = buffer[end_idx + len(end_pattern):]
                #print( "sentence:", sentence)
                #print("time sentence",datetime.datetime.now())
                #print("buffer:",buffer)
                
                return sentence.decode('utf-8')  # Decode the bytes into a string
                
        """
        count=0
        buff=1
        no_gps_count=0
        buffer = bytearray()
        start_pattern=b"$"
        GGA_pattern = b"$GNGGA"
        end_pattern = b"\r\n"

        #print("UART read buffer function")
        #print("time before while",datetime.datetime.now())
        while (count<1):
            data = self.hard_port.read(1)
            #print("1 byte read from buffer:", data)       
            #print("While loop")
            '''
            if (data == b"\n" ):
                
                if count>0:
                    print("has poll all the data in buffer")
                    break
                else:
                    no_gps_count=no_gps_count+1
                    if no_gps_count>8:
                        print("no gps data from buffer")
                        return
            else:
                buffer.extend(data) 
                if (data == [10]):
                    count =count+1
'''
            buffer.extend(data)
            
            if (data == b"\n"):
                count =count+1
                #print(buffer)
                    #print("count in while loop:",count)
        #print("time after while",datetime.datetime.now())
        #print("print buffer:",buffer)
        if buff>1:
            start_idx = buffer.rfind(GGA_pattern)
        else:
            start_idx = buffer.rfind(start_pattern)
        #print("start_idx:",start_idx)
        if start_idx != -1:  # Start pattern found
            end_idx = buffer.find(end_pattern, start_idx)
            if end_idx != -1:  # End pattern found
                sentence = buffer[start_idx:end_idx + len(end_pattern)]
                buffer = buffer[end_idx + len(end_pattern):]
                #print( "sentence:", sentence)
                #print("time sentence",datetime.datetime.now())
                #print("buffer:",buffer)
                return sentence.decode('utf-8')  # Decode the bytes into a string   
        
    def imu_alignment(self):
        """
        Sends a poll request for the ESF class with the ALG Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and ALG Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('ALG'))
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return payload

    def vehicle_dynamics(self):
        """
        Sends a poll request for the ESF class with the INS Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and INS Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, 0x15)
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return payload

    def esf_measures(self):
        """
        Sends a poll request for the ESF class with the MEAS Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and MEAS Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('MEAS'))
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return payload

    def esf_raw_measures(self):
        """
        Sends a poll request for the ESF class with the RAW Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and RAW Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('RAW'))
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return payload

    def reset_imu_align(self):
        """
        Sends a poll request for the ESF class with the RESETALG Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and RESETALG Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('RESETALG'))
        parse_tool = core.Parser([sp.ACK_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return payload

    def esf_status(self):
        """
        Sends a poll request for the ESF class with the STATUS Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and STATUS Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('STATUS'))
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return payload

    def port_settings(self):
        """
        Sends a poll request for the MON class with the COMMS Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and COMMS Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('COMMS'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def module_gnss_support(self):
        """
        Sends a poll request for the MON class with the GNSS Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and GNSS Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('GNSS'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def pin_settings(self):
        """
        Sends a poll request for the MON class with the HW3 Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and HW3 Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('HW3'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def installed_patches(self):
        """
        Sends a poll request for the MON class with the PATCH Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and HW3 Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('HW3'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def prod_test_pio(self):
        """
        Sends a poll request for the MON class with the PIO Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and PIO Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('PIO'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def prod_test_monitor(self):
        """
        Sends a poll request for the MON class with the PT2 Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and PT2 Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('PT2'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def rf_ant_status(self):
        """
        Sends a poll request for the MON class with the RF Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the ESF Class and RF Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('RF'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg


    def module_wake_state(self):
        """
        Sends a poll request for the MON class with the RXR Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the MON Class and RXR Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.mon_ms.get('RXR'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def sensor_production_test(self):
        """
        Sends a poll request for the MON class with the SPT Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the MON Class and SPT Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.mon_ms.get('SPT'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def temp_val_state(self):
        """
        Sends a poll request for the MON class with the TEMP Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the MON Class and TEMP Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.mon_ms.get('TEMP'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def module_software_version(self):
        """
        Sends a poll request for the MON class with the VER Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the MON Class and VER Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.mon_ms.get('VER'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg

    def scale_NAV_ATT(self, nav_payload):
        """
        This takes the UBX-NAV-ATT payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """

        att_roll = nav_payload.roll
        att_pitch = nav_payload.pitch
        att_head = nav_payload.heading
        att_roll_acc = nav_payload.accPitch
        att_head_acc = nav_payload.accHeading

        nav_payload = nav_payload._replace(roll= att_roll * (10**-5))
        nav_payload = nav_payload._replace(pitch= att_pitch * (10**-5))
        nav_payload = nav_payload._replace(heading= att_head * (10**-5))
        nav_payload = nav_payload._replace(accPitch= att_roll_acc * (10**-5))
        nav_payload = nav_payload._replace(accHeading= att_head_acc * (10**-5))

        return nav_payload

    def scale_NAV_DOP(self, nav_payload):
        """
        This takes the UBX-NAV-DOP payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """

        geo_dop = nav_payload.gDOP
        pos_dop = nav_payload.pDOP
        time_dop = nav_payload.tDOP
        vert_dop = nav_payload.vDOP
        horiz_dop = nav_payload.hDOP
        north_dop = nav_payload.nDOP
        east_dop = nav_payload.eDOP

        nav_payload = nav_payload._replace(gDOP= geo_dop * 0.01)
        nav_payload = nav_payload._replace(pDOP= pos_dop * 0.01)
        nav_payload = nav_payload._replace(tDOP= time_dop * 0.01)
        nav_payload = nav_payload._replace(vDOP= vert_dop * 0.01)
        nav_payload = nav_payload._replace(hDOP= horiz_dop * 0.01)
        nav_payload = nav_payload._replace(nDOP= north_dop * 0.01)
        nav_payload = nav_payload._replace(eDOP= east_dop * 0.01)

        return nav_payload

    def scale_NAV_EELL(self, nav_payload):
        """
        This takes the UBX-NAV-EELL payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """
        err_ellipse = nav_payload.errEllipseOrient
        nav_payload = nav_payload._replace(errEllipseOrient= err_ellipse * (10**-2))

        return nav_payload

    def scale_NAV_HPPOSECEF(self, nav_payload):
        """
        This takes the UBX-NAV-HPPOSECEF payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """
        ecef_x_hp = nav_payload.ecefXHp
        ecef_y_hp = nav_payload.ecefYHp
        ecef_z_hp = nav_payload.ecefZHp
        pos_acc = nav_payload.pAcc

        nav_payload = nav_payload._replace(ecefX=ecef_x_hp * 0.1)
        nav_payload = nav_payload._replace(ecefY=ecef_y_hp * 0.1)
        nav_payload = nav_payload._replace(ecefZ=ecef_z_hp * 0.1)
        nav_payload = nav_payload._replace(pAcc=pos_acc * 0.1)

        return nav_payload

    def scale_NAV_HPPOSLLH(self, nav_payload):
        """
        This takes the UBX-NAV-HPPOSLLH payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """

        print("HPPOSLLH: ", nav_payload)
        longitude = nav_payload.lon
        lon_Hp = nav_payload.lonHp
        latitude = nav_payload.lat
        lat_Hp = nav_payload.latHp
        height_Hp = nav_payload.heightHp
        height_sea = nav_payload.hMSLHp
        horiz_acc = nav_payload.hAcc
        vert_acc = nav_payload.vAcc

        nav_payload = nav_payload._replace(lon=longitude * (10**-7))
        nav_payload = nav_payload._replace(lat=latitude * (10**-7))
        nav_payload = nav_payload._replace(lonHp=lon_Hp * (10**-9))
        nav_payload = nav_payload._replace(latHp=lat_Hp * (10**-9))
        nav_payload = nav_payload._replace(heightHp=height_Hp * 0.1)
        nav_payload = nav_payload._replace(hMSLHp=height_sea * 0.1)
        nav_payload = nav_payload._replace(hAcc=horiz_acc * 0.1)
        nav_payload = nav_payload._replace(vAcc=vert_acc * 0.1)

        return nav_payload

    def scale_NAV_PVT(self, nav_payload):
        """
        This takes the UBX-NAV-PVT payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """
        #print("NAV-PVT",nav_payload)
        longitude = nav_payload.lon
        latitude = nav_payload.lat
        #head_mot = nav_payload.headMot
        #head_acc = nav_payload.headAcc
        pos_dop = nav_payload.pDOP
        #head_veh = nav_payload.headVeh
        #mag_dec = nav_payload.magDec
        #mag_acc = nav_payload.magAcc

        nav_payload = nav_payload._replace(lon=longitude *(10**-7))
        nav_payload = nav_payload._replace(lat=latitude * (10**-7))
        #nav_payload = nav_payload._replace(headMot = head_mot * (10**-5))
        #nav_payload = nav_payload._replace(headAcc = head_acc * (10**-5))
        nav_payload = nav_payload._replace(pDOP= pos_dop * 0.01)
        #nav_payload = nav_payload._replace(headVeh = head_veh * (10**-5))
        #nav_payload = nav_payload._replace(magDec = mag_dec * (10**-2))
        #nav_payload = nav_payload._replace(magAcc = mag_acc * (10**-2))

        return nav_payload

    def scale_NAV_RELPOSNED(self, nav_payload):
        """
        This takes the UBX-NAV-RELPOSNED payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """

        rel_pos_head = nav_payload.relPosHeading
        rel_pos_hpn = nav_payload.relPosHPN
        rel_pos_hpe = nav_payload.relPosHPE
        rel_pos_hpd = nav_payload.relPosHPD
        rel_pos_hpl = nav_payload.relPosHPLength

        acc_N = nav_payload.accN
        acc_E = nav_payload.accE
        acc_D = nav_payload.accD
        acc_L = nav_payload.accLength
        acc_H = nav_payload.accHeading

        nav_payload = nav_payload._replace(relPosHeading = rel_pos_head * (10**-5))
        nav_payload = nav_payload._replace(relPosHPN = rel_pos_hpn * 0.1)
        nav_payload = nav_payload._replace(relPosHPE = rel_pos_hpe * 0.1)
        nav_payload = nav_payload._replace(relPosHPD = rel_pos_hpd * 0.1)
        nav_payload = nav_payload._replace(relPosHPLength = rel_pos_hpl * 0.1)
        nav_payload = nav_payload._replace(accN = acc_N * 0.1)
        nav_payload = nav_payload._replace(accE = acc_E * 0.1)
        nav_payload = nav_payload._replace(accD = acc_D * 0.1)
        nav_payload = nav_payload._replace(accLength = acc_L * 0.1)
        nav_payload = nav_payload._replace(accHeading = acc_H * (10**-5))

        return nav_payload

    def scale_NAV_SAT(self, nav_payload):
        """
        This takes the UBX-NAV-SAT payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """
        pr_res = nav_payload.prRes
        nav_payload = nav_payload._replace(prRes= pr_res * 0.1)

        return nav_payload

    def scale_NAV_VELNED(self, nav_payload):
        """
        This takes the UBX-NAV-VELNED payload and scales the relevant fields
        as it's described in the datasheet.

        :return: Scaled verasion of the given payload.
        :rtype: namedtuple
        """

        velned_course_acc = nav_payload.cAcc
        att_head = nav_payload.heading

        nav_payload = nav_payload._replace(cAcc=velned_course_acc * (10**-5))
        nav_payload = nav_payload._replace(heading= att_head * (10**-5))

        return nav_payload




class sfeSpiWrapper(object):
    """
    sfeSpiWrapper

    Initialize the library with the given port.

    :param spi_port:    This library simply provides some ducktyping for spi so
                        that the ubxtranslator library doesn't complain. It
                        takes a spi port and then sets it to the ublox module's
                        specifications.

    :return:            The sfeSpiWrapper object.
    :rtype:             Object
    """

    def __init__(self, spi_port = None):

        if spi_port is None:
            self.spi_port = spidev.SpiDev()
        else:
            self.spi_port = spi_port

        #print("SPI PORT",self.spi_port)
        # For URCU
        self.spi_port.open(2,0) 
        self.spi_port.max_speed_hz =10000000 #7500000  #1000000 #Hz 7500000
        
        #self.spi_port.open(0,0) # This needs to be
        #self.spi_port.max_speed_hz = 5500 #Hz
        self.spi_port.mode = 0b00
    
    def read(self, read_data = 1):
        """
        Reads a byte or bytes of data from the SPI port. The bytes are
        converted to a bytes object before being returned.

        :return: The requested bytes
        :rtype: bytes
        """

        data = self.spi_port.readbytes(read_data)

        byte_data = bytes([])
        for d in data:
            byte_data = byte_data + bytes([d])
        return byte_data
        #return data
        
    def readbuffer(self, buff):
        """
        Reads a byte or bytes of data from the SPI port. The bytes are
        converted to a bytes object before being returned.

        :return: The requested bytes
        :rtype: bytes
        """
        count=0
        no_gps_count=0
        buffer = bytearray()
        start_pattern=b"$"
        GGA_pattern = b"$GNGGA"
        end_pattern = b"\r\n"
        """
        while True:
          data = self.spi_port.readbytes(read_data)
          buffer.extend(data)
          print(buffer)
          start_idx = buffer.find(start_pattern)
          print("start_idx:",start_idx)
          if start_idx != -1:  # Start pattern found
            end_idx = buffer.find(end_pattern, start_idx)
            if end_idx != -1:  # End pattern found
              sentence = buffer[start_idx:end_idx + len(end_pattern)]
              buffer = buffer[end_idx + len(end_pattern):]
              
              return sentence.decode('utf-8')  # Decode the bytes into a string
             """
        #print("SPI read buffer function")
        #print("time before while",datetime.datetime.now())
        while (count<buff):
            data = self.spi_port.readbytes(1)
            #print("1 byte read from buffer:", data)       
            #print("While loop")
            if (data == [255] ):
                
                if count>0:
                    #print("has poll all the data in buffer")
                    break
                else:
                    no_gps_count=no_gps_count+1
                    if no_gps_count>8:
                        #print("no gps data from buffer")
                        return
            else:
                buffer.extend(data) 
                if (data == [10]):
                    count =count+1

            
                    #print("count in while loop:",count)
        #print("time after while",datetime.datetime.now())
        #print("print buffer:",buffer)
        if buff>1:
            start_idx = buffer.rfind(GGA_pattern)
        else:
            start_idx = buffer.rfind(start_pattern)
        #print("start_idx:",start_idx)
        if start_idx != -1:  # Start pattern found
            end_idx = buffer.find(end_pattern, start_idx)
            if end_idx != -1:  # End pattern found
                sentence = buffer[start_idx:end_idx + len(end_pattern)]
                buffer = buffer[end_idx + len(end_pattern):]
                #print( "sentence:", sentence)
                #print("time sentence",datetime.datetime.now())
                #print("buffer:",buffer)
                return sentence.decode('utf-8')  # Decode the bytes into a string      


    def write(self, data):
        """
        Writes a byte or bytes of data to the SPI port.

        :return: True on completion
        :rtype: boolean
        """
        self.spi_port.xfer2(list(data))

        return True


