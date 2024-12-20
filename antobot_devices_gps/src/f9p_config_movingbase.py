#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     The purpose of this code is to configure and get the firmware version of the ublox f9p chip via SPI communication. 
#			     The configuration must be written to both flash and ram layer if its required to save permanently, if so the value of 'layers' in packet must be set as 0x05
#			     This script needs to run only once

# Contact:Aswathi Muralidharan 
# email: aswathi.muralidharan@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import spidev

class GPS:
    """GPS parsing module.	Can parse simple NMEA data sentences from SPI
	GPS modules to read latitude, longitude, and more.
    """
    def __init__(self,spiport):
        self.spiport = spiport
        # Initialize null starting values for GPS attributes.

    def prepare_cfg_packet(self, length):
        
        packet = bytearray(length)
        
        # prepare packet
        packet[0] = 0xb5
        packet[1] = 0x62
        packet[2] = 0x06
        packet[3] = 0x8a
        packet = set_length(packet, length)
        packet[6] = 0x00 # version
        packet[7] = 0x05 # layers 
        packet[8] = 0x00 # reserved
        packet[9] = 0x00 # reserved

    def set_length(self, packet, length):
        if length==20:
            packet[4] = 0x0a
        elif length==19:
            packet[4] = 0x09
        else:
            length_i = length - 8
            packet[4] = length_i.to_bytes(1,'big')
        packet[5] = 0x00 # length 1

        return packet

    def calculate_checksum(self, packet, length):
        # calculate ubx checksum
        chk_a = 0
        chk_b = 0
        for i in range(2, length-2):
            chk_a = chk_a + packet[i]
            chk_b = chk_b + chk_a        
        packet[length-2] = chk_a & 0xff
        packet[length-1] = chk_b & 0xff

        return packet
    
    def revert_to_default_mode(self):
        
        length = 21
        
        packet = bytearray(length)
        # prepare packet
        packet[0] = 0xb5
        packet[1] = 0x62
        packet[2] = 0x06
        packet[3] = 0x09
        packet[4] = 0x0d # length 0
        packet[5] = 0x00 # length 1

        packet[6] = 0xff 
        packet[7] = 0xff
        packet[8] = 0xff 
        packet[9] = 0xff
        packet[10] = 0x00
        packet[11] = 0x00
        packet[12] = 0x00
        packet[13] = 0x00
        packet[14] = 0xff
        packet[15] = 0xff
        packet[16] = 0xff
        packet[17] = 0xff
        packet[18] = 0x0f
        
        self.calculate_checksum(packet, length)
        
        return packet

    def cfg_rate_meas(self, rate):
        """Prepares UBX protocol sentence to set the measurement rate configuration"""
        """ Measurement rate is Nominal time between GNSS measurements"""
        """Key ID: 0x30210001"""
        #print("In config_rate")
        #CFG-RATE-MEAS ; VALUE = 0x7d = 125ms = 8Hz
        
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x01
        packet[11] = 0x00
        packet[12] = 0x21
        packet[13] = 0x30
        #value
        if rate == 5:
            packet[14] = 0xc8 #5hz:c8; 8hz:7d;
        elif rate == 8:
            packet[14] = 0x7d
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)

        return packet
        
    def cfg_rate_nav(self):
        """Prepares UBX protocol sentence to set the navigation rate"""
        """ navigation rate Ratio of number of measurements to number of navigation solutions"""
        """ KEY ID: 0x30210002"""
        #print("In config_rate_nav")
        #CFG-RATE-NAV ; VALUE = 0x01

        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x02
        packet[11] = 0x00
        packet[12] = 0x21
        packet[13] = 0x30
        #value
        packet[14] = 0x01 
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)

        return packet
    
    def cfg_valget_uart1_baudrate(self):

        # prepare packet
        length = 20
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x01
        packet[11] = 0x00
        packet[12] = 0x52
        packet[13] = 0x40
        #value
        packet[14] = 0x00
        packet[15] = 0x08
        packet[16] = 0x07
        packet[17] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
    
    def cfg_valget_uart2_baudrate(self):
        
        # prepare packet
        packet = self.prepare_cfg_packet(20)
        #keyid
        packet[10] = 0x01
        packet[11] = 0x00
        packet[12] = 0x53
        packet[13] = 0x40
        #value
        packet[14] = 0x00
        packet[15] = 0x08
        packet[16] = 0x07
        packet[17] = 0x00

        packet = self.calculate_checksum(packet, length)
        
        return packet
    
    def receive_gps(self):
        """Prepares UBX protocol sentence to get the HPPOSLLH"""
        packet = bytearray(8)
        # prepare packet
        packet[0] = 0xb5
        packet[1] = 0x62
        packet[2] = 0x01 #class id
        packet[3] = 0x14 #message id
        packet[4] = 0
        packet[5] = 0
        packet = self.calculate_checksum(packet, length)

        return packet
    
    def get_ver(self):
        """Prepares UBX protocol sentence to get the firmware version"""
        packet = bytearray(8)
        # prepare packet
        packet[0] = 0xb5
        packet[1] = 0x62
        packet[2] = 0x0a #class id
        packet[3] = 0x04 #message id
        packet[4] = 0
        packet[5] = 0
        packet = self.calculate_checksum(packet, length)

        return packet

    def write(self,payload):
       """function to write the poll request to the ublox f9p chip"""
       #print("Payload length:",type(payload))
       self.spiport.writebytes(payload)
       return 1
    
    def check_ubx_uart(self, received_bytes):
        """function defintion to check the acknowledgment message from ublox"""
        read_bytes = len(received_bytes)
        header = False    # packet header flag
        if (read_bytes > 0):   
            i = 1
            while (i < read_bytes): # read until no bytes left
                if (header == False and received_bytes[i - 1] == 0xb5 and received_bytes[i] == 0x62):
                    header = True
                    packet_start = i - 1
                    i = i + 1
                elif (header == True and received_bytes[i - 1] == 0x05 and (received_bytes[i] == 0x01 or received_bytes[i] == 0x00)):
                    if (received_bytes[i] == 0x01):
                        print("[ubx] UBX-ACK-ACK")
                    elif (received_bytes[i] == 0x00):
                        print("[ubx] UBX-ACK-NACK")
                    i = i + 1
                else:
                    i = i + 1
    
    def receive_ubx_bytes_from_spi(self):
        """function to read the message from ublox f9p"""
        print("Inside the receive ubx bytes function")
        buffer_size = 2048
        received_bytes = bytearray(buffer_size)
        start = False  # flag showing that synchronization character is found
        i = 0 # iteration
        j = 0 #
        while (j < 2048):  # max. number of iterations
            byte = self.spiport.readbytes(1)[0] # read one byte from spi
            if (byte == 0xb5): # if synchronization character is found
                start = True
            if (start == True): # if the packet start is found
                received_bytes[i] = byte # put byte to the buffer
                i = i + 1
            if (start == True and i == buffer_size):
                break # if buffer is full, exit loop
            j = j + 1
        return received_bytes

    def disable_gxgsv(self):
        """ configure the GSV message in SPI"""
        """ KEY ID: 0x209100c8"""
        # prepare packet
        length=18
        packet = self.prepare_cfg_packet(length)

        #key id
        packet[10] = 0xc8
        packet[11] = 0x00
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x00 # 0:disable, 1:enable
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
        
    def disable_gxrmc(self):
        """ configure the RMC message in SPI"""
        """ KEY ID: 0x209100af"""
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #key id
        packet[10] = 0xaf
        packet[11] = 0x00
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x00 #0: disable, 1: enable
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
        
    def disable_gxgsa(self):
        """ configure the GSA message in SPI"""
        """ KEY ID: 0x209100c3"""
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0xc3 
        packet[11] = 0x00
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x00 # 0: disbale, 1: enable
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
        
    def disable_gxvtg(self):
        """ configure the VTG message in SPI"""
        """ KEY ID: 0x209100b4"""

        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0xb4 
        packet[11] = 0x00
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x00 # 0: disbale, 1: enable
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
        
    def disable_gxgll(self):
        """ configure the GLL message in SPI"""
        """ KEY ID: 0x209100cd"""
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(18)
        
        #keyid
        packet[10] = 0xcd 
        packet[11] = 0x00 
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x00 # 0: disbale, 1: enable
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
        
    def config_f9p(self):      

        #disable the message type gxgsv
        ubx_rate_meas = self.cfg_rate_meas()
        self.spiport.writebytes(ubx_rate_meas)
        
        received_bytes = self.receive_ubx_bytes_from_spi()
        #print("Received bytes from ubx_disable_gxgsv",received_bytes)
        self.check_ubx_uart(received_bytes) 
        print("configured the measurement rate as 8Hz") 

        #disable the message type gxgsv
        ubx_disable_gxgsv = self.disable_gxgsv()
        self.spiport.writebytes(ubx_disable_gxgsv)
        
        received_bytes = self.receive_ubx_bytes_from_spi()
        self.check_ubx_uart(received_bytes) 
        print("disabled gxgsv") 
        
        #disable the message type gxrmc
        ubx_disable_gxrmc = self.disable_gxrmc()
        self.spiport.writebytes(ubx_disable_gxrmc)
    
        received_bytes = self.receive_ubx_bytes_from_spi()
        #print("Received bytes from set_baudrate",ubx_str)
        self.check_ubx_uart(received_bytes)
        print("disabled gxrmc") 
        
        #disable the message type gxgsa
        ubx_disable_gxgsa = self.disable_gxgsa()
        self.spiport.writebytes(ubx_disable_gxgsa)
    
        received_bytes = self.receive_ubx_bytes_from_spi()
        #print("Received bytes from set_baudrate",ubx_str)
        self.check_ubx_uart(received_bytes)
        print("disabled gxgsa") 
        
        #disable the message type gxvtg
        ubx_disable_gxvtg = self.disable_gxvtg()
        self.spiport.writebytes(ubx_disable_gxvtg)
    
        received_bytes = self.receive_ubx_bytes_from_spi()
        #print("Received bytes from set_baudrate",ubx_str)
        self.check_ubx_uart(received_bytes)
        print("disabled gxvtg") 
        
        #disable the message type gxgll
        ubx_disable_gxgll = self.disable_gxgll()
        self.spiport.writebytes(ubx_disable_gxgll)
    
        received_bytes = self.receive_ubx_bytes_from_spi()
        #print("Received bytes from set_baudrate",ubx_str)
        self.check_ubx_uart(received_bytes)
        print("disabled gxgll") 

    # moving base
    # HPG 1.32
    def cfg_msgout_uart2_rtcm1074(self):
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x60
        packet[11] = 0x03
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x01 
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet

    def cfg_msgout_uart2_rtcm1084(self):
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x65
        packet[11] = 0x03
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x01 
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
    
    def cfg_msgout_uart2_rtcm1094(self):
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x6a
        packet[11] = 0x03
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x01 
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)

        return packet
    
    def cfg_msgout_uart2_rtcm1124(self):
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x6f
        packet[11] = 0x03
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x01 
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
 
    def cfg_msgout_uart2_rtcm1230(self):
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x05
        packet[11] = 0x03
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x01 
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet
    
    def cfg_msgout_uart2_rtcm4072_0(self):
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = 0x00
        packet[11] = 0x03
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        packet[14] = 0x01 
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet    

    
    def config_uart2_rtcm(self):
        ## HPG 1.32
        packet = self.cfg_msgout_uart2_rtcm1074()
        self.spiport.writebytes(packet)
        print('set uart2 output rtcm1074')

        packet = self.cfg_msgout_uart2_rtcm1084()
        self.spiport.writebytes(packet)
        print('set uart2 output rtcm1084')

        packet = self.cfg_msgout_uart2_rtcm1094()
        self.spiport.writebytes(packet)
        print('set uart2 output rtcm1094')

        packet = self.cfg_msgout_uart2_rtcm1124()
        self.spiport.writebytes(packet)
        print('set uart2 output rtcm1124')

        packet = self.cfg_msgout_uart2_rtcm1230()
        self.spiport.writebytes(packet)
        print('set uart2 output rtcm1230')

        packet = self.cfg_msgout_uart2_rtcm4072_0()
        self.spiport.writebytes(packet)
        print('set uart2 output rtcm4072.0')


        # set uart_2 baud 460800
        packet = gps.cfg_valget_uart2_baudrate()
        gps.spiport.writebytes(packet)
        print('set uart2 baud 460800')

if __name__ == '__main__':
    try:
        spiport = spidev.SpiDev()
        spiport.open(2, 0)  # (2,0) for spi1, (0,0) for spi0
        spiport.max_speed_hz =  7800000 # 1000000, 15600000,62400000....
        spiport.mode = 0
        spiport.no_cs      
        gps = GPS(spiport)         

        #function to get the f9p firmware version
        packet = gps.get_ver()
        gps.write(packet)
        received_bytes = gps.receive_ubx_bytes_from_spi()
        #print("Firmware version of Ublox F9P: ",received_bytes) # To print out the firmware version of F9P if required

        # revert to the default mode
        packet = gps.revert_to_default_mode()
        gps.write(packet)

        #configure the f9p to block unwanted messages
        gps.config_f9p()
        
        # configure the uart2's output (for movingbase)
        gps.config_uart2_rtcm()

        #receive GPS messages
        packet = gps.receive_gps()
        gps.write(packet)
        received_bytes = gps.receive_ubx_bytes_from_spi()
        #print("GPS Message: ",received_bytes) # To print out the firmware version of F9P if required      

    except:
        pass
