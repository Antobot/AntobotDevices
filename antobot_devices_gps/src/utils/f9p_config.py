#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     The purpose of this code is to configure and get the firmware version of the ublox f9p chip via SPI communication. 
#			     The configuration must be written to both flash and ram layer if its required to save permanently, if so the value of 'layers' in packet must be set as 0x05
#			     This script needs to run only once

# Contact: Daniel Freer 
# email: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import spidev

class F9P_config:
    """GPS parsing module.	Can parse simple NMEA data sentences from SPI
	GPS modules to read latitude, longitude, and more.
    """
    def __init__(self,spiport, desired_messages, meas_rate):
        # Initialize null starting values for GPS attributes.
        self.spiport = spiport
        self.desired_messages = desired_messages
        self.meas_rate = meas_rate
        

    def prepare_cfg_packet(self, length):
        
        packet = bytearray(length)
        
        # prepare packet
        packet[0] = 0xb5
        packet[1] = 0x62
        packet[2] = 0x06
        packet[3] = 0x8a
        packet = self.set_length(packet, length)
        packet[6] = 0x00 # version
        packet[7] = 0x05 # layers 
        packet[8] = 0x00 # reserved
        packet[9] = 0x00 # reserved

        return packet

    def set_length(self, packet, length):
        print(length)
        if length==18:
            packet[4] = 0x0a
        elif length==17:
            packet[4] = 0x09
        else:
            length_i = length - 8
            print("length_i: {}".format(length_i))
            print(length_i.to_bytes(1,'big'))
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

    def cfg_rate_meas(self):
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
        if self.meas_rate == 5:
            packet[14] = 0xc8 #5hz:c8; 8hz:7d;
        elif self.meas_rate == 8:
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
        length = 20
        packet = self.prepare_cfg_packet(length)
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
        length = 8
        packet = bytearray(length)
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
        length = 8
        packet = bytearray(length)
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

    def set_rtcm_messages(self):
        all_messages = ['1074','1084','1094','1124','1230','4072']

        for msg in all_messages:
            if msg == '1074':
                key_id = 0x60
            elif msg == '1084':
                key_id = 0x65
            elif msg == '1094':
                key_id = 0x6a
            elif msg == '1124':
                key_id = 0x6f
            elif msg == '1230':
                key_id = 0x05
            elif msg == '4072':
                key_id = 0x00

            enable = True

            packet = self.config_uart2_rtcm_msg(key_id, enable)
            self.spiport.writebytes(packet)

            enable_text = "Disabled"
            if enable:
                enable_text = "Enabled"
            print("GPS Config: UART2 RTCM" + msg + " Message " + enable_text + "\n")

    def config_uart2_rtcm_msg(self, key_id, enable):
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #keyid
        packet[10] = key_id
        packet[11] = 0x03
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        if enable:
            packet[14] = 0x01 # 0:disable, 1:enable
        else:
            packet[14] = 0x00 # 0:disable, 1:enable
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)
        return packet 

    def set_gx_messages(self):
        all_messages = ['GSV', 'RMC', 'GSA', 'VTG', 'GLL', 'GST','GNS']

        for msg in all_messages:
            if msg == 'GSV':
                key_id = 0xc8
            elif msg == 'RMC':
                key_id = 0xaf
            elif msg == 'GSA':
                key_id = 0xc3
            elif msg == 'VTG':
                key_id = 0xb4
            elif msg == 'GLL':
                key_id = 0xcd
            elif msg == 'GST':
                key_id = 0xd7
            elif msg == 'GNS':
                key_id = 0xb9

            enable = msg in self.desired_messages

            packet = self.config_gx_message(key_id, enable)
            self.spiport.writebytes(packet)
        
            received_bytes = self.receive_ubx_bytes_from_spi()
            self.check_ubx_uart(received_bytes)

            enable_text = "Disabled"
            if enable:
                enable_text = "Enabled"
            print("GPS Config: " + msg + " Message " + enable_text + "\n")

    def config_gx_message(self, key_id, enable):
        # prepare packet
        length = 18
        packet = self.prepare_cfg_packet(length)

        #key id
        packet[10] = key_id
        packet[11] = 0x00
        packet[12] = 0x91
        packet[13] = 0x20
        #value
        if enable:
            packet[14] = 0x01 # 0:disable, 1:enable
        else:
            packet[14] = 0x00 # 0:disable, 1:enable
        packet[15] = 0x00
        packet = self.calculate_checksum(packet, length)

        return packet

    def config_f9p(self):      

        # configure the measurement rate of the chip
        
        ubx_rate_meas = self.cfg_rate_meas()
        self.spiport.writebytes(ubx_rate_meas)
        
        received_bytes = self.receive_ubx_bytes_from_spi()
        self.check_ubx_uart(received_bytes)            
        print("Configured the measurement rate as " + str(self.meas_rate) + " Hz") 

        self.set_gx_messages()

    def config_uart2_rtcm(self):
        ## Configure the F9P chip to be able to output RTCM messages for the dual-GPS setup (moving base)

        ## HPG 1.32
        self.set_rtcm_messages()

        # set uart_2 baud 460800
        packet = self.cfg_valget_uart2_baudrate()
        self.spiport.writebytes(packet)
        print('set uart2 baud 460800')
        


if __name__ == '__main__':
    
    moving_base = False
    desired_messages = ['GST', 'VTG']
    #desired_messages = []
    meas_rate = 8
    if moving_base:
        meas_rate = 5

    spiport = spidev.SpiDev()
    spiport.open(2, 0)  # (2,0) for spi1, (0,0) for spi0
    spiport.max_speed_hz =  7800000 # 1000000, 15600000,62400000....
    spiport.mode = 0
    spiport.no_cs

    f9p_cfg = F9P_config(spiport, desired_messages, meas_rate)         

    #function to get the f9p firmware version
    packet = f9p_cfg.get_ver()
    f9p_cfg.write(packet)
    received_bytes = f9p_cfg.receive_ubx_bytes_from_spi()
    #print("Firmware version of Ublox F9P: ",received_bytes) # To print out the firmware version of F9P if required

    # revert to the default mode
    if moving_base:
        packet = f9p_cfg.revert_to_default_mode()
        f9p_cfg.write(packet)

    #configure the f9p to block unwanted messages
    f9p_cfg.config_f9p()
    
    if moving_base:
        # configure the uart2's output (for movingbase)
        f9p_cfg.config_uart2_rtcm()

    #receive GPS messages
    packet = f9p_cfg.receive_gps()
    f9p_cfg.write(packet)
    received_bytes = f9p_cfg.receive_ubx_bytes_from_spi()     
