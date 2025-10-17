#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Based on:
	Sokrates80/sbus_driver_micropython git hub
	https://os.mbed.com/users/Digixx/code/SBUS-Library_16channel/file/83e415034198/FutabaSBUS/FutabaSBUS.cpp/
	https://os.mbed.com/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
	https://www.ordinoscope.net/index.php/Electronique/Protocoles/SBUS
"""

import asyncio
import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))
# from pyserial_asyncio.serial_asyncio import serial_asyncio
import pyserial_asyncio.serial_asyncio as serial_asyncio

class SBUSReceiver:
    class SBUSFramer(asyncio.Protocol):

        START_BYTE = 0x0f
        END_BYTE = 0x00
        SBUS_FRAME_LEN = 25

        def __init__(self, loop, port, frames, baudrate=100000):
            super().__init__()
            self.loop = loop
            self.port = port
            self.baudrate = baudrate
            self._in_frame = False
            self.transport = None
            self._frame = bytearray()
            self.frames = frames

        def connection_made(self, transport):
            self.transport = transport

        def data_received(self, data):
            #print('data:', data)
            for b in data:
                if self._in_frame:
                    self._frame.append(b)
                    if len(self._frame) == SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN:
                        decoded_frame = SBUSReceiver.SBUSFrame(self._frame)
                        #print(decoded_frame)
                        asyncio.run_coroutine_threadsafe(self.frames.put(decoded_frame), asyncio.get_running_loop())
                        #print('self.frames:', self.frames.qsize())
                        self._in_frame = False
                else:
                    if b == SBUSReceiver.SBUSFramer.START_BYTE:
                        self._in_frame = True
                        self._frame.clear()
                        self._frame.append(b)

        def connection_lost(self, exc):
            print('Connection lost, attempting to reconnect...')
            self.loop.create_task(self.reconnect())

            #asyncio.get_event_loop().stop()

        # new add by whd 20240710
        async def reconnect(self):
            await asyncio.sleep(1)
            try:
                await serial_asyncio.create_serial_connection(
                    self.loop,
                    lambda: SBUSReceiver.SBUSFramer(self.loop, self.port, self.frames, self.baudrate),
                    self.port,
                    baudrate=self.baudrate,
                    parity=serial.PARITY_EVEN,
                    stopbits=serial.STOPBITS_TWO,
                    bytesize=serial.EIGHTBITS
                )
            except Exception as e:
                print(f'Reconnection failed: {e}')
                self.loop.create_task(self.reconnect())

    class SBUSFrame:
        OUT_OF_SYNC_THD = 10
        SBUS_NUM_CHANNELS = 18
        SBUS_SIGNAL_OK = 0
        SBUS_SIGNAL_LOST = 1
        SBUS_SIGNAL_FAILSAFE = 2

        def __init__(self, frame):
            self.sbusChannels = [None] * SBUSReceiver.SBUSFrame.SBUS_NUM_CHANNELS

            channel_sum = int.from_bytes(frame[1:23], byteorder="little")

            for ch in range(0, 16):
                self.sbusChannels[ch] = channel_sum & 0x7ff
                channel_sum = channel_sum >> 11

            # to be tested, No 17 & 18 channel on taranis X8R
            if (frame[23]) & 0x0001:
                self.sbusChannels[16] = 2047
            else:
                self.sbusChannels[16] = 0
            # to be tested, No 17 & 18 channel on taranis X8R
            if ((frame[23]) >> 1) & 0x0001:
                self.sbusChannels[17] = 2047
            else:
                self.sbusChannels[17] = 0

            # Failsafe
            self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_OK
            if (frame[SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN - 2]) & (1 << 2):
                self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_LOST
            if (frame[SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN - 2]) & (1 << 3):
                self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_FAILSAFE

        def get_rx_channels(self):
            """
            Used to retrieve the last SBUS channels values reading
            :return:  an array of 18 unsigned short elements containing 16 standard channel values + 2 digitals (ch 17 and 18)
            """

            return self.sbusChannels

        def get_rx_channel(self, num_ch):
            """
            Used to retrieve the last SBUS channel value reading for a specific channel
            :param: num_ch: the channel which to retrieve the value for
            :return:  a short value containing
            """

            return self.sbusChannels[num_ch]

        def get_failsafe_status(self):
            """
            Used to retrieve the last FAILSAFE status
            :return:  a short value containing
            """

            return self.failSafeStatus

        def __repr__(self):
            return ",".join(str(ch) for ch in self.sbusChannels)

    def __init__(self):
        self._transport = None
        self._protocol = None
        self.frames = asyncio.Queue()

    @staticmethod
    async def create(port='/dev/ttyUSB0'):
        loop = asyncio.get_running_loop()
        receiver = SBUSReceiver()
        receiver._transport, receiver._protocol = await serial_asyncio.create_serial_connection(
            loop,
            lambda: SBUSReceiver.SBUSFramer(loop, port, receiver.frames),
            port,
            baudrate=100000,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS)
        return receiver

    async def get_frame(self):
        #return await self._protocol.frames.get()
        return await self.frames.get()



async def main():
    sbus = await SBUSReceiver.create("/dev/ttyUSB0")

    while True:

        frame = await sbus.get_frame()
        print(frame)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    #loop.run_forever()
    try:
        loop.run_until_complete(main())
    finally:
        loop.close()
