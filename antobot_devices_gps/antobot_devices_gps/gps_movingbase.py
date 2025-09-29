#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description: The purpose of this code is to use two GPS antennas to obtain accurate positioning and heading 
# information. The code retrieves correction messages from a base station and sends them to the u-blox F9P chip inside 
# the uRCU. It then receives correction messages from the u-blox F9P chip inside the uRCU and forwards them to another 
# u-blox F9P chip located externally.
# This script is intended for use with a dual GPS antenna setup, where a base station provides correction messages via MQTT.
    
# This script reports the following on GPS status:
# GPS status : Critical ; GPS status = 0
# GPS status : Warning ; GPS status = 1 - Float
# GPS status : Good ; GPS status = 3 - Fix

# Contact: Huaide Wang
# email: huaide.wang@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import os
import time
import yaml
import asyncio
import serial
import serial_asyncio # sudo pip install pyserial-asyncio
import struct

import traceback
from io import BytesIO

from dataclasses import dataclass
from pyrtcm import RTCMReader
from paho.mqtt import client as mqtt_client

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64
from geometry_msgs.msg import Vector3

from antobot_devices_msgs.msg import GpsHeading

buffer = b""
GPS_Epoch_Time = None

@dataclass
class PVTFrame:
    iTOW: int                 # GPS time of week of the navigation epoch. (ms)
    fixType: int              # 0 = no fix; 1 = dead reckoning only; 2 = 2D-fix; 3 = 3D-fix; 4 = GNSS + dead reckoning combined; 5 = time only fix 
    flag_carrSoln: int        # 0 = no carrier phase range solution; 1 = carrier phase range solution with floating ambiguities; 2 = carrier phase range solution with fixed ambiguities
    lon: float                # Longitude (1e-7 deg)
    lat: float                # Latitude (1e-7 deg)
    height: float             # Height above ellipsoid (mm)
    hAcc: int                 # Horizontal accuracy estimat(mm)
    status_fix: int           # fix type: 0, 1, 3

@dataclass
class RELPOSNEDFrame:
    iTOW: int                 # GPS time of week of the navigation epoch. (ms)
    relPosN: int              # North component of relative position vector
    relPosE: int              # East component of relative position vector
    relPosD: int              # Down component of relative position vector
    relPosLength: int         # Length of the relative position vector  (cm)
    relPosHeading: int        # Heading of the relative position vector (1e-5 deg)
    accLength: int            # Accuracy of length of the relative position vector (0.1 mm)
    accHeading: int           # Accuracy of heading of the relative position vector (1e-5 deg)
    flag_gnssFixOK: bool      # A valid fix (i.e within DOP & accuracy masks)
    flag_diffSoln: bool       # 1 if differential corrections were applied
    flag_relPosValid: bool    # 1 if relative position components and accuracies are valid and, in moving base mode only, if baseline is valid
    flag_carrSoln: int        # 0 = no carrier phase range solution; 1 = carrier phase range solution with floating; 2 = carrier phase range solution with fixed ambiguities
    flag_isMoving: bool       # 1 if the receiver is operating in moving base mode
    relPosHeadingValid: bool  # 1 if relPosHeading is valid
    relPosNormalized: bool    # 1 if the components of the relative position vector (including the high-precision parts) are normalized


# Shared time
class SharedTime:
    """Thread-safe  container for MQTT message timing."""
    
    def __init__(self):
        self._time = None
        self._lock = asyncio.Lock()
    
    async def set_time(self, time_value):
        async with self._lock:
            self._time = time_value
    
    async def get_time(self):
        async with self._lock:
            return self._time
    
    @property
    def time(self):
        return self._time
    
    @time.setter
    def time(self, value):
        self._time = value

# read rtcm message from base and write it to rover
class RTCMFramer(asyncio.Protocol):
    def __init__(self, movebase, port, rtcm_buff=None):
        super().__init__()

        self.movebase = movebase
        self.port = port

        self.transport = None
        self.transport_rover = None
        self.buffer = rtcm_buff


    def connection_made(self, transport):
        self.transport = transport

        # Tips: if want to read from UART, must write first.
        self.transport.write(b'0')

    def data_received(self, data):
        if self.transport_rover:
            self.transport_rover.write(data)
	
        if self.buffer:
            self.buffer.add_data(data)
            #print(f"RTCMFramer received data: {data}")
            
    def connection_lost(self, exc):
        #logger.error('Moving Base: Connection lost, attempting to reconnect...')
        asyncio.get_event_loop().create_task(self.reconnect())
        #asyncio.get_event_loop().stop()

    async def reconnect(self):
        await asyncio.sleep(1)
        try:
            self.movebase._transport_base, self.movebase._protocol_base = await serial_asyncio.create_serial_connection(
                asyncio.get_running_loop(),
                lambda: self,
                    self.port,
                    baudrate = 460800
                )
                
            if self.movebase._protocol_base:
                self.movebase._protocol_base.transport_rover = self.movebase._transport_rover
                    
                self.movebase.MyMQTT.set_transport(self.movebase._transport_base)
                #logger.info(f'Moving Base: Reconnection succeeded')
                
        except Exception as e:
            #logger.error(f'Moving Base: Reconnection failed: {e}')
            asyncio.get_event_loop().create_task(self.reconnect())


# read RELPOSNED message from rover and parse it to get heading
class RELPOSNEDFramer(asyncio.Protocol):

    def __init__(self, movebase, port, max_size=65536):
        super().__init__()

        self.movebase = movebase
        self.port = port
        self.transport = None

        self.SYNC1_BYTE = 0xb5
        self.SYNC2_BYTE = 0x62

        self.buffer = bytearray()
        self.max_size = max_size

        self.frames = asyncio.Queue()

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data):
        self.buffer.extend(data)
        #print(f"RELPOSNEDFramer: {len(self.buffer)}")
        if len(self.buffer) > self.max_size:
            del self.buffer[: self.max_size // 2]
        self.consume_data()
	
    def consume_data(self):
        #mv = memoryview(self.buffer)
        i = 0
        n = len(self.buffer)

        while True:
            j = self.buffer.find(self.SYNC1_BYTE, i)
            if j < 0 or j + 2 > n:
                break
            if self.buffer[j+1] != self.SYNC2_BYTE:
                i = j + 1
                continue

            if j + 6 > n:
                break

            length = self.buffer[j+4] | (self.buffer[j+5] << 8)
            total = length + 8 

            if j + total > n:
                break 

            #frame = mv[j : j + total]
            frame = bytes(self.buffer[j : j + total])
            #print(f"frame: {frame}")
            cls_id_len_payload = frame[2 : 6 + length]
            ck_a = 0
            ck_b = 0
            for b in cls_id_len_payload:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            if ck_a != frame[6 + length] or ck_b != frame[6 + length + 1]:
                i = j + 1
                continue

            # only parse NAV-RELPOSNED (class=0x01, id=0x3C)
            if frame[2] == 0x01 and frame[3] == 0x3C:
                payload = frame[6 : 6 + length]
                #print(f"RELPOSNEDFramer - payload: {payload}")
                
                relposned = self.parse_nav_relposned(payload)
                try:
                    self.frames.put_nowait(relposned)
                except asyncio.QueueFull:
                    _ = self.frames.get_nowait()
                    self.frames.task_done()
                    self.frames.put_nowait(relposned)
                
            i = j + total

        if i > 0:
            del self.buffer[:i]

    @staticmethod
    def parse_nav_relposned(payload: bytes):
        #print(payload)
        iTOW         = struct.unpack_from('<i', payload, 0x04)[0] # ms
        
        relPosN      = struct.unpack_from('<i', payload, 0x08)[0]
        relPosE      = struct.unpack_from('<i', payload, 0x0C)[0]
        relPosD      = struct.unpack_from('<i', payload, 0x10)[0]
        
        relPosLength = struct.unpack_from('<i', payload, 0x14)[0] # cm
        relPosHeading= struct.unpack_from('<i', payload, 0x18)[0] # deg, 1e-5
        
        accLength    = struct.unpack_from('<I', payload, 0x30)[0] # mm
        accHeading   = struct.unpack_from('<I', payload, 0x34)[0] # deg, 1e-5
        
        flags        = struct.unpack_from('<I', payload, 0x3C)[0]
        #print(f"relPosLength: {relPosLength}")
        return RELPOSNEDFrame(
            iTOW, relPosN, relPosE, relPosD, relPosLength, relPosHeading,
            accLength, accHeading,
            bool(flags & 0x01), bool(flags & 0x02), bool(flags & 0x04),
            (flags >> 3) & 0x03, bool(flags & 0x20),
            bool(flags & 0x100), bool(flags & 0x200)
        )

    def connection_lost(self, exc):
        #logger.error('Moving Rover: Connection lost, attempting to reconnect...')
        asyncio.get_event_loop().create_task(self.reconnect())
        #asyncio.get_event_loop().stop()

    async def reconnect(self):
        await asyncio.sleep(1)
        try:
            self.movebase._transport_rover, self.movebase._protocol_rover = await serial_asyncio.create_serial_connection(
                asyncio.get_running_loop(),
                lambda: self,
                self.port,
                baudrate=460800
                )
                
            if self.movebase._protocol_base:
                self.movebase._protocol_base.transport_rover = self.movebase._transport_rover
                #logger.info(f'Moving Rover: Reconnection succeeded')
            
        except Exception as e:
            #logger.error(f'Moving Rover: Reconnection failed: {e}')
            asyncio.get_event_loop().create_task(self.reconnect())


# get iTow of RTCM 1077 message
class RTCMBuffer:  

    def __init__(self, max_size: int = 8192):
        self.buffer = bytearray()
        self.itow_1077 = None  # iTOW from 1077 message for comparison
        self.max_size = max_size
        self._lock = asyncio.Lock()
        
        self._data_event = asyncio.Event()
        self._closed = False
    	
        self._i = 0           
        self._need_len = 0  

        self.SYNC_BYTE = 0xD3
        self.FRAME_BUDGET = 64
        self.TIME_BUDGET_NS = 2_000_000 # 2ms

    	    	
    def add_data(self, data: bytes) -> None:

        self.buffer.extend(data)
        #print(f"RTCMBuffer: {len(self.buffer)}")
        if len(self.buffer) > self.max_size:
            self.buffer = self.buffer[-self.max_size//2:]
        
        self._data_event.set()
    
    async def run(self):
    
        while not self._closed:

            await self._data_event.wait()
            self._data_event.clear()

            start_ns = time.monotonic_ns()
            frames_done = 0

            while True:
                async with self._lock:
                    made_progress = self.consume_data(start_ns)

                if not made_progress:
                    break

                frames_done += made_progress
                if frames_done >= 64 or (time.monotonic_ns() - start_ns) >= 2_000_000:
                    await asyncio.sleep(0)
                    frames_done = 0
                    start_ns = time.monotonic_ns()

    async def close(self):
        self._closed = True
        self._data_event.set()

    def get_itow_1077(self):
        return self.itow_1077
            
    def consume_data(self, start_ns):

        buf = self.buffer
        n = len(buf)
        i = self._i
        frames = 0
        progressed = False

        #mv = memoryview(buf)

        while True:
            if (time.monotonic_ns() - start_ns) >= self.TIME_BUDGET_NS:
                break

            if self._need_len == 0: # start to look for 0xD3
                j = buf.find(self.SYNC_BYTE, i)
                if j < 0: # do not  find 0xD3, wait for more data
                    if i > 0:
                        del buf[:i]
                    else:
                        buf.clear()

                    self._i = 0
                    self._need_len = 0
                    progressed = True
                    break
  
                if j != i:
                    progressed = True

                i = j
                if i + 3 > n:
                    break 

                length_10 = ((buf[i+1] & 0x03) << 8) | buf[i+2]
                if length_10 == 0 or length_10 > 1023:
                    i += 1
                    progressed = True
                    continue

                self._need_len = length_10 + 6
                progressed = True

            if i + self._need_len > n:
                break

            start = i
            end = i + self._need_len
            #payload = mv[start + 3 : end - 3]
            payload = bytes(buf[start + 3 : end - 3])

            # read head：DF002(12) DF003(12) DF004(30)
            msgnum = self.read_bits(payload, 0, 12)
            #print(f"msgnum: {msgnum}")
            if msgnum == 1074:
                tow_ms = self.read_bits(payload, 24, 30)
                self.itow_1077 = tow_ms
                #print(f" tow_ms: { tow_ms}")

            i = end
            self._need_len = 0
            frames += 1
            progressed = True

            if frames >= self.FRAME_BUDGET:
                break

        if i > 0:
            del buf[:i]
            self._i = 0
        else:
            self._i = i

        return frames if progressed else 0          
      
    def read_bits(self, mv, bitpos, nbits):
        v = 0
        for k in range(nbits):
            b = (bitpos + k) // 8
            r = 7 - ((bitpos + k) % 8)
            v = (v << 1) | ((mv[b] >> r) & 1)
        return v
            
# mqtt class: receive the rtcm message from fixed base and write it to base
class ACMQTT:
    def __init__(self, transport, mode, time):
        
        self.transport = transport
        self.mode = mode
        self.msg_rec_time = time
        if self.mode == 1:
            #logger.info(f'nRTK Mode: base station')     
            parent_directory = os.path.dirname(os.path.abspath(__file__))
            yaml_file_path = os.path.join(parent_directory, "../config/corrections_config.yaml")
            
            with open(yaml_file_path, 'r') as file:
                config = yaml.safe_load(file)
                self.client_id = 'Anto_MQTT_F9P_Sub_' + config['mqtt']['device_ID']
                self.topics_sub = "AntoCom/02/" + config['mqtt']['baseStation_ID'] + "/00"
                self.broker = config['mqtt']['mqtt_Broker']
                self.port = config['mqtt']['mqtt_Port']
                self.keepalive = config['mqtt']['mqtt_keepalive']
                self.mqtt_username = config['mqtt']['mqtt_UserName']
                self.mqtt_password = config['mqtt']['mqtt_PassWord']

            self.client = mqtt_client.Client(self.client_id)
            #self.client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, self.client_id) # for paho-mqtt 2.0

            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

        elif self.mode == 2:     
            #logger.info(f'nRTK Mode: ppp')
            parent_directory = os.path.dirname(os.path.abspath(__file__))
            yaml_file_path = os.path.join(parent_directory, "../../config/ppp_config.yaml")

            with open(yaml_file_path, 'r') as file:
                    config = yaml.safe_load(file)
                    client_id = config['device_ID']

            self.client_id = client_id 
            self.broker = 'pp.services.u-blox.com'
            self.port=8883
            self.topics_sub = [(f"/pp/ip/eu", 0), ("/pp/ubx/mga", 0), ("/pp/ubx/0236/ip", 0)]
            self.client = mqtt_client.Client(client_id=self.client_id)
            self.client.tls_set(certfile=os.path.join(parent_directory,"../../config/")+f'device-{self.client_id}-pp-cert.crt',keyfile=os.path.join(parent_directory,"../../config/")+f'device-{self.client_id}-pp-key.pem')
                        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.connect_broker()

    def connect_broker(self):
        try:
            if self.mode == 1:
                self.client.connect(self.broker, self.port, self.keepalive)
            elif self.mode == 2:
                self.client.connect(self.broker, self.port)

        except Exception as e:
            #logger.error(f"MQTT: Connection failed: {e}")
            pass
        self.client.loop_start() 

    def disconnect_broker(self):
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            #logger.info(f"MQTT: Connected to MQTT Broker successfully!")
            print(self.topics_sub)
            self.client.subscribe(self.topics_sub)
        else:
            #logger.error("MQTT: Failed to connect, return code {0}".format(rc))
            pass

    def publish_message(self, msg):
        result = self.client.publish(topic=self.topic, payload=msg, qos=0, retain=True)

        if result[0] == 0:
            return True
        else:
            #logger.error("MQTT: Failed to send message {0} to topic {1}".format(msg, self.topic))
            return False
    
    def on_message(self, client, userdata, msg):
        self.msg_rec_time.time  = time.time()
        #print(f"mqtt_time: {self.msg_rec_time.time }")
        # print(f"topic: {msg.topic}; msg: {msg.payload}")
        self.transport.write(msg.payload)

    def set_transport(self, transport_):
        self.transport = transport_


class ROS2Interface(Node):
    def __init__(self):
        super().__init__('ROS2')
        
        # publish
        self.pub_heading = self.create_publisher(GpsHeading, '/antobot_gps/heading', 10)
        
        self.pub_heading_urcu = self.create_publisher(Float64, '/antobot_gps/heading/urcu', 10) # the heading got form dual GPS directly
        self.pub_heading_robot = self.create_publisher(Float64, '/antobot_gps/heading/robot', 10) # the heading got form dual GPS directly
        self.pub_relposned = self.create_publisher(Vector3, '/antobot_gps/relposned', 10) # the heading got form dual GPS directly
        
        
        self.pub_delay = self.create_publisher(Int32, '/antobot_gps/delay', 10) # the delay between Relpos message and RTCM message

        # subscribe
        

    def costmapCallBack(self, msg):
        print(msg)



class MovingBase:

    def __init__(self, port1 = 'port_movingbase', port2 = 'port_movingrover', mode = 1, msg_rec_mqtt_time = None, rtcm_buffer = None, ros_node = None):
        
        # port
        self.port1 = port1
        self.port2 = port2

        # correction mode
        self.mode = mode
        self.mqtt_client = None

        # Transport and protocol objects
        self._transport_base = None
        self._protocol_base = None

        self._transport_rover = None
        self._protocol_rover = None
    
        # Performance monitoring
        self.msg_rec_mqtt_time = msg_rec_mqtt_time
        self.time_limit = 0.25
        
        self.rtcm_buffer = rtcm_buffer
        self.ros_node = ros_node
    
    async def initialize(self):
        try:
            # Setup serial connections
            await self._setup_serial_connections(self.rtcm_buffer)
            
            # Setup MQTT client
            self.mqtt_client = ACMQTT(self._transport_base, self.mode, self.msg_rec_mqtt_time)
            
            print("MovingBase initialization completed successfully")
            return True
            
        except Exception as e:
            print(e)
            traceback.print_exc()

    async def _setup_serial_connections(self, rtcm_buff) -> None:
        # movingbase: receive the RTCM of the basestation and output itself RTCM;
        # movingrovel: receive the RTCM of the movingbase and output heading;

        try:
            # Setup base station connection
            self._transport_base, self._protocol_base = await serial_asyncio.create_serial_connection(
                asyncio.get_running_loop(),
                lambda: RTCMFramer(self, self.port1, rtcm_buff),
                self.port1,
                baudrate=460800
            )
            print(f"Base station connected on {self.port1}")
            
            # Setup rover connection
            self._transport_rover, self._protocol_rover = await serial_asyncio.create_serial_connection(
                asyncio.get_running_loop(),
                lambda: RELPOSNEDFramer(self, self.port2),
                self.port2,
                baudrate=460800
            )
            print(f"Rover connected on {self.port2}")
            
            # Link transports for data forwarding
            if self._protocol_base and self._transport_rover:
                self._protocol_base.transport_rover = self._transport_rover
                
        except Exception as e:
            print(f"Failed to setup serial connections: {e}")
            raise


    async def get_relposned(self):

        # self._protocol_rover is an asyncio serial connection (defined in line 363)
        if self._protocol_rover:
            try:
                result = await asyncio.wait_for(self._protocol_rover.frames.get(), timeout=self.time_limit)
                return result
            except Exception as e:
                ##logger.error(f"Timeout: get_RELPOSNEDframe ({e})")
                return None
   
    
    # for test
    def close(self):
        if self._transport_base.is_closing():
            pass
           #logger.error(f"transport_base is closing")
        else:
           self._transport_base.close()
           #logger.error(f"close transport_base") 

        if self._transport_rover.is_closing():
            pass
           #logger.error(f"transport_rover is closing")
        else:
           self._transport_rover.close() 
           #logger.error(f"close transport_rover")

    async def run(self):
        try:           
            print("MovingBase started successfully")
            
            # Keep the event loop running
            while True:
                try:
                    # Process frames and handle any periodic tasks
                    frame = await self.get_relposned()
                    #print(frame)
                    
                    if frame:
                        # Process frame if needed
                        itow = self.rtcm_buffer.get_itow_1077()
                        #print(f"itow: {itow}")
                        #print(f"mqtt: {self.msg_rec_mqtt_time.time }; gps: {itow}; heading: {frame.iTOW}; diff: {itow-frame.iTOW}ms")
                    	
                        msg_heading = GpsHeading()
                        msg_heading.heading = frame.relPosHeading * 1e-5
                        msg_heading.length = frame.relPosLength * 1e-2
                        msg_heading.rel_pos_n = frame.relPosN * 1e-2
                        msg_heading.rel_pos_e = frame.relPosE * 1e-2
                        msg_heading.rel_pos_d = frame.relPosD * 1e-2
                        msg_heading.acc_heading = frame.accHeading * 1e-5
                        msg_heading.gnss_fix_ok = frame.flag_gnssFixOK
                        msg_heading.rel_pos_valid = frame.flag_relPosValid
                        msg_heading.carr_soln = frame.flag_diffSoln
                        msg_heading.is_moving = frame.flag_isMoving
                        msg_heading.heading_valid = frame.relPosHeadingValid
                        
    
                        self.ros_node.pub_heading.publish(msg_heading)
                        
                        
                        delay = Int32()
                        delay.data = itow-frame.iTOW
                        self.ros_node.pub_delay.publish(delay)
                        
                    # # Check connection health
                    # await self._check_connection_health()
                    #print(f"mqtt: {self.msg_rec_mqtt_time.time }; gps: {}; heading: {frame.}")
                    await asyncio.sleep(0.2)
                except asyncio.TimeoutError:
                    # Normal timeout, continue
                    continue
                except Exception as e:
                    print(f"Error in main loop: {e}")
                    await asyncio.sleep(1)  # Brief pause before continuing
                    
        except KeyboardInterrupt:
            print("Received interrupt signal, shutting down...")
        except Exception as e:
            print(f"Fatal error in run loop: {e}")
        finally:
            pass

async def spin_ros(node, period=0.005):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(period)
        
async def main():
    
    F9P1_UART = "/dev/ttyTHS1" # The f9p is in the uRCU
    F9P2_UART = "/dev/ttyCH341USB0" # The f9p is out of the uRCU
    # movebase = await MovingBase.create(F9P1_UART, F9P2_UART)
    
    rclpy.init()
    ros_node = ROS2Interface()

    msg_rec_mqtt_time = SharedTime()
    rtcm_buffer = RTCMBuffer()
    try:
        moving_base = MovingBase(F9P1_UART, F9P2_UART, 1, msg_rec_mqtt_time, rtcm_buffer, ros_node)

        if not await moving_base.initialize():
            print("Failed to initialize MovingBase")
            return
        
        # Create tasks
        tasks = [moving_base.run(), rtcm_buffer.run(), spin_ros(ros_node)]
        # tasks = [moving_base.run(), spin_ros(ros_node)]

        # Run main loop
        await asyncio.gather(*tasks)

    except Exception as e:
        print(e)
        traceback.print_exc()
    finally:
        # clean up
        ros_node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Application terminated by user")
    except Exception as e:
        print(f"Fatal error: {e}")
