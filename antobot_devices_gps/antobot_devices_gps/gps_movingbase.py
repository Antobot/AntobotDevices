#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description: The purpose of this code is to use two GPS antennas to obtain accurate positioning and heading 
# information. The code retrieves correction messages from a base station and sends them to the u-blox F9P chip inside 
# the uRCU. It then receives correction messages from the u-blox F9P chip inside the uRCU and forwards them to another 
# u-blox F9P chip located externally.
# This script is intended for use with a dual GPS antenna setup, where a base station provides correction messages via MQTT.

# Contact: Huaide Wang
# email: huaide.wang@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import time
import asyncio
import serial_asyncio
import struct
import traceback
import queue

from dataclasses import dataclass
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

from antobot_devices_msgs.msg import GpsHeading
from mavros_msgs.msg import RTCM

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
    def __init__(self):
        self._time = 0
        self._time_queue = queue.Queue(maxsize=5)

    @property
    def time(self):
        return self._time
    
    @time.setter
    def time(self, value):
        self._time = value

    def add_time_queue(self, value):
        if self._time_queue.full():
            _ = self._time_queue.get()
        self._time_queue.put(value)
    
    def get_latest_time_queue(self):
        if self._time_queue.empty():
            return None
        return list(self._time_queue.queue)[-1]

    def get_earliest_time_queue(self):
        if not self._time_queue.full():
            return None
        return list(self._time_queue.queue)[0]
    
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
        # 20250939: Comment this out, because URCU 1.5 doesn't need it.
        # self.transport.write(b'0')

    def data_received(self, data):
        if self.transport_rover and not self.transport_rover.is_closing():
            try:
                self.transport_rover.write(data)
            except Exception as e:
                pass
        if self.buffer:
            self.buffer.add_data(data)
            #print(f"RTCMFramer received data: {data}")
            
    def connection_lost(self, exc):
        print(f'Moving Base: Connection lost, attempting to reconnect... {exc}')
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
                    
                self.movebase.ros_node.set_rtcm_transport(self.movebase._transport_base)

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

        self.frames = asyncio.Queue(maxsize=1)

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
                    #print(f"frame size: {self.frames.qsize()}")
                except asyncio.QueueFull:
                    _ = self.frames.get_nowait()
                    self.frames.put_nowait(relposned)
                
            i = j + total

        if i > 0:
            del self.buffer[:i]

    @staticmethod
    def parse_nav_relposned(payload: bytes):
        #print(payload)
        iTOW = struct.unpack_from('<I', payload, 0x04)[0] # ms
        
        relPosN = struct.unpack_from('<i', payload, 0x08)[0]
        relPosE = struct.unpack_from('<i', payload, 0x0C)[0]
        relPosD = struct.unpack_from('<i', payload, 0x10)[0]
        
        relPosLength = struct.unpack_from('<i', payload, 0x14)[0] # cm
        relPosHeading = struct.unpack_from('<i', payload, 0x18)[0] # deg, 1e-5
        
        accLength = struct.unpack_from('<I', payload, 0x30)[0] # mm
        accHeading = struct.unpack_from('<I', payload, 0x34)[0] # deg, 1e-5
        
        flags = struct.unpack_from('<I', payload, 0x3C)[0]

        return RELPOSNEDFrame(
            iTOW, relPosN, relPosE, relPosD, relPosLength, relPosHeading,
            accLength, accHeading,
            bool(flags & 0x01), bool(flags & 0x02), bool(flags & 0x04),
            (flags >> 3) & 0x03, bool(flags & 0x20),
            bool(flags & 0x100), bool(flags & 0x200)
        )

    def connection_lost(self, exc):
        print(f'Moving Rover: Connection lost, attempting to reconnect... {exc}')
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

    	    	
    def add_data(self, data: bytes):

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
            #print(f"msgnum: {msgnum}; payload: {payload}")
            if msgnum == 1077:
                itow_ms = self.read_bits(payload, 24, 30)
                self.itow_1077 = itow_ms
                #print(f"itow_1077: {itow_ms}")
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


class ROS2Interface(Node):
    def __init__(self):
        super().__init__('gps_movingbase')
        
        self.declare_parameter('rtcm_topic', '/antobot_gps/rtcm')
        self.declare_parameter('port_movingbase', '/dev/ttyTHS1')
        self.declare_parameter('port_movingrover', '/dev/anto_gps')
        self.declare_parameter('antenna_baseline', 1.3)
        self.antenna_baseline = self.get_parameter('antenna_baseline').get_parameter_value().double_value
        
        self.loop = asyncio.get_running_loop()
        self.rtcm_last_time = 0
        self.msg_rec_heading_time = None


        self.timer_period_heading = 1
        
        # heading status
        self.heading_status = 0 # 0: No data; 1: frequency < 1; 2: frequency: 1 ~ 3; 3: requency > 3
        self.heading_status_last = -1

        # publish
        self.pub_heading = self.create_publisher(GpsHeading, '/antobot_gps/heading', 10)
        
        # timer
        self.timer_heading = self.create_timer(self.timer_period_heading, self.frequency_heading)

        # RTCM subscriber and transport management
        self.rtcm_transport = None
        rtcm_topic = self.get_parameter('rtcm_topic').get_parameter_value().string_value
        self.rtcm_subscription = self.create_subscription(RTCM, rtcm_topic, self.on_rtcm_message, 10)

    def frequency_heading(self):
        if self.msg_rec_heading_time:
            time_earliest = self.msg_rec_heading_time.get_earliest_time_queue()
            if time_earliest:
                fre = 5 / (time.time() - time_earliest)
                if fre < 1 and self.heading_status != 1:
                    self.heading_status = 1
                    self.get_logger().error("SN4112: Heading Frequency status: Critical (<1 hz)")
                    
                elif fre > 1 and fre <= 3 and self.heading_status != 2:
                    self.heading_status = 2
                    self.get_logger().warn("SN4112: Heading Frequency status: warning (1 ~ 3 hz)")

                elif fre > 3 and self.heading_status != 3:
                    self.heading_status = 3
                    self.get_logger().info("SN4112: Heading Frequency status: good (>3 hz)")

            elif self.heading_status_last != self.heading_status:
                self.heading_status = 0
                self.get_logger().error("SN4100: No Heading message received")


        elif self.heading_status_last != self.heading_status:
            self.heading_status = 0
            self.get_logger().error("SN4100: No Heading message received")

        if self.heading_status_last != self.heading_status:
            self.heading_status_last = self.heading_status
          
    def set_heading_time(self, msg_rec_heading_time):
        self.msg_rec_heading_time = msg_rec_heading_time

    def on_rtcm_message(self, msg: RTCM):
        self.rtcm_last_time = time.time()
        if self.rtcm_transport and not self.rtcm_transport.is_closing():
            try:
                payload = bytes(msg.data)
                self.loop.call_soon_threadsafe(self.rtcm_transport.write, payload)
            except Exception:
                pass

    def set_rtcm_transport(self, transport_):
        self.rtcm_transport = transport_

class MovingBase:

    def __init__(self, port1 = 'port_movingbase', port2 = 'port_movingrover', rtcm_buffer = None, ros_node = None):
        
        # port
        self.port1 = port1
        self.port2 = port2

        self.rtcm_subscriber = None

        # Transport and protocol objects
        self._transport_base = None
        self._protocol_base = None

        self._transport_rover = None
        self._protocol_rover = None
    
        # Performance monitoring
        self.msg_rec_heading_time = SharedTime()
        self.time_limit = 0.25
        
        self.rtcm_buffer = rtcm_buffer
        self.ros_node = ros_node
    
    async def initialize(self):
        try:
            # Setup serial connections
            await self._setup_serial_connections(self.rtcm_buffer)
            
            # Setup ROS RTCM transport
            self.ros_node.set_rtcm_transport(self._transport_base)

            # Setup ROS
            self.ros_node.set_heading_time(self.msg_rec_heading_time)


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
        if self._transport_base and not self._transport_base.is_closing():
            self._transport_base.close()
        if self._transport_rover and not self._transport_rover.is_closing():
            self._transport_rover.close()

    async def run(self):
        try:           
            print("MovingBase started successfully")
            msg_heading = GpsHeading()
            msg_heading.header= Header()
            msg_heading.header.frame_id = 'gps_frame'  # FRAME_ID
            
            heading_valid = True
            heading_valid_last = False
            
            # Keep the event loop running
            while True:
                try:
                    # Process frames and handle any periodic tasks
                    frame = await self.get_relposned()
                    #print(frame)
                    
                    if frame:
                        self.msg_rec_heading_time.add_time_queue(time.time())

                        msg_heading.header.stamp = self.ros_node.get_clock().now().to_msg() 
                        #msg_heading.heading = (450 - frame.relPosHeading * 1e-5) % 360
                        msg_heading.heading = (630 - frame.relPosHeading * 1e-5) % 360
                        msg_heading.length = frame.relPosLength * 1e-2
                        msg_heading.rel_pos_n = frame.relPosN * 1e-2
                        msg_heading.rel_pos_e = frame.relPosE * 1e-2
                        msg_heading.rel_pos_d = frame.relPosD * 1e-2
                        msg_heading.acc_heading = frame.accHeading * 1e-5
                        msg_heading.acc_length = frame.accLength * 1e-4
                        msg_heading.gnss_fix_ok = frame.flag_gnssFixOK
                        msg_heading.rel_pos_valid = frame.flag_relPosValid
                        msg_heading.carr_soln = frame.flag_carrSoln
                        msg_heading.is_moving = frame.flag_isMoving
                        # msg_heading.heading_valid = frame.relPosHeadingValid
                        
                        
                        rtcm_itow = self.rtcm_buffer.get_itow_1077()
                        if rtcm_itow:
                            time_diff = rtcm_itow - frame.iTOW
                            if time_diff < 0:
                                time_diff += 200
                            elif time_diff > 999:
                                time_diff = 999
                            msg_heading.time_diff = time_diff
                        else:
                            msg_heading.time_diff = 999 # 999 means no message received


                        heading_valid_ = (frame.flag_gnssFixOK and frame.flag_isMoving and frame.relPosHeadingValid and (abs(frame.relPosLength * 1e-2 - self.ros_node.antenna_baseline) < 0.1 ) and (frame.flag_carrSoln == 2))
                        print(f"heading_valid_: {heading_valid_}; flag_gnssFixOK:{frame.flag_gnssFixOK}; flag_isMoving: {frame.flag_isMoving}; relPosHeadingValid: {frame.relPosHeadingValid};  \
                              flag_carrSoln: {frame.flag_carrSoln == 2}; relPosLength: {frame.relPosLength * 1e-2}; {abs(frame.relPosLength * 1e-2 - self.ros_node.antenna_baseline) < 0.1}  ")
                        
                        if heading_valid != heading_valid_:
                            if heading_valid_:
                                self.ros_node.get_logger().info("SN4110: Heading message status: valid")
                            else:
                                self.ros_node.get_logger().error("SN4110: Heading message status: invalid")
                            heading_valid = heading_valid_
    			    
                        msg_heading.heading_valid = heading_valid_
                        self.ros_node.pub_heading.publish(msg_heading)
                        
                    elif heading_valid and heading_valid != heading_valid_last:
                        heading_valid = heading_valid_last
                        self.ros_node.get_logger().error("SN4110: Heading message status: invalid")
                        
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
        
async def async_main():
    
    rclpy.init()
    ros_node = ROS2Interface()
    
    F9P1_UART = ros_node.get_parameter('port_movingbase').get_parameter_value().string_value
    F9P2_UART = ros_node.get_parameter('port_movingrover').get_parameter_value().string_value

    rtcm_buffer = RTCMBuffer()
    try:
        moving_base = MovingBase(F9P1_UART, F9P2_UART, rtcm_buffer, ros_node)

        if not await moving_base.initialize():
            print("Failed to initialize MovingBase")
            return
        
        # Create tasks
        tasks = [moving_base.run(), rtcm_buffer.run(), spin_ros(ros_node)]

        # Run main loop
        await asyncio.gather(*tasks)

    except Exception as e:
        print(e)
        traceback.print_exc()
    finally:
        # clean up
        ros_node.destroy_node()
        rclpy.shutdown()
        
def main():
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        print("Application terminated by user")
    except Exception as e:
        print(f"Fatal error: {e}")
        
        
if __name__ == '__main__':
    main()
