import serial
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from builtin_interfaces.msg import Time
from datetime import datetime
from antobot_devices_msgs.msg import RTCM, GpsQual, GpsHeading


def nmea_crc(nmea_sentence):
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        crc = crc[:2]
    except Exception:
        return False

    calculated_checksum = 0
    for char in sentence:
        calculated_checksum ^= ord(char)

    calculated_checksum_hex = format(calculated_checksum, "X")
    return calculated_checksum_hex.zfill(2) == crc.upper()


def crc32_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table


CRC32_TABLE = crc32_table()


def nmea_expend_crc(sentence):
    try:
        body, crc = sentence[1:].split("*")
        crc = crc[:8]
    except Exception:
        return False

    crc_val = 0
    for byte in body.encode():
        crc_val = CRC32_TABLE[(crc_val ^ byte) & 0xFF] ^ (crc_val >> 8)
    crc_val &= 0xFFFFFFFF

    return format(crc_val, "08x") == crc.lower()


def parse_gga_latlon_to_decimal(lat_str, lat_dir, lon_str, lon_dir):
    """
    lat: ddmm.mmmmm
    lon: dddmm.mmmmm
    -> decimal degrees
    """
    if not lat_str or not lon_str:
        return None, None

    lat_deg = int(lat_str[:2])
    lat_min = float(lat_str[2:])
    lat = lat_deg + lat_min / 60.0
    if lat_dir == "S":
        lat = -lat

    lon_deg = int(lon_str[:3])
    lon_min = float(lon_str[3:])
    lon = lon_deg + lon_min / 60.0
    if lon_dir == "W":
        lon = -lon

    return lat, lon


def parse_gga(line):
    """
    Example:
    $GNGGA,023329.00,3130.32859803,N,12030.26166344,E,4,34,0.6,5.8160,M,7.9690,M,1.0,328*5B
    """
    body = line[1:line.find("*")]
    parts = body.split(",")

    data = {
        "utc": parts[1] if len(parts) > 1 else "",
        "lat_raw": parts[2] if len(parts) > 2 else "",
        "lat_dir": parts[3] if len(parts) > 3 else "",
        "lon_raw": parts[4] if len(parts) > 4 else "",
        "lon_dir": parts[5] if len(parts) > 5 else "",
        "qual": int(parts[6]) if len(parts) > 6 and parts[6] else 0,
        "num_sats": int(parts[7]) if len(parts) > 7 and parts[7] else 0,
        "hdop": float(parts[8]) if len(parts) > 8 and parts[8] else 0.0,
        "alt": float(parts[9]) if len(parts) > 9 and parts[9] else 0.0,
        "undulation": float(parts[11]) if len(parts) > 11 and parts[11] else 0.0,
        "age": float(parts[13]) if len(parts) > 13 and parts[13] else 0.0,
        "station_id": parts[14] if len(parts) > 14 else "",
    }

    lat, lon = parse_gga_latlon_to_decimal(
        data["lat_raw"], data["lat_dir"], data["lon_raw"], data["lon_dir"]
    )
    data["lat"] = lat
    data["lon"] = lon
    return data


def parse_gst(line):
    """
    Example:
    $GNGST,054013.00,0.67,1.67,1.37,115.3800,1.432,1.620,3.399*41
    """
    body = line[1:line.find("*")]
    parts = body.split(",")

    return {
        "utc": parts[1] if len(parts) > 1 else "",
        "rms": float(parts[2]) if len(parts) > 2 and parts[2] else 0.0,
        "smjr_std": float(parts[3]) if len(parts) > 3 and parts[3] else 0.0,
        "smnr_std": float(parts[4]) if len(parts) > 4 and parts[4] else 0.0,
        "orient": float(parts[5]) if len(parts) > 5 and parts[5] else 0.0,
        "lat_std": float(parts[6]) if len(parts) > 6 and parts[6] else 0.0,
        "lon_std": float(parts[7]) if len(parts) > 7 and parts[7] else 0.0,
        "alt_std": float(parts[8]) if len(parts) > 8 and parts[8] else 0.0,
    }


def parse_vtg(line):
    """
    Example:
    $GNVTG,123.119,T,130.046,M,0.00444,N,0.00822,K,A*38
    """
    body = line[1:line.find("*")]
    parts = body.split(",")

    return {
        "course_true": float(parts[1]) if len(parts) > 1 and parts[1] else 0.0,
        "course_mag": float(parts[3]) if len(parts) > 3 and parts[3] else 0.0,
        "speed_kn": float(parts[5]) if len(parts) > 5 and parts[5] else 0.0,
        "speed_kmh": float(parts[7]) if len(parts) > 7 and parts[7] else 0.0,
        "mode": parts[9] if len(parts) > 9 else "",
    }


def parse_pvtslna(line):
    """
    PVTSLNA:
    bestpos_type, hgt, lat, lon, hgtstd, latstd, lonstd, diffage,
    psrpos_type, ...
    vel_north, vel_east, vel_ground,
    heading_type, heading_length, heading_degree, heading_pitch,
    ...
    """
    body = line[1:line.find("*")]
    _header, payload = body.split(";", 1)
    p = payload.split(",")

    result = {
        "bestpos_type": p[0],
        "bestpos_hgt": float(p[1]),
        "bestpos_lat": float(p[2]),
        "bestpos_lon": float(p[3]),
        "bestpos_hgtstd": float(p[4]),
        "bestpos_latstd": float(p[5]),
        "bestpos_lonstd": float(p[6]),
        "bestpos_diffage": float(p[7]),

        "psrpos_type": p[8],
        "psrpos_hgt": float(p[9]),
        "psrpos_lat": float(p[10]),
        "psrpos_lon": float(p[11]),
        "undulation": float(p[12]),

        "bestpos_svs": int(p[13]),
        "bestpos_solnsvs": int(p[14]),
        "psrpos_svs": int(p[15]),
        "psrpos_solnsvs": int(p[16]),

        "vel_north": float(p[17]),
        "vel_east": float(p[18]),
        "vel_ground": float(p[19]),

        "heading_type": p[20],
        "heading_length": float(p[21]),
        "heading_degree": float(p[22]),
        "heading_pitch": float(p[23]),

        "heading_trackedsvs": int(p[24]),
        "heading_solnsvs": int(p[25]),
        "heading_ggl1": int(p[26]),
        "heading_ggl1l2": int(p[27]),

        "gdop": float(p[28]),
        "pdop": float(p[29]),
        "hdop": float(p[30]),
        "htdop": float(p[31]),
        "tdop": float(p[32]),
        "cutoff": float(p[33]),
    }
    return result


def parse_uniheadinga(line):
    """
    Example:
    #UNIHEADINGA,...;SOL_COMPUTED,NARROW_INT,10736.3838,88.3470,0.0876,0.0000,0.0001,0.0001,"201",52,29,29,29,3,01,3,0*xxxx
    """
    body = line[1:line.find("*")]
    _header, payload = body.split(";", 1)
    p = payload.split(",")

    return {
        "sol_stat": p[0],
        "pos_type": p[1],
        "length": float(p[2]),
        "heading": float(p[3]),
        "pitch": float(p[4]),
        "reserved": float(p[5]),
        "hdgstddev": float(p[6]),
        "ptchstddev": float(p[7]),
        "station_id": p[8].strip('"'),
        "num_sats": int(p[9]),
        "soln_sats": int(p[10]),
        "obs": int(p[11]),
        "multi": int(p[12]),
        "reserved2": int(p[13]),
        "ext_sol_stat": p[14],
        "gal_bds3_sig_mask": p[15],
        "gps_glo_bds2_sig_mask": p[16],
    }


class GgaPublisherNode(Node):
    def __init__(self):
        super().__init__("um982_bridge_node")

        self.declare_parameter("port", "/dev/ttyCH341USB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("gga_topic", "/antobot_gps/gga")
        self.declare_parameter("pvtslna_topic", "/antobot_gps/pvtslna")
        self.declare_parameter("heading_raw_topic", "/antobot_gps/uniheading")
        self.declare_parameter("send_init_command", True)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.gga_topic = self.get_parameter("gga_topic").get_parameter_value().string_value
        self.pvtslna_topic = self.get_parameter("pvtslna_topic").get_parameter_value().string_value
        self.heading_raw_topic = self.get_parameter("heading_raw_topic").get_parameter_value().string_value
        self.send_init_command = self.get_parameter("send_init_command").get_parameter_value().bool_value

        # raw topic
        self.gga_pub = self.create_publisher(String, self.gga_topic, 10)
        self.pvtslna_pub = self.create_publisher(String, self.pvtslna_topic, 10)
        self.heading_raw_pub = self.create_publisher(String, self.heading_raw_topic, 10)

        # typed topic
        self.fix_pub = self.create_publisher(NavSatFix, "/antobot_gps", 10)
        self.qual_pub = self.create_publisher(GpsQual, "/antobot_gps/quality", 10)
        self.heading_pub = self.create_publisher(GpsHeading, "/antobot_gps/heading", 10)

        self.rtcm_sub = self.create_subscription(
            RTCM, "/antobot_gps/rtcm", self.rtcm_callback, 10
        )

        self.last_gga = None
        self.last_gst = None
        self.last_vtg = None
        self.last_pv = None
        self.last_heading = None

        self.gps_qual_val = 0


        # Time state
        self.gps_timestamp = self.get_clock().now().to_msg()
        self.gps_time_offset = 99.0
        self.last_fix_stamp = self.gps_timestamp

        # Frequency estimation
        self.gps_time_buf = []

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5, write_timeout=0.5)
            # self.ser.flush()
            self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")
            raise

        if self.send_init_command:
            self.init_um982_output()
       
        self.timer = self.create_timer(0.05, self.read_serial)

    def init_um982_output(self):
        try:
            # Uncomment if you want to clear existing outputs first
            # self.ser.write(b"UNLOG COM2\r\n")
            # self.get_logger().info("Sent: UNLOG COM2")

            # 0.2 = 5Hz according to UM982 manual
            self.ser.write(b"GPGGA COM2 0.2\r\n")
            self.ser.flush()
            self.get_logger().info("Sent: GPGGA COM2 0.2")

            self.ser.write(b"GPGST COM2 0.2\r\n")
            # self.ser.flush()
            self.get_logger().info("Sent: GPGST COM2 0.2")

            self.ser.write(b"GPVTG COM2 0.2\r\n")
            # self.ser.flush()
            self.get_logger().info("Sent: GPVTG COM2 0.2")

            self.ser.write(b"PVTSLNA 0.2\r\n")
            # self.ser.flush()
            self.get_logger().info("Sent: PVTSLNA 0.2")

            self.ser.write(b"UNIHEADINGA 0.2\r\n")
            # self.ser.flush()
            self.get_logger().info("Sent: UNIHEADINGA 0.2")
        except Exception as e:
            self.get_logger().error(f"Failed to init UM982 output: {e}")

    def get_gps_timestamp_utc_from_gga(self, gga):
        """
        Build a Python datetime from GGA UTC field (hhmmss.ss) and today's date.
        """
        try:
            utc_str = gga["utc"]
            if not utc_str:
                return None

            today_date = datetime.today()
            year = today_date.year
            month = today_date.month
            day = today_date.day

            hh = int(utc_str[0:2])
            mm = int(utc_str[2:4])

            sec_float = float(utc_str[4:])
            ss = int(sec_float)
            micro = int((sec_float - ss) * 1e6)

            dt0 = datetime(
                year, month, day,
                hour=hh, minute=mm, second=ss, microsecond=micro
            )
            return dt0
        except Exception as e:
            self.get_logger().warn(f"Failed to parse GPS UTC from GGA: {e}")
            return None

    def update_gps_time_from_gga(self, gga):
        """
        Update GPS timestamp and ROS-vs-GPS offset using parsed GGA UTC.
        """
        current_time = self.get_clock().now().to_msg()
        dt0 = self.get_gps_timestamp_utc_from_gga(gga)

        if dt0 is not None:
            current_time_to_sec = current_time.sec + current_time.nanosec / 1e9
            self.gps_time_offset = current_time_to_sec - dt0.timestamp()

            sec = int(dt0.timestamp())
            nanosec = int((dt0.timestamp() - sec) * 1e9)

            self.gps_timestamp = Time(sec=sec, nanosec=nanosec)
            self.last_fix_stamp = self.gps_timestamp
            self.update_gps_frequency()
        else:
            self.gps_time_offset = 99.0
            self.gps_timestamp = current_time
            self.last_fix_stamp = current_time

    def update_gps_frequency(self):
        """
        Estimate frequency from GPS timestamps.
        """
        try:
            t = self.gps_timestamp.sec + self.gps_timestamp.nanosec / 1e9
            self.gps_time_buf.append(t)
            if len(self.gps_time_buf) > 10:
                self.gps_time_buf.pop(0)
        except Exception:
            pass

    def get_current_frequency_estimate(self):
        try:
            if len(self.gps_time_buf) < 2:
                return 0.0

            dt = self.gps_time_buf[-1] - self.gps_time_buf[0]
            if dt <= 0.0:
                return 0.0

            return float((len(self.gps_time_buf) - 1) / dt)
        except Exception:
            return 0.0

    def publish_navsatfix_from_pvtslna(self, pv):
        msg = NavSatFix()
        msg.header.stamp = self.gps_timestamp
        msg.header.frame_id = "gps"

        msg.latitude = pv["bestpos_lat"]
        msg.longitude = pv["bestpos_lon"]
        msg.altitude = pv["bestpos_hgt"]

        msg.status.service = NavSatStatus.SERVICE_GPS

        # if pv["bestpos_type"] in ["NARROW_INT", "NARROW_FLOAT", "PSRDIFF", "SINGLE"]:
        #     msg.status.status = NavSatStatus.STATUS_FIX
        # else:
        #     msg.status.status = NavSatStatus.STATUS_NO_FIX
        if self.gps_qual_val == 4:
            msg.status.status = 3
        elif self.gps_qual_val == 5:
            msg.status.status = 2
        elif self.gps_qual_val in [1, 2]:
            msg.status.status = 1
        msg.position_covariance[0] = pv["bestpos_latstd"] ** 2
        msg.position_covariance[4] = pv["bestpos_lonstd"] ** 2
        msg.position_covariance[8] = pv["bestpos_hgtstd"] ** 2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.last_fix_stamp = msg.header.stamp
        self.fix_pub.publish(msg)

    def publish_gpsqual(self):
        if self.last_gga is None:
            return

        msg = GpsQual()

        msg.stamp = self.last_fix_stamp
        msg.t_offset = float(self.gps_time_offset)

        # h_acc: prefer GST to align with F9P; fallback to PVTSLNA
        if self.last_gst is not None:
            msg.h_acc = float(math.sqrt(
                self.last_gst["lat_std"] ** 2 + self.last_gst["lon_std"] ** 2
            ))
        elif self.last_pv is not None:
            msg.h_acc = float(math.sqrt(
                self.last_pv["bestpos_latstd"] ** 2 + self.last_pv["bestpos_lonstd"] ** 2
            ))
        else:
            msg.h_acc = 0.0

        msg.gps_qual_val = int(self.last_gga["qual"])
        msg.num_sats = int(self.last_gga["num_sats"])
        msg.hor_dil = float(self.last_gga["hdop"])
        msg.geo_sep = float(self.last_gga["undulation"])

        self.gps_qual_val = int(self.last_gga["qual"])
        
        # sat_info not implemented for now
        msg.sat_info = []

        # v_cog / v_sog: prefer VTG to align with F9P
        if self.last_vtg is not None:
            msg.v_cog = float(self.last_vtg["course_true"])
            msg.v_sog = float(self.last_vtg["speed_kmh"])
        elif self.last_pv is not None:
            msg.v_cog = 0.0
            msg.v_sog = float(abs(self.last_pv["vel_ground"]) * 3.6)
        else:
            msg.v_cog = 0.0
            msg.v_sog = 0.0

        msg.frequency = self.get_current_frequency_estimate()

        self.qual_pub.publish(msg)

    def publish_heading_from_uniheading(self, uh):
        msg = GpsHeading()

        msg.header.stamp = self.last_fix_stamp
        msg.header.frame_id = "gps"

        msg.heading = float(uh["heading"])
        msg.length = float(uh["length"])

        # UNIHEADINGA does not directly provide N/E/D baseline components
        msg.rel_pos_n = 0.0
        msg.rel_pos_e = 0.0
        msg.rel_pos_d = 0.0

        msg.acc_heading = float(uh["hdgstddev"])
        msg.acc_length = 0.0

        msg.gnss_fix_ok = (uh["sol_stat"] == "SOL_COMPUTED")
        msg.rel_pos_valid = (uh["sol_stat"] == "SOL_COMPUTED" and uh["pos_type"] != "NONE")

        if self.last_pv is not None:
            msg.is_moving = abs(self.last_pv["vel_ground"]) > 0.1
        else:
            msg.is_moving = False

        msg.time_diff = 0

        # carr_soln: 0 none, 1 float, 2 fixed
        if uh["pos_type"] == "NARROW_INT":
            msg.carr_soln = 2
        elif uh["pos_type"] in ("NARROW_FLOAT", "L1_FLOAT", "IONOFREE_FLOAT"):
            msg.carr_soln = 1
        else:
            msg.carr_soln = 0

        msg.heading_valid = (uh["pos_type"] != "NONE")

        self.heading_pub.publish(msg)

    def read_serial(self):
        try:
            raw = self.ser.readline()
            self.get_logger().error(f"raw: {raw}")
            if not raw:
                self.get_logger().error(f"no raw")
                return

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                self.get_logger().error(f"no line")
                return

            if line.startswith("$command"):
                self.get_logger().info(f"Device response: {line}")
                return

            # GGA: for timestamp, time offset, quality core fields
            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                if nmea_crc(line):
                    raw_msg = String()
                    raw_msg.data = line + "\r\n"
                    self.gga_pub.publish(raw_msg)
                    self.get_logger().info(f"Published GGA: {line}")

                    self.last_gga = parse_gga(line)
                    self.update_gps_time_from_gga(self.last_gga)

                    self.publish_gpsqual()
                else:
                    self.get_logger().warn(f"Invalid GGA CRC: {line}")
                return

            # GST: for h_acc
            if line.startswith("$GNGST") or line.startswith("$GPGST"):
                if nmea_crc(line):
                    self.last_gst = parse_gst(line)
                    self.publish_gpsqual()
                else:
                    self.get_logger().warn(f"Invalid GST CRC: {line}")
                return

            # VTG: for v_cog / v_sog
            if line.startswith("$GNVTG") or line.startswith("$GPVTG"):
                if nmea_crc(line):
                    self.last_vtg = parse_vtg(line)
                    self.publish_gpsqual()
                else:
                    self.get_logger().warn(f"Invalid VTG CRC: {line}")
                return

            # PVTSLNA: for NavSatFix and fallback quality values
            if line.startswith("#PVTSLNA"):
                if nmea_expend_crc(line):
                    raw_msg = String()
                    raw_msg.data = line
                    self.pvtslna_pub.publish(raw_msg)
                    self.get_logger().info(f"Published PVTSLNA: {line}")

                    self.last_pv = parse_pvtslna(line)
                    self.publish_navsatfix_from_pvtslna(self.last_pv)

                    self.publish_gpsqual()
                else:
                    self.get_logger().warn(f"Invalid PVTSLNA CRC: {line}")
                return

            # UNIHEADINGA: for GpsHeading only
            if line.startswith("#UNIHEADINGA"):
                if nmea_expend_crc(line):
                    raw_msg = String()
                    raw_msg.data = line
                    self.heading_raw_pub.publish(raw_msg)
                    self.get_logger().info(f"Published UNIHEADINGA: {line}")

                    self.last_heading = parse_uniheadinga(line)
                    self.publish_heading_from_uniheading(self.last_heading)
                else:
                    self.get_logger().warn(f"Invalid UNIHEADINGA CRC: {line}")
                return

        except Exception as e:
            self.get_logger().error(f"Read serial failed: {e}")

    def rtcm_callback(self, msg):
        try:
            data = bytes(msg.data)
            self.ser.write(data)
        except Exception as e:
            self.get_logger().error(f"RTCM write failed: {e}")

    def destroy_node(self):
        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("Serial closed.")
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GgaPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()





