
import serial
import paho.mqtt.client as mqtt
import rospy, rospkg



class gpsCorrections():
    def __init__(self, corr_type):

        self.corr_type = corr_type

        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_devices_gps')
        yaml_file_path = os.path.join(packagePath, "/config/corrections_config.yaml")
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)
            self.ppp_client_id = config['ppp']['device_ID']
            self.ant_client_id = "Antobot_device_" + device_IDconfig['device_ID']
            self.ant_mqtt_topic_sub = "Anto_MQTT_F9P_" + config['base_ID']
            self.ant_broker = config['mqtt_Broker']
            self.ant_mqtt_port = config['mqtt_Port']
            self.mqtt_keepalive = config['mqtt_keepalive']
            # mqtt_UserName = config['mqtt_UserName']
            # mqtt_PassWord = config['mqtt_PassWord']
            mqtt_username = "antobot_device"
            mqtt_password = "SvHCWJFNP98h"

        if self.corr_type == "ppp":

            dev_port = rospy.get_param("/gps/urcu/device_port","/dev/ttyTHS0")
            baudrate = 460800

            self.mqtt_topics = [(f"/pp/ip/eu", 0), ("/pp/ubx/mga", 0), ("/pp/ubx/0236/ip", 0)]
            self.userdata = { 'gnss': self.nRTK_correction, 'topics': self.mqtt_topics }
            
        elif self.corr_type == "ant_mqtt":
            self.connect = False

            # Topic names and QoS
            self.mqtt_topic_sub = base_id
            self.mqtt_topic_pub = "Antobot_robot_gps"
            
        # Setting up hardware ports
        self.nRTK_correction = serial.Serial(port=dev_port, baudrate=baud)  #38400
        self.spiport = sfeSpiWrapper()
        self.gpio01 = 29
        self.GPIO = GPIO
        self.GPIO.setmode(GPIO.BOARD)
        self.GPIO.setup(self.gpio01, GPIO.OUT)
        #self.GPIO.output(self.gpio01, GPIO.HIGH) # if need 20240424

        # Setting up the MQTT Client
        if self.corr_type == "ppp":
            self.client = mqtt.Client(client_id=self.client_id, userdata=self.userdata)
            self.certfile=os.path.join(packagePath,"config/")+f'device-{self.client_id}-pp-cert.crt'
            self.keyfile=os.path.join(packagePath,"config/")+f'device-{self.client_id}-pp-key.pem'
            self.client.tls_set(certfile=self.certfile,keyfile=self.keyfile)
        elif self.corr_type == "ant_mqtt":
            self.client = mqtt.Client(self.ant_client_id)
            self.client.username_pw_set(mqtt_username, mqtt_password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.connect_broker()
            


    #call back when the broker is connected
    def connect_broker(self):
        try:
            if self.corr_type == "ppp":
                self.client.connect(self.server, port=8883)
            elif self.corr_type == "ant_mqtt":
                self.client.connect(self.broker, self.mqtt_port, self.mqtt_keepalive)
        except:
            print("Trying to connect ...")

        time.sleep(2)
        
    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self,client, userdata, flags, rc):
        
        if rc == 0:
            self.connect = True
            if self.corr_type == "ppp":
                self.client.subscribe(userdata['topics'])
            elif self.corr_type == "ant_mqtt":
                self.client.subscribe(self.mqtt_topic_sub)
        else: 
            print("Connection failed!", rc)

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self,client,userdata, msg):
        # write the corrections via UART2
        if self.corr_type == "ppp":
            data = userdata['gnss'].write(msg.payload)
        elif self.corr_type == "ant_mqtt":
            data = self.nRTK_correction.write(msg.payload)


def main():
    gps_corr = gpsCorrections("ppp")
    
    gps_corr.client.loop_start()
    # while(True):
    #     time.sleep(1)
    #     print("Running")


if __name__ == '__main__':
    main()
    