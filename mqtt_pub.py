import paho.mqtt.client as mqtt
import sys
import struct
import datetime
import time
    
HOSTNAME = "test.mosquitto.org"  # Sandbox broker
PORT = 1883  # Default port for unencrypted MQTT

class mqttPub:

    ID = 0x7

    pressure = None
    temperature = None

    # ID, size, time, [crc]
    # pack_format = "".join(["=","BHI", "hh"])

    def __init__(self, topic):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(HOSTNAME, port=PORT, keepalive=60, bind_address="")
        self.topic = topic

    def publish(self):
        try:
            a = time.time()
            t = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            self.client.publish(topic=self.topic, payload=f"t={t},P={self.pressure},T={self.temperature}", qos=0, retain=False)
            print(time.time() - a)
        except:
            pass

    @staticmethod        
    def on_connect(client, userdata, flags, rc):
        print("[MQTT] Connection result: " + str(rc))
        
    @staticmethod        
    def on_publish(client, userdata, mid):
        print("[MQTT] Sent: " + str(mid))
        
    @staticmethod        
    def on_disconnect(client, userdata, rc):
        if rc != 0:
            print("[MQTT] Disconnected unexpectedly")
    

if __name__ == "__main__":
    topic = "ELEC3848D34/field_info/pres_temp"  # '/' is used as the delimiter for sub-topics
    pub = mqttPub(topic)
    pub.pressure = 1e5
    pub.temperature = 27.3
    pub.publish()