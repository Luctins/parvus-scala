import paho.mqtt.client as mqtt
import time
topic = "interface_test"

#MQTT callbacks
def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))
	client.subscribe("interface_test",0)

def on_message(client, userdata, message):
	print("received publish on topic:", \
		  message.topic, "payload:", message.payload)

def on_log(client, userdata, level, buf):
	if (level == MQTT_LOG_ERR ) | (level == MQTT_LOG_WARNING):
		print("log:", buf)


client = mqtt.Client(client_id="teste")

client.on_connect = on_connect
client.on_message = on_message
client.on_log = on_log

client.connect("localhost", 1883, 120)

#starts background listener
client.loop_start()
client.enable_logger()
#cli.disable_logger()

x = 0
while 1:
	client.publish(topic, str(x))
	x += 1
	time.sleep(0.5)
