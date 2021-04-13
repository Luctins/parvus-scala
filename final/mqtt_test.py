import paho.mqtt.client as mqtt
import argparse
from time import sleep
from math import sin, pi
import sys

#MQTT callbacks
def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))
	cli.subscribe("test", 0)
	cli.subscribe("vasco", 0)
	#cli.subscribe("balanca/massa",0)

def on_message(client, userdata, message):
	print("received message " + str(message.payload) + "topic: " + str(message.topic))
	if str(message.topic) == 'test':
		cmd = str(message.payload)[2:-1] + '\n';
		print("recv:", cmd)
	if str(message.topic) == 'vasco':
		print("vasco")
	else:
		print("topic:", str(message.topic))

def on_log(client, userdata, level, buf):
	if (level == MQTT_LOG_ERR ) | (level == MQTT_LOG_WARNING):
		print("log:", buf)


cli = mqtt.Client(client_id="clp")
cli.on_connect = on_connect
cli.on_message = on_message
cli.on_log = on_log

cli.connect(sys.argv[1], int(sys.argv[2]), 120, bind_address='localhost')

#starts background listener
cli.loop_start()
#cli.enable_logger()
cli.disable_logger()

t=0
while 1:
	t += pi/16
	v = 10+10*sin(t)
	cli.publish("test", int(v))
	#print(v)
	sleep(0.5)
