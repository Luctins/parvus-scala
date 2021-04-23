#!/bin/python

from pyModbusTCP.server import ModbusServer, DataBank
from time import sleep
from random import uniform
import sys

# Create an instance of ModbusServer
server = ModbusServer(sys.argv[1], int(sys.argv[2]), no_block=True)

try:
	print("Start server...")
	server.start()
	print("Server is online")
	state = [0]
	while True:
		#DataBank.set_words(0, [int(uniform(0, 100))])
		#if state != DataBank.get_words(0, 20):
		state = DataBank.get_words(0, 20)
		DataBank.set_words()
		print("regs:", state)
		#print("Value of Register 1 has changed to " +str(state))
		sleep(0.5)

except:
	print("Shutdown server ...")
	server.stop()
	print("Server is offline")
