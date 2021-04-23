#!/bin/python
"""
Evaluate response from tank by varying the output of the input valve control
signal - with PID controller
@author: Henrique T. Moresco, Henrique Wolf, Lucas M. Mendes, Matheus R. Willemann
"""

#-------------------------------------------------------------------------------
# Library Imports

# from pymodbus.version import version
# from pymodbus.server.asynchronous import StartTcpServer
# from pymodbus.device import ModbusDeviceIdentification
# from pymodbus.datastore import ModbusSparseDataBlock, ModbusSequentialDataBlock
# from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
# from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

#from local_pymodbustcp.pyModbusTCP.server import ModbusServer, DataBank
from pyModbusTCP.server import ModbusServer, DataBank

from multiprocessing import Queue, Process
#from threading import Thread, Queue
import argparse as ap
from plant import Plant
from enum import Enum, unique, auto

from time import sleep, time
import sys
import re
import os
import logging
from ast import literal_eval as make_tuple #parse tuple

#import simple_pid as s_pid
#from pymodbus.client.sync import ModbusTcpClient

#-------------------------------------------------------------------------------
#Constants

MAX_Q_LEN = 50
DEC_OFS = 100

#-----------------------------------------------------------
# Utilities

def verify_is_ip(arg):
	"""
	validate if an argument is an ip address
	"""
	return \
		re.match('(([0-9]{1,3}\.){3}([0-9]{1,3}))|localhost', arg)

#------------------------------------------------------------------------------
# data processing thread

class SoftPLC():
	def __init__(self, plant_q, modbus_q, log):
		self.plant_q = plant_q
		self.server_q = modbus_q
		self.log = log

	class out_reg(Enum):
		LEVEL = 1
		OUTFLOW =  2
		OUT_VALVE = 3
		ERROR = 4
		K_P = 5
		K_I = 6
		K_D = 7


	class in_reg(Enum):
		START_BTN = 50
		STOP_BTN = 51
		EMERG_BTN = 52
		IN_VALVE = 53
		OUT_VALVE = 54
		AUTO_MODE = 55
		SETPOINT =  56
		K_P = 57
		K_I = 58
		K_D = 59

	#class ir(Enum):


	def __call__(self):
		t = 0
		#plant_q = self.plant_q
		mb_server_q = self.server_q
		last_regs = []
		plant_input_map = {
			self.in_reg.START_BTN: plant.start,
			self.in_reg.STOP_BTN:  plant.stop,
			self.in_reg.EMERG_BTN: plant.emergency,
			self.in_reg.IN_VALVE: plant.set_in_valve,
			self.in_reg.OUT_VALVE: plant.set_out_valve,
			self.in_reg.AUTO_MODE: plant.pid.set_auto_mode,
			self.in_reg.SETPOINT: plant.set_setpoint,
			self.in_reg.K_P: plant.set_kp,
			self.in_reg.K_I: plant.set_ki,
			self.in_reg.K_D: plant.set_kd,
		}
		while True:
			regs = DataBank.get_words(0, 10)
			print(regs)
			sleep(0.5)
			DataBank.set_words(2, [t])
			t += 1

			# _regs = set(regs)
			# change = _regs.difference(last_regs)
			# last_regs = regs
			# print(regs)
			# time.sleep(0.1)
			# if change:
			#	print("change:", change)
			#	for i in change:
			#		if i in plant_input_map.keys():
			#			plant_input_map[address](value)

			# #print("\n\n\n aaaaa \n\n\n")
			# if not self.plant_q.empty():
			#	res = self.plant_q.get_nowait()
			#	print("n\nres:", res)
			#	if res:
			#		# Write values from plant to modbus registers
			#		#DataBank.set_words(0, [int(uniform(0, 100))])
			#		DataBank.set_words(self.hr.LEVEL.value,
			#						   [int(DEC_OFS*res[plant.Output.LEVEL])],
			#						   )#ignore=1)
			#		DataBank.set_words(self.hr.OUTFLOW.value,
			#						   [int(DEC_OFS*res[plant.Output.OUTFLOW])],
			#						   )#ignore=1)
			#		DataBank.set_words(self.hr.IN_VALVE.value,
			#						   [int(DEC_OFS*res[plant.Output.IN_VALVE])],
			#						   )#ignore=1)
			#		DataBank.set_words(self.hr.OUT_VALVE.value,
			#						   [int(DEC_OFS*res[plant.Output.OUT_VALVE])],
			#						   )#ignore=1)
			#		DataBank.set_words(self.hr.SETPOINT.value,
			#						   [int(DEC_OFS*res[plant.Output.SETPOINT])],
			#						  )#ignore=1)
			#	else:
			#		print("plc: plant read error")

#------------------------------------------------------------------------------
# Implementation
#------------------------------------------------------------------------------

# Command line parser setup
parser = ap.ArgumentParser(description='Parvus Scala')
parser.add_argument('plant_ip', type=ascii, help='Modbus plant server IP')
parser.add_argument('server_ip', type=ascii, \
					help='Modbus server (self) IP')
parser.add_argument('-p', '--server_port', metavar='server_port',\
					type=int, help='Modbus ()self) server port, defaults to 5020',\
					default=5020, required=0)
parser.add_argument('--plant_port', type=int, metavar="plant port",\
					help='Modbus plant, defaults to 5020',\
					default=5020, required=0)
parser.add_argument('--tunings', type=make_tuple, \
					help='PID tunings as Kp,Ki,Kd',\
					metavar="K_p,K_i,K_d", required=0)
args = parser.parse_args()

#Check that input IP's makes sense

args.plant_ip = args.plant_ip[1:-1]
if not verify_is_ip(args.plant_ip):
	print('provided plant ip', args.ip, ' is invalid')
	sys.exit(-1)

args.server_ip = args.server_ip[1:-1]
if not verify_is_ip(args.server_ip):
	print('provided server ip', args.ip, ' is invalid')
	sys.exit(-1)

#------------------------------------------------------------------------------
# logging library
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

#--------------------------------------------------
# Plant setup

plant_queue = Queue(MAX_Q_LEN)

#p_tunings = (-41.0959, -0.0, -0.0 );
pi_tunings = (-12.7426, -1.453, -0.0)
#pid_tunings = (-5, -1.517, -13.593 )
tunings = pi_tunings

#Test if tunings were passed from the command line
if not args.tunings:
	print("using default tunings:", tunings)
else:
	print("tunings: \n  K_p:", tunings[0], "\n  K_d:", tunings[1] ,\
		  "\n  K_i:", tunings[2])
	tunings = args.tunings

# Create plant instance
plant = \
	Plant(tunings , (args.plant_ip, args.plant_port), log, plant_queue)
plant_proc = Process(target=plant.run, name="plant", args=(0, 0, 0))


#--------------------------------------------------
# Modbus server setup

modbus_out_q = Queue(MAX_Q_LEN)
# initval = 0
# modbus_blocks = {
#	'di':CallbackDataBlock({0:[initval]*100},   modbus_out_q, 'di'),
#	'co':CallbackDataBlock({0:[initval+1]*100}, modbus_out_q, 'co'),
#	'hr':CallbackDataBlock({0:[initval+2]*100}, modbus_out_q, 'hr'),
#	'ir':CallbackDataBlock({0:[initval+3]*100}, modbus_out_q, 'ir')
# }
# #modbus_block = CallbackDataBlock({0:[initval]*100}, modbus_out_q)

# modbus_store = \
#	ModbusSlaveContext(
#		di=modbus_blocks['di'],
#		co=modbus_blocks['co'],
#		hr=modbus_blocks['hr'],
#		ir=modbus_blocks['ir']
#	)
#		#di=modbus_block,
#		#co=modbus_block,
#		#hr=modbus_block,
#		#ir=modbus_block
# modbus_context = ModbusServerContext(slaves=modbus_store, single=True)
# modbus_store.setValues(3, SoftPLC.hr.K_P.value, [-1*int(100*tunings[0])])
# modbus_store.setValues(3, SoftPLC.hr.K_I.value, [-1*int(100*tunings[1])])
# modbus_store.setValues(3, SoftPLC.hr.K_D.value, [-1*int(100*tunings[2])])

# modbus_identity = ModbusDeviceIdentification()
# modbus_identity.VendorName = 'pymodbus'
# modbus_identity.ProductCode = 'PM'
# modbus_identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
# modbus_identity.ProductName = 'Parvus Scala'
# modbus_identity.ModelName = 'Parvus Scala 1.0'
# modbus_identity.MajorMinorRevision = version.short()


#------------------------------------------------------------------------------
# Soft PLC Instance

soft_plc = SoftPLC(plant_queue, modbus_out_q, log)

soft_plc_proc = Process(target=soft_plc, name="soft PLC") #maybe soft_plc.__call__()

print("server addr:", (args.server_ip, args.server_port) )

# start processes
soft_plc_proc.start()
#plant_proc.start()

# StartTcpServer(modbus_context, identity=modbus_identity,
#			   address=(args.server_ip, args.server_port))

# Create an instance of ModbusServer
server = ModbusServer(args.server_ip, args.server_port)

server.start()
