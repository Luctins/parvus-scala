#!/usr/bin/python3
"""
Evaluate response from tank by varying the output of the input valve control
signal - with PID controller
@author: Henrique T. Moresco, Henrique Wolf, Lucas M. Mendes, Matheus R. Willemann
"""

#-------------------------------------------------------------------------------
# Library Imports

from pymodbus.version import version
from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSparseDataBlock, ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

from multiprocessing import Queue, Process
#from threading import Thread, Queue
import argparse as ap
from plant import Plant
from enum import Enum, unique, auto

#import time
from time import sleep, time
import sys
import re
import os
import logging
from ast import literal_eval as make_tuple #parse tuple

from twisted.internet.task import LoopingCall

#import simple_pid as s_pid
#from pymodbus.client.sync import ModbusTcpClient

#-------------------------------------------------------------------------------
#Constants

MAX_Q_LEN = 50
DEC_OFS = 100

#------------------------------------------------------------------------------
# Global
global modbus_s_context
t = 0
#-----------------------------------------------------------
# Utilities

def verify_is_ip(arg):
	"""
	validate if an argument is an ip address
	"""
	return \
		re.match('(([0-9]{1,3}\.){3}([0-9]{1,3}))|localhost', arg)

# #------------------------------------------------------------------------------
# # Modbus data block
# class CallbackDataBlock(ModbusSequentialDataBlock):
#		#ModbusSparseDataBlock):
#	""" A datablock that stores the new value in memory
#	and passes the operation to a message queue for further
#	processing.
#	"""
#	#def __init__(self, values, queue, fx):
#	def __init__(self, addr, values, queue, fx):
#		"""
#		Initialize data block
#		"""
#		self.queue = queue
#		self.fx = fx
#		super(CallbackDataBlock, self).__init__(addr, values)
#		#super(CallbackDataBlock, self).__init__(values)

#	def setValues(self, address, values):
#		"""
#		Sets the requested values of the datastore
#		:param address: The starting address
#		:param values: The new values to be set
#		:param ignore: if the set command is self generated
#		"""
#		print("fx: {} addr: {} values: {}".format(self.fx, address, values[1:]))
#		if values[0] == 0:
#			if not self.queue.full():
#				self.queue.put_nowait((self.fx, address, values))
#			else:
#				print("callback: queue is full")
#		else:
#			print("skipped queue")

#		super(CallbackDataBlock, self).setValues(address, values[1:])

#		print("super(self)", super(CallbackDataBlock, self))
#		#super(CallbackDataBlock, self).setValues(address, value)
#		#super().setValues(address, value)

#------------------------------------------------------------------------------
# data processing thread

class SoftPLC():
	def __init__(self, plant_q, modbus_server_q, log):
		self.plant_q = plant_q
		self.modbus_server_q = modbus_server_q

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

	class hr(Enum):
		LEVEL = 1
		OUTFLOW =  2
		IN_VALVE = 3  #bidir
		OUT_VALVE = 4 #bidir
		AUTO_MODE = 5
		SETPOINT =  6
		ERROR = 7
		K_P = 8  #bidir
		K_I = 9  #bidir
		K_D = 10 #bidir

	class di(Enum):
		START_BTN = 11
		STOP_BTN = 12
		EMERG_BTN = 13

	#class ir(Enum):


	def __call__(self):
		global t
		#plant_q = self.plant_q
		mb_server_q = self.modbus_server_q
		while True:
			#read value from modbus server
			if not mb_server_q.empty():
				fx, address, value = mb_server_q.get_nowait()
				log.debug("read fx:{} address: {} value:{}".format(fx, address, value))
				# update plant from modbus register actions

				if fx == 'hr':
					plant_hr_map = {
						self.hr.IN_VALVE: plant.set_in_valve,
						self.hr.OUT_VALVE: plant.set_in_valve,
						self.hr.SETPOINT: plant.set_setpoint,
						self.hr.AUTO_MODE: plant.pid.set_auto_mode,
						self.hr.K_P: plant.set_kp,
						self.hr.K_I: plant.set_ki,
						self.hr.K_D: plant.set_kd,
					}
					# test address in one of the watched addresses
					if address in plant_hr_map.keys():
						plant_hr_map[address](value)

				elif fx == 'di':
					print("di")
					plant_ic_map = {
						self.di.START_BTN: plant.start,
						self.di.STOP_BTN: plant.stop, #TODO: stop actions
						self.di.EMERG_BTN: plant.emergency,
					}
					if address in plant_ic_map.keys():
						if value:
							plant_ic_map[address]()
				elif fx == 'co':
					log.error('invalid function code co')
				elif fx == 'ir':
					#TODO: use input registers instead of bidir holding registers
					log.error('invalid function code ir')


			#print("\n\n\n aaaaa \n\n\n")
			if not self.plant_q.empty():
				res = self.plant_q.get_nowait()
				print("n\nres:", res)
				if res:
				# Write values from plant to modbus registers
					pass
					#modbus_context[0].store['d'].setValues(11, [2], ignore=1)
					# v = mb_server_c[0].store['i']
					# v.setValues(self.hr.LEVEL.value,
					#			int(DEC_OFS*res[plant.Output.LEVEL]),
					#			ignore=1)
					# v.setValues(self.hr.OUTFLOW.value,
					#			int(DEC_OFS*res[plant.Output.OUTFLOW]),
					#			ignore=1)
					# v.setValues(self.hr.IN_VALVE.value,
					#			int(DEC_OFS*res[plant.Output.IN_VALVE]),
					#			ignore=1)
					# v.setValues(self.hr.OUT_VALVE.value,
					#			int(DEC_OFS*res[plant.Output.OUT_VALVE]),
					#			ignore=1)
					# v.setValues(self.hr.SETPOINT.value,
					#			int(DEC_OFS*res[plant.Output.SETPOINT]),
					#			ignore=1)

			sleep(0.5)
			values = [t, t+1, t+3]
			modbus_s_context[0].setValues(3, 0, values)
			print("registers: {} values: {}"
				  .format(modbus_s_context[0].getValues(3, 0, len(values)),
						  values))
			t+=1
			return


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
initval = 1
modbus_blocks = {
	# 'di':CallbackDataBlock({0:[initval]*100},   modbus_out_q, 'di'),
	# 'co':CallbackDataBlock({0:[initval+1]*100}, modbus_out_q, 'co'),
	# 'hr':CallbackDataBlock({0:[initval+2]*100}, modbus_out_q, 'hr'),
	# 'ir':CallbackDataBlock({0:[initval+3]*100}, modbus_out_q, 'ir')
	# 'di':CallbackDataBlock(   0,[initval  ]*100, modbus_out_q, 'di'),
	# 'co':CallbackDataBlock(1000,[initval+1]*100, modbus_out_q, 'co'),
	# 'hr':CallbackDataBlock(3000,[initval+2]*100, modbus_out_q, 'hr'),
	# 'ir':CallbackDataBlock(4000,[initval+3]*100, modbus_out_q, 'ir')
	'di':ModbusSequentialDataBlock(0,[initval  ]*100),
	'co':ModbusSequentialDataBlock(0,[initval+1]*100),
	'hr':ModbusSequentialDataBlock(0,[initval+2]*100),
	'ir':ModbusSequentialDataBlock(0,[initval+3]*100)
}
#modbus_block = CallbackDataBlock({0:[initval]*100}, modbus_out_q)

modbus_store = ModbusSlaveContext(
		di=modbus_blocks['di'],
		co=modbus_blocks['co'],
		hr=modbus_blocks['hr'],
		ir=modbus_blocks['ir']
)
		#di=modbus_block,
		#co=modbus_block,
		#hr=modbus_block,
		#ir=modbus_block
modbus_s_context = ModbusServerContext(slaves=modbus_store, single=True)

#modbus_store.setValues(3, SoftPLC.hr.K_P.value, [-1*int(100*tunings[0])])
#modbus_store.setValues(3, SoftPLC.hr.K_I.value, [-1*int(100*tunings[1])])
#modbus_store.setValues(3, SoftPLC.hr.K_D.value, [-1*int(100*tunings[2])])

modbus_identity = ModbusDeviceIdentification()
modbus_identity.VendorName = 'pymodbus'
modbus_identity.ProductCode = 'PM'
modbus_identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
modbus_identity.ProductName = 'Parvus Scala'
modbus_identity.ModelName = 'Parvus Scala 1.0'
modbus_identity.MajorMinorRevision = version.short()

#------------------------------------------------------------------------------
# Soft PLC Instance

soft_plc = SoftPLC(plant_queue, modbus_out_q, log)

loop = LoopingCall(f=soft_plc)
loop.start(0.25, now=False)
#soft_plc_proc = Process(target=soft_plc, name="soft PLC") #maybe soft_plc.__call__()

# start processes
#soft_plc_proc.start()
#plant_proc.start()


StartTcpServer(modbus_s_context, identity=modbus_identity,
			   address=(args.server_ip, args.server_port))
