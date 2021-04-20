#!/bin/python
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
from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

from multiprocessing import Queue, Process
import argparse as ap
from plant import Plant
import re
from threading import Thread
from enum import Enum, unique, auto
import queue
import time
import sys
import os
import logging
from ast import literal_eval as make_tuple #parse tuple

#import simple_pid as s_pid
#from pymodbus.client.sync import ModbusTcpClient

#-------------------------------------------------------------------------------
#Constants

MAX_Q_LEN = 50

#-----------------------------------------------------------
# Utilities

def verify_is_ip(arg):
	"""
	validate if an argument is an ip address
	"""
	return \
		re.match('(([0-9]{1,3}\.){3}([0-9]{1,3}))|localhost', arg)

#------------------------------------------------------------------------------
# Modbus data block

class CallbackDataBlock(ModbusSparseDataBlock):
	""" A datablock that stores the new value in memory
	and passes the operation to a message queue for further
	processing.
	"""
	def __init__(self, values, queue, fx):
		"""
		Initialize data block
		"""
		self.queue = queue
		self.fx = fx
		super(CallbackDataBlock, self).__init__(values)

	def setValues(self, address, value, ignore=0):
		"""
		Sets the requested values of the datastore

		:param address: The starting address
		:param values: The new values to be set
		:param ignore: if the set command is self generated
		"""
		if not ignore:
			print("addr:", address, "value:", value)
			if not self.queue.full():
				self.queue.put_nowait((self.fx, address, value))
			else:
				print("callback:  queue is full")
		else:
			print("ignored queue")
		super(CallbackDataBlock, self).setValues(address, [value])

#------------------------------------------------------------------------------
# data processing thread

class SoftPLC():
	def __init__(self, plant_q, modbus_server_q, modbus_server_context, log):
		self.plant_q = plant_q
		self.modbus_server_q = modbus_server_q
		self.modbus_server_context = modbus_server_context

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

	def __call__(self):
		#plant_q = self.plant_q
		mb_server_q = self.modbus_server_q
		mb_server_c = self.modbus_server_context
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
					self.log("di")
					plant_ic_map = {
						self.di.START_BTN: plant.start,
						self.di.STOP_BTN: plant.stop, #TODO: stop actions
						self.di.EMERG_BTN: plant.emergency,
					}
					if address in set(item.value for item in ic):
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
					#modbus_context[0].store['d'].setValues(11, [2], ignore=1)
					mb_server_c[0].store['h'].setValues(3, hr.LEVEL, \
														res[plant.Output.LEVEL],\
														ignore=1)
					mb_server_c[0].store['h'].setValues(3, \
														hr.OUTFLOW, \
														res[plant.Output.OUTFLOW],\
														ignore=1)
					mb_server_c[0].store['h'].setValues(3, hr.IN_VALVE, \
														res[plant.Output.IN_VALVE],\
														ignore=1)
					mb_server_c[0].store['h'].setValues(3, hr.OUT_VALVE, \
														res[plant.Output.OUT_VALVE],\
														ignore=1)
					mb_server_c[0].store['h'].setValues(3, hr.SETPOINT, \
														res[plant.Output.SETPOINT],\
														ignore=1)
					#TODO: test if this also generates a callback
					# (this would generate a infinite callback loop)

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

plant_queue = queue.Queue(MAX_Q_LEN)

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

modbus_out_q = queue.Queue(MAX_Q_LEN)
modbus_store = \
	ModbusSlaveContext(di=CallbackDataBlock([17]*100, modbus_out_q, 'di'),
					   co=CallbackDataBlock([17]*100, modbus_out_q, 'co'),
					   hr=CallbackDataBlock([17]*100, modbus_out_q, 'hr'),
					   ir=CallbackDataBlock([17]*100, modbus_out_q, 'ir'))
modbus_context = ModbusServerContext(slaves=modbus_store, single=True)
#modbus_store.setValues(3, SoftPLC.hr.K_P.value, [int(100*tunings[0])])
#modbus_store.setValues(3, SoftPLC.hr.K_I.value, [int(100*tunings[1])])
#modbus_store.setValues(3, SoftPLC.hr.K_D.value, [int(100*tunings[2])])

modbus_identity = ModbusDeviceIdentification()
modbus_identity.VendorName = 'pymodbus'
modbus_identity.ProductCode = 'PM'
modbus_identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
modbus_identity.ProductName = 'Parvus Scala'
modbus_identity.ModelName = 'Parvus Scala 1.0'
modbus_identity.MajorMinorRevision = version.short()

#------------------------------------------------------------------------------
# Soft PLC Instance

soft_plc = SoftPLC(plant_queue, modbus_out_q, modbus_store, log)

soft_plc_proc = Process(target=soft_plc, name="soft PLC") #maybe soft_plc.__call__()

# start processes
soft_plc_proc.start()
plant_proc.start()

StartTcpServer(modbus_context, identity=modbus_identity,
			   address=(args.server_ip, args.server_port))
