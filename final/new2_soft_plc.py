#!/bin/python
"""
NOTA:
este código e as 2 outras versões em anexo são uma demonstração

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

import time
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
# Modbus data block


		#ModbusSequentialDataBlock):
# class CallbackDataBlock(ModbusSparseDataBlock):
#	""" A datablock that stores the new value in memory
#	and passes the operation to a message queue for further
#	processing.
#	"""
#	def __init__(self, values, queue, fx):
#		"""
#		Initialize data block
#		"""
#		self.queue = queue
#		self.fx = fx
#		super(CallbackDataBlock, self).__init__(values)
#		#super().__init__(values)

#	def setValues(self, address, value, ignore=0):
#		"""
#		Sets the requested values of the datastore
#		:param address: The starting address
#		:param values: The new values to be set
#		:param ignore: if the set command is self generated
#		"""
#		print("addr:", address, "value:", value)
#		if not ignore:
#			if not self.queue.full():
#				self.queue.put_nowait((self.fx, address, value))
#			else:
#				print("callback:  queue is full")
#		else:
#			print("skipped queue")
#		super(CallbackDataBlock, self).setValues(address, value)
#		#super(CallbackDataBlock, self).setValues(address, value)
#		#super().setValues(address, value)

#------------------------------------------------------------------------------
# data processing thread

class SoftPLC():
	def __init__(self, plant_q, modbus_server_q, modbus_server_context, log):
		self.plant_q = plant_q
		self.modbus_server_q = modbus_server_q
		self.modbus_server_context = modbus_server_context

	class di(Enum):
		BASE = 1000

	class co(Enum):
		START_BTN = 2001
		STOP_BTN = 2002
		EMERG_BTN = 2003
		BASE = 2000

	class hr(Enum):
		LEVEL = 3001
		OUTFLOW =  3002
		IN_VALVE = 3003
		OUT_VALVE = 3004
		SETPOINT =  3005
		ERROR = 3006
		AUTO_MODE = 3007
		K_P = 3008
		K_I = 3009
		K_D = 3010
		BASE = 3000

	class ir(Enum):
		IN_VALVE =  4001
		OUT_VALVE = 4002
		AUTO_MODE = 4003
		SETPOINT =  4004
		K_P = 4005
		K_I = 4006
		K_D = 4007
		BASE = 4000

	def __call__(self):
		t = 0
		#plant_q = self.plant_q
		mb_server_q = self.modbus_server_q
		mb_server_c = self.modbus_server_context

		plant_co_map = {
			self.co.START_BTN: plant.start,
			self.co.STOP_BTN:  plant.stop,
			self.co.EMERG_BTN: plant.emergency,
		}
		plant_hr_map = {
			self.hr.IN_VALVE: plant.set_in_valve,
			self.hr.OUT_VALVE: plant.set_out_valve,
			self.hr.AUTO_MODE: plant.pid.set_auto_mode,
			self.hr.SETPOINT: plant.set_setpoint,
			self.hr.K_P: plant.set_kp,
			self.hr.K_I: plant.set_ki,
			self.hr.K_D: plant.set_kd,
		}

		last_ic = {}
		last_ir = {}

		while True:

			# regs, change = get_change(mb_server_c[0], 1, last_ic)
			# print("i:", i, regs, "change:", c)
			# if change:
			#	for val in change:
			#		if val in plant_ir_map.keys():
			#			plant_ir_map[val](r)

			base = self.co.BASE.value
			#print( mb_server_c[0].store['h'].values[0:20])
			regs = mb_server_c[0].getValues(2, 1000, 20); print("reg:", regs)
			regs = mb_server_c[0].getValues(1, 2000, 20); print("reg:", regs)
			regs = mb_server_c[0].getValues(3, 3000, 20); print("reg:", regs)
			regs = mb_server_c[0].getValues(4, 4000, 20); print("reg:", regs)
			#_regs = set(regs)
			#change = _regs.difference(last_regs)
			time.sleep(0.5)
			last_co = regs
			continue
			if change:
				for i in range(0, len(change)):
					if (i+base) in plant_ir_map.keys():
						plant_di_map[base+i](change[i])



			#print("\n\n\n aaaaa \n\n\n")
			if not self.plant_q.empty():
				res = self.plant_q.get_nowait()
				print("n\nres:", res)
				if res:
				# Write values from plant to modbus registers
					#modbus_context[0].store['d'].setValues(11, [2], ignore=1)
					v = mb_server_c[0]
					v.setValues(3,self.hr.LEVEL.value,
								int(DEC_OFS*res[plant.Output.LEVEL]),
								ignore=1)
					v.setValues(3,self.hr.OUTFLOW.value,
								int(DEC_OFS*res[plant.Output.OUTFLOW]),
								ignore=1)
					v.setValues(3,self.hr.IN_VALVE.value,
								int(DEC_OFS*res[plant.Output.IN_VALVE]),
								ignore=1)
					v.setValues(3,self.hr.OUT_VALVE.value,
								int(DEC_OFS*res[plant.Output.OUT_VALVE]),
								ignore=1)
					v.setValues(3,self.hr.SETPOINT.value,
								int(DEC_OFS*res[plant.Output.SETPOINT]),
								ignore=1)

					#print("mem:", mb_server_c[0].store['i'].values[0:20])
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
initval = 0
#modbus_block = ModbusSparseDataBlock({0:[0]*1000})
modbus_block = ModbusSparseDataBlock({1000:[0]*1000, 2000:[0]*1000,
									  3000:[0]*1000, 4000:[0]*1000 })
modbus_blocks = {
	'di':ModbusSparseDataBlock({1000:[0]*1000}),
	'co':ModbusSparseDataBlock({2000:[0]*1000}),
	'hr':ModbusSparseDataBlock({3000:[initval+2]*1000}),
	'ir':ModbusSparseDataBlock({4000:[initval+3]*1000})

	# 'di':ModbusSparseDataBlock({0:[0]*1000}),
	# 'co':ModbusSparseDataBlock({0:[0]*1000}),
	# 'hr':ModbusSparseDataBlock({0:[initval]*1000}),
	# 'ir':ModbusSparseDataBlock({0:[initval+3]*1000})

	# 'di':ModbusSequentialDataBlock(0,[0]*1000),
	# 'co':ModbusSequentialDataBlock(0,[0]*1000),
	# 'hr':ModbusSequentialDataBlock(0,[0]*1000),
	# 'ir':ModbusSequentialDataBlock(0,[0]*1000)
}
	# 'di':CallbackDataBlock({0:[initval]*100},   modbus_out_q, 'di'),
	# 'co':CallbackDataBlock({0:[initval+1]*100}, modbus_out_q, 'co'),
	# 'hr':CallbackDataBlock({0:[initval+2]*100}, modbus_out_q, 'hr'),
	#'ir':CallbackDataBlock({0:[initval+3]*100}, modbus_out_q, 'ir')
#modbus_block = CallbackDataBlock({0:[initval]*100}, modbus_out_q)

modbus_store = \
	ModbusSlaveContext(
		di=modbus_block,
		co=modbus_block,
		hr=modbus_block,
		ir=modbus_block
		# di=modbus_blocks['di'],
		# co=modbus_blocks['co'],
		# hr=modbus_blocks['hr'],
		# ir=modbus_blocks['ir']
	)
modbus_context = ModbusServerContext(slaves=modbus_store, single=True)
modbus_store.setValues(3, SoftPLC.hr.K_P.value, [-1*int(100*tunings[0])])
modbus_store.setValues(3, SoftPLC.hr.K_I.value, [-1*int(100*tunings[1])])
modbus_store.setValues(3, SoftPLC.hr.K_D.value, [-1*int(100*tunings[2])])

modbus_identity = ModbusDeviceIdentification()
modbus_identity.VendorName = 'pymodbus'
modbus_identity.ProductCode = 'PM'
modbus_identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
modbus_identity.ProductName = 'Parvus Scala'
modbus_identity.ModelName = 'Parvus Scala 1.0'
modbus_identity.MajorMinorRevision = version.short()

#------------------------------------------------------------------------------
# Soft PLC Instance

soft_plc = SoftPLC(plant_queue, modbus_out_q, modbus_context, log)

soft_plc_proc = Process(target=soft_plc, name="soft PLC") #maybe soft_plc.__call__()

# start processes
soft_plc_proc.start()
#plant_proc.start()

# modbus_context[0].setValues(1, 2, [50])
# modbus_context[0].setValues(2, 2, [50])
# modbus_context[0].setValues(3, 2, [50])
# modbus_context[0].setValues(4, 3, [50])
# print("val:")
# print(modbus_context[0].getValues(1,2,1))
# print(modbus_context[0].getValues(2,2,1))
# print(modbus_context[0].getValues(3,2,1))
# print(modbus_context[0].getValues(4,2,1))
#t += 1
#t %= 99
#print(modbus_context[0].store['h'])

StartTcpServer(modbus_context, identity=modbus_identity,
			   address=(args.server_ip, args.server_port))
