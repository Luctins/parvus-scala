#!/bin/python
"""
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
import argparse as ap
from plant import Plant, DEC_OFS
from enum import Enum, unique, auto

from time import sleep, time
import sys
import re
import os
import logging
from ast import literal_eval as make_tuple #parse tuple

from twisted.internet.task import LoopingCall

#-------------------------------------------------------------------------------
# Constants

MAX_Q_LEN = 50
#logging level
LOG_LEVEL = logging.INFO #DEBUG

#-----------------------------------------------------------
# Utilities

def verify_is_ip(arg):
	"""
	validate if an argument is an ip address
	"""
	return re.match('(([0-9]{1,3}\.){3}([0-9]{1,3}))|localhost', arg)

#------------------------------------------------------------------------------
# Modbus data block

class CallbackDataBlock(ModbusSequentialDataBlock):
	"""
	A datablock that stores the new value in memory
	and passes the operation to a message queue for further
	processing.
	"""
	def __init__(self, addr, values, queue, fx):
		"""
		Initialize data block
		"""
		self.queue = queue
		self.fx = fx
		self.log = logging.getLogger()
		super(CallbackDataBlock, self).__init__(addr, values)

	def setValues(self, address, values):
		"""
		Sets the requested values of the datastore
		:param address: The starting address
		:param values: The new values to be set, if the first value of address
		is a string, it signals that it's a self generated command and should
		not be sent to the queue #GAMBIARRA
		:param ignore: if the set command is self generated
		"""
		# if first value on values is str, don't put it on the queue
		# (self generated)
		if type(values[0]) == str:
			self.log.debug("skipped queue")
			values = values[1:]
		else:
			if not self.queue.full():
				self.queue.put_nowait((self.fx, address, values))
			else:
				self.log.error("callback:  queue is full")

		self.log.debug("address: {} values: {}".format(address, values))
		#call parent class method
		super(CallbackDataBlock, self).setValues(address, values)

#------------------------------------------------------------------------------
# Data processing loop

class SoftPLC():
	"""
	Class that integrates a modbus server and PID controller input and outputs
	"""
	def __init__(self, plant_q, modbus_server_q, modbus_server_context, log):
		"""
		Initialize variables
		"""
		self.plant_out_q = plant_q['out']
		self.plant_in_q  = plant_q['in']
		self.modbus_q = modbus_server_q
		self.modbus_c = modbus_server_context
		self.log = log

	class hr(Enum):
		"""
		Holding registers
		"""
		# Out 0:49
		LEVEL = 0
		OUTFLOW =  1
		ERROR = 2
		DEC_OFS = 3
		# Bidir 50:99
		OUT_VALVE = 50
		K_P = 51
		K_I = 52
		K_D = 53
		# In 100:-
		IN_VALVE = 100
		SETPOINT = 101

	class co(Enum):
		"""
		Binary inputs
		"""
		START_BTN = 0
		STOP_BTN = 1
		EMERG_BTN = 2
		AUTO_MODE = 3

	def __call__(self):
		"""
		Main method called in a loop
		"""

		# mapping of addresses and commands
		plant_co_map = {
			self.co.START_BTN.value: plant.Command.START,
			self.co.STOP_BTN.value:  plant.Command.STOP,
			self.co.EMERG_BTN.value: plant.Command.EMERGENCY,
			self.co.AUTO_MODE.value: plant.Command.AUTO_MODE,
		}
		# mapping of holding registers
		plant_hr_map = {
			self.hr.IN_VALVE.value:  plant.Command.IN_VALVE,
			self.hr.OUT_VALVE.value: plant.Command.OUT_VALVE,
			self.hr.SETPOINT.value:  plant.Command.SETPOINT,
			self.hr.K_P.value:       plant.Command.SET_K_P,
			self.hr.K_I.value:       plant.Command.SET_K_I,
			self.hr.K_D.value:       plant.Command.SET_K_D,
		}

		# Process write requests
		# read modbus requests
		if not self.modbus_q.empty():
			fx, address, value = self.modbus_q.get_nowait()

			if type(value) == list and len(value) > 1:
				self.log.error("multi register write!\n\n\n")

			cmd = () #initialize variable
			address -= 1 #remove addresss offset
			self.log.debug('fx: %s address: %i value: %i' % (fx, address, value[0]))

			# process request by function code
			if fx == 'hr':
				if address in plant_hr_map.keys():
					cmd = (plant_hr_map[address], value[0])
				else:
					self.log.warning('unkwnown hr address %i' % address)
			if fx == 'co':
				if address in plant_co_map.keys():
					cmd = (plant_co_map[address], value[0])
				else:
					self.log.warning('unkwnown co address %i' % address)
			else:
				self.log.warning("write on read only address")

			#send command to plant
			if cmd and (not self.plant_in_q.full()):
				self.plant_in_q.put_nowait(cmd)

		#read plant output values
		if not self.plant_out_q.empty():
			res = self.plant_out_q.get_nowait()
			if res:
				#update modbus registers from plant result
				v = self.modbus_c[0]
				v.setValues(3, self.hr.LEVEL.value,
							['s', int(DEC_OFS*res[plant.Output.LEVEL])])
				v.setValues(3, self.hr.OUTFLOW.value,
							['s', int(DEC_OFS*res[plant.Output.OUTFLOW])])
				v.setValues(3, self.hr.IN_VALVE.value,
							['s', int(DEC_OFS*res[plant.Output.IN_VALVE])])
				v.setValues(3, self.hr.OUT_VALVE.value,
							['s', int(DEC_OFS*res[plant.Output.OUT_VALVE])])
				v.setValues(3, self.hr.SETPOINT.value,
							['s', int(DEC_OFS*res[plant.Output.SETPOINT])])

#------------------------------------------------------------------------------
# Implementation
#------------------------------------------------------------------------------

# Command line parser setup
parser = ap.ArgumentParser(description='Parvus Scala')
parser.add_argument('plant_ip', type=ascii, help='Modbus plant server IP')
parser.add_argument('server_ip', type=ascii, \
					help='Modbus server (self) IP')
parser.add_argument('-p', '--server_port', metavar='server_port',\
					type=int, help='Modbus (self) server port, defaults to 5020',\
					default=5020, required=0)
parser.add_argument('--plant_port', type=int, metavar="plant port",\
					help='Modbus plant, defaults to 5020',\
					default=5020, required=0)
parser.add_argument('--tunings', type=make_tuple, \
					help='PID tunings as Kp,Ki,Kd',\
					metavar="K_p,K_i,K_d", required=0)
args = parser.parse_args()

#------------------------------------------------------------------------------
# logging library
logging.basicConfig()
log = logging.getLogger()
log.setLevel(LOG_LEVEL)

#Check that input IP's makes sense
args.plant_ip = args.plant_ip[1:-1]
if not verify_is_ip(args.plant_ip):
	log.error('provided plant ip', args.ip, ' is invalid')
	sys.exit(-1)

args.server_ip = args.server_ip[1:-1]
if not verify_is_ip(args.server_ip):
	log.error('provided server ip', args.ip, ' is invalid')
	sys.exit(-1)

#--------------------------------------------------
# Plant setup

#plant input and output queues
plant_queues = { 'out':Queue(MAX_Q_LEN), 'in':Queue(MAX_Q_LEN) }

#alternate controllers
p_tunings = (-41.0959, -0.0, -0.0 )
pid_tunings = (-5, -1.517, -13.593 )

pi_tunings = (-12.7426, -1.453, -0.0)
tunings = pi_tunings

#Test if tunings were passed from the command line
if not args.tunings:
	log.info("using default tunings:", tunings)
else:
	log.info("tunings:\n\tK_p: %.3f\n\tK_d: %.3f\n\tK_i: %.3f"
			 % (tunings[0], tunings[1], tunings[2]))
	tunings = args.tunings

# Create plant instance
plant = Plant(tunings , (args.plant_ip, args.plant_port), plant_queues,
			  log_level=LOG_LEVEL)
plant_proc = Process(target=plant.run, name="plant", args=(0, 0, 0))

#--------------------------------------------------
# Modbus server setup

modbus_q = Queue(MAX_Q_LEN)
initval = 21

modbus_store = ModbusSlaveContext(
	di=CallbackDataBlock(0, [initval]*1000, modbus_q, "di"),
	co=CallbackDataBlock(0, [initval]*1000, modbus_q, "co"),
	hr=CallbackDataBlock(0, [initval]*1000, modbus_q, "hr"),
	ir=CallbackDataBlock(0, [initval]*1000, modbus_q, "ir")
)

modbus_context = ModbusServerContext(slaves=modbus_store, single=True)

#Set initial values for registers
modbus_store.setValues(3, SoftPLC.hr.K_P.value, [int(-1*DEC_OFS*tunings[0])])
modbus_store.setValues(3, SoftPLC.hr.K_I.value, [int(-1*DEC_OFS*tunings[1])])
modbus_store.setValues(3, SoftPLC.hr.K_D.value, [int(-1*DEC_OFS*tunings[2])])
modbus_store.setValues(3, SoftPLC.hr.DEC_OFS.value, [DEC_OFS])
modbus_store.setValues(3, SoftPLC.hr.IN_VALVE.value, [5*DEC_OFS])
modbus_store.setValues(1, SoftPLC.co.AUTO_MODE.value, [True])

modbus_identity = ModbusDeviceIdentification()
modbus_identity.VendorName = 'pymodbus'
modbus_identity.ProductCode = 'PS'
modbus_identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
modbus_identity.ProductName = 'Parvus Scala'
modbus_identity.ModelName = 'Parvus Scala 1.0'
modbus_identity.MajorMinorRevision = version.short()

#------------------------------------------------------------------------------
# Soft PLC Instance

soft_plc_loopdelay = 0.010 #10 ms
soft_plc = SoftPLC(plant_queues, modbus_q, modbus_context, log)
soft_plc_loop = LoopingCall(f=soft_plc)

# start processes
soft_plc_loop.start(soft_plc_loopdelay)
plant_proc.start()

StartTcpServer(modbus_context, identity=modbus_identity,
			   address=(args.server_ip, args.server_port))
