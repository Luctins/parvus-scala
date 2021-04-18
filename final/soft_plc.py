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
import plant
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
				print("queue is full")
		super(CallbackDataBlock, self).setValues(address, value)

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

	class ic(Enum):
		START_BTN = 11
		STOP_BTN = 12
		EMERG_BTN = 13

	def __call__(self):
		p_sys_ui_q = self.plant_q['in']
		p_ui_sys_q = self.plant_q['out']
		mb_server_q = self.modbus_server_q
		mb_server_c = self.modbus_server_context
		while True:
			#read value from modbus server
			fx, address, value = mb_server_q.get_nowait()
			if (fx and address and value):
				# update plant from modbus register actions
				if fx == 'hr':
					# test address in one of the watched addresses
					plant_hr_map = {
						hr.IN_VALVE: plant.set_in_valve,
						hr.OUT_VALVE: plant.set_in_valve,
						hr.SETPOINT: plant.set_setpoint,
						hr.AUTO_MODE: plant.pid.set_set_point,
						hr.K_P: plant.set_kp,
						hr.K_I: plant.set_ki,
						hr.K_D: plant.set_kd,
					}
					if address in plant_input_map.keys():
						plant_input_map[address](value)

				elif fx == 'ic':
					plant_ic_map = {
						ic.START_BTN: plant.start,
						ic.STOP_BTN: plant.stop, #TODO: stop actions
						ic.EMERG_BTN: plant.emergency,
					}
					if address in set(item.value for item in ic):
						if value:
							plant_ic_map[address]();
				elif fx == 'dc':
					log.error('invalid function code write')
				elif fx == 'ir':
					#TODO: use input registers instead of bidir holding registers
					log.error('invalid function code write')


			log.debug("read fx", fx, "address", address, "value", value)

			res = p_sys_ui_q.get_nowait()
			if res:
				# Write values from plant to modbus registers
				mb_server_c.setValues(3, hr.LEVEL, res[plant.Output.LEVEL])
				mb_server_c.setValues(3, hr.OUTFLOW, res[plant.Output.OUTFLOW])
				mb_server_c.setValues(3, hr.IN_VALVE, res[plant.Output.IN_VALVE])
				mb_server_c.setValues(3, hr.OUT_VALVE, res[plant.Output.OUT_VALVE])
				mb_server_c.setValues(3, hr.SETPOINT, res[plant.Output.SETPOINT])
				#TODO: test if this also generates a callback
				# (this would generate a infinite callback loop)

#------------------------------------------------------------------------------
# Implementation
#------------------------------------------------------------------------------

# Command line parser setup
parser = ap.ArgumentParser(description='Parvus Scala')
parser.add_argument('plant_ip', type=ascii, help='Modbus plant server IP')
parser.add_argument('server_ip', type=ascii, \
					help='Modbus server server (self) IP')
parser.add_argument('-p', metavar='port',\
					type=int, help='Modbus server, defaults to 520',\
					default=520, required=0)
parser.add_argument('--plant_port', type=int, \
					help='Modbus plant, defaults to 520',\
					default=520, required=0)
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

# Open Plant logfile
logdir = 'log'
logname = \
	'data_log_{}_P{:.3f}_I{:.3f}_D{:.3f}.csv'.format(int(time.time()),\
													 pid.Kp, pid.Ki, pid.Kd)
# Create 'log/' folder if it does not exist
if not os.path.exists('./'+logdir+'/'):
	os.mkdir(logdir)
	print('created logdir:', logdir)
logf = open((logdir+'/'+logname).encode('utf8'), 'wb')
print('opened logfile:', logname)

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
	plant(tunings , (args.plant_ip, args.plant_port), logf, log, plant_queue)
plant_proc = Process(target=plant.run, args=(0, 0, 0))

#--------------------------------------------------
# Modbus server setup

modbus_out_q = queue.Queue(MAX_Q_LEN)
modbus_store = \
	ModbusSlaveContext(di=CallbackDataBlock([17]*100, modbus_out_q, 'di'),
					   co=CallbackDataBlock([17]*100, modbus_out_q, 'co'),
					   hr=CallbackDataBlock([17]*100, modbus_out_q, 'hr'),
					   ir=CallbackDataBlock([17]*100, modbus_out_q, 'ir'))
modbus_context = ModbusServerContext(slaves=store, single=True)
modbus_context.setValues(3, hr.K_P, tunings[0])
modbus_context.setValues(3, hr.K_I, tunings[1])
modbus_context.setValues(3, hr.K_D, tunings[2])

modbus_identity = ModbusDeviceIdentification()
modbus_identity.VendorName = 'pymodbus'
modbus_identity.ProductCode = 'PM'
modbus_identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
modbus_identity.ProductName = 'Parvus Scala'
modbus_identity.ModelName = 'Parvus Scala 1.0'
modbus_identity.MajorMinorRevision = version.short()

#------------------------------------------------------------------------------
# Soft PLC Instance

soft_plc = SoftPLC(plant_queues, modbus_out_q, modbus_store, log)

soft_plc_proc = Process(target=soft_plc) #maybe soft_plc.__call__()

# start processes
modbus_server.start()
plant_proc.start()

StartTcpServer(context, identity=identity, \
			   address=(args.port, args.server_port))
