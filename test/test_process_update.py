#!/usr/bin/python3

#-------------------------------------------------------------------------------
# Library Imports
from pymodbus.version import version
from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSparseDataBlock, ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

import logging
from multiprocessing import Process
from time import sleep, time
import sys
import os

#from twisted.internet.task import LoopingCall


print (sys.argv)

sys.argv = (sys.argv + ["localhost", "5020", "0"] ) if len(sys.argv) <= 1 else sys.argv
#------------------------------------------------------------------------------
# Global
global g_modbus_s_context

global USE_GLOBAL

try:
	USE_GLOBAL = sys.argv[3]
except IndexError as e:
	USE_GLOBAL = 0


def test_process(context):
		t = 0
		while True:
			values = [t + v for v in range(0, 5)]
			if USE_GLOBAL == 1:
				g_modbus_s_context[0].setValues(3, 0, values)
				print("global registers: {} values: {}"
					  .format(g_modbus_s_context[0].getValues(3, 0, len(values)),
							  values))
			else:
				context[0].setValues(3, 0, values)
				print("reg: {} values: {}"
					  .format(context[0].getValues(3, 0, len(values)),
							  values))
			t+=1
			sleep(0.5)

#------------------------------------------------------------------------------
# logging library
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

#--------------------------------------------------
# Modbus server setup

initval = 1
modbus_store = ModbusSlaveContext(
		di=ModbusSequentialDataBlock(0,[initval  ]*100),
		co=ModbusSequentialDataBlock(0,[initval+1]*100),
		hr=ModbusSequentialDataBlock(0,[initval+2]*100),
		ir=ModbusSequentialDataBlock(0,[initval+3]*100),
)

g_modbus_s_context = ModbusServerContext(slaves=modbus_store, single=True)

modbus_identity = ModbusDeviceIdentification()
modbus_identity.VendorName = 'pymodbus'
modbus_identity.ProductCode = 'PM'
modbus_identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
modbus_identity.ProductName = 'Test'
modbus_identity.ModelName = 'Test 1.0'
modbus_identity.MajorMinorRevision = version.short()

#------------------------------------------------------------------------------
# Test Process

_proc = Process(target=test_process, name="test", args=(g_modbus_s_context,))

# start processes
_proc.start()

StartTcpServer(g_modbus_s_context, identity=modbus_identity,
			   address=(sys.argv[1], int(sys.argv[2])))
