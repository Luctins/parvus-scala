#!/usr/bin/env python
"""
Pymodbus Server With Callbacks
--------------------------------------------------------------------------

This is an example of adding callbacks to a running modbus server
when a value is written to it. In order for this to work, it needs
a device-mapping file.
"""
# --------------------------------------------------------------------------- #
# import the modbus libraries we need
# --------------------------------------------------------------------------- #
from pymodbus.version import version
from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer
import time


# --------------------------------------------------------------------------- #
# import the python libraries we need
# --------------------------------------------------------------------------- #
from multiprocessing import Queue, Process
import sys
from math import pi, sin
# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# --------------------------------------------------------------------------- #
# create your custom data block with callbacks
# --------------------------------------------------------------------------- #


class CallbackDataBlock(ModbusSparseDataBlock):
	""" A datablock that stores the new value in memory
	and passes the operation to a message queue for further
	processing.
	"""

	def __init__(self, values,  queue):
		"""
		"""
		#self.devices = devices
		self.queue = queue

		#values = {k: 0 for k in devices.keys()}
		#values[0xbeef] = len(values)  # the number of devices
		super(CallbackDataBlock, self).__init__(values)

	def setValues(self, address, value):
		""" Sets the requested values of the datastore

		:param address: The starting address
		:param values: The new values to be set
		"""
		#print("\naddr:", address, "value:", value)
		super(CallbackDataBlock, self).setValues(address, value)

# --------------------------------------------------------------------------- #
# define your callback process
# --------------------------------------------------------------------------- #


def device_writer(arg):
	""" A worker process that processes new messages
	from a queue to write to device outputs

	"""
	t = 0
	while True:
		time.sleep(0.2)
		x = int(5+5*sin(t))
		print('x:', x)
		arg[0].store['d'].setValues(1, [x])
		t += pi/16


def run_callback_server():
	# ----------------------------------------------------------------------- #
	# initialize your data store
	# ----------------------------------------------------------------------- #
	queue = Queue()
	#devices = read_device_map("device-mapping")
	block= ModbusSequentialDataBlock(1, [0xff]*100)
	#block = CallbackDataBlock([60, 60, 22, 8, 21, 42, 51, 5] + [17]*1000,queue)
	store = ModbusSlaveContext(di=block, co=block, hr=block, ir=block)
	context = ModbusServerContext(slaves=store, single=True)

	# ----------------------------------------------------------------------- #
	# initialize the server information
	# ----------------------------------------------------------------------- #
	identity = ModbusDeviceIdentification()
	identity.VendorName = 'pymodbus'
	identity.ProductCode = 'PM'
	identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
	identity.ProductName = 'pymodbus Server'
	identity.ModelName = 'pymodbus Server'
	identity.MajorMinorRevision = version.short()

	# ----------------------------------------------------------------------- #
	# run the server you want
	# ----------------------------------------------------------------------- #
	p = Process(target=device_writer, args=(context,))
	p.start()
	StartTcpServer(context, identity=identity, address=(sys.argv[1], int(sys.argv[2])))


if __name__ == "__main__":
	run_callback_server()
