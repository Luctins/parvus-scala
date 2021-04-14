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
from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer


# --------------------------------------------------------------------------- #
# import the python libraries we need
# --------------------------------------------------------------------------- #
from multiprocessing import Queue, Process
import sys

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
		print("\n\n\n\n\naddr:", address, "value:", value)
		super(CallbackDataBlock, self).setValues(address, value)
		self.queue.put((self.devices.get(address, None), value))

# --------------------------------------------------------------------------- #
# define your callback process
# --------------------------------------------------------------------------- #


def rescale_value(value):
	""" Rescale the input value from the range
	of 0..100 to -3200..3200.

	:param value: The input value to scale
	:returns: The rescaled value
	"""
	s = 1 if value >= 50 else -1
	c = value if value < 50 else (value - 50)
	return s * (c * 64)


def device_writer(queue):
	""" A worker process that processes new messages
	from a queue to write to device outputs

	:param queue: The queue to get new messages from
	"""
	while True:
		#TODO: use this taskt to put data into our message queue and pass
		# values from the GUI to the plant
		device, value = queue.get()
		scaled = rescale_value(value[0])
		log.debug("Write(%s) = %s" % (device, value))
		if not device: continue


def run_callback_server():
	# ----------------------------------------------------------------------- #
	# initialize your data store
	# ----------------------------------------------------------------------- #
	queue = Queue()
	#devices = read_device_map("device-mapping")
	block = CallbackDataBlock([17]*100,queue)
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
	p = Process(target=device_writer, args=(queue,))
	p.start()
	StartTcpServer(context, identity=identity, address=(sys.argv[1], int(sys.argv[2])))


if __name__ == "__main__":
	run_callback_server()
