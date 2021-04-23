#!/usr/bin/env python
"""
Pymodbus Server With Updating Thread
--------------------------------------------------------------------------

This is an example of having a background thread updating the
context while the server is operating. This can also be done with
a python thread::

	from threading import Thread

	thread = Thread(target=updating_writer, args=(context,))
	thread.start()
"""
# --------------------------------------------------------------------------- #
# import the modbus libraries we need
# --------------------------------------------------------------------------- #
from pymodbus.version import version
from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

# --------------------------------------------------------------------------- #
# import the twisted libraries we need
# --------------------------------------------------------------------------- #
from twisted.internet.task import LoopingCall

# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# --------------------------------------------------------------------------- #
# define your callback process
# --------------------------------------------------------------------------- #


def updating_writer(a):
	""" A worker process that runs every so often and
	updates live values of the context. It should be noted
	that there is a race condition for the update.

	:param arguments: The input arguments to the call
	"""
	log.debug("updating the context")
	context = a[0]
	slave_id = 0x00
	values = context[slave_id].getValues(3, 0, count=5)
	values = [v + 1 for v in values]
	log.debug("new values: " + str(values))
	context[slave_id].setValues(3, 0, values)


def run_updating_server():
	# ----------------------------------------------------------------------- #
	# initialize your data store
	# ----------------------------------------------------------------------- #

	store = ModbusSlaveContext(
		di=ModbusSequentialDataBlock(0, [10]*1000),
		co=ModbusSequentialDataBlock(0, [11]*1000),
		hr=ModbusSequentialDataBlock(0, [13]*1000),
		ir=ModbusSequentialDataBlock(0, [14]*1000))
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
	time = 5  # 5 seconds delay
	loop = LoopingCall(f=updating_writer, a=(context,))
	loop.start(time, now=False) # initially delay by time
	StartTcpServer(context, identity=identity, address=("localhost" , 5020))


if __name__ == "__main__":
	run_updating_server()
