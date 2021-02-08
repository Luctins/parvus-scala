#!/bin/python

"""
Evaluate response from tank by varying the output of the input  valve control
signal
@author: Henrique T. Moresco, Henrique Wolf, Lucas M. Mendes, Matheus R. Willemann
"""

###############################################################################
# Library Imports

from pymodbus.client.sync import ModbusTcpClient
import sys
import time
from functools import reduce

###############################################################################
# Functions
CLP_UNIT = 0x01

def start_sym(client):
	rq = client.write_coil(0, 1, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def stop_sym(client):
	rq = client.write_coil(0, 0, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def pause_sym(client):
	rq = client.write_coil(1, 1, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def unpause_sym(client):
	rq = client.write_coil(1, 0, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def write_in_valve(value, client):
	rq = client.write_register(0, value, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def write_out_valve(value, client):
	rq = client.write_register(1, value, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def read_in_reg(client):
	reg = client.read_input_registers(0, 2, unit=CLP_UNIT)
	assert(reg.function_code < 0x80)
	return reg

def all_is_same(vec):
	return all(el == vec[0] for el in vec)

def run_sym(client, in_valve, out_valve, logname, _stop_sym=0, _continue_sym = 0 ):
	timescale = 4
	timestep = 0.5/timescale

	print('opening logfile:', logname)

	log = open(logname, 'wb')
	line = "t, level, outflow, in_valve"
	print(line)
	log.write(line.encode('utf-8')+b'\n')

	level = 0
	last_l = [ -1, -2, -3, -4, -5, -6, -7, -8, -9, -10]

	write_in_valve(in_valve, client)
	write_out_valve(out_valve, client)

	if _continue_sym:
		unpause_sym(client)
	else:
		#format CSV line and save to file
		line = "{:.4}, {}, {}, {}".format(0.0, 0.0, 0.0, 0.0)
		log.write((line+'\n').encode('utf-8'))
		unpause_sym(client)
		start_sym(client)

	start_time = time.time()
	stop_time = 0
	while level != 1000:

		if not all_is_same(last_l) and not stop_time:
			stop_time = time.time()
			print("started counting timeout")

		if stop_time:
			if (time.time() - stop_time) > (800/timescale):
				print("timeout reached")
				break

		#Read level sensor, fluid output
		r = read_in_reg(client)
		if (r.function_code > 0x80):
			print("skip")
			continue
		level, outflow = r.registers

		#format CSV line and save to file
		line = "{:.4}, {}, {}, {}"\
			.format(time.time() - start_time, level, outflow, in_valve)
		log.write((line+'\n').encode('utf-8'))

		# Append level to list
		last_l = last_l[1:] + [level]
		print(line, last_l, "stop_time: ", stop_time-time.time())
		#print(last_l)

		time.sleep(timestep)

	if _stop_sym:
		stop_sym(client)
	else:
		pause_sym(client)

	log.close()

###############################################################################
# Implementation

#TODO: check that argv is a IP

print(sys.argv)
client = ModbusTcpClient(sys.argv[1])
print('client connected!')

time.sleep(1)

# Reset registers
rq = client.write_coils(0, [False]*3, unit=CLP_UNIT)
assert(rq.function_code < 0x80)

print('reset coils')

logname = "step_test.csv"
run_sym(client, 500, 500, b"start_step_test.csv", _stop_sym= 0)
run_sym(client, 600, 500, b"step_test.csv", _stop_sym= 1, _continue_sym= 1)

stop_sym(client)

time.sleep(3)

client.close()
