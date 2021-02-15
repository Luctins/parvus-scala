#!/bin/python

"""
Evaluate response from tank by varying the output of the input valve control
signal - with PID controller
@author: Henrique T. Moresco, Henrique Wolf, Lucas M. Mendes, Matheus R. Willemann
"""

#TODO: implement async read and write to registers, also pack read and writes to get beter efficiency

#-------------------------------------------------------------------------------
# Library Imports

from pymodbus.client.sync import ModbusTcpClient
import sys
import time
from functools import reduce
from simple_pid import PID
import argparse as ap

#-------------------------------------------------------------------------------
# Constants
CLP_UNIT = 0x01

#input registers
INPUT_SP = 2
INPUT_LVL = 1

#control coils
CTL_STOP_START = 0
CTL_PAUSE = 1

#control registers
REG_IN_VALVE  = 0
REG_OUT_VALVE = 1

#-------------------------------------------------------------------------------
# Functions

#-----------------------------------------------------------
# Utilities

def verify_is_ip(arg):
	"""
	validate if an argument is an ip address
	"""
	return re.match('(([0-9]{1,3}\.){3})[0-9]{1,3}|localhost', arg)

#-----------------------------------------------------------
# System control

def get_setpoint(client):
	rq = client.read_input_registers(INPUT_SP, INPUT_SP, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)
	return rq.registers

def get_level_value(client):
	rq = client.read_input_registers(INPUT_LVL, INPUT_LVL, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)
	return rq.registers

def start_sym(client):
	rq = client.write_coil(CTL_STOP_START, 1, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def stop_sym(client):
	rq = client.write_coil(CTL_STOP_START, 0, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def pause_sym(client, pause=1):
	rq = client.write_coil(CTL_PAUSE, pause, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def unpause_sym(client):
	rq = client.write_coil(CTL_PAUSE, 0, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def write_in_valve(value, client):
	rq = client.write_register(REG_IN_VALVE, value, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def write_out_valve(value, client):
	rq = client.write_register(REG_OUT_VALVE, value, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)


def read_in_reg(client):
	""" Read all input registers """
	reg = client.read_input_registers(0, 2, unit=CLP_UNIT)
	assert(reg.function_code < 0x80)
	return reg

def all_is_same(vec):
	""" test that all elements in a vector are the same"""
	return all(el == vec[0] for el in vec)


def run_sym(client, contr, logf, _no_ctl=0, _continue_sim=0, T_scale = 4,\
			_T_step=0.25):
	"""
	Run simulation loop until system stabilizes
	"""

	T_step = _T_step/T_scale

	level = 0
	last_l = [ -1, -2, -3, -4, -5, -6, -7, -8, -9, -10]

	if not _continue_sym:
		#format CSV line and save to file
		line = "{:.4}, {}, {}, {}".format(0.0, 0.0, 0.0, 0.0)
		log.write((line+'\n').encode('utf-8'))

	start_time = time.time()
	stop_time = 0
	unpause_sym(client)
	if _no_ctl:
		while True:
			if (not stop_time) and all_is_same(last_l):
				stop_time = time.time()
				print("started counting timeout")
				#timeout
			if stop_time:
				if (time.time() - stop_time) > (800/T_scale):
					print("timeout reached")
					break

				#Read level sensor, fluid output
			r = read_in_reg(client)
			level, outflow = r.registers
			#TODO: measure deltat between updates

			#format CSV line and save to file
			line = "{:.4},\t{},\t{},\t{}"\
				.format(time.time() - start_time, level, outflow, in_valve)
			log.write((line+'\n').encode('utf-8'))

			# Append level to list
			last_l = last_l[1:] + [level]
			print(line, last_l, "stop_time: ", stop_time-time.time())
			#print(last_l)

			time.sleep(T_step)
	else:
		while False:
			#TODO: PID controller
			v = get_level_value(client)
			control = pid(v); control *= 100
			write_in_valve(control);

#TODO: save data to logfile and test end condition
#TODO: create never_stop mode, for testing (waits for keystroke to stop)

			pass

#-------------------------------------------------------------------------------
# Implementation

#--------------------------------------
# Set up variables

parser = ap.ArgumentParser(description='Parvus Scala')

parser.add_argument('--ip', type=ascii, help='Modbus server IP')

args = parser.parse_args()

if (not args.ip):
	print("missing dest ip address")
	sys.exit(-1)
else:
	if not verify_is_ip(args.ip):
		print('provided ip', args.ip, ' is invalid')
		sys.exit(-1)

#------------------------------
# PID Controller

pid = PID()
pid.setpoint = 5.0
pid.Ki = 1
pid.Kp = 2
pid.Kd = 0

#sys.exit(0)

#------------------------------
#modbus TCP Client

client = ModbusTcpClient(args.ip)
print('connected to:', args.ip)

#wait connection to be stablished
time.sleep(1)

#-----------------------------------------------------------
# Run simulation

# Reset registers
rq = client.write_coils(0, [False]*3, unit=CLP_UNIT)
assert(rq.function_code < 0x80)

print('reset coils')

print('opening logfile:',)
log = open(b'data_log.csv', 'ab')
line = "t, level, outflow, in_valve"
print(line)
log.write(line.encode('utf-8')+b'\n')

# (setup) Run with no controller until level stabilizes at 5
pause_sym(client); start_sym(client)
run_sym(client, None, log, no_ctl=1)
pause_sym(client)

#Step output valve and connect controller
write_out_valve(600, client)
pid.setpoint = 6.0
run_sym(client, pid, log, _continue_sim= 1)
stop_sym(client)

time.sleep(2)

log.close()
client.close()
