#!/bin/python

"""
Evaluate response from tank by varying the output of the input valve control
signal - with PID controller
@author: Henrique T. Moresco, Henrique Wolf, Lucas M. Mendes, Matheus R. Willemann
"""

#TODO: implement async read and write to registers, also pack read and writes to get beter efficiency
#TODO: implement controller disable with auto mode
#-------------------------------------------------------------------------------
# Library Imports

from pymodbus.client.sync import ModbusTcpClient
import sys
import time
from functools import reduce
from simple_pid import PID
import argparse as ap
#from readkeys import getch
import select
import re

#-------------------------------------------------------------------------------
# Constants
CLP_UNIT = 0x01

#input registers
INPUT_LVL = 1
INPUT_SP = 2

#control coils
CTL_STOP_START = 0
CTL_PAUSE = 1

#control registers
REG_IN_VALVE  = 0
REG_OUT_VALVE = 1

DEC_OFS = 100 #decimal offset e.g: 101 == 1.01
TANK_MAX_LVL = 10

global ca
#-------------------------------------------------------------------------------
# Functions

#-----------------------------------------------------------
# Utilities

def verify_is_ip(arg):
	"""
	validate if an argument is an ip address
	"""
	return True#re.match('([0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3})|localhost', arg)

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

def start_sim(client):
	rq = client.write_coil(CTL_STOP_START, 1, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def stop_sim(client):
	rq = client.write_coil(CTL_STOP_START, 0, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def pause_sim(client, pause=1):
	rq = client.write_coil(CTL_PAUSE, pause, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def unpause_sim(client):
	rq = client.write_coil(CTL_PAUSE, 0, unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def write_in_valve(value, client):
	rq = client.write_register(REG_IN_VALVE, int(value), unit=CLP_UNIT)
	assert(rq.function_code < 0x80)

def write_out_valve(value, client):
	rq = client.write_register(REG_OUT_VALVE, int(value), unit=CLP_UNIT)
	assert(rq.function_code < 0x80)


def read_in_reg(client):
	""" Read all input registers """
	reg = client.read_input_registers(0, 3, unit=CLP_UNIT)
	assert(reg.function_code < 0x80)
	return reg

def all_is_same(vec):
	""" test that all elements in a vector are the same"""
	return all(el == vec[0] for el in vec)


def run_sim(client, contr, logf, setpoint, out_valve, in_valve, \
			_continue_sim=0, _stop_sim=0, _no_stop=0, _read_sp=0, \
			T_scale=4, _T_step=0.25):
	"""
	Run simulation loop until system stabilizes
	"""

	def w_log(_t, _level, _of, _sp, _c, _iv, logf):
		"""
		Write to logfile in csv format
		"""
		l = [_level, _of, _sp, _c, _iv]
		line = "{:.4f}".format(_t)
		for v in l:
			line += ",\t{:0.4f}".format(v)
		logf.write((line+'\n').encode('utf-8'))
		return line

	def ap_l(l, n):
		"""
		add to rolling list of last measurements
		"""
		return [n] + l[:-1]

	#Constants
	STOP_TIMEOUT = 100
	V_OFS = DEC_OFS*TANK_MAX_LVL
	T_step = _T_step/T_scale

	print("""
	Simulation parameters
	ctrl: {}
	setpoint: {}
	T_step: {}
	""".format(contr.auto_mode, setpoint, T_step))

	pause_sim(client);

	if not _continue_sim:
		line = 't,\tlevel,\toutflow, \tout_valve, \tsetpoint,,\tin_valve'
		print(line)
		log.write(line.encode('utf-8')+b'\n')

	last_l = [ -1, -2, -3, -4, -5, -6, -7, -8, -9, -10]
	stop_time = 0
	start_time = time.time()
	level = 0
	sp_unset = 1

	if _no_stop:
		print('running in continuos mode, press "q RET" to stop')

	write_in_valve(int(in_valve*V_OFS), client)
	write_out_valve(int(out_valve*V_OFS), client)

	start_sim(client)
	unpause_sim(client)


	#TODO: measure deltat between updates (modbus commands)

	#check if controller enabled, if not run until tank reaches 50% point
	if not contr.auto_mode:

		while True:
			level, outflow = read_in_reg(client).registers

			line = w_log(time.time() - start_time, level/V_OFS,\
						 outflow/V_OFS, out_valve, setpoint, in_valve,\
						 logf=log)
			last_l = ap_l(last_l, level)

			if stop_time:
				if (time.time() - stop_time) > (STOP_TIMEOUT/T_scale):
					print("timeout reached, done")
					break
			elif all_is_same(last_l):
				stop_time = time.time()
				print("started counting timeout")
				break

			print(line, last_l)
			time.sleep(T_step)

	else:
		contr.setpoint = setpoint;
		client.write_register(3, int(setpoint), unit=CLP_UNIT)
		while True:
			iter_t = time.time() #measure iteration time
			level, outflow, setpoint = read_in_reg(client).registers #get modbus registers
			if _read_sp:
				contr.setpoint = setpoint/V_OFS
				if contr.setpoint:
					sp_unset = 0
				if not sp_unset:
					client.write_register(3, int(setpoint), unit=CLP_UNIT)

			level /= V_OFS
			c = 1-contr(level) #obtain control value using input 0-1
			write_out_valve(int(c*V_OFS), client) #write to system

			line = w_log(time.time() - start_time, level, outflow/V_OFS, c,\
						 contr.setpoint, in_valve, logf=log)
			last_l = ap_l(last_l, int(level*DEC_OFS))

			#stop cond
			if (not _no_stop):
				if stop_time:
					if (time.time() - stop_time) > (STOP_TIMEOUT/T_scale):
						print("timeout reached, done")
						break
				elif all_is_same(last_l):
					stop_time = time.time()
					print("started counting timeout")

			#read char from console
			v = select.select([sys.stdin], [], [], 0)[0]
			if v:
				k = v[0].read(1)
				if k[0] == 'q':
					break
				sys.stdin.flush()

			d = time.time() - iter_t
			print(line, "dt: {}".format(int(d*1000)), "\t", last_l)
			time.sleep(T_step - d if d < T_step else 0.0) # delay at most T_step

	if _stop_sim:
		stop_sim(client);
	else:
		pause_sim(client)


#-------------------------------------------------------------------------------
# Implementation

#--------------------------------------
# Set up variables

parser = ap.ArgumentParser(description='Parvus Scala')

parser.add_argument('ip', type=ascii, help='Modbus server IP')

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

pid.Kp=2.0 #2.0
pid.Ki=2.0 #1.0
pid.Kd=0.2
pid.sample_time=None
pid.output_limits=(0.0,1.0)
pid.proportional_on_measurement=1

#sys.exit(0)

#------------------------------
#modbus TCP Client
print(args.ip, args.ip.encode('utf8'))
#client = ModbusTcpClient(args.ip)
client = ModbusTcpClient(args.ip[1:-1])
print('connected to:', args.ip)

#wait connection to be stablished
time.sleep(1)

#-----------------------------------------------------------
# Run simulation

# Reset registers
rq = client.write_coils(0, [False]*3, unit=CLP_UNIT)
assert(rq.function_code < 0x80)
print('reset coils')

stop_sim(client) #clean previous state

#Create Logfile
logname = 'data_log{}.csv'.format(int(time.time()))
log = open(logname.encode('utf8'), 'wb')
print('opened logfile:', logname)

# (setup) Run with no controller until level stabilizes at 5

#pid.auto_mode = 0
#run_sim(client, pid, log, 0.0, 5.0, 5.0, T_scale=1)
#Step output valve and connect controller

#time.sleep(10);

pid.auto_mode = 1
run_sim(client, pid, log, 0.5, 0.5, 0.6, _stop_sim=1, T_scale=1, _T_step=0.01, \
		_no_stop=1, _read_sp=1)

time.sleep(2)

log.close()
client.close()
