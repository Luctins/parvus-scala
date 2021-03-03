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
import os
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
	STOP_TIMEOUT = 1000
	#Voltage offset between modbus data ranges (int) 0-1000 and (float) 0.0-1.0
	V_OFS = DEC_OFS*TANK_MAX_LVL
	T_step = _T_step/T_scale #timestep adjusted for the timescale

	print("""
	Simulation parameters
	ctrl: {}
	setpoint: {}
	T_step: {}
	T_scale: {}
	""".format(contr.auto_mode, setpoint, T_step, T_scale))

	pause_sim(client);

	#Print logfile header if not continuing simulation
	#Out valve is the control signal if controller connected
	if not _continue_sim:
		line = 't,level,outflow,out_valve,setpoint,in_valve'
		print(line)
		log.write(line.encode('utf-8')+b'\n')

	last_l = [ -1, -2, -3, -4, -5, -6, -7, -8, -9, -10]
	level = 0
	ret = in_valve #return value
	stop_time = 0

	if _no_stop:
		print('running in continuos mode, press "q RET" to stop')

	#write initial values
	write_in_valve(int(in_valve*V_OFS), client)
	write_out_valve(int(out_valve*V_OFS), client)

	#start simulation and unpause
	start_sim(client)
	unpause_sim(client)

	start_time = time.time()
	#check if controller enabled, if not run until tank level stabilizes
	if not contr.auto_mode:
		while True:
			level, outflow = read_in_reg(client).registers

			#log data to file
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
		contr.set_auto_mode(True, out_valve) #pass to controller last level value
		last_t = time.time()
		while True:
			iter_t = time.time() #measure iteration time

			#get modbus registers
			level, outflow, setpoint = read_in_reg(client).registers
			level /= V_OFS #scale down values from 0-1000 -> 0.0-1.0

			if _read_sp:
				if setpoint:
					contr.setpoint = setpoint/(V_OFS)
					client.write_register(3, int(setpoint), unit=CLP_UNIT)

			#Obtain control signal, also adjust delta time for Timescale
			c = contr(level, dt=(time.time() - last_t)*T_scale)
			last_t = time.time()

			write_out_valve(int(c*V_OFS), client) #write control signal to sys

			#write log data to file
			line = w_log(time.time() - start_time, level, outflow/V_OFS, c,\
						 contr.setpoint, in_valve, logf=log)
			#stop cond
			if (not _no_stop):
				last_l = ap_l(last_l, int(level*DEC_OFS))
				if stop_time:
					if (time.time() - stop_time) > (STOP_TIMEOUT/T_scale):
						print("timeout reached, done")
						ret = c;
						break
				elif all_is_same(last_l):
					stop_time = time.time()
					print("started counting timeout")

			#Read char from console
			v = select.select([sys.stdin], [], [], 0)[0]
			if v:
				k = v[0].read(1)
				sys.stdin.flush()
				if k[0] == 'q':
					ret = c;
					break

			print(line, "dt: {}".format(int(d*1000)), "\t", last_l)
			d = time.time() - iter_t
			time.sleep(T_step - d if d < T_step else 0.0) # delay at most T_step

	if _stop_sim:
		stop_sim(client)
	else:
		pause_sim(client)
	return ret #return the last control value, to be passed as a arg to the controller


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

# Open logfile
logdir = 'log'
logname = 'data_log{}.csv'.format(int(time.time()))
if not os.path.exists('./'+logdir+'/'):
	os.mkdir(logdir)
	print('created logdir:', logdir)
log = open((logdir+'/'+logname).encode('utf8'), 'wb')
print('opened logfile:', logname)
sys.exit(0)
#------------------------------
# PID Controller
pid = PID()

pid.Kp= -2.0 #2.0
pid.Ki= -1.0 #1.0
pid.Kd= -0.0
pid.sample_time=None
pid.output_limits=(0,1)
pid.proportional_on_measurement=0

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


# (setup) Run with no controller until level stabilizes at 5
pid.auto_mode = 0
last_v = run_sim(client, pid, log, 0.0, 0.5, 0.5, T_scale=4, _T_step=0.01)

#Step output valve and connect controller
pid.auto_mode = 1
run_sim(client, pid, log, 0.5, last_v, 0.6, _stop_sim=1, T_scale=4, _T_step=0.01, \
		_no_stop=1, _read_sp=1)

time.sleep(2)

log.close()
client.close()
