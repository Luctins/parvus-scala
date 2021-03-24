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
	return \
		re.match('([0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3})|localhost',\
				 arg)

#-----------------------------------------------------------
# System control

def try_modbus_ex(rq):
	try:
		assert(rq.function_code < 0x80)
	except AttributeError as e:
		pass

def get_setpoint(client):
	rq = client.read_input_registers(INPUT_SP, INPUT_SP, unit=CLP_UNIT)
	try_modbus_ex(rq)
	return rq.registers

def get_level_value(client):
	rq = client.read_input_registers(INPUT_LVL, INPUT_LVL, unit=CLP_UNIT)
	try_modbus_ex(rq)
	return rq.registers

def start_sim(client):
	rq = client.write_coil(CTL_STOP_START, 1, unit=CLP_UNIT)
	try_modbus_ex(rq)

def stop_sim(client):
	rq = client.write_coil(CTL_STOP_START, 0, unit=CLP_UNIT)
	try_modbus_ex(rq)

def pause_sim(client, pause=1):
	rq = client.write_coil(CTL_PAUSE, pause, unit=CLP_UNIT)
	try_modbus_ex(rq)

def unpause_sim(client):
	rq = client.write_coil(CTL_PAUSE, 0, unit=CLP_UNIT)
	try_modbus_ex(rq)

def write_in_valve(value, client):
	rq = client.write_register(REG_IN_VALVE, int(value), unit=CLP_UNIT)
	try_modbus_ex(rq)

def write_out_valve(value, client):
	rq = client.write_register(REG_OUT_VALVE, int(value), unit=CLP_UNIT)
	try_modbus_ex(rq)


def read_in_reg(client):
	"""
	Read all input registers
	"""
	r = client.read_input_registers(0, 4, unit=CLP_UNIT)
	assert(r.function_code < 0x80)
	return r.registers

def all_is_same(vec):
	"""
	Test that all elements in a vector are the same
	"""
	return all(el == vec[0] for el in vec)


def run_sim(client, contr, logf, setpoint, out_valve, in_valve, \
			_continue_sim=0, _end_sim=0, _no_stop=0, _read_sp=0, \
			T_scale=1, _T_step=0.01):
	"""
	Run simulation loop until system stabilizes
	"""

	def w_log(_t, _level, _of, _sp, _c, _iv, _dt, logf):
		"""
		Write to logfile in csv format
		"""
		l = [_level, _of, _sp, _c, _iv, _dt]
		line = "{:.4f}".format(_t)
		for v in l:
			line += ",\t{:0.4f}".format(v)
		logf.write((line+'\n').encode('utf-8'))
		return line

	def ap_l(l, n):
		"""
		Add to rolling list of last measurements
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
	tunings: {}
	setpoint: {}
	in_valve: {}
	out_valve: {}
	T_step: {}
	T_scale: {}
	""".format(contr.auto_mode, pid.tunings, setpoint, in_valve, out_valve, \
			   T_step, T_scale))

	pause_sim(client);

	#Print logfile header if not continuing simulation
	#Out valve is the control signal if controller connected
	if not _continue_sim:
		line = 't,level,outflow,out_valve,setpoint,in_valve,delta_t'
		print(line)
		log.write(line.encode('utf-8')+b'\n')

	last_l = [ -1, -2, -3, -4, -5, -6, -7, -8, -9, -10]
	last_t = time.time()
	level = 0
	dt = 0
	ret = out_valve #return value
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
			level, outflow, setpoint, T_scale = read_in_reg(client)

			#log data to file
			line = w_log(time.time() - start_time, level/V_OFS,\
						 outflow/V_OFS, out_valve, setpoint, in_valve,\
						 time.time() - last_t, logf=log)
			last_t = time.time()
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
			#time.sleep(T_step)

	else:
		contr.setpoint = setpoint;
		client.write_register(3, int(setpoint), unit=CLP_UNIT)
		contr.set_auto_mode(True, out_valve) #pass to controller last valve value
		last_t = time.time()
		c = 0
		while True:
			iter_t = time.time() #measure iteration time

			#get modbus registers
			level, outflow, setpoint, T_scale = read_in_reg(client)
			level /= V_OFS #scale down values from 0-1000 -> 0.0-1.0

			if _read_sp:
				if setpoint:
					contr.setpoint = setpoint/(V_OFS)
					client.write_register(3, int(setpoint), unit=CLP_UNIT)

			#Obtain control signal, also adjust delta time for Timescale
			dt = (time.time() - last_t)
			c = contr(level) #, dt= (dt if dt > 0.001 else 0.001))
			last_t = time.time()

			write_out_valve(int(c*V_OFS), client) #write control signal to sys

			#Write log data to file
			line = w_log(time.time() - start_time, level, outflow/V_OFS, c,\
						 contr.setpoint, in_valve, dt, logf=log)
			#Stop condition
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
			d = time.time() - iter_t
			print(line, "\t", last_l)
			#d = time.time() - iter_t #this is duplicated on purpose
			#time.sleep(T_step - d if d < T_step else 0.0) # delay at most T_step
		ret = c

	if _end_sim:
		stop_sim(client)
	else:
		pause_sim(client)

	#return the last control value, to be passed as a arg to the cont
	return ret


#-------------------------------------------------------------------------------
# Implementation

#--------------------------------------
# Set up variables

parser = ap.ArgumentParser(description='Parvus Scala')

parser.add_argument('ip', type=ascii, help='Modbus server IP')

args = parser.parse_args()

if (not args.ip):
	print("missing dest ip address")
	sys.exit
	(-1)
else:
	if not verify_is_ip(args.ip[1:-1]):
		print('provided ip', args.ip, ' is invalid')
		sys.exit(-1)

#------------------------------
# PID Controller
pid = PID()

p_tunings = (-41.0959, -0.0, -0.0 ) #(-22.5, -0.0, -0.0)
pi_tunings = (-12.7426, -1.453, -0.0)
pid_tunings = (-5, -1.517, -13.593 )

pid.tunings= pi_tunings
pid.sample_time= None
pid.output_limits=(0, 1)
pid.proportional_on_measurement=0
pid.bias = 0.0

# Open logfile
logdir = 'log'
logname = \
	'data_log_{}_P{:.3f}_I{:.3f}_D{:.3f}.csv'.format(int(time.time()),\
													 pid.Kp, pid.Ki, pid.Kd)

# Create 'log/' folder if it does not exist
if not os.path.exists('./'+logdir+'/'):
	os.mkdir(logdir)
	print('created logdir:', logdir)
log = open((logdir+'/'+logname).encode('utf8'), 'wb')
print('opened logfile:', logname)

#------------------------------
#modbus TCP Client
print(args.ip, args.ip.encode('utf8'))
client = ModbusTcpClient(args.ip[1:-1])
print('connected to:', args.ip)

#Wait connection to be stablished
time.sleep(0.5)

#-----------------------------------------------------------
# Run simulation

# Reset registers
rq = client.write_coils(0, [False]*3, unit=CLP_UNIT)

assert(rq.function_code < 0x80)
print('reset coils')

stop_sim(client) #Clean previous state

# (setup) Run with no controller until level stabilizes at 5
#pid.auto_mode = 0
#run_sim(client, pid, log, 0.0, 0.5, 0.5, _T_step=0.01, _end_sim=0)

pid.auto_mode = 1
run_sim(client, pid, log, 0.5, 0.5, 0.6, _T_step=0.01, _end_sim=1, _no_stop=1)

#last_v = run_sim(client, pid, log, 0.0, 0.5, 0.5, T_scale=4, _T_step=0.01)

#Step output valve and connect controller
#pid.auto_mode = 1
#run_sim(client, pid, log, 0.5, last_v, 0.6, _continue_sim=1, _end_sim=1, \
#		T_scale=1, _T_step=0.01, _no_stop=1, _read_sp=1)

time.sleep(2)

log.close()
client.close()
