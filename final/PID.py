#!/bin/python
"""
PID controller implementation that connecting to a modbus interface
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

class system(object):
	def __init__(self, _client, _contr, _logf):
		self.client = _client
		self.contr = _contr
		self.logf = _logf
		pass
	def w_log(logf, _time, _level, _outf, _control, _inv, _sp, _dt, _first=0):
		"""
		Write to logfile in csv format
		_level = tank level
		_outf = water flow
		_control = control signal (out valve)
		_inv = in valve
		_sp = setpoint
		_dt = delta t
		"""
		if _first:
			line = 't, level, outflow, out_valve, in_valve, setpoint, delta_t'
			print(line)
			log.write(line.encode('utf-8')+b'\n')
			return line
		else:
			l = [_level, _outf, _control, _inv, _sp, _dt]
			line = "{:.4f}".format(_time)
			for v in l:
				line += ",\t{:0.4f}".format(v)
				logf.write((line+'\n').encode('utf-8'))
				return line

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

	def start(client):
		rq = client.write_coil(CTL_STOP_START, 1, unit=CLP_UNIT)
		try_modbus_ex(rq)

	def stop(client):
		rq = client.write_coil(CTL_STOP_START, 0, unit=CLP_UNIT)
		try_modbus_ex(rq)

	def pause(client, pause=1):
		rq = client.write_coil(CTL_PAUSE, pause, unit=CLP_UNIT)
		try_modbus_ex(rq)

	def unpause(client):
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
	def run(self, setpoint, out_valve, in_valve, \
			_continue_sim=0, _end_sim=0, _no_stop=0, _read_sp=0, \
			T_scale=1, _T_step=0.01):
		"""
		Run simulation loop until system stabilizes
		"""

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

		contr = self.contr
		pid = self.pid
		client = self.client
		logf = self.logf

		print("""
		Simulation parameters
		 ctrl: {}
		 tunings: {}
		 setpoint: {}
		 in_valve: {}
		 out_valve: {}
		 T_step: {}
		 T_scale: {}
		""".format(contr.auto_mode, pid.tunings, setpoint, in_valve,\
				   out_valve, T_step, T_scale))

		pause_sim(client);

		#Print logfile header if not continuing simulation
		#the control signal is the same as out valve
		if not _continue_sim:
			w_log(logf, 0.0, 0.0, 0.0, out_valve, in_valve, \
				  self.contr.setpoint, 0.0, _first=1)

		last_l = [ -1, -2, -3, -4, -5, -6, -7, -8, -9, -10]
		last_t = time.time()
		level = 0
		dt = 0
		ret = out_valve #return value
		stop_time = 0
		contr.setpoint = setpoint;
		c = out_valve

		if _no_stop:
			print('running in continuos mode, press "q RET" to stop')

		#write initial values
		write_in_valve(int(in_valve*V_OFS), client)
		write_out_valve(int(out_valve*V_OFS), client)

		#start simulation and unpause
		start_sim(client)
		unpause_sim(client)

		#client.write_register(3, int(setpoint), unit=CLP_UNIT) #TODO: what is this?

		# initialize controller with previous value
		if contr.auto_mode:
			contr.set_auto_mode(True, out_valve)

		last_t = time.time()
		start_time = time.time()
		#Simulation Loop
		while True:
			level, outflow, setpoint, T_scale = read_in_reg(client)
			level /= V_OFS #scale down values from 0-1000 -> 0.0-1.0

			#Obtain control signal, also adjust delta time for Timescale
			dt = (time.time() - last_t)
			last_t = time.time()

			#test if running in closed/open loop
			if contr.auto_mode:
				c = contr(level) #; dt= (dt if dt > 0.001 else 0.001))
				write_out_valve(int(c*V_OFS), client) #write control signal to sys

				if _read_sp:
					if setpoint:
						contr.setpoint = setpoint/(V_OFS)
						client.write_register(3, int(setpoint), unit=CLP_UNIT)

				#TODO: use fixed sample time
				#d = time.time() - iter_t #this is duplicated on purpose
				#time.sleep(T_step - d if d < T_step else 0.0) # delay at most T_step

			last_l = ap_l(last_l, int(level*V_OFS))

			#stop condition
			if (not _no_stop):
				if stop_time:
					if (time.time() - stop_time) > (STOP_TIMEOUT/T_scale):
						print("timeout reached, done")
						break
				elif all_is_same(last_l):
					stop_time = time.time()
					print("started counting timeout")
					#break
			else:
				#Read char from console
				v = select.select([sys.stdin], [], [], 0)[0]
				if v:
					k = v[0].read(1); sys.stdin.flush()
					if k[0] == 'q':
						break

			#d = time.time() - iter_t
			print(line, "\t", last_l)
			#time.sleep(T_step)

		ret = c
		if _end_sim:
			stop_sim(client)
		else:
			pause_sim(client)

		#return the last control value, to be passed as a arg to the cont
		return ret
