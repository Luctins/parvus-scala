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
#import select
import re
import queue
from enum import Enum, unique, auto

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

class plant(object):
	def __init__(self, _tunings, _dest_addr, _logf, _cmdq, _outq):
		"""
		Initialize PID Controller and Modbus connection
		"""
		# PID Controller
		print("PID tunings: ", _tunings)
		pid = PID()
		pid.tunings= _tunings
		pid.sample_time= None #TODO: use sample time
		pid.output_limits= (0, 1)
		pid.proportional_on_measurement= 0
		#pid.bias = 0.0
		self.pid = pid

		#modbus TCP Client
		self.client = ModbusTcpClient(host=_dest_addr[0], port=dest_addr[1])
		print('client connected to:', _dest_addr)

		#Logfile
		self.client.logf = _logf

		#TODO:check input parameters
		self.cmd_q = _cmdq
		self.out_q = _outq

	@unique
	class Command(Enum):
		AUTO_MODE = auto()
		TUNINGS = auto()
		SETPOINT = auto()
		STOP = auto()
		IN_VALVE = auto()
		OUT_VALVE = auto()

	@unique
	class Output(Enum):
		 TIME = auto()
		 LEVEL = auto()
		 OUTFLOW = auto()
		 IN_VALVE = auto()
		 OUT_VALVE = auto()
		 SETPOINT = auto()
		 DT = auto()

	def w_log(logf, data, _first=0):
		"""
		Write to logfile in csv format
		"""
		line = ''
		if _first:
			#line = 't, level, outflow, out_valve, in_valve, setpoint, delta_t'
			for k in data: line += '"' + str(K).split('.')[-1] +'",'
		else:
			for k in data: line += ",\t{:0.4f}".format(data[k]);
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

	def run(self, setpoint, out_valve, in_valve, \
			_continue_sim=0, _end_sim=0, T_scale=1, _T_step=0.01):
		"""
		Run simulation loop until system stabilizes
		"""

		#local "macros"
		append_l = lambda l, n: [n] + l[:-1]
		all_is_same = lambda vec: all(el == vec[0] for el in vec)

		#Constants
		STOP_TIMEOUT = 1000

		#Voltage offset between modbus data ranges (int) 0-1000 and (float) 0.0-1.0
		V_OFS = DEC_OFS*TANK_MAX_LVL
		T_step = _T_step/T_scale #timestep adjusted for the timescale

		pid = self.pid
		client = self.client
		logf = self.logf

		print("""
		Simulation parameters
		\tctrl: {}
		\ttunings: {}
		\tsetpoint: {}
		\tin_valve: {}
		\tout_valve: {}
		\tT_step: {}
		\tT_scale: {}
		""".format(contr.auto_mode, pid.tunings, setpoint, in_valve,\
				   out_valve, T_step, T_scale))

		pause_sim(client);
		res = { self.Output.TIME : 0,
				self.Output.LEVEL : 0,
				self.Output.OUTFLOW : 0,
				self.Output.OUT_VALVE : 0,
				self.Output.IN_VALVE : 0,
				self.Output.SETPOINT : 0,
				self.Output.DT : 0,
		}

		#Print logfile header if not continuing simulation
		if not _continue_sim:
			w_log(logf, res, _first=1)

		last_t = time.time(); level = 0; dt = 0
		stop_time = 0; contr.setpoint = setpoint;
		c = out_valve; last_c = None

		def _set(obj, arg):
			obj = arg
		resolve_cmd = {
			self.Command.AUTO_MODE : lambda arg: pid.set_auto_mode(True, arg) ,
			self.Command.SETPOINT  : lambda arg: _set(pid.setpoint, arg),
			self.Command.TUNINGS   : lambda arg: _set(pid.tunings, arg),
			self.Command.IN_VALVE  : lambda arg: self.write_in_valve(arg, self.client),
			self.Command.OUT_VALVE : lambda art: self.write_out_valve(arg, self.client)
		}

		#write initial values
		write_in_valve(int(in_valve*V_OFS), client)
		write_out_valve(int(c*V_OFS), client)

		# initialize controller with initial value
		if pid.auto_mode: pid.set_auto_mode(True, out_valve)

		#start simulation and unpause
		start_sim(client); unpause_sim(client)
		last_t = time.time(); start_time = last_t


		#Simulation Loop
		while True:
			#Read input values
			level, outflow, setpoint, T_scale = read_in_reg(client)
			level /= V_OFS #scale down values from 0-1000 -> 0.0-1.0

			#Obtain control signal, also adjust delta time for Timescale
			dt = (time.time() - last_t); last_t = time.time()

			#Read commands from queue
			try:
				cmd = self.cmd_q.get_nowait()
				if cmd[0] == self.Command.STOP:
					break
				elif cmd[0]:
					resolve_cmd[cmd[0]](cmd[1])
			except queue.Empty:
				pass

			#Test if running in closed/open loop
			if pid.auto_mode: c = pid(level)
			if c != last_c:
				write_out_valve(int(c*V_OFS), client)
				last_c = c

			res = { self.Output.TIME : time,
				self.Output.LEVEL : level,
				self.Output.OUTFLOW : outflow,
				self.Output.OUT_VALVE : setpoint,
				self.Output.IN_VALVE : in_valve,
				self.Output.SETPOINT : setpoint,
				self.Output.DT : dt,
			}
			self.w_log(res)
			try:
				self.out_q.put_nowait()
			except queue.Full:
				pass

			print(line)

			#TODO: use fixed sample time, preferrably inside the controller
			#d = time.time() - iter_t #this is duplicated on purpose
			#time.sleep(T_step - d if d < T_step else 0.0) # delay at most T_step

		if _end_sim:
			stop_sim(client)
		else:
			pause_sim(client)

		#return the last control value, to be passed as a arg to the cont
		return c
