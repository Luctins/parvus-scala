#!/bin/python
"""
PID controller implementation that connecting to a modbus interface
"""

#TODO: implement async read and write to registers, also pack read and writes to get beter efficiency
#TODO: implement controller disable with auto mode
#-------------------------------------------------------------------------------
# Library Imports

import sys
import os
import time
import re
from enum import Enum, unique, auto
from multiprocessing import Queue
from functools import reduce

import logging
from pymodbus.client.sync import ModbusTcpClient
from simple_pid import PID


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

class Plant():
	def __init__(self, _tunings, _dest_addr, _queues,
				 log_level=logging.DEBUG):
		"""
		Initialize PID Controller and Modbus connection
		"""
		logging.basicConfig()
		print("__name__ %s" % __name__ )
		self.log = logging.getLogger(__name__)
		self.log.setLevel(log_level)

		print('self.log: ', self.log)
		self.log.debug("test")

		print("plant setup:\n")
		# PID Controller
		print("PID tunings: ", _tunings)
		pid = PID()
		pid.tunings= _tunings
		pid.sample_time= None #TODO: use sample time
		pid.output_limits= (0, 1)
		pid.proportional_on_measurement= 0
		pid.auto_mode = 0
		#pid.bias = 0.0
		self.pid = pid

		#modbus TCP Client
		print('plant addr:', _dest_addr)
		self.client = ModbusTcpClient(host=_dest_addr[0], port=_dest_addr[1])
		print('client connected')
		# Open Plant logfile
		logdir = 'log'
		logname = 'data_log_{}_P{:.3f}_I{:.3f}_D{:.3f}.csv'\
		.format(int(time.time()), pid.Kp, pid.Ki, pid.Kd)

		# Create 'log/' folder if it does not exist
		if not os.path.exists('./'+logdir+'/'):
			os.mkdir(logdir)
			print('created logdir:', logdir)
		self.logf = open((logdir+'/'+logname).encode('utf8'), 'wb')
		print('opened logfile:', logname)

		self.out_q = _queues['out']
		self.in_q =  _queues['in']

	@unique
	class Command(Enum):
		STOP = auto()
		START = auto()
		EMERGENCY = auto()
		AUTO_MODE = auto()
		TUNINGS = auto()
		SETPOINT = auto()
		IN_VALVE = auto()
		OUT_VALVE = auto()
		SET_K_P = auto()
		SET_K_I = auto()
		SET_K_D = auto()

	@unique
	class Output(Enum):
		 TIME = auto()
		 LEVEL = auto()
		 OUTFLOW = auto()
		 IN_VALVE = auto()
		 OUT_VALVE = auto()
		 SETPOINT = auto()
		 DT = auto()

	class AutoModeEnabledException(Exception):
		"""
		raised if trying to set valves but controller is in auto mode
		"""
		pass

	def w_log(self, data, _first=0):
		"""
		Write to logfile in csv format
		"""
		line = ''
		if _first:
			#line = 't, level, outflow, out_valve, in_valve, setpoint, delta_t'
			for k in data: line += '"' + str(k).split('.')[-1] +'",'
		else:
			for k in data.keys(): line += "{:0.4f}\t".format(data[k]);
		self.logf.write((line+'\n').encode('utf-8'))
		return line

	def try_modbus_ex(self, rq):
		try:
			assert(rq.function_code < 0x80)
		except AttributeError as e:
			pass

	def get_setpoint(self):
		rq = self.client.read_input_registers(INPUT_SP, INPUT_SP, unit=CLP_UNIT)
		self.try_modbus_ex(rq)
		return rq.registers

	def get_level_value(self):
		rq = self.client.read_input_registers(INPUT_LVL, INPUT_LVL, unit=CLP_UNIT)
		self.try_modbus_ex(rq)
		return rq.registers

	def start(self):
		rq = self.client.write_coil(CTL_STOP_START, 1, unit=CLP_UNIT)
		self.try_modbus_ex(rq)

	def stop(self):
		rq = self.client.write_coil(CTL_STOP_START, 0, unit=CLP_UNIT)
		self.try_modbus_ex(rq)

	def pause(self, pause=1):
		rq = self.client.write_coil(CTL_PAUSE, pause, unit=CLP_UNIT)
		self.try_modbus_ex(rq)

	def unpause(self):
		rq = self.client.write_coil(CTL_PAUSE, 0, unit=CLP_UNIT)
		self.try_modbus_ex(rq)

	def write_in_valve(self, value):
		rq = self.client.write_register(REG_IN_VALVE, int(value), unit=CLP_UNIT)
		self.try_modbus_ex(rq)

	def write_out_valve(self, value):
		rq = self.client.write_register(REG_OUT_VALVE, int(value), unit=CLP_UNIT)
		self.try_modbus_ex(rq)

	def read_in_reg(self):
		"""
		Read all input registers
		"""
		r = self.client.read_input_registers(0, 4, unit=CLP_UNIT)
		assert(r.function_code < 0x80)
		return r.registers

	def set_kp(self, val):
		self.pid.tunings[0] = val;

	def set_ki(self, val):
		self.pid.tunings[2] = val;

	def set_kd(self, val):
		self.pid.tunings[3] = val;

	def set_setpoint(self, val):
		self.pid.setpoint = val

	def set_out_valve(self, val):
		if not self.pid.auto_mode:
			self.write_out_valve(val)
		else:
			self.log.error("auto mode is enabled")
			#raise self.AutoModeEnabledException
	def set_in_valve(self, val):
		self.write_in_valve(val)
		# if not self.pid.auto_mode:
		# else:
		#	raise self.AutoModeEnabledException

	def emergency(self):
		"""
		Emergency Button Actions
		"""
		self.stop()
		self.pid.set_auto_mode(0)
		self.write_in_valve(0); self.write_out_valve(1)

	def run(self, setpoint, out_valve, in_valve, \
			_continue_sim=0, _end_sim=0, T_scale=1, _T_step=0.300):
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

		self.log.info("""
		Simulation initial parameters
		\tctrl: {}
		\ttunings: {}
		\tsetpoint: {}
		\tin_valve: {}
		\tout_valve: {}
		\tT_step: {}
		\tT_scale: {}
		""".format(pid.auto_mode, pid.tunings, setpoint, in_valve,\
				   out_valve, T_step, T_scale))

		self.pause();
		res = {
			self.Output.TIME : 0,
			self.Output.LEVEL : 0,
			self.Output.OUTFLOW : 0,
			self.Output.OUT_VALVE : 0,
			self.Output.IN_VALVE : 0,
			self.Output.SETPOINT : 0,
			self.Output.DT : 0,
		}

		cmd_map = {
			self.Command.STOP : self.stop ,
			self.Command.START : self.start ,
			self.Command.EMERGENCY : self.emergency ,
			self.Command.AUTO_MODE : self.pid.set_auto_mode ,
			#self.Command.TUNINGS : self.set_kp ,
			self.Command.SETPOINT : self.set_setpoint ,
			self.Command.IN_VALVE : self.set_in_valve ,
			self.Command.OUT_VALVE : self.set_out_valve ,
			self.Command.SET_K_P : self.set_kp ,
			self.Command.SET_K_I : self.set_ki ,
			self.Command.SET_K_D : self.set_kd ,
		}

		#vector of command numbers
		cmd_values = self.Output.__members__.values()
		#[v.value for v in self.Output.__members__.values()]

		#Print logfile header if not continuing simulation
		if not _continue_sim:
			self.w_log(res, _first=1)

		last_t = time.time(); level = 0; dt = 0
		stop_time = 0; pid.setpoint = setpoint;
		c = out_valve; last_c = None

		#write initial values
		self.write_in_valve(int(in_valve*V_OFS))
		self.write_out_valve(int(c*V_OFS))

		# initialize controller with initial value
		if pid.auto_mode: pid.set_auto_mode(True, out_valve)

		#start simulation and unpause
		self.start(); self.unpause()
		last_t = time.time(); start_t = last_t

		#Simulation Loop
		dt = 0;
		sleep_t = 0;
		while True:
			last_t = time.time()
			#Read input values
			level, outflow, setpoint, T_scale = self.read_in_reg()
			level /= V_OFS #scale down values from 0-1000 -> 0.0-1.0

			#Test if running in closed/open loop
			if pid.auto_mode: c = pid(level)
			if c != last_c:
				self.write_out_valve(int(c*V_OFS))
				last_c = c

			res = {
				self.Output.TIME : time.time() - start_t,
				self.Output.LEVEL : level,
				self.Output.OUTFLOW : outflow,
				self.Output.OUT_VALVE : setpoint,
				self.Output.IN_VALVE : in_valve,
				self.Output.SETPOINT : setpoint,
				self.Output.DT : dt+sleep_t,
			}
			line = self.w_log(res)
			if not self.out_q.full():
				self.out_q.put_nowait(res)
			else:
				self.log.error('plant: out queue is full')
			self.log.info(line)

			if not self.in_q.empty():
				cmd, arg = self.in_q.get_nowait()
				self.log.debug("cmd: {} arg: {}".format(cmd, arg))
				if 1 or cmd in cmd_values:
					print('ok')
					if arg == None:
						cmd_map[cmd]()
					else:
						cmd_map[cmd](arg)
					print(cmd_map[cmd])

			dt = (time.time() - last_t);
			#print('dt: %f T_step: %f' % ( dt, T_step ))
			#Delay at most T_step
			if dt < T_step:
				sleep_t = T_step - dt
				time.sleep(sleep_t)

		if _end_sim:
			self.stop()
		else:
			self.pause()

		#return the last control value, to be passed as a arg to the cont
		return c
