#!/bin/python
"""
Control a atmosferic tank level with a PID controller, via a modbus client.
@author: Henrique T. Moresco, Henrique Wolf, Lucas M. Mendes, Matheus R. Willemann
"""

#-------------------------------------------------------------------------------
# Library Imports
import sys
import os
import time
import re
from enum import Enum, unique, auto
from multiprocessing import Queue
from functools import reduce
from pymodbus.client.sync import ModbusTcpClient
from simple_pid import PID
import logging

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

#decimal offset e.g: 101 == 1.01
DEC_OFS = 1000
TANK_MAX_LVL = 10

#Value offset between modbus data ranges (int) 0-1000 and (float) 0.0-1.0
V_OFS = DEC_OFS*TANK_MAX_LVL


class Plant():
	def __init__(self, _tunings, _dest_addr, _queues,
				 log_level=logging.DEBUG):
		"""
		Initialize PID Controller and Modbus connection
		"""
		#configure logging facility
		logging.basicConfig()
		self.log = logging.getLogger(__name__)
		self.log.setLevel(log_level)


		self.log.info("plant setup:\n")
		self.log.info("PID tunings: K_p:%f K_i:%f K_d:%f" % (_tunings[0],
															 _tunings[1],
															 _tunings[2]))
		# initialize PID Controller
		pid = PID()
		pid.tunings= _tunings
		pid.sample_time= None
		pid.output_limits= (0, 1) #limit output to positive values
		pid.proportional_on_measurement= 0
		pid.auto_mode = 1
		self.pid = pid #set pid controller object

		#initialize modbus TCP Client
		self.log.info('plant addr: {}'.format(_dest_addr))
		self.client = ModbusTcpClient(host=_dest_addr[0], port=_dest_addr[1])
		self.log.info('client connected')

		# Open Plant logfile
		logdir = 'log'
		logname = 'data_log_{}_P{:.3f}_I{:.3f}_D{:.3f}.csv'\
		.format(int(time.time()), pid.Kp, pid.Ki, pid.Kd)

		# Create 'log/' folder if it does not exist
		if not os.path.exists('./'+logdir+'/'):
		#if not os.path.exists('.\\'+logdir+'\\'): #for windows
			os.mkdir(logdir)
			self.log.info('created logdir: $s' % logdir)
		self.logf = open((logdir+'/'+logname).encode('utf8'), 'wb')
		self.log.info('opened logfile: %s' % logname)

		# store output and command queues
		self.out_q = _queues['out']
		self.in_q =  _queues['in']
		self.c = 0 #reset last control signal variable

	@unique
	class Command(Enum):
		"""
		Commands from the PLC
		"""
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
		DEC_OFS = auto()

	@unique
	class Output(Enum):
		"""
		Output values
		"""
		TIME = auto()
		LEVEL = auto()
		OUTFLOW = auto()
		IN_VALVE = auto()
		OUT_VALVE = auto()
		SETPOINT = auto()
		DT = auto()

	# class AutoModeEnabledException(Exception):
	#	"""
	#	raised if trying to set valves but controller is in auto mode
	#	"""
	#	pass

	def w_log(self, data, _first=0):
		"""
		Write data to logfile in csv format
		:param data   dict in the output format
		:param _first if 1 write the CSV header from the dict keys
		"""
		line = ''
		if _first:
			for k in data: line += '"' + str(k).split('.')[-1] +'",'
		else:
			for k in data.keys(): line += "{:0.4f}\t".format(data[k]);
		self.logf.write((line+'\n').encode('utf-8'))
		return line

	def try_modbus_ex(self, rq):
		"""
		Try to run a modbus command and catch exceptions
		"""
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

	def start(self, arg=1):
		"""
		Start the simulation
		"""
		if arg:
			rq = self.client.write_coil(CTL_STOP_START, 1, unit=CLP_UNIT)
			self.try_modbus_ex(rq)

	def stop(self, arg=1):
		"""
		Stop the simulation
		"""
		if arg:
			rq = self.client.write_coil(CTL_STOP_START, 0, unit=CLP_UNIT)
			self.try_modbus_ex(rq)

	def pause(self, arg=1, pause=1):
		"""
		Pause the simulation
		"""
		if arg:
			rq = self.client.write_coil(CTL_PAUSE, pause, unit=CLP_UNIT)
			self.try_modbus_ex(rq)

	def unpause(self):
		"""
		Unpause the simulation
		"""
		rq = self.client.write_coil(CTL_PAUSE, 0, unit=CLP_UNIT)
		self.try_modbus_ex(rq)

	def emergency(self, arg=0):
		"""
		Emergency Button actions
		"""
		self.pid.set_auto_mode(0) #disable PID controller
		#close all valves
		self.write_in_valve(0); self.write_out_valve(0)

	def write_in_valve(self, value):
		"""
		Write input valve value
		"""
		self.log.debug("in valve value: %i" % value)
		rq = self.client.write_register(REG_IN_VALVE, value, unit=CLP_UNIT)
		self.try_modbus_ex(rq) # test result

	def write_out_valve(self, value):
		"""
		Write output valve value
		"""
		self.log.debug("out valve value: %i" % value)
		rq = self.client.write_register(REG_OUT_VALVE, value, unit=CLP_UNIT)
		self.try_modbus_ex(rq) # test result

	def read_in_reg(self):
		"""
		Read all input registers
		"""
		r = self.client.read_input_registers(0, 4, unit=CLP_UNIT)
		assert(r.function_code < 0x80)
		return r.registers

	def set_kp(self, val):
		"""
		Setter for K_p
		"""
		self.pid.tunings = (
			(-1*val)/DEC_OFS,
			self.pid.tunings[1],
			self.pid.tunings[2]
			)
		self.log.info("new tunings: {}".format(self.pid.tunings))

	def set_ki(self, val):
		"""
		Setter for K_i
		"""
		self.pid.tunings = (
			self.pid.tunings[0],
			(-1*val)/DEC_OFS,
			self.pid.tunings[2]
			)
		self.log.info("new tunings: {}".format(self.pid.tunings))

	def set_kd(self, val):
		"""
		Setter for K_d
		"""
		self.pid.tunings = (
			self.pid.tunings[0],
			self.pid.tunings[1],
			(-1*val)/DEC_OFS
			)
		self.log.info("new tunings: {}".format(self.pid.tunings))

	def set_setpoint(self, val):
		"""
		Setter to be used with the command queue
		"""
		self.pid.setpoint = val/DEC_OFS
		self.log.info('new setpoint %.3f' % self.pid.setpoint)

	def set_out_valve(self, val):
		"""
		Setter to be used with the command queue
		"""
		t = int(val*TANK_MAX_LVL)
		self.c = val/DEC_OFS
		self.log.info('set_out_valve: val: %i c: %0.2f' % (t, self.c))
		if not self.pid.auto_mode:
			self.write_out_valve(t)
		else:
			self.log.error("auto mode is enabled")
			#raise self.AutoModeEnabledException

	def set_in_valve(self, val):
		"""
		Setter to be used with the command queue
		"""
		t = int(val*TANK_MAX_LVL)
		self.in_valve = val/DEC_OFS
		self.log.info('set_in_valve: val: %i inv: %0.3f' % (t, self.in_valve))
		self.write_in_valve(t)

	def run(self, setpoint, out_valve, in_valve, \
			_continue_sim=0, _end_sim=0, T_scale=1, _T_step=0.300):
		"""
		Main function,
		Run simulation loop with fixed time, outputting values to a queue and
		processing commands from a queue
		"""

		# def append_l(l, n):
		#	return [n] + l[:-1]

		#local functions
		append_l = lambda l, n: [n] + l[:-1] #append to a rolling list
		all_is_same = lambda vec: all(el == vec[0] for el in vec)
		def do_nothing(arg):
			pass

		#Constants
		STOP_TIMEOUT = 1000

		#intialize local variables
		T_step = _T_step/T_scale #timestep adjusted for the timescale
		pid = self.pid
		client = self.client
		logf = self.logf
		self.in_valve = in_valve
		self.pid.setpoint = setpoint

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
		# output object format
		res = {
			self.Output.TIME : 0,
			self.Output.LEVEL : 0,
			self.Output.OUTFLOW : 0,
			self.Output.OUT_VALVE : 0,
			self.Output.IN_VALVE : 0,
			self.Output.SETPOINT : 0,
			self.Output.DT : 0,
		}

		# mapping of commands to functions
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
			self.Command.DEC_OFS : do_nothing
		}

		#vector of command number
		cmd_values = self.Output.__members__.values()

		#Print logfile header if not continuing simulation
		if not _continue_sim:
			self.w_log(res, _first=1)



		#write initial values
		self.write_in_valve(int(in_valve*V_OFS))
		self.write_out_valve(int(c*V_OFS))

		# initialize controller with initial value
		if pid.auto_mode: pid.set_auto_mode(True, out_valve)

		#start simulation and unpause
		self.start(); self.unpause()
		last_t = time.time(); start_t = last_t

		#Initialize Simulation Loop variables
		level = 0
		stop_time = 0
		pid.setpoint = setpoint
		c = out_valve
		last_c = None
		dt = 0
		sleep_t = 0

		#simulation loop
		while True:
			last_t = time.time()

			#Read input values
			level, outflow, setpoint, T_scale = self.read_in_reg()
			level /= V_OFS #scale down values from 0-1000 -> 0.0-1.0

			#check if controller enabled/not enabled
			if pid.auto_mode:
				self.c = pid(level)
				if self.c != last_c:
					#write control signal
					self.write_out_valve(int(self.c*V_OFS))
					last_c = self.c

			#assemble output object
			res = {
				self.Output.TIME      : time.time() - start_t,
				self.Output.LEVEL     : level,
				self.Output.OUTFLOW   : outflow/V_OFS,
				self.Output.OUT_VALVE : self.c,
				self.Output.IN_VALVE  : self.in_valve,
				self.Output.SETPOINT  : self.pid.setpoint,
				self.Output.DT        : dt+sleep_t,
			}
			line = self.w_log(res) #write CSV log line
			#send to output queue
			if not self.out_q.full():
				self.out_q.put_nowait(res)
			else:
				self.log.error('plant: out queue is full')
			self.log.info(line)

			#try reading input commands
			if not self.in_q.empty():
				cmd, arg = self.in_q.get_nowait()
				self.log.debug("cmd: {} arg: {}".format(cmd, arg))
				if 1 or cmd in cmd_values:
					self.log.debug('action: {} arg:{}'.format(cmd_map[cmd], arg))
					cmd_map[cmd](arg)

			dt = (time.time() - last_t)
			#Delay at most T_step
			if dt < T_step:
				sleep_t = T_step - dt
				time.sleep(sleep_t)

		#used if the sym loop has a end condition, not used right now
		if _end_sim:
			self.stop()
		else:
			self.pause()


#------------------------------------------------------------------------------
# Main

if __name__ == '__main__':
	print('This file should not be called directly, but included from '
		  'soft_plc.py try running "python3 soft_plc.py -h" for more '
		  'information')
