#!/bin/python
"""
Evaluate response from tank by varying the output of the input valve control
signal - with PID controller
@author: Henrique T. Moresco, Henrique Wolf, Lucas M. Mendes, Matheus R. Willemann
"""

#-------------------------------------------------------------------------------
# Library Imports

import sys
import os
import time
import argparse as ap
import simple_pid as s_pid
import PID
from pymodbus.client.sync import ModbusTcpClient
import re

#-----------------------------------------------------------
# Utilities

def verify_is_ip(arg):
	"""
	validate if an argument is an ip address
	"""
	return \
		re.match('([0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3})|localhost',\
				 arg)

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
pid = s_pid.PID()

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
system = PID(client, pid, logf)

# Reset registers
rq = client.write_coils(0, [False]*3, unit=CLP_UNIT)

assert(rq.function_code < 0x80)
print('reset coils')

pid.stop(client) #Clean previous state

system.pid.auto_mode = 1

system.run(client, pid, log, 0.5, 0.5, 0.6, _T_step=0.01, _end_sim=1, _no_stop=1)

time.sleep(2)

log.close()
client.close()
