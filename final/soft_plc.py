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
import plant
import re
import queue
from threading import Thread
#import simple_pid as s_pid
#from pymodbus.client.sync import ModbusTcpClient

#-------------------------------------------------------------------------------
#Constants

MAX_Q_LEN = 50

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

#parse command line arguments
parser = ap.ArgumentParser(description='Parvus Scala')
parser.add_argument('ip', type=ascii, help='Modbus server IP')
parser.add_argument('port', type=int, help='Modbus server port', required=0)
args = parser.parse_args()

if (not args.ip):
	print("missing dest ip address")
	sys.exit(-1)
else:
	args.ip = args.ip[1:-1]
	if not verify_is_ip(args.ip):
		print('provided ip', args.ip, ' is invalid')
		sys.exit(-1)

if (not args.port):
	args.port = 502;
	print("using default port:", args.port)

# Open logfile
logdir = 'log'
logname = \
	'data_log_{}_P{:.3f}_I{:.3f}_D{:.3f}.csv'.format(int(time.time()),\
													 pid.Kp, pid.Ki, pid.Kd)

# Create 'log/' folder if it does not exist
if not os.path.exists('./'+logdir+'/'):
	os.mkdir(logdir)
	print('created logdir:', logdir)
logf = open((logdir+'/'+logname).encode('utf8'), 'wb')
print('opened logfile:', logname)

#create control queues
ui_to_sys_q = queue.Queue(MAX_Q_LEN)
sys_to_ui_q = queue.Queue(MAX_Q_LEN)

#instatiate plant and PID controller
pid_p_tunings = (-41.0959, -0.0, -0.0 ) #(-22.5, -0.0, -0.0)
pi_tunings = (-12.7426, -1.453, -0.0)
pid_tunings = (-5, -1.517, -13.593 )

plant = plant(pi_tunings, (args.ip, args.port), logf, ui_to_sys_q, sys_to_ui_q)

plant_thr = Thread(target= plant.run, args=(0, 0, 0))

#TODO: instantiate modbus UI server Here
#and start plant thread with plant_thr.start()
