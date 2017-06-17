#!/usr/bin/env python

__author__ = 'mtomic'

import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from mav_msgs.msg import MotorSpeed
from std_msgs.msg import Float32, Float64
import math
from math import cos, sin, tan, tanh, pi, atan2, asin
from rosgraph_msgs.msg import Clock
import tf
import numpy as np
import matplotlib.pyplot as plt



class SMC_basic_ctl(object):
	"""
	This class implements basic SMC control for pitch and roll angles. 

	Subscribes to:
		/arducopter/imu
		/arducopter/sensors/pose1
		/arducopter/euler_ref
		/arducopter/pose_ref

	Publishes:
		/arducopter/sigma
		/arducopter/euler_angles
		/arducopter/movable_mass_0_position_controller/command
		/arducopter/movable_mass_1_position_controller/command
		/arducopter/movable_mass_2_position_controller/command
		/arducopter/movable_mass_3_position_controller/command

	"""

	def __init__(self):
		
		# MASA
		self.m = 0.208
		self.mq = 2.083
		self.M = self.mq + (4 * self.m)

		# DIMENZIJE
		self.lm = 0.17
		self.arm_offset = 0.08
		self.l_rot = 0.265
		self.l = self.lm + self.arm_offset

		# MOMENTI INERCIJE
		# Body withous masses
		self.Iq_xx = 0.0827
		self.Iq_yy = 0.0827
		self.Iq_zz = 0.0104

		# Identified moment of inertia of the entire quad
		self.I_yy = 0.0857

		# OSTALI PARAMETRI
		self.g = 9.81

		# Q filter
		self.T_dob1 = 0.3
		self.T_dob2 = 0.2
		self.D = 5

		# Ref prefilter
		self.T_f1 = 0.17
		self.T_f2 = 0.17

		# SMC
		self.smc_P = 0.2
		self.smc_I = 35
		self.smc_D = 0.1
		self.lam = 40.0
		self.eta = 0.25
		self.tanh_n = 20.0

		# Matlab params
		"""self.smc_P = 0.2
		self.smc_I = 10
		self.smc_D = 0.1
		self.lam = 40.0
		self.eta = 0.5
		self.tanh_n = 20"""

		# Sigma
		self.sigma_I_old_p = 0
		self.sigma_I_old_r = 0
		self.sigma_pitch = 0
		self.sigma_roll = 0

		# Other
		self.clock = Clock()
		self.t_old = 0
		self.start_flag = False
		self.euler_angles = Vector3()
		self.euler_rates = Vector3()

		# Prefilter
		self.uk_pitch = 0				# u(k-1)
		self.ukk_pitch = 0				# u(k-2)
		self.yk_pitch = 0				# y(k-1)
		self.ykk_pitch = 0				# y(k-2)
		self.y_pitch = [0]				# y(k)
		self.yd_pitch = [0]				# y'
		self.ydd_pitch = [0]			# y''
		self.t = []
		self.br = 0

		self.uk_roll = 0				# u(k-1)
		self.ukk_roll = 0				# u(k-2)
		self.yk_roll = 0				# y(k-1)
		self.ykk_roll = 0				# y(k-2)
		self.y_roll = [0]				# y(k)
		self.yd_roll = [0]				# y'
		self.ydd_roll = [0]				# y''



		self.ref_pitch = 0				# REF
		self.ref_roll = 0				# REF
		self.ref_z = 0
		self.pose_z = 0

		self.Ts = 0.01

		self.lpp = 0
		self.lpdp = 0
		self.lpddp = 0

		self.lpr = 0
		self.lpdr = 0
		self.lpddr = 0

		

		


		#### SUBSCRIBERS ####
		rospy.Subscriber('/arducopter/imu', Imu, self.imu_cb)
		rospy.Subscriber('/clock', Clock, self.clock_cb)
		rospy.Subscriber('/arducopter/euler_ref', Vector3, self.ref_cb)
		rospy.Subscriber('/arducopter/sensors/pose1', PoseWithCovarianceStamped, self.pose_cb)
		rospy.Subscriber('/arducopter/pos_ref', Vector3, self.pose_ref_cb)

		#### PUBLISHERS ####
		self.pub_mass0 = rospy.Publisher('/arducopter/movable_mass_0_position_controller/command', Float64, queue_size = 1)
		self.pub_mass1 = rospy.Publisher('/arducopter/movable_mass_1_position_controller/command', Float64, queue_size = 1)
		self.pub_mass2 = rospy.Publisher('/arducopter/movable_mass_2_position_controller/command', Float64, queue_size = 1)
		self.pub_mass3 = rospy.Publisher('/arducopter/movable_mass_3_position_controller/command', Float64, queue_size = 1)

		self.pub_ref = rospy.Publisher('/arducopter/ref', Vector3, queue_size = 1)
		self.pub_ref_d = rospy.Publisher('/arducopter/refd', Vector3, queue_size = 1)
		self.pub_ref_dd = rospy.Publisher('/arducopter/refdd', Vector3, queue_size = 1)
		self.s_pub = rospy.Publisher('/arducopter/sigma', Float64, queue_size = 1)
		self.dx_pub = rospy.Publisher('/arducopter/dx', Float64, queue_size = 1)
		self.euler_ang = rospy.Publisher('arducopter/euler_angles', Vector3, queue_size = 1)
		self.motor_cmd = rospy.Publisher('/arducopter/command/motors', MotorSpeed, queue_size = 1)

		#rospy.sleep(0.01)



	def run(self):
	
		while rospy.get_time() == 0:
			print 'Waiting for clock server to start...'

		print 'First clock message received.'


		while not self.start_flag:
			print 'Waiting for the first measurement...'
			rospy.sleep(0.5)

		print 'Starting attitude control.'

		clock_old = self.clock


		while not rospy.is_shutdown():
			Ts = 0.01
			rospy.sleep(Ts)

			# TIME
			clock_now_pitch = self.clock
			dt = (clock_now_pitch.clock - clock_old.clock).to_sec()
			clock_old = clock_now_pitch
			
			dt = Ts

			# ANGLES
			pitch_m = self.euler_angles.y
			pitch_r = self.euler_rates.y
			ref_pitch = self.ref_pitch

			roll_m = self.euler_angles.x
			roll_r = self.euler_rates.x
			ref_roll = self.ref_roll


			###################
			###### PITCH ######
			###################


			# PUBLISHER REFERENCE - ZA TESTIRANJE
			ref_msg = Vector3()
			ref_msg.y = self.y_pitch[self.lpp-1]
			self.pub_ref.publish(ref_msg)

			#ref_msg = Vector3()
			#ref_msg.y = self.yd_pitch[lpd-1]
			#self.pub_ref_d.publish(ref_msg)

			#ref_msg = Vector3()
			#ref_msg.y = self.ydd_pitch[lpdd-1]
			#self.pub_ref_dd.publish(ref_msg)

			# Sigma
			[self.sigma_I_old_p, self.sigma_pitch] = self.sigma(self.y_pitch[self.lpp-1], self.yd_pitch[self.lpdp-1], pitch_m, pitch_r, dt, self.sigma_I_old_p)

			# PUBLISHER SIGMA - ZA TESTIRANJE
			s_msg = Float64()
			s_msg.data = self.sigma_pitch
			self.s_pub.publish(s_msg)

			# Control signal

			dx_pitch = self.control_smc(self.sigma_pitch, self.y_pitch[self.lpp-1], self.yd_pitch[self.lpdp-1], self.ydd_pitch[self.lpddp-1], pitch_m, self.tanh_n)

			#print dx_pitch
			dx_msg = Float64()
			dx_msg.data = dx_pitch
			self.dx_pub.publish(dx_msg)



			###################
			######  ROLL ######
			###################


			# Sigma
			[self.sigma_I_old_r, self.sigma_roll] = self.sigma(self.y_roll[self.lpr-1], self.yd_roll[self.lpdr-1], roll_m, roll_r, dt, self.sigma_I_old_r)

			# Control signal

			dy_roll = self.control_smc(self.sigma_roll, self.y_roll[self.lpr-1], self.yd_roll[self.lpdr-1], self.ydd_roll[self.lpddr-1], roll_m, self.tanh_n)


			# AKO SE NALAZIMO NA PODU ZAKOCI MASE - DA SE NE KRECE U POCETNOM TRENUTKU
			if self.pose_z <= 0.13:
				dx_pitch = 0.0
				dy_roll = 0.0

			# SLIJETANJE
			if self.ref_z == 0.0 and self.pose_z <= 0.13:
				motors_msg = MotorSpeed()
				motors_msg.motor_speed = [0.0, 0.0, 0.0, 0.0]
				self.motor_cmd.publish(motors_msg)

			# Publishing mass positions
			mass0_cmd_msg = Float64()
			mass0_cmd_msg.data = dx_pitch

			mass2_cmd_msg = Float64()
			mass2_cmd_msg.data = -dx_pitch

			mass1_cmd_msg = Float64()
			mass1_cmd_msg.data = -dy_roll

			mass3_cmd_msg = Float64()
			mass3_cmd_msg.data = dy_roll

			self.pub_mass0.publish(mass0_cmd_msg)
			self.pub_mass1.publish(mass1_cmd_msg)
			self.pub_mass2.publish(mass2_cmd_msg)
			self.pub_mass3.publish(mass3_cmd_msg)

			angle_msg = Vector3()
			angle_msg.x = self.euler_angles.x
			angle_msg.y = self.euler_angles.y
			angle_msg.z = self.euler_angles.z
			self.euler_ang.publish(angle_msg)




	# USER FUNCTIONS
	# Prefiltar
	def ref_pref(self, uk, ukk, yk, ykk):
		return 0.001663730036015*uk + 0.001599747657011*ukk + 1.885746287709750*yk - 0.889009765402775*ykk

	# Aproksimacija prve derivacije
	def dy(self, y, Ts):
		l = len(y)
		return (y[l-1] - y[l-2]) / Ts

	# Aproksimacija druge derivacije
	def ddy(self, y, Ts):
		l = len(y)
		return (y[l-1] - 2*y[l-2] + y[l-3]) / Ts**2

	def dead_zone(self, signal, tol):
		if signal > -tol and signal < tol:
			return 0
		else:
			return signal

	def satur(self, u):
		lim_high = self.lm/2
		lim_low = -lim_high

		if u > lim_high:
			return lim_high
		elif u < lim_low:
			return lim_low
		else:
			return u


	def sigma(self, ref, ref_d, angle_m, angle_r, dt, sigma_I_old):
		sigma = 0
		sigma_P = 0
		sigma_I = 0
		sigma_D = 0
		tol = 0.001

		e = ref - angle_m
		de = ref_d - angle_r

		# Proportional term
		sigma_P = e * self.smc_P * self.Iq_yy**-1		

		# Integral term
		if self.smc_I == 0:
			sigma_I = 0
		else:
			sigma_I = sigma_I_old + self.smc_I * e * dt

		sigma_I_old = sigma_I

		# Derivative term
		sigma_D = self.smc_D * de / dt

		sigma = sigma_P + sigma_I + sigma_D
		

		

		self.br += 1
		if self.br % 30 == 0:
			#print sigma
			pass


		sigma = self.dead_zone(sigma, tol)


		# Sigma
		return sigma_I_old, sigma


	def control_smc(self, sigma, ref, ref_d, ref_dd, angle_m, n):
		e = ref - angle_m

		u_nom = 0
		u_ctl_vector = 0
		u_tmp = 0

		# This part of the controller uses nominal model of the system in 
		# order to design a stabilizing controller

		u_nom = (e * self.lam + ref_dd) * self.Iq_yy + ref_d * self.smc_P
		u_ctl_vector = self.eta * tanh(sigma)

		

		u_tmp = (u_nom + u_ctl_vector) / (2 * self.m * self.g)

		return self.satur(u_tmp)



	# CALLBACK FUNCTIONS
	def imu_cb(self, msg):

		""" IMU callback function, used to extract roll and pitch angles, along with their rates.
		"""

		if not self.start_flag:
			self.start_flag = True

		qx = msg.orientation.x
		qy = msg.orientation.y
		qz = msg.orientation.z
		qw = msg.orientation.w

		quaternion = (
			qx,
			qy,
			qz,
			qw)

		euler = tf.transformations.euler_from_quaternion(quaternion)

		self.euler_angles.x = euler[0] #* pi/180			# ROLL
		self.euler_angles.y = euler[1] #* pi/180			# PITCH
		self.euler_angles.z = euler[2] #* pi/180			# YAW

		# gyro
		p = msg.angular_velocity.x
		q = msg.angular_velocity.y
		r = msg.angular_velocity.z

		sx = sin(self.euler_angles.x)
		cx = cos(self.euler_angles.x)
		cy = cos(self.euler_angles.y)
		ty = tan(self.euler_angles.y)

		# gyro measurements to rates
		self.euler_rates.x = p + sx * ty * q + cx * ty * r
		self.euler_rates.y = cx * q - sx * r
		self.euler_rates.z = sx / cy * q + cx / cy * r

	def clock_cb(self, msg):
		self.clock = msg

	def ref_cb(self, msg):
		self.ref_roll = msg.x
		self.ref_pitch = msg.y

		# Filtriranje reference
		# Referencu spremam u listu radi urednosti, 
		# kasnije pri proracunu koristim zadnju spremljenu vrijdnost

		### PITCH ###

		self.y_pitch.append(self.ref_pref(self.uk_pitch, self.ukk_pitch, self.yk_pitch, self.ykk_pitch))
		self.yd_pitch.append(self.dy(self.y_pitch, self.Ts))

		if len(self.y_pitch) > 1:
			self.ydd_pitch.append(self.ddy(self.y_pitch, self.Ts))

		# Azuriranje stanja
		self.lpp = len(self.y_pitch)
		self.lpdp = len(self.yd_pitch)
		self.lpddp = len(self.ydd_pitch)

		self.ukk_pitch = self.uk_pitch
		self.uk_pitch = self.ref_pitch
		self.ykk_pitch = self.yk_pitch
		self.yk_pitch = self.y_pitch[self.lpp-1]	

		### ROLL ###

		self.y_roll.append(self.ref_pref(self.uk_roll, self.ukk_roll, self.yk_roll, self.ykk_roll))
		self.yd_roll.append(self.dy(self.y_roll, self.Ts))

		if len(self.y_roll) > 1:
			self.ydd_roll.append(self.ddy(self.y_roll, self.Ts))

		# Azuriranje stanja
		self.lpr = len(self.y_roll)
		self.lpdr = len(self.yd_roll)
		self.lpddr = len(self.ydd_roll)

		self.ukk_roll = self.uk_roll
		self.uk_roll = self.ref_roll
		self.ykk_roll = self.yk_roll
		self.yk_roll = self.y_roll[self.lpr-1]



	def pose_cb(self, msg):
		self.pose_z = msg.pose.pose.position.z

	def pose_ref_cb(self, msg):
		self.ref_z = msg.z


if __name__ == '__main__':

	rospy.init_node('movable_mass_ctl')
	mm_smc = SMC_basic_ctl()
	mm_smc.run()

	

