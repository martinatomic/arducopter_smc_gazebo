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
		self.lm = 0.16
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
		# T = 0.17
		"""self.smc_P = 0.2
		self.smc_I = 35
		self.smc_D = 0.1
		self.lam = 40.0
		self.eta = 0.25
		self.tanh_n = 20.0"""

		# T = 0.2
		"""self.smc_P = 0.7
		self.smc_I = 25
		self.smc_D = 0.1
		self.lam = 15.0
		self.eta = 0.145
		self.tanh_n = 20"""

		# T = 0.3
		"""self.smc_P = 0.7 #0.5
		self.smc_I = 25 #30
		self.smc_D = 0.1 #0.1
		self.lam = 10.0 #0.15
		self.eta = 0.145
		self.tanh_n = 50"""

		"""self.smc_P = 0.2 #0.5
		self.smc_I = 25 #30
		self.smc_D = 0.1 #0.1
		self.lam = 15 #0.15
		self.eta = 0.1055
		self.tanh_n = 100"""
		# FINFINFIN
		self.smc_P = 0.5 #0.5
		self.smc_I = 30 #30
		self.smc_D = 0.1 #0.1
		self.lam = 35 #0.15
		self.eta = 0.255
		self.tanh_n = 100

		# Matlab params
		"""self.smc_P = 0.2
		self.smc_I = 10
		self.smc_D = 0.1
		self.lam = 40.0
		self.eta = 0.5
		self.tanh_n = 20"""

		# Derivativni clan
		self.ydp = 0
		self.ydr = 0

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
		self.y_pitch = 0			# y(k)
		self.yd_pitch = 0			# y'
		self.ydd_pitch = 0			# y''
		self.t = []
		self.br = 0

		self.uk_roll = 0				# u(k-1)
		self.ukk_roll = 0				# u(k-2)
		self.yk_roll = 0				# y(k-1)
		self.ykk_roll = 0				# y(k-2)
		self.y_roll = 0				# y(k)
		self.yd_roll = 0				# y'
		self.ydd_roll = 0			# y''



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

		# DOB
		self.delta_pitch = 0
		self.dob_q_ukp = 0
		self.dob_q_ukkp = 0
		self.dob_q_yp = 0
		self.dob_q_ykp = 0
		self.dob_q_ykkp = 0

		self.dob_pq_up = 0 
		self.dob_pq_ukp = 0
		self.dob_pq_ukkp = 0
		self.dob_pq_yp = 0
		self.dob_pq_ykp = 0
		self.dob_pq_ykkp = 0

		self.delta_roll = 0
		self.dob_q_ukr = 0
		self.dob_q_ukkr = 0
		self.dob_q_yr = 0
		self.dob_q_ykr = 0
		self.dob_q_ykkr = 0

		self.dob_pq_ur = 0 
		self.dob_pq_ukr = 0
		self.dob_pq_ukkr = 0
		self.dob_pq_yr = 0
		self.dob_pq_ykr = 0
		self.dob_pq_ykkr = 0

		self.u_pitch_dob = 0
		self.u_roll_dob = 0

		self.konst = 0

		


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

		self.pub_ref = rospy.Publisher('/arducopter/ref', Vector3, queue_size = 10)
		self.pub_ref_d = rospy.Publisher('/arducopter/refd', Vector3, queue_size = 10)
		self.pub_ref_dd = rospy.Publisher('/arducopter/refdd', Vector3, queue_size = 10)
		self.s_pub = rospy.Publisher('/arducopter/sigma', Float64, queue_size = 10)
		self.dx_pub = rospy.Publisher('/arducopter/dx', Float64, queue_size = 10)
		self.euler_ang = rospy.Publisher('arducopter/euler_angles', Vector3, queue_size = 10)
		self.motor_cmd = rospy.Publisher('/arducopter/command/motors', MotorSpeed, queue_size = 10)
		self.delta_pub = rospy.Publisher('/arducopter/delta', Float64, queue_size = 10)
		self.konst_pub = rospy.Publisher('/nula', Float32, queue_size = 10)

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

			ydr = self.ydr
			ydp = self.ydp


			###################
			###### PITCH ######
			###################


			# PUBLISHER REFERENCE - ZA TESTIRANJE
			ref_msg = Vector3()
			ref_msg.y = self.y_pitch
			ref_msg.x = self.y_roll
			self.pub_ref.publish(ref_msg)

			#ref_msg = Vector3()
			#ref_msg.y = self.yd_pitch[lpd-1]
			#self.pub_ref_d.publish(ref_msg)

			#ref_msg = Vector3()
			#ref_msg.y = self.ydd_pitch[lpdd-1]
			#self.pub_ref_dd.publish(ref_msg)

			# Sigma
			[self.sigma_I_old_p, self.ydp, self.sigma_pitch] = self.sigma(self.y_pitch, self.yd_pitch, pitch_m, pitch_r, dt, self.sigma_I_old_p, ydp)

			# PUBLISHER SIGMA - ZA TESTIRANJE
			s_msg = Float64()
			s_msg.data = self.sigma_pitch
			self.s_pub.publish(s_msg)

			konst_msg = Float32()
			konst_msg.data = self.konst
			self.konst_pub.publish(konst_msg)

			# DOB Delta
			[self.dob_q_ukkp, self.dob_q_ukp, self.dob_q_ykkp, self.dob_q_ykp, self.dob_pq_ukkp, self.dob_pq_ukp, self.dob_pq_ykkp, self.dob_pq_ykp, self.delta_pitch] = self.DOB(self.u_pitch_dob, self.dob_q_ukp, self.dob_q_ukkp, self.dob_q_ykp, self.dob_q_ykkp, pitch_m, self.dob_pq_ukp, self.dob_pq_ukkp, self.dob_pq_ykp, self.dob_pq_ykkp)
			
			# PUBLISHER DELTA - ZA TESTIRANJE
			d_msg = Float64()
			d_msg.data = self.delta_pitch
			self.delta_pub.publish(d_msg)


			# Control signal
			[dx_pitch, self.u_pitch_dob] = self.control_smc(self.sigma_pitch, self.y_pitch, self.yd_pitch, self.ydd_pitch, pitch_m, self.tanh_n, self.delta_pitch)

			#print dx_pitch
			dx_msg = Float64()
			dx_msg.data = dx_pitch
			self.dx_pub.publish(dx_msg)



			###################
			######  ROLL ######
			###################


			# Sigma
			[self.sigma_I_old_r, self.ydr, self.sigma_roll] = self.sigma(self.y_roll, self.yd_roll, roll_m, roll_r, dt, self.sigma_I_old_r, ydr)

			# DOB Delta
			[self.dob_q_ukkr, self.dob_q_ukr, self.dob_q_ykkr, self.dob_q_ykr, self.dob_pq_ukkr, self.dob_pq_ukr, self.dob_pq_ykkr, self.dob_pq_ykr, self.delta_roll] = self.DOB(self.u_roll_dob, self.dob_q_ukr, self.dob_q_ukkr, self.dob_q_ykr, self.dob_q_ykkr, roll_m, self.dob_pq_ukr, self.dob_pq_ukkr, self.dob_pq_ykr, self.dob_pq_ykkr)
	
			# Control signal
			[dy_roll, self.u_roll_dob] = self.control_smc(self.sigma_roll, self.y_roll, self.yd_roll, self.ydd_roll, roll_m, self.tanh_n, self.delta_roll)


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
		# T = 0.17 tr = 0.571
		#return 0.001663730036015*uk + 0.001599747657011*ukk + 1.885746287709750*yk - 0.889009765402775*ykk

		# T = 0.18 tr = 0.604
		#return 0.001487227265081*uk + 0.001433151735758*ukk + 1.891918937813531*yk - 0.894839316814370*ykk

		# T = 0.19 tr = 0.638
		#return 0.001337389456382*uk + 0.001291276763003*ukk + 1.897458960032874*yk - 0.900087626252259*ykk

		# T = 0.2 tr = 0.672
		#return 0.001209104274250*uk + 0.001169464760281*ukk + 1.902458849001428*yk - 0.904837418035960*ykk

		# T = 0.21 tr = 0.705
		#return 0.001098428269691*uk + 0.001064104940069*ukk + 1.906993909666953*yk - 0.909156442876713*ykk

		# T = 0.22 tr = 0.739
		#return 0.001002280264976*uk + 9.723634807182754e-04*ukk + 1.911126072536569*yk - 0.913100716282263*ykk

		# T = 0.23 tr = 0.772
		#return 9.182246242982179e-04*uk + 8.919912644264549e-04*ukk + 1.914906736136762*yk - 0.916716952025487*ykk

		# T = 0.25 tr = 0.839
		#return 7.789832815838621e-04*uk + 7.584848004055028e-04*ukk + 1.921578878304646*yk - 0.923116346386636*ykk

		# T = 0.3 tr = 0.839
		return 5.433628352605679e-04*uk + 5.433628352605679e-04*ukk + 1.934432200964012*yk - 0.935506985031618*ykk
		#return 1.973532271095919e04*uk + 1.947393117030134e04*ukk + 1.960397346613511*yk - 0.960789439152323*ykk


		#return 3.073401709590146e-04*uk + 3.073401709590146e-04*ukk + 1.950619824056665*yk - 0.951229424500714*ykk

	# Aproksimacija prve derivacije
	def dy(self, y, yk, Ts):
		return (y - yk) / Ts

	# Aproksimacija druge derivacije
	def ddy(self, y, yk, ykk, Ts):
		return (y - 2*yk + ykk) / Ts**2

	def pt1(self, y, u):
		return 0.632120558828558*u + 0.367879441171442*y
		#return 0.181269246922018*u + 0.818730753077982*y

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

	def Q(self, uk, ukk, yk, ykk):
		return 5.433628352605679e-04*uk + 5.433628352605679e-04*ukk + 1.934432200964012*yk - 0.935506985031618*ykk

	def P_Q(self, u, uk, ukk, yk, ykk):
		return 0.058694444444444*u -0.117356996509142*uk + 0.058662552064697*ukk + 1.934432200964012*yk - 0.935506985031618*ykk

	def DOB(self, qu, q_uk, q_ukk, q_yk, q_ykk, pq_u, pq_uk, pq_ukk, pq_yk, pq_ykk):
		q = self.Q(q_uk, q_ukk, q_yk, q_ykk)
		pq = self.P_Q(pq_u, pq_uk, pq_ukk, pq_yk, pq_ykk)

		delta = pq - q

		#azuriranje stanja
		#q_ukk = q_uk
		#q_uk = qu
		#q_ykk = q_yk
		#q_yk = q

		#pq_ukk = pq_uk
		#pq_uk = pq_u
		#pq_ykk = pq_yk
		#pq_yk = pq

		return q_uk, qu, q_yk, q, pq_uk, pq_u, pq_yk, pq, delta



	def sigma(self, ref, ref_d, angle_m, angle_r, dt, sigma_I_old, y):
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
		#print 'sigma_D = ', sigma_D

		sigma_D = self.pt1(y, sigma_D)
		#print 'PT1 = ', sigma_D
		#print ''

		sigma = sigma_P + sigma_I + sigma_D
		

		

		self.br += 1
		if self.br % 30 == 0:
			#print sigma
			pass


		#sigma = self.dead_zone(sigma, tol)


		# Sigma
		return sigma_I_old, sigma_D, sigma


	def control_smc(self, sigma, ref, ref_d, ref_dd, angle_m, n, delta):
		e = ref - angle_m

		u_nom = 0
		u_ctl_vector = 0
		u_tmp = 0

		# This part of the controller uses nominal model of the system in 
		# order to design a stabilizing controller

		u_nom = (e * self.lam + ref_dd) * self.Iq_yy + ref_d * self.smc_P

		#if abs(ref) > 0.12 and abs(ref) < 0.15:
		#	self.eta = 0.17
		#elif abs(ref) >= 0.15:
		#	self.eta = 0.23
		#elif abs(ref) <= 0.1:
		#	self.eta = 0.09
		#else:
		#	self.eta = 0.145

		u_ctl_vector = self.eta * tanh(sigma)

		u = (u_nom + u_ctl_vector - delta)
		u_tmp = u / (2 * self.m * self.g)

		return self.satur(u_tmp), u



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

		self.y_pitch = self.ref_pref(self.uk_pitch, self.ukk_pitch, self.yk_pitch, self.ykk_pitch)
		self.yd_pitch = self.dy(self.y_pitch, self.yk_pitch, self.Ts)

		
		self.ydd_pitch = self.ddy(self.y_pitch, self.yk_pitch, self.ykk_pitch, self.Ts)

		# Azuriranje stanja

		self.ukk_pitch = self.uk_pitch
		self.uk_pitch = self.ref_pitch
		self.ykk_pitch = self.yk_pitch
		self.yk_pitch = self.y_pitch

		### ROLL ###

		self.y_roll = self.ref_pref(self.uk_roll, self.ukk_roll, self.yk_roll, self.ykk_roll)
		self.yd_roll = self.dy(self.y_roll, self.yk_roll, self.Ts)

		self.ydd_roll = self.ddy(self.y_roll,self.yk_roll, self.ykk_roll, self.Ts)

		# Azuriranje stanja

		self.ukk_roll = self.uk_roll
		self.uk_roll = self.ref_roll
		self.ykk_roll = self.yk_roll
		self.yk_roll = self.y_roll



	def pose_cb(self, msg):
		self.pose_z = msg.pose.pose.position.z

	def pose_ref_cb(self, msg):
		self.ref_z = msg.z


if __name__ == '__main__':

	rospy.init_node('movable_mass_ctl')
	mm_smc = SMC_basic_ctl()
	mm_smc.run()

	

