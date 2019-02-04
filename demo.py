#! /usr/bin/python
# -*- coding: utf-8 -*-
"""

Dubins path planner sample code

author Atsushi Sakai(@Atsushi_twi)

License MIT

"""
import math
import matplotlib

linewidth=5
movie=False

def mod2pi(theta):
	return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)


def pi_2_pi(angle):
	while(angle >= math.pi):
		angle = angle - 2.0 * math.pi

	while(angle <= -math.pi):
		angle = angle + 2.0 * math.pi

	return angle

def general_planner(planner, alpha, beta, d):
	sa = math.sin(alpha)
	sb = math.sin(beta)
	ca = math.cos(alpha)
	cb = math.cos(beta)
	c_ab = math.cos(alpha - beta)
	mode = list(planner)
	#print(mode)

	planner_uc = planner.upper()

	if planner_uc == 'LSL':
		tmp0 = d + sa - sb
		p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
		if p_squared < 0:
			return None
		tmp1 = math.atan2((cb - ca), tmp0)
		t = mod2pi(-alpha + tmp1)
		p = math.sqrt(p_squared)
		q = mod2pi(beta - tmp1)
		#  print(math.degrees(t), p, math.degrees(q))

	elif planner_uc == 'RSR':
		tmp0 = d - sa + sb
		p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
		if p_squared < 0:
			return None
		tmp1 = math.atan2((ca - cb), tmp0)
		t = mod2pi(alpha - tmp1)
		p = math.sqrt(p_squared)
		q = mod2pi(-beta + tmp1)

	elif planner_uc == 'LSR':
		p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
		if p_squared < 0:
			return None
		p = math.sqrt(p_squared)
		tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
		t = mod2pi(-alpha + tmp2)
		q = mod2pi(-mod2pi(beta) + tmp2)

	elif planner_uc == 'RSL':
		p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
		if p_squared < 0:
			return None
		p = math.sqrt(p_squared)
		tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
		t = mod2pi(alpha - tmp2)
		q = mod2pi(beta - tmp2)

	elif planner_uc == 'RLR':
		tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
		if abs(tmp_rlr) > 1.0:
			return None

		p = mod2pi(2 * math.pi - math.acos(tmp_rlr))
		t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
		q = mod2pi(alpha - beta - t + mod2pi(p))

	elif planner_uc == 'LRL':
		tmp_lrl = (6. - d * d + 2 * c_ab + 2 * d * (- sa + sb)) / 8.
		if abs(tmp_lrl) > 1:
			return None
		p = mod2pi(2 * math.pi - math.acos(tmp_lrl))
		t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.)
		q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

	else:
		print('bad planner:', planner)

	path = [t, p, q]

	# Lowercase directions are driven in reverse.
	for i in [0, 2]:
		if planner[i].islower():
			path[i] = (2 * math.pi) - path[i]
	# This will screw up whatever is in the middle.

	cost = sum(map(abs, path))

	return(path, mode, cost)




def plot_arrow(x, y, yaw, length=3.0, width=1.0, fc="r", ec="k", color='black'):
	u"""
	Plot arrow
	"""
	import matplotlib.pyplot as plt

	if not isinstance(x, float):
		for (ix, iy, iyaw) in zip(x, y, yaw):
			plot_arrow(ix, iy, iyaw, color=color)
	else:
		plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw), fc=fc, ec=ec, head_width=width, head_length=width, color='aqua')
		plt.plot(x, y, marker='o', color=color)


def dubins_path(start, end, curvature):
	(sx, sy, syaw) = start
	(ex, ey, eyaw) = end
	c = curvature

	ex = ex - sx
	ey = ey - sy

	lex = math.cos(syaw) * ex + math.sin(syaw) * ey
	ley = - math.sin(syaw) * ex + math.cos(syaw) * ey
	leyaw = eyaw - syaw
	D = math.sqrt(lex ** 2.0 + ley ** 2.0)
	d = D / c
	print('D:', D)

	theta = mod2pi(math.atan2(ley, lex))
	alpha = mod2pi(- theta)
	beta = mod2pi(leyaw - theta)

	#planners = ['RSr', 'rSR', 'rSr', 'LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL']
	planners = ['LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL']
	#planners = ['RSr']

	bcost = float("inf")
	bt, bp, bq, bmode = None, None, None, None

	for planner in planners:
		#t, p, q, mode = planner(alpha, beta, d)
		solution = general_planner(planner, alpha, beta, d)

		if solution is None:
			continue

		(path, mode, cost) = solution
		(t, p, q) = path
		if bcost > cost:
			# best cost
			bt, bp, bq, bmode = t, p, q, mode
			bcost = cost

	#  print(bmode)
	return(zip(bmode, [bt*c, bp*c, bq*c], [c] * 3))



def plot_solution(start, end, solution, show_work=3):
	print(start, end, solution)

	#plt.legend()
	plt.clf()
	plt.axis(x_axis + y_axis)
	#plt.grid(True, xdata=[5])
	
	curvature = solution[0][2]
	plt.text(-15, -23, 'start: (%d, %d, %d)' % (start[0], start[1], (90 - math.degrees(start[2])) % 360))
	plt.text(-15, -24, 'end: (%d, %d, %d)' % (end[0], end[1], (90 - math.degrees(end[2])) % 360))
	plt.text(-15, -25, 'curvature: %d' % (curvature))

	plt.text(-5, +23, 'Dubins path', multialignment='center', size=20)
	if False:
		plt.text(+5, -23, 'solution: %s %0.1f, %s %0.1f, %s %0.1f' % (
			solution[0][0], solution[0][1],
			solution[1][0], solution[1][1],
			solution[2][0], solution[2][1],
		))
	else:
		plt.text(+5, -23, 'solution: %s %s %s (%0.1f, %0.1f, %0.1f)' % (
			solution[0][0], solution[1][0], solution[2][0],
			solution[0][1], solution[1][1], solution[2][1],
		))

	plot_arrow(*start, color='green')
	plot_arrow(*end, color='red')

	ax = fig.add_subplot(111)

	current_position = start

	#plt.plot((start[0], end[0]), (start[1], end[1]))
	

	#plot_arrow(0.0,0.0,0)

	(sx, sy, syaw) = start
	(ex, ey, eyaw) = end
	
	ex = ex - sx
	ey = ey - sy

	lex = math.cos(syaw) * ex + math.sin(syaw) * ey
	ley = - math.sin(syaw) * ex + math.cos(syaw) * ey
	leyaw = eyaw - syaw

	px = [0]
	py = [0]
	pyaw = [0]

	for (mode, length, curvature) in solution:
		print('mode:', mode, 'length:', length, 'curvature:', curvature)


			
		if mode is 'L':
			print('left')
			center = (
				current_position[0] + math.cos(current_position[2] + math.pi/2.0) * curvature,	
				current_position[1] + math.sin(current_position[2] + math.pi/2.0) * curvature,	
			)
			#plt.gca().add_artist(plt.Circle(center, curvature, fill=False))
			#arcs = [matplotlib.patches.Arc(xy=center, width=curvature, height=curvature, angle=0, theta1=0, theta2=0)] 
			#arcs = [matplotlib.patches.Arc(xy=center, width=curvature * 2, height=curvature * 2, angle=0, theta1=0, theta2=360)] 

			circumference = math.pi * curvature
			theta1 = math.degrees(current_position[2])-90
			theta2 = theta1 + (180 * length/circumference)

			if show_work == 1:
				ax.add_artist(matplotlib.patches.Arc(
					xy=center, 
					width=curvature * 2,
					height=curvature * 2,
					#angle=0,
					#theta1=theta1,
					#theta2=theta2,
					color='purple',
					linewidth=linewidth,
				))
			elif show_work == 2:
				ax.add_artist(matplotlib.patches.Arc(
					xy=center, 
					width=curvature * 2,
					height=curvature * 2,
					angle=0,
					theta1=theta1,
					theta2=theta2,
					color='purple',
					linewidth=linewidth,
				))



			new_position = (
				center[0] + math.cos(math.radians(theta2)) * curvature,
				center[1] + math.sin(math.radians(theta2)) * curvature,
				math.radians(theta2 + 90)
			)

		elif mode is 'l':
			print('left, reverse')
			center = (
				current_position[0] + math.cos(current_position[2] + math.pi/2.0) * curvature,	
				current_position[1] + math.sin(current_position[2] + math.pi/2.0) * curvature,	
			)
			#plt.gca().add_artist(plt.Circle(center, curvature, fill=False))
			#arcs = [matplotlib.patches.Arc(xy=center, width=curvature, height=curvature, angle=0, theta1=0, theta2=0)] 
			#arcs = [matplotlib.patches.Arc(xy=center, width=curvature * 2, height=curvature * 2, angle=0, theta1=0, theta2=360)] 

			circumference = math.pi * curvature
			theta1 = math.degrees(current_position[2])-90
			theta2 = theta1 + (180 * length/circumference)

			if show_work == 1:
				ax.add_artist(matplotlib.patches.Arc(
					xy=center, 
					width=curvature * 2,
					height=curvature * 2,
					#angle=0,
					#theta1=theta1,
					#theta2=theta2,
					color='purple',
					linewidth=linewidth,
				))
			elif show_work == 2:
				ax.add_artist(matplotlib.patches.Arc(
					xy=center, 
					width=curvature * 2,
					height=curvature * 2,
					angle=0,
					theta1=theta1,
					theta2=theta2,
					color='purple',
					linewidth=linewidth,
				))



			new_position = (
				center[0] + math.cos(math.radians(theta2)) * curvature,
				center[1] + math.sin(math.radians(theta2)) * curvature,
				math.radians(theta2 + 90)
			)

		elif mode is 'R':
			print('right')
			center = (
				current_position[0] + math.cos(current_position[2] - math.pi/2.0) * curvature,	
				current_position[1] + math.sin(current_position[2] - math.pi/2.0) * curvature,	
			)
			circumference = math.pi * curvature
			theta2 = math.degrees(current_position[2])+90
			theta1 = theta2 - (180 *length/circumference)


			if show_work == 1:
				ax.add_artist(matplotlib.patches.Arc(
					xy=center, 
					width=curvature * 2,
					height=curvature * 2,
					#angle=0,
					#theta1=theta1,
					#theta2=theta2,
					color='blue',
					linewidth=linewidth,
				))
			elif show_work == 2:
				ax.add_artist(matplotlib.patches.Arc(
					xy=center, 
					width=curvature * 2,
					height=curvature * 2,
					angle=0,
					theta1=theta1,
					theta2=theta2,
					color='blue',
					linewidth=linewidth,
				))

			new_position = (
				center[0] + math.cos(math.radians(theta1)) * curvature,
				center[1] + math.sin(math.radians(theta1)) * curvature,
				math.radians(theta1 - 90)
			)

		elif mode is 'r':
			print('right, reverse')
			center = (
				current_position[0] + math.cos(current_position[2] - math.pi/2.0) * curvature,	
				current_position[1] + math.sin(current_position[2] - math.pi/2.0) * curvature,	
			)
			circumference = math.pi * curvature
			#theta2 = math.degrees(current_position[2])+90
			#theta1 = theta2 - (180 *length/circumference)
			theta1 = math.degrees(current_position[2])+90
			theta2 = theta1 + (180 * length/circumference)
			#theta1 = math.degrees(current_position[2])+90
			#theta2 = theta1 - (180 * length/circumference)


			if show_work == 1:
				ax.add_artist(matplotlib.patches.Arc(
					xy=center, 
					width=curvature * 2,
					height=curvature * 2,
					#angle=0,
					#theta1=theta1,
					#theta2=theta2,
					color='orange',
					linewidth=linewidth,
				))
			elif show_work == 2:
				ax.add_artist(matplotlib.patches.Arc(
					xy=center, 
					width=curvature * 2,
					height=curvature * 2,
					angle=0,
					theta1=theta1,
					theta2=theta2,
					color='orange',
					linewidth=linewidth,
				))

			new_position = (
				center[0] + math.cos(math.radians(theta1)) * curvature,
				center[1] + math.sin(math.radians(theta1)) * curvature,
				math.radians(theta1 - 90)
			)

		elif mode is 'S':
			print('straight')
			new_position = (
				current_position[0] + math.cos(current_position[2]) * length,
				current_position[1] + math.sin(current_position[2]) * length,
				current_position[2],
			)
			if show_work == 2:
				plt.plot(
					(current_position[0], new_position[0]),
					(current_position[1], new_position[1]),
					color='yellow',
					linewidth=linewidth,
				)

		else:
			raise Exception, 'unknown mode: %s' % (mode)

		current_position = new_position


	plt.ion()
	plt.draw()
	plt.pause(0.001)

if __name__ == '__main__':
	import matplotlib.pyplot as plt
	import time
	import random


	x_axis = [-35, +35]
	y_axis = [-20, +20]

	
	fig = plt.figure(figsize=(16,9), dpi=100)

	image_count = 0

	for i in range(20):
		if True: 
			#end = (float(random.randrange(*(plt.xlim()))), float(random.randrange(*(plt.ylim()))), math.radians(random.randrange(360)))
			#end = (float(random.randrange(*x_axis)), float(random.randrange(*y_axis)), math.radians(random.randrange(360)))
			end = (
				float(random.randrange(x_axis[0]+5, x_axis[1]-5)), 
				float(random.randrange(y_axis[0]+5, y_axis[1]-5)), 
				math.radians(random.randrange(360))
			)
			curvature = float(random.randrange(1,8))

			print('curvature = ', curvature)
			print('end = ', end)
		else:
			#end = (5.0, 5.0, math.radians(120))
			#end = (10.0, 2.0, math.radians(120))
			#curvature = 4.0
			#end = (3.0, -3.0, 5.532693728822025)
			curvature = 4.0
			end = (4.0, 3.0, 2.6878070480712677)


		#start = (0.0, 0.0, math.radians(+40))
		start = (0.0, 0.0, math.radians(+90))


		solution = dubins_path(start=start, end=end, curvature=curvature)

		for show_work in range(3):
			plot_solution(start=start, end=end, solution=solution, show_work=show_work)
			if movie:
				plt.savefig('/tmp/mpl/%03d.png' % (image_count))
			else:
				time.sleep(3)
			image_count += 1
		
		if not movie:
			time.sleep(3)

