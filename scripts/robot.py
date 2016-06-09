#!/usr/bin/env python
import random as r
import math as m
import numpy as np
from copy import deepcopy

import rospy
from cse_190_assi_1.srv import *
from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities, pathMessage, PolicyList
from std_msgs.msg import Bool, Float32, String
from read_config import read_config
from mdp import *

"""
- Change the belief to account for walls
- Calculate MDP
	1) Get temp/texture reading
	2) Calculate belief based on readings
	3) Use most likely probability as the robot's position
	4) Use position to find where to move next (using policy found by MDP)
	5) Move, and update belief after that move (either moves correctly, or doesnt move)
		a) change update move belief to account for a fixed room size
	6) Publish everything 
"""

class Robot():
	""" Robot class. Has the following fields:
		config          -- grid properties parsed from configuration.json
		temperature     -- last temperature reading
		texture         -- last texture reading
		position        -- where the robot thinks it is with most likeliehood
		move       -- the robot's next move towards the goal
		prior_belief    -- the robot's belief about its position
		path_publisher         -- publisher to results/path_data
		temperature_subscriber -- subscriber to temp_sensor/data
		texture_publisher      -- publisher to results/texture_data
		temperature_publisher  -- publisher to results/temperature_data
		temperature_activation -- publisher to temp_sensor/activation
		position_publisher     -- publisher to results/probabilities
		policy_publisher       -- publisher to results/policy_data
		done_publisher         -- publisher to map_node/sim_complete
		request_texture        -- service proxy for requestTexture
		request_move           -- service proxy for moveService
		policy          -- where to go at each grid cell
		move_instruction -- maps direction to a move instruction
		threshold       -- how confident the robot needs to be that it reached the goal
	"""

	def __init__(self):
		# initialize robot node
		rospy.init_node("robot")

		# parse config file
		self.config = read_config()
		self.goal = self.config['goal']
		self.threshold = self.config['threshold']

		# initialize temperature and texture fields
		self.temperature = 0.0
		self.texture = ''
		self.position = [0,0]
		self.move = ''

		# initialize robot's belief about position
		initial_belief = 1/(float((len(self.config['pipe_map']) * len(self.config['pipe_map'][0]))) - float(len(self.config['walls'])))

		self.prior_belief = [[initial_belief for x in range(len(self.config['pipe_map'][y]))] for y in range(len(self.config['pipe_map']))]

		for x, row in enumerate(self.prior_belief):
			for y, col in enumerate(row):
				if [x,y] in self.config['walls']:
					col = 0.0

		#calculate MDP here
		self.policy = mdp(self.config)
		self.move_instruction = {
			'N': [-1,0],
			'W': [0,-1],
			'S': [1,0],
			'E': [0,1],
			'WALL': [2,2],
			'GOAL': [0,0]
		}

		# Initialize subscribers, publishers, and services
		self.init_ros_things()	

		# Publish bool message to temp sensor's activation topic
		bool_msg = Bool()
		bool_msg.data = True
		self.temperature_activation.publish(bool_msg)

		# Publish the results of MDP
		policy_list = [x for y in self.policy for x in y]
		self.policy_publisher.publish(policy_list)

		rospy.sleep(1)
		rospy.spin()

		rospy.signal_shutdown(self)

	def init_ros_things(self):
		#publish each step in the path taken
		self.path_publisher = rospy.Publisher(
				"/results/path_data",
				pathMessage,
				queue_size = 10
		)

		#publish MDP policy list
		self.policy_publisher = rospy.Publisher(
				"/results/policy_data",
				PolicyList,
				queue_size = 10
		)

		#publish temperature sensor switch
		self.temperature_activation = rospy.Publisher(
				"/temp_sensor/activation",
				Bool,
				queue_size = 10
		)

		#subscribe to temperature readings
		self.temperature_subscriber = rospy.Subscriber(
				"/temp_sensor/data",
				temperatureMessage,
				self.handle_temperature_message
		)

		#publish most recent temperature reading
		self.temperature_publisher = rospy.Publisher(
				"/results/temperature_data",
				Float32,
				queue_size = 10
		)

		#publish most recent texture reading
		self.texture_publisher = rospy.Publisher(
				"/results/texture_data",
				String,
				queue_size = 10
		)

		#publish beliefs about position
		self.position_publisher = rospy.Publisher(
				"/results/probabilities",
				RobotProbabilities, 
				queue_size = 10
		)

		# initialize services
		rospy.wait_for_service("requestTexture")
		self.request_texture = rospy.ServiceProxy(
				"requestTexture",
				requestTexture
		)

		rospy.wait_for_service("moveService")
		self.request_move = rospy.ServiceProxy(
				"moveService",
				moveService
		)

		# initialize the data transcriber publisher
		self.done_publisher = rospy.Publisher(
				"/map_node/sim_complete",
				Bool,
				queue_size = 10
		)

		# sleep to allow the topics to activate
		rospy.sleep(3)

	def publish_results(self):
		""" Publishes the results of the temperature and texture
			data, as well as the probability matrix of the robot's
			belief on its position. """

		# publish temperature data
		tmp = Float32() 
		tmp.data = self.temperature
		self.temperature_publisher.publish(tmp)

		# publish texture data
		tex = String()
		tex.data = self.texture
		self.texture_publisher.publish(tex)

		# publish probability data
		prob = RobotProbabilities()
		prob.data = [x for y in self.prior_belief for x in y] 
		self.position_publisher.publish(prob)

		# publish path data
		path_data = pathMessage()
		path_data.location = self.position
		self.path_publisher.publish(path_data)

	def handle_temperature_message(self, message):
		""" The robot will execute the following when it gets a new
			temperature reading:
			- update its belief using the temperature reading
			- request a texture reading
			- update its belief using the texture reading	
			- make a move (if there is a next move to make)
			- update its belief after the move
			- publish the results at the end of each step

			If all moves have been made, the robot will die.
			"""

		# update the temperature
		self.temperature = message.temperature
		self.update_belief_temp()

		# request data from the texture sensor
		texture_response = self.request_texture()
		self.texture = texture_response.data
		self.update_belief_texture()

		# Check most likely position of robot
		val = self.check_most_likely_position()
		mlp = val[0]

		# check to make sure there are moves left
		if list(mlp) == self.goal and val[1] >= self.threshold:
			self.publish_results()
			self.done_publisher.publish(True)
			rospy.sleep(1)
			rospy.signal_shutdown(self)
			return

		# request map service to move robot
		direction = self.policy[mlp[0]][mlp[1]]
		next_move = self.move_instruction[direction]
		if next_move == [2,2]:
			print "Stuck in wall"
			return
		print "We think we're here:", mlp, "so let's move", direction
		self.request_move(next_move)
		self.update_belief_movement(next_move)

		self.prior_belief[mlp[0] + next_move[0]][mlp[1] + next_move[1]] *= 1.3
		self.prior_belief = self.normalize(self.prior_belief)

		self.position = str(mlp)
		self.publish_results()

	def update_belief_temp(self):
		""" Update the robot's belief about its position given the
			temperature reading. """

		updated_belief = []
		temp_map = self.config['pipe_map']
		sigma = self.config['temp_noise_std_dev']

		# calculate the bayes discrete filter
		for i, row in enumerate(temp_map):
			to_append = []

			for j, col in enumerate(row):
				p_likelihood = 0   # P(temp sensed | xi)
				p_prior = self.prior_belief[i][j]   # P(xi)
				# probability that the sensor got reading given prior belief

				if temp_map[i][j] == 'H':
					p_likelihood = (1/(m.sqrt(2*m.pi)*sigma)) * m.exp((-1*((self.temperature-40)**2))/(2*(sigma**2)))
				elif temp_map[i][j] == '-':
					p_likelihood = (1/(m.sqrt(2*m.pi)*sigma)) * m.exp((-1*((self.temperature-25)**2))/(2*(sigma**2)))
				elif temp_map[i][j] == 'C':
					p_likelihood = (1/(m.sqrt(2*m.pi)*sigma)) * m.exp((-1*((self.temperature-20)**2))/(2*(sigma**2)))

				# posterior numerator = likelihood * prior
				p_posterior_num = p_likelihood * p_prior
				to_append.append(p_posterior_num)
			updated_belief.append(to_append)

		# normalize the belief and save into prior
		updated_belief = self.normalize(updated_belief)
		self.prior_belief = updated_belief
	
	def update_belief_texture(self):
		""" Update the robot's belief about its position given the
			texture reading """
		updated_belief = []
		texture_map = self.config['texture_map']
		p_correct = self.config['prob_tex_correct']
		p_incorrect = 1 - p_correct

		# calculate the bayes discrete filter
		for i, row in enumerate(texture_map):
			to_append = []
			for j, col in enumerate(row):
				p_likelihood = 0   # P(texture sensed | xi)
				p_prior = self.prior_belief[i][j]   # P(xi)

				# probability the sensor got the reading given prior belief
				if self.texture == texture_map[i][j]:
					p_likelihood = p_correct
				else:
					p_likelihood = p_incorrect

				p_posterior = p_likelihood * p_prior 
				to_append.append(p_posterior)
			updated_belief.append(to_append)

		# normalize the distribution
		updated_belief = self.normalize(updated_belief)
		# update robot's belief
		self.prior_belief = updated_belief

	def update_belief_movement(self, move):
		""" Update the robot's belief about its position based on 
			the move it just did. """
		belief = []
		# intialize belief to be same size as prior belief, but zeroed out
		for rows in self.prior_belief:
			to_append = []
			for cols in rows:
				to_append.append(0.0)
			belief.append(to_append)

		# calculate probabilities
		for y in range(len(self.prior_belief)):
			for x in range(len(self.prior_belief[y])):
				# Account for robot not moving
				prob = self.prior_belief[y][x]
				prob *= 1-self.config['prob_move_correct']
				belief[y][x] += prob

				# Account for robot moving
				y_ = move[0]
				x_ = move[1]
				if y-y_ in range(len(self.prior_belief)) and x-x_ in range(len(self.prior_belief[0])):
					prob = self.prior_belief[y-y_][x-x_]
					prob *= self.config['prob_move_correct']
					belief[y][x] += prob

		# normalize the matrix and save into self.prior_belief
		self.prior_belief = belief

	def check_most_likely_position(self):
		""" Returns a pair of coordinates representing the most likely
			grid cell the robot is in, and the probability of that. """
		maxx = 0
		x = 0
		y = 0
		for i, row in enumerate(self.prior_belief):
			for j, col in enumerate(row):
				if col > maxx:
					maxx = col
					x = i
					y = j
		return [(x, y), maxx]

	def check_normal(self, matrix):
		sanity_check = 0
		for i in matrix:
			for j in i:
				sanity_check += j
		print "This should be 1.0:", sanity_check

	def normalize(self, matrix):
		""" Returns a normalized version of the matrix param,
	   		given the normalization factor. """
		k = 0.0
		for i in matrix:
			for j in i:
			   k += j
		to_return = []
		i = 0

		while i < len(matrix):
			to_add = []
			j = 0
			while j < len(matrix[0]):
				matrix[i][j] /= k
				to_add.append(matrix[i][j])
				j += 1
			to_return.append(to_add)
			i += 1

		return to_return

if __name__ == "__main__":
	donut_rover = Robot()
