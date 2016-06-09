# mdp implementation needs to go here
import sys
import copy

def mdp(config):
#inputs 1-5 used in A*
	start = config['starting_pos']
	goal  = config['goal']
	walls = config['walls']
	pits  = config['pits']
	move_list = config['possible_moves']
	map_size = config['map_size']

#probabilities of correct movements
	prob_f = config['prob_move_forward']
	prob_b = config['prob_move_backward']
	prob_l = config['prob_move_left']
	prob_r = config['prob_move_right']

#rewards for things
	reward_step = config['reward_for_each_step']
	reward_wall = config['reward_for_hitting_wall']
	reward_goal = config['reward_for_reaching_goal']
	reward_pit = config['reward_for_falling_in_pit']

#numbers important for the mdp algorithm
	discount_factor = config['discount_factor']
	max_iterations = config['max_iterations']
	threshold_difference = config['threshold_difference'] 

	policy_map = [["" for y in range(map_size[1])] for x in range(map_size[0])]

	value_map = [[0.0 for y in range(map_size[1])] for x in range(map_size[0])]

	for wall in walls:
		policy_map[wall[0]][wall[1]] = "WALL"
		value_map[wall[0]][wall[1]] =0.0# reward_wall 
	for pit in pits:
		policy_map[pit[0]][pit[1]] = "PIT"
		value_map[pit[0]][pit[1]] = 0.0#reward_pit

	policy_map[goal[0]][goal[1]] = "GOAL"
	value_map[goal[0]][goal[1]] = 0.0#reward_goal 


	iterations = 0
	converge = 0
#TODO still need to figure publishing
# this is the FULLY OPERATIONAL mdp
	while(iterations < max_iterations):
		new_value_map = copy.deepcopy(value_map)
		#iterate through each cell and calculate the thing
		for row,(rowp,rowv) in enumerate(zip(policy_map,value_map)):
			for col,(valp,valv) in enumerate(zip(rowp,rowv)):
				#iterate through all possible moves from a cell
				if valp == "WALL" or valp == "GOAL" or valp == "PIT":
					continue
				maxv = -sys.maxint-1 
				maxp = ""
				for move in move_list:
					if not row+move[0] in range(0,map_size[0]) or not col+move[1] in range(0,map_size[1]):
						forward = valv 
						reward_f = reward_wall
					else:
						forward = new_value_map[row+move[0]][col+move[1]]
						cell = policy_map[row+move[0]][col+move[1]]
						reward_f = reward_step
						if cell == "GOAL":
							reward_f += reward_goal
						if cell == "PIT":
							reward_f += reward_pit
						if cell == "WALL":
							forward = valv
							reward_f = reward_wall

					if not row-move[0] in range(0,map_size[0]) or not col-move[1] in range(0,map_size[1]):
						backward = valv 
						reward_b = reward_wall
					else:
						backward =new_value_map[row-move[0]][col-move[1]]
						cell =policy_map[row-move[0]][col-move[1]]
						reward_b = reward_step
						if cell == "GOAL":
							reward_b += reward_goal
						if cell == "PIT":
							reward_b += reward_pit
						if cell == "WALL":
							backward = valv
							reward_b = reward_wall

					if not row-move[1] in range(0,map_size[0]) or not col-move[0] in range(0,map_size[1]):
						left = valv
						reward_l = reward_wall
					else:
						left = new_value_map[row-move[1]][col-move[0]]
						cell = policy_map[row-move[1]][col-move[0]]
						reward_l = reward_step
						if cell == "GOAL":
							reward_l += reward_goal
						if cell == "PIT":
							reward_l += reward_pit
						if cell == "WALL":
							left = valv
							reward_l = reward_wall

					if not row+move[1] in range(0,map_size[0]) or not col+move[0] in range(0,map_size[1]): 
						right = valv
						reward_r = reward_wall
					else:
						right = new_value_map[row+move[1]][col+move[0]]
						cell = policy_map[row+move[1]][col+move[0]]
						reward_r = reward_step
						if cell == "GOAL":
							reward_r += reward_goal
						if cell == "PIT":
							reward_r += reward_pit
						if cell == "WALL":
							right = valv
							reward_r = reward_wall

					v_f = prob_f*(reward_f+(discount_factor*forward))
					v_b = prob_b*(reward_b+(discount_factor*backward))
					v_l = prob_l*(reward_l+(discount_factor*left))
					v_r = prob_r*(reward_r+(discount_factor*right))
					summ = v_f + v_b + v_l + v_r

					#get the maximum value and policy
					if summ > maxv:
						maxv = summ
						if move == [0,1]:
							maxp = "E"
						elif move == [0,-1]:
							maxp = "W"
						elif move == [1,0]:
							maxp = "S"
						else:
							maxp = "N"
				policy_map[row][col] = maxp
				new_value_map[row][col] = maxv


		
		#determine convergence
		diff = absolute_diff(value_map,new_value_map,policy_map)
		if diff < threshold_difference:
			converge += 1
		else:
			converge = 0

		value_map = new_value_map
		iterations += 1
		if converge == 2:
			break
	
#flat_policy_map = [x for y in policy_map for x in y]
	return policy_map




#function for finding absolute difference to determine convergence
def absolute_diff(a,b,policy_map):
	summ = 0
	for rowv_a,rowv_b,rowp in zip(a,b,policy_map):
		for colv_a,colv_b,colp in zip(rowv_a,rowv_b,rowp):
			if colp != "WALL" and colp != "PIT" and colp != "GOAL":
				summ += abs(colv_a-colv_b)
	return summ
