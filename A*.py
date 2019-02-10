#!/usr/bin/env python
import numpy
import Queue as Q
import IPython

grid = [[0,0,1,0,0,0],
		[0,0,1,0,0,0],
		[0,0,0,0,1,0],
		[0,0,1,1,1,0],
		[0,0,0,0,0,0]]


heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]


# print (numpy.shape(grid))

init = [0,0]
goal = [4,5]

delta = [[-1,0],
		 [0,-1],
		 [1,0], 
		 [0,1]]

delta_name = ['^','<','v','>']

expand = [[-1 for idx in range(len(grid[0]))] for idy in range(len(grid))]

def search():
	closed = [[0 for idx in range(len(grid[0]))] for idy in range(len(grid))]

	# print (closed)

	closed[init[0]][init[1]] = 1
	
	# print (closed)
	# x = init[0]
	# # print (x)
	# y = init[1]
	# g = 0

	open = Q.PriorityQueue()
	x = 0
	y = 0
	g = 0
	cost = 1

	open.put([g,x,y])

	found = False
	resign = False
	counter = 0

	# 


	while found is False and resign is False:

		
		print ("initial open list:")
		print open.queue



		if open.empty()== True:
			resign = True
			print ('fail')

		else:
			next = open.get()

			x = next[1]
			y = next[2]
			g = next[0]

			if x == goal[0] and y== goal[1]:
				found = True
				print ('next')

			else:
				for i in range(len(delta)):
					x2 = x + delta[i][0]
					y2 = y + delta[i][1]
					if x2 >=0 and x2 <len(grid) and y2>=0 and y2 <len(grid[0]):
						if closed[x2][y2] == 0 and grid[x2][y2] == 0:
							g2 = g + cost + heuristic[x2][y2]
							open.put([g2,x2,y2])
							print ('append list items')
							print ([g2,x2,y2])
							closed[x2][y2] = 1
							expand[x2][y2] = g2


				
search()

for i in range(len(expand)):
	print(expand[i])


# plan = [[' ' for idx in range(len(grid[0]))] for idy in range(len(grid))]
# x = goal[0]
# y = goal[1]

# # IPython.embed()

# while x !=init[0] and y!=init[1]:
# 	x2 = x-delta[expand[x][y]][0]
# 	y2 = y-delta[expand[x][y]][1]
# 	plan[x2][y2] = delta_name[expand[x][y]]
# 	x = x2
# 	y = y2


# for i in range(len(plan)):
# 	print plan[i]


# def print_path():
# 	x=0 
# 	y=0 
# 	g=0 

# 	for i in range(len(delta)):
# 		gx[i] = expand[x+delta[i][0]][y+delta[i][1]]

