#!/usr/bin/env python
import numpy
import IPython

grid = [[0,0,1,0,0,0],
		[0,0,1,0,0,0],
		[0,0,0,0,1,0],
		[0,0,1,1,1,0],
		[0,0,0,0,1,0]]

# print (numpy.shape(grid))

init = [0,0]
goal = [4,5]

delta = [[-1,0],
		 [0,-1],
		 [1,0],
		 [0,1]]

delta_name = ['^','<','v','>']

cost = 1


# IPython.embed()

def search():
	# closed = [[0 for row in range(len(grid [0]))] for col in range(len(grid))
	closed = [[0,0,0,0,0,0],
			  [0,0,0,0,0,0],
			  [0,0,0,0,0,0],
			  [0,0,0,0,0,0],
			  [0,0,0,0,0,0]]	 

	# print (closed)

	closed[init[0]][init[1]] = 1
	# print (closed)
	# x = init[0]
	# # print (x)
	# y = init[1]
	# g = 0

	x = 1
	y = 5
	g = 0

	open = [[g,x,y]]

	found = False
	resign = False

	print ("initial open list:")
	for i in range(len(open)):
		print (open[i])


	while found is False and resign is False:

		print ("initial open list:")
		for i in range(len(open)):
			print (open[i])
			print (closed)



		if len(open) ==0:
			resign = True
			print ('fail')

		else:
			open.sort()
			open.reverse()
			next = open.pop()

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
					if x2 >=0 and x2 <5 and y2>=0 and y2 <6:
						if closed[x2][y2] == 0 and grid[x2][y2] == 0:
							g2 = g + cost
							open.append([g2,x2,y2])
							print ('append list items')
							print ([g2,x2,y2])
							closed[x2][y2] = 1

search()


