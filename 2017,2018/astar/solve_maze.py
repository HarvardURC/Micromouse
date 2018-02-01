class Cell:
	walls = {'north':0, 'south':0, 'east':0, 'west':0}
	costs = {'f':0, 'g':0, 'h':0}
	
	def __init__(self, x, y):
		self.x = x
		self.y = y
	def __str__(self):
		return "a"

maze = [[0 for x in range(16)] for y in range(16)]

for i in range(16):
	for j in range(16):
		maze = Cell(i, j)

print(maze)
