class grid:

	def __init__(self, _col, _row, _mode):
		self.mode = _mode #2 mode of grid can be generate
		self.col = _col
		self.row = _row
		if self.mode == 'sym':
			self.map = [[' ' for col in range(_col)] for row in range(_row)]
		elif self.mode == 'num':
			self.map = [[0 for col in range(_col)] for row in range(_row)]
	
	def set_obstacle(self, x0, y0, x_len, y_len):
		if self.mode == 'sym' : obs = '#'
		elif self.mode == 'num' : obs = 1
		
		for i in range(x0, x0 + x_len):
			for j in range(y0, y0 + y_len):
				self.map[i][j] = obs

	def print_map(self):
		for row in range(self.row):
			print self.map[row]

	def __repr__(self):	
		return repr(self.map)

def main():
	mygrid = grid(10, 8, 'sym')
	mygrid.set_obstacle(2, 2, 4, 4)
	mygrid.set_obstacle(0, 0, 1, 7)
	mygrid.print_map()

if __name__ == '__main__':
	main()