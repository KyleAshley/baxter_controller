
class dataFilter:
	def __init__(self, size):
		self.size = 0
		self.max_size = size
		self.data = []
		self.sum = 0.0
		self.avg = 0.0

	def add(self, val):
		if(len(self.data) < self.max_size):
			self.data.insert(0,val)
		else:
			self.data.pop()
			self.data.insert(0,val)
		self.size = len(self.data)

	def average(self):
		self.sum = 0.0
		for x in self.data:
			self.sum = self.sum + x
		self.avg = self.sum / len(self.data)


def clamp(val, minval, maxval):
	if val < minval:
		return minval
	elif val > maxval:
		return maxval
	else:
		return val