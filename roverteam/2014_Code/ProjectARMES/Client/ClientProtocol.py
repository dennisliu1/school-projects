class ClientProtocol:

	PROTOCOL_TIME = 'time:'
	PROTOCOL_NAVPT = 'navPt:'
	PROTOCOL_NAVRT = 'navRt:'
	PROTOCOL_TASK = 'task:'

	TASK_LIST_TYPE_REFRESH = 0
	TASK_LIST_TYPE_UNDO = 1
	TASK_LIST_TYPE_DONE = 2
	TASK_LIST_TYPE_ADD = 3

	def __init__(parent=None):
		pass

	def encodeTimeMsg(self, time):
		return 'time:'+str(time)

	def decodeTimeMsg(self, msg):
		return int(msg.split(':')[1])

	def encodeNavPointClearMsg(self):
		ret = 'navPt:'
		ret += 'clear'
		return ret

	def encodeNavPointMsg(self, points):
		ret = 'navPt:'
		ret += str(points[0])+','+str(points[1])+','+str(points[2])
		return ret

	def decodeNavPointMsg(self, msg):
		ret = (msg.split(':'))[1].split('_')
		if ret[0] == 'clear':
			return 'clear'
		arr = []
		for i in range(len(ret)):
			spi = ret[i].split(',')
			if len(spi) == 3:
				arr.append([spi[0],spi[1],spi[2]])# x,y, str
		return arr

	def encodeNavRouteClearMsg(self):
		ret = 'navRt:'
		ret += 'clear'
		return ret

	def encodeNavRouteMsg(self, routes):
		ret = 'navRt:'
		ret += str(routes[0][0])+','+str(routes[0][1])+','+str(routes[1][0])+','+str(routes[1][1])
		return ret

	def decodeNavRouteMsg(self, msg):
		ret = (msg.split(':'))[1].split('_')
		if ret[0] == 'clear':
			return ret[0]
		arr = []
		for i in range(len(ret)):
			spi = ret[i].split(',')
			if len(spi) == 4:
				arr.append([spi[0],spi[1],spi[2],spi[3]])# x,y, x2,y2
		return arr

	def encodeTaskListMsg(self, type, msg):
		ret = 'task:'
		if type == self.TASK_LIST_TYPE_REFRESH: # refresh
			ret += 'refresh='+msg
		elif type == self.TASK_LIST_TYPE_UNDO: # 
			ret += 'undo='
		elif type == self.TASK_LIST_TYPE_DONE:
			ret += 'done='
		elif type == self.TASK_LIST_TYPE_ADD:
			ret += 'add='+msg
		return ret

	def decodeTaskListMsg(self, msg):
		ret = msg.split(':')
		if ret[1].startswith('refresh='):
			arr = ret[1].split('=')[1].split('_')
			result = []
			for i in range(len(arr)):
				result.append(arr[i].split(','))
			return ret[1],result
		elif ret[1].startswith('undo'):
			return ret[1],''
		elif ret[1].startswith('done'):
			return ret[1],''
		elif ret[1].startswith('add='):
			arr = ret[1].split('=')[1].split('_')
			result = []
			for i in range(len(arr)):
				result.append(arr.split(','))
			return ret[1],result