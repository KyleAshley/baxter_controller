import os
import sys
import cv2
from datetime import datetime
# lsusb -v -d 045e:02ae | grep -e "Bus\|iSerial"

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

def removePCDs(path):
    for root, dirs, files in os.walk(path):
        for currentFile in files:
            exts=('.pcd')
            if any(currentFile.lower().endswith(ext) for ext in exts):
                print "Removing file: " + currentFile
                os.remove(os.path.join(root, currentFile))

def speak(message, wait=True):
   speak_process = subprocess.Popen('./untitled.sh "' + message + '"', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
   if wait:
       speak_process.wait()

def criticalError(message):
    critical_sound_process = subprocess.Popen('aplay pacman_death.wav', shell=True,
                                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                              cwd="./ErrorLib/")
    critical_process = subprocess.Popen('./spacey.sh "' + '\033[1m\033[31m[CRITICAL]\033[0m ' + message + '"', shell=True,
                                        cwd="./ErrorLib/")
    critical_process.wait()


def queryShutoff():
	for i in range(10):
		key = cv2.waitKey(30)
		if key == 27:
			return 1

	return 0

def retrieve_times_to_file(times_list):
	f = open('times_'+str(datetime.now())+'.txt', 'w')

	for t in range(len(times_list)-1):
		f.write(str(times_list[t+1] - times_list[t])+'\n')

	f.write('\nTime to TABLE\nTime to OPE\nTime to COLOR\nTime to GRASP\nTime to USER\nTime to BACK')
	f.close()
