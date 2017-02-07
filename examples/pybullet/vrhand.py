#script to control a simulated robot hand using a VR glove
#see https://twitter.com/erwincoumans/status/821953216271106048
#and https://www.youtube.com/watch?v=I6s37aBXbV8
#vr glove was custom build using Spectra Symbolflex sensors (4.5")
#inside a Under Armour Batting Glove, using DFRobot Bluno BLE/Beetle
#with BLE Link to receive serial (for wireless bluetooth serial)

import serial
import time
import pybullet as p

#first try to connect to shared memory (VR), if it fails use local GUI
c = p.connect(p.SHARED_MEMORY)
#p.resetSimulation()
p.setGravity(0,0,-10)
print(c)
if (c<0):
		p.connect(p.GUI)
		
#load the MuJoCo MJCF hand
objects = p.loadMJCF("MPL/mpl2.xml")

hand=objects[0]
ho = p.getQuaternionFromEuler([0,3.14,0])
hand_cid = p.createConstraint(hand,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0.1,0,0],[0.500000,0.300006,0.700000],ho)
print ("hand_cid")
print (hand_cid)
for i in range (p.getNumJoints(hand)):
	p.setJointMotorControl2(hand,i,p.POSITION_CONTROL,0,0)

#clamp in range 400-600
minV = 400
maxV = 600

POSITION=1
ORIENTATION=2
BUTTONS=6

p.setRealTimeSimulation(1)

def convertSensor(x):
	v = minV
	try:
		v = float(x)
	except ValueError:
		v = minV    
	if (v<minV):
		v=minV
	if (v>maxV):
		v=maxV
	b = (v-minV)/float(maxV-minV)
	return (1.0-b)
	
ser = serial.Serial(port='COM3',baudrate=115200,parity=serial.PARITY_ODD,stopbits=serial.STOPBITS_TWO,bytesize=serial.SEVENBITS)
if (ser.isOpen()):
	while True:
		events = p.getVREvents()
		for e in (events):
			if (e[BUTTONS][33]&p.VR_BUTTON_IS_DOWN):
				p.changeConstraint(hand_cid,e[POSITION],e[ORIENTATION], maxForce=50)
			
		while ser.inWaiting() > 0:
			line = str(ser.readline())
			words = line.split(",")
			if (len(words)==6):
				middle = convertSensor(words[1])
				pink = convertSensor(words[2])
				index = convertSensor(words[3])
				thumb = convertSensor(words[4])+0.2

				p.setJointMotorControl2(hand,5,p.POSITION_CONTROL,1.3)
				p.setJointMotorControl2(hand,7,p.POSITION_CONTROL,thumb)
				p.setJointMotorControl2(hand,9,p.POSITION_CONTROL,thumb)
				p.setJointMotorControl2(hand,11,p.POSITION_CONTROL,thumb)
				
				p.setJointMotorControl2(hand,15,p.POSITION_CONTROL,index)
				p.setJointMotorControl2(hand,17,p.POSITION_CONTROL,index)
				p.setJointMotorControl2(hand,19,p.POSITION_CONTROL,index)

				p.setJointMotorControl2(hand,22,p.POSITION_CONTROL,middle)
				p.setJointMotorControl2(hand,24,p.POSITION_CONTROL,middle)
				p.setJointMotorControl2(hand,26,p.POSITION_CONTROL,middle)
				
				p.setJointMotorControl2(hand,38,p.POSITION_CONTROL,pink)
				p.setJointMotorControl2(hand,40,p.POSITION_CONTROL,pink)
				p.setJointMotorControl2(hand,42,p.POSITION_CONTROL,pink)

				ringpos = 0.5*(pink+middle)
				p.setJointMotorControl2(hand,30,p.POSITION_CONTROL,ringpos)
				p.setJointMotorControl2(hand,32,p.POSITION_CONTROL,ringpos)
				p.setJointMotorControl2(hand,34,p.POSITION_CONTROL,ringpos)
								
				#print(middle)
				#print(pink)
				#print(index)
				#print(thumb)