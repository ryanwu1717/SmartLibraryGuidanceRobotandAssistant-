import math
import sympy
import numpy as np
import time
import RPi.GPIO as GPIO


import ScanUtility
import bluetooth._bluetooth as bluez
import pylab

#from numpy import *
from math import acos,pi
from math import sqrt
from decimal import Decimal,getcontext
from hcsr04sensor import sensor

#define setting
getcontext().prec = 30
# BeaconX = [1,2,3] #beacon coordinate X
# BeaconY = [2,3,2] #beacon coordinate Y
# mtx = 0 #car x distance
# mty = 0 #car y distance
# CoordinateX = 0 #original coordinate X 
# CoordinateY = 0 #original coordinate Y
# CoordinateX2 = 0 #next coordinate X
# CoordinateY2 = 0 #next coordinate X
# TargetX = 0 #target coordinate X
# TargetY = 0 #target coordinate Y
StepTime = 2 #one step spend time
TurnTime = 2 #turn a round spend time
v = [] #vector original
w = [] #next Vector

# da = 1 # math.sqrt(np.square(mtx-BeaconX[0])+np.square(mty-BeaconY[0]))
# db = 1 # math.sqrt(np.square(mtx-BeaconX[1])+np.square(mty-BeaconY[1])) 
# dc = 1 # math.sqrt(np.square(mtx-BeaconX[2])+np.square(mty-BeaconY[2]))

txpower = -64
tStart = time.time()
 
filterarr1 = []
filterarr2 = []
filterarr3 = []
filterarr4 = []

filterDict = {'33ac1549-8924-456e-a468-3297f43adfd2': [],
         'e2c56db5-dffb-48d2-b060-d0f5a71096e0': [], 
         '778d4ded-c660-48fb-b476-053008aba325': [],
         'c007ec27-169d-465d-8bd2-1059af0ff693': []}
beaconPoint = {'0': [3,1],
         '1': [0,3], 
         '2': [1,0],
         '3': [4,0]}


arrLen = 20
returnFilter1 = 0
returnFilter2 = 0
returnFilter3 = 0
returnFilter4 = 0
checkFilter = bool(False)
keypoint1 = []
keypoint2 = []

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 25
GPIO_ECHO = 8
GPIO_ENA = 27
GPIO_PosR = 18
GPIO_NegR = 17
GPIO_PosL = 23
GPIO_NegL = 22
GPIO_ENB = 24
distance = 0

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_PosR, GPIO.OUT)
GPIO.setup(GPIO_NegR, GPIO.OUT)
GPIO.setup(GPIO_PosL, GPIO.OUT)
GPIO.setup(GPIO_NegL, GPIO.OUT)
GPIO.setup(GPIO_ENA, GPIO.OUT)
GPIO.setup(GPIO_ENB, GPIO.OUT)

#set GPIO PWM frenquence
p = GPIO.PWM(GPIO_ENA, 1000)
q = GPIO.PWM(GPIO_ENB, 1000)

# def RssitoDis():
# ========================================================

#funtion code

#MOVE funtion
def stop():
    print('stop action')
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)
    print('stop over')

def forward():
    print('forward action')
    GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, True)
    GPIO.output(GPIO_NegL, False)
#    time.sleep(1)
#    print('forward stop')
#    stop()

def backward():
    print('backward action')
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, True)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, True)
    time.sleep(2)
    print('backward stop')
    stop()

def turnRight():
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, True)
    GPIO.output(GPIO_NegL, False)
    time.sleep(0.5)
    stop()

def turnLeft():
    GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)
    time.sleep(0.5)
    stop()

#==========================================================


#hc_sr04 sensor funtion
def d1():
    sr04 = sensor.Measurement(GPIO_TRIGGER, GPIO_ECHO)
#    print(sr04)
    raw_measurement = sr04.raw_distance()
#    print(raw_measurement)
    distance = sr04.distance_metric(raw_measurement)
#    print(distance)
    print('distance is {:.1f} cm'.format(distance))
    #time.sleep(1)
    return distance


def triposition(xa,ya,da,xb,yb,db,xc,yc,dc): 
    x,y = sympy.symbols('x y')
    f1 = 2*x*(xa-xc)+np.square(xc)-np.square(xa)+2*y*(ya-yc)+np.square(yc)-np.square(ya)-(np.square(dc)-np.square(da))
    f2 = 2*x*(xb-xc)+np.square(xc)-np.square(xb)+2*y*(yb-yc)+np.square(yc)-np.square(yb)-(np.square(dc)-np.square(db))
    result = sympy.solve([f1,f2],[x,y])
    #print f1
    #print f2
    locx,locy = result[x],result[y]
    #print locx
    #print locy
    return [locx,locy]

# def Target(a,b):
# def forward():
# 	global da, db, dc
# 	da = da + 0.414
# 	db = db + 1.000
# 	dc = dc + 0.414	

# 	#print da, db, dc 
# 	return da, db, dc 


def d2():
    [locx,locy] = triposition(BeaconX[0],BeaconY[0],da,BeaconX[1],BeaconY[1],db,BeaconX[2],BeaconY[2],dc)
	
    global CoordinateX, CoordinateY, CoordinateX2, CoordinateY2

    CoordinateX = locx
    CoordinateY = locy

    forward()
	#print da, db, dc

    [locx2,locy2] = triposition(BeaconX[0],BeaconY[0],da,BeaconX[1],BeaconY[1],db,BeaconX[2],BeaconY[2],dc)

    CoordinateX2 = locx2
    CoordinateY2 = locy2


    return CoordinateX, CoordinateY, CoordinateX2, CoordinateY2

def Coordinate_to_Vector():
    global v, w
    v = [int(keypoint1[0] - TargetX), int(keypoint1[1] - TargetY)]
    w = [int(keypoint2[0] - TargetX), int(keypoint2[1] - TargetY)]

	#print "CtVs"
	#print v, w
	#print "CtVe"
    return v, w

def angle():
    global v, w
    x = np.array(v,dtype=np.float64)
    y = np.array(w,dtype=np.float64)
	#print x
	#print y

    Lx = np.sqrt(x.dot(x))
    Ly = np.sqrt(y.dot(y))

    cos_angle = x.dot(y) / (Lx*Ly)

    angle = np.arccos(cos_angle)
    angle2 = angle * 360/2/np.pi

	#print angle2

    return angle2





#raynwu tri position def

def mode(l):
  count_dict={};
  for i in l:
    if i in count_dict:
        count_dict[i]+=1;
    else:
        count_dict[i]=1;
  max_appear=0
  for v in count_dict.values():
    if v>max_appear:
        max_appear=v;
  if max_appear==1:
    return;
  mode_list=[];
  for k,v in count_dict.items():
    if v==max_appear:
        mode_list.append(k);
  return mode_list;

def calculateDistance(rssi):
  try:
    txPower = -63
    if rssi == 0 :
      return -1

    ratio = rssi*1.0 / txpower
    if ratio < 1.0:
      ans= math.pow(ratio, 10)
      return int(ans*100)
    else :
      ans2=(0.89976  (ratio ** 7.7095)) + 0.111
      return int(ans*100)
  except KeyboardInterrupt:
    pass
  except Exception as e:
    # print("inE3")
    # print (e)
    pass
#Set bluetooth device. Default 0.
dev_id = 0

def inLoop():
  try:
    returnedList = ScanUtility.parse_events(sock, 10)
    if not returnedList is None:
      for item in returnedList:
        # print(item)
        # print(item['type'])
        if not item is None:
          if(item['type'] == "iBeacon"):
            return item ;
          else :
            return {}
        else :
          return {}




  except Exception as e:
    # print("inE1")
    # print (e)
    return {}
    pass

try:
      print(dev_id)
      sock = bluez.hci_open_dev(dev_id)
      print(sock)
      print ("\n *** Looking for BLE Beacons ***\n")
      print ("\n *** CTRL-C to Cancel ***\n")
except:
      print ("Error accessing bluetooth")

ScanUtility.hci_enable_le_scan(sock)

#Scans for iBeacons
def checkRssi(x,z):
  sz = 20 

  # x = 0.1  
  # z = np.random.normal(x, 0.1, size=sz) 
  Q = 1e-5 
  R = 1e-2 

  
  x_predict = np.zeros(sz)  
  P_predict = np.zeros(sz)  
  x_update = np.zeros(sz)  
  P_update = np.zeros(sz)  
  K = np.zeros(sz)

  x_update[0] = 0.0
  P_update[0] = 1.0

  for k in range(1, sz):
      x_predict[k] = x_update[k - 1]
      P_predict[k] = P_update[k - 1] + Q

      K[k] = P_predict[k] / (P_predict[k] + R)
      x_update[k] = x_predict[k] + K[k] * (z[k] - x_predict[k])
      P_update[k] = (1 - K[k]) * P_predict[k]

  pylab.rcParams['font.sans-serif'] = ['FangSong']  
  pylab.rcParams['axes.unicode_minus'] = False
  pylab.figure()
  pylab.plot(z, 'k+', label='Observations')  
  pylab.plot(x_update, 'b-', label='estimated')
  pylab.axhline(x, color='g', label='actual')
  pylab.legend()
  pylab.show()
  pylab.close()
  # print(z,x_update)
  # print(x_update[9])
  return(x_update[19])

def insec(x,y,R,a,b,S):
  d = math.sqrt((abs(a-x))**2 + (abs(b-y))**2)
  if d > (R+S) or d < (abs(R-S)):
    # print ("Two circles have no intersection")
    return 'wrong'
  elif d == 0 and R==S :
    # print ("Two circles have same center!")
    return 'wrong'
  else:
    A = (R**2 - S**2 + d**2) / (2 * d)
    h = math.sqrt(R**2 - A**2)
    x2 = x + A * (a-x)/d
    y2 = y + A * (b-y)/d
    x3 = round(x2 - h * (b - y) / d,2)
    y3 = round(y2 + h * (a - x) / d,2)
    x4 = round(x2 + h * (b - y) / d,2)
    y4 = round(y2 - h * (a - x) / d,2)
    print (x3, y3)
    print (x4, y4)
    c1=np.array([x3, y3])
    c2=np.array([x4, y4])
    return c1,c2

   
def getPosition(lenArr):
  # print(beaconPoint['0'][0],beaconPoint['0'][1])
  tmpNode=insec(beaconPoint['0'][0],beaconPoint['0'][1],lenArr[0],beaconPoint['1'][0],beaconPoint['1'][1],lenArr[0])
  # print(lenArr)

  for i in range(0,4,1):
    for j in range(i,4,1):
      # print(beaconPoint[str(i)])
      # print(beaconPoint[i][0],beaconPoint[i][1],lenArr[i],beaconPoint[j][0],beaconPoint[j][1],lenArr[j])
      returnNode = insec(beaconPoint[str(i)][0],beaconPoint[str(i)][1],lenArr[i],beaconPoint[str(j)][0],beaconPoint[str(j)][1],lenArr[j])
      if  returnNode != 'wrong' :
        print(returnNode)
        findNear(returnNode,i,j,lenArr)
        return

      #   print(returnNode)
      # else :
      #   print('failed')


def twodis(arr1,arr2):
  p1=np.array(arr1)
  p2=np.array(arr2)
  p3=p2-p1
  p4=math.hypot(p3[0],p3[1])
  print('distance',p4)

  return p4

def findNear(node,point1,point2,lenArr):
  print(node,point1,point2)
  for i in range(0,4,1):
    if i != point1 and i != point2:
      print(i)
      twodis(node[0],beaconPoint[str(i)])
      twodis(node[1],beaconPoint[str(i)])

      print(i,lenArr)
      # twodis(node[0],beaconPoint[str(i)])

      return




#main 
try:

	distance = d1()
	print(distance)
	p.start(65)
	q.start(65)
	print('ctrl-c to close \n')
	print('the default motor is Forward\n')



	while True:
		try:
		    tmpObj =  inLoop()
		    # print(tmpObj)
		    if "type" in tmpObj:
		      # print('in')
		      # print(tmpObj['uuid'] )
		      if tmpObj['uuid'] == "e2c56db5-dffb-48d2-b060-d0f5a71096e0":
		        tmpNum1 = calculateDistance(tmpObj['rssi'])
		        if not tmpNum1 is None:
		          # print(tmpNum1)
		          # print(len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
		          # filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']
		       	  if len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) == arrLen:
		            # print('in')
		            # print(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])

		            filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].pop(0)
		            filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
		            returnFilter1=int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
		            # print('return',int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])))
		            # print('returnFilter1',returnFilter1)
		            checkFilter = bool(True)
		            
		          elif len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) < arrLen:
		            filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
		            # print(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])


		          # checkRssi(tmpNum1)
		        # print(calculateDistance(tmpObj['rssi']))
		      elif tmpObj['uuid'] == "778d4ded-c660-48fb-b476-053008aba325":
		          tmpNum2 = calculateDistance(tmpObj['rssi'])
		          if not tmpNum2 is None:
		            if len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) == arrLen:
		            
		              filterDict['778d4ded-c660-48fb-b476-053008aba325'].pop(0)
		              filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)
		              returnFilter2=int(checkRssi(tmpNum2,filterDict['778d4ded-c660-48fb-b476-053008aba325']))

		              checkFilter = bool(True)

		            
		            elif len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) < arrLen:
		              filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)


		      elif tmpObj['uuid'] == "33ac1549-8924-456e-a468-3297f43adfd2":
		        tmpNum3 = calculateDistance(tmpObj['rssi'])
		        if not tmpNum3 is None:
		          if len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) == arrLen:
		            
		            filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].pop(0)
		            filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)

		            returnFilter3=int(checkRssi(tmpNum3,filterDict['33ac1549-8924-456e-a468-3297f43adfd2']))
		            checkFilter = bool(True)

		            
		          elif len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) < arrLen:
		            filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)
		      elif tmpObj['uuid'] == "c007ec27-169d-465d-8bd2-1059af0ff693":
		        tmpNum4 = calculateDistance(tmpObj['rssi'])
		        if not tmpNum4 is None:
		          if len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) == arrLen:
		            
		            filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].pop(0)
		            filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)

		            returnFilter4=int(checkRssi(tmpNum4,filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']))
		            checkFilter = bool(True)

		            
		          elif len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) < arrLen:
		            filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)


		        # prin:t(calculateDistance(tmpObj['rssi']))
		    # else:
		    #   continue

		    tEnd = time.time()
		    tmpTime = tEnd-tStart
		    # print(tmpTime)
		    # print("")

		    if (tmpTime > 3 and checkFilter == bool(True)):
		      tStart = time.time()
		      tmpArr = [returnFilter1,returnFilter2,returnFilter3,returnFilter4]
		      print(tmpArr)
		      print('in')
		      getPosition(tmpArr)
		    keypoint1 = node[0]

		#print(distance)
		    distance = d1()

		    if distance >= 200:
		        print(distance)
		        p.ChangeDutyCycle(100)
		        q.ChangeDutyCycle(100)
		        forward()


		        tmpObj =  inLoop()
		      # print(tmpObj)
		        if "type" in tmpObj:
		      # print('in')
		      # print(tmpObj['uuid'] )
		          if tmpObj['uuid'] == "e2c56db5-dffb-48d2-b060-d0f5a71096e0":
		            tmpNum1 = calculateDistance(tmpObj['rssi'])
		            if not tmpNum1 is None:
		          # print(tmpNum1)
		          # print(len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
		          # filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']
		       	      if len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) == arrLen:
		            # print('in')
		            # print(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])

		               filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].pop(0)
		               filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
		               returnFilter1=int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
		            # print('return',int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])))
		            # print('returnFilter1',returnFilter1)
		               checkFilter = bool(True)
		            
		            elif len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) < arrLen:
		              filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
		            # print(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])


		          # checkRssi(tmpNum1)
		        # print(calculateDistance(tmpObj['rssi']))
		          elif tmpObj['uuid'] == "778d4ded-c660-48fb-b476-053008aba325":
		            tmpNum2 = calculateDistance(tmpObj['rssi'])
		            if not tmpNum2 is None:
		              if len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) == arrLen:
		            
		                filterDict['778d4ded-c660-48fb-b476-053008aba325'].pop(0)
		                filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)
		                returnFilter2=int(checkRssi(tmpNum2,filterDict['778d4ded-c660-48fb-b476-053008aba325']))

		                checkFilter = bool(True)

		            
		              elif len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) < arrLen:
		                filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)


		          elif tmpObj['uuid'] == "33ac1549-8924-456e-a468-3297f43adfd2":
		            tmpNum3 = calculateDistance(tmpObj['rssi'])
		            if not tmpNum3 is None:
		              if len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) == arrLen:
		            
		                filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].pop(0)
		                filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)

		                returnFilter3=int(checkRssi(tmpNum3,filterDict['33ac1549-8924-456e-a468-3297f43adfd2']))
		                checkFilter = bool(True)

		            
		              elif len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) < arrLen:
		                filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)
		          elif tmpObj['uuid'] == "c007ec27-169d-465d-8bd2-1059af0ff693":
		            tmpNum4 = calculateDistance(tmpObj['rssi'])
		            if not tmpNum4 is None:
		              if len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) == arrLen:
		            
		                filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].pop(0)
		                filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)

		                returnFilter4=int(checkRssi(tmpNum4,filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']))
		                checkFilter = bool(True)

		            
		              elif len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) < arrLen:
		                filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)


		        # prin:t(calculateDistance(tmpObj['rssi']))
		    # else:
		    #   continue

		        tEnd = time.time()
		        tmpTime = tEnd-tStart
		    # print(tmpTime)
		    # print("")

		        if (tmpTime > 3 and checkFilter == bool(True)):
		          tStart = time.time()
		          tmpArr = [returnFilter1,returnFilter2,returnFilter3,returnFilter4]
		          print(tmpArr)
		          print('in')
		          getPosition(tmpArr)
		        keypoint2 = node[0]

		        print("Coordinate_to_Vectorstart")

		        Coordinate_to_Vector()
		
		        print("Coordinate_to_Vectorend")

		        print("anglestart")

		        angle()

		        print("angleend")

		        if (v == w):
		            forward()
		            time.sleep(StepTime)

		        else :
		            turnRight()
		            time.sleep((angle()/180)*TurnTime)
		            forward()

#            d1()
		    elif distance < 200 and distance >= 130:

		        print(distance)
		        p.ChangeDutyCycle(85)
		        q.ChangeDutyCycle(85)
		        forward()

		        tmpObj =  inLoop()
		    # print(tmpObj)
		        if "type" in tmpObj:
		      # print('in')
		      # print(tmpObj['uuid'] )
		          if tmpObj['uuid'] == "e2c56db5-dffb-48d2-b060-d0f5a71096e0":
		            tmpNum1 = calculateDistance(tmpObj['rssi'])
		            if not tmpNum1 is None:
		          # print(tmpNum1)
		          # print(len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
		          # filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']
		              if len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) == arrLen:
		            # print('in')
		            # print(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])

		                filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].pop(0)
		                filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
		                returnFilter1=int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
		            # print('return',int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])))
		            # print('returnFilter1',returnFilter1)
		                checkFilter = bool(True)
		            
		              elif len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) < arrLen:
		                filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
		            # print(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])


		          # checkRssi(tmpNum1)
		        # print(calculateDistance(tmpObj['rssi']))
		            elif tmpObj['uuid'] == "778d4ded-c660-48fb-b476-053008aba325":
		              tmpNum2 = calculateDistance(tmpObj['rssi'])
		              if not tmpNum2 is None:
		                if len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) == arrLen:
		            
		                  filterDict['778d4ded-c660-48fb-b476-053008aba325'].pop(0)
		                  filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)
		                  returnFilter2=int(checkRssi(tmpNum2,filterDict['778d4ded-c660-48fb-b476-053008aba325']))

		                  checkFilter = bool(True)

		                elif len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) < arrLen:
		                  filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)


		            elif tmpObj['uuid'] == "33ac1549-8924-456e-a468-3297f43adfd2":
		              tmpNum3 = calculateDistance(tmpObj['rssi'])
		              if not tmpNum3 is None:
		                if len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) == arrLen:
		            
		                  filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].pop(0)
		                  filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)
		                  returnFilter3=int(checkRssi(tmpNum3,filterDict['33ac1549-8924-456e-a468-3297f43adfd2']))
		                  checkFilter = bool(True)

		            
		                elif len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) < arrLen:
		                  filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)
		            
		            elif tmpObj['uuid'] == "c007ec27-169d-465d-8bd2-1059af0ff693":
		              tmpNum4 = calculateDistance(tmpObj['rssi'])
		              if not tmpNum4 is None:
		                if len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) == arrLen:
		            
		                  filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].pop(0)
		                  filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)
		                  returnFilter4=int(checkRssi(tmpNum4,filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']))
		                  checkFilter = bool(True)

		            
		                elif len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) < arrLen:
		                  filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)


		        # prin:t(calculateDistance(tmpObj['rssi']))
		    # else:
		    #   continue

		        tEnd = time.time()
		        tmpTime = tEnd-tStart
		    # print(tmpTime)
		    # print("")

		        if (tmpTime > 3 and checkFilter == bool(True)):
		          tStart = time.time()
		          tmpArr = [returnFilter1,returnFilter2,returnFilter3,returnFilter4]
		          print(tmpArr)
		          print('in')
		          getPosition(tmpArr)
		        keypoint2 = node[0]

		        print("Coordinate_to_Vectorstart")

		        Coordinate_to_Vector()
		
		        print("Coordinate_to_Vectorend")

		        print("anglestart")

		        angle()

		        print("angleend")

		        if (v == w):
		            forward()
		            time.sleep(StepTime)

		        else :
		            turnRight()
		            time.sleep((angle()/180)*TurnTime)
		            forward()

			
#            d1()
		    elif distance < 130 and distance >= 70:

		        print(distance)
		        p.ChangeDutyCycle(65)
		        q.ChangeDutyCycle(65)
		        forward()

		        tmpObj =  inLoop()
		    # print(tmpObj)
		        if "type" in tmpObj:
		      # print('in')
		      # print(tmpObj['uuid'] )
		          if tmpObj['uuid'] == "e2c56db5-dffb-48d2-b060-d0f5a71096e0":
		            tmpNum1 = calculateDistance(tmpObj['rssi'])
		            if not tmpNum1 is None:
		          # print(tmpNum1)
		          # print(len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
		          # filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']
		              if len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) == arrLen:
		            # print('in')
		            # print(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])

		                filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].pop(0)
		                filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
		                returnFilter1=int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
		            # print('return',int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])))
		            # print('returnFilter1',returnFilter1)
		                checkFilter = bool(True)
		            
		              elif len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) < arrLen:
		                filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
		            # print(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'])


		          # checkRssi(tmpNum1)
		        # print(calculateDistance(tmpObj['rssi']))
		            elif tmpObj['uuid'] == "778d4ded-c660-48fb-b476-053008aba325":
		              tmpNum2 = calculateDistance(tmpObj['rssi'])
		              if not tmpNum2 is None:
		                if len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) == arrLen:
		            
		                  filterDict['778d4ded-c660-48fb-b476-053008aba325'].pop(0)
		                  filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)
		                  returnFilter2=int(checkRssi(tmpNum2,filterDict['778d4ded-c660-48fb-b476-053008aba325']))

		                  checkFilter = bool(True)

		                elif len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) < arrLen:
		                  filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)


		            elif tmpObj['uuid'] == "33ac1549-8924-456e-a468-3297f43adfd2":
		              tmpNum3 = calculateDistance(tmpObj['rssi'])
		              if not tmpNum3 is None:
		                if len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) == arrLen:
		            
		                  filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].pop(0)
		                  filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)
		                  returnFilter3=int(checkRssi(tmpNum3,filterDict['33ac1549-8924-456e-a468-3297f43adfd2']))
		                  checkFilter = bool(True)

		            
		                elif len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) < arrLen:
		                  filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)
		            
		            elif tmpObj['uuid'] == "c007ec27-169d-465d-8bd2-1059af0ff693":
		              tmpNum4 = calculateDistance(tmpObj['rssi'])
		              if not tmpNum4 is None:
		                if len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) == arrLen:
		            
		                  filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].pop(0)
		                  filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)
		                  returnFilter4=int(checkRssi(tmpNum4,filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']))
		                  checkFilter = bool(True)

		            
		                elif len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) < arrLen:
		                  filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)


		        # prin:t(calculateDistance(tmpObj['rssi']))
		    # else:
		    #   continue

		        tEnd = time.time()
		        tmpTime = tEnd-tStart
		    # print(tmpTime)
		    # print("")

		        if (tmpTime > 3 and checkFilter == bool(True)):
		          tStart = time.time()
		          tmpArr = [returnFilter1,returnFilter2,returnFilter3,returnFilter4]
		          print(tmpArr)
		          print('in')
		          getPosition(tmpArr)
		        keypoint2 = node[0]
		
		        print("Coordinate_to_Vectorstart")

		        Coordinate_to_Vector()
		
		        print("Coordinate_to_Vectorend")

		        print("anglestart")

		        angle()

		        print("angleend")

		        if (v == w):
		            forward()
		            time.sleep(StepTime)

		        else :
		            turnRight()
		            time.sleep((angle()/180)*TurnTime)
		            forward()
#            d1()
		    else:
		        stop()
		        print(distance)
		        p.ChangeDutyCycle(70)
		        q.ChangeDutyCycle(70)
		        backward()
		        time.sleep(2)
		        p.ChangeDutyCycle(65)
		        q.ChangeDutyCycle(65)
		        turnRight()
		        gap1 = d1()
		        turnLeft()
		        turnLeft()
		        gap2 = d1()
		        if gap1 > gap2:
		            turnRight()
		            turnRight()
		except Exception as e:
    # print("inE2")
    # print (e)
		  pass
#            d1()
		# print("d2start")

		# d2()

		# print("d2end")

		# print("Coordinate_to_Vectorstart")
		
		# Coordinate_to_Vector()
	
		# print("Coordinate_to_Vectorend")

		# print("anglestart")

		# angle()
			
		# print("angleend")

		# if (v == w):
		# 	forward()
		# 	time.sleep(StepTime)
		# 	d2()
		

		# else :
		# 	turnRight()
		# 	time.sleep((angle()/180)*TurnTime)
		# 	forward()
		# 	d2()


		


		# locx2-locx = gapx
		# locy2-locy = gapy
		# if locx2**2 > locx**2:
		# 	backward()
		# 	if locy2**2 > locy**2:
		# 		turnRight()
		# 		turnRight()

		# 	else:
		# 		turnRight()
		# elif locx2**2 < locx**2:
		# 	forward()
		# 	if locy2**2 > locy**2:
		# 		turnLeft()

		# 	else:
except KeyboardInterrupt:
    print('closetheapp')
except UnboundLocalError:
    print('Error1')
except SystemError:
    print('Error2')
finally:
    GPIO.cleanup()

# except Exception as e:
# 	raise
# else:
# 	pass
# finally:
# 	pass	
