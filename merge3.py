import ScanUtility
import bluetooth._bluetooth as bluez
import math
import time
import numpy as np
import pylab
from hcsr04sensor import sensor
import time
import RPi.GPIO as GPIO
import threading
import smbus    #import SMBus module of I2C
#import Astar
from astar_python.astar import Astar


lock = threading.Lock()

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 16
GPIO_ECHO = 12
GPIO_ENA = 20
GPIO_PosR = 6
GPIO_NegR = 13
GPIO_PosL = 19
GPIO_NegL = 26
GPIO_ENB = 21
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
p = GPIO.PWM(GPIO_ENA, 500)
q = GPIO.PWM(GPIO_ENB, 500)

#set SR04 sensor
sr04 = sensor.Measurement(GPIO_TRIGGER, GPIO_ECHO)


txpower = -64
tStart = time.time()

arrLen = 20
filterDict = {'1': [],
         '2': [], 
         '3': [],
         '4': []}
# filterDict = {'33ac1549-8924-456e-a468-3297f43adfd2': [],
#        'e2c56db5-dffb-48d2-b060-d0f5a71096e0': [], 
#        '5a5a5a5a-5a5a-5a5a-5a5a-5a5a5a5a5a5a': [],a
#        'c007ec27-169d-465d-8bd2-1059af0ff693': []}
returnFilter1 = 0
returnFilter2 = 0
returnFilter3 = 0
returnFilter4 = 0
checkFilter = bool(False)
beaconPoint = {'0': [0,0],
              '1': [1,1], 
              '2': [4,2],
              '3': [2,2]}
arr1=[]
arr2=[]
arr3=[]

Register_A     = 0              #Address of Configuration register A
Register_B     = 0x01           #Address of configuration register B
Register_mode  = 0x02           #Address of mode register

X_axis_H    = 0x03              #Address of X-axis MSB data register
Z_axis_H    = 0x05              #Address of Z-axis MSB data register
Y_axis_H    = 0x07              #Address of Y-axis MSB data register
declination = -0.07098          #define declination angle of location where measurement going to be done
x_offset = 172
y_offset = -83
scale = 0.92
x_offset = 0
y_offset = 0
scale = 1
  #Set declination angle on your location and fix heading
  #You can find your declination on: http://magnetic-declination.com/
  #(+) Positive or (-) for negative
  #台北地區的磁偏角是 4'4 WEST (negative)
  #Formula: (deg + (min / 60.0)) / (180 / M_PI);
pi          = 3.14159265359     #define pi value
curr_angle = 180
target_angle = 180
standard_north = 90
moving = False
standards = [74,166,263,347] #N,E,S,W
obst_dist = 200
"""
grid =  [
[0   ,0   ,None,None,None,None,None,None,None,None],
[None,0   ,None,None,None,None,None,None,None,None],
[None,0   ,0   ,0   ,None,None,None,None,None,None],
[None,None,None,0   ,0   ,None,None,None,None,None],
[None,None,None,None,0   ,0   ,None,None,None,None],
[None,None,None,None,None,0   ,0   ,None,None,None],
[None,None,None,None,None,None,0   ,None,None,None],
[None,None,None,None,None,None,0   ,0   ,0   ,None],
[None,None,None,None,None,None,None,None,0   ,None],
[None,None,None,None,None,None,None,None,0   ,0   ]
]
"""
"""
grid = [
[0,0,0],
[0,None,0],
[0,0,0]
]
"""
grid = [
[0,0],
[0,None]
]
"""
grid = [
[0,0,0,0,0],
[0,None,None,None,0],
[0,0,0,0,0],
[0,None,None,None,0],
[0,0,0,0,0]
]
"""
beacon_loc = [
[3,2],
[1,4]
]
beacon_grid = [
[1,-1,2],
[-1,-1,-1],
[3,-1,4]
]

route = []
curr_pos = [1,0]
tar_pos = [0,1]


def Magnetometer_Init():
        #write to Configuration Register A
        bus.write_byte_data(Device_Address, Register_A, 0x7C)

        #Write to Configuration Register B for gain
        bus.write_byte_data(Device_Address, Register_B, 0xa0)

        #Write to mode Register for selecting mode
        bus.write_byte_data(Device_Address, Register_mode, 0)
  
  

def read_raw_data(addr):
    
        #Read raw 16-bit value
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from module
        if(value > 32768):
            value = value - 65536
        return value
def read_angle():
        #x = read_raw_data(X_axis_H)
        #z = read_raw_data(Z_axis_H)
        #y = read_raw_data(Y_axis_H)
        x = (read_raw_data(X_axis_H)-x_offset)*scale
        z = read_raw_data(Z_axis_H)
        y = (read_raw_data(Y_axis_H)-y_offset)*scale
        global curr_angle

        heading = math.atan2(y, x) + declination
        
        #Due to declination check for >360 degree
        if(heading > 2*pi):
                heading = heading - 2*pi

        #check for sign
        if(heading < 0):
                heading = heading + 2*pi

        #convert into angle
        heading_angle = int(heading * 180/pi)
        curr_angle = heading_angle
        #print ("Heading Angle = %d°" %curr_angle)

#funtion code 
def stop():
    print('stop action')
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)
    print('stop over')

def forward(tar_pos):
    print('forward action')
    target_beacon = beacon_loc[tar_pos[0]][tar_pos[1]]
    tmpRSSI = -100
    #return
    #GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    #GPIO.output(GPIO_NegL, True)
    time.sleep(0.2)
    q.start(100)
    p.start(100)
    runcount = 0 #TMP
    runcount2 = 0 #TMP
    RSSI_count = 0 #TMP
    global obst_dist 
    while True:
      angle_diff = curr_angle - target_angle
      p_s = 100
      q_s = 100
      q.ChangeDutyCycle(q_s)
      p.ChangeDutyCycle(p_s)
      mode = 1
      #print(obst_dist)
      if(obst_dist < 80):
        #print(obst_dist)
        obstacleAvoidance()
      if(angle_diff > 180): 
        angle_diff = angle_diff - 360
      if(angle_diff < -180): 
        angle_diff = angle_diff + 360
      if(abs(angle_diff) < 5):#1
        #print("Straight")
        if(mode == 2):
          q.ChangeDutyCycle(q_s)
        if(mode == 3):
          p.ChangeDutyCycle(p_s)
        mode = 1
        #p_value = 100
        #q_value = 100
      elif(angle_diff > 0):#2
        #print("Truning Left")
        if(mode == 1):
          q.ChangeDutyCycle(q_s-10)
        if(mode == 3):
          p.ChangeDutyCycle(p_s)
          q.ChangeDutyCycle(q_s-10)
        mode = 2
        #p_value = p_value -10
        #q_value = 100
      elif(angle_diff < 0):#3
        #print("Turning Right")
        if(mode == 1):
          p.ChangeDutyCycle(p_s-10)
        if(mode == 2):
          q.ChangeDutyCycle(q_s)
          p.ChangeDutyCycle(p_s-10)
        mode = 3
        #p_value = 100
        #q_value = q_value -10
      runcount += 1
      tmpRSSI = -100
      try:
        tmpObj =  inLoop()
        #print(tmpObj)
        if "type" in tmpObj:
          if(tmpObj['uuid'] == "c0b6a76b-ec47-4c30-bfbb-1b435f5ce2f8"):
            if tmpObj['major'] == target_beacon:
              if (tmpObj['rssi']) is not None:
                tmpRSSI = (tmpObj['rssi'])
      except Exception as e:
        pass
      
      runcount += 1
      time.sleep(0.01)
      #print(target_beacon)
      if(tmpRSSI > -50):
        print('reached {bcn}'.format(bcn=target_beacon))
        RSSI_count += 1
      if(RSSI_count > 0):
        break
      if(runcount >= 10): 
        print(tmpRSSI)
        print(curr_angle)
        print(target_angle)
        runcount2 += 1
        runcount = 0
      if(runcount2 > 50):
        break

    print('forward stop')
    stop()
    p.ChangeDutyCycle(p_s)
    q.ChangeDutyCycle(q_s)

def backward(a):
    print('backward action')
    GPIO.output(GPIO_PosR, False)
    #GPIO.output(GPIO_NegR, True)
    GPIO.output(GPIO_PosL, False)
    #GPIO.output(GPIO_NegL, True)
    if(a == 0):
        time.sleep(0.75)
    else:
        time.sleep(a)
    print('backward stop')
    stop()

def turnLeft(a):
    print('actionL')
    #return
    GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, True)
    GPIO.output(GPIO_NegL, False)
    p_s = 50
    q_s = 50
    q.ChangeDutyCycle(q_s)
    p.ChangeDutyCycle(p_s)
    #------------Temporary measure
    global target_angle
    #target_angle = curr_angle - 90

    print("Start:%d" %curr_angle)
    """
    if(target_angle > 360):
      target_angle = target_angle - 360
    if(target_angle < 0):
      target_angle = target_angle + 360
    """
    print("Target:%d" %target_angle)
    while True:
      #time.sleep(0.1)
      #print("curr_angle {angle}".format(angle=curr_angle))
      if(abs(curr_angle - target_angle) < 10):
        break
    print("Left target achieved")
    stop()
    global moving
    moving = False
    print('overL')
def turnRight(a):
    print('actionR')
    #return
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, True)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, True)
    p_s = 50
    q_s = 50
    q.ChangeDutyCycle(q_s)
    p.ChangeDutyCycle(p_s)
    #------------Temporary measure
    global target_angle
    #target_angle = curr_angle + 90

    print("Start:%d" %curr_angle)
    """
    if(target_angle > 360):
      target_angle = target_angle - 360
    if(target_angle < 0):
      target_angle = target_angle + 360
    """
    print("Target:%d" %target_angle)
    while True:
      #time.sleep(0.1)
      #print("curr_angle {angle}".format(angle=curr_angle))
      if(abs(curr_angle - target_angle) < 10):
        break
    print("Right target achieved")
    stop()
    global moving
    moving = False
    print('overR')

def turnbackLeft(a):
    print('actionL')
    GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)
    if(a==0):
        time.sleep(1)
    else:
        time.sleep(a)
    stop()
    print('overL')
def autoTurn():
    global target_angle
    if(target_angle > 360):
      target_angle = target_angle - 360
    if(target_angle < 0):
      target_angle = target_angle + 360
    a_diff = curr_angle - target_angle
    print(a_diff)
    global moving
    moving = True
    if(a_diff < 0):
      turnRight(0)
    else:
      turnLeft(0)
    while moving:
      time.sleep(0.01)
def obstacleAvoidance():
    #print("Avoid")
    flag = False
    p_s = 15
    q_s = 15
    global curr_angle
    target_angle = curr_angle

    GPIO.output(GPIO_PosR, False)
    #GPIO.output(GPIO_NegR, True)
    GPIO.output(GPIO_PosL, False)
    #GPIO.output(GPIO_NegL, True)
    q.ChangeDutyCycle(q_s)
    p.ChangeDutyCycle(p_s)
    while True:
      print("try right")
      print(obst_dist)
      time.sleep(1)
      if(obst_dist > 100):
        flag = True
        print("go right")
        break
      if((curr_angle - target_angle) > 90):
        print("no way pass on the right")
        break
    if(not flag):
      #GPIO.output(GPIO_PosR, True)
      GPIO.output(GPIO_NegR, False)
      #GPIO.output(GPIO_PosL, True)
      GPIO.output(GPIO_NegL, False)
      q.ChangeDutyCycle(q_s)
      p.ChangeDutyCycle(p_s)
      while True:
        print("try left")
        print(obst_dist)
        time.sleep(1)
        if(obst_dist > 100):
          flag = True
          print("go Left")
        if((curr_angle - target_angle) < -90):
          print("no way pass on the Left")
          break
      


def d1():
    try:
      global sr04
  #    print(sr04)
      raw_measurement = sr04.raw_distance(sample_wait = 0.03,sample_size = 5)
  #    print(raw_measurement)
      distance = sr04.distance_metric(raw_measurement)
  #    print(distance)
      #print('distance is {:.1f} cm'.format(distance))
      #time.sleep(1)
      return distance
    except:
      return 200
      print('SR04 Exception')

#main

def fix():
    # turnbackLeft(1.0)
    # turnLeft(0)
    backward(0)

    forward(0)

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
      return int(ans2*100)
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
        #print(item)
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
  sz = 20 # 数据量

  # x = 0.1  # 真实值
  # z = np.random.normal(x, 0.1, size=sz)  # 观测值，服从高斯分布
  Q = 1e-5  # 过程噪声
  R = 1e-2  # 观测噪声

  # 为变量分配空间
  x_predict = np.zeros(sz)  # x的先验估计，也就是预测值
  P_predict = np.zeros(sz)  # P的先验估计
  x_update = np.zeros(sz)  # x的后验估计，也就是最终的估计量
  P_update = np.zeros(sz)  # 协方差的后验估计
  K = np.zeros(sz)  # 卡尔曼增益

  # 赋初值
  x_update[0] = 0.0
  P_update[0] = 1.0

  for k in range(1, sz):
      # 预测过程
      x_predict[k] = x_update[k - 1]
      P_predict[k] = P_update[k - 1] + Q

      # 更新过程
      K[k] = P_predict[k] / (P_predict[k] + R)
      x_update[k] = x_predict[k] + K[k] * (z[k] - x_predict[k])
      P_update[k] = (1 - K[k]) * P_predict[k]

  pylab.rcParams['font.sans-serif'] = ['FangSong']  # 指定默认字体
  pylab.rcParams['axes.unicode_minus'] = False  # 解决保存图像是负号'-'显示为方块的问题

  pylab.figure()
  pylab.plot(z, 'k+', label='观测值')  # 观测值
  pylab.plot(x_update, 'b-', label='估计值')  # 估计值
  pylab.axhline(x, color='g', label='真实值')  # 真实值
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

#todo, get position from beacon with no interference
def getPosition(lenArr):
  # print(beaconPoint['0'][0],beaconPoint['0'][1])
  # tmpNode=insec(beaconPoint['0'][0],beaconPoint['0'][1],lenArr[0],beaconPoint['1'][0],beaconPoint['1'][1],lenArr[0])
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


def twodis(array1,array2):
  p1=np.array(array1)
  p2=np.array(array2)
  p3=p2-p1
  p4=math.hypot(p3[0],p3[1])
  print('distance',p4)

  return p4

def findNear(node,point1,point2,lenArr):
  print(node,point1,point2)
  try:
    for i in range(0,4,1):
      if i != point1 and i != point2:
        print(i)
        tmpDis1 = twodis(node[0],beaconPoint[str(i)])
        tmpDis2 = twodis(node[1],beaconPoint[str(i)])
        tmpDis1 = abs(tmpDis1-lenArr[i])
        tmpDis2 = abs(tmpDis2-lenArr[i])
        # print(tmpDis1,tmpDis2)
        if(tmpDis1<tmpDis2):
          lastNode = node[0]
        else:
          lastNode = node[1]
        tmpPosition = lastNode
        print(i,lenArr)

        print(tmpPosition)

        # twodis(node[0],beaconPoint[str(i)])

        return
  except  Exception as e:
    # print(e)
    pass
def job():
  global curr_pos,tar_pos
  global standards
  global target_angle
  p.start(100)
  q.start(100)
  time.sleep(1)
  target_angle = standards[0]
  #turnRight(0)
  forward([0,0])
  return
  #print(curr_angle)
  #forward([0,1])
  #forward(0)
  #turnLeft(0)
  #forward(0)
  #sleep(5)
  mode = -1; #0 = standby, -1 = testrun, 1 = Setup Route, 2 = No route, 3 = running
  # stop()
  # return
  #stop()
  while True:
    #forward(0.4)
    #print(tmpRSSI)
    #read_angle()
    if(mode == -1):
      print("Testrun")
      astar = Astar(grid,False)
      route = astar.run(curr_pos, tar_pos)
      #print(route)
      mode = 1
      if(route == None):
        mode = 2
    if(mode == 1):
      node_iter = iter(route)
      print(route)
      mode = 3
      #nextnode = next(node_iter,[-1,-1])
    if(mode == 2):
      print("No route")
      time.sleep(1)
    if(mode == 3):
      curr_pos = next(node_iter,[-1,-1])
      while True:
        print("----------")
        print("Moving")
        print("----------")
        print("now at {coord}".format(coord=curr_pos))
        print("beacon is {beacon}".format(beacon=beacon_loc[curr_pos[0]][curr_pos[1]]))
        nextnode = next(node_iter,[-1,-1])
        if(nextnode == [-1,-1]):
          mode = 4
          break
        print("----------")
        if(curr_pos[1] == nextnode[1]):
          if(nextnode[0] > curr_pos[0]):
            #target_angle = standard_north+180
            target_angle = standards[2]
            print("Go South,{ang}".format(ang=target_angle))
          else:
            #target_angle = standard_north
            target_angle = standards[0]
            print("Go North,{ang}".format(ang=target_angle))
        else:
          if(nextnode[1] > curr_pos[1]):
            #target_angle = standard_north-90
            target_angle = standards[1]
            print("Go East,{ang}".format(ang=target_angle))
          else:
            #target_angle = standard_north+90
            target_angle = standards[3]
            print("Go West,{ang}".format(ang=target_angle))
        if(abs(target_angle - curr_angle) > 30):
          autoTurn()
          print("turning...")
        print("----------")
        print("Go Forward")
        print("----------")
        forward(nextnode)
        curr_pos = nextnode
    if(mode == 4):
      print("target reached")
      break

    if (-40) < tmpRSSI < (-27):
      print('in')
      stop()
      break
  return
def heading():
  while True:
    read_angle()
    time.sleep(0.01)
    pass
  #   if len(tmpPosition) != 0:
  #     print(tmpPosition)
    
 
  # p.start(80)
  # q.start(80)
  # # fix()
  # print('inJob')
  # for i in range(3):
  #   forward(0);
  # turnRight(0);
  # for i in range(3):
  #   forward(0);
  # turnLeft(0);
  # for i in range(3):
  #   forward(0);
def obstacle():
  global obst_dist
  while True:
    obst_dist = d1()
    #print(obst_dist)
    pass
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x1e   # HMC5883L magnetometer device address

Magnetometer_Init()     # initialize HMC5883L magnetometer 
t = threading.Thread(target = job)
t.start()
t2 = threading.Thread(target = heading)
t2.start()
t3 = threading.Thread(target = obstacle)
t3.start()

tmpRSSI = -100
while True:
  try:
    tmpObj =  inLoop()
    if "type" in tmpObj:
      if(tmpObj['uuid'] == "c0b6a76b-ec47-4c30-bfbb-1b435f5ce2f8"):
        if tmpObj['major'] == 1:
          if (tmpObj['rssi']) is not None:
            tmpRSSI = (tmpObj['rssi'])
            #print(tmpRSSI)
          # if not tmpNum1 is None:
          #   if len(filterDict['1']) == arrLen:
          #     filterDict['1'].pop(0)
          #     filterDict['1'].append(tmpNum1)
          #     returnFilter1=int(checkRssi(tmpNum1,filterDict['1']))
          #     checkFilter = bool(True)
          #   elif len(filterDict['1']) < arrLen:
          #     filterDict['1'].append(tmpNum1)

  except Exception as e:
    pass

t.join()
