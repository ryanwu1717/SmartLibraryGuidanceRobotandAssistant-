import ScanUtility
import bluetooth._bluetooth as bluez
import math
import time
import numpy as np
import pylab
import threading


txpower = -64
tStart = time.time()
arr1 = []
arr2 = []
arr3 = []
arr4 = []
 
filterarr1 = []
filterarr2 = []
filterarr3 = []
filterarr4 = []

filterDict = {'33ac1549-8924-456e-a468-3297f43adfd2': [],
         'e2c56db5-dffb-48d2-b060-d0f5a71096e0': [], 
         '778d4ded-c660-48fb-b476-053008aba325': [],
         'c007ec27-169d-465d-8bd2-1059af0ff693': []}

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

arrLen = 20
while True:
  try:
    tmpObj =  inLoop()
    # print(tmpObj)
    if "type" in tmpObj:
      # print(tmpObj['uuid'] )
      if tmpObj['uuid'] == "e2c56db5-dffb-48d2-b060-d0f5a71096e0":
        tmpNum1 = calculateDistance(tmpObj['rssi'])
        if not tmpNum1 is None:
          # print(len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
          # filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']
          if len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) == arrLen:
            filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].pop(0)
            filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)
            returnFilter1=int(checkRssi(tmpNum1,filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']))
            arr1.append(returnFilter1)
            
          elif len(filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0']) < arrLen:
            filterDict['e2c56db5-dffb-48d2-b060-d0f5a71096e0'].append(tmpNum1)


          # checkRssi(tmpNum1)
        # print(calculateDistance(tmpObj['rssi']))
      elif tmpObj['uuid'] == "778d4ded-c660-48fb-b476-053008aba325":
        tmpNum2 = calculateDistance(tmpObj['rssi'])
        if not tmpNum2 is None:
          if len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) == arrLen:
            
            filterDict['778d4ded-c660-48fb-b476-053008aba325'].pop(0)
            filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)
            returnFilter2=int(checkRssi(tmpNum2,filterDict['778d4ded-c660-48fb-b476-053008aba325']))

            
          elif len(filterDict['778d4ded-c660-48fb-b476-053008aba325']) < arrLen:
            filterDict['778d4ded-c660-48fb-b476-053008aba325'].append(tmpNum2)


      elif tmpObj['uuid'] == "33ac1549-8924-456e-a468-3297f43adfd2":
        tmpNum3 = calculateDistance(tmpObj['rssi'])
        if not tmpNum3 is None:
          if len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) == arrLen:
            
            filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].pop(0)
            filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)

            returnFilter3=int(checkRssi(tmpNum3,filterDict['33ac1549-8924-456e-a468-3297f43adfd2']))
            arr3.append(returnFilter3)
            
          elif len(filterDict['33ac1549-8924-456e-a468-3297f43adfd2']) < arrLen:
            filterDict['33ac1549-8924-456e-a468-3297f43adfd2'].append(tmpNum3)
      elif tmpObj['uuid'] == "c007ec27-169d-465d-8bd2-1059af0ff693":
        tmpNum4 = calculateDistance(tmpObj['rssi'])
        if not tmpNum4 is None:
          if len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) == arrLen:
            
            filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].pop(0)
            filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)

            returnFilter4=int(checkRssi(tmpNum4,filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']))
            arr4.append(returnFilter3)
            
          elif len(filterDict['c007ec27-169d-465d-8bd2-1059af0ff693']) < arrLen:
            filterDict['c007ec27-169d-465d-8bd2-1059af0ff693'].append(tmpNum4)


        # prin:t(calculateDistance(tmpObj['rssi']))
    # else:
    #   continue

    tEnd = time.time()
    tmpTime = tEnd-tStart
    # print(tmpTime)
    # print("")

    if (tmpTime > 3):
      tStart = time.time()
      # print(arr1)
      # print(arr2)
      # print(arr3)
      print('beacon1',returnFilter1)
      print('beacon2',returnFilter2)
      print('beacon3',returnFilter3)
      print('beacon4',returnFilter4)
      arr1 = []
      arr2 = []
      arr3 = []
      arr4 = []
  except Exception as e:
    # print("inE2")
    # print (e)
    pass



