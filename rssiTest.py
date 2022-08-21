import ScanUtility
import bluetooth._bluetooth as bluez
import math
import time

txpower = -64
tStart = time.time()
arr1 = []
arr2 = []
arr3 = []

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
    txPower = -59
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
    print("inE3")
    print (e)
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
    print("inE1")
    print (e)
    pass
    return {}

try:
      sock = bluez.hci_open_dev(dev_id)
      print ("\n *** Looking for BLE Beacons ***\n")
      print ("\n *** CTRL-C to Cancel ***\n")
except:
      print ("Error accessing bluetooth")

ScanUtility.hci_enable_le_scan(sock)
#Scans for iBeacons

while True:
  try:
    tmpObj =  inLoop()
    if "type" in tmpObj:
      print(tmpObj)
      if tmpObj['uuid'] == "e2c56db5-dffb-48d2-b060-d0f5a71096e0":
        arr1.append(calculateDistance(tmpObj['rssi']))
        print(calculateDistance(tmpObj['rssi']))

    # elif tmpObj['uuid'] == "c007ec27-169d-465d-8bd2-1059af0ff693":
    #   arr2.append(calculateDistance(tmpObj['rssi']))
    #   # print(calculateDistance(tmpObj['rssi']))
    # elif tmpObj['uuid'] == "33ac1549-8924-456e-a468-3297f43adfd2":
    #   arr3.append(calculateDistance(tmpObj['rssi']))
    #   # print(calculateDistance(tmpObj['rssi']))
    # else:
    #   continue

    tEnd = time.time()
    tmpTime = tEnd-tStart
    print(tmpTime)
    print("")

    # if (tmpTime > 2):
    #   tStart = time.time()
    #   print(mode(arr1))
    #   print(mode(arr2))
    #   print(mode(arr3))
    #   arr1 = []
    #   arr2 = []
    #   arr3 = []
  except Exception as e:
    print("inE2")
    print (e)
    pass
