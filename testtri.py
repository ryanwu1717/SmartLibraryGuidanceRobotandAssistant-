import math
import sympy 
import numpy as np


def triposition(xa,ya,da,xb,yb,db,xc,yc,dc):
    x,y = sympy.symbols('x,y')
    f1 = sympy.Eq(0,2*x*(xa-xc)+xc*xc-xa*xa+2*y*(ya-yc)+yc*yc-ya*ya-(dc*dc-da*da))
    #f2 = sympy.Eq(0,2*x*(xb-xc)+xc*xc-xb*xb+2*y*(yb-yc)+yc*yc-yb*yb-(dc*dc-db*db))
    #f1 = 2*x*(xa-xc)+np.square(xc)-np.square(xa)+2*y*(ya-yc)+np.square(yc)-np.square(ya)-(np.square(dc)-np.square(da))
    #f2 = 2*x*(xb-xc)+np.square(xc)-np.square(xb)+2*y*(yb-yc)+np.square(yc)-np.square(yb)-(np.square(dc)-np.square(db))
    #f1 = sympy.Eq(x + y - 5)
    #f2 = sympy.Eq(x - y + 7)
    result = sympy.solve((f1),(x,y))
    return result 
    #locx,locy = result[x],result[y]
    #return [locx,locy]
def insec(x,y,R,a,b,S):
 d = math.sqrt((abs(a-x))**2 + (abs(b-y))**2)
 if d > (R+S) or d < (abs(R-S)):
  print ("Two circles have no intersection")
  return
 elif d == 0 and R==S :
  print ("Two circles have same center!")
  return
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
def twodis(arr1,arr2):
  p1=np.array(arr1)
  p2=np.array(arr2)
  p3=p2-p1
  p4=math.hypot(p3[0],p3[1])
  return p4
print(triposition(-5,0,5,0,5,5,12,0,12))
twoArr = insec(5,0,5,0,12,12)
print(twoArr)
print(twodis(twoArr[0],[-5,0]))
print(twodis(twoArr[1],[-5,0]))
