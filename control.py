import math
import time
import pypot.dynamixel
import itertools
ports = pypot.dynamixel.get_available_ports()
dxl_io = pypot.dynamixel.DxlIO(ports[0])
def convertcar2pol(x,y):
    r=math.sqrt(x**2+y**2)
    theta=math.atan(y/x)*180/math.pi
    return r,theta
def control(s1,alpha,beta,xT,yT,t):
    kp1 = 25*7
    kp2 = 40*7
    kp3 = 0.5
    kp4 = 0.5
    rT,thetaT=convertcar2pol(xT,yT)
    s2=s1/math.tan(beta*math.pi/180)

    sd=rT*math.tan(math.acos(rT/12))
    #calculate errors
    e1_t=s1-rT
    e2_t=s2-sd

    #error threshold
    if e1_t<0.4 and e2_t<0.1 :
        print "stop!!!!!!!"
        e1_t=0.01
        e2_t=0.01
    
    #time
    if(t>100):
        t=0.02
    #print "time :: ",t
    #introduce controller without constraints
    s1_t1 = s1 - kp1*(e1_t)*t - kp3*t*math.fabs((s1*math.tan((alpha/2)*math.pi/180)-s2))
    s2_t1 = s2 - kp2*(e2_t)*t - kp4*t*math.fabs((s1*math.tan((alpha/2)*math.pi/180)-s2))
    #print s1_t1-s1,s2_t1-s2,t
    m3=(s1_t1-s1)/2.2869751969
    m2=-1*(s2_t1-s2)/2.2869751969
    print m2,m3
    a2=dxl_io.get_present_position(dict(zip([2], itertools.repeat(500))))
    a3=dxl_io.get_present_position(dict(zip([3], itertools.repeat(500))))
    dxl_io.set_goal_position(dict(zip([2], itertools.repeat(a2[0]+m2))))
    dxl_io.set_goal_position(dict(zip([3], itertools.repeat(a3[0]+m3))))
    time.sleep(0.3)
    #print rT,sd
#print convertcar2pol(10,10)
