import math
import time
import pypot.dynamixel
import itertools
ports = pypot.dynamixel.get_available_ports()
dxl_io = pypot.dynamixel.DxlIO(ports[0])
def convertcar2pol(x,y):
    r=math.sqrt(x**2+y**2)
    theta=math.atan(y/x)*180/math.pi
    if theta<0:
        theta=theta+180
    return r,theta
def control(s1,alpha,beta,xT,yT,t,theta,s3,s4):

    kp1 = 25*7
    kp2 = 25*7
    kp3 = 5
    kp4 = 5
    
    rT,thetaT=convertcar2pol(xT,yT)
    s2=s1/math.tan(beta*math.pi/180)

    sd=rT*math.tan(math.acos(rT/12))
    #print "thetaT :: ",thetaT
    #calculate errors
    e1_t=s1-rT
    e2_t=s2-sd
    #print e1_t,e2_t
    #error threshold
    if e1_t<0.3 and e2_t<0.2 :
        print "stop!!!!!!!"
        e1_t=0.000001
        e2_t=0.000001
    if e1_t>0:
        kp1=25*7
        kp2=25*7
        
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
    
        
    #print m2,m3
    dxl_io.set_moving_speed(dict(zip([1], itertools.repeat(512))))
    dxl_io.set_moving_speed(dict(zip([2], itertools.repeat(512))))
    dxl_io.set_moving_speed(dict(zip([3], itertools.repeat(512))))
    dxl_io.set_moving_speed(dict(zip([4], itertools.repeat(512))))
    a2=dxl_io.get_present_position(dict(zip([2], itertools.repeat(500))))
    a3=dxl_io.get_present_position(dict(zip([3], itertools.repeat(500))))
    dxl_io.set_goal_position(dict(zip([2], itertools.repeat(a2[0]+m2))))
    dxl_io.set_goal_position(dict(zip([3], itertools.repeat(a3[0]+m3))))

    #angle control
    etheta=theta-thetaT
    kp3=30
    kp4=50
    ke  = 70 #temporary expansion
    ke2 = 70
    #controllers
    s3_t1 = s3 + kp3*etheta*t + ke*(e2_t)*t - 3 - ke2*(e1_t)*t
    s4_t1 = s4 - kp4*etheta*t + ke*(e2_t)*t - 3 - ke2*(e1_t)*t
    
    #print etheta
    a1=dxl_io.get_present_position(dict(zip([1], itertools.repeat(500))))
    a4=dxl_io.get_present_position(dict(zip([4], itertools.repeat(500))))


     #motors command
    m1 = (s3_t1-s3)/2.1964567
    m4 = (s4_t1-s4)/2.1964567
    dxl_io.set_goal_position(dict(zip([1], itertools.repeat(a1[0]+m1))))
    dxl_io.set_goal_position(dict(zip([4], itertools.repeat(a4[0]-m4))))
    #print m1,m2,m3,m4
    if m4>0:
        kp3=50
        kp3=30
        print "swapped"

    
    time.sleep(0.02)
    #print rT,sd
#print convertcar2pol(10,10)
    
