#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16MultiArray
#from geometry_msgs.msg import Accel
import sys, select, termios, tty
ROSCAR_MAX_ACCELL_VEL = 130
ROSCAR_MAX_STEERING_VEL = 180
ROSCAR_MIN_ACCELL_VEL = 99
ROSCAR_MIN_STEERING_VEL = 0
msg = """
Control Your ROSCAR!---------------------------
Moving around:
q    w    e
a    s    d        
z    x
w/s : increase/decrease accell velocity 
a/d : increase/decrease steering velocity 

space key: force stop
q : steering angle : 45
e : steering angle : 135
z : decrease accell velocity 
x : steering init


CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():    
    tty.setraw(sys.stdin.fileno())    
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)    
    if rlist:        
        key = sys.stdin.read(1)    
    else:        
        key = ''    
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)    
    return key

def vels(target_accell_vel, target_steering_vel):
    return "currently:\taccell vel %s\t steering vel %s " % (target_accell_vel,target_steering_vel)
   
    

def constrain(input, low, high):    
    if input < low:      
        input = low    
    elif input > high:      
        input = high    
    else:      
        input = input    
        
    return input

def checkACCELLLimitVelocity(vel):    
    vel = constrain(vel, -ROSCAR_MIN_ACCELL_VEL, ROSCAR_MAX_ACCELL_VEL)   
    return vel

def checkSTEERINGLimitVelocity(vel):    
    vel = constrain(vel, -ROSCAR_MIN_STEERING_VEL, ROSCAR_MAX_STEERING_VEL)    
    return vel

if __name__ == '__main__':    
    settings = termios.tcgetattr(sys.stdin)    
    
    pub = rospy.Publisher('roscar_teleop_cmd_vel', UInt16MultiArray, queue_size=10)    
    rospy.init_node('roscar_teleop',anonymous=True)    
    
    teleop_int = UInt16MultiArray()    
    teleop_int.data = [0,0]        
    
    status = 0    
    target_accell_vel = 90    
    target_steering_vel = 90        
    
    try:
        print msg
        while(1):
            key = getKey()
            if key == 'w' :                
                target_accell_vel +=0.1                 
                target_accell_vel = checkACCELLLimitVelocity(target_accell_vel)                
                status = status + 0.1                
                print vels(target_accell_vel,target_steering_vel)            
            elif key == 's' :                
                target_accell_vel =90                
                #target_accell_vel = checkACCELLLimitVelocity(target_accell_vel)                 
                status = 0                
                print vels(target_accell_vel,target_steering_vel)            
            elif key == 'a' :                
                target_steering_vel +=1                
                target_steering_vel = checkSTEERINGLimitVelocity(target_steering_vel)                
                status = status + 1                
                print vels(target_accell_vel,target_steering_vel)            
            elif key == 'd' :                
                target_steering_vel -=1                
                target_steering_vel = checkSTEERINGLimitVelocity(target_steering_vel)                
                status = status + 1                
                print vels(target_accell_vel,target_steering_vel)            
            elif key == ' '  :                
                target_accell_vel   = 0                
                target_steering_vel  = 90                
                print vels(target_accell_vel, target_steering_vel)            
            elif key == 'x' :                
                target_steering_vel = 90
                target_steering_vel = checkSTEERINGLimitVelocity(target_steering_vel)                
                status = status + 1                
                print vels(target_accell_vel,target_steering_vel)
                
            elif key == 'q' :
                target_steering_vel = 45
                target_steering_vel = checkSTEERINGLimitVelocity(target_steering_vel)                
                status = status + 1                
                print vels(target_accell_vel,target_steering_vel)
                
            elif key == 'e' :
                target_steering_vel = 135
                status = status + 1
                target_steering_vel = checkSTEERINGLimitVelocity(target_steering_vel)
                print vels(target_accell_vel,target_steering_vel)
            
            elif key == 'z' :                
                target_accell_vel -=1                
                target_accell_vel = checkSTEERINGLimitVelocity(target_accell_vel)                
                status = status + 1                
                print vels(target_accell_vel,target_steering_vel)    
                                
            elif target_accell_vel > 255   :                
                target_accell_vel = 254            
            elif target_accell_vel < 0   :                
                target_accell_vel = 1            
            else:                
                if (key == '\x03'):                    
                    break            
                
            if status == 20 :                
                print msg                
                status = 0         
                   
            teleop_int.data[0] = target_accell_vel            
            teleop_int.data[1] = target_steering_vel            
            pub.publish(teleop_int)
            
                
    except rospy.ROSInterruptException:
        pass
    finally:
        pub.publish(teleop_int)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
