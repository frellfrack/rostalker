#!/usr/bin/env python3

# Revision $Id$

## Simple speed controller using tkinter slider that publishes std_msgs/Int16
## to the 'speed' topic and Spinboxs that publish std_msgs/Int32 on kp and ki 
## topic

import rospy
from std_msgs.msg import Int16,Float32
from tkinter import *

class speedcontroller:
    def __init__(self):
    
        self.pubSpeed = rospy.Publisher('speed', Int16, queue_size=10)
        self.pubkp = rospy.Publisher('kp', Float32, queue_size=10)
        self.pubki = rospy.Publisher('ki', Float32, queue_size=10)

        self.subActual_speed =  rospy.Subscriber("actual_speed", Int16, self.getActual)

        rospy.init_node('Speed_Controller', anonymous=True)
        rate = rospy.Rate(10) 
        self.master= Tk()
        self.master.title('Speed Controller')
        self.slider_label = Label(self.master,text='RPM:').pack()
        self.speedslider = Scale(self.master, from_=-120, to=120,orient=HORIZONTAL,length=880,tickinterval=1,command=self.setspeed)
        self.speedslider.set(0)
        self.speedslider.pack()
        
        current_value = StringVar(value=5)
        self.kp_label = Label(self.master,text='Kp:').pack()
        self.kp = Spinbox(self.master,from_=0,to=30,textvariable=current_value,wrap=True,increment=0.01,format="%.2f",command=self.setkp)
        self.kp.pack()

        current_value = StringVar(value=10)
        self.ki_label = Label(self.master,text='Ki:').pack()
        self.ki = Spinbox(self.master,from_=0,to=30,textvariable=current_value,wrap=True,increment=0.01,format="%.2f",command=self.setki)
        self.ki.pack()

        self.actualSpeed_label = Label(self.master,text='Actual Speed:').pack()
        self.actualSpeed = Entry(self.master)
        self.actualSpeed.pack()
        self.master.mainloop()

    def setspeed(self,event):
        self.pubSpeed.publish(self.speedslider.get())
        rospy.loginfo("speed: %s",self.speedslider.get())

    def setkp(self):
        self.pubkp.publish(float(self.kp.get()))
        rospy.loginfo("kp: %s",self.kp.get())
    def setki(self):
        self.pubki.publish(float(self.ki.get()))
        rospy.loginfo("ki: %s"+self.ki.get())
        
    def getActual(self,data):
        data.data 
        # insert some actual code here to update actualSpeed
        
if __name__ == '__main__':
    try:
        tmp = speedcontroller()
    except rospy.ROSInterruptException:
        pass
