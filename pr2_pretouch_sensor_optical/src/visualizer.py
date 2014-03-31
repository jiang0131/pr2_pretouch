#!/usr/bin/env python
# Visualize the OpticalBeams sensor data by subsribing to the sensor topic
# This is useful for verifying whether the sensing is working correctly
# @Liang-Ting Jiang (jianglt@uw.edu)

from Tkinter import Tk, Canvas, Frame, BOTH
import rospy
from pr2_pretouch_sensor_optical.msg import OpticalBeams

TOPIC_LEFT = 'optical/left'
TOPIC_RIGHT = 'optical/right'

class OpticalBeamsGUI(Frame):
  
    def __init__(self, parent):
        Frame.__init__(self, parent)   
        self.parent = parent        
        self.r = []
        self.initUI()

    def beamCallbackLeft(self, msg):
        self.changeColor(msg.broken, 'l')

    def beamCallbackRight(self, msg):
        self.changeColor(msg.broken, 'r')
    
    def changeColor(self, broken, side='r'):
        colors = ['red' if b else 'blue' for b in broken]
        k = 4 if side=='l' else 0
        for i in range(len(colors)):
            self.canvas.itemconfigure(self.r[k+i], fill=colors[i])
            
    def initUI(self):
        self.parent.title("Opticl Beams")        
        self.pack(fill=BOTH, expand=1)
        self.canvas = Canvas(self)

        # Right
        self.canvas.create_text(200, 50, text='r_gripper', font=("Helvetica",30))
        # CH1
        self.r.append(self.canvas.create_rectangle(0, 100, 200, 300, fill="blue"))
        self.canvas.create_text(100, 200, text='CH1', font=("Helvetica",30))
        # CH2
        self.r.append(self.canvas.create_rectangle(200, 300, 400, 500, fill="blue"))
        self.canvas.create_text(300, 400, text='CH2', font=("Helvetica",30))
        # CH3
        self.r.append(self.canvas.create_rectangle(200, 100, 400, 300, fill="blue"))
        self.canvas.create_text(300, 200, text='CH3', font=("Helvetica",30))
        # CH4
        self.r.append(self.canvas.create_rectangle(0, 300, 200, 500, fill="blue"))
        self.canvas.create_text(100, 400, text='CH4', font=("Helvetica",30))

        # Left
        self.canvas.create_text(700, 50, text='l_gripper', font=("Helvetica",30))
        # CH1
        self.r.append(self.canvas.create_rectangle(500, 100, 700, 300, fill="blue"))
        self.canvas.create_text(600, 200, text='CH1', font=("Helvetica",30))
        # CH2
        self.r.append(self.canvas.create_rectangle(700, 300, 900, 500, fill="blue"))
        self.canvas.create_text(800, 400, text='CH2', font=("Helvetica",30))
        # CH3
        self.r.append(self.canvas.create_rectangle(700, 100, 900, 300, fill="blue"))
        self.canvas.create_text(800, 200, text='CH3', font=("Helvetica",30))
        # CH4
        self.r.append(self.canvas.create_rectangle(500, 300, 700, 500, fill="blue"))
        self.canvas.create_text(600, 400, text='CH4', font=("Helvetica",30))

        self.canvas.pack(fill=BOTH, expand=1)

    def start(self):
        self.sub1 = rospy.Subscriber(TOPIC_LEFT, OpticalBeams, self.beamCallbackLeft)
        self.sub2 = rospy.Subscriber(TOPIC_RIGHT, OpticalBeams, self.beamCallbackRight)

def main():
    rospy.init_node("optical_beams_gui", anonymous=True)
    root = Tk()
    ob = OpticalBeamsGUI(root)
    root.geometry("900x500+300+300")
    ob.start()
    root.mainloop()  

if __name__ == '__main__':
    main()  
