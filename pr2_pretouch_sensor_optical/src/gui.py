#!/usr/bin/env python
# Visualize the OpticalBeams sensor data by subsribing to the sensor topic
# @Liang-Ting Jiang (jianglt@uw.edu)

from Tkinter import Tk, Canvas, Frame, BOTH
import rospy
from pr2_pretouch_sensor_optical.msg import OpticalBeams

TOPIC = 'optical/right'

class OpticalBeamsGUI(Frame):
  
    def __init__(self, parent):
        Frame.__init__(self, parent)   
        self.parent = parent        
        self.r = []
        self.initUI()

    def beamCallback(self, msg):
        self.changeColor(msg.broken)
    
    def changeColor(self, broken):
        colors = ['red' if b else 'blue' for b in broken]
        for i in range(len(self.r)):
            self.canvas.itemconfigure(self.r[i], fill=colors[i])
            
    def initUI(self):
        self.parent.title("Opticl Beams")        
        self.pack(fill=BOTH, expand=1)
        self.canvas = Canvas(self)
        # CH1
        self.r.append(self.canvas.create_rectangle(0, 0, 200, 200, fill="blue"))
        self.canvas.create_text(100, 100, text='CH1', font=("Helvetica",30))
        # CH2
        self.r.append(self.canvas.create_rectangle(200, 200, 400, 400, fill="blue"))
        self.canvas.create_text(300, 300, text='CH2', font=("Helvetica",30))
        # CH3
        self.r.append(self.canvas.create_rectangle(200, 0, 400, 200, fill="blue"))
        self.canvas.create_text(300, 100, text='CH3', font=("Helvetica",30))
        # CH4
        self.r.append(self.canvas.create_rectangle(0, 200, 200, 400, fill="blue"))
        self.canvas.create_text(100, 300, text='CH4', font=("Helvetica",30))

        self.canvas.pack(fill=BOTH, expand=1)

    def start(self):
        self.sub = rospy.Subscriber(TOPIC, OpticalBeams, self.beamCallback)

def main():
    rospy.init_node("optical_beams_gui", anonymous=True)
    root = Tk()
    ob = OpticalBeamsGUI(root)
    root.geometry("400x400+300+300")
    ob.start()
    root.mainloop()  

if __name__ == '__main__':
    main()  
