#!/usr/bin/env python3

import rospy
import numpy as np

class Paths:
    def __init__(self):
        self.trajectory_name = rospy.get_param('path_type', "ellipse")
        parameter_path = f"{self.trajectory_name}/parameters"
        self.params = rospy.get_param(parameter_path, None)
        self.t, self.x, self.y, self.vx, self.vy, self.theta, self.w = None, None, None, None, None, None, None
        self.Tf = None
        self.Ts = None

        print("Trajectory selected:", self.trajectory_name)
        if self.params is None:
            rospy.logerr(f"Parameters for {self.trajectory_name} not found. Check your YAML file.")
            rospy.signal_shutdown("Invalid trajectory parameters.")

        self.trajectory_functions = {
            "ellipse": self.ellipse,
            "lemniscate": self.lemniscate,
            "sine": self.sine,
            "rectangle": self.rectangle,
            "line": self.line,
            "spiral": self.spiral
        }

        self.compute_trajectory()

    def compute_trajectory(self):
        if self.trajectory_name in self.trajectory_functions:
            self.t, self.x, self.y, self.vx, self.vy, self.theta, self.w = self.trajectory_functions[self.trajectory_name]()
        else:
            raise ValueError(f"Unknown trajectory type: {self.trajectory_name}")

    def spiral(self):
        a         = self.params[0]  
        b         = self.params[1]  
        x_center  = self.params[2]
        y_center  = self.params[3]
        Ts        = self.params[4]
        Tf        = self.params[5]
        angle     = self.params[6]
        t = np.arange(0, Tf, Ts)
        r = a + b * self.t
        x = x_center + r * np.sin(2 * np.pi * t / Tf)
        y = y_center + r * np.cos(2 * np.pi * t / Tf)
        vx = r * 2 * np.pi / Tf * np.cos(2 * np.pi * t / Tf) + b * np.sin(2 * np.pi * t / Tf)
        vy = -r * 2 * np.pi / Tf * np.sin(2 * np.pi * t / Tf) + b * np.cos(2 * np.pi * t / Tf)
        print(f"\033[38;2;0;255;0mGenerated spiraltrajectory: a: {a} b: {b} xc: {x_center} yc: {y_center} Ts: {Ts} Tf: {Tf}\033[0m")
            
        theta = np.linspace(0, 0, int(len(t)))
        w = np.linspace(0, 0, int(len(t)))
            
        if angle:
            theta = np.arctan2(vy,vx)
            for i in range(0,int(len(theta))):
                if i == 0:
                    w[0] = 0
                if i > 0:
                    w[i] = (theta[i] - theta[i-1])/Ts
        
        self.Tf = Tf
        self.Ts = Ts
        return t, x, y, vx, vy, theta, w
        
    def ellipse(self):
        a         = self.params[0]  
        b         = self.params[1]  
        x_center  = self.params[2]
        y_center  = self.params[3]
        Ts        = self.params[4]
        Tf        = self.params[5]
        angle     = self.params[6]
        t = np.arange(0, Tf, Ts)
        x = x_center + a * np.sin(2 * np.pi * t / Tf)
        y = y_center + b * np.cos(2 * np.pi * t / Tf)
        vx =  a * 2 * np.pi / Tf * np.cos(2 * np.pi * t / Tf)
        vy = -b * 2 * np.pi / Tf * np.sin(2 * np.pi * t / Tf)
        print(f"\033[38;2;0;255;0mGenerated ellipse trajectory: a: {a} b: {b} xc: {x_center} yc: {y_center} Ts: {Ts} Tf: {Tf}\033[0m")
        self.Tf = Tf
        self.Ts = Ts
        
        theta = np.linspace(0, 0, int(len(t)))
        w = np.linspace(0, 0, int(len(t)))
        
        if angle:
            theta = np.arctan2(vy,vx)
            for i in range(0,int(len(theta))):
                if i == 0:
                    w[0] = 0
                if i > 0:
                    w[i] = (theta[i] - theta[i-1])/Ts

        self.Tf = Tf
        self.Ts = Ts
        return t, x, y, vx, vy, theta, w
    
    def lemniscate(self):
        a         = self.params[0]  
        b         = self.params[1]  
        x_center  = self.params[2]
        y_center  = self.params[3]
        Ts        = self.params[4]
        Tf        = self.params[5]
        angle     = self.params[6]
        t = np.arange(0, Tf, Ts)
        

        x = x_center + a * np.sin(2 * np.pi * t / Tf)
        y = y_center + b * np.sin(2 * np.pi * t / Tf) * np.cos(2 * np.pi * t / Tf)
       
        vx = a * 2 * np.pi / Tf * np.cos(2 * np.pi * t / Tf)
        vy = b * 2 * np.pi / Tf * (np.cos(2 * np.pi * t / Tf)**2 - np.sin(2 * np.pi * t / Tf)**2) 
        print(f"\033[38;2;0;255;0mGenerated Lemniscate trajectory: a: {a} b: {b} xc: {x_center} yc: {y_center} Ts: {Ts} Tf: {Tf}\033[0m")

        self.Tf = Tf
        self.Ts = Ts
        
        theta = np.linspace(0, 0, int(len(t)))
        w = np.linspace(0, 0, int(len(t)))

        if angle:
            theta = np.arctan2(vy,vx)
            for i in range(0,int(len(theta))):
                if i == 0:
                    w[0] = 0
                if i > 0:
                    w[i] = (theta[i] - theta[i-1])/Ts

        self.Tf = Tf
        self.Ts = Ts
        return t, x, y, vx, vy, theta, w
    
    def sine(self):
        amplitude = self.params[0]  
        n         = self.params[1]
        frequency = self.params[2]   
        x_center  = self.params[3]
        y_center  = self.params[4]
        Ts        = self.params[5]
        Tf        = self.params[6]
        angle     = self.params[7]

        
        t = np.arange(0, Tf, Ts)
        x = x_center + t / n
        y = y_center + amplitude * np.sin(2 * np.pi * frequency * t / Tf)

        vx = np.ones_like(t) / n
        vy = amplitude * 2 * np.pi * frequency / Tf * np.cos(2 * np.pi * frequency * t / Tf)
        print(f"\033[38;2;0;255;0mGenerated Sine trajectory: ampl: {amplitude} freq: {frequency} xc: {x_center} yc: {y_center} Ts: {Ts} Tf: {Tf}\033[0m")

        self.Tf = Tf
        self.Ts = Ts

        theta = np.linspace(0, 0, int(len(t)))
        w = np.linspace(0, 0, int(len(t)))

        if angle:
            theta = np.arctan2(vy,vx)
            for i in range(0,int(len(theta))):
                if i == 0:
                    w[0] = 0
                if i > 0:
                    w[i] = (theta[i] - theta[i-1])/Ts
        
        self.Tf = Tf
        self.Ts = Ts
        return t, x, y, vx, vy, theta, w
        
    
    def line(self):
        x_start = self.params[0]  
        y_start = self.params[1]  
        x_end   = self.params[2]
        y_end   = self.params[3]
        Ts      = self.params[4]
        Tf      = self.params[5]
        angle   = self.params[6]
        t = np.arange(0, Tf, Ts)
   
        x = np.linspace(x_start, x_end, len(t))
        y = np.linspace(y_start, y_end, len(t))

        vx = (x_end - x_start) / Tf * np.ones_like(t)
        vy = (y_end - y_start) / Tf * np.ones_like(t)
        print(f"\033[38;2;0;255;0mGenerated Line trajectory: xS: {x_start} yS: {y_start} xE: {x_end} yE: {y_end} Ts: {Ts} Tf: {Tf}\033[0m")

        theta = np.linspace(0, 0, int(len(t)))
        w = np.linspace(0, 0, int(len(t)))

        if angle:
            theta = np.arctan2(vy,vx)
            for i in range(0,int(len(theta))):
                if i == 0:
                    w[0] = 0
                if i > 0:
                    w[i] = (theta[i] - theta[i-1])/Ts
        
        self.Tf = Tf
        self.Ts = Ts
        return t, x, y, vx, vy, theta, w
    
    def rectangle(self, length = 6, width = 4, center = [0, 0]):
        length = self.params[0]  
        width  = self.params[1]  
        x_center = self.params[2]
        y_center = self.params[3]
        Ts = self.params[4]
        Tf = self.params[5]
        angle     = self.params[6]
        t = np.arange(0, Tf, Ts)
        
        
        x1 = np.linspace(x_center, x_center+length, int(len(t)/4.0))
        y1 = np.linspace(y_center, y_center, int(len(t)/4.0))
        
        x2 = np.linspace(x_center+length, x_center+length, int(len(t)/4.0))
        y2 = np.linspace(y_center, y_center+width, int(len(t)/4.0))
        
        x3 = np.linspace(x_center+length, x_center, int(len(t)/4.0))
        y3 = np.linspace(y_center+width, y_center+width, int(len(t)/4.0))
        
        x4 = np.linspace(x_center, x_center, int(len(t)/4.0))
        y4 = np.linspace(y_center+width, y_center, int(len(t)/4.0))
        
        x = np.concatenate((x1,x2,x3,x4))
        y = np.concatenate((y1,y2,y3,y4))

        vx1 = np.linspace(4*length/Tf, 4*length/Tf, int(len(t)/4.0))
        vy1 = np.linspace(0, 0, int(len(t)/4.0))
        
        vx2 = np.linspace(0, 0, int(len(t)/4.0))
        vy2 = np.linspace(4*width/Tf, 4*width/Tf, int(len(t)/4.0))
        
        vx3 = np.linspace(-4*length/Tf, -4*length/Tf, int(len(t)/4.0))
        vy3 = np.linspace(0, 0, int(len(t)/4.0))
        
        vx4 = np.linspace(0, 0, int(len(t)/4.0))
        vy4 = np.linspace(-4*width/Tf, -4*width/Tf, int(len(t)/4.0))
        
        vx = np.concatenate((vx1,vx2,vx3,vx4))
        vy = np.concatenate((vy1,vy2,vy3,vy4))

        print(f"\033[38;2;0;255;0mGenerated Rectangle trajectory: L: {length} w: {width} xc: {x_center} yc: {y_center} Ts: {Ts} Tf: {Tf}\033[0m")
    
        self.Tf = Tf
        self.Ts = Ts

        theta = np.linspace(0, 0, int(len(t)))
        w = np.linspace(0, 0, int(len(t)))

        if angle:
            theta = np.arctan2(vy,vx)
            for i in range(0,int(len(theta))):
                if i == 0:
                    w[0] = 0
                if i > 0:
                    w[i] = (theta[i] - theta[i-1])/Ts
        
        self.Tf = Tf
        self.Ts = Ts
        return t, x, y, vx, vy, theta, w
