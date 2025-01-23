#!/usr/bin/env python3

import rospy
import numpy as np

class Paths:
    def __init__(self, tm = 0.15, tf = 60):
        self.tm = tm
        self.tf = tf
        self.t = np.arange(0, self.tf, self.tm)

    def time(self):
        return self.t

    def ellipse(self, a = 3, b = 2, center = [0, -2]):
        x_center, y_center = center
        x = x_center + a * np.sin(2 * np.pi * self.t / self.tf)
        y = y_center + b * np.cos(2 * np.pi * self.t / self.tf)
        return x, y
                
    def ellipse_d(self, a = 3, b = 2, center = [0, -2]):
        x_center, y_center = center
        x = a * 2 * np.pi / self.tf * np.cos(2 * np.pi * self.t / self.tf)
        y = -b * 2 * np.pi / self.tf * np.sin(2 * np.pi * self.t / self.tf)
        return x, y
        
    def lemniscate(self, a = 3, center = [0, 0]):
        x_center, y_center = center
        x = x_center + a * np.sin(2 * np.pi * self.t / self.tf)
        y = y_center + a * np.sin(2 * np.pi * self.t / self.tf) * np.cos(2 * np.pi * self.t / self.tf)
        return x, y
        
    def lemniscate_d(self, a = 3, center = [0, 0]):
        x_center, y_center = center
        x = a * 2 * np.pi / self.tf * np.cos(2 * np.pi * self.t / self.tf)
        y = a * (np.cos(2 * np.pi * self.t / self.tf)**2 - np.sin(2 * np.pi * self.t / self.tf)**2) * 2 * np.pi / self.tf
        return x, y
    
    def spiral(self, a = 0.5, b = 0.05, center = [0, -0.5]):
        x_center, y_center = center
        r = a + b * self.t
        x = x_center + r * np.sin(2 * np.pi * self.t / self.tf)
        y = y_center + r * np.cos(2 * np.pi * self.t / self.tf)
        return x, y
        
    def spiral_d(self, a = 0.5, b = 0.05, center = [0, -0.5]):
        x_center, y_center = center
        r = a + b * self.t
        x = r * 2 * np.pi / self.tf * np.cos(2 * np.pi * self.t / self.tf) + b * 2 * np.pi / self.tf * np.sin(2 * np.pi * self.t / self.tf)
        y = -r * 2 * np.pi / self.tf * np.sin(2 * np.pi * self.t / self.tf) + b * 2 * np.pi / self.tf * np.cos(2 * np.pi * self.t / self.tf)
        return x, y

    def line(self, start = [0, 0], end = [12, 10]):
        x_start, y_start = start
        x_end, y_end = end
        x = np.linspace(x_start, x_end, len(self.t))
        y = np.linspace(y_start, y_end, len(self.t))
        return x, y
        
    def line_d(self, start = [0, 0], end = [12, 10]):
        x_start, y_start = start
        x_end, y_end = end
        x = (x_end - x_start) / self.tf * np.ones_like(self.t)
        y = (y_end - y_start) / self.tf * np.ones_like(self.t)
        return x, y
        
    def sine(self, amplitude = 2, frequency = 0.5, n = 5, center = [0, 0]):
        x_center, y_center = center
        x = x_center + self.t / n
        y = y_center + amplitude * np.sin(2 * np.pi * frequency * self.t / self.tf)
        return x, y
    
    def sine_d(self, amplitude = 2, frequency = 0.5, n = 5, center = [0, 0]):
        x_center, y_center = center
        x = np.ones_like(self.t) / n
        y = amplitude * 2 * np.pi * frequency / self.tf * np.cos(2 * np.pi * frequency * self.t / self.tf)
        return x, y
    
    def rectangle(self, length = 6, width = 4, center = [0, 0]):
        x_center, y_center = center
        
        x1 = np.linspace(x_center, x_center+length, int(len(self.t)/4.0))
        y1 = np.linspace(y_center, y_center, int(len(self.t)/4.0))
        
        x2 = np.linspace(x_center+length, x_center+length, int(len(self.t)/4.0))
        y2 = np.linspace(y_center, y_center+width, int(len(self.t)/4.0))
        
        x3 = np.linspace(x_center+length, x_center, int(len(self.t)/4.0))
        y3 = np.linspace(y_center+width, y_center+width, int(len(self.t)/4.0))
        
        x4 = np.linspace(x_center, x_center, int(len(self.t)/4.0))
        y4 = np.linspace(y_center+width, y_center, int(len(self.t)/4.0))
        
        x = np.concatenate((x1,x2,x3,x4))
        y = np.concatenate((y1,y2,y3,y4))
        return x, y
        
    def rectangle_d(self, length = 6, width = 4, center = [0, 0]):
        x_center, y_center = center
        
        x1 = np.linspace(length/(int(len(self.t)/4.0)), length/(int(len(self.t)/4.0)), int(len(self.t)/4.0))
        y1 = np.linspace(0, 0, int(len(self.t)/4.0))
        
        x2 = np.linspace(0, 0, int(len(self.t)/4.0))
        y2 = np.linspace(width/(int(len(self.t)/4.0)), width/(int(len(self.t)/4.0)), int(len(self.t)/4.0))
        
        x3 = np.linspace(-length/(int(len(self.t)/4.0)), -length/(int(len(self.t)/4.0)), int(len(self.t)/4.0))
        y3 = np.linspace(0, 0, int(len(self.t)/4.0))
        
        x4 = np.linspace(0, 0, int(len(self.t)/4.0))
        y4 = np.linspace(-width/(int(len(self.t)/4.0)), -width/(int(len(self.t)/4.0)), int(len(self.t)/4.0))
        
        x = np.concatenate((x1,x2,x3,x4))
        y = np.concatenate((y1,y2,y3,y4))
        return x, y
