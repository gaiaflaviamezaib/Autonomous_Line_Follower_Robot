from __future__ import division
import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685

import random

#modified init_pwm values, self.maxPos, minPos, made ctrlRange
#be for each servomotor for accurate turn angles
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

init_pwm0 = 355
init_pwm1 = 347
init_pwm2 = 213

class ServoCtrl():

    def __init__(self):
#        self.sc_direction = [1, 1, 1]
#        self.initPos = [init_pwm0, init_pwm1, init_pwm2]
#        self.nowPos  = [300, 300, 300]
#        self.lastPos = [300, 300, 300]
#        self.maxPos  = [360, 360, 360]
#        self.minPos  = [240, 240, 240]
#
#        self.ctrlRangeMax = 360
#        self.ctrlRangeMin = 240
#        self.angleRange = 180
        
        self.sc_direction = [1, 1, 1]
        self.initPos = [init_pwm0, init_pwm1, init_pwm2]
        self.nowPos  = [300, 300, 300]
        self.lastPos = [300, 300, 300]
        self.maxPos  = [535, 530, 435]
        self.minPos  = [185, 135, 150]

        self.ctrlRangeMax = 360
        self.ctrlRangeMin = 150
        self.angleRange = 90
        
        self.maxRangeWheels = self.maxPos[0]
        self.minRangeWheels = self.minPos[0]
        self.angleRangeWheels = 50
        
        self.maxRangeHeadHoriz = self.maxPos[1]
        self.minRangeHeadHoriz = self.minPos[1]
        self.angleRangeHeadHoriz = 160
        
        self.maxRangeHeadVert = self.maxPos[2]
        self.minRangeHeadVert = self.minPos[2]
        self.angleRangeHeadVert = 120
        
    def moveInit(self):
        for i in range(0,3):
            pwm.set_pwm(i,0,self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
      
    def pwmGenOut(self, angleInput, ID=None):
    
        if ID == 0:
#            print("ba")
            return int(round(((self.maxRangeWheels-self.minRangeWheels)/self.angleRangeWheels*angleInput),0))
        
        elif ID == 1:
#            print("bo")
            if angleInput < 0:
                angleInput *= 1.075#1.125
#                
        elif ID == 2:
#            print("bo")
#            if angleInput < 0:
#                angleInput *= 1.075#1.125
#                
            
            return int(round(((self.maxRangeHeadVert-self.minRangeHeadVert)/self.angleRangeHeadVert*angleInput),0))
        
        return int(round(((self.ctrlRangeMax-self.ctrlRangeMin)/self.angleRange*angleInput),0))
        

    def moveAngle(self, ID, angleInput, printPowers=False):
    
        self.nowPos[ID] = int(self.initPos[ID] + self.sc_direction[ID]*self.pwmGenOut(angleInput, ID))
        
        if printPowers:
            print(f"Power before correction for angle",angleInput,":",self.nowPos[ID])
        if self.nowPos[ID] > self.maxPos[ID]:
            self.nowPos[ID] = self.maxPos[ID]
        elif self.nowPos[ID] < self.minPos[ID]:
            self.nowPos[ID] = self.minPos[ID]
        if printPowers:
            print(f"Power after correction for angle",angleInput,":",self.nowPos[ID])
        self.lastPos[ID] = self.nowPos[ID]
        pwm.set_pwm(ID, 0, self.nowPos[ID])
    


if __name__ == '__main__':
    sc = ServoCtrl()
    while 1:
        sc.moveAngle(0,(random.random()*100-50))
        time.sleep(1)
        sc.moveAngle(1,(random.random()*100-50))
        time.sleep(1)
        pass
    pass
