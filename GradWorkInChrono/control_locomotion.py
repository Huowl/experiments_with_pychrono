import scipy.interpolate as spline
import numpy as np


def trajInterpolate(points, period, evalTimes):
    num_points = np.size(points)
    traj_times = np.linspace(0,period,num_points)
    
    tck = spline.splrep(traj_times,points)
    time_arr = np.arange(0,period+evalTimes,evalTimes)
    
    out = spline.splev(time_arr, tck)
    return np.c_[time_arr, out]

class ControlLocomotion:
    def __init__(self, points, period, evalTimes, offsets_legs = {"front" : 50, "rear": 50}):
        
        self.__idx_motors = ['fr_hip', 'fr_knee', 'rr_hip', 'rr_knee']
        self.__points_motor = {}
        self.__traj_motor = {}
        self.__num_motor_points = int(np.size(points)/4)
        self.__offsets_legs = offsets_legs
        self.__period = period
        
        strt = 0
        end = self.__num_motor_points - 1
        for idx in self.__idx_motors:
            self.__points_motor[idx] = np.r_[points[strt:end],points[strt]]
            #print(self.__points_motor[idx])
            self.__traj_motor[idx] = trajInterpolate(self.__points_motor[idx], period, evalTimes)
            strt += self.__num_motor_points
            end += self.__num_motor_points
            
    def get_trajMotor(self, name_motor = "fr_hip"):
        return self.__traj_motor[name_motor]
    
    def get_pointsMotor(self,name_motor = "fr_hip"):
        return np.c_[np.linspace(0,self.__period,np.size(self.__points_motor[name_motor])),
                     self.__points_motor[name_motor]]
        
    def ger_indexMotors(self):
        return self.__idx_motors
    
    def getAggressiveness(self):
        n = 0
        diffs = np.array([])
        for idx in self.__idx_motors:
            diffs = np.c_[np.diff(self.__points_motor[idx])]
        for i in range(np.size(diffs)-1):
            if np.sign(diffs[i]/(diffs[i+1]+np.spacing(1))) < 0 & np.mod(i,np.size(self.__points_motor['fr_hip'])):
                n += 1
                
        return n
        
            
    def get_inputMotor(self, current_time, name_motor = "fr_hip_right"):
        det_motor = name_motor.split("_")
        des_idx_motor = det_motor[0] + "_" + det_motor[1]
        input_motor = 0.
        traj_des_motor = self.__traj_motor[des_idx_motor]
        #print(name_motor)
        #test1 = np.abs(traj_des_motor[:,0]-np.mod(current_time,traj_des_motor[-1,0]))
        #test2 = np.abs(traj_des_motor[:,0]-np.mod(current_time-self.__offsets_legs["front"]*traj_des_motor[-1,0]/100,traj_des_motor[-1,0]))# < 10**-4
        #print(current_time,np.min(test2),np.min(test1))
        ##print(test1)
        if det_motor[2] == "right":
            if det_motor[0] == "fr":
                input_motor = traj_des_motor[np.abs(traj_des_motor[:,0]-np.mod(current_time,traj_des_motor[-1,0])) < 10**-4,1]
            else:
                input_motor = traj_des_motor[np.abs(traj_des_motor[:,0]-np.mod(current_time-self.__offsets_legs["rear"]*traj_des_motor[-1,0]/100,traj_des_motor[-1,0])) < 10**-4,1]
        else:
            if det_motor[0] == "fr":
                input_motor = traj_des_motor[np.abs(traj_des_motor[:,0]-np.mod(current_time-self.__offsets_legs["front"]*traj_des_motor[-1,0]/100,traj_des_motor[-1,0])) < 10**-4,1]
            else:
                input_motor = traj_des_motor[np.abs(traj_des_motor[:,0]-np.mod(current_time,traj_des_motor[-1,0])) < 10**-4,1]
        return input_motor
