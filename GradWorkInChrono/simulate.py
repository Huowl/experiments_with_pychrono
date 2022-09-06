import pychrono.core as chr
import pychrono.irrlicht as chrirr
import robot_parameters as param
import quadruped_corpus as quadruped
import numpy as np

GROUND_DIM = param.ground

class Simulate:
    def __init__(self, quadruped, time_step, stop_time):
        
        #chr.SetChronoDataPath('D:/Programms/anaconda3/pkgs/pychrono-7.0.0-py39_0/Library/data/')
        self.__model_quadruped = quadruped
        self.__time_step = time_step
        self.__stop_time = stop_time
        self.__chr_system = chr.ChSystemNSC()
        chr.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0005)
        chr.ChCollisionModel.SetDefaultSuggestedMargin(0.0005)
        self.__chr_system.SetSolverMaxIterations(70)
        
        self.__model_quadruped.setChronoSystem(self.__chr_system)
        
        ground_material = chr.ChMaterialSurfaceNSC()
        ground_material.SetRestitution(0.1)
        ground_material.SetFriction(0.9)
        
        
        ground_body = chr.ChBodyEasyBox(GROUND_DIM[0],
                                        GROUND_DIM[2],
                                        GROUND_DIM[1],
                                        1000, True,
                                        True,
                                        ground_material)
        ground_body.SetPos(chr.ChVectorD(GROUND_DIM[0]/2-0.5,0,0))
        ground_body.SetBodyFixed(True)
        
        self.__chr_system.Add(ground_body)
        
        z_bound_robot = chr.ChLinkLockPlanePlane()
        z_bound_robot.Initialize(self.__model_quadruped.getCorpusBody(),ground_body,
                                 chr.ChCoordsysD(chr.ChVectorD(0,0.44,0),chr.ChQuaternionD(1,0,0,0)))
        self.__chr_system.Add(z_bound_robot)
        
        
    def initilizeAnimation(self):
        self.__myapplication = chrirr.ChIrrApp(self.__chr_system, 'Simulate_Quadruped',
                                        chrirr.dimension2du(1280,720))

        self.__myapplication.AddTypicalSky()
        self.__myapplication.AddTypicalCamera(chrirr.vector3df(0,0,1.7))
        self.__myapplication.AddLightWithShadow(chrirr.vector3df(2,4,2),    # point
                                        chrirr.vector3df(0,0,0),    # aimpoint
                                        9,                 # radius (power)
                                        1,9,               # near, far
                                        60)                # angle of FOV
        self.__myapplication.AssetBindAll()
        self.__myapplication.AssetUpdateAll()
        self.__myapplication.AddShadowAll()
        
        self.__myapplication.SetTimestep(self.__time_step)
        self.__myapplication.SetTryRealtime(False)
    
    def FlagsStopSimulate(self,curr_time, testing=False):
        coord = self.__model_quadruped.getCoordCorpus()
        speed = self.__model_quadruped.getVelCorpus()
        
        flag_flying_robot = coord.y > 0.7
        flag_fallen_robot = coord.y <= 0.1
        flag_stay_robot = (np.abs(coord.x) < 0.1) & (curr_time > 2)
        flag_back_walk_robot = (speed.x <= -0.05) & (curr_time > 1.5)
        flag_stop_simulation = curr_time >= self.__stop_time
        
        #print(f"coord_y: {coord.y}, time: {curr_time}")
        
        stop_simulation = ((flag_flying_robot | flag_fallen_robot |
                      flag_stay_robot | flag_back_walk_robot | flag_stop_simulation) &
                           ~testing | (testing & (flag_stop_simulation | flag_fallen_robot)))

        return stop_simulation
        
    def getReward(self, desired_velocity):
        reward = ((10*(self.__final_coord_x - desired_velocity*self.__stop_time))**2/
                  (self.__final_time**4+np.spacing(1)))*pow(2,self.__model_quadruped.getLocomotion().getAggressiveness())
        
        return reward
    
    def startNoAnimation(self):
        self.__chr_system.SetChTime(0)
        stop = False

        while(not stop):
            self.__chr_system.DoStepDynamics(self.__time_step)
            
            curr_time = self.__chr_system.GetChTime()
            self.__model_quadruped.updateControlLegs(curr_time)
            stop = self.FlagsStopSimulate(curr_time)            
            
            self.__final_time = curr_time
            self.__final_coord_x = self.__model_quadruped.getCorpusBody().GetFrame_COG_to_abs().GetPos().x
        print(f"x_stop: {self.__final_coord_x}, time_stop: {self.__final_time}, agressive: {self.__model_quadruped.getLocomotion().getAggressiveness()}")
    
    def start(self):
        self.__final_time = 0.
        self.__final_coord_x = 0.
        while(self.__myapplication.GetDevice().run()):
            self.__myapplication.BeginScene()
            self.__myapplication.DrawAll()
            curr_time = self.__chr_system.GetChTime()
            #chrirr.drawAllLinkframes(self.__chr_system, self.__myapplication.GetVideoDriver(),1)
            #chrirr.drawAllLinks(self.__chr_system, self.__myapplication.GetVideoDriver(),1)
            #chrirr.drawAllCOGs(self.chr_system, self.__myapplication.GetVideoDriver(),1)
            self.__model_quadruped.updateControlLegs(curr_time)
            stop = self.FlagsStopSimulate(curr_time, True)
            self.__myapplication.DoStep()
            self.__myapplication.EndScene()
            
            self.__final_time = curr_time
            self.__final_coord_x = self.__model_quadruped.getCoordCorpus().x
            self.__model_quadruped.update(curr_time)
            if stop:
                self.__myapplication.GetDevice().closeDevice()
        #return time
        print(f"x_stop: {self.__final_coord_x}, time_stop: {self.__final_time}, agressive: {self.__model_quadruped.getLocomotion().getAggressiveness()}")
        del self.__myapplication
    
    