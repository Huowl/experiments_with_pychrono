from doctest import debug_script
import pychrono.core as chr
import pychrono.irrlicht as chrirr
import numpy as np
import leg as lg
import robot_parameters as param
import control_locomotion as ctrl

BODY_DIM = param.body_dim
BODY_DENSITY = param.body_density

HIP_OFFSET = param.hip_offset
HIP_RADIUS = param.hip_radius
HIP_LENGTH = param.hip_length
HIP_DENSITY = param.hip_density

def assemblingRobot(chrono_system,corpus_body,coord_body):
    idx_legs = ['fr_right', 'fr_left', 'rr_right', 'rr_left']
    vec_hip = {}
    vec_off_leg = {}
    vec_hip_body = {}
    vec_legs  = {}
    for idx in idx_legs:
        if idx.split('_')[0] == 'rr':
            sig_x = -1
        else:
            sig_x = 1
        if idx.split('_')[1] == 'right':
            sig_z = -1
        else:
            sig_z = 1
        
        vec_hip[idx] = coord_body + chr.ChVectorD(sig_x*(BODY_DIM[0]/2+HIP_OFFSET),0,sig_z*(BODY_DIM[1]-HIP_LENGTH)/2)
        vec_off_leg[idx] = chr.ChVectorD(0,0,sig_z*(HIP_LENGTH+lg.FEMUR_WIDTH)/2)
        
        vec_hip_body[idx] = chr.ChBodyEasyCylinder(HIP_RADIUS, HIP_LENGTH, HIP_DENSITY)
        vec_hip_body[idx].SetPos(vec_hip[idx])
        vec_hip_body[idx].SetRot(chr.Q_ROTATE_Y_TO_Z)
        chrono_system.Add(vec_hip_body[idx])
        
        joint_corp_hip = chr.ChLinkMateFix()
        joint_corp_hip.Initialize(vec_hip_body[idx],corpus_body,chr.ChFrameD(vec_hip[idx]))
        chrono_system.Add(joint_corp_hip)
        
        vec_legs[idx] = lg.leg(chrono_system, vec_hip[idx] + vec_off_leg[idx], idx.split('_')[0] == 'rr', idx.split('_')[1] == 'right')
        vec_legs[idx].CreateBodies()
        vec_legs[idx].CreaterJoints()
        
        joint_corp_leg = chr.ChLinkMateFix()
        joint_corp_leg.Initialize(vec_legs[idx].getHipBody(),vec_hip_body[idx],chr.ChFrameD(vec_hip[idx] + vec_off_leg[idx]))
        chrono_system.Add(joint_corp_leg)
        
    return idx_legs, vec_legs
        
    


class quadruped:
    def __init__(self, start_position):
        self.__strt_coord = start_position
        self.__corpus_body = chr.ChBodyEasyBox(BODY_DIM[0],BODY_DIM[2],
                                        BODY_DIM[1],BODY_DENSITY)
        self.__corpus_body.SetPos(start_position)
        self.__traj_x = np.array([0, start_position.x])
        self.__vel_x_t = np.array([0,0])
        
    def setChronoSystem(self, chrono_system):
        self.__chr_system = chrono_system
        self.__chr_system.Add(self.__corpus_body)
        
        self.__idx_legs, self.__legs = assemblingRobot(self.__chr_system, self.__corpus_body,self.__strt_coord)
        
    def getCoordCorpus(self):
        return self.__corpus_body.GetFrame_COG_to_abs().GetPos()
    
    def getVelCorpus(self):
        return self.__corpus_body.GetFrame_COG_to_abs().GetPos_dt()
        
    def getCorpusBody(self):
        return self.__corpus_body   
        
    def setLocomotion(self, locomotion):
        self.__locomotion = locomotion
        
    def getLocomotion(self):
        return self.__locomotion
    
    def update(self,current_time):
        data_x = np.array([current_time, self.getCoordCorpus().x])
        vel_x_data = np.array([current_time, self.getVelCorpus().x])
        self.__traj_x = np.c_[self.__traj_x, data_x]
        self.__vel_x_t = np.c_[self.__vel_x_t, vel_x_data]
        
    def getTrajX(self):
        return self.__traj_x
    
    def getVelX(self):
        return self.__vel_x_t
        
    def updateControlLegs(self,current_time):
        
        for idx in self.__idx_legs:
            des_pos_hip = self.__locomotion.get_inputMotor(current_time,idx.split('_')[0]+'_hip_'+idx.split('_')[1])
            des_pos_knee = self.__locomotion.get_inputMotor(current_time,idx.split('_')[0]+'_knee_'+idx.split('_')[1])
            
            self.__legs[idx].PDControl(des_pos_hip,des_pos_knee)
    
    def TestingSimulate(self, time_stop=6, time_step=0.001):
        myapplication = chrirr.ChIrrApp(self.__chr_system, 'Test_Quadruped_Body',
                                        chrirr.dimension2du(1280,720))

        myapplication.AddTypicalSky()
        myapplication.AddTypicalCamera(chrirr.vector3df(0,0,1.7))
        myapplication.AddLightWithShadow(chrirr.vector3df(2,4,2),    # point
                                        chrirr.vector3df(0,0,0),    # aimpoint
                                        9,                 # radius (power)
                                        1,9,               # near, far
                                        60)                # angle of FOV
        myapplication.AssetBindAll()
        myapplication.AssetUpdateAll()
        myapplication.AddShadowAll()
        
        time = []
        
        myapplication.SetTimestep(time_step)
        myapplication.SetTryRealtime(True)
        while(myapplication.GetDevice().run()):
            myapplication.BeginScene()
            myapplication.DrawAll()
            chrirr.drawAllLinkframes(self.__chr_system, myapplication.GetVideoDriver(),1)
            #chrirr.drawAllLinks(self.__chr_system, myapplication.GetVideoDriver(),1)
            #chrirr.drawAllCOGs(self.chr_system, myapplication.GetVideoDriver(),1)
            time.append(self.__chr_system.GetChTime())
            myapplication.DoStep()
            myapplication.EndScene()
            if self.__chr_system.GetChTime() > time_stop:
                myapplication.GetDevice().closeDevice()
        #return time