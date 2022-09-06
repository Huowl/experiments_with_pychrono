import pychrono.core as chr
import pychrono.irrlicht as chrirr
import numpy as np
import robot_parameters as param
import scipy as sp


FEMUR_LENGTH = param.femur_length
FEMUR_HEIGHT = param.femur_height
FEMUR_WIDTH = param.femur_width
FEMUR_DENSITY = param.femur_density

TIBIA_LENGTH = param.tibia_length
TIBIA_HEIGHT = param.tibia_height
TIBIA_WIDTH = param.tibia_width
TIBIA_DENSITY = param.tibia_density

FOOT_DENSITY = param.foot_density
HIP_DENSITY = param.hip_density

COEF_P_FEMUR = 80
COEF_D_FEMUR = 1
COEF_P_TIBIA = 80
COEF_D_TIBIA = 1
    
    

class leg():
    def __init__(self, system, hip_coordinate_joints = chr.ChVectorD(0,0,0), right=True, forward=True):
        self.__teta_hip = 0
        self.__teta_knee = 0
        self.right = right
        self.forward = forward
        self.__chr_system = system
        self.hip_coord = hip_coordinate_joints
        
        
    def CreateBodies(self):
        start_position = self.hip_coord
         
        self.__hip_body = chr.ChBodyEasyCylinder(FEMUR_HEIGHT,FEMUR_WIDTH,HIP_DENSITY)
        self.__hip_body.SetPos(start_position)
        self.__hip_body.SetRot(chr.Q_ROTATE_Y_TO_Z)
        self.__hip_body.SetCollide(False)
        self.__chr_system.Add(self.__hip_body)
        
        self.__femur_body =  chr.ChBodyEasyBox(FEMUR_LENGTH,
                                       FEMUR_HEIGHT, FEMUR_WIDTH,
                                       FEMUR_DENSITY)
        self.__femur_body.SetPos(start_position + chr.ChVectorD(0,-FEMUR_LENGTH/2,0))                               
        self.__femur_body.SetRot(chr.Q_ROTATE_X_TO_Y)
        self.__femur_body.SetCollide(False)
        self.__chr_system.Add(self.__femur_body)
        
        knee_body = chr.ChBodyEasyCylinder(FEMUR_HEIGHT/2, FEMUR_WIDTH,FOOT_DENSITY)
        knee_body.SetPos(start_position + chr.ChVectorD(0,-FEMUR_LENGTH,0))
        knee_body.SetRot(chr.Q_ROTATE_Y_TO_Z)
        knee_body.SetCollide(False)
        self.__chr_system.Add(knee_body)
        
        knee_weld = chr.ChLinkMateFix()
        knee_weld.Initialize(knee_body,self.__femur_body,chr.ChFrameD(start_position + chr.ChVectorD(0,-FEMUR_LENGTH,0)))
        self.__chr_system.Add(knee_weld)
       
        self.__tibia_body = chr.ChBodyEasyBox(TIBIA_LENGTH,
                                      TIBIA_HEIGHT, TIBIA_WIDTH,
                                      TIBIA_DENSITY)
        self.__tibia_body.SetPos(start_position + chr.ChVectorD(0,-FEMUR_LENGTH-TIBIA_LENGTH/2,0))
        self.__tibia_body.SetRot(chr.Q_ROTATE_X_TO_Y)
        self.__tibia_body.SetCollide(False)
        self.__chr_system.Add(self.__tibia_body)
        
        
        foot_material = chr.ChMaterialSurfaceNSC()
        foot_material.SetFriction(0.9)
        foot_material.SetDampingF(0.3)
        foot_material.SetCompliance(10**-7)
        foot_material.SetComplianceT(10**-7)
        self.__foot_body = chr.ChBodyEasySphere(param.foot_radius*1.2, FOOT_DENSITY,
                                                True,True,
                                                foot_material)
        self.__foot_body.SetPos(start_position + chr.ChVectorD(0,-FEMUR_LENGTH-TIBIA_LENGTH,0))
        self.__chr_system.Add(self.__foot_body)
        
        foot_weld = chr.ChLinkMateFix()
        foot_weld.Initialize(self.__foot_body,self.__tibia_body,chr.ChFrameD(start_position+chr.ChVectorD(0,-FEMUR_LENGTH-TIBIA_LENGTH,0)))
        self.__chr_system.Add(foot_weld)
        
    def CreaterJoints(self):
        
        #self.__motor_hip = chr.ChLinkMotorRotationTorque()
        self.__motor_hip = chr.ChLinkMotorRotationAngle()
        self.__motor_hip.Initialize(self.__femur_body,self.__hip_body,chr.ChFrameD(self.hip_coord))
        self.__chr_system.Add(self.__motor_hip)
        
        self.__teta_hip = self.__motor_hip.GetMotorRot()
        
       #torque = chr.ChFunction_Sine(0,1,0.05)
       #self.__motor_hip.SetTorqueFunction(torque)
        
        #self.__motor_knee = chr.ChLinkMotorRotationTorque()
        self.__motor_knee = chr.ChLinkMotorRotationAngle()
        self.__motor_knee.Initialize(self.__tibia_body,self.__femur_body,chr.ChFrameD(self.hip_coord+chr.ChVectorD(0,-FEMUR_LENGTH,0)))
        self.__chr_system.Add(self.__motor_knee)
        
        self.__teta_knee = self.__motor_knee.GetMotorRot()
        
    def PDControl(self,des_teta_hip, des_teta_knee):
        self.__teta_hip = self.__motor_hip.GetMotorRot()
        self.__teta_knee = self.__motor_knee.GetMotorRot()

        func_hip = chr.ChFunction_Const(des_teta_hip[0])
        self.__motor_hip.SetAngleFunction(func_hip)
        
        func_knee = chr.ChFunction_Const(des_teta_knee[0])
        self.__motor_knee.SetAngleFunction(func_knee)
        
        #print(des_teta_hip,des_teta_knee)
        #err_hip = des_teta_hip[0] - self.__teta_hip
        #err_knee = des_teta_knee[0]  - self.__teta_knee
        
        #u_hip = COEF_P_FEMUR*err_hip/80 #+ COEF_D_FEMUR*(-self.__motor_hip.GetMotorRot_dt())
        
        #func_hip = chr.ChFunction_Const(u_hip)
        #self.__motor_hip.SetTorqueFunction(func_hip)
        
        #u_knee = COEF_P_TIBIA*err_knee/80 #+ COEF_D_TIBIA*(-self.__motor_knee.GetMotorRot_dt())
        
        #func_knee = chr.ChFunction_Const(u_knee)
        #self.__motor_knee.SetTorqueFunction(func_knee)

        
    def getTetaHip(self):
        return self.__teta_hip
    
    def getTetaKnee(self):
        return self.__teta_knee
    
    def getHipBody(self):
        return self.__hip_body
        

    def SimulateTest(self, time_stop=6, time_step=0.001):
        myapplication = chrirr.ChIrrApp(self.__chr_system, 'Test_Quadruped_Leg',
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
        ang_hip = []
        omg_hip = []
        
        myapplication.SetTimestep(time_step)
        myapplication.SetTryRealtime(True)
        while(myapplication.GetDevice().run()):
            self.__teta_hip = self.__motor_hip.GetMotorRot()
            self.__teta_knee = self.__motor_knee.GetMotorRot()
            myapplication.BeginScene()
            myapplication.DrawAll()
            chrirr.drawAllLinkframes(self.__chr_system, myapplication.GetVideoDriver(),1)
            chrirr.drawAllLinks(self.__chr_system, myapplication.GetVideoDriver(),1)
            #chrirr.drawAllCOGs(self.__chr_system, myapplication.GetVideoDriver(),1)
            time.append(self.__chr_system.GetChTime())
            ang_hip.append(self.__motor_hip.GetMotorRot())
            omg_hip.append(self.__motor_hip.GetMotorRot_dt())
            myapplication.DoStep()
            myapplication.EndScene()
            if self.__chr_system.GetChTime() > time_stop:
                myapplication.GetDevice().closeDevice()
        return time, ang_hip, omg_hip
    
    def SimulateTestControl(self, time_stop=6, time_step=0.001):
        myapplication = chrirr.ChIrrApp(self.__chr_system, 'Test_Quadruped_Leg_Control',
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
        ang_hip = []
        omg_hip = []
        

        
        myapplication.SetTimestep(time_step)
        myapplication.SetTryRealtime(True)
        while(myapplication.GetDevice().run()):
            self.__teta_hip = self.__motor_hip.GetMotorRot()
            self.__teta_knee = self.__motor_knee.GetMotorRot()
            myapplication.BeginScene()
            myapplication.DrawAll()
            chrirr.drawAllLinkframes(self.__chr_system, myapplication.GetVideoDriver(),1)
            chrirr.drawAllLinks(self.__chr_system, myapplication.GetVideoDriver(),1)
            time.append(self.__chr_system.GetChTime())
            ang_hip.append(self.__motor_hip.GetMotorRot())
            omg_hip.append(self.__motor_hip.GetMotorRot_dt())
            myapplication.DoStep()
            myapplication.EndScene()
            if self.__chr_system.GetChTime() > time_stop:
                myapplication.GetDevice().closeDevice()
        return time, ang_hip, omg_hip