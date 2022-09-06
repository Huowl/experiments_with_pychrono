from cmath import cos
import pychrono.core as chrono
import pychrono.irrlicht as chrirr
import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt

def Feedback(q):
    K = np.asarray([-95.33425181, -44.15974824, -29.72448497, -17.59465727])
    q1 = q[0].GetMotorRot() 
    q2 = q[1].GetMotorRot()
    d_q1 = q[0].GetMotorRot_dt()
    d_q2 = q[1].GetMotorRot_dt()
    
    x = np.asarray([q1, q2, d_q1, d_q2]).T
    
    u = -K @ x
    
    #print(u)
    return u

class AcrobotSystem():
    def __init__(self, chrono_system, len_1, len_2, body_material):
        self.chrono_system = chrono_system
        
        self.len_rod_1 = len_1
        self.len_rod_2 = len_2
        self.density = 1000 # kg/m^3
        self.xz_rod = np.max([len_1, len_2])/10
        self.xyz_cube = np.max([len_1, len_2])/5
        
        self.GetIinertialParams()
        self.CreateBodies(body_material)
        
    def GetIinertialParams(self):
        self.mass_rod_1 = self.xz_rod**2*self.len_rod_1*self.density
        self.mass_rod_2 = self.xz_rod**2*self.len_rod_2*self.density
        self.mass_cube = self.xyz_cube**3

        self.inertia_rod_1 = self.len_rod_1**2*self.mass_rod_1
        self.inertia_rod_2 = self.len_rod_2**2*self.mass_rod_2
        self.inertia_cube = 1/6*self.xyz_cube**2*self.mass_cube
        
        return {"m1":self.mass_rod_1,"m2":self.mass_rod_2,"I1":self.inertia_rod_1,"I2":self.inertia_rod_2}
    
    def CreateBodies(self, body_material):
        mboxtexture = chrono.ChTexture()
        mboxtexture.SetTextureFilename(chrono.GetChronoDataFile('textures/cubetexture_borders.png'))
        
        # Cube/ground
        
        self.body_cube = chrono.ChBody()
        self.body_cube.SetPos(chrono.ChVectorD(0,0,0))
        self.body_cube.SetMass(self.mass_cube)
        self.body_cube.SetInertiaXX(chrono.ChVectorD(self.inertia_cube,self.inertia_cube,self.inertia_cube))
        self.body_cube.SetBodyFixed(True)
        self.body_cube.SetCollide(False)
        self.body_cube.GetCollisionModel().ClearModel()
        self.body_cube.GetCollisionModel().AddBox(body_material,self.xyz_cube/2,self.xyz_cube/2,self.xyz_cube/2)
        self.body_cube.GetCollisionModel().BuildModel()
        
        cube_shape = chrono.ChBoxShape()
        cube_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.xyz_cube/2,self.xyz_cube/2,self.xyz_cube/2)
        self.body_cube.GetAssets().push_back(cube_shape)
        self.body_cube.GetAssets().push_back(mboxtexture)
        
        self.chrono_system.Add(self.body_cube)
        
        # Rod 1
        
        self.body_rod_1 = chrono.ChBody()
        #self.body_rod_1.SetPos(chrono.ChVectorD(0,-0.6*self.xyz_cube - self.len_rod_1/2,0))
        self.body_rod_1.SetPos(chrono.ChVectorD(0, self.len_rod_1/2,0))
        self.body_rod_1.SetRot(chrono.Q_ROTATE_X_TO_Y)
        self.body_rod_1.SetMass(self.mass_rod_1)
        self.body_rod_1.SetInertiaXX(chrono.ChVectorD(self.inertia_rod_1,self.inertia_rod_1,self.inertia_rod_1))
        self.body_rod_1.SetBodyFixed(False)
        self.body_rod_1.SetCollide(False)

        self.body_rod_1.GetCollisionModel().ClearModel()
        self.body_rod_1.GetCollisionModel().AddBox(body_material,self.len_rod_1/2,self.xz_rod/2,self.xz_rod/2)
        self.body_rod_1.GetCollisionModel().BuildModel()

        rod_1_shape = chrono.ChBoxShape()
        rod_1_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.len_rod_1/2,self.xz_rod/2,self.xz_rod/2)
        self.body_rod_1.GetAssets().push_back(rod_1_shape)
        self.body_rod_1.GetAssets().push_back(mboxtexture)
        
        self.chrono_system.Add(self.body_rod_1)
        
        # Rod 2
        
        self.body_rod_2 = chrono.ChBody()
        #self.body_rod_2.SetPos(chrono.ChVectorD(0,-1.2*self.xyz_cube - self.len_rod_2/2 - self.len_rod_1,0))
        self.body_rod_2.SetPos(chrono.ChVectorD(0, self.len_rod_2/2 + self.len_rod_1,0))
        self.body_rod_2.SetRot(chrono.Q_ROTATE_X_TO_Y)
        self.body_rod_2.SetMass(self.mass_rod_2)
        self.body_rod_2.SetInertiaXX(chrono.ChVectorD(self.inertia_rod_2,self.inertia_rod_2,self.inertia_rod_2))
        self.body_rod_2.SetBodyFixed(False)
        self.body_rod_2.SetCollide(False)

        self.body_rod_2.GetCollisionModel().ClearModel()
        self.body_rod_2.GetCollisionModel().AddBox(body_material,self.len_rod_2/2,self.xz_rod/2,self.xz_rod/2)
        self.body_rod_2.GetCollisionModel().BuildModel()

        rod_2_shape = chrono.ChBoxShape()
        rod_2_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.len_rod_2/2,self.xz_rod/2,self.xz_rod/2)
        self.body_rod_2.GetAssets().push_back(rod_2_shape)
        self.body_rod_2.GetAssets().push_back(mboxtexture)
        
        self.chrono_system.Add(self.body_rod_2)
    
    def AddJoints(self):

        # Ground to first rod
        self.joint_rev_cto1 = chrono.ChLinkMotorRotationTorque()
        frame_rev_1 = chrono.ChFrameD(chrono.ChVectorD(0,0,0),chrono.ChQuaternionD(1,0,0,0))
        self.joint_rev_cto1.Initialize(self.body_rod_1,self.body_cube,frame_rev_1)

        self.chrono_system.AddLink(self.joint_rev_cto1)

        # First to second rod
        self.motor_1to2 = chrono.ChLinkMotorRotationTorque()
        #frame_rev_2 = chrono.ChFrameD(chrono.ChVectorD(0,-0.9*self.xyz_cube-self.len_rod_1,0))
        frame_rev_2 = chrono.ChFrameD(chrono.ChVectorD(0,self.len_rod_1,0))
        self.motor_1to2.Initialize(self.body_rod_2,self.body_rod_1,frame_rev_2)
        self.chrono_system.AddLink(self.motor_1to2)
        
    def Simulate(self, time = 20, time_step = 0.005):
        myapplication = chrirr.ChIrrApp(self.chrono_system, 'Acrobot', chrirr.dimension2du(720,720))

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
        
        myapplication.SetTimestep(time_step)
        myapplication.SetTryRealtime(True)

        arr_time = []
        ang1 = []
        ang2 = []
        omg1 = []
        omg2 = []
        input = []
        while(myapplication.GetDevice().run()):
            q = [self.joint_rev_cto1, self.motor_1to2]
            input = Feedback(q)
            if self.chrono_system.GetChTime() < 0.2:
                self.motor_1to2.SetTorqueFunction(chrono.ChFunction_Const(1))
            else:
                self.motor_1to2.SetTorqueFunction(chrono.ChFunction_Const(input))
            
            arr_time.append(self.chrono_system.GetChTime())
            ang1.append(q[0].GetMotorRot())
            ang2.append(q[1].GetMotorRot())
            omg1.append(q[0].GetMotorRot_dt())
            omg2.append(q[1].GetMotorRot_dt())

            myapplication.BeginScene()
            myapplication.DrawAll()
            chrirr.drawAllLinkframes(self.chrono_system, myapplication.GetVideoDriver(),1)
            chrirr.drawAllLinks(self.chrono_system, myapplication.GetVideoDriver(),1)
            myapplication.DoStep()
            myapplication.EndScene()
            if self.chrono_system.GetChTime() > time:
                myapplication.GetDevice().closeDevice()
        return {'time': arr_time, 'angle_1' : ang1, 'angle_2': ang2, 'omega_1': omg1, 'omega_2':omg2}

        
        
        
        