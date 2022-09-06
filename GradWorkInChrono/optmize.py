import pychrono.core as chr
import pychrono.irrlicht as chrirr
import numpy as np
import matplotlib.pyplot as plt

import control_locomotion as ctrl
import leg as l
import quadruped_corpus as corpus
import simulate

from scipy.optimize import differential_evolution


def CostFunc(p, gait_period, eval_times, quadruped, desired_velocity):
    points = np.array(p[0:np.size(p)-2])
    offsets_legs = {"front": p[np.size(p)-2], "rear":p[-1]}
    #print(points)
    #print(offsets_legs)
    str_coord =chr.ChVectorD(0,0.5,0) 
    quadruped = corpus.quadruped(str_coord)
    sim = simulate.Simulate(quadruped,0.01,10)
    locomotion = ctrl.ControlLocomotion(points,gait_period,eval_times,offsets_legs)
    quadruped.setLocomotion(locomotion)
    sim.startNoAnimation()
    reward = sim.getReward(desired_velocity)
    del sim, quadruped, locomotion, str_coord
    print(f"reward: {reward}")
    return reward

num_points = 6
desired_velocity = 0.375
gait_period = 1.2
eval_times = 0.0001

str_coord =chr.ChVectorD(0,0.44,0) 
quadruped = corpus.quadruped(str_coord)
sim = simulate.Simulate(quadruped,0.01,10)

'''
max_iteratation = 10
population_size = 20

bound_fr_hip = [(-np.pi/3, np.pi/3) for i in range(num_points)]
bound_fr_knee = [(-np.pi/6, np.pi/6) for i in range(num_points)]
bound_rr_hip = [(-np.pi/3, np.pi/3) for i in range(num_points)]
bound_rr_knee = [(-np.pi/6, np.pi/6) for i in range(num_points)]
bound_offsets = [(30, 60) for i in range(2)]
bounds = (bound_fr_hip + bound_fr_knee +
          bound_rr_hip + bound_rr_knee +
          bound_offsets)

result = differential_evolution(CostFunc,bounds,
                                strategy="randtobest1exp",
                                maxiter = max_iteratation,
                                popsize= population_size,
                                args = (gait_period,eval_times,
                                        quadruped,desired_velocity))
print(result)
'''
'''
     fun: 0.2855690518667741
 message: 'Maximum number of iterations has been exceeded.'
    nfev: 5747
     nit: 10
 success: False
       x: array([-8.27036336e-01, -9.14139875e-01, -8.01905144e-01,  4.36161226e-03,
       -4.73355355e-01, -6.48437845e-01, -7.00414117e-02, -3.21588811e-01,
       -1.75154797e-01,  3.33727809e-01,  2.18355551e-01, -2.75041830e-01,
       -8.08453009e-01,  5.97527854e-01, -6.70705811e-01,  1.77950699e-01,
       -4.44435191e-01,  8.91545400e-01,  4.98765246e-01,  4.73386779e-01,
       -4.42799625e-01,  5.50894017e-02,  2.77285461e-01,  4.98817220e-01,
        4.38495449e+01,  5.90734996e+01])
'''
x = np.array(([-8.27036336e-01, -9.14139875e-01, -8.01905144e-01,  4.36161226e-03,
       -4.73355355e-01, -6.48437845e-01, -7.00414117e-02, -3.21588811e-01,
       -1.75154797e-01,  3.33727809e-01,  2.18355551e-01, -2.75041830e-01,
       -8.08453009e-01,  5.97527854e-01, -6.70705811e-01,  1.77950699e-01,
       -4.44435191e-01,  8.91545400e-01,  4.98765246e-01,  4.73386779e-01,
       -4.42799625e-01,  5.50894017e-02,  2.77285461e-01,  4.98817220e-01,
        4.38495449e+01,  5.90734996e+01]))

points = np.array(x[0:np.size(x)-2])
offsets_legs = {"front": x[np.size(x)-2], "rear":x[-1]}
str_coord =chr.ChVectorD(0,0.5,0) 
quadruped = corpus.quadruped(str_coord)
sim = simulate.Simulate(quadruped,0.01,10)
locomotion = ctrl.ControlLocomotion(points,gait_period,eval_times,offsets_legs)
quadruped.setLocomotion(locomotion)

sim.initilizeAnimation()
sim.start()
reward = sim.getReward(desired_velocity)
print(reward)