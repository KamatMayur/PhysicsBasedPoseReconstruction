# import Physics_Based_Pose_Reconstruction as pbpr

# sim = pbpr.simulation("C:/Users/mayur/Desktop/PhysicsBasedPoseReconstruction/Physics_Based_Pose_Reconstruction_CPP/Physics_Based_Pose_Reconstruction/Mujoco_Models/Humanoid/Ybot.xml")

# sim.simulate()
import threading
import time
import sys
import Physics_Based_Pose_Reconstruction as pbpr
import numpy as np

# adds the control signals
def controller(sim, control_time=10.0, control_frequency=1/30):
    start_time = time.time()
    while time.time() - start_time < control_time:
        print('time elapsed = ', time.time() - start_time)
        arr = np.random.uniform(-0.1, 0.1, sim.info["act_num"])
        sim.controls(arr)
        time.sleep(control_frequency) 
        

sim = pbpr.simulation("C:/Users/mayur/OneDrive/Documents/GitHub/PhysicsBasedPoseReconstruction/Physics_Based_Pose_Reconstruction_CPP/Physics_Based_Pose_Reconstruction/Mujoco_Models/Humanoid/Ybot.xml")


# Creating threads
simulate_thread = threading.Thread(target=sim.simulate)
controller_thread = threading.Thread(target=controller, args=(sim,))

# Starting threads
simulate_thread.start()
controller_thread.start()
 
# Waiting for threads to finish
simulate_thread.join()
controller_thread.join()



# uncomment this if you wanna test saving and loading data
# sim.simulate_from_state(True, 10.0, 1.0/17.0)
# sim.save_data("C:/Users/mayur/Desktop/hum.mjdata")
# sim.load_data("C:/Users/mayur/Desktop/hum.mjdata")
# sim.simulate_from_state(True, 10.0, 1.0/17.0)
# sim.save_data("C:/Users/mayur/Desktop/hum.mjdata")
