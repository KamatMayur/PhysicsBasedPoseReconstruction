# import Physics_Based_Pose_Reconstruction as pbpr

# sim = pbpr.simulation("C:/Users/mayur/Desktop/PhysicsBasedPoseReconstruction/Physics_Based_Pose_Reconstruction_CPP/Physics_Based_Pose_Reconstruction/Mujoco_Models/Humanoid/Ybot.xml")

# sim.simulate()
import threading
import time
import Physics_Based_Pose_Reconstruction as pbpr

x = True
def print_info(sim):
    global x
    while True:
        x = not x 
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if x : print(sim.add_actuator_value("elbow_left", +1.0))
            else : print(sim.add_actuator_value("elbow_left", -1.0))
            if elapsed_time > 0.5 : break

        try:
            print(sim.info)
        except UnicodeDecodeError as e:
            print(f"UnicodeDecodeError: {e}")
        time.sleep(.6)
        

# Creating an instance of the class
sim = pbpr.simulation("C:/Users/mayur/OneDrive/Documents/GitHub/PhysicsBasedPoseReconstruction/Physics_Based_Pose_Reconstruction_CPP/Physics_Based_Pose_Reconstruction/Mujoco_Models/Humanoid/Ybot.xml")

# Creating threads
simulate_thread = threading.Thread(target=sim.simulate)
print_thread = threading.Thread(target=print_info, args=(sim,))

# Starting threads
simulate_thread.start()
print_thread.start()

# Waiting for threads to finish
print_thread.join()
simulate_thread.join()

