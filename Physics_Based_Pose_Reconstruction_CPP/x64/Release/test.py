import Physics_Based_Pose_Reconstruction as pbpr
import multiprocessing

path = 'C:/Users/mayur/Desktop/PhysicsBasedPoseReconstruction/Physics_Based_Pose_Reconstruction_CPP/Physics_Based_Pose_Reconstruction/Mujoco_Models/Humanoid/Ybot.xml'
sim = pbpr.simulation(path)

# Function to run the simulation
def run_simulation(path):
    sim.simulate()

if __name__ == "__main__":
    # Create a separate process for the simulation
    sim_process = multiprocessing.Process(target=run_simulation, args=(path,))

    # Start the simulation process
    sim_process.start()

    # Print "Hello world" while the simulation is running
    print("Hello world")
    x = 0
    while x != 10000:
        print(sim.info['y'])
        print(x, '\n')
        x += 1
   
    # Optionally, you can wait for the simulation process to finish
    sim_process.join()