import Physics_Based_Pose_Reconstruction as pbpr
import threading
import queue

# Create a queue to share data between threads
result_queue = queue.Queue()

# Define a function to run the simulation
def run_simulation():
    pbpr.simulate("C:/Users/mayur/OneDrive/Documents/GitHub/PhysicsBasedPoseReconstruction/Physics_Based_Pose_Reconstruction_CPP/Physics_Based_Pose_Reconstruction/Mujoco_Models/Humanoid/ybot.xml")
    result = pbpr.get_jnt_pos()
    result_queue.put(result)  # Put the result in the queue

# Start the simulation thread
sim_thread = threading.Thread(target=run_simulation)
sim_thread.start()

# Continue to access values while the simulation is running
while True:
    try:
        val = result_queue.get_nowait()
        print(val.jnt_num)
        # Process the result as needed
    except queue.Empty:
        # Queue is empty, so the simulation hasn't finished yet
        # You can do other work or just wait here
        pass

    # Add a condition to break the loop if the simulation is done
    if not sim_thread.is_alive():
        break