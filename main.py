import utils

results = utils.getpositions.position_from_image('test_image1.jpg')

utils.plot3dcoordinates.plot_pose(results=results)


## to be done 
'''
1. try to make the spine look better
2. properly update the csv with the new landmarks
3. make the gym env
4. access the env from py
5. might have to include another func for ploting the pose from 3d model
'''
# utils.getcoordinates.get_coordinates()

print(len(results.pose_landmarks.landmark))