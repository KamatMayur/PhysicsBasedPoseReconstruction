import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_pose(results):


    # import matplotlib.pyplot as plt
    x = []
    y= []
    z= []
    for idx , landmark in enumerate(results.pose_landmarks.landmark):
        x.append(landmark.x)
        y.append(landmark.y)
        z.append(landmark.z)

    # plt.plot([int(i) for i in range(len(x))] , x , color ='r')
    # plt.plot( [int(i) for i in range(len(y))] , y , color ='g')
    # plt.plot( [int(i) for i in range(len(z)) ] , z, color ='b')
    # plt.show()


    fig = plt.figure()
    ax = fig.add_subplot(111 , projection='3d')
    # ax.plot_surface(x , y ,z)
    ax.scatter(x, y, z, marker='o', color='b')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()


