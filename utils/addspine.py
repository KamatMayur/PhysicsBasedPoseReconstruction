def cal_spine_coordinates(results):
    
    top_spine_x =(results.pose_landmarks.landmark[11].x + results.pose_landmarks.landmark[12].x)/2
    bottom_spine_x =(results.pose_landmarks.landmark[24].x + results.pose_landmarks.landmark[23].x)/2


    top_spine_y =(results.pose_landmarks.landmark[11].y + results.pose_landmarks.landmark[12].y)/2
    bottom_spine_y =(results.pose_landmarks.landmark[24].y + results.pose_landmarks.landmark[23].y)/2

    top_spine_z =(results.pose_landmarks.landmark[11].z + results.pose_landmarks.landmark[12].z)/2
    bottom_spine_z =(results.pose_landmarks.landmark[24].z + results.pose_landmarks.landmark[23].z)/2

    top_spine = [top_spine_x , top_spine_y , top_spine_z]
    bottom_spine = [bottom_spine_x , bottom_spine_y , bottom_spine_z]

    return top_spine , bottom_spine

def AddSpine(results):
    
    top_spine , bottom_spine = cal_spine_coordinates(results=results)

    new_landmark = results.pose_landmarks.landmark.add()
    new_landmark.x = top_spine[0]  
    new_landmark.y = top_spine[1] 
    new_landmark.z = top_spine[2]

    new_landmark = results.pose_landmarks.landmark.add()
    new_landmark.x = bottom_spine[0]  
    new_landmark.y = bottom_spine[1] 
    new_landmark.z = bottom_spine[2]

    return results