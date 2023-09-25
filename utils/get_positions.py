import cv2
import mediapipe as mp
import numpy as np
import matplotlib.pyplot as plt
import csv

def create_spine(results):


    top_spine_x =(results.pose_landmarks.landmark[11].x + results.pose_landmarks.landmark[12].x)/2
    bottom_spine_x =(results.pose_landmarks.landmark[24].x + results.pose_landmarks.landmark[23].x)/2


    top_spine_y =(results.pose_landmarks.landmark[11].y + results.pose_landmarks.landmark[12].y)/2
    bottom_spine_y =(results.pose_landmarks.landmark[24].y + results.pose_landmarks.landmark[23].y)/2

    top_spine_z =(results.pose_landmarks.landmark[11].z + results.pose_landmarks.landmark[12].z)/2
    bottom_spine_z =(results.pose_landmarks.landmark[24].z + results.pose_landmarks.landmark[23].z)/2

    top_spine = [top_spine_x , top_spine_y , top_spine_z]
    bottom_spine = [bottom_spine_x , bottom_spine_y , bottom_spine_z]

    print(len(results.pose_landmarks.landmark))




mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Create a CSV file to store landmarks
output_file = 'all_landmarks.csv'
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Frame', 'Landmark', 'X', 'Y', 'Z'])


def position_from_image(input_image):
    image = cv2.imread(input_image)

    frame_rgb = cv2.cvtColor(image , cv2.COLOR_BGR2RGB)
    frame_rgb.flags.writeable = False

    results = pose.process(frame_rgb)

    if results.pose_landmarks:
        for idx, landmark in enumerate(results.pose_landmarks.landmark):
            x = landmark.x
            y = landmark.y
            z = landmark.z

        # Write the landmarks to the CSV file
            with open(output_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([1, idx, x, y, z])

    frame_rgb.flags.writeable = True
    frame_rgb = cv2.cvtColor(frame_rgb , cv2.COLOR_RGB2BGR)

    mp_drawing.draw_landmarks(frame_rgb , results.pose_landmarks , mp_pose.POSE_CONNECTIONS)


    top_spine_x =(results.pose_landmarks.landmark[11].x + results.pose_landmarks.landmark[12].x)/2
    bottom_spine_x =(results.pose_landmarks.landmark[24].x + results.pose_landmarks.landmark[23].x)/2


    top_spine_y =(results.pose_landmarks.landmark[11].y + results.pose_landmarks.landmark[12].y)/2
    bottom_spine_y =(results.pose_landmarks.landmark[24].y + results.pose_landmarks.landmark[23].y)/2

    top_spine_z =(results.pose_landmarks.landmark[11].z + results.pose_landmarks.landmark[12].z)/2
    bottom_spine_z =(results.pose_landmarks.landmark[24].z + results.pose_landmarks.landmark[23].z)/2

    top_spine = [top_spine_x , top_spine_y , top_spine_z]
    bottom_spine = [bottom_spine_x , bottom_spine_y , bottom_spine_z]

    print(top_spine , bottom_spine)
    print(results.pose_landmarks.landmark[23] , results.pose_landmarks.landmark[24] )
    print(results.pose_landmarks.landmark[0])
    # print(results.pose_landmarks.landmark[12])




    cv2.imshow('Loaded Image', frame_rgb)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return results



def position_from_webcam():
    frame_count = 0
    cap = cv2.VideoCapture(0)  # Use 0 for the default camera, or specify a video file


    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1

        # Convert the frame to RGB (MediaPipe requires RGB input)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_rgb.flags.writeable = False

        # Run pose estimation on the frame
        results = pose.process(frame_rgb)

        # Write all landmarks to the CSV file
        if results.pose_landmarks:
            for idx, landmark in enumerate(results.pose_landmarks.landmark):
                x = landmark.x
                y = landmark.y
                z = landmark.z

                # Write the landmarks to the CSV file
                with open(output_file, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([frame_count, idx, x, y, z])

        # Display the frame with landmarks (optional)
        frame_rgb.flags.writeable = True
        frame_rgb = cv2.cvtColor(frame_rgb , cv2.COLOR_RGB2BGR)

        mp_drawing.draw_landmarks(frame_rgb , results.pose_landmarks , mp_pose.POSE_CONNECTIONS)

        cv2.imshow('Video Stream', frame_rgb)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
   
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    return results
    

def position_from_video(input_video):

    frame_count = 0

    cap = cv2.VideoCapture(input_video)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1

        # Convert the frame to RGB (MediaPipe requires RGB input)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_rgb.flags.writeable = False

        # Run pose estimation on the frame
        results = pose.process(frame_rgb)

        # Write all landmarks to the CSV file
        if results.pose_landmarks:
            for idx, landmark in enumerate(results.pose_landmarks.landmark):
                x = landmark.x
                y = landmark.y
                z = landmark.z

                # Write the landmarks to the CSV file
                with open(output_file, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([frame_count, idx, x, y, z])

        # Display the frame with landmarks (optional)
        frame_rgb.flags.writeable = True
        frame_rgb = cv2.cvtColor(frame_rgb , cv2.COLOR_RGB2BGR)

        mp_drawing.draw_landmarks(frame_rgb , results.pose_landmarks , mp_pose.POSE_CONNECTIONS)

        cv2.imshow('Video Stream', frame_rgb)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    return results

# position_from_image("test_image1.jpg")