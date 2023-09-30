import cv2
import mediapipe as mp
import numpy as np
import matplotlib.pyplot as plt
import csv
import utils.addspine as addspine


mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Create a CSV file to store landmarks
# output_file = 'all_landmarks.csv'
# with open(output_file, mode='w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(['Frame', 'Landmark', 'X', 'Y', 'Z'])

def remove_unwanted_landmarks(pose_landmarks):
    # Define the indices of the landmarks to be removed (1 to 10)
    unwanted_landmarks_indices = list(range(1, 11))
    
    # Create a new list of landmarks excluding unwanted landmarks
    filtered_landmarks = [pose_landmarks[i] for i in range(len(pose_landmarks)) if i not in unwanted_landmarks_indices]
    
    return filtered_landmarks


def position_from_image(input_image):
    image = cv2.imread(input_image)

    frame_rgb = cv2.cvtColor(image , cv2.COLOR_BGR2RGB)
    frame_rgb.flags.writeable = False

    results = pose.process(frame_rgb)
    results = addspine.AddSpine(results = results)

    # if results.pose_landmarks:
    #     for idx, landmark in enumerate(results.pose_landmarks.landmark):
    #         x = landmark.x
    #         y = landmark.y
    #         z = landmark.z


        # # Write the landmarks to the CSV file
        #     with open(output_file, mode='a', newline='') as file:
        #         writer = csv.writer(file)
        #         writer.writerow([1, idx, x, y, z])
    filtered_landmarks = remove_unwanted_landmarks(results.pose_landmarks.landmark)
    results.pose_landmarks.landmark.clear()
    results.pose_landmarks.landmark.extend(filtered_landmarks)

    frame_rgb.flags.writeable = True
    frame_rgb = cv2.cvtColor(frame_rgb , cv2.COLOR_RGB2BGR)

    # mp_drawing.draw_landmarks(frame_rgb , results.pose_landmarks , mp_pose.POSE_CONNECTIONS)
    mp_drawing.draw_landmarks(frame_rgb , results.pose_landmarks )

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
        results = addspine.AddSpine(results = results)

        # Write all landmarks to the CSV file
        # if results.pose_landmarks:
        #     for idx, landmark in enumerate(results.pose_landmarks.landmark):
        #         x = landmark.x
        #         y = landmark.y
        #         z = landmark.z

                # # Write the landmarks to the CSV file
                # with open(output_file, mode='a', newline='') as file:
                #     writer = csv.writer(file)
                #     writer.writerow([frame_count, idx, x, y, z])

        # Display the frame with landmarks (optional)
        filtered_landmarks = remove_unwanted_landmarks(results.pose_landmarks.landmark)
        results.pose_landmarks.landmark.clear()
        results.pose_landmarks.landmark.extend(filtered_landmarks)

        frame_rgb.flags.writeable = True
        frame_rgb = cv2.cvtColor(frame_rgb , cv2.COLOR_RGB2BGR)

        mp_drawing.draw_landmarks(frame_rgb , results.pose_landmarks )

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
        results = addspine.AddSpine(results = results)

        # Write all landmarks to the CSV file
        # if results.pose_landmarks:
        #     for idx, landmark in enumerate(results.pose_landmarks.landmark):
        #         x = landmark.x
        #         y = landmark.y
        #         z = landmark.z

                # # Write the landmarks to the CSV file
                # with open(output_file, mode='a', newline='') as file:
                #     writer = csv.writer(file)
                #     writer.writerow([frame_count, idx, x, y, z])

        # Display the frame with landmarks (optional)
        filtered_landmarks = remove_unwanted_landmarks(results.pose_landmarks.landmark)
        results.pose_landmarks.landmark.clear()
        results.pose_landmarks.landmark.extend(filtered_landmarks)

        frame_rgb.flags.writeable = True
        frame_rgb = cv2.cvtColor(frame_rgb , cv2.COLOR_RGB2BGR)

        mp_drawing.draw_landmarks(frame_rgb , results.pose_landmarks)
        # mp_drawing.draw_landmarks(frame_rgb , results.pose_landmarks , mp_pose.POSE_CONNECTIONS)

        cv2.imshow('Video Stream', frame_rgb)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    return results

# position_from_image("test_image1.jpg")