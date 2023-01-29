#!/usr/bin/python3

import depthai as dai
import numpy as np
import rospy
import cv2
import blobconverter
import time
from sensor_msgs.msg import Image
from robot.msg import Coords
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pipeline = dai.Pipeline()
coords = Coords()

pub = rospy.Publisher('/Camera_frames', Image, queue_size=1)
pub2 = rospy.Publisher('robot_coords', Coords, queue_size=1)
rospy.init_node('OakImage', anonymous = False)
rate = rospy.Rate(15)

FRAME_SIZE = (640, 360)

DET_INPUT_SIZE = (544, 320)
model_name = "person-detection-retail-0013"
zoo_type = "depthai"
blob_path = None

def display_info(frame, bbox, coordinates, status, status_color, fps):
    # Display bounding box
    cv2.rectangle(frame, bbox, status_color[status], 2)

    # Display coordinates
    if coordinates is not None:
        coord_x, coord_y, coord_z = coordinates
        cv2.putText(frame, f"X: {int(coord_x)} mm", (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"Y: {int(coord_y)} mm", (bbox[0] + 10, bbox[1] + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"Z: {int(coord_z)} mm", (bbox[0] + 10, bbox[1] + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

    # Create background for showing details
    cv2.rectangle(frame, (5, 5, 175, 100), (50, 0, 0), -1)

    # Display authentication status on the frame
    cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color[status])

    # Display instructions on the frame
    cv2.putText(frame, f'FPS: {fps:.2f}', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255))





def talker():

    cam = pipeline.createColorCamera()
    cam.setPreviewSize(FRAME_SIZE[0], FRAME_SIZE[1])
    cam.setInterleaved(False)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)

    mono_left = pipeline.createMonoCamera()
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right = pipeline.createMonoCamera()
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    stereo = pipeline.createStereoDepth()

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    if model_name is not None:
        blob_path = blobconverter.from_zoo(
            name=model_name,
            shaves=6,
            zoo_type=zoo_type
        )

    face_spac_det_nn = pipeline.createMobileNetSpatialDetectionNetwork()
    face_spac_det_nn.setConfidenceThreshold(0.7)
    face_spac_det_nn.setBlobPath(blob_path)
    face_spac_det_nn.setDepthLowerThreshold(10)
    face_spac_det_nn.setDepthUpperThreshold(5000)

    face_det_manip = pipeline.createImageManip()
    face_det_manip.initialConfig.setResize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])
    face_det_manip.initialConfig.setKeepAspectRatio(False)

    cam.preview.link(face_det_manip.inputImage)
    face_det_manip.out.link(face_spac_det_nn.input)
    stereo.depth.link(face_spac_det_nn.inputDepth)

    x_preview_out = pipeline.createXLinkOut()
    x_preview_out.setStreamName("preview")
    cam.preview.link(x_preview_out.input)


    det_out = pipeline.createXLinkOut()
    det_out.setStreamName('det_out')
    face_spac_det_nn.out.link(det_out.input)

    # Frame count
    frame_count = 0

    # Placeholder fps value
    fps = 0

    # Used to record the time when we processed last frames
    prev_frame_time = 0

    # Used to record the time at which we processed current frames
    new_frame_time = 0

    status_color = {
    'Human Detected': (0, 255, 0),
    'No Human Detected': (0, 0, 255)
    }


    while not rospy.is_shutdown():

        with dai.Device(pipeline) as device:

            # Output queue will be used to get the right camera frames from the outputs defined above
            q_cam = device.getOutputQueue(name="preview", maxSize=1, blocking=False)

            # Output queue will be used to get nn data from the video frames.
            q_det = device.getOutputQueue(name="det_out", maxSize=1, blocking=False)

            # # Output queue will be used to get nn data from the video frames.
            # q_bbox_depth_mapping = device.getOutputQueue(name="bbox_depth_mapping_out", maxSize=4, blocking=False)

            while True:
                # Get right camera frame
                in_cam = q_cam.get()
                frame = in_cam.getCvFrame()

                bbox = None
                coordinates = None

                inDet = q_det.tryGet()

                if inDet is not None:
                    detections = inDet.detections

                    # if face detected
                    if len(detections) is not 0:
                        detection = detections[0]

                        # Correct bounding box
                        xmin = max(0, detection.xmin)
                        ymin = max(0, detection.ymin)
                        xmax = min(detection.xmax, 1)
                        ymax = min(detection.ymax, 1)

                        # Calculate coordinates
                        x = int(xmin * FRAME_SIZE[0])
                        y = int(ymin * FRAME_SIZE[1])
                        w = int(xmax * FRAME_SIZE[0] - xmin * FRAME_SIZE[0])
                        h = int(ymax * FRAME_SIZE[1] - ymin * FRAME_SIZE[1])

                        bbox = (x, y, w, h)

                        # Get spacial coordinates
                        coord_x = detection.spatialCoordinates.x
                        coord_y = detection.spatialCoordinates.y
                        coord_z = detection.spatialCoordinates.z

                        coordinates = (coord_x, coord_y, coord_z)

                        coords.X = float(coord_x)
                        coords.Y = float(coord_y)
                        coords.Z = float(coord_z)

                        pub2.publish(coords)

                # Check if a face was detected in the frame
                if bbox:
                    # Face detected
                    status = 'Human Detected'
                else:
                    # No face detected
                    status = 'No Human Detected'
                    coords.X = 0.0
                    coords.Y = 0.0
                    coords.Z = 0.0  

                pub2.publish(coords)

                # Display info on frame
                display_info(frame, bbox, coordinates, status, status_color, fps)

                # Calculate average fps
                if frame_count % 10 == 0:
                    # Time when we finish processing last 100 frames
                    new_frame_time = time.time()

                    # Fps will be number of frame processed in one second
                    fps = 1 / ((new_frame_time - prev_frame_time) / 10)
                    prev_frame_time = new_frame_time

                # Capture the key pressed
                key_pressed = cv2.waitKey(1) & 0xff

                # Stop the program if Esc key was pressed
                if key_pressed == 27:
                    break

                # Increment frame count
                frame_count += 1

                msg = bridge.cv2_to_imgmsg(frame,"bgr8")

                pub.publish(msg)

                rate.sleep()

                # Check for keyboard input
                key = cv2.waitKey(1)

                if key == ord('q'):
                    # Quit when q is pressed
                    break

                if rospy.is_shutdown():
                    break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
