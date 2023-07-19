import os
import cv2
import numpy as np

# from analysis.MotionArrowsOverlay import overlay_arrows_combined_frames
from depth_trackers.depth_analysis import depth_from_h264_vectors
from mapping.utils.Constants import Constants
from mapping.utils.file import load_camera_data_json


def topdown_view(depth: np.ndarray):
    # depth map to cloud, clip it at 700cm to prevent outliers
    depth[:, 2] = np.clip(depth[:, 2], 0, 700)
    depth[:, :2] = np.squeeze(cv2.undistortPoints(depth[None, :, :2], cam_mat, dist_coeff))*depth[:, 2:]
    # depth is now a Nx3 3d point cloud
    # empty picture
    pic = np.full((200,200,3), 255,dtype=np.uint8)
    for d in depth:
        cv2.circle(pic, (int(d[0]), int(d[1])), 4, (0,0,0), 1)
    return pic


save_video: bool = False  # turn off when saved video not required
show_video: bool = True  # turn off when video window not required
top_down: bool = False  # turn on when working on topdown view
cam_dir = os.path.join(Constants.ROOT_DIR, "mapping_wrappers/camera_config/pi/camera_data_480p.json")
cam_mat, dist_coeff, _ = load_camera_data_json(cam_dir)
path = os.path.join(Constants.ROOT_DIR, "results/depth_test1")
# angles1 = np.loadtxt(os.path.join(path, "tello_angles1.csv"))
# angles2 = np.loadtxt(os.path.join(path, "tello_angles2.csv"))
# best_pair = generate_angle_pairs(angles1, angles2)  # doesn't work
cap1 = cv2.VideoCapture(os.path.join(path, "rot1.h264"))
cap2 = cv2.VideoCapture(os.path.join(path, "rot2.h264"))
if save_video:
    writer = cv2.VideoWriter(os.path.join(path, "depth.mp4"), cv2.VideoWriter_fourcc("M", "P", "4", "V"), 40,
                             (640, 480))
else:
    writer = None  # just to stop warning
# Initialize the feature detector (e.g., ORB, SIFT, etc.)
detector = cv2.ORB_create(nfeatures=1000)
while True:
    # cap2.set(cv2.CAP_PROP_POS_FRAMES, pair - 1)  # seek to best pair, doesn't work
    ret1, frame1 = cap1.read()
    if not ret1:
        if save_video:
            writer.release()
        break
    ret2, frame2 = cap2.read()
    # combined_frame = np.concatenate((frame1, frame2), axis=1)
    # cv2.imshow("frames", combined_frame)
    # ORB
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    # Detect keypoints and compute descriptors for both images
    keypoints1, descriptors1 = detector.detectAndCompute(gray1, None)
    keypoints2, descriptors2 = detector.detectAndCompute(gray2, None)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Perform the matching
    matches = matcher.match(descriptors1, descriptors2)
    print(len(matches), "matches using ORB")
    # show matches
    # frame_with_matches = cv2.drawMatches(frame1, keypoints1, frame2, keypoints2, matches[:100], None)
    # cv2.imshow("ORB matches", frame_with_matches)
    # cv2.waitKey(0)
    # continue
    # show depth
    points1 = np.array([keypoints1[match.queryIdx].pt for match in matches])
    points2 = np.array([keypoints2[match.trainIdx].pt for match in matches])
    depth = depth_from_h264_vectors(np.hstack((points1, points2)), cam_mat,
                                    30)  # you might want to save one of these for the topdown view
    if top_down:
        depth_frame = topdown_view(np.hstack((points1, depth[:, None])))
    else:
        depth_frame = frame1.copy()
        int_points1 = points1.astype(int)
        depth_color = np.clip(depth * 255 / 500, 0, 255)[:, None]  # clip  values from 0 to 5m and scale to 0-255(color range)
        for color, point in zip(depth_color, int_points1):
            cv2.rectangle(depth_frame, point[::] - 5, point[::] + 5, color, -1)
    if show_video:
        cv2.imshow("depth ORB", depth_frame)
        cv2.waitKey(0)  # need some minimum time because opencv doesnt work without it
    if save_video:
        writer.write(depth_frame)

# similar method that uses motion vectors
# points3d = triangulate_points(keypoints1, keypoints2, matches, 60, cam_mat, dist_coeff)
# depth_frame = frame1.copy()
# frame1[(points3d[:, :2]).astype(int)] = 255*(points3d[:, 3]/np.max(points3d[:, 3]))

# Motion Vectors
# vectors = ffmpeg_encode_extract(frame1, frame2, subpixel=False)
# print(len(vectors), "matches using MV")
# combined_frame = np.concatenate((frame1, frame2), axis=1)
# combined_frame = overlay_arrows_combined_frames(combined_frame, vectors, max_vectors=50)
# cv2.imshow("MV matches", combined_frame)
# cv2.waitKey(0)
