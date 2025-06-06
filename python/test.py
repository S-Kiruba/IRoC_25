# import cv2

# cap = cv2.VideoCapture(0)  # Use 0 or camera index

# # Read first frame
# ret, prev_frame = cap.read()
# prev_frame = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# while True:
#     ret, curr_frame = cap.read()
#     if not ret:
#         break

#     curr_frame = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

#     # TODO: Process VO here

#     cv2.imshow("Live Feed", curr_frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

#     prev_frame = curr_frame.copy()

# cap.release()
# cv2.destroyAllWindows()


# import cv2
# import numpy as np

# # Camera parameters (replace with your calibration)
# K = np.array([[718.8560, 0, 320],
#               [0, 718.8560, 240],
#               [0, 0, 1]])

# cap = cv2.VideoCapture(0)
# if not cap.isOpened():
#     raise Exception("Cannot open camera")

# # Parameters for Shi-Tomasi corner detection
# feature_params = dict(maxCorners=2000,
#                       qualityLevel=0.01,
#                       minDistance=7,
#                       blockSize=7)

# # Parameters for Lucas-Kanade optical flow
# lk_params = dict(winSize=(21, 21),
#                  maxLevel=3,
#                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

# # Take first frame, detect corners
# ret, old_frame = cap.read()
# if not ret:
#     raise Exception("Failed to grab first frame")

# old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
# p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

# # Trajectory drawing setup
# traj = np.zeros((600, 600, 3), dtype=np.uint8)
# pose = np.eye(4)
# alpha = 0.9
# smoothed_pose = np.eye(4)
# scale_factor = 0.1

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break
#     frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Calculate optical flow (track features)
#     p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

#     # Select good points
#     if p1 is None or st is None:
#         # No points tracked, reset
#         p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
#         old_gray = frame_gray.copy()
#         continue

#     good_new = p1[st == 1]
#     good_old = p0[st == 1]

#     if len(good_new) > 8:
#         # Compute Essential matrix
#         E, mask = cv2.findEssentialMat(good_old, good_new, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)

#         if E is not None and E.shape == (3, 3):
#             # Recover pose
#             _, R, t, mask_pose = cv2.recoverPose(E, good_old, good_new, K)

#             t_norm = t / np.linalg.norm(t)

#             Rt = np.eye(4)
#             Rt[:3, :3] = R
#             Rt[:3, 3] = (t_norm * scale_factor).flatten()

#             pose = pose @ np.linalg.inv(Rt)
#             smoothed_pose[:3, 3] = alpha * smoothed_pose[:3, 3] + (1 - alpha) * pose[:3, 3]

#             x = int(smoothed_pose[0, 3] * 100) + 300
#             z = int(smoothed_pose[2, 3] * 100) + 300
#             if 0 <= x < 600 and 0 <= z < 600:
#                 cv2.circle(traj, (x, z), 2, (0, 255, 0), -1)
#                 cv2.putText(traj, f"x={smoothed_pose[0,3]:.2f}, z={smoothed_pose[2,3]:.2f}",
#                             (10, 580), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

#     # Draw tracked points on frame
#     for i, (new, old) in enumerate(zip(good_new, good_old)):
#         a, b = new.ravel()
#         c, d = old.ravel()
#         cv2.circle(frame, (int(a), int(b)), 3, (0, 0, 255), -1)
#         cv2.line(frame, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 1)

#     cv2.imshow('Live Feed - Optical Flow Tracking', frame)
#     cv2.imshow('Trajectory', traj)

#     # Update for next iteration
#     old_gray = frame_gray.copy()
#     p0 = good_new.reshape(-1, 1, 2)

#     # If tracked points are too few, detect new features
#     if len(p0) < 150:
#         p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()

import cv2
import numpy as np

K = np.array([[718.8560, 0, 320],
              [0, 718.8560, 240],
              [0, 0, 1]])

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise Exception("Cannot open camera")

orb = cv2.ORB_create(nfeatures=3000)

lk_params = dict(winSize=(21, 21),
                 maxLevel=3,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

traj = np.zeros((600, 600, 3), dtype=np.uint8)
pose = np.eye(4)
alpha = 0.9
scale_factor = 0.1

ret, old_frame = cap.read()
if not ret:
    raise Exception("Cannot grab first frame")

old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
kp = orb.detect(old_gray, None)
p0 = cv2.KeyPoint_convert(kp).reshape(-1, 2)

frame_idx = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Frame capture failed")
        break

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if len(p0) < 200:
        print(f"[INFO] Low tracked points ({len(p0)}), redetecting features")
        kp = orb.detect(frame_gray, None)
        p0 = cv2.KeyPoint_convert(kp).reshape(-1, 2)

    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0.reshape(-1,1,2), None, **lk_params)

    if p1 is None or st is None:
        print("[WARNING] Optical flow tracking lost all points. Redetecting features.")
        kp = orb.detect(frame_gray, None)
        p0 = cv2.KeyPoint_convert(kp).reshape(-1, 2)
        old_gray = frame_gray.copy()
        continue

    good_new = p1.reshape(-1, 2)[st.flatten() == 1]
    good_old = p0.reshape(-1, 2)[st.flatten() == 1]

    print(f"[Frame {frame_idx}] Tracked points: {len(good_new)}")

    if len(good_new) > 8:
        E, mask = cv2.findEssentialMat(good_old, good_new, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)

        if E is not None and E.shape == (3,3):
            _, R, t, mask_pose = cv2.recoverPose(E, good_old, good_new, K)
            t_norm = t / (np.linalg.norm(t) + 1e-8)

            trans_norm = np.linalg.norm(t)
            print(f"Translation norm: {trans_norm:.6f}")

            # Only update pose if translation is significant
            if trans_norm > 0.005:
                Rt = np.eye(4)
                Rt[:3,:3] = R
                Rt[:3,3] = (t_norm * scale_factor).flatten()
                pose = pose @ np.linalg.inv(Rt)
                print(f"Updated pose: translation = {pose[:3,3]}")

                x = int(pose[0,3] * 100) + 300
                z = int(pose[2,3] * 100) + 300

                if 0 <= x < 600 and 0 <= z < 600:
                    cv2.circle(traj, (x, z), 2, (0, 255, 0), -1)
            else:
                print("Translation too small, skipping pose update.")
        else:
            print("[WARNING] Essential matrix not found or invalid.")
    else:
        print("[WARNING] Not enough good points for pose recovery.")

    # Draw tracking points and lines
    for new_pt, old_pt in zip(good_new, good_old):
        a, b = new_pt.ravel()
        c, d = old_pt.ravel()
        cv2.circle(frame, (int(a), int(b)), 3, (0, 0, 255), -1)
        cv2.line(frame, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 1)

    cv2.imshow("Live Feed", frame)
    cv2.imshow("Trajectory", traj)

    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1, 2)

    frame_idx += 1

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("Quitting...")
        break

cap.release()
cv2.destroyAllWindows()

