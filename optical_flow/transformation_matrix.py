import cv2
import numpy as np
import matplotlib.pyplot as plt

# Initialize Video Capture
cap = cv2.VideoCapture("output_video.mp4")

# Read first frame
ret, frame1 = cap.read()
gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

# Detect Features to Track (Increased Sensitivity)
p0 = cv2.goodFeaturesToTrack(gray1, maxCorners=500, qualityLevel=0.2, minDistance=5)

# Initialize trajectory tracking
trajectory = [(0, 0)]
global_position = np.array([0, 0], dtype=np.float32)

# Sensitivity Parameters
THRESHOLD = 0.5  # Reduced for finer movement detection
SCALE_FACTOR = 0.2  # Increased to amplify small movements
alpha = 0.6  # Lower EMA for faster response

# Initialize EMA
smoothed_dx, smoothed_dy = 0, 0

# Plot Setup
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], 'bo-', markersize=2)
ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)

while True:
    ret, frame2 = cap.read()
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Compute Optical Flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(gray1, gray2, p0, None)

    if p1 is not None and p0 is not None:
        # Estimate Transformation with RANSAC
        M, _ = cv2.estimateAffinePartial2D(p0, p1, method=cv2.RANSAC, ransacReprojThreshold=2.0)

        if M is not None:
            dx, dy = M[0, 2], M[1, 2]

            # Apply Exponential Moving Average (EMA) for smoothing
            smoothed_dx = alpha * smoothed_dx + (1 - alpha) * dx
            smoothed_dy = alpha * smoothed_dy + (1 - alpha) * dy

            # Apply threshold & scale
            if abs(smoothed_dx) > THRESHOLD or abs(smoothed_dy) > THRESHOLD:
                global_position += np.array([smoothed_dx, -smoothed_dy]) * SCALE_FACTOR
                trajectory.append(tuple(global_position))

            # Update Trajectory Plot
            x_vals, y_vals = zip(*trajectory)
            line.set_xdata(x_vals)
            line.set_ydata(y_vals)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.01)

            print(f"Global Position: ({global_position[0]:.2f}, {global_position[1]:.2f})")

    # Update for next frame
    gray1 = gray2.copy()
    p0 = p1

    cv2.imshow("Frame", frame2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
