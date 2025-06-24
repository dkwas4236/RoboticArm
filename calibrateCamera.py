import cv2
import numpy as np
import glob

# chessboard dimensions: number of inner corners (rows, columns)
chessboard_size = (7, 7)  
# size of a square on the chessboard
square_size = 1.0

obj_points = []  # 3D points in real-world space
img_points = []  # 2D points in image plane

# prepare object points
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

# load images from directory
images = glob.glob('*.jpg') 

# loop through images and find chessboard corners
for img_path in images:
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if ret:
        img_points.append(corners)
        obj_points.append(objp)

        # draw the corners and show them
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey()

cv2.destroyAllWindows()

# perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# save calibration results to a file
np.savez('camera_calibration_result.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

# display camera matrix and distortion coefficients
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)

# undistort a test image
test_img = cv2.imread('chessboard_image20.jpg') 
h, w = test_img.shape[:2]
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
undistorted_img = cv2.undistort(test_img, camera_matrix, dist_coeffs, None, new_camera_matrix)

# show the undistorted image
cv2.imshow('Undistorted Image', undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
