import cv2
import numpy as np

# Parameters for the calibration pattern
pattern_size = (7, 7)  # Number of internal corners in the calibration pattern
square_size = 0.032  # Size of each square in meters (assuming square_size = 1 cm)

# Parameters for capturing calibration images
num_images = 10

def get_camera_resolution():
    # Create a VideoCapture object for the camera
    cap = cv2.VideoCapture(0)

    # Read a frame from the camera
    ret, frame = cap.read()

    # Get the image width and height
    image_height, image_width, _ = frame.shape

    # Release the camera
    cap.release()

    return image_width, image_height

def capture_calibration_images():
    # Create a VideoCapture object for the camera
    cap = cv2.VideoCapture(2)

    # Get the image width and height of the camera
    image_width, image_height = get_camera_resolution()

    image_counter = 0

    while image_counter < num_images:
        # Read an image from the camera
        ret, image = cap.read()

        # Display the image
        cv2.imshow("Capture Calibration Images", image)

        # Wait for a key press
        key = cv2.waitKey(1)

        # Press 's' to save the current image
        if key == ord('s'):
            image_filename = f"calibration_image_{image_counter}.jpg"
            cv2.imwrite(image_filename, image)
            print(f"Image {image_counter + 1} saved as {image_filename}")
            image_counter += 1

        # Press 'q' to quit capturing images
        if key == ord('q'):
            break

    # Release the camera and close the OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

def perform_calibration(image_width, image_height):
    # Create arrays to store object points and image points from all calibration images
    obj_points = []  # 3D coordinates of the calibration pattern corners
    img_points = []  # 2D coordinates of the detected corners in the calibration images

    # Generate object points for the calibration pattern
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    # Read calibration images and detect corners
    for i in range(num_images):
        image_filename = f"calibration_image_{i}.jpg"
        image = cv2.imread(image_filename)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        # If corners are found, add object points and image points
        if ret:
            obj_points.append(objp)
            img_points.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(image, pattern_size, corners, ret)
            cv2.imshow("Detected Corners", image)
            cv2.waitKey(500)

    # Perform camera calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (image_width, image_height), None, None)

    # Save the calibration matrix and distortion coefficients to a file
    calibration_data = {
        'camera_matrix': mtx,
        'distortion_coefficients': dist
    }
    cv2.FileStorage('camera_calibration.xml', cv2.FILE_STORAGE_WRITE).write('calibration_data', calibration_data)

    print("Camera calibration completed.")

if __name__ == "__main__":
    # Capture calibration images
    capture_calibration_images()

    # Get the image width and height of the camera
    image_width, image_height = get_camera_resolution()

    # Perform camera calibration
    perform_calibration(image_width, image_height)
