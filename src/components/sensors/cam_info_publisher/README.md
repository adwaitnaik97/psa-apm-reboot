### Cam Info Publisher

This nodes publishes the camera intrinsics to <image_namespace>/camera_info as defined in a YAML file. Each camera_info publication is triggered by an image publication. See the sample launch file for param details.

#### The ROS camera_calibration package outputs a YAML file containing:

image_width, image_height, camera_name,

distortion_model, distortion_coefficients

camera_matrix (9 elements),

rectification_matrix (9 elements = Identity for my monocular camera),

projection_matrix (12 elements)

#### The camera_info message contains:

width, height,

distortion_model, D

K (9 elements) - "Intrinsic camera matrix for the raw (distorted) images"

R (9 elements) - "Rectification matrix (stereo cameras only) A rotation matrix aligning the camera coordinate system to the ideal stereo image plane so that epipolar lines in both stereo images are parallel."

P (12 elements) - "intrinsic matrix of the processed (rectified) image.... Normally, monocular cameras will also have R = the identity and P[1:3,1:3] = K"

#### mappings

camera_matrix is mapped into K
projection_matrix into P
distortion_model and distortion_coefficients into distortion_model and D

other values are left as default.

width, height and the header including timestamp are copied from the incoming message

For more info see:

http://wiki.ros.org/image_pipeline/CameraInfo

https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
