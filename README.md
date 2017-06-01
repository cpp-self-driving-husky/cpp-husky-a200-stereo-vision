# CPP Husky A200 Stereo Vision
Review Status: Not Reviewed
## Introduction
This document covers Stereoscopic Vision specified in [Cal Poly Pomona's Self Driving Husky A200](https://github.com/cpp-self-driving-husky/cpp-husky-a200-design-doc). Stereoscopic Vision is one way for an autonomous vehicle to determine how far away an object is.
## User Stories
* As a user, I want to be able to rely on the data that the stereoscopic camera is giving to the Husky A200.
## Requirements
* The vehicle shall detect stationary obstacles to avoid.
* The vehicle shall use multiple sensor types that are unrelated in data acquisition method to detecct obstacles.
## Solution
On the Husky A200, a stereoscopic camera will be used along with LIDAR and sonar in order to aid in obstacle avoidance. A stereoscopic camera takes two images (one from each camera) and then creates a disparity map. This disparity map can be used to calculate the depth an object is away from the camera.


## Important Data From ROS Topics
* ### Data from sensor_msgs/Image
    * This topic can give the image matrix data as a single array with size step * rows
    * The step refers to the length of a row in bytes
    * The rows refer to the height of the image

* ### Data from sensor_msgs/CameraInfo
    * This topic contains four arrays that correspond to the matrices D, K, R, and P (all of which are zeroed out when the camera is not calibrated.
    * Matrix D represents the distortion model (which is "plumb_bob" for most cameras)
    * Matrix K represents the raw (distorted) images and is a 3x3 row-major matrix
    * Matrix R represents a rotation matrix. This matrix is used to align the camera coordinate system to the ideal stereo image plane. This is also a 3x3 row-major matrix
    * Matrix P represents a projection/camera matrix. This matrix is the prjection of 3D points in the camera coordinate frame to 2D pixel coordinates uning the focal lengths and the principal point. The projection (x, y) of a 3D point [X Y Z]' onto the rectified image iscalculated by the following:
        * [u v w]' = P * [X Y Z 1]'
        * x = u / w
        * y = v / w
* ### Data from stereo_msgs/DisparityImage (Not sure if we have access to this topic right now)
    * This topic contians an image in the form of a sensor_msgs/Image and it is used to calculate the disparities between the principle points of the two cameras using the equation d = x_l - x_r - (cx_l - cx_r)
    * It also contains the focal length (as a float, f, in pixels) and the baseline (as a float, T, in world units)
        * These two can be used to calculate the depth from the camera using the equation Z = fT / d
    * The min and max disparity are given so that the min and max depth so that we know the points outside the range could not be seen
    * Finally the smallest allowed increment in disparity is given (as a float, delta_d) which allows us to calculate the smallest achievable depth range resolution using the euqation delta_Z = (Z^2/fT)*delta_d
## Terms
* Disparity Map: When two images are taken from slightly different locations, the objects in the image appear to shift. This apparent shifting (or difference in pixel positions) between the two images is called disparity. The disparity between the two images creates the disparity map.
## References
* [ROS Topic sensor_msgs/Image Documentation](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
* [ROS Topic sensor_msgs/CameraInfo Documentation](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
* [ROS Topic stereo_msgs/DisparityImage Documentation](http://docs.ros.org/jade/api/stereo_msgs/html/msg/DisparityImage.html)
