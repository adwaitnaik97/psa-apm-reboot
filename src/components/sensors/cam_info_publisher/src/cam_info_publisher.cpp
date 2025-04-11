#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string>

class CalibrationPublisher {
 public:
  CalibrationPublisher() : it_(nh_) {
    ros::NodeHandle private_nh("~");

    // subscription and publications
    image_sub = it_.subscribe("image_raw", 1,
                              &CalibrationPublisher::ImageCallback, this);
    pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 10, true);

    // read camera_info from ros param server
    cv::Mat _camera_matrix, _distortion_coefficients,
            _rectification_matrix, _projection_matrix;
    std::string _distortion_model;
    int _image_width, _image_height;

    private_nh.getParam("distortion_model", _distortion_model);
    private_nh.getParam("image_width", _image_width);
    private_nh.getParam("image_height", _image_height);
    ReadMatrixFromParam(private_nh, std::string("camera_matrix"),
                        _camera_matrix);
    ReadMatrixFromParam(private_nh, std::string("distortion_coefficients"),
                        _distortion_coefficients);
    ReadMatrixFromParam(private_nh, std::string("rectification_matrix"),
                       _rectification_matrix);
    ReadMatrixFromParam(private_nh, std::string("projection_matrix"),
                        _projection_matrix);

    // write all params to msg
    cam_info_msg_.distortion_model = _distortion_model;
    cam_info_msg_.width = _image_width;
    cam_info_msg_.height = _image_height;
    for (int row = 0; row < _camera_matrix.rows; row++) {
      for (int col = 0; col < _camera_matrix.cols; col++) {
        cam_info_msg_.K[row * _camera_matrix.cols + col] =
            _camera_matrix.at<double>(row, col);
      }
    }
    for (int row = 0; row < _distortion_coefficients.rows; row++) {
      for (int col = 0; col < _distortion_coefficients.cols; col++) {
        cam_info_msg_.D.push_back(
            _distortion_coefficients.at<double>(row, col));
      }
    }
    for (int row = 0; row < _rectification_matrix.rows; row++) {
      for (int col = 0; col < _rectification_matrix.cols; col++) {
        cam_info_msg_.R[row * _rectification_matrix.cols + col] =
            _rectification_matrix.at<double>(row, col);
      }
    }
    for (int row = 0; row < _projection_matrix.rows; row++) {
      for (int col = 0; col < _projection_matrix.cols; col++) {
        cam_info_msg_.P[row * _projection_matrix.cols + col] =
            _projection_matrix.at<double>(row, col);
      }
    }

    // defaults for these should be fine
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/RegionOfInterest.html
    // cam_info_msg_.binning_x = 1;
    // cam_info_msg_.binning_y = 1;
    // cam_info_msg_.roi.do_rectify = true;
  }

 private:

  // ROS sub/pub params
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Publisher pub_;
  image_transport::Subscriber image_sub;

  // cam_info msg to be published
  // http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  sensor_msgs::CameraInfo cam_info_msg_;

  void ImageCallback(const sensor_msgs::ImageConstPtr& img_msg) {
    cam_info_msg_.header = img_msg->header;
    cam_info_msg_.width = img_msg->width;
    cam_info_msg_.height = img_msg->height;
    pub_.publish(cam_info_msg_);
  }

  void ReadMatrixFromParam(const ros::NodeHandle& nh,
                              const std::string& param, cv::Mat& cv_matrix) const {
    // 1D vector for temporarily holding matrix elements
    std::vector<double> _data;

    // matrix dimensions
    int _rows, _cols;

    // read from param server
    nh.getParam(param + std::string("/data"), _data);
    nh.getParam(param + std::string("/rows"), _rows);
    nh.getParam(param + std::string("/cols"), _cols);

    // init matrix
    cv::Mat(_rows, _cols, CV_64F, _data.data()).copyTo(cv_matrix);
  }
};

int main(int argc, char** argv) {
  // ROS NODE INIT
  ros::init(argc, argv, "calibration_publisher");
  const CalibrationPublisher cal_pub;
  ros::spin();
  return 0;
}
