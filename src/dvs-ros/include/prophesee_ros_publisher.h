#ifndef PROPHESEE_ROS_PUBLISHER_H_
#define PROPHESEE_ROS_PUBLISHER_H_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <metavision/sdk/driver/camera.h>
#include "ddsmetadata/msg/dds_meta_data.hpp"
#include <rclcpp/duration.hpp>
#include <cv_bridge/cv_bridge.h>

/// Publishes data from Prophesee sensor to ROS topics
class PropheseeWrapperPublisher: public rclcpp::Node {
 public:
  /// \brief Constructor
  PropheseeWrapperPublisher();

  /// \brief Destructor
  ~PropheseeWrapperPublisher();

  /// \brief Starts the camera and starts publishing data
  void startPublishing();

 private:
  /// \brief Opens the camera
  bool openCamera();

  /// \brief Publishes CD events
  void publishCDEvents();

  /// \brief Publisher for camera info
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  /// \brief Publisher for CD events
  rclcpp::Publisher<ddsmetadata::msg::DDSMetaData>::SharedPtr pub_cd_events_;

  /// \brief Instance of Camera class
  /// Used to access data from a camera
  Metavision::Camera camera_;

  /// \brief Instance of Events Array
  /// Accumulated Array of events
  std::vector<Metavision::EventCD> event_buffer_;

  /// \brief Message for publishing the camera info
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  /// \brief Path to the file with the camera settings (biases)
  std::string biases_file_;

  /// \brief Raw file to read instead of live camera
  std::string raw_file_to_read_;

  /// \brief Camera name in string format
  std::string camera_name_;

  /// \brief Wall time stamps
  rclcpp::Time start_timestamp_, last_timestamp_;

  /// \brief If showing CD events
  bool publish_cd_;

  /// \brief Activity Filter Temporal depth (configuration)
  /// Desirable Temporal depth in micro seconds
  int activity_filter_temporal_depth_;

  /// \brief Event buffer time stamps
  rclcpp::Time event_buffer_start_time_, event_buffer_current_time_;

  /// \brief  Mean gravity value at Earth surface [m/s^2]
  static constexpr double GRAVITY = 9.81;
};

#endif /* PROPHESEE_ROS_PUBLISHER_H_ */
