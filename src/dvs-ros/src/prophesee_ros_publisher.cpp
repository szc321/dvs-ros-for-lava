#include "prophesee_ros_publisher.h"
#include <unistd.h>
PropheseeWrapperPublisher::PropheseeWrapperPublisher() :
  Node("Dvs_To_DDS_Node"),
  biases_file_(""),
  raw_file_to_read_(""),
  publish_cd_(true) {
  camera_name_ = "PropheseeCamera_optical_frame";

  const std::string topic_cam_info  =
                        "/prophesee/" + camera_name_ + "/camera_info";
  const std::string topic_cd_event_buffer =
                        "/prophesee/" + camera_name_ + "/cd_events_buffer";
  pub_info_ =  rclcpp::Node::create_publisher<sensor_msgs::msg::CameraInfo>(
                    topic_cam_info,
                    rclcpp::SystemDefaultsQoS());

    if (publish_cd_)
        pub_cd_events_ =
            rclcpp::Node::create_publisher<ddsmetadata::msg::DDSMetaData>(
                    topic_cd_event_buffer,
                    rclcpp::SystemDefaultsQoS());

    while (!openCamera()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(rclcpp::Node::get_logger(), "Trying to open camera...");
    }

    // Add camera runtime error callback
    camera_.add_runtime_error_callback(
          [this](const Metavision::CameraException &e) {
            RCLCPP_WARN(rclcpp::Node::get_logger(), "%s", e.what());
          });

    // Get the sensor config
    Metavision::CameraConfiguration config = camera_.get_camera_configuration();
    auto &geometry                        = camera_.geometry();
    RCLCPP_INFO(rclcpp::Node::get_logger(),
                "[CONF] Width:%i, Height:%i",
                geometry.width(), geometry.height());
    RCLCPP_INFO(rclcpp::Node::get_logger(),
                "[CONF] Serial number: %s", config.serial_number.c_str());

    // Publish camera info message
    cam_info_msg_.width           = geometry.width();
    cam_info_msg_.height          = geometry.height();
    cam_info_msg_.header.frame_id = "PropheseeCamera_optical_frame";
}

PropheseeWrapperPublisher::~PropheseeWrapperPublisher() {
    camera_.stop();
    rclcpp::shutdown();
}

bool PropheseeWrapperPublisher::openCamera() {
    bool camera_is_opened = false;

    // Initialize the camera instance
    try {
        if (raw_file_to_read_.empty()) {
            camera_ = Metavision::Camera::from_first_available();

            if (!biases_file_.empty()) {
                RCLCPP_INFO(rclcpp::Node::get_logger(),
                            "[CONF] Loading bias file: %s",
                            biases_file_.c_str());
                camera_.biases().set_from_file(biases_file_);
            }
        } else {
            camera_ = Metavision::Camera::from_file(raw_file_to_read_);
            RCLCPP_INFO(rclcpp::Node::get_logger(),
                        "[CONF] Reading from raw file: %s",
                        raw_file_to_read_.c_str());
        }

        camera_is_opened = true;
    } catch (Metavision::CameraException &e) {
        RCLCPP_WARN(rclcpp::Node::get_logger(), "%s", e.what()); }
    return camera_is_opened;
}

void PropheseeWrapperPublisher::startPublishing() {
    camera_.start();
    start_timestamp_ = rclcpp::Clock().now();
    last_timestamp_  = start_timestamp_;

    if (publish_cd_)
        publishCDEvents();

    rclcpp::Rate loop_rate(5);
    while (rclcpp::ok()) {
        /** Get and publish camera info **/
        cam_info_msg_.header.stamp = rclcpp::Clock().now();
        pub_info_->publish(cam_info_msg_);
        loop_rate.sleep();
    }
}

void PropheseeWrapperPublisher::publishCDEvents() {
    // Initialize and publish a buffer of CD events
  try {
    Metavision::CallbackId cd_callback =
      camera_.cd().add_callback([this](const Metavision::EventCD *ev_begin,
                                      const Metavision::EventCD *ev_end) {
        // Check the number of subscribers to the topic
        if (ev_begin < ev_end) {
          // Compute the current local buffer size with new CD events
          const unsigned int buffer_size = ev_end - ev_begin;
          // Get the current time
          event_buffer_current_time_ = rclcpp::Time((
                      start_timestamp_.nanoseconds()+
                      ev_begin->t * 1000.00));
          /** In case the buffer is empty we set the starting time stamp **/
          if (event_buffer_.empty()) {
              // Get starting time
              event_buffer_start_time_ = event_buffer_current_time_;
          }
          /** Insert the events to the buffer **/
          auto inserter = std::back_inserter(event_buffer_);
          /** When there is not activity filter **/
          std::copy(ev_begin, ev_end, inserter);
          /** Get the last time stamp **/
          event_buffer_current_time_ = rclcpp::Time(
            start_timestamp_.nanoseconds() + (ev_end - 1)->t * 1000.00);
        }

        if ((event_buffer_current_time_ - event_buffer_start_time_) >=
                                    rclcpp::Duration::from_seconds(100.0e-6)) {
            /** Create the message **/
          ddsmetadata::msg::DDSMetaData::UniquePtr dds_buffer(
                                  new ddsmetadata::msg::DDSMetaData());
          char* mdata_ptr = reinterpret_cast<char*>(malloc(13));
          char* dds_info_ptr = reinterpret_cast<char*>(malloc(16));
          int insert_index = 0;
          // Sensor geometry in header of the message
          int64_t buffer_time = event_buffer_current_time_.nanoseconds();
          RCLCPP_INFO(rclcpp::Node::get_logger(),
                      "[CONF] The data buffer_time = %d", buffer_time);
          unsigned int height = camera_.geometry().height();
          unsigned int width  = camera_.geometry().width();

          /** Set the buffer size for the msg **/
          dds_buffer->nd = 1;
          dds_buffer->type = 2;
          dds_buffer->elsize = 1;
          dds_buffer->total_size = event_buffer_.size()*13+16;
          dds_buffer->dims[0] = dds_buffer->total_size;
          dds_buffer->strides[0] = 1;
          for (int i = 1; i < 5; i++) {
              dds_buffer->dims[i] = 0;
              dds_buffer->strides[i] = 0;
          }
          memcpy(dds_info_ptr, &buffer_time, 8);
          memcpy(dds_info_ptr+8, &width, 4);
          memcpy(dds_info_ptr+12, &height, 4);
          dds_buffer->mdata = std::vector<unsigned char>(dds_info_ptr,
                                                         dds_info_ptr+16);
          // Copy the events to the ros buffer format
          for (const Metavision::EventCD *it = std::addressof(event_buffer_[0]);
               it != std::addressof(event_buffer_[event_buffer_.size()]);
               ++it) {
              int64_t time = start_timestamp_.nanoseconds() + it->t * 1000.00;
              RCLCPP_INFO(rclcpp::Node::get_logger(),
                          "[CONF] The data buffer_size = %d",
                          event_buffer_.size());
              RCLCPP_INFO(rclcpp::Node::get_logger(),
                          "[CONF] The data [width, height] = [%d,%d]",
                          width, height);
              RCLCPP_INFO(rclcpp::Node::get_logger(),
                          "[CONF] The data [x,y,p,stamp] = [%d,%d,%d,%ld]",
                          it->x, it->y, it->p, time);
              memcpy(mdata_ptr, &it->x, 2);
              memcpy(mdata_ptr+2, &it->y, 2);
              memcpy(mdata_ptr+4, &it->p, 1);
              memcpy(mdata_ptr+5, &time, 8);
              dds_buffer->mdata.insert(
                            dds_buffer->mdata.begin()+16+insert_index,
                            mdata_ptr, mdata_ptr+13);
              insert_index += 13;
              memset(mdata_ptr, 0, 13);
          }
          rclcpp::Time dds_buffer_finished_timestamp = rclcpp::Clock().now();
          RCLCPP_INFO(rclcpp::Node::get_logger(),
                      "Already publihsed once, end time is %d",
                      dds_buffer_finished_timestamp.nanoseconds());
          free(mdata_ptr);
          free(dds_info_ptr);
          // Publish the message
          pub_cd_events_->publish(std::move(dds_buffer));
          // Clean the buffer for the next itteration
          event_buffer_.clear();
        }
    });
  } catch (Metavision::CameraException &e) {
      RCLCPP_WARN(rclcpp::Node::get_logger(), "%s", e.what());
      publish_cd_ = false;
  }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    PropheseeWrapperPublisher wp;
    wp.startPublishing();
    rclcpp::shutdown();
    return 0;
}
