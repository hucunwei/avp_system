#include "subscriber/ipm_seg_image_subscriber.h"


IPMSegImageSubscriber::IPMSegImageSubscriber(ros::NodeHandle &nh, const std::string &topic) {
    sub_ = nh.subscribe(topic, 10, &IPMSegImageSubscriber::ipmSegImageCallback, this);
}

void IPMSegImageSubscriber::ipmSegImageCallback(const sensor_msgs::CompressedImageConstPtr &msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    image_buffer_.push_back(*msg);
    // Optional: Limit buffer size
    if (image_buffer_.size() > max_buffer_size_) {
        image_buffer_.pop_front();
    }
}

std::deque<sensor_msgs::CompressedImage> IPMSegImageSubscriber::getBuffer() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return image_buffer_;
}

sensor_msgs::CompressedImage IPMSegImageSubscriber::getBufferFront(){
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto result = image_buffer_.front();
    image_buffer_.pop_front();
    return result;
}
    

    