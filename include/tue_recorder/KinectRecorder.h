#ifndef TUE_LOGGER_KINECT_LOGGER_H_
#define TUE_LOGGER_KINECT_LOGGER_H_

#include "RecorderBase.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <rosbag/bag.h>

class KinectRecorder : public Recorder {

public:

    KinectRecorder();

    virtual ~KinectRecorder();

protected:

    std::string depth_topic_, rgb_topic_, rgb_info_topic_, depth_info_topic_;

    ros::Subscriber sub_rgb_image_;
    ros::Subscriber sub_depth_image_;
    ros::Subscriber sub_rgb_info_;
    ros::Subscriber sub_depth_info_;

    sensor_msgs::Image last_rgb_image_msg_;
    sensor_msgs::Image last_depth_image_msg_;
    sensor_msgs::CameraInfo last_rgb_info_msg_;
    sensor_msgs::CameraInfo last_depth_info_msg_;

    tue_recorder::Start config_;

    ros::Time start_time_;

    std::string filename_;
    ros::Time last_snapshot_time_;

    rosbag::Bag bag_out_;

    bool initialize();

    void start(const tue_recorder::Start& msg);

    void stop();

    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);

    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);

    void rgbInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

    void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

};

#endif
