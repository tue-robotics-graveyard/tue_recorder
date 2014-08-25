#include "tue_recorder/KinectRecorder.h"

KinectRecorder::KinectRecorder() {
}

KinectRecorder::~KinectRecorder() {
}

bool KinectRecorder::initialize() {
    if (!getParam("depth_topic", depth_topic_) || !getParam("rgb_topic", rgb_topic_)
            || !getParam("depth_info_topic", depth_info_topic_) || !getParam("rgb_info_topic", rgb_info_topic_)) {
        ROS_ERROR("Parameters 'rgb_topic', 'depth_topic', 'rgb_info_topic' and 'depth_info_topic' must be set.");
        return false;
    }

    return true;
}

void KinectRecorder::start(const tue_recorder::Start& msg) {
    if (!isActive()) {
        start_time_ = ros::Time::now();
        config_ = msg;

        ros::NodeHandle nh;
        sub_rgb_image_ = nh.subscribe(rgb_topic_, 1, &KinectRecorder::rgbImageCallback, this);
        sub_depth_image_ = nh.subscribe(depth_topic_, 1, &KinectRecorder::depthImageCallback, this);
        sub_rgb_info_ = nh.subscribe(depth_info_topic_, 1, &KinectRecorder::rgbInfoCallback, this);
        sub_depth_info_ = nh.subscribe(depth_info_topic_, 1, &KinectRecorder::depthInfoCallback, this);

        last_rgb_image_msg_.header.stamp = ros::Time(0);
        last_depth_image_msg_.header.stamp = ros::Time(0);
        last_rgb_info_msg_.header.stamp = ros::Time(0);
        last_depth_info_msg_.header.stamp = ros::Time(0);

        filename_ = generateFilename(config_) + ".bag";
        bag_out_.open(filename_, rosbag::bagmode::Write);
//        bag_out_.setCompression(rosbag::compression::BZ2);   // compression takes a considerable amount of time (order of 0.5 seconds per snapshot)
    }
}

void KinectRecorder::stop() {
    sub_rgb_image_.shutdown();
    sub_depth_image_.shutdown();
    sub_rgb_info_.shutdown();
    sub_depth_info_.shutdown();
}

void KinectRecorder::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (isActive()) {
        last_rgb_image_msg_ = *msg;

        if (last_depth_image_msg_.header.stamp != ros::Time(0) && last_rgb_info_msg_.header.stamp != ros::Time(0) &&
                last_depth_info_msg_.header.stamp != ros::Time(0)) {
//            std::cout << std::abs((last_rgb_image_msg_.header.stamp - last_depth_image_msg_.header.stamp).toSec()) << std::endl;

            ros::Time current_time = ros::Time::now();
            if (config_.frequency == 0 || (current_time - last_snapshot_time_).toSec() > (1.0 / config_.frequency)) {

                ROS_INFO_STREAM("Taking snapshot at time " << current_time);

                bag_out_.write("rgb", last_rgb_image_msg_.header.stamp, last_rgb_image_msg_);
                bag_out_.write("depth", last_depth_image_msg_.header.stamp, last_depth_image_msg_);
                bag_out_.write("rgb_info", last_rgb_image_msg_.header.stamp, last_rgb_info_msg_);
                bag_out_.write("depth_info", last_depth_image_msg_.header.stamp, last_depth_info_msg_);

                if (config_.frequency == 0 || ros::Time::now() > start_time_ + config_.max_duration) {
                    bag_out_.close();
                    Recorder::stopCallback(tue_recorder::Stop());
                }

                last_snapshot_time_ = current_time;
            }
        }
    }
}

void KinectRecorder::depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (isActive()) {
        last_depth_image_msg_ = *msg;
    }
}

void KinectRecorder::rgbInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (isActive()) {
        last_rgb_info_msg_ = *msg;
    }
}

void KinectRecorder::depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (isActive()) {
        last_depth_info_msg_ = *msg;
    }
}

int main(int argc, char **argv) {
    KinectRecorder recorder;
    if (!recorder.init(argc, argv)) {
        return 1;
    }
    recorder.run();
}
