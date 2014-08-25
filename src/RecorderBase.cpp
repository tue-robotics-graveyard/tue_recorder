#include "tue_recorder/RecorderBase.h"

#include <boost/filesystem.hpp>

Recorder::Recorder() : active_(false) {
}

Recorder::~Recorder() {
}

bool Recorder::init(int argc, char **argv) {
    ros::init(argc, argv, "recorder");

    bool success = true;

    for(int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        size_t i_split = arg.find('=');
        if (i_split == std::string::npos) {
            ROS_ERROR("Invalid parameter: %s", arg.c_str());
            success = false;
        } else {
            std::string key = arg.substr(0, i_split);
            std::string value = arg.substr(i_split + 1);
            params_[key] = value;
        }
    }

    if (!getParam("rec_base_dir", rec_base_dir_)) {
        ROS_ERROR("Parameter 'rec_base_dir' must be set.");
        success = false;
    }

    if (!getParam("name", source_id_)) {
        ROS_ERROR("Parameter 'name' must be set.");
        success = false;
    }

    if (!success) {
        return false;
    }

    return this->initialize();
}


void Recorder::startCallback(const tue_recorder::Start& msg) {
    if (msg.source == source_id_) {
        this->start(msg);
        active_ = true;
    }
}

void Recorder::stopCallback(const tue_recorder::Stop& msg) {
    this->stop();
    active_ = false;
}

void Recorder::run() {
    ros::NodeHandle nh("~");
    sub_start_ = nh.subscribe("/recorder/start", 10, &Recorder::startCallback, this);
    sub_stop_ = nh.subscribe("/recorder/stop", 10, &Recorder::stopCallback, this);

    ros::spin();
}

bool Recorder::getParam(const std::string& key, std::string& value, const std::string& default_value) const {
    std::map<std::string, std::string>::const_iterator it = params_.find(key);
    if (it == params_.end()) {
        if (default_value != "") {
            value = default_value;
            return true;
        } else {
            return false;
        }
    }
    value = it->second;
    return true;
}

std::string Recorder::generateFilename(const tue_recorder::Start& msg) {
    time_t rawtime;
    time (&rawtime);
    struct tm* timeinfo = localtime(&rawtime);

    char date_str_buf[80];
    strftime(date_str_buf, 80,"%Y-%m-%d", timeinfo);
    std::string date_str = date_str_buf;

    char time_str_buf[80];
    strftime(time_str_buf, 80,"%I-%M-%S", timeinfo);
    std::string time_str = time_str_buf;

    // create directory
    boost::filesystem::path dir(rec_base_dir_ + "/" + date_str);
    if (boost::filesystem::create_directories(dir)) {
        std::cout << "Success" << "\n";
    }

    std::string filename = rec_base_dir_ + "/" + date_str + "/" + source_id_ + "-";
    if (!msg.context.empty()) {
        filename += msg.context + "-";
    }
    filename += date_str + "-" + time_str;

    return filename;
}
