#ifndef _RecorderBase_H_
#define _RecorderBase_H_

#include <ros/ros.h>
#include <tue_recorder/Start.h>
#include <tue_recorder/Stop.h>

#include <string>

class Recorder {

public:

    Recorder();

    virtual ~Recorder();

    bool init(int argc, char **argv);

    bool getParam(const std::string& key, std::string& value, const std::string& default_value = std::string()) const;

    void run();

    inline bool isActive() const { return active_; }

protected:

    bool active_;

    std::map<std::string, std::string> params_;

    std::string rec_base_dir_;

    std::string source_id_;

    ros::Subscriber sub_start_;

    ros::Subscriber sub_stop_;    

    virtual bool initialize() { return true; }

    virtual void start(const tue_recorder::Start& msg) {}

    virtual void stop() {}

    void startCallback(const tue_recorder::Start& msg);

    void stopCallback(const tue_recorder::Stop& msg);

    std::string generateFilename(const tue_recorder::Start& msg);

};

#endif
