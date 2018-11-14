#include<iostream>
#include<algorithm>
#include<fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>


using namespace std;

std::string pkg_path, setting_path;
std::string left_img_path, right_img_path, time_file_path;
std::string bag_name;
void LoadImages(vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimestamps);
void imageToRos(const cv::Mat image, sensor_msgs::Image* image_msg);

void getRosParam(ros::NodeHandle &nh_param)
{
    pkg_path = ros::package::getPath("image_to_bag");
    
    // system config file path
    if (!nh_param.getParam("config_file", setting_path))
	setting_path = "/config/config.yaml";
    
    cv::FileStorage fs(setting_path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::string("Could not open file ") + setting_path;
    fs["left_image_path"] >> left_img_path;
    fs["right_image_path"] >> right_img_path;
    fs["time_file_path"] >> time_file_path;
    fs["bag_name"] >> bag_name;
    fs.release();
    
    bag_name = pkg_path + bag_name;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_bag_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    getRosParam(nh_param);
    ros::Time timestamp_start;
    timestamp_start = ros::Time::now();
    
    std::vector<std::string> vstrImageLeft,vstrImageRight;
    std::vector<double> vTimestamps;
    std::cout << "loading the image path....." << std::endl;
    LoadImages(vstrImageLeft, vstrImageRight, vTimestamps);
    std::cout << "loading the image finished." << std::endl;
    const int nImages = vstrImageLeft.size();
    const uint64_t seconds_To_NanoSeconds = 1e9;
    
    rosbag::Bag my_bag;
    my_bag.open(bag_name, rosbag::bagmode::Write);
    
      // Convert images.
    cv::Mat left_img, right_img;
    for (size_t image_id = 0; image_id < nImages; ++image_id) 
    {
        left_img = cv::imread(vstrImageLeft[image_id]);
	right_img = cv::imread(vstrImageRight[image_id]);
	uint64_t timestamp_now_na;
        ros::Time timestamp_now;
	timestamp_now_na = timestamp_start.toNSec() + vTimestamps[image_id] * seconds_To_NanoSeconds;
	timestamp_now.fromNSec(timestamp_now_na);

        sensor_msgs::Image left_img_msg, right_img_msg;
        imageToRos(left_img, &left_img_msg);
        left_img_msg.header.stamp = timestamp_now;
        left_img_msg.header.frame_id = "0";
        my_bag.write("/cam0/image_raw", timestamp_now, left_img_msg);
	
	imageToRos(right_img, &right_img_msg);
	right_img_msg.header.stamp = timestamp_now;
        right_img_msg.header.frame_id = "1";
        my_bag.write("/cam1/image_raw", timestamp_now, right_img_msg);
    }
    
    printf("finished!!!!");
    return 0;
}

void LoadImages(vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    fTimes.open(time_file_path.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = left_img_path + ss.str() + ".png";
        vstrImageRight[i] = right_img_path + ss.str() + ".png";
    }
}

void imageToRos(const cv::Mat image, sensor_msgs::Image* image_msg) 
{
    cv_bridge::CvImage image_cv_bridge;
    image_cv_bridge.image = image.clone();

    if (image.type() == CV_8U) {
      image_cv_bridge.encoding = "mono8";
    } else if (image.type() == CV_8UC3) {
      image_cv_bridge.encoding = "bgr8";
    }
    image_cv_bridge.toImageMsg(*image_msg);
}

