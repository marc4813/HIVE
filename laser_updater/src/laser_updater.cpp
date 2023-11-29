#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "string.h"
#include <limits>



class LaserScanTimeUpdater{
    public:
        LaserScanTimeUpdater(std::string input_topic, std::string output_topic){
            // Init nodehandle 
            nh_ = ros::NodeHandle("~");

            // Subscribe to topic
            laserSubscriber_ = nh_.subscribe(input_topic, 1, &LaserScanTimeUpdater::laserScanCallback, this);

            // Publish laserscan on "laserupdated" topic
            updated_laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic, 1);
            
        }

        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& input_scan){
            sensor_msgs::LaserScan updated_scan = *input_scan;
            updated_scan.header.stamp = ros::Time::now();

            // Remove points within circle encapsulating the entire agent.
            for(int i = 0; i<665; ++i)
            {
                if(updated_scan.ranges[i] <= 0.3302 && updated_scan.ranges[i] > 0.02)
                    updated_scan.ranges[i] = 0.00000;
            } 

            updated_laser_scan_pub_.publish(updated_scan);
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber laserSubscriber_;
        ros::Publisher updated_laser_scan_pub_; 

};

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_time_updater");

    std::string input_topic = argv[1];
    std::string output_topic = argv[2];

    LaserScanTimeUpdater updater = LaserScanTimeUpdater(input_topic, output_topic);

    ros::spin();
    return 0;
}