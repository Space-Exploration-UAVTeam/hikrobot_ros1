#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <hikrobot/exposure.h>
using namespace std;


ros::Publisher count_pub;
int exposure_time = 3200;
int brightness_upper = 120;//想要亮度范围更小，需要降低比例系数
int brightness_lower = 100;
float scale = 1.05;//office室内=1.05；公园=?
void count_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = ptr->image;
    // cout<<"image type:"<<image.type()<<endl;				
    ROS_INFO("count start!");
    uchar B,G,R;
    int gray;
    int avg=0;
    int count=0;
    int raduis=0;

    for(int i=0; i<512; i+=5)//row
    {
    	for(int j=0; j<640; j+=5)//colum
    	{
            raduis = sqrt((i-512)*(i-512) + (j-640)*(j-640));
            if(raduis<360)
            {
                count++;
                B = image.at<cv::Vec3b>(i,j)[0];//0-255
                G = image.at<cv::Vec3b>(i,j)[1];//0-255
                R = image.at<cv::Vec3b>(i,j)[2];//0-255
                gray = R*0.299+G*0.587+B*0.114;//灰度心理学公式
                avg = avg + gray;
            }
    	}
    }
    avg=avg/count;
    cout<<"image left avg brightness = "<<avg<<" with sample point num = "<<count<<endl;				
    ROS_INFO("count end!");

    hikrobot::exposure msg;
    if(exposure_time>200 && exposure_time<40000)
    {
        if(avg < brightness_lower)
        {
            exposure_time *=scale;
        }
        else if(avg > brightness_upper)
        {
            exposure_time /=scale;
        }
    }
    cout<<"exposure_time = "<<exposure_time<<endl;				
    msg.exposure_time = exposure_time;
    count_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    count_pub = n.advertise<hikrobot::exposure>("/hikrobot/exposure", 1);
    ros::Subscriber left_sub = n.subscribe("/hikrobot/image_left", 1, count_callback);
    ros::spin();
    // ros::Rate r(10);//hz
    // while(ros::ok())
    // {
    //     r.sleep();//根据前面的定义的loop_rate,设置0.1s的暂停
    //     ros::spinOnce();//非阻塞
    // }
    return 0;
}