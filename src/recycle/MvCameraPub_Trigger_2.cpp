#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mavros_msgs/CommandTriggerControl.h"
#include <mavros_msgs/CamIMUStamp.h>
#include <mutex>

using namespace cv; 
using namespace std;
#define MAX_BUF_SIZE    (1280*1024*3)

image_transport::Publisher left_pub;
image_transport::Publisher right_pub;
sensor_msgs::ImagePtr mMsg;
sensor_msgs::ImagePtr nMsg;
sensor_msgs::Image image_1;
sensor_msgs::Image image_2;
std_msgs::Header header;
std::vector<sensor_msgs::Image> image_buffer_1_;
std::vector<sensor_msgs::Image> image_buffer_2_;
std::vector<ros::Time> timestamp_buffer_;
std::vector<int> timestamp_seq_buffer_;//
std::vector<int> image_seq_buffer_1_;
std::vector<int> image_seq_buffer_2_;
int FrameCountS = 0;
int FrameCountL = 0;
int FrameCountR = 0;
std::mutex m_buffer;


bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

static void* WorkThread_1(void* m_handle)
{
    printf("WorkThread_1 start! \n");
    ros::Rate loop_rate(10);
 	int mRet = -1;
	int mBufSize = MAX_BUF_SIZE;
	unsigned char*  mFrameBuf = NULL;
	mFrameBuf = (unsigned char*)malloc(mBufSize);
	MV_FRAME_OUT_INFO_EX stInfo;
	memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	/////////////////上层应用程序需要根据帧率，控制好调用该接口的频率

	while (ros::ok())
	{
		mRet = MV_CC_GetImageForBGR(m_handle, mFrameBuf, mBufSize, &stInfo, 1000);
		if (mRet != 0)
		{
			cout << "error:GetImageForRGB:" << setbase(16) << mRet << endl;
		}
		else
		{
			int width = stInfo.nWidth;
			int height = stInfo.nHeight;
			if (stInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
			{
				header.frame_id="hikrobot";
				header.stamp=ros::Time::now();
				header.seq = FrameCountL;
				cv::Mat mImg(height, width, CV_8UC3, mFrameBuf);//3通道！！！
		        // cv::namedWindow('left', WINDOW_NORMAL);
				// cv::resizeWindow('left', 640, 512);
				// cv::moveWindow('KF-Tracking',0,550);
				// cv::imshow("right", mImg);
				// cv::waitKey(10);
				mMsg = cv_bridge::CvImage(header, "bgr8", mImg).toImageMsg();//mono8
				// right_pub.publish(mMsg);
                image_1 = *mMsg;
				m_buffer.lock();
                image_buffer_1_.push_back(image_1); 
                image_seq_buffer_1_.push_back(FrameCountL); 
				m_buffer.unlock();
                FrameCountL++;			
			}
		}
		loop_rate.sleep();
	}
}
static void* WorkThread_2(void* n_handle)
{
    printf("WorkThread_2 start! \n");
    ros::Rate loop_rate(10);
 	int nRet = -1;
	int nBufSize = MAX_BUF_SIZE;
	unsigned char*  nFrameBuf = NULL;
	nFrameBuf = (unsigned char*)malloc(nBufSize);
	MV_FRAME_OUT_INFO_EX stInfo;
	memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	/////////////////上层应用程序需要根据帧率，控制好调用该接口的频率

	while (ros::ok())
	{
		nRet = MV_CC_GetImageForBGR(n_handle, nFrameBuf, nBufSize, &stInfo, 1000);
		if (nRet != 0)
		{
			cout << "error:GetImageForRGB:" << setbase(16) << nRet << endl;
		}
		else
		{
			int width = stInfo.nWidth;
			int height = stInfo.nHeight;
			if (stInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
			{
				header.frame_id="hikrobot";
				header.stamp=ros::Time::now();
				header.seq = FrameCountR;
				cv::Mat nImg(height, width, CV_8UC3, nFrameBuf);
		        // cv::namedWindow('right', WINDOW_NORMAL);
				// cv::resizeWindow('right', 640, 512);
				// cv::moveWindow('KF-Tracking',0,550);
				// cv::imshow("left", nImg);
				// cv::waitKey(10);
				nMsg = cv_bridge::CvImage(header, "bgr8", nImg).toImageMsg();//mono8
				// left_pub.publish(nMsg);
                image_2 = *nMsg;
				m_buffer.lock();
                image_buffer_2_.push_back(image_2);
                image_seq_buffer_2_.push_back(FrameCountR); 
				m_buffer.unlock();
			}
			FrameCountR++;
		}
		loop_rate.sleep();
	}
}

void stampAndPublishImage(unsigned int index)
{
    cout<<endl<<"timestamp_seq_buffer_:";
    for (int i=0;i<timestamp_seq_buffer_.size();i++)
        cout<<timestamp_seq_buffer_.at(i)<<" ";
    cout<<endl<<"image_seq_buffer_1_:";
    for (int i=0;i<image_seq_buffer_1_.size();i++)
        cout<<image_seq_buffer_1_.at(i)<<" ";
    cout<<endl<<"image_seq_buffer_2_:";
    for (int i=0;i<image_seq_buffer_2_.size();i++)
        cout<<image_seq_buffer_2_.at(i)<<" ";
    // cout<<"index:"<<index<<endl;
	// int timestamp_index = findInStampBuffer(index);
    // cout<<endl<<"timestamp_index:"<<timestamp_index<<endl;
	// if (timestamp_index >= 0) 
	// {
		// Copy corresponding images and time stamps
		sensor_msgs::Image image_left;
		sensor_msgs::Image image_right;
		image_left = image_buffer_1_.at(index);//ImagePtr转Image，否则不能直接修改header
		image_right = image_buffer_2_.at(index);

		// Add half of exposure time to the actual trigger time ？？？
		//image.header.stamp在 frameGrabLoop()中已经置零，为何要相加？？只为了能够传递(cam_params_.exposure / 2)吗？？
		// double timestamp = image.header.stamp.toSec() + timestamp_buffer_.at(timestamp_index).frame_stamp.toSec();
		// double timestamp = timestamp_buffer_.at(timestamp_index);

		image_left.header.stamp = timestamp_buffer_.at(index);
		image_right.header.stamp = timestamp_buffer_.at(index);
		// Publish image in ROS
        left_pub.publish(image_left);
        right_pub.publish(image_right);
        cout << "image left No."<< image_seq_buffer_1_.at(index) 
            << " and image right No."<< image_seq_buffer_2_.at(index)
            <<" are publish with time_sequence No."<< timestamp_seq_buffer_.at(index) 
            <<" at time "<< timestamp_buffer_.at(index) << endl;
		// Erase published images and used timestamp from buffer
        m_buffer.lock();
        image_buffer_1_.erase(image_buffer_1_.begin());
		image_buffer_2_.erase(image_buffer_2_.begin());
		image_seq_buffer_1_.erase(image_seq_buffer_1_.begin());
		image_seq_buffer_2_.erase(image_seq_buffer_2_.begin());
		timestamp_seq_buffer_.erase(timestamp_seq_buffer_.begin());
		timestamp_buffer_.erase(timestamp_buffer_.begin());
        m_buffer.unlock();
	// } 
	// else 
	// {
    //     printf("\n No timestamp_index found!\n");
	// 	// return 1;
	// }
}

void bufferTimestamp(const mavros_msgs::CamIMUStamp &msg)
{
    FrameCountS++;
	m_buffer.lock();
    timestamp_seq_buffer_.push_back(FrameCountS);
	timestamp_buffer_.push_back(msg.frame_stamp);
	m_buffer.unlock();
    if (image_buffer_1_.size()>0 && image_buffer_2_.size()>0 && timestamp_buffer_.size()>0) 
    {
        // unsigned int i;
        // for (i = 0; i < image_buffer_1_.size() && i < image_buffer_2_.size();i++) 
        // {
        //     stampAndPublishImage(i);
        // }
        stampAndPublishImage(0);
    }
    else
    {
        printf("image not enought~\n");
    }
    // Check whether buffer has stale data and if so, throw away oldest
    if (image_buffer_1_.size() > 10) 
    {
        image_buffer_1_.erase(image_buffer_1_.begin());
        ROS_ERROR_THROTTLE(1, "image_buffer_1_ Dropping image");
    }
    if (image_buffer_2_.size() > 10) 
    {
        image_buffer_2_.erase(image_buffer_2_.begin());
        ROS_ERROR_THROTTLE(1, "image_buffer_2_ Dropping image");
    }
    if (timestamp_buffer_.size() > 10) 
    {
        timestamp_seq_buffer_.erase(timestamp_seq_buffer_.begin());
        timestamp_buffer_.erase(timestamp_buffer_.begin());
        ROS_ERROR_THROTTLE(1, "timestamp_buffer_ Dropping stamp and sequence");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MvCameraPub");
    ros::NodeHandle n("~");
    ROS_INFO("process start!");
	int Exposure_Time;
	n.getParam("ExposureTime", Exposure_Time);
    image_transport::ImageTransport it(n);
    left_pub = it.advertise("/hikrobot/image_left", 1);
	right_pub = it.advertise("/hikrobot/image_right", 1);
    ros::Subscriber ros_timestamp_sub_;
    ros_timestamp_sub_ = n.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 1, bufferTimestamp);
    ros::ServiceClient triggerClient_;
    triggerClient_ = n.serviceClient<mavros_msgs::CommandTriggerControl>("/mavros/cmd/trigger_control");
    // ros::Rate loop_rate(10);
    // ros::spin(); 

///////////////////////////////////////////////////////////////0311
 	int mRet = -1;
	int nRet = -1;
	void* m_handle = NULL;
	void* n_handle = NULL;
	//枚举子网内指定的传输协议对应的所有设备
	unsigned int nTLayerType = MV_USB_DEVICE;//MV_GIGE_DEVICE
	MV_CC_DEVICE_INFO_LIST m_stDevList = { 0 };
	nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);//枚举设备，赋值给m_stDevList
	if (nRet != 0)
	{
		printf("error: EnumDevices fail [%x]\n", nRet);
	}
	//打印设备信息
	if (m_stDevList.nDeviceNum > 0)
	{
		for (int i = 0; i < m_stDevList.nDeviceNum; i++)
		{
			printf("[Camera %d]:\n", i);
			MV_CC_DEVICE_INFO* pDeviceInfo = m_stDevList.pDeviceInfo[i];
			if (NULL != pDeviceInfo)
			{
				PrintDeviceInfo(pDeviceInfo);            
			} 
		}  
	} 
	else
	{
		printf("no camera found!\n");
	} 
	////////////////////////////////选择查找到的在线设备，创建设备句柄
	int mDeviceIndex = 0;//第一台设备
	MV_CC_DEVICE_INFO m_stDevInfo = { 0 };
	memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[mDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
	mRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);//创建句柄m_handle
	if (mRet != 0)
	{
		printf("error: CreateHandle fail [%x]\n", mRet);
	}
	//连接设备
	unsigned int mAccessMode = MV_ACCESS_Exclusive;
	unsigned short mSwitchoverKey = 0;
	mRet = MV_CC_OpenDevice(m_handle, mAccessMode, mSwitchoverKey);
	if (mRet != 0)
	{
		printf("error: OpenDevice fail [%x]\n", mRet);
	}

	// 设置触发模式
	mRet = MV_CC_SetTriggerMode(m_handle, MV_TRIGGER_MODE_OFF);
	// mRet = MV_CC_SetTriggerMode(m_handle, MV_TRIGGER_MODE_ON);
	// 设置触发源
	mRet = MV_CC_SetTriggerSource(m_handle, MV_TRIGGER_SOURCE_LINE2);

	//设置自动曝光与曝光上下限
	//mRet = MV_CC_SetExposureAutoMode(m_handle, 2);
	//mRet = MV_CC_SetAutoExposureTimeLower(m_handle, 4567);
	//mRet = MV_CC_SetAutoExposureTimeUpper(m_handle, 99999);
	// //手动设置曝光		
	mRet = MV_CC_SetExposureTime(m_handle, Exposure_Time);/////////////室外下午500，室内15000
	//开始采集图像
	mRet = MV_CC_StartGrabbing(m_handle);
	if (mRet != 0)
	{
		printf("error: StartGrabbing fail [%x]\n", mRet);
	}

	////////////////////////////////选择查找到的在线设备，创建设备句柄
 	int nDeviceIndex = 1;//第二台设备
	MV_CC_DEVICE_INFO n_stDevInfo = { 0 };
	memcpy(&n_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
	nRet = MV_CC_CreateHandle(&n_handle, &n_stDevInfo);//创建句柄n_handle
	if (nRet != 0)
	{
		printf("error: CreateHandle fail [%x]\n", nRet);
	}
	//连接设备
	unsigned int nAccessMode = MV_ACCESS_Exclusive;
	unsigned short nSwitchoverKey = 0;
	nRet = MV_CC_OpenDevice(n_handle, nAccessMode, nSwitchoverKey);
	if (nRet != 0)
	{
		printf("error: OpenDevice fail [%x]\n", nRet);
	}

	// 设置触发模式
	nRet = MV_CC_SetTriggerMode(n_handle, MV_TRIGGER_MODE_OFF);
	// nRet = MV_CC_SetTriggerMode(n_handle, MV_TRIGGER_MODE_ON);
	// 设置触发源
	nRet = MV_CC_SetTriggerSource(n_handle, MV_TRIGGER_SOURCE_LINE2);

	//设置自动曝光与曝光上下限
	//nRet = MV_CC_SetExposureAutoMode(n_handle, 2);
	//nRet = MV_CC_SetAutoExposureTimeLower(n_handle, 4567);
	//nRet = MV_CC_SetAutoExposureTimeUpper(n_handle, 99999);
	//手动设置曝光
	nRet = MV_CC_SetExposureTime(n_handle, Exposure_Time);/////////////室外下午500，室内15000
	//开始采集图像
	nRet = MV_CC_StartGrabbing(n_handle);
	if (nRet != 0)
	{
		printf("error: StartGrabbing fail [%x]\n", nRet);
	}

    pthread_t mThreadID;
    mRet = pthread_create(&mThreadID, NULL ,WorkThread_1 , m_handle);
    pthread_t nThreadID;
    nRet = pthread_create(&nThreadID, NULL ,WorkThread_2 , n_handle);

///////////////////////////////////////////////////////////////////
    // ROS_INFO("sleep for 0.2s");
    // usleep(200000);
    mavros_msgs::CommandTriggerControl srv_;
    srv_.request.trigger_enable = false;
    if (triggerClient_.call(srv_)) 
    {
        ROS_INFO("Successfully disabled camera trigger");
    } else {
        ROS_ERROR("Failed to call trigger_control service");
        return -1;
    }
    srv_.request.trigger_enable = true;
    if (triggerClient_.call(srv_)) 
    {
        ROS_INFO("Successfully enabled camera trigger");
    } else {
        ROS_ERROR("Failed to call trigger_control service");
        return -1;
    }


////////////////////////////////////////////////////////////////////
    ros::spin(); 
	// while (ros::ok())
	// {
    //     cout << "Grabing Images..." << endl;
    //     // ros::spinOnce();
	// 	loop_rate.sleep();
	// }
    printf("exit\n");
	//停止采集图像 
	mRet = MV_CC_StopGrabbing(m_handle);
	nRet = MV_CC_StopGrabbing(n_handle);
	//关闭设备，释放资源
	mRet = MV_CC_CloseDevice(m_handle);
	nRet = MV_CC_CloseDevice(n_handle);
	//销毁句柄，释放资源
	mRet = MV_CC_DestroyHandle(m_handle);
	nRet = MV_CC_DestroyHandle(n_handle);

    return 0;
}