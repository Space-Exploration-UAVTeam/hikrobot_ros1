#include "MvCameraControl.h"
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv; 
using namespace std;
#define MAX_BUF_SIZE    (1280*1024*3)

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

 
// void main()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "MvCameraPub");
    ros::NodeHandle n("~");
    // ROS_INFO("process start!");
    image_transport::ImageTransport it(n);
    image_transport::Publisher left_pub = it.advertise("/hikrobot/image_left", 1);
	image_transport::Publisher right_pub = it.advertise("/hikrobot/image_right", 1);
    sensor_msgs::ImagePtr mMsg;
    sensor_msgs::ImagePtr nMsg;
    std_msgs::Header header;
    ros::Rate loop_rate(10);

	int mRet = -1;
	void* m_handle = NULL;
	//枚举子网内指定的传输协议对应的所有设备
	unsigned int nTLayerType = MV_USB_DEVICE;//MV_GIGE_DEVICE
	MV_CC_DEVICE_INFO_LIST m_stDevList = { 0 };
	mRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);//枚举设备，赋值给m_stDevList
	if (mRet != 0)
	{
		printf("error: EnumDevices fail [%x]\n", mRet);
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
	// //手动设置初始曝光		
	mRet = MV_CC_SetExposureTime(m_handle, 25000);//室外下午500，室内25000
	//开始采集图像
	mRet = MV_CC_StartGrabbing(m_handle);
	if (mRet != 0)
	{
		printf("error: StartGrabbing fail [%x]\n", mRet);
	}

	int nBufSize = MAX_BUF_SIZE;
	unsigned int    nTestFrameSize = 0;
	unsigned char*  mFrameBuf = NULL;
	mFrameBuf = (unsigned char*)malloc(nBufSize);
	MV_FRAME_OUT_INFO_EX stInfo;
	memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	/////////////////上层应用程序需要根据帧率，控制好调用该接口的频率

	while (ros::ok())
	{
		// if (nTestFrameSize > 5000)//帧数限制
		// {
		// 	break;
		// }
		mRet = MV_CC_GetImageForBGR(m_handle, mFrameBuf, nBufSize, &stInfo, 1000);
		// mRet = MV_CC_GetOneFrameTimeout(m_handle, mFrameBuf, nBufSize, &stInfo, 1000);
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
				header.seq = nTestFrameSize;
				cv::Mat mImg(height, width, CV_8UC3, mFrameBuf);//3通道！！！
		        // cv::namedWindow('left', WINDOW_NORMAL);
				// cv::moveWindow('KF-Tracking',0,550);
				// cv::imshow("right", mImg);
				// cv::waitKey(10);
				mMsg = cv_bridge::CvImage(header, "bgr8", mImg).toImageMsg();//mono8
				left_pub.publish(mMsg);
				cout << "image_header:" << header << endl;
			}
			nTestFrameSize++;
		}
		loop_rate.sleep();
	}
  

	//停止采集图像 
	mRet = MV_CC_StopGrabbing(m_handle);
	//关闭设备，释放资源
	mRet = MV_CC_CloseDevice(m_handle);
	//销毁句柄，释放资源
	mRet = MV_CC_DestroyHandle(m_handle);

	return 0;
}
