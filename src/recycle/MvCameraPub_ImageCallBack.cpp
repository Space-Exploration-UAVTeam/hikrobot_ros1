// https://blog.csdn.net/konglingshneg/article/details/89084543
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

int nRet = MV_OK;
void* handle = NULL;
int nRet2 = MV_OK;
void* handle2 = NULL;
int FrameCount = 0;
image_transport::Publisher left_pub;
sensor_msgs::ImagePtr mMsg;
std_msgs::Header header;

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
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{    
    if (pFrameInfo)
    {
        printf("Get left Frame: Width[%d], Height[%d], nFrameNum[%d]\n", 
            pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);
    }
    cv::Mat mImg(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);//3通道不可以！！！无论PixelFormat如何设定
    cv::imshow("left", mImg);
    cv::waitKey(10);   

    header.frame_id="hikrobot";
    header.stamp=ros::Time::now();
    header.seq = FrameCount;
    mMsg = cv_bridge::CvImage(header, "bgr8", mImg).toImageMsg();//mono8
    left_pub.publish(mMsg); 
    FrameCount++;
}
void __stdcall ImageCallBackEx2(unsigned char * pData2, MV_FRAME_OUT_INFO_EX* pFrameInfo2, void* pUser2)
{    
    if (pFrameInfo2)
    {
        printf("Get right Frame: Width[%d], Height[%d], nFrameNum[%d]\n", 
            pFrameInfo2->nWidth, pFrameInfo2->nHeight, pFrameInfo2->nFrameNum);
    }
    cv::Mat mImg2(pFrameInfo2->nHeight, pFrameInfo2->nWidth, CV_8UC1, pData2);//3通道不可以！！！无论PixelFormat如何设定
    cv::imshow("right", mImg2);
    cv::waitKey(10);   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MvCameraPub");
    ros::NodeHandle n("~");
    ROS_INFO("process start!");
    image_transport::ImageTransport it(n);
    left_pub = it.advertise("/hikrobot/image_left", 1);
    // sensor_msgs::ImagePtr mMsg;
    // std_msgs::Header header;
    ros::Rate loop_rate(1);

    // ch:枚举设备 | Enum device
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    //打印设备信息
    if (stDeviceList.nDeviceNum > 0)
    {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            PrintDeviceInfo(pDeviceInfo);            
        }  
    } 
    else
    {
        printf("Find No Devices!\n");
    }

    //////////////////////////////////////////////////////////
    unsigned int nIndex = 0;
    // ch:选择设备并创建句柄 | Select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    // ch:打开设备 | Open device
    nRet = MV_CC_OpenDevice(handle);
    //设置相机曝光时间
    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    // Sleep(10);//如果不加这个sleep，曝光时间的设置会失败。
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", 25000);
    //设置帧率
    nRet = MV_CC_SetFrameRate(handle, 10.0);
    //设置图片格式
    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_RGB8_Packed);//PixelType_Gvsp_RGB8_Packed，PixelType_Gvsp_Mono8
    // ch:设置触发模式为off 
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    // // ch:设置触发模式为on 
    // nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_ON);
    // //设置Enum型参数
    // unsigned int enMode = MV_TRIGGER_SOURCE_LINE0; //设置触发源为软触发
    // ch:设置为硬触发模式 | en:Set trigger mode as hardware trigger
    // nRet = MV_CC_SetEnumValue(handle, "TriggerSource", enMode);
    // ch:注册抓图回调 | en:Register image callback
    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
    // ch:开始取流 | 自带循环！！！频率为FPS！！！
    nRet = MV_CC_StartGrabbing(handle);

    // //////////////////////////////////////////////////////////
    nIndex = 1;
    // ch:选择设备并创建句柄 | Select device and create handle
    nRet2 = MV_CC_CreateHandle(&handle2, stDeviceList.pDeviceInfo[nIndex]);
    // ch:打开设备 | Open device
    nRet2 = MV_CC_OpenDevice(handle2);
    //设置相机曝光时间
    nRet2 = MV_CC_SetEnumValue(handle2, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    // Sleep(10);//如果不加这个sleep，曝光时间的设置会失败。
    nRet2 = MV_CC_SetFloatValue(handle2, "ExposureTime", 25000);
    //设置帧率
    nRet2 = MV_CC_SetFrameRate(handle2, 10.0);
    //设置图片格式
    nRet2 = MV_CC_SetEnumValue(handle2, "PixelFormat", PixelType_Gvsp_Mono8);//PixelType_Gvsp_RGB8_Packed，PixelType_Gvsp_Mono8
    // ch:设置触发模式为off 
    nRet2 = MV_CC_SetEnumValue(handle2, "TriggerMode", MV_TRIGGER_MODE_OFF);
    // // ch:设置触发模式为on 
    // nRet2 = MV_CC_SetEnumValue(handle2, "TriggerMode", MV_TRIGGER_MODE_ON);
    // //设置Enum型参数
    // unsigned int enMode = MV_TRIGGER_SOURCE_LINE0; //设置触发源为软触发
    // ch:设置为硬触发模式 | en:Set trigger mode as hardware trigger
    // nRet2 = MV_CC_SetEnumValue(handle2, "TriggerSource", enMode);
    // ch:注册抓图回调 | en:Register image callback
    nRet2 = MV_CC_RegisterImageCallBackEx(handle2, ImageCallBackEx2, handle2);
    // ch:开始取流 | 自带循环！！！频率为FPS！！！
    nRet2 = MV_CC_StartGrabbing(handle2);

	while (ros::ok())
	{
        cout << "Grabing Images..." << endl;
		loop_rate.sleep();
	}
    // ch:停止取流
    nRet = MV_CC_StopGrabbing(handle);
    // ch:关闭设备 | en:Close device
    nRet = MV_CC_CloseDevice(handle);
    // ch:销毁句柄 | en:Destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    // ch:停止取流
    // nRet2 = MV_CC_StopGrabbing(handle2);
    // // ch:关闭设备 | en:Close device
    // nRet2 = MV_CC_CloseDevice(handle2);
    // // ch:销毁句柄 | en:Destroy handle
    // nRet2 = MV_CC_DestroyHandle(handle2);
    return 0;
}