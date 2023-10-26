#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
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
#define CAMERA_NUM  2

image_transport::Publisher left_pub;
image_transport::Publisher right_pub;
sensor_msgs::ImagePtr mMsg;
sensor_msgs::ImagePtr nMsg;
std_msgs::Header header;

int FrameCountL = 0;
int FrameCountR = 0;

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

static void* WorkThread(void* pUser)
{
    int nRet = MV_OK;

    MVCC_STRINGVALUE stStringValue = {0};
    char camSerialNumber[256] = {0};
    nRet = MV_CC_GetStringValue(pUser, "DeviceSerialNumber", &stStringValue);
    if (MV_OK == nRet)
    {
        memcpy(camSerialNumber, stStringValue.chCurValue, sizeof(stStringValue.chCurValue));
    }
    else
    {
        printf("Get DeviceUserID Failed! nRet = [%x]\n", nRet);
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return NULL;
    }
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData)
    {
        return NULL;
    }
    unsigned int nDataSize = stParam.nCurValue;

    while(1)
    {
        nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            cv::Mat mImg(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);//3通道不可以！！！无论PixelFormat如何设定
            cv::Mat image;
            cv::cvtColor(mImg, image, cv::COLOR_GRAY2BGR);//fisheye接收3通道            
            if(strcmp(camSerialNumber , "00F10962718") == 0) 
            {
                header.frame_id="hikrobot";
                header.stamp=ros::Time::now();
                header.seq = FrameCountL;
                mMsg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();//mono8
                left_pub.publish(mMsg); 
                FrameCountL++;
                printf("cam[%s]:Pub Left!\n", camSerialNumber);
                // cv::imshow("left", mImg);//卡死
                // cv::waitKey(10);   
            }
            if(strcmp(camSerialNumber , "00F10962704") == 0) 
            {
                header.frame_id="hikrobot";
                header.stamp=ros::Time::now();
                header.seq = FrameCountR;
                mMsg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();//mono8
                right_pub.publish(mMsg); 
                FrameCountR++;
                printf("cam[%s]:Pub Right!\n", camSerialNumber);
                // cv::imshow("right", mImg);//卡死
                // cv::waitKey(10); 
            }
        }
        // else
        // {
        //     printf("cam[%s]:Get One Frame failed![%x]\n", camSerialNumber, nRet);//打印阻塞
        // }
    }
    free(pData);
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MvCameraPub");
    ros::NodeHandle n("~");
    ROS_INFO("process start!");
    image_transport::ImageTransport it(n);
    left_pub = it.advertise("/hikrobot/image_left", 1);
	right_pub = it.advertise("/hikrobot/image_right", 1);
    ros::Rate loop_rate(10);

    int nRet = MV_OK;
    void* handle[CAMERA_NUM] = {NULL};

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return -1;
    }
    unsigned int nIndex = 0;
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            } 
            PrintDeviceInfo(pDeviceInfo);            
        }  
    } 
    else
    {
        printf("Find No Devices!\n");
        return -1;
    }

    if(stDeviceList.nDeviceNum < CAMERA_NUM)
    {
        printf("only have %d camera\n", stDeviceList.nDeviceNum);
        return -1;
    }
	
	// 提示为多相机测试
	printf("Start %d camera Grabbing Image test\n", CAMERA_NUM);

    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // printf("Please Input Camera Index: ");
        // scanf("%d", &nIndex);
        nIndex = i;
        // 选择设备并创建句柄
        nRet = MV_CC_CreateHandle(&handle[i], stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            MV_CC_DestroyHandle(handle[i]);
            return -1;
        }
        // 打开设备
        nRet = MV_CC_OpenDevice(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            MV_CC_DestroyHandle(handle[i]);
            return -1;
        }
    }

    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // 设置触发模式为off
        nRet = MV_CC_SetEnumValue(handle[i], "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_SetTriggerMode fail! nRet [%x]\n", i, nRet);
        }

        //设置帧率
        nRet = MV_CC_SetFrameRate(handle[i], 10.0);

        // 开始取流
        nRet = MV_CC_StartGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_StartGrabbing fail! nRet [%x]\n",i, nRet);
            return -1;
        }
        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL ,WorkThread , handle[i]);
        if (nRet != 0)
        {
            printf("Cam[%d]: thread create failed.ret = %d\n",i, nRet);
            return -1;
        }
    }

    // PressEnterToExit();
	while (ros::ok())
	{
        cout << "Grabing Images..." << endl;
		loop_rate.sleep();
	}

    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // 停止取流
        nRet = MV_CC_StopGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return -1;
        }
        // 关闭设备
        nRet = MV_CC_CloseDevice(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            return -1;
        }
        // 销毁句柄
        nRet = MV_CC_DestroyHandle(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            return -1;
        }
    }
    printf("exit\n");
    return 0;
}