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

using namespace cv; 
using namespace std;
#define CAMERA_NUM  2

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
    printf("WorkThread start! \n");
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
    }
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData)
    {
        printf("Get data fail!\n");
    }
    unsigned int nDataSize = stParam.nCurValue;

    printf("cam[%s] ready!\n", camSerialNumber);

    while(ros::ok())
    {
        nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            // printf("cam[%s]:Get One Frame![%x]\n", camSerialNumber, nRet);//
            cv::Mat mImg(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);//3通道不可以！！！无论PixelFormat如何设定
            cv::Mat image;
            cv::cvtColor(mImg, image, cv::COLOR_GRAY2BGR);//fisheye接收3通道
            if(strcmp(camSerialNumber , "00F10962704") == 0) 
            {
                header.frame_id="hikrobot";
                // header.stamp=ros::Time::now();
                header.seq = FrameCountL;
                mMsg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();//mono8
                image_1 = *mMsg;
                image_buffer_1_.push_back(image_1); 
                image_seq_buffer_1_.push_back(FrameCountL); 
                FrameCountL++;
            }
            if(strcmp(camSerialNumber , "00F10962718") == 0) 
            {
                header.frame_id="hikrobot";
                // header.stamp=ros::Time::now();
                header.seq = FrameCountR;
                nMsg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();//mono8
                image_2 = *nMsg;
                image_buffer_2_.push_back(image_2);
                image_seq_buffer_2_.push_back(FrameCountR); 
                FrameCountR++;
            }
        }
        else
        {
            printf("cam[%s]:Get One Frame failed![%x]\n", camSerialNumber, nRet);//打印阻塞
        }
        // ROS_INFO("sleep for 0.1s");
        // usleep(100000);
    }
    free(pData);
    // return 0;
}

int findInStampBuffer(unsigned int index)
{
	// Check whether image in image buffer with "index" has corresponding element in timestamp buffer
	// index的有效性，关键在于最初的"/mavros/cam_imu_sync/cam_imu_stamp"话题以及相机驱动最初得到的图像，不能被遗漏，所以需要 trigger_ready_srv_
	// 如何处理异常：某一帧buffer_1收到而buffer_2缺失???
    // pixhawk可靠性差，经常重启才有trigger信号
    cout<<endl<<"timestamp_seq_buffer_:";
    for (int i=0;i<timestamp_seq_buffer_.size();i++)
        cout<<timestamp_seq_buffer_.at(i)<<" ";
    cout<<endl<<"image_seq_buffer_1_:";
    for (int i=0;i<image_seq_buffer_1_.size();i++)
        cout<<image_seq_buffer_1_.at(i)<<" ";
    cout<<endl<<"image_seq_buffer_2_:";
    for (int i=0;i<image_seq_buffer_2_.size();i++)
        cout<<image_seq_buffer_2_.at(i)<<" ";

    unsigned int k = 0;
	while (k < timestamp_seq_buffer_.size() && ros::ok()) 
	{
		if (timestamp_seq_buffer_.at(k)==image_seq_buffer_1_.at(index)) && (timestamp_seq_buffer_.at(k)==image_seq_buffer_1_.at(index))
		{
			return k;
		} 
		else 
		{
			k += 1;
		}
	}
	return -1;
}

void stampAndPublishImage(unsigned int index)
{
    cout<<"index:"<<index<<endl;
	int timestamp_index = findInStampBuffer(index);
    cout<<endl<<"timestamp_index:"<<timestamp_index<<endl;
	if (timestamp_index >= 0) 
	{
		// Copy corresponding images and time stamps
		sensor_msgs::Image image_left;
		sensor_msgs::Image image_right;
		image_left = image_buffer_1_.at(index);//ImagePtr转Image，否则不能直接修改header
		image_right = image_buffer_2_.at(index);

		// Add half of exposure time to the actual trigger time ？？？
		//image.header.stamp在 frameGrabLoop()中已经置零，为何要相加？？只为了能够传递(cam_params_.exposure / 2)吗？？
		// double timestamp = image.header.stamp.toSec() + timestamp_buffer_.at(timestamp_index).frame_stamp.toSec();
		// double timestamp = timestamp_buffer_.at(timestamp_index);

		image_left.header.stamp = timestamp_buffer_.at(timestamp_index);
		image_right.header.stamp = timestamp_buffer_.at(timestamp_index);
		// Publish image in ROS
        left_pub.publish(image_left);
        right_pub.publish(image_right);
        cout << "image left No."<< image_seq_buffer_1_.at(index) 
            << " and image right No."<< image_seq_buffer_2_.at(index)
            <<" are publish with time_sequence No."<< timestamp_seq_buffer_.at(timestamp_index) 
            <<" at time "<< timestamp_buffer_.at(timestamp_index) << endl;
		// Erase published images and used timestamp from buffer
		image_buffer_1_.erase(image_buffer_1_.begin() + index);
		image_buffer_2_.erase(image_buffer_2_.begin() + index);
		image_seq_buffer_1_.erase(image_seq_buffer_1_.begin() + index);
		image_seq_buffer_2_.erase(image_seq_buffer_2_.begin() + index);
		timestamp_seq_buffer_.erase(timestamp_seq_buffer_.begin() + timestamp_index);
		timestamp_buffer_.erase(timestamp_buffer_.begin() + timestamp_index);
		// return 0;
	} 
	else 
	{
        printf("\n No timestamp_index found!\n");
		// return 1;
	}
}

void bufferTimestamp(const mavros_msgs::CamIMUStamp &msg)
{
    timestamp_seq_buffer_.push_back(FrameCountS);
    FrameCountS++;
	timestamp_buffer_.push_back(msg.frame_stamp);
    // cout << "image buffer1 size = "<< image_buffer_1_.size()  << ", image buffer2 size = "
    //     << image_buffer_2_.size() << ", timestamp buffer size = " << timestamp_buffer_.size() << endl;
    if (image_buffer_1_.size()>0 && image_buffer_2_.size()>0 && timestamp_buffer_.size()>0) 
    {
        unsigned int i;
        for (i = 0; i < image_buffer_1_.size() && i < image_buffer_2_.size();i++) 
        {
            stampAndPublishImage(i);
        }
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
    image_transport::ImageTransport it(n);
    left_pub = it.advertise("/hikrobot/image_left", 1);
	right_pub = it.advertise("/hikrobot/image_right", 1);
    ros::Subscriber ros_timestamp_sub_;
    ros_timestamp_sub_ = n.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 1, bufferTimestamp);
    ros::ServiceClient triggerClient_;
    triggerClient_ = n.serviceClient<mavros_msgs::CommandTriggerControl>("/mavros/cmd/trigger_control");
    ros::Rate loop_rate(10);
    // ros::spin(); 

    unsigned int nIndex = 0;
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
	printf("Start %d camera Grabbing Image\n", CAMERA_NUM);

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
        // 设置触发模式
        nRet = MV_CC_SetTriggerMode(handle[i], MV_TRIGGER_MODE_ON);
        // 设置触发源
        nRet = MV_CC_SetTriggerSource(handle[i], MV_TRIGGER_SOURCE_LINE2);
        // 设置帧率
        nRet = MV_CC_SetFrameRate(handle[i], 10.0);
        //手动设置曝光
        nRet = MV_CC_SetExposureTime(handle[i], 25000);
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
        else
        {
            printf("Cam[%d]: thread create!\n",i);
        }
    }

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

    ros::spin(); 
	// while (ros::ok())
	// {
    //     cout << "Grabing Images..." << endl;
    //     // ros::spinOnce();
	// 	loop_rate.sleep();
	// }
    printf("exit\n");
    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // 停止取流
        nRet = MV_CC_StopGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return -1;
        }
        else
        {
            printf("MV_CC_StopGrabbing!\n");
        }
        // 关闭设备
        nRet = MV_CC_CloseDevice(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            return -1;
        }
        else
        {
            printf("MV_CC_CloseDevice!\n");
        }
        // 销毁句柄
        nRet = MV_CC_DestroyHandle(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            return -1;
        }
        else
        {
            printf("MV_CC_DestroyHandle!\n");
        }
    }
    return 0;
}