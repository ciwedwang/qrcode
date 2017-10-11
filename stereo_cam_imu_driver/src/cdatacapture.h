#ifndef CDATACAPTURE_H
#define CDATACAPTURE_H
#include <string>
#include <stdint.h>
#include <string.h>
#include "cyusb.h"
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include <cv_bridge/cv_bridge.h>
#include "imgframe.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <csignal>
#include <unistd.h>
typedef unsigned char byte;
class CDataCapture
{

public:
    CDataCapture();
  
    int Open(cyusb_handle *ch);//CDataProcess *pProcess);
    int Close();
    int Input( byte* lpData,unsigned int dwSize );
    void Run();
    unsigned int pair_id;
    int fillAxisIMU(short *imu,byte* buffer);
    int bytesToIMU(byte* bufbyte,int len,void* output,int outsel=0);
    //int GetFrame(byte *m_pInData,unsigned long pointer,unsigned long len,int Cam_Num);
private:
    int			m_iCount;		//数据计数器
    int			m_iRowIndex;	//行索引
    bool        m_bFindDbFive;	//标记是否找到55
    byte*		m_pInData;		//接收数据缓冲
    byte*		m_pOutData;		//输出数据缓冲
    byte* m_pOutDataLeft;
    byte* m_pOutDataRight;
    byte* m_pIMU;
    std::string wname;
    //wqueue<imgFrame*>  *disqueue;
    //imgFrame *inputframe;
    ros::NodeHandle comm_nh, param_nh, left_nh, right_nh;
    ros::Publisher IMU_pubGs;
    ros::Publisher pub_expotime;
    image_transport::ImageTransport it;  
    cyusb_handle *h2;
    camera_info_manager::CameraInfoManager info_mgr_left;
    camera_info_manager::CameraInfoManager info_mgr_right;
    image_transport::Publisher pub, pub_left, pub_right, pub_concat;
    bool f_capture;
    byte *m_pReadBuff;
    int lastRowIdx;
    bool b_header;
    bool b_imu;
    std::unique_ptr<DFrameStruct> dFrame;

};

#endif // CDATACAPTURE_H
