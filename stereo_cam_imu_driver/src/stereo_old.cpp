#include "cdatacapture.h"
#include <ros/ros.h>
#include <ros/time.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"
#include "std_msgs/Float64.h"

cyusb_handle *h1;
CDataCapture *m_pDataCapture;
unsigned char *data=new unsigned char[4];
void Cs(int handle){
    m_pDataCapture->Close();

};
void setCamera(int index){
    memset(data,0,2);
    data[0]=0x02;  //index&0xff;
       
        
    data[1] = 10;
    if(h1!=NULL)
    cyusb_control_write(h1,0x40,0xd0,0,0,data,2,100);

    //addded by songle
    data[0] = 10;
    cyusb_control_write(h1,0x40,0xde,0,0,data,1,100);
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stereo_ros");
    int r=cyusb_open();
    int usbNum=0;
    if(r<0)
    {
        ROS_INFO("error opening lib");
        cyusb_close();
    return 0;
    }
    else if(r==0)
    {
        ROS_INFO("no device found");
        cyusb_close();
        return 0;
    }
    h1=cyusb_gethandle(usbNum);
    r=cyusb_kernel_driver_active(h1,0);
    if(r!=0)
    {
        ROS_INFO("kernel driver active, exitting");
        cyusb_close();
        return 0;
    }
    // r = cyusb_claim_interface(h1, 0);
    if ( r != 0 ) {
        ROS_INFO("Error in claiming interface\n\t");
        cyusb_close();
        return 0;
    }

    m_pDataCapture = new CDataCapture();
    m_pDataCapture->Open(h1);
    signal(SIGINT, Cs);
    setCamera(20);
    m_pDataCapture->Run();
    return 0;
}

