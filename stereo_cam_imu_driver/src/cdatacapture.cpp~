#include "cdatacapture.h"
#include "ros/ros.h"
#include <unistd.h>
#include <time.h>
#include <cmath>
#include <boost/thread/thread.hpp>
#include "geometry_msgs/Vector3.h"

#define STANDARD_G (((double)9.794)/1000.0f)
#define dps_to_radia (3.14159/180.0f)
using namespace sensor_msgs;
ros::Time current_time;
ros::Duration dt(0.2);  //5HZ
ros::Time imu_time;
ros::Time stamp;
int g_width = 640;
int g_height = 480;
long ReadDataBytes = 1024*1024*4;
bool first_time = true;
geometry_msgs::Vector3 v;
geometry_msgs::PointStamped expotime_;
float angular_factor = 1000 * 0.0174533 / 32768;
float accel_factor = 4.0 * 9.794 / 32768.0;
void bias_fcn(geometry_msgs::Vector3 msg) {
    v = msg;
}


int CDataCapture::bytesToIMU(byte* bufbyte,int len,void* output,int outsel)  //unsigned int* uiout,float *fout,int outsel=0)
{
    //float output;
    union
    {
        float fvalue;
        short uint16;
        unsigned int time;
        byte b[4];
    } utemp;

    for(int n=0; n<4; n++)
    {
        utemp.b[n] = 0;
    }

    for(int i=0; i<len; i++)
    {
        byte tb = *(bufbyte+len-i-1);
        //byte tb = *(bufbyte+i);
        utemp.b[i] = tb;
    }
    /**((byte*)(&output) + 3) = b0;
    *((byte*)(&output) + 2) = b1;
    *((byte*)(&output) + 1) = b2;
    *((byte*)(&output) + 0) = b3;*/
    if(len == 2)
        //*(float*)output=utemp.fvalue;
        *(short*) output = utemp.uint16;
    if(len == 4)
        *(unsigned int*)output = utemp.time;
    return len;
}

int CDataCapture::fillAxisIMU(short *imu, byte* buffer)
{
    void* tempNUM = new void*;
    int k = 0;
    for(int j=0; j<3; j++,k+=2)
    {
        bytesToIMU(buffer+k, 2, tempNUM,0);  
        *(imu+j)=*(short*)tempNUM;
    }
    delete tempNUM;

    return k;
}

CDataCapture::CDataCapture() :
    comm_nh(""), it(comm_nh), param_nh("~"), left_nh("cam0"), right_nh("cam1"), info_mgr_left(left_nh, "cameraLeft"), info_mgr_right(right_nh, "cameraRight")
{
    // it = ros::NodeHandle("");
    std::unique_ptr<byte*> test;
    m_iCount = 0;
    m_iRowIndex = 0;
    m_bFindDbFive = false;
    m_pInData = NULL;
    m_pOutData = NULL;
    m_pReadBuff = NULL;
    f_capture = true;
    b_header = false;
    b_imu = false;
    IMU_pubGs = comm_nh.advertise<sensor_msgs::Imu>("imu0", 20);
    //  IMU_pubGs = comm_nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10);
    pub_left = it.advertise("cam0//image_raw", 1);
    pub_right = it.advertise("cam1//image_raw", 1);
    pub_expotime = comm_nh.advertise<geometry_msgs::PointStamped>("expotime_us", 1);
}


void CDataCapture::Run()
{
    //try to open the serial device.
    int r = cyusb_open();
    int usbNum = 0; //s_usbNum.toInt();

    //subscribe gyro's bias 
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("bias", 1, bias_fcn);


    if(r < 0)
    {
        std::cout<<"error opening lib";
        cyusb_close();
        return ;
    }
    else if(r == 0)
    {
        std::cout<<"no device";
        cyusb_close();
        return ;
    }
    h2 = cyusb_gethandle(usbNum);
    r = cyusb_kernel_driver_active(h2,0);
    if(r != 0)
    {
        std::cout<<"kernel driver active, exitting";
        cyusb_close();
        return ;
    }
    r = cyusb_claim_interface(h2, 0);
    if ( r != 0 ) {
        std::cout<<"Error in claiming interface";
        cyusb_close();
        return ;
    }
    int transferred = 0;
    unsigned char data[4];
    data[0] = (g_width&0xff<<8)>>8;
    data[1] = g_width&0xff;
    data[2] = (g_height&0xff<<8)>>8;
    data[3] = g_height&0xff;
    //cyusb_control_write(h2,0x40,0xD1,0,0,data,4,100);

    //create the thread to make sure the stamp time of IMU and image synchronously.
    //boost::thread thread_time(time_syn);
    //thread_time.join();

    while (f_capture && ros::ok()) {
        //if(cv::waitKey(1)=='e')break;
        //apture_time = ros::Time::now();
        r = cyusb_bulk_transfer(h2, 0x86, m_pReadBuff, ReadDataBytes, &transferred,1);
        if(transferred >= 0)
        {
            Input(m_pReadBuff,transferred);
            ros::spinOnce();
        }
        //usleep(1);
        //rate.sleep();
    }
    if(m_pOutData != NULL)
    {
        delete[] m_pOutData;
        //m_pOutData=NULL;
    }
    if(m_pInData != NULL)
    {
        delete[] m_pInData;
        //m_pInData=NULL;
    }
    cyusb_close();
    h2 = NULL;

}

int CDataCapture::Open(cyusb_handle *ch)
{
    f_capture = true;

    m_pInData = new byte[(ReadDataBytes*2)];
    m_pReadBuff = new byte[ReadDataBytes];
    memset(m_pInData,0,(ReadDataBytes+g_width+3)*sizeof(byte));
    return 0;
}

int CDataCapture::Close()
{
    if(f_capture == false)
        return -1;
    f_capture = false;
    usleep(9999);
    //delete &queue;
    //thread1->join();

    return 0;
}


int CDataCapture::Input( byte* lpData, unsigned int dwSize )
{
    int iBytes = 0;
    iBytes = dwSize+m_iCount;//m_iCount上一次拷贝剩余数据
    memcpy(m_pInData+m_iCount,lpData,dwSize);
    bool b_header=false, b_imu=false;

    for(int i=0; i<iBytes; ++i)
    {
        int imulen = 0;
        if ((i + g_width * g_height * 3) >= iBytes) //如果剩下的最后几个数据长度小于video_width*2+2行号个，不足以构成完整一行，拷贝到下一缓存
        {
            m_iCount = iBytes - i;
            memcpy(m_pInData, m_pInData + i, m_iCount);
            return 0;
        }
        if(m_pInData[i]==0x33 && m_pInData[i+1]==0xcc && m_pInData[i+14]==0x22 && m_pInData[i+15]==0xdd && b_header==false)
        {
	    if (first_time == true)
            {
                current_time = ros::Time::now();
                first_time = false;
            }
            else 
            {
                current_time += dt;
            }
        //    ROS_INFO(" ");  //debug

            dFrame.reset(new DFrameStruct);
            uint tempet = m_pInData[i+2]<<8;
            tempet += m_pInData[i+3];
            (*dFrame).expotime = tempet;
            (*dFrame).imgCnt = m_pInData[i+4];
            (*dFrame).IMUSamplesCnt = m_pInData[i+5];
            imulen = dFrame->IMUSamplesCnt*dFrame->IMUSampleLength;
            //(*dFrame).IMUPresent=m_pInData[i+5];
            unsigned int temp = m_pInData[i+6]<<8;
            temp = temp+m_pInData[i+7];
            (*dFrame).height = temp;
            temp = m_pInData[i+8]<<8; 
            temp += m_pInData[i+9];
            (*dFrame).width = temp; //(m_pInData[i+8]<<8+m_pInData[i+9]);

            (*dFrame).timestamp = m_pInData[i+10]<<8*3+m_pInData[i+11]<<8*2+m_pInData[i+12]<<8+m_pInData[i+13];
            dFrame->IMUData.reset(new IMUDataStruct[dFrame->IMUSamplesCnt]);
            dFrame->IMUDataBuffer.reset(new byte[imulen]);
            dFrame->leftData.reset(new byte[dFrame->height*dFrame->width]);
            dFrame->rightData.reset(new byte[dFrame->height*dFrame->width]);
            m_pOutDataLeft = dFrame->leftData.get();
            m_pOutDataRight = dFrame->rightData.get();
            m_pIMU = dFrame->IMUDataBuffer.get();
            i += 16;
            b_header = true;

        }


        if(m_pInData[i]==0x66 && m_pInData[i+1]==0xdd && m_pInData[i+imulen+2]==0x44 && m_pInData[i+imulen+3]==0xbb && b_header)
        {
            memcpy(m_pIMU, m_pInData+i+2, imulen);
            i += imulen+4;
            sensor_msgs::Imu imu_msg;
            //imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "/map";
            int k = 0;
            void *tempNum = new void*;
            IMUDataStruct *m_IMU = dFrame->IMUData.get();
            byte* m_IMUBuffer = dFrame->IMUDataBuffer.get();
		ROS_INFO("%u, %u", dFrame->IMUSamplesCnt, dFrame->IMUSampleLength);

            for(int j=0; j<dFrame->IMUSamplesCnt; j++)
            {

                //time Stamp
                k += bytesToIMU(m_IMUBuffer+k,4,tempNum);
                (m_IMU+j)->timeStamp=*(unsigned int*)tempNum;
                //accel
                k += fillAxisIMU(((IMUDataStruct*)(m_IMU+j))->accelData,m_IMUBuffer+k);
                //temperature
                k += bytesToIMU(m_IMUBuffer+k,2,tempNum);
                (m_IMU+j)->temperData = *(short*)tempNum;
                //gyro
                k += fillAxisIMU(((IMUDataStruct*)(m_IMU+j))->gyroData,m_IMUBuffer+k);

                //calculate IMU's timestamp
                /* double sec;
                sec = current_time.toSec() + 0.0025*(j);
                imu_time.sec = int(sec);
                imu_time.nsec = (sec - int(sec)) * 1e9; */
                imu_time = current_time + ros::Duration(0.004*j);
                imu_msg.header.stamp = imu_time;
 
                //get IMU's linear acceleration and angular velocity from serial
                imu_msg.linear_acceleration.x = ((IMUDataStruct*)(m_IMU+j))->accelData[0]; //*((short *)(&(m_pIMU[6])));// * STANDARD_G;
                imu_msg.linear_acceleration.y = ((IMUDataStruct*)(m_IMU+j))->accelData[1]; //*((short *)(&(m_pIMU[8])));// * STANDARD_G;
                imu_msg.linear_acceleration.z = ((IMUDataStruct*)(m_IMU+j))->accelData[2]; //*((short *)(&(m_pIMU[10])));// * STANDARD_G;
                imu_msg.angular_velocity.x = ((IMUDataStruct*)(m_IMU+j))->gyroData[0]; // *((short *)(&(m_pIMU[12])));// * dps_to_radia;
                imu_msg.angular_velocity.y = ((IMUDataStruct*)(m_IMU+j))->gyroData[1]; // *((short *)(&(m_pIMU[14])));// * dps_to_radia;
                imu_msg.angular_velocity.z = ((IMUDataStruct*)(m_IMU+j))->gyroData[2]; // *((short *)(&(m_pIMU[16])));// * dps_to_radia;


                //converter IMU raw data to metric data

                // since we are using 4g range and the register is 16 bits
                // -4g maps to a raw value of -32768
                // +4g maps to a raw value of 32767
/*
                imu_msg.linear_acceleration.x *= 8.0/32768.0;
                imu_msg.linear_acceleration.y *= 8.0/32768.0;
                imu_msg.linear_acceleration.z *= 8.0/32768.0;
*/
                //change  Accelerometer units to metres/sec^2
                imu_msg.linear_acceleration.x *= accel_factor;
                imu_msg.linear_acceleration.y *= accel_factor;
                imu_msg.linear_acceleration.z *= accel_factor;

                // since we are using 1000 degrees/seconds(dps) range and the register is 16 bits
                // -1000 maps to a raw value of -32768
                // +1000 maps to a raw value of 32767
/*
                imu_msg.angular_velocity.x *= 3.2768;
                imu_msg.angular_velocity.y *= 3.2768;
                imu_msg.angular_velocity.z *= 3.2768;
*/
                //change Gyroscope units to radians/second
                //Convert gyroscope degrees/sec to radians/sec
                imu_msg.angular_velocity.x *= angular_factor;
                imu_msg.angular_velocity.y *= angular_factor;
                imu_msg.angular_velocity.z *= angular_factor;

                //minus bias
                imu_msg.angular_velocity.x -= v.x;
                imu_msg.angular_velocity.y -= v.y;
                imu_msg.angular_velocity.z -= v.z;
                //  ROS_INFO("The imu_msg.angular_velocity.x = %f", imu_msg.angular_velocity.x);
                //  ROS_INFO("The imu_msg.angular_velocity.y = %f", imu_msg.angular_velocity.y);
                //  ROS_INFO("The imu_msg.angular_velocity.z = %f", imu_msg.angular_velocity.z);



                IMU_pubGs.publish(imu_msg);
                //   rate.sleep();

            }
            //FillImuMessage(imu_msg, data, binary_output_);
            //imu_msg.orientation.x=angX;
            //ROS_INFO("TIME: %u", *(( unsigned int) *(&m_pIMU[0])));
            //IMUDataStruct* temp;
            //temp = m_pIMU;


            b_imu = true;
        }
        else if(b_header==true && m_bFindDbFive==false)
        {
            b_header = false;
            b_imu = false;
        }

        if (b_header && b_imu)
        {

            unsigned int datalen = dFrame->width * dFrame->height;
            expotime_.point.x = static_cast<float>(dFrame->expotime) * 27.185;
            memcpy(m_pOutDataLeft, m_pInData + i, datalen);
            memcpy(m_pOutDataRight, m_pInData + i + datalen, datalen);
            //m_pDataProcess->Input(dFrame, (*dFrame).width * (*dFrame).height);




            //capture_time.nsec -= 5247912;
            // thread_time.join();
            ImagePtr image(new Image);
            ImagePtr image2(new Image);
            //usleep(15000);
            image->height = dFrame->height;
            image->width = dFrame->width;
            image->step = dFrame->width;
            image->header.stamp = current_time;
            //image->header.seq = pair_id;
            image->data.resize(image->step * image->height);
            image->encoding = image_encodings::MONO8;

            image2->height = dFrame->height;
            image2->width = dFrame->width;
            image2->step = dFrame->width;
            image2->header.stamp = current_time;
            //image2->header.seq = pair_id;
            image2->data.resize(image->step * image->height);
            image2->encoding = image_encodings::MONO8;

            expotime_.header.stamp = current_time;

            image->header.frame_id = "cameraRight";
            memcpy(&image->data[0], m_pInData + i, image->data.size());

            image2->header.frame_id = "cameraLeft";
            memcpy(&image2->data[0], m_pInData + i + datalen, image2->data.size());


            pub_right.publish(image);
            pub_left.publish(image2);
            pub_expotime.publish(expotime_);

            b_imu = false;
            b_header = false;
            m_bFindDbFive = false;
            i = i + datalen*2 - 1;
        }

    } //for
    return 0;
}


