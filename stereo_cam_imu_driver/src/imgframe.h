#ifndef IMGFRAME_H
#define IMGFRAME_H
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <memory>
#include <list>
#include <vector>
using namespace std;
typedef unsigned char byte;
struct IMUDataStruct
{
    unsigned int timeStamp;
    /*float temperData;
    float accelData[3];
    float gyroData[3];*/
    short temperData;
    short accelData[3];
     short gyroData[3];
};
struct DFrameStruct
{
    unsigned int width;
    unsigned int height;
    byte IMUPresent;
    byte imgCnt;
    unsigned int expotime;
    unsigned int IMUSamplesCnt;
    unsigned int IMUSampleLength;//18
    unsigned int timestamp;
    //byte* leftData;
    //byte* rightData;
    //IMUDataStruct *IMUData;
    //byte* IMUDataBuffer;
    std::unique_ptr<byte> leftData;
    std::unique_ptr<byte>rightData;
    std::unique_ptr<IMUDataStruct>IMUData;
    std::unique_ptr<byte>IMUDataBuffer;
    DFrameStruct():IMUSampleLength(18)
    {
    }

};

class imgFrame
{
private:
    //byte * imgBuf;
    int m_camNum;
    vector<unique_ptr<byte> > list_imgBuf;
    int IMUDatawidth=2;
public:
    int m_width;
    int m_height;
    long timestamp;
    byte IMUPresent;
    unsigned int IMUSamplesCnt;
    unsigned int IMUSampleLength;//18
    byte imTimeStamp[4];
    IMUDataStruct IMUData;
    //byte* IMUDataBuff;
    unique_ptr<byte> IMUDataBuff;
    unique_ptr<byte>imgBuf;
    unique_ptr<byte>imgBuf1;
    unique_ptr<byte>imgBuf2;
    unique_ptr<byte>imgBuf3;
    imgFrame(int width,int height,int camNum):m_width(width),m_height(height),m_camNum(camNum)
    {
        IMUSampleLength=18;
        if(camNum>0)
        {
            imgBuf1.reset(new byte[height*width]);
        }
        if(camNum>1)
        {
            imgBuf2.reset(new byte[height*width]);
        }
        if(camNum>2)
        {
            imgBuf3.reset(new byte[height*width]);
        }
        /*
        for (int i=0;i<m_camNum;i++)
        {
            imgBuf.reset(new byte[height*width]);
            list_imgBuf.push_back(move(imgBuf));
            memset(list_imgBuf[i].get(),0,height*width);
        }
        */
    }
    ~imgFrame(void)
    {
        //if (imgBuf!=NULL)
       // delete imgBuf;
    }
    unique_ptr<byte> getimgbufp(int idx)
    {
        auto rst=move(list_imgBuf[idx]);
        /*
    switch(idx)
    {
    case 0:
        return imgBuf1;
        break;
    case 1:
        return imgBuf2;
        break;
    case 3:
        return imgBuf3;
        break;
    default:
        return imgBuf1;
        break;

    }
    */
        return rst;
    }
    int setIMUSampleCnt(int num)
    {
        IMUSamplesCnt=num;
        IMUDataBuff.reset(new byte[IMUSamplesCnt*IMUSampleLength]);
        return IMUSamplesCnt*IMUSampleLength;
    }
};
#endif // IMGFRAME_H
