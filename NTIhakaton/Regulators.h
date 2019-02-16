#pragma once
#include "Detectors.h"

int time_for_back = 0;
int napr_for_back = 0;

float kp = 1.3;
float k_depth = 6;

using namespace std;

float Yaw360(float yaw)
{
    if(yaw < 0) yaw = 360 + yaw;
    return yaw;
}

float Yaw180 (float yaw)
{
     if ( yaw < 180) yaw = yaw;   
     else yaw = (360-yaw)*(-1);
     return yaw;
} 

void yawPReg(float yawToSet, float depthToSet, int power)
{
    float u = mur.getYaw() - yawToSet;    
    if (u < 0) u += 360;
    if(u > 180) u -= 360;
    u *=  kp;
    float depth = mur.getInputAOne() - depthToSet;
    depth *= k_depth;
    mur.setPorts(-power + u, -power - u, -depth, 0);
}

void yawPRegToLine(float& yawToSet, float depthToSet, int power) 
{
    mur.addDetectorToList(Object::RECTANGLE, 0);
    while(true)
    {
        yawPReg(yawToSet, depthToSet, power);
        cv::Mat image = mur.getCameraOneFrame();
        if (image.empty()) continue;
        auto obj = DetectLine(image);
        if (obj.angle > 0 || obj.angle < -1) 
        {
            std::cout << "Line Detected" << std::endl;
            //sleepFor(4000);
            //yawToSet += detectAngle();
            /*std::cout << "before = " << yawToSet << std::endl;
            yawToSet += obj.angle;
            std::cout << "after = " << yawToSet << std::endl;*/
            break;
        }
    }
}

void yawPRegToRectangle(float yawToSet, float depthToSet, int power)
{
    bool end = true;
    mur.addDetectorToList(Object::RECTANGLE, 0);
    while(end)
    {
        yawPReg(yawToSet, depthToSet, power);
        cv::Mat image = mur.getCameraOneFrame();
        for (const auto &obj : mur.getDetectedObjectsList(0)) 
        {
            if (obj.type == Object::RECTANGLE)
            {
                if (obj.r > 10) 
                {
                    std::cout<<"Rectangle Detected"<<std::endl;
                    end = false;
                }
            }
        }
    }
}

void yawPRegTime(float yaw, float depth, int power, int time)
{
    Timer t;
    t.start();
    while(t.elapsed() < time) yawPReg(yaw, depth, power);
}

void goBack(float yawToSet)
{
    Timer t;
    t.start();
    int powerD = 0;
    if(napr_for_back == 1) powerD = 20;
    if(napr_for_back == -1) powerD = -20;
    while(t.elapsed() < time_for_back)
    {
        float u = mur.getYaw() - yawToSet;    
        if (u < 0) u += 360;
        if(u > 180) u -= 360;
        u *=  kp;
        float depth = mur.getInputAOne() - 80;
        depth *= k_depth;
        mur.setPorts(u, -u, -depth, powerD);
    }
}

void yawPRegLag()
{
    Timer t;
    t.start();
    int powerD = 0;
    while(true)
    {
        cv::Mat image = mur.getCameraOneFrame();
        Object obj = GreenBox(image);
        //cout << obj.x << endl;
        if(obj.x > 165) 
        {
            powerD = 20;
            napr_for_back = -1;
        }
        if(obj.x < 155) 
        {
            powerD = -20;
            napr_for_back = 1;
        }
        if((obj.x >= 155 && obj.x <= 165) || (obj.x >= 105 && obj.x <= 115))
        {
            break;
        }
        float u = mur.getYaw() - 0;    
        if (u < 0) u += 360;
        if(u > 180) u -= 360;
        u *=  kp;
        float depth = mur.getInputAOne() - 80;
        depth *= k_depth;
        mur.setPorts(u, -u, -depth, powerD);
    }
    time_for_back = t.elapsed();
}

void yawPRegLagBall(int napr)
{
   int powerD = 20 * napr;
   while(true)
    {
        cv::Mat image = mur.getCameraTwoFrame();
        Object obj = detectGreenBall(image);
        if(obj.x > 165) 
        {
            powerD = -20;
        }
        if(obj.x < 155) 
        {
            powerD = 20;
        }
        if((obj.x >= 155 && obj.x <= 165) || (obj.x >= 105 && obj.x <= 115))
        {
            break;
        }
        float u = mur.getYaw() - 0;    
        if (u < 0) u += 360;
        if(u > 180) u -= 360;
        u *=  kp;
        float depth = mur.getInputAOne() - 80;
        depth *= k_depth;
        mur.setPorts(u, -u, -depth, powerD);
    }
}

void backToLine(float yawToSet)
{
    int powerD = 0;
    if(napr_for_back == 1) powerD = 20;
    if(napr_for_back == -1) powerD = -20;
    while(true)
    {
        cv::Mat image = mur.getCameraOneFrame();
        Object obj = DetectLine(image);
        if(obj.x > 150 && obj.x < 170) break;
        float u = mur.getYaw() - yawToSet;    
        if (u < 0) u += 360;
        if(u > 180) u -= 360;
        u *=  kp;
        float depth = mur.getInputAOne() - 90;
        depth *= k_depth;
        mur.setPorts(u, -u, -depth, powerD);
    }
}

void regYellow(float yawToSet, int napr)
{
    int powerD = 0;
    int powerC = 0;
    bool flag = true;
    while(true)
    {
        cv::Mat image = mur.getCameraTwoFrame();
        Object obj;
        if(napr == 1)
        {
            obj = DetectYellow2(image);
        }
        if(napr == -1)
        {
            obj = DetectYellow1(image);
        }
        cv::circle(image, cv::Point2i(obj.x, obj.y), 2, cv::Scalar(255, 0, 100), 1);
        cv::imshow("1", image);
        cv::waitKey(10);
        cout << "obj.x = " << obj.x << endl;
        cout << "obj.y = " << obj.y << endl;
        float u = mur.getYaw() - yawToSet;    
        if (u < 0) u += 360;
        if(u > 180) u -= 360;
        u *=  kp;
        if(flag)
        {
            if(obj.x > 160) napr_for_back = -1;
            else napr_for_back = 1;
            flag = false;
        }
        if(obj.x > 160) powerD = 20;
        if(obj.x < 160) powerD = -20;
        if(obj.y > 145) powerC = 20;
        if(obj.y < 145) powerC = -20;
        if(obj.x > 155 && obj.x < 165 && obj.y > 140 && obj.y < 150)
        {
            mur.shoot();
            break;
        }
        mur.setPorts( -1 + u, - 1 - u, powerC, powerD);
    }
}