#pragma once
#include "Detectors.h"

#define GREEN_LOWER 50, 150, 160
#define GREEN_UPPER 80, 255, 255

#define RED_LOWER 0, 180, 145
#define RED_UPPER 50, 255, 255

#define BOX_LOWER 60, 250, 90
#define BOX_UPPER 255, 255, 110

#define BOX_RED_LOWER 0, 200, 115
#define BOX_RED_UPPER 255, 255, 140

#define GOAL_LOWER 155, 160, 120
#define GOAL_UPPER 255, 255, 200

#define YELLOW_LOWER1 0, 150, 110
#define YELLOW_UPPER1 80, 255, 200

#define YELLOW_LOWER2 0, 150, 210
#define YELLOW_UPPER2 85, 255, 255

void Binarization_(int n1, int n2, int n3, int m1, int m2, int m3)
{
    auto img = mur.getCameraOneFrame();
    auto fin = img.clone();
    cv::cvtColor(img, fin, CV_BGR2HSV);
    cv::Scalar lower(n1, n2, n3);
    cv::Scalar upper(m1, m2, m3);
    cv::inRange(fin, lower, upper, fin);
    cv::imshow("Image", fin);
    cv::waitKey(10);
}

Object DetectLine(cv::Mat &image)
{
    Object obj;
    std::vector<cv::vector<cv::Point>> conturs;
    cv::Mat fin = image.clone();
    cv::RotatedRect r;
    cv::RotatedRect result;
    cv::cvtColor(image, image, CV_BGR2HSV);
    cv::Scalar lower(0, 100, 60); //0 100 0
    cv::Scalar upper(20, 255, 255); // 255 255 255
    cv::inRange(image, lower, upper, image);
    cv::findContours(image, conturs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    auto max_area = 0;
    for(size_t i = 0; i < conturs.size(); i++){
        std::vector<cv::Point> hull;
        cv::convexHull(conturs[i], hull, true);
        cv::approxPolyDP(hull, hull, 24, true);
        std::vector<std::vector<cv::Point>> vec;
        vec.push_back(hull);
        if (hull.size() < 3) continue;  
        if(conturs.at(i).size() < 5) continue;
        r = cv::fitEllipse(conturs.at(i));
        obj.x = r.center.x;
        obj.y = r.center.y;
        obj.angle = r.angle;
        auto area = cv::contourArea(conturs.at(i));     
        if(max_area < area){
          result = r;
          max_area = area;
        }          
    } 
     return obj;
}

int detected = 0;


int DetectGate(cv::Mat &image)
{
    Object obj;
    std::vector<cv::vector<cv::Point>> conturs;
    //cv::Mat fin = image.clone();
    image = image(cv::Rect(0, 0, 320, 100));
    cv::RotatedRect r;
    cv::RotatedRect result;
    cv::cvtColor(image, image, CV_BGR2HSV);
    cv::Scalar lower(GREEN_LOWER); 
    cv::Scalar upper(GREEN_UPPER); 
    cv::inRange(image, lower, upper, image);
    //cv::Mat cropedImage = image(cv::Rect(0, 0, 300, 100));
    cv::imshow("bin", image);
    cv::waitKey(10); 
    cv::findContours(image, conturs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    auto max_area = 0;
    for(size_t i = 0; i < conturs.size(); i++){
        std::vector<cv::Point> hull;
        cv::convexHull(conturs[i], hull, true);
        cv::approxPolyDP(hull, hull, 24, true);
        std::vector<std::vector<cv::Point>> vec;
        vec.push_back(hull);
        if (hull.size() < 3) continue;  
        if(conturs.at(i).size() < 5) continue;
        r = cv::fitEllipse(conturs.at(i));
        obj.x = r.center.x;
        obj.y = r.center.y;
        obj.angle = r.angle;
        auto area = cv::contourArea(conturs.at(i));     
        if(max_area < area){
          result = r;
          max_area = area;
        }          
    }
    std::cout << "gate.x = " << obj.x << std::endl;
    if(detected > 3) return obj.x > 200 ? 1 : 0; // 1 - справа, 0 - слева
    else 
    {
        detected ++;
        return -1;
    }
}

float Yaw(float angle)
{
    float angleNEW = angle;
    if(angleNEW > 90) angleNEW = (-1)*(180 - angleNEW);
    return angleNEW;
}

float detectAngle()
{
    mur.addDetectorToList(Object::RECTANGLE, 0);
    cv::Mat image = mur.getCameraOneFrame();
    Object obj = DetectLine(image);
    float yaw = Yaw(obj.angle);
    return yaw;
}

Object GreenBox(cv::Mat &image)
{
    Object obj;
    std::vector<cv::vector<cv::Point>> conturs;
    cv::Mat fin = image.clone();
    cv::RotatedRect r;
    cv::RotatedRect result;
    cv::cvtColor(image, image, CV_BGR2HSV);
    cv::Scalar lower(BOX_LOWER);
    cv::Scalar upper(BOX_UPPER);
    cv::inRange(image, lower, upper, image);
    //cv::imshow("1", image);
    //cv::waitKey(10);
    cv::findContours(image, conturs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    auto max_area = 0;
    for(size_t i = 0; i < conturs.size(); i++){
        std::vector<cv::Point> hull;
        cv::convexHull(conturs[i], hull, true);
        cv::approxPolyDP(hull, hull, 24, true);
        std::vector<std::vector<cv::Point>> vec;
        vec.push_back(hull);
        if (hull.size() < 3) continue;  
        if(conturs.at(i).size() < 5) continue;
        r = cv::fitEllipse(conturs.at(i));
        obj.x = r.center.x;
        obj.y = r.center.y;
        obj.angle = r.angle;
        auto area = cv::contourArea(conturs.at(i));     
        if(max_area < area){
          result = r;
          max_area = area;
        }          
    } 
     return obj;
}

int DetectGoal()
{
    cv::Mat image = mur.getCameraTwoFrame();
    Object obj;
    std::vector<cv::vector<cv::Point>> conturs;
    //cv::Mat fin = image.clone();
    cv::RotatedRect r;
    cv::RotatedRect result;
    cv::cvtColor(image, image, CV_BGR2HSV);
    cv::Scalar lower(GOAL_LOWER); 
    cv::Scalar upper(GOAL_UPPER); 
    cv::inRange(image, lower, upper, image);
    //cv::Mat cropedImage = image(cv::Rect(0, 0, 300, 100));
    //cv::imshow("bin", image);
    //cv::waitKey(10); 
    cv::findContours(image, conturs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    auto max_area = 0;
    for(size_t i = 0; i < conturs.size(); i++){
        std::vector<cv::Point> hull;
        cv::convexHull(conturs[i], hull, true);
        cv::approxPolyDP(hull, hull, 24, true);
        std::vector<std::vector<cv::Point>> vec;
        vec.push_back(hull);
        if (hull.size() < 3) continue;  
        if(conturs.at(i).size() < 5) continue;
        r = cv::fitEllipse(conturs.at(i));
        obj.x = r.center.x;
        obj.y = r.center.y;
        obj.angle = r.angle;
        auto area = cv::contourArea(conturs.at(i));     
        if(max_area < area){
          result = r;
          max_area = area;
        }          
    }
    //std::cout << obj.x << std::endl;
    if(obj.x > 160) return 1;
    else return -1;
}


Object DetectYellow1(cv::Mat &image)
{
    Object obj;
    std::vector<cv::vector<cv::Point>> conturs;
    //cv::Mat fin = image.clone();
    cv::RotatedRect r;
    cv::RotatedRect result;
    cv::cvtColor(image, image, CV_BGR2HSV);
    cv::Scalar lower(YELLOW_LOWER1); 
    cv::Scalar upper(YELLOW_UPPER1); 
    cv::inRange(image, lower, upper, image);
    //cv::Mat cropedImage = image(cv::Rect(0, 0, 300, 100));
    //cv::imshow("bin", image);
    //cv::waitKey(10); 
    cv::findContours(image, conturs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    auto max_area = 0;
    for(size_t i = 0; i < conturs.size(); i++){
        std::vector<cv::Point> hull;
        cv::convexHull(conturs[i], hull, true);
        cv::approxPolyDP(hull, hull, 24, true);
        std::vector<std::vector<cv::Point>> vec;
        vec.push_back(hull);
        if (hull.size() < 3) continue;  
        if(conturs.at(i).size() < 5) continue;
        r = cv::fitEllipse(conturs.at(i));
        obj.x = r.center.x;
        obj.y = r.center.y;
        obj.angle = r.angle;
        auto area = cv::contourArea(conturs.at(i));     
        if(max_area < area){
          result = r;
          max_area = area;
        }          
    }
    //std::cout << obj.x << std::endl;
    return obj;
}


Object DetectYellow2(cv::Mat &image)
{
    Object obj;
    std::vector<cv::vector<cv::Point>> conturs;
    //cv::Mat fin = image.clone();
    cv::RotatedRect r;
    cv::RotatedRect result;
    cv::cvtColor(image, image, CV_BGR2HSV);
    cv::Scalar lower(YELLOW_LOWER2); 
    cv::Scalar upper(YELLOW_UPPER2); 
    cv::inRange(image, lower, upper, image);
    //cv::Mat cropedImage = image(cv::Rect(0, 0, 300, 100));
    //cv::imshow("bin", image);
    //cv::waitKey(10); 
    cv::findContours(image, conturs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    auto max_area = 0;
    for(size_t i = 0; i < conturs.size(); i++){
        std::vector<cv::Point> hull;
        cv::convexHull(conturs[i], hull, true);
        cv::approxPolyDP(hull, hull, 24, true);
        std::vector<std::vector<cv::Point>> vec;
        vec.push_back(hull);
        if (hull.size() < 3) continue;  
        if(conturs.at(i).size() < 5) continue;
        r = cv::fitEllipse(conturs.at(i));
        obj.x = r.center.x;
        obj.y = r.center.y;
        obj.angle = r.angle;
        auto area = cv::contourArea(conturs.at(i));     
        if(max_area < area){
          result = r;
          max_area = area;
        }          
    }
    //std::cout << obj.x << std::endl;
    return obj;
}
