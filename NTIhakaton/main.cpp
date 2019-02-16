#include <murAPI.hpp>
//#include "Binarization.h"
#include "Regulators.h"
#include "Detectors.h"
 
using namespace std;
 
int main()
{
    //Binarization();
    //return 0;
    int napr = 0;
    float yawToSet = 0;
    Object greenBall;
    yawPRegTime(yawToSet, 70, 20, 4000);
    cv::Mat image = mur.getCameraTwoFrame();
    greenBall = detectGreenBall(image);
    //cout << greenBall.x << endl;
    yawPRegTime(yawToSet, 70, 30, 3000);
    yawPRegToLine(yawToSet, 70, 30);
    yawPRegTime(yawToSet, 70, 30, 1000);
    yawToSet = detectAngle();
    if(greenBall.x > 230) napr = 1;
    else napr = -1;
    while(true)
    {
        cv::Mat img = mur.getCameraTwoFrame();
        Object obj = detectGreenBall(img);
        if(napr == 1) mur.setPorts(0, 0, 15, 20);
        if(obj.x > 150 && obj.x < 170) break;
    }
    
    yawPRegTime(0, 90, 30, 3000);
    yawPRegTime(0, 90, -30, 3000);
    yawPRegTime(35, 110, 0, 7000);
    //regYellow(35, 1);
    mur.shoot();
    
    yawPRegTime(0, 90, 0, 2000);
    
    Timer t;
    t.start();
    while(t.elapsed() < 14000)
    {
        mur.setPorts(0, 0, 7, -20);
    }
    yawPRegTime(yawToSet, 70, 30, 3000);
    yawPRegTime(0, 70, -30, 3000);

    t.start();
    while(t.elapsed() < 500)
    {
        mur.setPorts(0, 0, 7, 20);
    }
    yawPRegTime(yawToSet, 40, 30, 3000);
    yawPRegToLine(yawToSet, 50, 30);
    yawPRegTime(yawToSet, 70, 30, 100);
    yawToSet += detectAngle();
    yawPRegTime(yawToSet, 70, 30, 5000);
    mur.drop();
    yawToSet = 0;
    yawPRegTime(yawToSet, 70, 30, 500);
    yawPRegToLine(yawToSet, 50, 30);
    yawPRegTime(yawToSet, 70, 30, 100);
    yawToSet += detectAngle();
    yawPRegTime(yawToSet - 3, 70, 30, 3000);
    yawPRegTime(yawToSet, 1000, 0, 10000);
    /*yawPRegToLine(yawToSet, 70, 30);
    yawPRegTime(yawToSet, 70, 30, 1000);
    yawToSet += detectAngle();
    yawPRegTime(yawToSet, 70, 30, 4000);*/
    return 0;
}