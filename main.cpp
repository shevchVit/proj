#include <murAPI.hpp>
#include "Detectors.h"
#include "Regulators.h"
//#include "Binarization.h"

using namespace std;

float YawToSet = 0;
//green
// 50 150 160
// 80 200 255
//red
// 0 180 145
// 50 255 255

// goal
// 155 160 120
// 255 255 200

// yellow
// 0 150 110
// 80 255 200

// yellow
// 0 150 210
// 85 255 255


int main()
{
    
    //Binarization();
    //return 0;
    int goal = 0;
    int gate = -1;
    yawPRegTime(0, 80, 0, 1000);
    while(gate == -1)
    {
        yawPReg(0, 80, 0);
        cv::Mat image = mur.getCameraTwoFrame(); 
        gate = DetectGate(image);
        if(gate == 0) 
        {
            cout << "left" << endl;
        }
        if(gate == 1) 
        {
            cout << "right" << endl;
        }
    }
    if(gate == 0) 
    {
        YawToSet = 330;
        yawPRegTime(YawToSet, 90, 15, 3500);
        YawToSet = 0;
        yawPRegToLine(YawToSet, 90, 15);
        
        
        goal = DetectGoal();
        cout << "goal = " << goal << endl;
        
        yawPRegTime(YawToSet, 90, 15, 3000);
 
        YawToSet += detectAngle();
        //cout << "New YawToSet = " << YawToSet << endl;
        yawPRegTime(YawToSet, 90, 15, 2000);
        yawPRegToLine(YawToSet, 90, 15);
        yawPRegTime(YawToSet, 90, 15, 500);
        YawToSet += detectAngle();
        //cout << "New YawToSet = " << YawToSet << endl;
        yawPRegTime(YawToSet, 80, 15, 3000);
        yawPRegToRectangle(YawToSet, 80, 15);

        yawPRegToRectangle(YawToSet, 80, 15);
        yawPRegLag();
        mur.drop();
        goBack(0);
        
        if(goal == -1) 
        {
            YawToSet = -30;
        }
        if(goal == 1) 
        {
            YawToSet = 30;
        }
        
        yawPRegTime(YawToSet, 90, 15, 4000);
        yawPRegToLine(YawToSet, 90, 15);
        yawPRegTime(YawToSet, 90, 15, 1000);
        YawToSet += detectAngle();
                
        regYellow(YawToSet, goal);
        backToLine(YawToSet);
        yawPRegTime(YawToSet, 90, 15, 5000);
        yawPRegToLine(YawToSet, 90, 15);
        yawPRegTime(YawToSet, 90, 15, 500);
        YawToSet += detectAngle();
        YawToSet += 180;
        yawPRegTime(YawToSet, 60, 15, 10000);
        yawPRegToRectangle(YawToSet, 60, 15);
        yawPRegTime(YawToSet, 0, -10, 1000);
    }
    if(gate == 1) 
    {
        YawToSet = 30;
        yawPRegTime(YawToSet, 90, 15, 7000);
        YawToSet = 0;
        yawPRegToLine(YawToSet, 90, 15);
        yawPRegTime(YawToSet, 90, 15, 1000);
        YawToSet += detectAngle();
        //cout << "New YawToSet = " << YawToSet << endl;
        yawPRegTime(YawToSet, 90, 15, 5000);
        yawPRegToLine(YawToSet, 90, 15);
        yawPRegTime(YawToSet, 90, 15, 1500);
        YawToSet += detectAngle();
        //cout << "New YawToSet = " << YawToSet << endl;
        yawPRegTime(YawToSet, 80, 15, 3000);
     
        goal = DetectGoal();
        cout << "goal = " << goal << endl;
        yawPRegToRectangle(YawToSet, 80, 15);
        yawPRegLag();
        mur.drop();
        goBack(0);
        
        if(goal == -1) YawToSet = -30;
        if(goal == 1) YawToSet = 30;
        
        yawPRegTime(YawToSet, 90, 15, 4000);
        yawPRegToLine(YawToSet, 90, 15);
        yawPRegTime(YawToSet, 90, 15, 1000);
        YawToSet += detectAngle();
                
        regYellow(YawToSet, goal);
        backToLine(YawToSet);
        yawPRegTime(YawToSet, 90, 15, 5000);
        yawPRegToLine(YawToSet, 90, 15);
        yawPRegTime(YawToSet, 90, 15, 500);
        YawToSet += detectAngle();
        YawToSet += 180;
        yawPRegTime(YawToSet, 60, 15, 10000);
        yawPRegToRectangle(YawToSet, 60, 15);
        yawPRegTime(YawToSet, 0, -10, 1000);
    }
}