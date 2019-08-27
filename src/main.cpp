#include <stdio.h>
#include "act_d435.h"
#include "robot_locator.h"
#include <iostream>
#include "own_serial.h"
using namespace std;
#define IFSERIAL   
int main(int argc, char* argv[])
{
#ifdef IFSERIAL
    int fd2 = 0;
    own_serial ownSerial("/dev/ttyUSB0");
    ownSerial.init(fd2);
    //fd = ownSerial.isopen();
    if(fd2 ==-1)
    {
        cout<<"can't open serial\n";
        exit(0);
    }
#endif
    ActD435 D435;
    RobotLocator Locator;
    D435.init();
    Locator.init(D435);
    Locator.status = DISTANCE_CALCULATE;
    cv::TickMeter tk;
    tk.start();
    while(true)
    {     
        
        Locator.updateImage();   
        tk.start(); 
        switch(Locator.status)
        {
            case GESTURE_RECOGNITION:
            {
                
            }
            break;

            case DISTANCE_CALCULATE:
            {
                Locator.findBoundary();
            }
            break;
        }
#ifdef IFSERIAL    
        if(Locator.status == DISTANCE_CALCULATE)
            ownSerial.writeData(Locator.leftDistance,Locator.rightDistance,Locator.cornerAngle,Locator.lineStatus);
#endif
        tk.stop();
        cout<<tk.getTimeSec()<<endl;
        tk.reset();
    }

	return 0;
}
