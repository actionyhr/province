#include "act_d435.h"
#include "robot_locator.h"
#include <iostream>
#include "gesture_detect.h"
#include "own_serial.h"
using namespace std;
#define IFSERIAL   
extern int foundFlag;
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
    D435.init2();
    Locator.init(D435);
    Locator.status = GESTURE_RECOGNITION;
    GestureDetector hand;
    hand.init();
    int count[7]={ 0 };
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
                cv::Mat temp = Locator.preprocess();
                int getMsg=hand.detect(temp);
                char msg;
                switch (getMsg)
                {
                    case 0: count[0]++; break;
                    case 1: count[1]++; break;
                    case 2: count[2]++; break;
                    case 3: count[3]++; break;
                    case 4: count[4]++; break;
                    case 5: count[5]++; break;
                    case 6: count[6]++; break;
                }
                for(int i=0;i<7;i++)
                {
                    if(count[i]>30)
                    {
                        msg=i+1;
                        Locator.status=DISTANCE_CALCULATE;
                        ownSerial.writeData(msg);
                        break;
                    }
                }
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
