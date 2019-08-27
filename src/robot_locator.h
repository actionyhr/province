#define _CRT_SECURE_NO_WARNINGS

#ifndef ROBOT_LOCATOR_H_
#define ROBOT_LOCATOR_H_

//left is zero, right is one 

#define STD_ROI {-0.6f, 0.6f, 0.0f, 2.5f}
#include <cmath>
#include "act_d435.h"

#define DISTANCE(X1,Y1,X2,Y2) (sqrt(pow(X1-X2,2)+pow(Y1-Y2,2)))
#define DISTANCELINE(X,Y,A,B) (fabs(A*X+B-Y)/sqrt((pow(A,2)+1)))

using namespace cv;
enum BallColor{black, white, pink};
typedef struct
{	
	//location
	int x;
	int y;
	//ballNum
	int num;
}BallCluster;

typedef struct
{
	float k;
	float b;
}Line;
//-- Algorithm implementation for robot locating
class RobotLocator
{
public:
	RobotLocator();
	RobotLocator(const RobotLocator&) = delete;
	RobotLocator& operator=(const RobotLocator&) = delete;
	~RobotLocator();

	void init(ActD435& d435);
	void updateImage(void);
	void showImage(void);
	void sign_color(cv::Mat &color);
	void imagePrecess(void);
	void findBoundary(void);
	void getBoundaryPoint(cv::Mat color);
	void findCorner(std::vector<Line> &lines, cv::Point2f *corner);
	void getPointFromPixel(float colorPixel[2],float point[3]);
	void findLine();
	void threeDDoundary(void);
	void highOrderFilter(std::vector<cv::Point2f> &cloud, int order);
	void findMaxAndIndex(int x[],int length,int& max,int& index);
public:
	float leftDistance = 0.0;
	float rightDistance = 0.0;
	float cornerAngle = 0.0;
	float recoStatus = 0;
	vector<Line> boundaryLines;
	unsigned int status;
	unsigned int lineStatus;
	uint16_t* depthData;

	
private:
	ActD435*        thisD435;
	std::vector<BallCluster> ballClusterVec;
	uint16_t* m_data;
	//camera parameters
	rs2_intrinsics color_intrin;
	rs2_intrinsics depth_intrin;
	rs2_extrinsics depth2color_extrin;
	rs2_extrinsics color2depth_extrin;
	
	//PCL modules
	cv::Mat depthImage;
	cv::Mat afterLine;
	//opencv modules
	cv::Mat srcImage;
	cv::Mat dstImage;
	cv::Mat bgImage;
	cv::Mat allBallImage;

	cv::Mat channelB;
	cv::Mat channelG;
	cv::Mat channelR;

	cv::Mat channelH;
	cv::Mat channelS;
	
	cv::Mat channelL;
	cv::Mat channelA;
	
	cv::Mat LABImage;
	cv::Mat HSVImage;

    cv::Mat SigedColorImage;

	std::vector<cv::Mat> channels;
    vector<vector<cv::Point2d>>firstVector;
	vector<vector<cv::Point2d>>boundaryVector;
	std::vector<cv::Point2d> point;
	std::vector<cv::Point2d> *choicePoint;
};

#endif
