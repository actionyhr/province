#define _CRT_SECURE_NO_WARNINGS

#ifndef ACT_D435_H_
#define ACT_D435_H_

#include <iostream>
//#include <fstream> 
#include <fstream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rs_advanced_mode.h>
#define IFCAMERA
#define IMAGE_WIDTH 	640
#define IMAGE_HEIGHT 	480
#define DEPTH_ACCURACY  1.0 / 1.013

#define GESTURE_RECOGNITION 	0
#define DISTANCE_CALCULATE 		1


//-- ROI of an object
typedef struct
{
	double xMin;
	double xMax;

	double yMin;
	double yMax;

	double zMin;
	double zMax;

} ObjectROI;

using namespace std;
class ActD435
{
public:
	ActD435();
	ActD435(const ActD435&) = delete;
	ActD435 operator=(const ActD435&) = delete;
	~ActD435();

	void init(void);
	void update(void);
	inline cv::Mat getSrcImage(void){return srcImage;}
	inline cv::Mat getDepthImage(void){return depthImage;}
	inline uint16_t* getData(void){return data;}
	void getCameraParam(rs2_intrinsics& _color_intrin, rs2_intrinsics& _depth_intrin, rs2_extrinsics& _depth2color_extrin, rs2_extrinsics& _color2depth_extrin, uint16_t* _data);
public:
	uint16_t* data;
private:

private:
	rs2::context ctx;
	size_t device_count;	
	rs2_intrinsics color_intrin;
	rs2_intrinsics depth_intrin;
	rs2_extrinsics depth2color_extrin;
	rs2_extrinsics color2depth_extrin;
	rs2::pointcloud  rs2Cloud;
	rs2::points      rs2Points;
	rs2::align       align;
	rs2::pipeline    pipe;
	rs2::config      cfg;
	rs2::frameset    frameSet;	
	cv::Mat 		srcImage;
	cv::Mat         depthImage;
};
#endif
