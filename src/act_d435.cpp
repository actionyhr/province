#include "act_d435.h"

ActD435::ActD435():
align(RS2_STREAM_COLOR)
{    

}

void ActD435::init(void)
{
	pipe.stop();
    #ifndef IFCAMERA
	cfg.enable_device_from_file("/home/action/bag/all.bag");
#else
	auto devices = ctx.query_devices();
	device_count = devices.size();
	if (!device_count)
	{
		cout <<"No device detected. Is it plugged in?\n";
		return;
	}
	// Get the first connected device
	auto dev = devices[0];

	// Enter advanced mode
	if (dev.is<rs400::advanced_mode>())
	{
		// Get the advanced mode functionality
		auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

		// Load and configure .json file to device
		ifstream t("../modeJson/HighResHighAccuracyPreset.json");
		string str((istreambuf_iterator<char>(t)), istreambuf_iterator<char>());
		advanced_mode_dev.load_json(str);
	}
	else
	{
	cout << "Current device doesn't support advanced-mode!\n";
	return;
	}
	//-- Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
#endif
    //cfg.enable_device_from_file("/home/action/bag/1m54.bag");
    //cfg.enable_stream(RS2_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_BGR8, 15);
    //cfg.enable_stream(RS2_STREAM_DEPTH, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Z16, 15);
    cout << "enable stream Ok ..\n";
  	rs2_sensor *depthSensor;
    pipe.start(cfg);
    cout << "start cfg Ok ..\n";

    //-- Wait for frames from the camera to settle
	for (int i = 0; i < 5; i++)
	{
		//Drop several frames for auto-exposure
		frameSet = pipe.wait_for_frames();
	}

    rs2::video_frame colorFrame = frameSet.get_color_frame();
	rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();

    rs2::stream_profile dprofile = alignedDepthFrame.get_profile();
    rs2::stream_profile cprofile = colorFrame.get_profile();
    
    rs2::video_stream_profile cvsprofile(cprofile);
    color_intrin = cvsprofile.get_intrinsics();

    rs2::video_stream_profile dvsprofile(dprofile);
    depth_intrin = dvsprofile.get_intrinsics();

    depth2color_extrin = dprofile.get_extrinsics_to(cprofile);
    color2depth_extrin = cprofile.get_extrinsics_to(dprofile);
    cout << "Get intrinics ..\n";

    cout << "Camera D435 init done ..\n";
}

void ActD435::init2(void)
{
    #ifndef IFCAMERA
	cfg.enable_device_from_file("/home/action/bag/all.bag");
#else
	auto devices = ctx.query_devices();
	device_count = devices.size();
	if (!device_count)
	{
		cout <<"No device detected. Is it plugged in?\n";
		return;
	}
	// Get the first connected device
	auto dev = devices[0];

	// Enter advanced mode
	if (dev.is<rs400::advanced_mode>())
	{
		// Get the advanced mode functionality
		auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

		// Load and configure .json file to device
		ifstream t("../modeJson/hand.json");
		string str((istreambuf_iterator<char>(t)), istreambuf_iterator<char>());
		advanced_mode_dev.load_json(str);
	}
	else
	{
	cout << "Current device doesn't support advanced-mode!\n";
	return;
	}
	//-- Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
#endif
    //cfg.enable_device_from_file("/home/action/bag/1m54.bag");
    //cfg.enable_stream(RS2_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_BGR8, 15);
    //cfg.enable_stream(RS2_STREAM_DEPTH, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Z16, 15);
    cout << "enable stream Ok ..\n";
  	rs2_sensor *depthSensor;
    pipe.start(cfg);
    cout << "start cfg Ok ..\n";

    //-- Wait for frames from the camera to settle
	for (int i = 0; i < 5; i++)
	{
		//Drop several frames for auto-exposure
		frameSet = pipe.wait_for_frames();
	}

    rs2::video_frame colorFrame = frameSet.get_color_frame();
	rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();

    rs2::stream_profile dprofile = alignedDepthFrame.get_profile();
    rs2::stream_profile cprofile = colorFrame.get_profile();
    
    rs2::video_stream_profile cvsprofile(cprofile);
    color_intrin = cvsprofile.get_intrinsics();

    rs2::video_stream_profile dvsprofile(dprofile);
    depth_intrin = dvsprofile.get_intrinsics();

    depth2color_extrin = dprofile.get_extrinsics_to(cprofile);
    color2depth_extrin = cprofile.get_extrinsics_to(dprofile);
    cout << "Get intrinics ..\n";

    cout << "Camera D435 init done ..\n";
}

void ActD435::update()
{
	cv::TickMeter tk;
    tk.start();

    frameSet = pipe.wait_for_frames();

    tk.stop();
    //cout<<" "<<tk.getTimeMilli()<<" ";
    //auto frame =align.process(frameSet);

    rs2::video_frame colorFrame = frameSet.get_color_frame();
    rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();

    srcImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
#ifndef IFCAMERA
	cvtColor(srcImage, srcImage,cv::COLOR_RGB2BGR);
#endif
    depthImage = cv::Mat(cv::Size(640, 480), CV_16UC1, (void*)alignedDepthFrame.get_data(), cv::Mat::AUTO_STEP);
    data = (uint16_t*)alignedDepthFrame.get_data();
}

void ActD435::getCameraParam(rs2_intrinsics& _color_intrin, rs2_intrinsics& _depth_intrin, rs2_extrinsics& _depth2color_extrin, rs2_extrinsics& _color2depth_extrin, uint16_t* _data)
{
    _color_intrin = color_intrin;
    _depth_intrin = depth_intrin;
    _depth2color_extrin = depth2color_extrin;
    _color2depth_extrin = color2depth_extrin;
}

ActD435::~ActD435()
{    

}

