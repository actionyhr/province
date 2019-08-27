/*

- @Author: mikey.zhaopeng 

- @Date: 2019-08-01 09:11:59 

- @Last Modified by: mikey.zhaopeng

- @Last Modified time: 2019-08-01 15:34:02
  */
  #include "robot_locator.h"
  std::ofstream outfile("test.txt");
  RobotLocator::RobotLocator()
  {
  srcImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  dstImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  bgImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  allBallImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelB = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelG = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelR = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);

  channelH = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelS = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);

  channelL = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
  channelA = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);

  LABImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);
  HSVImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);

  depthImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_16UC1);

}

void RobotLocator::init(ActD435& d435)
{

    cout << "Initializing locator..." << endl;

    //-- Set input device
    thisD435 = &d435;

    thisD435->getCameraParam(color_intrin, depth_intrin, depth2color_extrin, color2depth_extrin, m_data);		
    //-- Drop several frames for stable point cloud
    for (int i = 0; i < 3; i++)
    {
        thisD435->update();
    }
    cout << "Locator init done..." << endl;

    }
    void RobotLocator::updateImage(void)
    {   
        thisD435->update();
        srcImage = thisD435->getSrcImage().clone();
        depthImage = thisD435->getDepthImage().clone();
        m_data = thisD435->getData();
        depthData = thisD435->data;
    //thisTagDetector->apriltagDetect(srcImage);


}
void RobotLocator::sign_color(cv::Mat &color)
{
   cv::Mat color_hsv;
   cv::cvtColor(color,color_hsv,cv::COLOR_BGR2HSV); 
   for(int i=0;i<230;i++)
    {
        auto p=color.ptr(i);
        for(int j=0;j<640;j++)
        {
          p[j*3]=255;
          p[j*3+1]=255;
          p[j*3+2]=255;
        }
    }
  for(int i=230;i<480;i++)
  {
     uchar* p=color.ptr(i);
     const uchar* hsv_data=color_hsv.ptr(i);
      for(int j=0;j<640;j++)
      {
           if(hsv_data[j*3]>5&&hsv_data[j*3]<35&&hsv_data[j*3+1]>100)
          {
                    
                    p[j*3]=0;
                    p[j*3+1]=0;
                    p[j*3+2]=0;
          }
          else
          {
                    p[j*3]=255;
                    p[j*3+1]=255;
                    p[j*3+2]=255;
          }
      }
  }
  for(int rows=235;rows<475;rows++)
  {
      for(int cols=5;cols<635;cols++)
      {
          int flag=0;
          for(int i=-3;i<3;i++)
          {
              for(int j=-3;j<3;j++)
              {
                  uchar* ptr=color.ptr(rows+i);
                  uchar b=ptr[(cols+j)*3];
                  uchar g=ptr[(cols+j)*3+1];
                  uchar r=ptr[(cols+j)*3+2];
                  if(!(b==0&&g==0&&r==0))
                  {
                      flag=1;
                      break;
                  }
              }
              if(flag==1)
              break;
          }
          if(flag==0)
          {
              color.at<Vec3b>(rows,cols)[2]=255;
          }
      }
  }
  for(int i=230;i<480;i++)
  {
      uchar* p=color.ptr(i);
      for(int j=0;j<640;j++)
      {
           if(p[j*3]==0&&p[j*3+1]==0&&p[j*3+2]==255)
          {
                    
                    p[j*3]=0;
                    p[j*3+1]=0;
                    p[j*3+2]=0;
          }
          else
          {
                    p[j*3]=255;
                    p[j*3+1]=255;
                    p[j*3+2]=255;
          }
      }
  }
}
void RobotLocator::findMaxAndIndex(int x[],int length,int& max,int& index)
{
  max=0;
  index=0;
  for(int i=0;i<length;i++)
  {
    max=max>x[i]?max:x[i];
  }
  for(int i=0;i<length;i++)
  {
    if(max==x[i])
    {
      index=i;
      break;
    }
  }
}
void RobotLocator::imagePrecess(void)
{
    afterLine = srcImage.clone();
    SigedColorImage=srcImage.clone();
    sign_color(SigedColorImage);
}
void RobotLocator::findBoundary(void)
{
    
    cv::TickMeter tk;
    tk.start();
	imagePrecess();
	cv::Point2f corner;
    getBoundaryPoint(SigedColorImage);
    findLine();   
    float point[3];
    float pixel[2];
    tk.stop();

}
void RobotLocator::getPointFromPixel(float colorPixel[2],float point[3])
{
    float depthPixel[2];
    rs2_project_color_pixel_to_depth_pixel(depthPixel,depthData,1.0f,0.5,5,&depth_intrin,&color_intrin,
                                            &color2depth_extrin,&depth2color_extrin,colorPixel);
    
    int raw = depthPixel[1];
    int clo = depthPixel[0];
    float depth = depthData[raw * depth_intrin.width + clo];
    rs2_deproject_pixel_to_point(point , &depth_intrin, depthPixel ,depth);
}
void RobotLocator::findCorner(std::vector<Line> &lines, cv::Point2f *corner)
{
    //cv::Point corner;
    corner->x = -(lines[0].b-lines[1].b)/(lines[0].k-lines[1].k);
    corner->y = lines[0].k * corner->x + lines[0].b;
}
void RobotLocator::getBoundaryPoint(cv::Mat color)
{
    firstVector.clear();
    point.clear();
    cv::TickMeter tk;
   
    float pointmid[3];
    float pixel[2];
    vector<Point3f> point3D;
    vector<Point2d>  Point2D;
    vector<Point2d> pointVector;
    vector<Point2d> pointVector2;
    Mat visionData=Mat::zeros(Size(1000,1000),CV_8UC3);
    for(int rows=0;rows<480;rows++)
    {
      uchar* ptr=color.ptr(rows);
      for(int cols=0;cols<640;cols++)
      {
        if(ptr[cols*3]==0&&ptr[cols*3+1]==0&&ptr[cols*3+2]==0)
        {
          Point2D.push_back(Point(cols,rows));
        }
      }
    }
    for(int i=0;i<Point2D.size();i++)
    {
       pixel[0]=Point2D.at(i).x;
       pixel[1]=Point2D.at(i).y;
       getPointFromPixel(pixel,pointmid);
       if(pointmid[1]>0&&pointmid[1]<500)
       {
       point3D.push_back(Point3d(pointmid[0],pointmid[1],pointmid[2]));
       }
    }
    for(int k=0;k<point3D.size();k++)
    {
        int rows=point3D.at(k).z/10+100;
        int cols=point3D.at(k).x/10+300;
        if(rows<1000&&cols<1000)
        {
        visionData.at<Vec3b>(rows,cols)[0]=255;
        visionData.at<Vec3b>(rows,cols)[1]=255;
        visionData.at<Vec3b>(rows,cols)[2]=255;
        point.push_back(Point2d(cols,rows));
        }
    }
    //----------------霍夫线变换-------------------//
    Mat gray;
    cvtColor(visionData,gray,COLOR_BGR2GRAY);
    vector<cv::Vec2f> line;
    vector<cv::Vec2f> lines_up; 
    vector<cv::Vec2f> lines_down;  
    double twoLine_K[2]={ 0 };
    double twoLine_b[2]={ 0 };
    cv::HoughLines(gray,line,1,CV_PI/180,30,0,0);
    int mostFittedLineUpIndex=0;
    int mostFittedLineDownIndex=0;
    int MaxFittedPointNumOnLineUp=0;
    int MaxFittedPointNumOnLineDown=0;
    //----------------按升降讲程序分为两类-------------------//
    if(line.size()>0)
    {
        for(int i=0;i<line.size();i++)
        {
            float rho=line[i][0],theta=line[i][1];
            double k=-(cos(theta)/sin(theta));
            double b=rho/sin(theta);
            if(k>=0)
            lines_up.push_back(line.at(i));
            else
            lines_down.push_back(line.at(i));
        }
    }
    int fittedPointNumUP[2000]={ 0 };
    int fittedPointNumDown[2000]={ 0 };
    //----------------按升降讲程序分为两类-------------------//
    //----------------根据内点的个数选出最优的两条直线-----------------//
    if(lines_up.size()>0)
    {
        for(int i=0;i<lines_up.size();i++)
        {
            float rho=lines_up[i][0],theta=lines_up[i][1];
            double k=-(cos(theta)/sin(theta));
            double b=rho/sin(theta);
            for(int j=0;j<point.size();j++)
            {
                double distance=fabs(point.at(j).y-k*point.at(j).x-b)/sqrt(k*k+1);
                if(distance<5)
                {
                fittedPointNumUP[i]++;
                pointVector.push_back(Point2d(point3D.at(j).x,point3D.at(j).z));
                }
            }
        }
        findMaxAndIndex(fittedPointNumUP,lines_up.size(),MaxFittedPointNumOnLineUp,mostFittedLineUpIndex);
    }
    if(lines_down.size()>0)
    {
        for(int i=0;i<lines_down.size();i++)
        {
        float rho=lines_down[i][0],theta=lines_down[i][1];
        double k=-(cos(theta)/sin(theta));
        double b=rho/sin(theta);
        for(int j=0;j<point.size();j++)
        {
            double distance=fabs(point.at(j).y-k*point.at(j).x-b)/sqrt(k*k+1);
            if(distance<5)
            {
             pointVector2.push_back(Point2d(point3D.at(j).x,point3D.at(j).z));
             fittedPointNumDown[i]++;
            }
        }
        }
        findMaxAndIndex(fittedPointNumDown,lines_down.size(),MaxFittedPointNumOnLineDown,mostFittedLineDownIndex);
    }
    //----------------根据内点的个数选出最优的两条直线-----------------//
    //---------------算出两条线的斜率和截距--------------------//
    if(MaxFittedPointNumOnLineUp>200)  
    {
            float rho=lines_up[mostFittedLineUpIndex][0],theta=lines_up[mostFittedLineUpIndex][1];
            double k=-(cos(theta)/sin(theta));
            double b=rho/sin(theta);
            twoLine_K[0]=k;
            twoLine_b[0]=b;
    }
    if(MaxFittedPointNumOnLineDown>200)  
    {
            float rho=lines_down[mostFittedLineDownIndex][0],theta=lines_down[mostFittedLineDownIndex][1];
            double k=-(cos(theta)/sin(theta));
            double b=rho/sin(theta);
            twoLine_K[1]=k;
            twoLine_b[1]=b;
    }
    //---------------算出两条线的斜率和截距--------------------//
    //---------------画出线---------------//
    for(int i=0;i<1000;i++)
    {
        if(twoLine_K[0]!=0&&twoLine_K[1]!=0)
        {
            if((i*twoLine_K[0]+twoLine_b[0]<1000&&i*twoLine_K[0]+twoLine_b[0]>0)&&(i*twoLine_K[1]+twoLine_b[1]<1000&&i*twoLine_K[1]+twoLine_b[1]>0))
            {
                visionData.at<Vec3b>(i*twoLine_K[0]+twoLine_b[0],i)[2]=255;
                visionData.at<Vec3b>(i*twoLine_K[1]+twoLine_b[1],i)[2]=255;
            }
        }
        else
        {
            if((i*twoLine_K[0]+twoLine_b[0]<1000&&i*twoLine_K[0]+twoLine_b[0]>0)||(i*twoLine_K[1]+twoLine_b[1]<1000&&i*twoLine_K[1]+twoLine_b[1]>0))
            {
                visionData.at<Vec3b>(i*twoLine_K[0]+twoLine_b[0],i)[2]=255;
                visionData.at<Vec3b>(i*twoLine_K[1]+twoLine_b[1],i)[2]=255;
            }
        }
        
    }
    //---------------画出线---------------//
    if(pointVector.size()>50)
    firstVector.push_back(pointVector);
    if(pointVector2.size()>50)
    firstVector.push_back(pointVector2);
    //----------------霍夫线变换-------------------//
    imshow("Data",visionData);
    imshow("boundary",color);
    waitKey(1);
}
void RobotLocator::findLine()
{
    Vec4f fitline;
    Line l;
    Point2f corner;
    boundaryLines.clear();
    for(int i=0;i<firstVector.size();i++)
    {
        fitLine(firstVector.at(i),fitline,cv::DIST_L2,0,0.01,0.01);
        l.k = fitline[1]/fitline[0];
        l.b = fitline[3] - l.k * fitline[2];
        boundaryLines.push_back(l);
    }
    if(firstVector.size()==2)
    {
        findCorner(boundaryLines,&corner);
        leftDistance=fabs(boundaryLines.at(0).b)/sqrt(boundaryLines.at(0).k*boundaryLines.at(0).k+1);
        rightDistance=fabs(boundaryLines.at(1).b)/sqrt(boundaryLines.at(1).k*boundaryLines.at(1).k+1);
        cornerAngle=atanf(corner.y/corner.x)/CV_PI*180;
        if(cornerAngle>0)
        cornerAngle=90-cornerAngle;
        else
        cornerAngle=-(90+cornerAngle);
        lineStatus=3;
    }
    lineStatus=1;
    cout<<leftDistance<<" "<<rightDistance<<" "<<cornerAngle<<" ";
}  
void RobotLocator::showImage(void)
{
        cv::imshow("src", srcImage);
        //cv::imshow("HSVImage", HSVImage);
        //cv::imshow("allBallImage", allBallImage);
        cv::imshow("afterLine", afterLine);
        cv::waitKey(1);
}
RobotLocator::~RobotLocator()
{
}