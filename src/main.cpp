#include<iostream>
#include"librealsense2/rs.hpp"
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<librealsense2/rsutil.h>
#include<math.h>
#define PI 3.14159265358979
using namespace std;
using namespace cv;
using namespace rs2;
void sign_color(cv::Mat &color);
void getBoundaryPoint(Mat &color,vector<cv::Point2d> &point);
vector<cv::Point2d> ExponentialSmoothing(vector<cv::Point2d> point);
void draw_line(Mat &color,vector<cv::Point2d> point);
void sort(vector<cv::Point2d> &p,int flag);
bool sortFun_y(const  cv::Point2d &p1,  const cv::Point2d &p2);
void polynomial_curve_fit(vector<Point2d> points, int n, cv::Mat& A);
void removeNoise(Mat polynomial,int n,vector<cv::Point2d> inpuArray,vector<cv::Point2d> &outputArray);
int main()
{
   rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
    rs2::pipeline_profile profile = pipe.start(cfg);
   while(1)
   {
          vector<cv::Point2d> BoundaryPoint;
          vector<cv::Point2d> usefulBoundaryPoint;
          vector<cv::Point2d> SmoothBoundaryPoint;
          rs2::frameset frames=pipe.wait_for_frames();
          rs2::frame frame_color=frames.get_color_frame();
          rs2::depth_frame depth=frames.get_depth_frame();
          Mat Polynomial;
          Mat color_image(Size(640,480),CV_8UC3,(void*)frame_color.get_data(),Mat::AUTO_STEP);
          Mat color_clone=color_image.clone();
          double time0=static_cast<double>(cv::getTickCount());
          sign_color(color_clone);
          getBoundaryPoint(color_clone,BoundaryPoint);
          //polynomial_curve_fit(BoundaryPoint,10,Polynomial);
          //removeNoise(Polynomial,10,BoundaryPoint,usefulBoundaryPoint);
          //SmoothBoundaryPoint=ExponentialSmoothing(BoundaryPoint);
          draw_line(color_clone,BoundaryPoint);
          cv::imshow("color",color_clone);
          cv::waitKey(10);
          time0=((double)cv::getTickCount()-time0)/cv::getTickFrequency();
          cout<<"运行时间为："<<time0<<"秒"<<endl;
   }
}
void sign_color(cv::Mat &color)
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
         /* if(hsv_data[j*3]>95&&hsv_data[j*3]<115&&hsv_data[j*3+1]>190&&hsv_data[j*3+2]>20&&hsv_data[j*3+2]<150)
          {
                    p[j*3]=255;
                    p[j*3+1]=0;
                    p[j*3+2]=0;
          }
          else if(hsv_data[j*3]>60&&hsv_data[j*3]<88&&hsv_data[j*3+1]>100&&hsv_data[j*3+2]>20)
          {
                    p[j*3]=0;
                    p[j*3+1]=255;
                    p[j*3+2]=0;
          }
          else if(hsv_data[j*3]>0&&hsv_data[j*3]<5&&hsv_data[j*3+1]>150&&hsv_data[j*3+2]>10)
          {
                    p[j*3]=0;
                    p[j*3+1]=0;
                    p[j*3+2]=255;
          }
           else if(hsv_data[j*3]>160&&hsv_data[j*3]<180&&hsv_data[j*3+1]>200&&hsv_data[j*3+2]>10)
          {
                    p[j*3]=0;
                    p[j*3+1]=0;
                    p[j*3+2]=255;
          }*/
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
}
void getBoundaryPoint(Mat &color,vector<cv::Point2d> &point)
{
  int flag=0;    
  int above[640]={ 0 };
  int below[640]={ 0 };
  int mode=0;
  int num=0;
  uchar* p=color.ptr(231);
  for(int cols=0;cols<640;cols++)
  {
    flag=0;
    for(int i=0;i<10;i++)
    {
      uchar blue=p[(cols+i)*3];
      uchar green=p[(cols+i)*3+1];
      uchar red=p[(cols+i)*3+2];
      if(!(blue==0&&green==0&&red==0))
      {
        flag=1;
        break;
      }
    }
    if(flag==0)
    {
        num++;
    }
  }
  if(num>100)
  mode=1;
  for(int cols=0;cols<640;cols++)
  {   
    for(int rows=470;rows>240;rows--)
    {
        flag=0;
        for(int i=0;i<10;i++)
        {
          uchar* ptr=color.ptr(rows-i);
          uchar b=ptr[cols*3];
          uchar g=ptr[cols*3+1];
          uchar r=ptr[cols*3+2];
          if(!(b==0&&g==0&&r==0))
          {
            flag=1;
            break;
          }
        }
        if(flag==0)
        {
          if(rows>=475)
          {
            below[cols]=0;
            break;
          }      
          else 
          { 
            below[cols]=rows;
            break;
          }
        }
    }
  }
   for(int cols=0;cols<640;cols++)
  {   
    for(int rows=470;rows>240;rows--)
    {
       flag=0;
        for(int i=1;i<10;i++)
        {
          uchar* ptr1=color.ptr(rows-i);
          uchar b1=ptr1[cols*3];
          uchar g1=ptr1[cols*3+1];
          uchar r1=ptr1[cols*3+2];
          uchar* ptr2=color.ptr(rows+i);
          uchar b2=ptr2[cols*3];
          uchar g2=ptr2[cols*3+1];
          uchar r2=ptr2[cols*3+2];
          if(!(b1==255&&g1==255&&r1==255))
          {
            flag=1;
            break;
          }
          if(!(b2==0&&g2==0&&r2==0))
          {
            flag=1;
            break;
          }
        }
        if(flag==0)
        {
            above[cols]=rows;
            break;
        }

    }
  }
  for(int i=0;i<640;i++)
  {
    if((abs(above[i]-below[i])<65&&abs(above[i]-below[i])>10&&above[i]!=0&&below[i]!=0)||mode==1)
    {
       if(i<=10)
         point.push_back(Point2d(i,below[i]));
       else if(abs(below[i]-below[i-1])<3&&abs(below[i]-below[i-2])<4&&abs(below[i]-below[i-3])<5)
         point.push_back(Point2d(i,below[i]));
       else if(below[i-1]==0)
         point.push_back(Point2d(i,below[i]));
         
    }
  }
  
  /*if(point.size()>100)
  {
    int relatedPointNum[point.size()]={ 0 };
    for(int i=0;i<point.size();i++)
    {
      for(int j=0;j<i;j++)
      {
        double x1=point.at(i).x;
        double y1=point.at(i).y;
        double x2=point.at(j).x;
        double y2=point.at(j).y;
        double distance=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
        if(distance<=30)
        {
          relatedPointNum[i]++;
          if(relatedPointNum[i]>=20)
          break;
        }
      }
      for(int j=i+1;j<point.size();j++)
      {
        double x1=point.at(i).x;
        double y1=point.at(i).y;
        double x2=point.at(j).x;
        double y2=point.at(j).y;
        double distance=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
        if(distance<=30)
        {
          relatedPointNum[i]++;
          if(relatedPointNum[i]>=20)
          break;
        }
      }
      if(relatedPointNum[i]<20)
      {
        point.erase(point.begin()+i);
        i--;
      }
    }
  }*/
  /*/-------------------数据修复------------------//
  if(point.size()>400)
  {
    double begin;
    double end=0;
    int last=0;
    int next=0;
    Vec4f fitLine;
    double k=0;
    double b=0;
    for(int i=1;i<point.size();i++)
    {
      if(abs(point.at(i).x-point.at(i-1).x)>20)
      {
        if(i>30&&i<570)
        {
            last=i-30;
            next=i+30;
            begin=i+30;
            end=i+70;            
        }
        if(i<30)
        {
            last=0;
            next=i+30;
            begin=i+30;
            end=i+70; 
        }
      }
    }
    if(begin!=0&&end!=0)
    {
      vector<cv::Point2d> usefulPoint; 
      for(int i=begin;i<end;i++)
      {
        usefulPoint.push_back(point.at(i));
      }
      cv::fitLine(usefulPoint,fitLine,cv::DIST_L2,0,0.01,0.01);
      k=fitLine[1]/fitLine[0];
      b=fitLine[3]-k*fitLine[2];
      for(int i=next;i<=last;i++)
      {
        point.push_back(cv::Point2d(i,i*k+b));
      }
    }
  }
  //-------------------数据修复------------------/*/
  //-------------------画出数据-----------------//
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  for(int i=0;i<point.size();i++)
  {
    image.at<Vec3b>(point.at(i).y,point.at(i).x)[0]=255;
  }
  imshow("修复前",image);
  //-------------------画出数据-----------------//   
}
vector<cv::Point2d> ExponentialSmoothing(vector<cv::Point2d> point)
{
  //---------------------数据的平滑处理--------------//
  vector<cv::Point2d> Forcast;
  cv::Point2d forcast;
  if(point.size()>10)
  {
  forcast=point.at(0);
  Forcast.push_back(forcast);
  for(int i=1;i<point.size();i++)
  {
    forcast.x=point.at(i-1).x;
    forcast.y=0.2*(double)point.at(i-1).y+0.8*(double)Forcast.at(i-1).y;
    Forcast.push_back(forcast);
  }
  //---------------------数据的平滑处理--------------//
  /*/-----------------数据处理------------------//
  int begin=0,end=0;
  double k[Forcast.size()-1];
  for(int i=1;i<Forcast.size();i++)
  {
      k[i-1]=fabs(Forcast.at(i).y-Forcast.at(i-1).y);
  }
  for(int i=0;i<Forcast.size()-1;i++)
  {
    if(k[i]>1.5)
    {
       begin=i;
       break;
    }
  }
   for(int i=Forcast.size()-2;i>=0;i--)
  {
    if(k[i]>1.5)
    {
       end=i;
       break;
    }
  }
  Forcast.erase(Forcast.begin()+begin,Forcast.begin()+end);
  //-----------------数据处理------------------/*/
  /*/-----------------数据处理2------------------//
  cv::Vec4f fitLine;
 cv::fitLine(firstVector[0],fitLine,cv::DIST_L2,0,0.01,0.01);
  //----------------数据处理2-----------------/*/

  //------------------显示所采集的点---------------//
    cv::Mat drawLine=cv::Mat::zeros(Size(640,480),CV_8UC1);
    for(int i=0;i<Forcast.size();i++)
  {   
      auto ptr=drawLine.ptr((int)Forcast.at(i).y);
      ptr[(int)Forcast.at(i).x]=255;   
  }
  imshow("smooth",drawLine);
  }
  //------------------显示所采集的点---------------//
  return Forcast;
}
void draw_line(Mat &color,vector<cv::Point2d> point)
{
    cv::Mat drawLine=cv::Mat::zeros(Size(640,480),CV_8UC1);
    for(int i=0;i<point.size();i++)
  {   
      auto ptr=drawLine.ptr((int)point.at(i).y);
      ptr[(int)point.at(i).x]=255;   
  }
  vector<cv::Vec2f> lines; 
  cv::HoughLines(drawLine,lines,1,CV_PI/180,80,0,0);
  for(size_t i=0;i<lines.size();i++)
  {
    for(size_t j=0;j<lines.size();j++)
    {
      if((fabs(lines[i][1]-lines[j][1])<0.04)&&i!=j)
      {
        lines[j][0]=0;
        lines[j][1]=0;
      }
    }
  }
  int count=0;
  for(size_t i=0;i<lines.size();i++)
  {
    if(lines[i][0]!=0&&lines[i][1]!=0)
    {
      float rho=lines[i][0],theta=lines[i][1];
      cv::Point pt1,pt2;
      double k=-(cos(theta)/sin(theta));
      double b=rho/sin(theta);
      pt1.x=0;
      pt1.y=b;
      pt2.x=640;
      pt2.y=640*k+b;
      line(color,pt1,pt2,Scalar(0,0,255),1,CV_AA);
      count++;
      for(int i=0;i<point.size();i++)
      {
        cout<<fabs(point.at(i).y-k*point.at(i).x-b)/sqrt(k*k+1)<<" ";
      }
    }
  }
  cout<<count<<" ";
  point.clear();
  if(count==1)
  {
    float rho=lines[0][0],theta=lines[0][1];
    cv::Point pt1,pt2;
    double k=-(cos(theta)/sin(theta));
    double b=rho/sin(theta);
    for(int i=0;i<640;i++)
    {
        point.push_back(Point2d(i,k*i+b));
    }
  }
  if(count==2)
  {
    float rho=lines[0][0],theta=lines[0][1];
    double k1=-(cos(theta)/sin(theta));
    double b1=rho/sin(theta);
    rho=lines[1][0],theta=lines[1][1];
    double k2=-(cos(theta)/sin(theta));
    double b2=rho/sin(theta);
    int x=(b2-b1)/(k1-k2);
    cout<<k1<<" "<<k2<<" "<<x<<" ";
    if(x<640&&x>0)
    {
      if(k1<0)
      {
        for(int i=0;i<x;i++)
        {
          point.push_back(Point2d(i,k1*i+b1));
        }
        for(int i=x;i<640;i++)
        {
          point.push_back(Point2d(i,k2*i+b2));
        }
      }
      else
      {
        for(int i=0;i<x;i++)
        {
          point.push_back(Point2d(i,k2*i+b2));
        }
        for(int i=x;i<640;i++)
        {
          point.push_back(Point2d(i,k1*i+b1));
        }
      }
    }
  }
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  for(int i=0;i<point.size();i++)
  {
    image.at<Vec3b>(point.at(i).y,point.at(i).x)[0]=255;
  }
  imshow("修复后",image);
}
void sort(vector<cv::Point2d> &p,int flag)
{
  if(flag==0)
  sort(p.begin(),p.end(),sortFun_y);
}
bool sortFun_y(const  cv::Point2d &p1,  const cv::Point2d &p2)
{
  return p1.y < p2.y;
}
void polynomial_curve_fit(vector<Point2d> points, int n, cv::Mat& A)
{
	//Number of key points
	int N = points.size();
 
	//构造矩阵X
	cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int j = 0; j < n + 1; j++)
		{
			for (int k = 0; k < N; k++)
			{
				X.at<double>(i, j) = X.at<double>(i, j) +
					std::pow((double)points.at(k).x, i + j);
			}
		}
	}
 
	//构造矩阵Y
	cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int k = 0; k < N; k++)
		{
			Y.at<double>(i, 0) = Y.at<double>(i, 0) +
				std::pow((double)points.at(k).x, i) * (double)points.at(k).y;
		}
	}
 
	A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	//求解矩阵A
	cv::solve(X, Y, A, cv::DECOMP_LU);
    //画出点和曲线
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    image.setTo(cv::Scalar(100, 0, 0));
    for (int i = 0; i < points.size(); i++)
    {
        cv::circle(image, points[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
    std::vector<cv::Point> points_fitted;
     for (int i = 0; i < points.size(); i++)
    {
        int x=points.at(i).x;
        double y =0;
        for(int i=0;i<=n;i++)
        {
            y+=A.at<double>(i,0)*pow(x,i);
        }
        points_fitted.push_back(cv::Point(x, y));
    }
    cv::polylines(image, points_fitted, false, cv::Scalar(0, 255, 255), 1, 8, 0);

    cv::imshow("原函数", image);


    cv::waitKey(1);
}
void removeNoise(Mat polynomial,int n,vector<cv::Point2d> inpuArray,vector<cv::Point2d> &outputArray)
{
  Mat polynomialDerivative=Mat(n+1,1,CV_64FC1);
  for(int i=0;i<n+1;i++)
  {
    polynomialDerivative.at<double>(i,0)=polynomial.at<double>(i,0)*i;
  }
 //---------------画出导函数的图像----------------------//
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    image.setTo(cv::Scalar(100, 0, 0));
    std::vector<cv::Point> points_fitted;
    std::vector<cv::Point> coordinate_system;
     for (int x = 0; x < 600; x++)
    {
        double y =0;
        for(int i=1;i<=n;i++)
        {
            y+=polynomialDerivative.at<double>(i,0)*pow(x,i-1);
        }
        points_fitted.push_back(cv::Point(x, y*100+200));
    }
     for (int x = 0; x < 640; x++)
    {
        coordinate_system.push_back(cv::Point(x, 200));
    }
    cv::polylines(image, points_fitted, false, cv::Scalar(0, 255, 255), 1, 8, 0);
    cv::polylines(image, coordinate_system, false, cv::Scalar(0, 0, 255), 1, 8, 0);
  //---------------画出导函数的图像----------------------//
  //---------------寻找导函数零点-----------------------//
   vector<int> zeroPoint;
   for(int i=3;i<points_fitted.size()-3;i++)
   {
     if((points_fitted.at(i-1).y-200)*(points_fitted.at(i+1).y-200)<=0&&(points_fitted.at(i-3).y-200)>(points_fitted.at(i+3).y-200))
     zeroPoint.push_back(points_fitted.at(i).x);
   }
    for (int i = 0; i < zeroPoint.size(); i++)
    {
        cv::circle(image, cv::Point(zeroPoint[i],200), 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
  //---------------寻找导函数零点----------------------//
  //------------------寻找坏点范围--------------------//
    int distance[zeroPoint.size()]={ 0 };
    int r[zeroPoint.size()]={ 0 };
    for(int i=0;i<zeroPoint.size();i++)
    {
       int j=1;
      while(zeroPoint.at(i)-j>=10&&zeroPoint.at(i)+j<=630&&zeroPoint.at(i)-j>0&&zeroPoint.at(i)+j<points_fitted.size())
      { 
        int nowDistance=abs(points_fitted.at(zeroPoint.at(i)-j).y-points_fitted.at(zeroPoint.at(i)+j).y);
        if(nowDistance>distance[i])
        {
          distance[i]=nowDistance;
        }
        if(nowDistance<distance[i]||zeroPoint.at(i)-j<10||zeroPoint.at(i)-j>630)
        {
          r[i]=j;
          break;
        }
        j+=1;
      }
    }
    int index=0;
    int max=distance[0];
    for(int i=1;i<zeroPoint.size();i++)
    {
      if(distance[i]>max)
      {
        index=i;
        max=distance[i];
      }
    }
  //-------------------寻找坏点范围--------------------//
  /*/--------------------修补直线-----------------------//
  if(max>70)
  {  
    int begin=zeroPoint[index]-r[index]*2;
    int end=zeroPoint[index]+r[index]*2;
    if(begin<0)
    begin=0;
    if(end>639)
    end=639;
    if(inpuArray.size()>600)
    {
      for(int i=begin;i<end;i++)
      {
        inpuArray.at(i).y=0;
      }
    }
  }
  for(int i=0;i<inpuArray.size();i++)
  {
    if(inpuArray.at(i).y!=0)
    {
      outputArray.push_back(inpuArray.at(i));
    }
  }
  //--------------------修补直线-----------------------/*/
   cv::imshow("导函数", image);
   //-----------画数据-------------//
    cv::Mat drawLine=cv::Mat::zeros(Size(640,480),CV_8UC1);
      for(int i=0;i<outputArray.size();i++)
    {   
        auto ptr=drawLine.ptr((int)outputArray.at(i).y);
        ptr[(int)outputArray.at(i).x]=255;   
    }
    imshow("berforsmooth",drawLine);
}












 