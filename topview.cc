/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images

   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM

   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "fisheye_param.h"

#include <iostream>
#include <string>
#include <vector>

using namespace std;


cv::Mat src_left, src_front, src_right, src_back;
cv::Mat large;
cv::Mat topview_full;

cv::Matx33d Rotation_left, Rotation_front, Rotation_right, Rotation_back;
cv::Vec3d Translation_left, Translation_front, Translation_right, Translation_back;

FisheyeParam ocam_model_left, ocam_model_front, ocam_model_right, ocam_model_back;


const int ROWS = 800;
const int COLS = 800;
const int HIGH = -1;

void translateTwc2Tcw(cv::Matx33d& R, cv::Vec3d& t)
{
  R = R.t();  //由Twc转化为Tcw
  t = -R * t;
}

void readTransformation(std::string filename, cv::Matx33d& R, cv::Vec3d& t)
{
  std::ifstream result_in(filename.data());
  assert(result_in.is_open());

  cv::Matx44d T;
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 4; j++)
    {
        result_in >> T(i,j);
    }
  }

  R = T.get_minor<3,3>(0,0);
  t = cv::Vec3d(T(0,3), T(1,3), T(2,3));
  result_in.close();
}


void LoadMultiCamImages(vector<vector<string>> &vvstrImageFilenames)
{
    vvstrImageFilenames.resize(4);
    for(int i = 0 ;  i<  6000 ;i++)
       {
           char imagename1[100];
           char imagename2[100];
           char imagename3[100];
           char imagename4[100];

           sprintf(imagename1,"/home/user/codes/mendeo_data/bmp/frame_vc9_%d.bmp",i);
           sprintf(imagename2,"/home/user/codes/mendeo_data/bmp/frame_vc10_%d.bmp",i);
           sprintf(imagename3,"/home/user/codes/mendeo_data/bmp/frame_vc11_%d.bmp",i);
           sprintf(imagename4,"/home/user/codes/mendeo_data/bmp/frame_vc12_%d.bmp",i);

           string simageName1 = string(imagename1);
           string simageName2 = string(imagename2);
           string simageName3 = string(imagename3);
           string simageName4 = string(imagename4);

           vvstrImageFilenames[0].push_back(simageName1);
           vvstrImageFilenames[1].push_back(simageName2);
           vvstrImageFilenames[2].push_back(simageName3);
           vvstrImageFilenames[3].push_back(simageName4);
      }

}

void Load()
{
  ocam_model_left.Load("../intrinsic_parameters/left/calib.txt");
  ocam_model_front.Load("../intrinsic_parameters/front/calib.txt");
  ocam_model_right.Load("../intrinsic_parameters/right/calib.txt");
  ocam_model_back.Load("../intrinsic_parameters/back/calib.txt");

  readTransformation("../result/left.txt", Rotation_left, Translation_left);  //Rwc, twc
  readTransformation("../result/front.txt", Rotation_front, Translation_front);
  readTransformation("../result/right.txt", Rotation_right, Translation_right);
  readTransformation("../result/back.txt", Rotation_back, Translation_back);

}

void topview(cv::Mat& output, cv::Mat left, cv::Mat front, cv::Mat right, cv::Mat back, 
             cv::Matx33d R_l, cv::Vec3f t_l, cv::Matx33d R_f, cv::Vec3f t_f, cv::Matx33d R_r, cv::Vec3f t_r, cv::Matx33d R_b, cv::Vec3f t_b, 
             FisheyeParam& ocam_left, FisheyeParam& ocam_front, FisheyeParam& ocam_right, FisheyeParam& ocam_back)
{
  output = cv::Mat::zeros(ROWS, COLS, left.type());
  
  int width = output.cols;
  int height = output.rows;
  float k = (float)height / width;
  float scale = 25;
  float fx = scale, fy = scale*k;
  float theta = 25 * CV_PI / 180;
  cv::Point2f point2d;
  for(int i = 0; i < height; i++)
    for(int j = 0; j < width; j++)
    {
      float x = j - width/2, y = i - height/2;
      float tan_theta = tan(theta);
      cv::Vec3f world_point(x / fx, y / fy, 0);
      if(x < y * tan_theta && x < -y * tan_theta)
      {
        // Pc = Rcw * Pw + tcw
        // world_point = R_l * world_point; // Rcw
        // world_point = world_point + t_l; // tcw

        // Pw = Rwc * Pc + twc
        world_point = world_point - t_l; // twc
        world_point = R_l.t() * world_point; // Rwc
        point2d = ocam_left.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
        int u = point2d.x, v = point2d.y;
        if(u >= 0 && u < left.cols && v >= 0 && v < left.rows)
          output.at<cv::Vec3b>(i,j) = left.at<cv::Vec3b>(v, u);
      }
      else if(x > y * tan_theta && x < -y * tan_theta)
      {
        // Pc = Rcw * Pw + tcw
        // world_point = R_f * world_point; // Rcw
        // world_point = world_point + t_f; // tcw

        // Pw = Rwc * Pc + twc
        world_point = world_point - t_f; // twc
        world_point = R_f.t() * world_point; // Rwc
        point2d = ocam_front.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
        int u = point2d.x, v = point2d.y;
        if(u >= 0 && u < front.cols && v >= 0 && v < front.rows)
          output.at<cv::Vec3b>(i,j) = front.at<cv::Vec3b>(v, u);
      }
      else if(x > -y * tan_theta && x > y * tan_theta)
      {
        // Pc = Rcw * Pw + tcw
        // world_point = R_r * world_point; // Rcw
        // world_point = world_point + t_r; // tcw

        // Pw = Rwc * Pc + twc
        world_point = world_point - t_r; // twc
        world_point = R_r.t() * world_point; // Rwc
        point2d = ocam_right.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
        int u = point2d.x, v = point2d.y;
        if(u >= 0 && u < right.cols && v >= 0 && v < right.rows)
          output.at<cv::Vec3b>(i,j) = right.at<cv::Vec3b>(v, u);
      }
      else
      {
        // Pc = Rcw * Pw + tcw
        // world_point = R_b * world_point; // Rcw
        // world_point = world_point + t_b; // tcw

        // Pw = Rwc * Pc + twc
        world_point = world_point - t_b; // twc
        world_point = R_b.t() * world_point; // Rwc
        point2d = ocam_back.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
        int u = point2d.x, v = point2d.y;
        if(u >= 0 && u < back.cols && v >= 0 && v < back.rows)
          output.at<cv::Vec3b>(i,j) = back.at<cv::Vec3b>(v, u);
      }
    }
}


void mergeImages(cv::Mat img1, cv::Mat img2, cv::Mat img3, cv::Mat img4, cv::Mat& dst)
{
  dst = cv::Mat(img1.rows, img1.cols+img2.cols+img3.cols+img4.cols, img1.type(), cv::Scalar::all(0));
  cv::Mat ROI = dst(cv::Rect(0, 0, img1.cols, img1.rows));
  img1.copyTo(ROI);
  ROI = dst(cv::Rect(img1.cols, 0, img2.cols, img2.rows));
  img2.copyTo(ROI);
  ROI = dst(cv::Rect(img1.cols+img2.cols, 0, img3.cols, img3.rows));
  img3.copyTo(ROI);
  ROI = dst(cv::Rect(img1.cols+img2.cols+img3.cols, 0, img4.cols, img4.rows));
  img4.copyTo(ROI);
}

int main(int argc, char *argv[])
{
  Load();

  vector<vector<string>> vvstrImageFilenames;
  LoadMultiCamImages(vvstrImageFilenames);

  int nImages = vvstrImageFilenames[0].size();

  for(int i = 10 ; i< nImages; i++)
  {
      src_left = cv::imread(vvstrImageFilenames[0][i]); 
      assert(!src_left.empty());

      src_front = cv::imread(vvstrImageFilenames[1][i]); 
      assert(!src_front.empty());

      src_right = cv::imread(vvstrImageFilenames[2][i]); 
      assert(!src_right.empty());

      src_back = cv::imread(vvstrImageFilenames[3][i]); 
      assert(!src_back.empty());

      topview(topview_full, src_left, src_front, src_right, src_back, Rotation_left, Translation_left, Rotation_front, Translation_front, Rotation_right, Translation_right, Rotation_back, Translation_back, ocam_model_left, ocam_model_front, ocam_model_right, ocam_model_back);
      mergeImages(src_left, src_front, src_right, src_back,large);

      cv::namedWindow( "topview", 0 );
      cv::imshow( "topview", topview_full);

      cv::namedWindow( "large", 0 );
      cv::imshow( "large", large);

      cv::waitKey(1);
  }

  while((char)cv::waitKey(10) != 'q');
  cv::imwrite("topview_full.jpg", topview_full);
  cv::imwrite("large.jpg", large);

  return 0;
}

