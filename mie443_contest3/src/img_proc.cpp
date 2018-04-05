#include "func_header.h"
#include <string>
#include <unistd.h>
using namespace cv;
using namespace cv::xfeatures2d;

#include <sstream>

#include <cmath>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

//Calculate SSD
double getSimilarity(cv::Mat A, cv::Mat B ) {
  if ( A.rows > 0 && A.rows == B.rows && A.cols > 0 && A.cols == B.cols ) {
      // Calculate the L2 relative error between images.
      double errorL2 = norm( A, B, CV_L2 );
      // Convert to a reasonable scale, since L2 error is summed across all pixels of the image.
      double similarity = errorL2 / (double)( A.rows * A.cols );
      return similarity;
  }
  else {
      return 100000000.0;  // Return a bad value
  }
}

int findPic(imageTransporter &imgTransport, cv::Mat &imgs_track, int iteration){//modified function
  // printf("Error-2\n");
  // this usually support jpg but not png??? need to test??
  imgs_track = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/wanted.png", IMREAD_GRAYSCALE );
  cv::namedWindow("view");
  int foundPic = 0;//0: not found, 1: found
  cv::Mat video, video_resize;
  double SSD1 = 100;
  video = imgTransport.getImg(); // For actual funcfor (int i = 0; i < scene_transformed,size(),height; i++)tion
  

  // printf("Error-1\n");
  if(!video.empty()){
    // printf("Error0.5\n");
    Size size(imgs_track.rows,imgs_track.cols);//the dst image size,e.g.100x100
    resize(video,video_resize,size);//resize image
	  // fill with your code
    Mat video_grey;
    cv::cvtColor(video, video_grey, cv::COLOR_BGR2GRAY);
    // WANTED
    
    // printf("styupid7");
    // printf("Error0\n");
    SSD1 = feature2D_homography(imgs_track, video_grey);
    // printf("Error1\n");
    double error_threshold = 0.15;
    printf("SSD1: %lf\n", SSD1);

    if (SSD1 < error_threshold){
      foundPic = 1;
    }
    
    //cv::imshow("view", video);
    //change address*******************************************************************************************
    cv::imwrite("/home/turtlebot/catkin_ws/src/mie443_contest3/tag"+ patch::to_string(iteration) +".jpg", video); // CHANGE IMAGE PATH
    cv::waitKey(10);
    video.release();
    cv::waitKey(10);
  }
  // foundPic = 0;
  return foundPic;
}

/** @function main */
double feature2D_homography (cv::Mat img_object, cv::Mat img_scene )//(const char *image1, const char *image2 )
{

  if( !img_object.data || !img_scene.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

//   SurfFeatureDetector detector( minHessian ); 
  Ptr<SURF> detector = SURF::create( minHessian );
  // printf("styupid8");
  // printf("REACH 2\n");
  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector->detect( img_object, keypoints_object );
  detector->detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  Ptr<SurfDescriptorExtractor> extractor = SURF::create();

  Mat descriptors_object, descriptors_scene;

  extractor->compute( img_object, keypoints_object, descriptors_object );
  extractor->compute( img_scene, keypoints_scene, descriptors_scene );
  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )double SSD = getSimilarity(img_object, scene_transformed);
  std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
     { good_matches.push_back( matches[i]); }
  }

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }
  //printf("styupid11");
  Mat H = findHomography( scene, obj, RANSAC ); // Find transformation from object to scene
  /*if(scene.size()<5){
    return 999990;
  }*/

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);

  obj_corners[0] = cvPoint(0,0); 
  obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); 
  obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  try
  {
    if (! H.empty())
    {    
      perspectiveTransform( obj_corners, scene_corners, H);
    }
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    Mat scene_transformed;
    warpPerspective(img_scene, scene_transformed, H, img_object.size()); // Warp scene image to zoom into object portion
    //change address*******************************************************************************************
    //cv::imwrite("/home/andrew/catkin_ws/src/mie443_contest2/src/tag_from_scene.jpg", scene_transformed); // CHANGE IMAGE PATH
    
    //-- Show detected matchesdouble
    //imshow( "Good Matches & Object detection", img_matches );
    

    // Andrew: Calculate the SSD between images (wraped scene to object:scene_transformed and view:img_object)
    cv::Mat img_object_bw;
    cv::Mat scene_transformed_bw;
    // Convert images to binary
    cv::threshold(scene_transformed, scene_transformed_bw, 128.0, 255.0, THRESH_BINARY);
    cv::threshold(img_object, img_object_bw, 128.0, 255.0, THRESH_BINARY);
    //double SSD = getSimilarity(img_object, scene_transformed);
    double SSD = getSimilarity(img_object_bw, scene_transformed_bw);
    //imshow( "Warped Scene to Object_2", img_object_bw );
    //imshow( "Warped Scene to Object", scene_transformed_bw );
    
    waitKey(10);
    return SSD;
  }
  catch (int e)
  {
    return 99999;
  }
  

  
  }

  /** @function readme */
  void readme()
{ std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }