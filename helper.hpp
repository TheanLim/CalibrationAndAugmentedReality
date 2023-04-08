//
//  helper.hpp
//  Project4
//  This file contains helper functions for the main file
//  Created by Thean Cheat Lim on 3/20/23.
//

#ifndef helper_hpp
#define helper_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Do cornerSubPix and SolvePnP at once. Same args as opencv's 
int cornerSubPixSolvePnP(Mat &frame, Mat &rvec, Mat &tvec, vector<Vec3f>&worldPoints, vector<Point2f> &imagePoints, Mat &cameraMatrix, Mat &distortionCoefficients, int winSize = 11);

// Construct pyramids with a roof connecting them and store it at a vector of points
// objectPoints - A vector of points (3D)
int constructPyramidRoof(vector<Point3f> &objectPoints);

// Draw pyramids with a roof connecting them onto frame, using imagePoints
// frame - Image frame
// imagePoints - a vector of 2D image plane points
int drawPyramidRoof(Mat &frame, vector<Point2f> &imagePoints);

// Construct a Torus and store it at a vector of points
// Also compute a vector of indices and polygon points, which are useful for drawing onto image plane after projection
// objectPoints -  A vector of points (3D)
// indices - indices; useful for drawing onto image plane after projecting objectPoints
// polygon - polygon; useful for drawing onto image plane after projecting objectPoints
int constructTorus(vector<Point3f> &objectPoints, vector<int> &indices, vector<Point> &polygon);

// Draw a Torus using imagePoints
// frame - Image frame
// imagePoints - a vector of 2D image plane points
// indices - indices; useful for drawing onto image plane after projecting objectPoints
// polygon - polygon; useful for drawing onto image plane after projecting objectPoints
int drawTorus(Mat &frame, vector<Point2f> &imagePoints, vector<int> &indices, vector<Point> &polygon);

// Load Images and Videos from directory
// images - A vector of images
// videos - A vector of videos
// frameCounters - a list of frame counter, useful for keeping track if a video is running out of frames
int readImagesVideosFromDir(vector<Mat> &images,vector<VideoCapture> &videos, vector<int> &frameCounters);
#endif /* helper_hpp */
