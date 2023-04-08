//
//  helper.cpp
//  Project4
//  This file contains helper functions for the main file
//  Created by Thean Cheat Lim on 3/20/23.
//

#include "helper.hpp"
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <cstring>

using namespace cv;
using namespace std;

// Do cornerSubPix and SolvePnP at once. Same args as opencv's 
int cornerSubPixSolvePnP (Mat &frame, Mat &rvec, Mat &tvec, vector<Vec3f>&worldPoints, vector<Point2f> &imagePoints, Mat &cameraMatrix, Mat &distortionCoefficients, int winSize){
    // Improve the detected corners' accuracy
    Mat grayScaleFrame;
    cvtColor(frame, grayScaleFrame, COLOR_BGR2GRAY);
    cornerSubPix(grayScaleFrame, imagePoints, Size(winSize, winSize), Size(-1, -1),
                 TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
    solvePnP(worldPoints, imagePoints, cameraMatrix, distortionCoefficients, rvec, tvec);

    return 0;
}

// Construct pyramids with a roof connecting them and store it at a vector of points
// objectPoints - A vector of points
int constructPyramidRoof(vector<Point3f> &objectPoints){
    // First Slanted Pyramid
    objectPoints.push_back(Point3f(0, 0, 0)); // origin
    objectPoints.push_back(Point3f(2, 0, 0)); // x-> right
    objectPoints.push_back(Point3f(0, -2, 0)); // y -> down
    objectPoints.push_back(Point3f(2, -1, 3)); // z -> diag on lower right
    objectPoints.push_back(Point3f(2, -2, 0)); // diag
    // Second Slanted Pyramid
    objectPoints.push_back(Point3f(3, -3, 0)); // origin
    objectPoints.push_back(Point3f(5, -3, 0)); // x-> right
    objectPoints.push_back(Point3f(3, -5, 0)); // y -> down
    objectPoints.push_back(Point3f(4, -3, 3)); // z -> diag on upper left
    objectPoints.push_back(Point3f(5, -5, 0)); // diag
    // Third Slanted Pyramid
    objectPoints.push_back(Point3f(6, 0, 0)); // origin
    objectPoints.push_back(Point3f(8, 0, 0)); // x-> right
    objectPoints.push_back(Point3f(6, -2, 0)); // y -> down
    objectPoints.push_back(Point3f(6, -1, 3)); // z -> diag on lower right
    objectPoints.push_back(Point3f(8, -2, 0)); // diag
    return 0;
}

// Draw pyramids with a roof connecting them onto frame, using imagePoints
// frame - Image frame
// imagePoints - a vector of 2D image plane points
int drawPyramidRoof(Mat &frame, vector<Point2f> &imagePoints){
    // Draw the axis on the image
    int thickness = 4;
    // First Slanted Pyramid
    Scalar firstColor =Scalar(235,206,135);
    line(frame, imagePoints[0], imagePoints[1], firstColor, thickness);
    line(frame, imagePoints[0], imagePoints[2], firstColor, thickness);
    line(frame, imagePoints[0], imagePoints[3], firstColor, thickness);
    line(frame, imagePoints[1], imagePoints[3], firstColor, thickness);
    line(frame, imagePoints[1], imagePoints[4], firstColor, thickness);
    line(frame, imagePoints[2], imagePoints[3], firstColor, thickness);
    line(frame, imagePoints[2], imagePoints[4], firstColor, thickness);
    line(frame, imagePoints[3], imagePoints[4], firstColor, thickness);
    // Second Slanted Pyramid
    Scalar secondColor = Scalar(114,128,250); // Salmon
    line(frame, imagePoints[5], imagePoints[6], secondColor, thickness);
    line(frame, imagePoints[5], imagePoints[7], secondColor, thickness);
    line(frame, imagePoints[5], imagePoints[8], secondColor, thickness);
    line(frame, imagePoints[6], imagePoints[8], secondColor, thickness);
    line(frame, imagePoints[6], imagePoints[9], secondColor, thickness);
    line(frame, imagePoints[7], imagePoints[8], secondColor, thickness);
    line(frame, imagePoints[7], imagePoints[9], secondColor, thickness);
    line(frame, imagePoints[8], imagePoints[9], secondColor, thickness);
    // Third Slanted Pyramid
    Scalar thirdColor =Scalar(140,180,210);
    line(frame, imagePoints[10], imagePoints[11], thirdColor, thickness);
    line(frame, imagePoints[10], imagePoints[12], thirdColor, thickness);
    line(frame, imagePoints[10], imagePoints[13], thirdColor, thickness);
    line(frame, imagePoints[11], imagePoints[13], thirdColor, thickness);
    line(frame, imagePoints[11], imagePoints[14], thirdColor, thickness);
    line(frame, imagePoints[12], imagePoints[13], thirdColor, thickness);
    line(frame, imagePoints[12], imagePoints[14], thirdColor, thickness);
    line(frame, imagePoints[13], imagePoints[14], thirdColor, thickness);
    // Connect the tips of pyramids
    vector<Point> trianglePoints;
    trianglePoints.push_back(imagePoints[3]);
    trianglePoints.push_back(imagePoints[8]);
    trianglePoints.push_back(imagePoints[13]);
    // Create a vector of vectors of points, where each vector represents a contour
    vector<vector<Point>> contours;
    contours.push_back(trianglePoints);
    // Transparent Top/ Roof
    /*https:stackoverflow.com/a/67426795/19481647*/
    // draw red filled contour on image background
    Mat roof;
    frame.copyTo(roof);
    drawContours(roof, contours, 0,  Scalar(0,128,0) , -1);
    //blend with original image
    double alpha = 0.5;
    addWeighted(frame, alpha, roof, 1-alpha, 0, frame);
    return 0;
}

// Construct a Torus and store it at a vector of points
// Also compute a vector of indices and polygon points, which are useful for drawing onto image plane after projection
// objectPoints -  A vector of points
// indices - indices; useful for drawing onto image plane after projecting objectPoints
// polygon - polygon; useful for drawing onto image plane after projecting objectPoints
int constructTorus(vector<Point3f> &objectPoints, vector<int> &indices, vector<Point> &polygon){
    int major_steps = 10;
    int minor_steps = 5;
    double major_radius = 1.0;
    double minor_radius = 0.75;
    Point3f translation(4, -3, 0);
    for (int i = 0; i < major_steps; i++) {
        double theta = 2 * CV_PI * i / major_steps;
        Point3f center(major_radius * cos(theta), major_radius * sin(theta), 0.0);
        for (int j = 0; j < minor_steps; j++) {
            double phi = 2 * CV_PI * j / minor_steps;
            double x = (major_radius + minor_radius * cos(phi)) * cos(theta);
            double y = (major_radius + minor_radius * cos(phi)) * sin(theta);
            double z = minor_radius * sin(phi);
            Point3f point(x, y, z);
            objectPoints.push_back(center + point + translation);
        }
    }
    // Define the indices of the points that make up the lines of the torus
    for (int i = 0; i < major_steps; i++) {
        int start_index = i * minor_steps;
        int end_index = ((i + 1) % major_steps) * minor_steps;
        for (int j = 0; j < minor_steps; j++) {
            int index1 = start_index + j;
            int index2 = end_index + j;
            int index3 = end_index + (j + 1) % minor_steps;
            int index4 = start_index + (j + 1) % minor_steps;
            indices.push_back(index1);
            indices.push_back(index2);
            indices.push_back(index3);
            indices.push_back(index4);
        }
    }
    for (auto index : indices) {
        Point3f point = objectPoints[index];
        Point2f image_point(point.x, point.y); // project to image plane
        polygon.push_back(image_point);
    }
    return 0;
}

// Draw a Torus using imagePoints
// frame - Image frame
// imagePoints - a vector of 2D image plane points
// indices - indices; useful for drawing onto image plane after projecting objectPoints
// polygon - polygon; useful for drawing onto image plane after projecting objectPoints
int drawTorus(Mat &frame, vector<Point2f> &imagePoints, vector<int> &indices, vector<Point> &polygon){
    vector<Point> contour;
    for (int i = 0; i < indices.size(); i += 4) {
        contour.clear();
        for (int j = 0; j <4; j++){
            contour.push_back(
                              Point(
                                    imagePoints[indices[i+j]].x,
                                    imagePoints[indices[i+j]].y)
                              );
        }
        const Point* pts[1] = {contour.data()};
        int npts[] = {(int)contour.size()};
        Mat torus;
        frame.copyTo(torus);
        fillPoly(torus, pts, npts, 1, Scalar(155, 181, 208));
        double alpha = 0.5;
        addWeighted(frame, 1-alpha, torus, alpha, 0, frame);
    }
    return 0;
}

// Load Images and Videos from directory
// images - A vector of images
// videos - A vector of videos
// frameCounters - a list of frame counter, useful for keeping track if a video is running out of frames
int readImagesVideosFromDir(vector<Mat> &images,vector<VideoCapture> &videos, vector<int> &frameCounters){
    char dirname[] = "assets";
    char buffer[256];
    DIR *dirp;
    struct dirent *dp;
    // open the directory
    dirp = opendir( dirname );
    if( dirp == NULL) {
      printf("Cannot open directory %s\n", dirname);
      exit(-1);
    }
    
    // loop over all the files in the image file listing
    while( (dp = readdir(dirp)) != NULL ) {
        // check if the file is an image
        if(
           strstr(dp->d_name, ".jpg") ||
           strstr(dp->d_name, ".png") ||
           strstr(dp->d_name, ".ppm") ||
           strstr(dp->d_name, ".tif")
           )
        {
            // build the overall filename
            strcpy(buffer, dirname);
            strcat(buffer, "/");
            strcat(buffer, dp->d_name);
            
            Mat img = imread(buffer, IMREAD_COLOR);
            images.push_back(img);
        }
        if(
           strstr(dp->d_name, ".mp4")
           ){
               // build the overall filename
               strcpy(buffer, dirname);
               strcat(buffer, "/");
               strcat(buffer, dp->d_name);
               
               VideoCapture vid = VideoCapture(buffer);
               videos.push_back(vid);
               frameCounters.push_back(0);
        }
    }
    return 0;
}

