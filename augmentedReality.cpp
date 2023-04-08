//
//  augmentedReality.cpp
//  Project4
//  These codes are about calibrating camera to obtain intrinsic parameters, projecting objects into the scene
//  and overlaying images and videos onto ArUco markers.
//  Created by Thean Cheat Lim on 3/16/23.
//

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "helper.hpp"

using namespace cv;
using namespace std;

int MIN_CALIBRATION_CNT = 5;
char CAM_DISTORTION_YML [] = "cameraDistortion.yml"; // file to store camera intrinsic params

int main(int argc, char *argv[]) {
    VideoCapture *capdev;
    
    // open the video device
    capdev = new VideoCapture(0);
    if( !capdev->isOpened() ) {
            printf("Unable to open video device\n");
            return(-1);
    }

    // get some properties of the image
    Size refS((int) capdev->get(CAP_PROP_FRAME_WIDTH ),
              (int) capdev->get(CAP_PROP_FRAME_HEIGHT));
    int fps = capdev->get(CAP_PROP_FPS);
    printf("Expected size: %d %d\n", refS.width, refS.height);
    cout << "Frames per second :" << fps;

    namedWindow("Video", 1); // identifies a window
    Mat frame;
    char persistKey = 'n';
    
    // Read in Camera Matrix and Distortion Coeff if exists
    Mat cameraMatrix, distortionCoefficients;
    FileStorage camDistortionYml(CAM_DISTORTION_YML, FileStorage::READ);
    if(camDistortionYml.isOpened()){
        camDistortionYml["camera_matrix"] >> cameraMatrix;
        camDistortionYml["distortion_coefficients"] >> distortionCoefficients;
    }
    camDistortionYml.release();
    
    // Define the number of inner corners of the chessboard
    int boardWidth = 9;
    int boardHeight = 6;
    Size boardSize(boardWidth, boardHeight);
    // Task 2: Select Calibration Images
    vector<Point2f> calibrationCorners;
    Mat calibrationFrame;
    vector<vector<Vec3f>> pointList;
    vector<vector<Point2f>> cornerList;
    // Fixed Point set
    vector<Vec3f> chessboardFixedPointSet;
    for (int j = 0; j < boardHeight; j++){
        for (int i = 0; i<boardWidth; i++){
            Vec3f temp(i, -j, 0);
            chessboardFixedPointSet.push_back(temp);
        }
    }
    
    // Extension Assets -- videos and images
    vector<Mat> images;
    vector<VideoCapture> videos;
    bool readImagesVideos = false;
    Mat vidFrame;
    vector<int>frameCounters;
    
    // Loop forever to read in video frame
    for(;;) {
        *capdev >> frame; // get a new frame from the camera, treat as a stream
        if(frame.empty() ) {
            printf("frame is empty\n");
            break;
        }
        
        if(persistKey =='d'){
            // Task 1: Detect and extract Chessboard corners
            // Create an empty vector of Point2f to store the detected corners
            vector<Point2f> corners;
            // Find the chessboard corners
            bool found = findChessboardCorners(frame, boardSize, corners);
            if (found){
                // Improve the detected corners' accuracy
                Mat grayScaleFrame;
                cvtColor(frame, grayScaleFrame, COLOR_BGR2GRAY);
                int winSize = 11; // Default
                cornerSubPix(grayScaleFrame, corners, Size(winSize, winSize), Size(-1, -1),
                             TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
                
                // Draw the detected corners on the image
                drawChessboardCorners(frame, boardSize, corners, found);
                
                // Print out the number of detected corners and the coordinates of the first corner
                printf("Number of corners detected: %lu\n", corners.size());
                printf("Coordinates of the first corner: (%f, %f)\n", corners[0].x, corners[0].y);
                
                // Make a copy
                calibrationCorners = corners;
                frame.copyTo(calibrationFrame);
            } else {
                printf("No chessboard is found\n");
            }
        }
        
        if(persistKey =='p'){
            // Task 4: Calculate Current Position of the Camera
            if (cameraMatrix.empty()) printf("Press d and then s to calibrate the camera first.");
            else {
                vector<Point2f> corners;
                bool found = findChessboardCorners(frame, boardSize, corners);
                if (found){
                    Mat rvec, tvec;
                    cornerSubPixSolvePnP(frame, rvec, tvec, chessboardFixedPointSet, corners, cameraMatrix, distortionCoefficients);
                    
                    // Print rotation and translation data
                    cout << "rotation vector: " << rvec << endl;
                    cout << "translation vector: " << tvec << endl;
                }
            }
        }
        
        if(persistKey =='x'){
            // Task 5: Project Outside Corners or 3D Axes
            if (cameraMatrix.empty()) printf("Press d and then s to calibrate the camera first.");
            else {
                vector<Point2f> corners;
                bool found = findChessboardCorners(frame, boardSize, corners);
                if (found){
                    Mat rvec, tvec;
                    cornerSubPixSolvePnP(frame, rvec, tvec, chessboardFixedPointSet, corners, cameraMatrix, distortionCoefficients);
                    
                    vector<Point3f> objectPoints;
                    objectPoints.push_back(Point3f(0, 0, 0)); // origin
                    objectPoints.push_back(Point3f(1, 0, 0)); // x
                    objectPoints.push_back(Point3f(0, -1, 0)); // y
                    objectPoints.push_back(Point3f(0, 0, 1)); // z
                    
                    vector<Point2f> imagePoints;
                    projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);
                    
                    // Draw the axis on the image
                    arrowedLine(frame, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 2);
                    arrowedLine(frame, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 2);
                    arrowedLine(frame, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 2);
                }
            }
        }
        
        if(persistKey =='o'){
            // Task 6: Create a Virtual Object
            if (cameraMatrix.empty()) printf("Press d and then s to calibrate the camera first.");
            else {
                vector<Point2f> corners;
                bool found = findChessboardCorners(frame, boardSize, corners);
                if (found){
                    Mat rvec, tvec;
                    cornerSubPixSolvePnP(frame, rvec, tvec, chessboardFixedPointSet, corners, cameraMatrix, distortionCoefficients);
                    
                    vector<Point3f> objectPoints;
                    vector<Point2f> imagePoints;
                    constructPyramidRoof(objectPoints);
                    projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);
                    drawPyramidRoof(frame, imagePoints);
                }
            }
        }
        
        if(persistKey =='t'){
            // Extension: - Torus on chessboard
            if (cameraMatrix.empty()) printf("Press d and then s to calibrate the camera first.");
            else {
                vector<Point2f> corners;
                bool found = findChessboardCorners(frame, boardSize, corners);
                if (found){
                    Mat rvec, tvec;
                    cornerSubPixSolvePnP(frame, rvec, tvec, chessboardFixedPointSet, corners, cameraMatrix, distortionCoefficients);
                    
                    vector<Point3f> objectPoints;
                    vector<Point2f> imagePoints;
                    vector<int> indices;
                    vector<Point> polygon;
                    constructTorus(objectPoints, indices, polygon);
                    projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);
                    drawTorus(frame, imagePoints, indices, polygon);
                }
            }
        }
        
        if(persistKey =='h'){ // Harris corner
            // Task 7: Detect Robust Features - Harris Corner
            /*https:docs.opencv.org/3.4/d4/d7d/tutorial_harris_detector.html*/
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            
            Mat corners, cornersScaled;
            int blockSize = 3;
            int apertureSize = 3;
            double k = 0.06;
            double threshold = 128;
            cornerHarris(gray, corners, blockSize, apertureSize, k);
            normalize(corners, corners, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
            
            // Draw circles at the corner
            Scalar color = Scalar(114,128,250);
            int radius = 5;
            int thickness = 5;
            for( int i = 0; i < corners.rows; i++){
                float *uptr = corners.ptr<float>(i);
                for( int j = 0; j < corners.cols; j++){
                    if((int)uptr[j]>threshold){
                        circle(frame, Point(j,i), radius, color, thickness);
                    }
                }
            }
        }
        
        if(persistKey =='f'){ // FAST algorithm
            // Task 7 and Extension1: Detect Robust Features - Harris Corner
            /*https:blog.francium.tech/feature-detection-and-matching-with-opencv-5fd2394a590*/
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            
            // Create FAST feature detector and compute keypoints
            Ptr<FeatureDetector> detector = FastFeatureDetector::create(50);
            vector<KeyPoint> keypoints;
            detector->detect(gray, keypoints);

            // Draw keypoints on the image
            Scalar color = Scalar(114,128,250);
            drawKeypoints(frame, keypoints, frame, color);
        }
        
        if(persistKey =='v' or persistKey =='a'){
            // Extension 3: ArUco + display video/images
            /* https:docs.opencv.org/4.7.0/d5/dae/tutorial_aruco_detection.html*/
            /* https:www.youtube.com/watch?v=wB4BRWNuJM4&list=PLJ958Ls6nowUnzTXcdBBmO96NG5AWTq_N&index=4*/
            if (not readImagesVideos) {
                readImagesVideosFromDir(images, videos, frameCounters);
                readImagesVideos = true;
            }
            vector<int> markerIds;
            vector<vector<Point2f>> markerCorners, rejectedCandidates;
            aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
            aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
            aruco::ArucoDetector detector(dictionary, detectorParams);
            detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);
            
            if (persistKey =='v' and markerIds.size() == 0){
                // Reset video frame
                frameCounters.clear();
                for (int i = 0; i<videos.size(); i++){
                    videos[i].set(CAP_PROP_POS_FRAMES, 0);
                    frameCounters.push_back(0);
                }
            } else {
                for (int i = 0; i<markerIds.size(); i++){
                    Mat curImg;
                    int index=0;
                    if (persistKey =='v') {
                        if (markerIds[i]>videos.size()){
                            cout<<markerIds[i]<<" "<<videos.size()<<endl;
                            continue;  //ignore this box ->no overlay
                        }
                        index = markerIds[i]-1;
                        VideoCapture vid = videos[index];
                        if (frameCounters[i]==vid.get(CAP_PROP_FRAME_COUNT)){
                            // Reset video frame
                            vid.set(CAP_PROP_POS_FRAMES, 0);
                            frameCounters[i] = 0;
                        }
                        vid.read(curImg);
                    } else if (persistKey=='a'){
                        index = markerIds[i]%images.size();
                        curImg = images[index];
                    }
                    if (curImg.rows==0) continue;
                    int rows = curImg.rows;
                    int cols = curImg.cols;
                    vector<Point2f> srcPoints;
                    srcPoints.push_back(Point2f(0, 0));
                    srcPoints.push_back(Point2f(cols, 0));
                    srcPoints.push_back(Point2f(cols, rows));
                    srcPoints.push_back(Point2f(0, rows));
                    
                    Mat H = findHomography(srcPoints, markerCorners.at(i));
                    Mat warpedImage;
                    warpPerspective(curImg, warpedImage, H, frame.size());
                    Mat mask = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
                    
                    vector<Point2f> thisCorner2F = markerCorners.at(i);
                    vector<Point> thisCornerCasted;
                    for (size_t i = 0; i < thisCorner2F.size(); i++) {
                        thisCornerCasted.push_back(Point((int)thisCorner2F[i].x, (int)thisCorner2F[i].y));
                    }
                    fillConvexPoly(mask, thisCornerCasted, 255);
                    bitwise_and(warpedImage,warpedImage,frame, mask = mask);
                    if (persistKey=='v')frameCounters[index]++;
                }
            }
        }
        
        imshow("Video", frame);
        
        // see if there is a waiting keystroke
        char key = waitKey(10);
        if( key == 'q') break;
        
        if(key =='s'){
            if (calibrationCorners.empty()) printf("No chessboard corners were detected. Press 'd' first.\n");
            else {
                // Task 2: Select Calibration Images
                // Save corners and real word point sets
                cornerList.push_back(calibrationCorners);
                pointList.push_back(chessboardFixedPointSet);
                // Save the calibration image
                string calibrationFrameFn = "calibration" + std::to_string(cornerList.size()) + ".png";
                imwrite(calibrationFrameFn, calibrationFrame);
                
                if (cornerList.size()>=MIN_CALIBRATION_CNT){
                    // Task 3: Calibrate the Camera
                    // Initialize camera_matrix
                    cameraMatrix = Mat::eye(3, 3, CV_64F);
                    cameraMatrix.at<double>(0, 2) = frame.cols/2.0;
                    cameraMatrix.at<double>(1, 2) = frame.rows/2.0;
                    // distortion coefficient matrix. Initialize with zero
                    distortionCoefficients = Mat::zeros(8, 1, CV_64F);
                    vector<Mat> rvecs, tvecs;
                    
                    printf("Camera Matrix and Distortion coefficients BEFORE calibration: \n");
                    cout << cameraMatrix << endl;
                    cout << distortionCoefficients << endl;
                    
                    double reprojection_error = calibrateCamera(pointList, cornerList, frame.size(), cameraMatrix, distortionCoefficients, rvecs, tvecs, CALIB_FIX_ASPECT_RATIO);
                    
                    printf("Camera Matrix and Distortion coefficients AFTER calibration: \n");
                    cout << cameraMatrix << endl;
                    cout << distortionCoefficients << endl;
                    printf("Reprojection Error: %f\n", reprojection_error);
                    
                    // Save Camera Matrix and Distortion coefficients
                    FileStorage cameraDistortion(CAM_DISTORTION_YML, FileStorage::WRITE);
                    cameraDistortion << "camera_matrix" << cameraMatrix;
                    cameraDistortion << "distortion_coefficients" << distortionCoefficients;
                    cameraDistortion.release();
                }
            }
            // back to previous key
            key = persistKey;
        }
        // Persist key
        if (strchr("nsqdpaofhxvt", key)){ /*https:stackoverflow.com/a/19548575/19481647*/
            persistKey = key;
        }
    }
    
    delete capdev;
    return(0);
}
