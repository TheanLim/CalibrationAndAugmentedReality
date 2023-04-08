# CalibrationAndAugmentedReality

## Description
The goal of the project is to create a program that can detect a target and place virtual objects in the scene relative to the target, accounting for camera and target movement. The process involves calibrating the camera using a chessboard image to determine distance and then generating virtual objects on the chessboard. The virtual objects, including pyramids and a torus (donut), are projected onto the image plane in real time based on the movement of the camera or target. In addition to the chessboard, I have also used ArUco markers and overlayed images/videos on top of it.

## Demo

### Projecting a Virtual Object onto the scene
I created three slanted pyramids with the top connected with a green-colored roof.  
<img src="/images/Pyramid1.png" width="300" height="200">
<img src="/images/Pyramid2.png" width="300" height="200">
<img src="/images/Pyramid3.png" width="300" height="200">

### Projecting a Torus (Donut) onto the scene
<img src="/images/Torus.png" width="800" height="500">

### Overlaying Images on ArUco marks
I overlaid images onto those ArUco marks:  
<img src="/images/Aruco.png" width="300" height="200">
<img src="/images/ArucoOverlay.png" width="300" height="200">

I can even rotate the ArUco marks; the images will be rotated accordingly!  
<img src="/images/ArucoOverlayRotated.png" width="300" height="200">

### Overlaying Videos on ArUco marks
Please find a screen recording here: https://drive.google.com/file/d/1Jw_fpgG_UMm4sRolrb11tb1F5-4cMZAq/view?usp=sharing 

## Instructions
Run `augmentedReality.cpp`
You must also include a directory named "assets", where you keep several images and mp4 video clips. These will be overlaid onto ArUco markers. Download this folder from https://drive.google.com/drive/folders/18wfNYKXNvqpsX728QG43EM0cFL6ZNF04?usp=sharing  
In addition, you should start by calibrating your camera with a [chessboard](/markers/Chessboard.pdf). This can be done by pressing "d", and then "s" at least 5 times (each from diff angle of the chessboard).

Some useful hotkeys:
- n = Do nothing/ reset the camera frames
- s = Save the current frame for camera calibration. Need to press 'd' before pressing 's'. Need to save at least 5 images, and then calibration will happen by itself.
- q = Quit program
- d = Detect chessboard corners and display it
- p = Calculate the Camera's current position and print out the rotation and translation vectors.
- a = Overlay images onto ArUco markers found. Do nothing if no markers are found.
- v = Overlay videos onto ArUco markers found. Do nothing if no markers are found.
- o = Draw virtual objects (Three pyramids with the top connected with a "roof") onto the chessboard
- f = Find and display FAST features onto the frame
- h = Find and display Harris corners features onto the frame
- x = Display the 3D axes on the chessboard attached to the origin
- t = Draw a Torus, aka donut, onto the chessboard.

## OS and IDE
OS: MacOS Ventura 13.0.1 (22A400)

IDE: XCode


## Acknowledgements
- Calibration: https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html 
- Torus : https://www.vedantu.com/maths/torus 
- Harris Corner: https://docs.opencv.org/3.4/d4/d7d/tutorial_harris_detector.html 
- FAST Algorithm: https://blog.francium.tech/feature-detection-and-matching-with-opencv-5fd2394a590 
- ArUco:
  - https://docs.opencv.org/4.7.0/d5/dae/tutorial_aruco_detection.html 
  - https://www.youtube.com/watch?v=wB4BRWNuJM4&list=PLJ958Ls6nowUnzTXcdBBmO96NG5AWTq_N&index=4 
- https://stackoverflow.com/questions/67423382/overlay-filled-semi-transparent-contours/67426795#67426795 
