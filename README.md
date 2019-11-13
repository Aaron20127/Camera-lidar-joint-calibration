## About ##

This is a sample method to calibrate camera and lidar jointly. I think this method is suitable for 16, 32, 64 line-lidar. If you want to run this code successfully, please read the discription carefully. 

## Steps ##

### 1. Required software ###

 - PolyWorks 2019.  
 - Matlab 2019a.

### 2. Procedure ###

Doawload this code and perform the following steps.

* **Base requirements**
1. All images must be undistorted. This matter is of the utmost importance.
2. Calibration board requires black and white checkerboard lattice and one side is odd, one side is even.
3. The degree of corner between y-axis of image and the long side of chessbooard should less than 45, and they shouldn't be completely parallel. Pose of calibration board in image must look like second row shown below (first row is wrong pose):
.<div align=center><img src="https://github.com/Aaron20127/Camera-lidar-joint-calibration/blob/master/readme/chessboard.jpg" width="350" height="350" /></div>
4. The degree of corner between z-axis of lidar and the long side of chessbooard should less than 45, and they shouldn't be completely parallel. for example:
.<div align=center><img src="https://github.com/Aaron20127/Camera-lidar-joint-calibration/blob/master/readme/lidar.jpg" width="350" height="350" /></div>

* **Data Storage**

1. Put your camera images (.png) into `data/images` and you have to name it numerically.
2. Put your lidar data (.txt) into `data/pointcloud` and each line of the file has only xyz coordinates, for example:
```
    0.5363237262 -0.3014609218 -0.1039963961
    0.5608119369 -0.3181324303 -0.1093295515
    0.5810572505 -0.3322938681 -0.1137738377
    0.6180613041 -0.3567755520 -0.1217735633
    ...
```

* **Fit the chessboad in the point cloud**

1. open PolyWorks2019 -> tool -> PolyWorks|inspector.
![](readme/fit_chessboard_1.jpg)
2. file -> input -> pointcloud, choose .txt file
3. only choose 'millimeter' and 'Space', click ok. 
![](readme/fit_chessboard_2.jpg)
4. choose -> unit -> interaction
![](readme/fit_chessboard_3.jpg)
5. choose background points, press delete key to delete these points, only save chessboard points.
![](readme/fit_chessboard_4.jpg)
6. measure -> feature -> create
![](readme/fit_chessboard_5.jpg)
7. choose square
![](readme/fit_chessboard_6.jpg)
8. choose fitting
![](readme/fit_chessboard_7.jpg)
9. modify parameter
![](readme/fit_chessboard_8.jpg)
10. choose max
![](readme/fit_chessboard_9.jpg)
11. choose -> unit -> interaction, choose all the points
![](readme/fit_chessboard_10.jpg)
![](readme/fit_chessboard_11.jpg)
12. creation
![](readme/fit_chessboard_12.jpg)
13. square -> output
![](readme/fit_chessboard_13.jpg)
14. Save '.igs' file to `data/chessboard_pointcloud_igs`. Note that the prefix name must be a number and its prefix name must match the prefix name of the image in `data/images`.
![](readme/fit_chessboard_14.jpg)

15. Repeat the above steps until all pointcloud of chessboard are fitted.

* **Configuration**

1. Open matlab2019 and change the matlab folder to the root directory `Camera-lidar-joint-calibration/`.
2. Open `joint_calibration.m` file and you need to modify the following options.
```
    x_grids = 5; % number of grids of short edges of chessboard
    y_grids = 8; % number of grids of long edges of chessboard

    imageType = 'png'; % image format

    % camera parameters
    focalLength    = [2525.9, 2528.1]; % fx, fy
    principalPoint = [942.9102, 584.8342]; % cx, cy
    imageSize = [1080, 1920]; % image size
```
![](readme/configuration_1.jpg)

* **Check the result of image corner detection**

1. If you run this code for the first time, you should check the result of image corner detection carefully.  If the corners of some images are not detected successfully, the `MinCornerMetric` parameter needs to be adjusted . So you should modify the configuration as below. 
```
    MinCornerMetric = 0.4; % Adjusting this parameter can better detect corner points
    onlyShowDetection = 'true';% only show result of image corner detection
```
![](readme/corner_detection_1.jpg)

2. Run `joint_calibration.m` to check the result. Green represents the first detected corner.
![](readme/corner_detection_2.jpg)

* **Joint calibration**

1. If corner detection is successful, you should modify the cofiguration to `onlyShowDetection = 'false'` to start calibrating jointly.

2. Reprojection error will be shown in images and the cross symbol represents the reprojection point.
![](readme/joint_calibration_1.jpg)

3. All the pointcloud will be prejected into corresponding image.
![](readme/joint_calibration_2.jpg)

4. Calibration result `R` and `T` are in the command window.
![](readme/joint_calibration_3.jpg)

* **Notes**

1. The Projection formula from point cloud to image looks like this
```
    Pc = K * R * (Pw - T)
```

2. For better results, you shold use at least 20 pairs pointcloud and images. And the calibration board should cover all positions of the images as much as possible.
