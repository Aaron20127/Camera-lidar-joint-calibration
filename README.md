## About ##

This is a sample method to calibrate camera and lidar jointly. I think this method is suitable for 16, 32, 64 line-lidar. If you want to run this code successfully, please read the discription carefully. This idear doesn't all come from me and its origin is in this link https://blog.csdn.net/qq_29462849/article/details/88748028#comments.

## Steps ##

### 1. Required software ###

 - PolyWorks 2019.  
 - Matlab 2019a.

### 2. Procedure ###

Doawload this code and perform the following steps.

* **Calibration board requirements**

1. Calibration board requires black and white checkerboard lattice and one side is odd, one side is even.
2. Pose of calibration board in image must look like the one shown below (first row is wrong pose):
.<div align=center><img src="https://github.com/Aaron20127/Camera-lidar-joint-calibration/blob/master/chessboard.jpg" width="450" height="450" /></div>

* **Data Storage**

1.Put your camera images (.png) into `data/images` and you have to name it numerically.
2.Put your lia






