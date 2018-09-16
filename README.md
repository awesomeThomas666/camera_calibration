# camera_calibration
# 相机标定模块使用说明
## 1.在DASH中输入cheese打开相机，从各个角度拍摄标定棋盘10~20张，一定要变换相机姿态，减少误差。
## 2.将ccheese拍摄到的图片拷贝到camera_calibration目录下，注意不要出现（*.jpg命名，其中*为自然数字，否则图片有可能被覆盖）。
## 3.打开终端，执行./rename.sh讲图片名统一编号为自然数序列。
## 4.在终端执行./camera_calibration boardWidth boardHeight squareSize frameNumber
### boardWidth :棋盘横向角点数目 如：9
### boardHeight :棋盘纵向角点数目 如：6
### squareSize ：棋盘中每个格子（要求是正方形）的实际边长，单位：mm 如：25
### frameNumber：要计算的图片数量 如：17
