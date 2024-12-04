#pragma once
#define HANDEYECALIBRATIONDLL extern "C" __declspec(dllexport)

#include "HandEyeCalibrationParam.h"

// 读取标定数据(图像和位姿)
// dataNum 表示数据个数，至少为3
// isGray = true 表示输入图像为灰度图, false表示rgb图
// isDegree = true 表示机械臂末端旋转角度单位为度, false表示弧度
// rotationType = 0 表示旋转方式为欧拉角, rotationType = 1 表示旋转方式为轴角
HANDEYECALIBRATIONDLL bool setCalibrateData(const CalibrateData* calibDatas, int dataNum, bool isDegree, int rotationType);

// 设置棋盘格参数(行列角点数、方格尺寸)
HANDEYECALIBRATIONDLL void setChessboardParams(const ChessboardParam* chessboardParam);

// 读取相机内参文件(yml格式)
// recalibrate: 是否重新标定相机内参
HANDEYECALIBRATIONDLL bool readCameraIntrinsics(const char* intrinsicsPath, bool recalibrate);

// 执行手眼标定
// type = 0 为眼在手上, type = 1 为眼在手外
// optimize = true 开启优化, optimize = false 关闭优化
HANDEYECALIBRATIONDLL bool calibrateHandEye(int type, bool optimize);

// 获取相机内参
HANDEYECALIBRATIONDLL void getCameraIntrinsic(float* cameraIntrinsics);

// 保存手眼标定矩阵至文件
// 文件后缀支持 ini 或 dat
HANDEYECALIBRATIONDLL bool saveHandEyeMatrix(const char* savePath);

// 获取手眼矩阵结果(数组长度为 16)
HANDEYECALIBRATIONDLL void getHandEyeMatrix(float* handEyeMatrix);

// 获取标定误差(数组长度为 6)
// 分别表示x, y, z, rx, ry, rz 的标定误差
// errorType = 0 为最大误差, errorType = 1 为平均误差
HANDEYECALIBRATIONDLL bool getCalibrateError(float* error, int errorType);