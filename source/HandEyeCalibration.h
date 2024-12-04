#pragma once
#include <string>
#include <array>
#include <vector>

using std::string;
using std::array;
using std::vector;

class HandEyeCalibration
{
public:

	struct ImageDataInfo2D
	{
		int width;
		int height;
		int channel;
		unsigned char* image_data;

		ImageDataInfo2D()
		{
			width = 0;
			height = 0;
			channel = 0;
			image_data = nullptr;
		}
	};

	struct RobotPose
	{
		double x;
		double y;
		double z;
		double rx;
		double ry;
		double rz;

		RobotPose()
		{
			x = 0;
			y = 0;
			z = 0;
			rx = 0;
			ry = 0;
			rz = 0;
		}
	};

	struct CalibrateData
	{
		ImageDataInfo2D img;
		RobotPose pose;
	};

	enum class RotationType
	{
		EULER_ANGLE = 0,
		AXIS_ANGLE = 1,
	};

	enum class HandEyeType
	{
		EYE_IN_HAND = 0,
		EYE_TO_HAND = 1,
	};

	enum class CalibrateErrorType
	{
		MAX_DIFF_ERROR  = 0,
		MEAN_DIFF_ERROR = 1,
		RMSE            = 2,
	};

	HandEyeCalibration();

	~HandEyeCalibration();
	
	//读取位姿和标定板图像,需一一对应,至少3组
	//isDegree为true表示角度,false表示弧度
	bool readPose(string poseList, bool isDegree = true);
	bool readImage(const vector<string>& imgFiles_);

	//读取标定数据(图像和位姿)
	//isGray为true表示输入图像为灰度图,false表示rgb图
	//isDegree为true表示角度,false表示弧度
	bool readCalibrateData(const std::vector<CalibrateData>& calibDatas, bool isDegree = true);

	//设置旋转方式
	void setRotationType(RotationType type);

	//设置棋盘格参数
	void setChessboardParams(array<int, 2> patternSize_, double squareSize_);

	//设置相机内参和畸变系数,recalibration=true时重新进行相机内参标定
	void setCameraIntrinsicAndDist(array<array<double, 3>, 3> cameraIntrinsic_, array<double, 5> distCoeffs_, bool recalibration_ = false);

	//读取相机内参
	bool ReadCameraIntrinsics(const char* intrinsicsPath, bool recalibrate = false);

	//进行手眼标定
	bool calibrate(HandEyeType handEyeType = HandEyeType::EYE_IN_HAND, bool useOptimize = false);

	//获取相机内参
	array<array<double, 3>, 3> GetCameraIntrinsic() const;

	//获取畸变系数
	array<double, 5> GetDistCoeffs() const;

	//获取手眼矩阵
	//眼在手上时为相机到末端坐标系的变换矩阵
	//眼在手外时为相机到基坐标系的变换矩阵
	array<array<double, 4>, 4> GetHandEyeMatrix() const;

	//保存手眼矩阵
	//文件后缀支持ini和dat
	bool SaveHandEyeMatrix(const std::string& filename) const;

	//获取相机标定的重投影误差
	double GetReprojectionError() const;

	//获取标定误差
	array<double, 6> GetCalibrateError(CalibrateErrorType errorType = CalibrateErrorType::MAX_DIFF_ERROR) const;

	//获取有效棋盘格图像索引
	const vector<int>& GetValidImgIndex() const;

	//保存转换后的棋盘格点云
	bool savePointCloudTransformed(string outputDir) const;

	//获取执行失败消息
	string GetErrorMessage() const;

private:
	vector<array<double, 6>> poses;//机械臂末端位姿x,y,z,rx,ry,rz
	RotationType rotationType;//旋转方式
	vector<ImageDataInfo2D> imgs;//标定板图像
	array<int, 2> patternSize;//棋盘格行列角点数
	double squareSize;//棋盘格方块大小mm
	array<array<double, 3>, 3> cameraIntrinsic;//相机内参矩阵
	array<double, 5> distCoeffs;//畸变系数
	bool recalibration;//是否重新进行相机标定
	array<array<double, 4>, 4> Hcg;//手眼转换矩阵
	array<double, 6> rms;//x,y,z,rx,ry,rz的rms误差
	double meanReprojectionError;//重投影误差
	vector<int> validImgIndex;//有效棋盘格图像索引
	vector<array<array<double, 4>, 4>> gripperPoseMatrix;//末端位姿矩阵
	vector<array<array<double, 4>, 4>> cameraPoseMatrix;//末端位姿矩阵
	array<array<double, 6>, 3> handEyeCalibrateError;//手眼标定的rms误差(x,y,z,rx,ry,rz)

	vector<vector<array<double, 3>>> cornersTransformed;
	vector<array<double, 3>> rotateAngleTransformed;

	string errorMsg;

	void Clear();
};

typedef HandEyeCalibration::RotationType RotationType;
typedef HandEyeCalibration::HandEyeType HandEyeType;
typedef HandEyeCalibration::CalibrateErrorType CalibrateErrorType;