#include "HandEyeCalibrationSDK.h"
#include <opencv2/opencv.hpp>
#include <string>

#if 0
const int dataNum = 15;

// 图像文件
std::string imgPath[dataNum] =
{
	"calib_001.bmp",
	"calib_002.bmp",
	"calib_003.bmp",
	"calib_004.bmp",
	"calib_005.bmp",
	"calib_006.bmp",
	"calib_007.bmp",
	"calib_008.bmp",
	"calib_009.bmp",
	"calib_010.bmp",
	"calib_011.bmp",
	"calib_012.bmp",
	"calib_013.bmp",
	"calib_014.bmp",
	"calib_015.bmp",
};

// 机械臂末端位姿x y z rx ry rz
std::vector<std::vector<double>> RobotPoses =
{
	{-902.265,100.337,94.905,4.182,-12.311,-3.993},
	{-889.446,13.769,67.790,30.839,-17.665,12.509},
	{-872.795,197.700,75.552,-26.329,-15.693,-23.285},
	{-935.888,171.601,71.698,-15.555,-29.241,-10.966},
	{-946.649,49.826,101.566,20.994,-29.950,-1.122},
	{-941.544,-68.482,127.709,28.434,-30.748,0.715},
	{-938.929,100.423,128.090,-23.692,-34.136,-10.909},
	{-991.195,-55.314,128.095,-23.693,-34.135,-10.909},
	{-937.058,-257.980,168.896,6.609,-1.166,-10.076},
	{-932.491,40.050,163.068,-29.391,-15.867,-26.655},
	{-1048.663,-8.046,162.189,-5.329,-33.216,-19.337},
	{-906.036,33.733,166.518,-33.685,-7.093,-36.013},
	{-906.038,33.728,26.244,-37.490,-17.496,20.103},
	{-914.218,-263.815,60.806,28.764,-23.377,-6.893},
	{-851.293,124.512,60.818,-1.439,-12.039,5.534},
};

int main()
{
	// 1. 读取图像和对应机械臂末端位姿
	CalibrateData* calibDatas = new CalibrateData[dataNum];
	const std::string folder = "../sample/SampleData/";
	for (int i = 0; i < dataNum; ++i)
	{
		cv::Mat img = cv::imread(folder + imgPath[i], -1);

		calibDatas[i].img.width = img.cols;
		calibDatas[i].img.height = img.rows;
		calibDatas[i].img.channel = img.channels();
		size_t size = img.total() * img.elemSize();
		calibDatas[i].img.image_data = new unsigned char[size];
		memcpy(calibDatas[i].img.image_data, img.data, size);

		calibDatas[i].pose.x = RobotPoses[i][0];
		calibDatas[i].pose.y = RobotPoses[i][1];
		calibDatas[i].pose.z = RobotPoses[i][2];
		calibDatas[i].pose.rx = RobotPoses[i][3];
		calibDatas[i].pose.ry = RobotPoses[i][4];
		calibDatas[i].pose.rz = RobotPoses[i][5];
	}
	bool success = setCalibrateData(calibDatas, dataNum, true, 0);

	// 2. 设置棋盘格参数
	ChessboardParam* chessBoardParam = new ChessboardParam;
	chessBoardParam->cornersHorizontal = 11;
	chessBoardParam->cornersVertical = 8;
	chessBoardParam->squareSize = 20;
	setChessboardParams(chessBoardParam);

	// 3. 读取相机内参
	std::string filePath = folder + "intrinsics.yml";
	success = readCameraIntrinsics(filePath.c_str(), false);

	// 4. 执行标定
	success = calibrateHandEye(1, true);

	// 5. 获取标定矩阵
	float* handEyeMatrix = new float[16];
	getHandEyeMatrix(handEyeMatrix);

	// 6. 保存标定文件
	filePath = "handEye.ini";
	success = saveHandEyeMatrix(filePath.c_str());

	// 7. 获取标定误差平均值
	float* err = new float[6];
	success = getCalibrateError(err, 0);

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			std::cout << handEyeMatrix[j + 4 * i] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	for (int i = 0; i < 6; ++i)
	{
		std::cout << err[i] << " ";
	}
	std::cout << std::endl;

	return 0;
}
#else

// 图像文件
std::vector<std::string> imgPath =
{
	"snapshot_001.bmp",
	//"snapshot_002.bmp",
	//"snapshot_003.bmp",
	//"snapshot_004.bmp",
	"snapshot_005.bmp",
	"snapshot_006.bmp",
	/*"snapshot_007.bmp",
	"snapshot_008.bmp",
	"snapshot_009.bmp",
	"snapshot_010.bmp",
	"snapshot_011.bmp",
	"snapshot_012.bmp",
	"snapshot_013.bmp",
	"snapshot_014.bmp",
	"snapshot_015.bmp",
	"snapshot_016.bmp",
	"snapshot_017.bmp",
	"snapshot_018.bmp",
	"snapshot_019.bmp",
	"snapshot_020.bmp",
	"snapshot_021.bmp",
	"snapshot_022.bmp",
	"snapshot_023.bmp",
	"snapshot_024.bmp",*/
};

// 机械臂末端位姿x y z rx ry rz
std::vector<std::vector<double>> RobotPoses =
{
	{301.29,126.70,463.27,-176.84,-0.82,172.29 },
	//{292.20,122.43,464.33,-174.28,-8.08,-177.31 },
	//{335.87,225.07,422.69,-177.56,-3.04,174.43},
	//{336.27,221.23,422.90,-178.36,-5.51,170.94},
	{335.13,217.04,423.19,-177.21,-7.00,-171.88},
	{349.96,139.58,400.04,-177.82,-5.57,176.98},
	/*{366.13,77.73,400.41,179.72,-8.16,175.81},
	{360.59,68.42,394.38,-175.50,-7.19,-177.89},
	{357.13,67.50,394.97,-175.96,-10.21,-171.87},
	{356.48,67.32,395.10,-173.23,-9.26,172.78},
	{351.59,73.89,396.23,-170.93,-11.97,-179.07},
	{275.70,125.10,392.48,-167.12,-10.21,176.86},
	{243.54,112.59,364.25,-170.20,-8.19,175.18},
	{192.11,200.25,369.39,-174.42,-0.39,-178.11},
	{186.92,204.73,369.27,-177.23,-3.65,174.02},
	{184.32,199.80,369.85,-173.65,-5.34,175.48},
	{213.05,164.50,350.01,-170.67,-9.39,171.36},
	{209.99,164.54,334.20,-170.60,-8.53,176.58},
	{222.66,173.10,500.93,-175.46,-1.71,176.25},
	{222.66,173.10,500.93,-176.04,-2.82,-168.95},
	{200.10,154.69,518.49,-166.70,-11.28,176.41},
	{243.47,89.61,520.11,-172.45,-6.69,166.12},
	{194.09,71.07,538.02,175.92,14.99,177.44},
	{206.44,-10.16,538.02,-178.83,15.48,174.33},*/
};

int main()
{
	// 1. 读取图像和对应机械臂末端位姿
	const int dataNum = imgPath.size();
	CalibrateData* calibDatas = new CalibrateData[dataNum];
	const std::string folder = "C:/Users/weim172122/Desktop/calibrate_data/";
	for (int i = 0; i < dataNum; ++i)
	{
		cv::Mat img = cv::imread(folder + "image/" + imgPath[i], -1);

		calibDatas[i].img.width = img.cols;
		calibDatas[i].img.height = img.rows;
		calibDatas[i].img.channel = img.channels();
		size_t size = img.total() * img.elemSize();
		calibDatas[i].img.image_data = new unsigned char[size];
		memcpy(calibDatas[i].img.image_data, img.data, size);

		calibDatas[i].pose.x = RobotPoses[i][0];
		calibDatas[i].pose.y = RobotPoses[i][1];
		calibDatas[i].pose.z = RobotPoses[i][2];
		calibDatas[i].pose.rx = RobotPoses[i][3];
		calibDatas[i].pose.ry = RobotPoses[i][4];
		calibDatas[i].pose.rz = RobotPoses[i][5];
	}
	bool success = setCalibrateData(calibDatas, dataNum, true, 0);

	// 2. 设置棋盘格参数
	ChessboardParam* chessBoardParam = new ChessboardParam;
	chessBoardParam->cornersHorizontal = 11;
	chessBoardParam->cornersVertical = 8;
	chessBoardParam->squareSize = 15;
	setChessboardParams(chessBoardParam);

	// 3. 读取相机内参
	std::string filePath = folder + "cameraIntrinsic.yml";
	success = readCameraIntrinsics(filePath.c_str(), false);

	// 4. 执行标定
	success = calibrateHandEye(0, true);

	// 5. 获取标定矩阵
	float* handEyeMatrix = new float[16];
	getHandEyeMatrix(handEyeMatrix);

	// 6. 保存标定文件
	filePath = "handEye.ini";
	success = saveHandEyeMatrix(filePath.c_str());

	// 7. 获取标定误差平均值
	float* err = new float[6];
	success = getCalibrateError(err, 1);

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			std::cout << handEyeMatrix[j + 4 * i] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	for (int i = 0; i < 6; ++i)
	{
		std::cout << err[i] << " ";
	}
	std::cout << std::endl;

	return 0;
}
#endif