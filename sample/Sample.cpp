#include "HandEyeCalibrationSDK.h"
#include <opencv2/opencv.hpp>
#include <string>

const int dataNum = 15;

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

// x y z rx ry rz
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