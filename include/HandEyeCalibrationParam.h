#pragma once

namespace
{
	// 图像数据
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

	// 机械臂末端位姿
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

	// 标定数据
	// 当前图像和对应的机械臂末端位姿
	struct CalibrateData
	{
		ImageDataInfo2D img;
		RobotPose pose;
	};

	// 棋盘格尺寸参数
	struct ChessboardParam
	{
		int cornersHorizontal;// 水平方向角点个数
		int cornersVertical;// 竖直方向角点个数
		double squareSize;// 方格大小(mm)

		ChessboardParam()
		{
			cornersHorizontal = 0;
			cornersVertical = 0;
			squareSize = 0;
		}
	};
}
