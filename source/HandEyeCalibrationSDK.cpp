#include "HandEyeCalibrationSDK.h"
#include <HandEyeCalibration.h>
#include <array>

static HandEyeCalibration* g_pHandEye = new HandEyeCalibration();

bool setCalibrateData(const CalibrateData* calibDatas, int dataNum, bool isDegree, int rotationType)
{
    if (dataNum < 3)
    {
        return false;
    }
    vector<HandEyeCalibration::CalibrateData> vCalibData(dataNum);
    for (int i = 0; i < dataNum; ++i)
    {
        size_t imageSize = calibDatas->img.channel * calibDatas->img.width * calibDatas->img.height;
        vCalibData[i].img.width = calibDatas->img.width;
        vCalibData[i].img.height = calibDatas->img.height;
        vCalibData[i].img.channel = calibDatas->img.channel;
        vCalibData[i].img.image_data = new unsigned char[imageSize];
        memcpy(vCalibData[i].img.image_data, calibDatas->img.image_data, imageSize);
        vCalibData[i].pose.x = calibDatas->pose.x;
        vCalibData[i].pose.y = calibDatas->pose.y;
        vCalibData[i].pose.z = calibDatas->pose.z;
        vCalibData[i].pose.rx = calibDatas->pose.rx;
        vCalibData[i].pose.ry = calibDatas->pose.ry;
        vCalibData[i].pose.rz = calibDatas->pose.rz;
        calibDatas++;
    }

    bool success = g_pHandEye->readCalibrateData(vCalibData, isDegree);
    g_pHandEye->setRotationType(HandEyeCalibration::RotationType(rotationType));
    return success;
}

void setChessboardParams(const ChessboardParam* chessboardParam)
{
    g_pHandEye->setChessboardParams({ chessboardParam->cornersHorizontal, chessboardParam->cornersVertical }, chessboardParam->squareSize);
}

bool readCameraIntrinsics(const char* intrinsicsPath, bool recalibrate)
{
   return g_pHandEye->ReadCameraIntrinsics(intrinsicsPath, recalibrate);
}

bool calibrateHandEye(int type, bool optimize)
{
    return g_pHandEye->calibrate(HandEyeType(type), optimize);
}

bool saveHandEyeMatrix(const char* savePath)
{
    return g_pHandEye->SaveHandEyeMatrix(savePath);
}

void getHandEyeMatrix(float* handEyeMatrix)
{
    std::array<std::array<double, 4>, 4> handEyeMatrix_ = g_pHandEye->GetHandEyeMatrix();
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            handEyeMatrix[j + i * 4] = handEyeMatrix_[i][j];
        }
    }
}

bool getCalibrateError(float* error, int errorType)
{
    if (errorType > 2)
    {
        for (int i = 0; i < 6; ++i)
        {
            error[i] = 1e10;
        }
        return false;
    }

    std::array<double, 6> error_ = g_pHandEye->GetCalibrateError(CalibrateErrorType(errorType));
    for (int i = 0; i < 6; ++i)
    {
        error[i] = error_[i];
    }
    return true;
}