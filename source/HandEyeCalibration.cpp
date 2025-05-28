#include "HandEyeCalibration.h"
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <direct.h>   // for _mkdir
#include <Eigen/Dense>

#pragma region Internal Functions

//提取棋盘格角点
void findChessboardCorners(vector<vector<cv::Point2f>>& corners_, vector<int>& validImgIndex_, const vector<HandEyeCalibration::ImageDataInfo2D>& imgs, array<int, 2> patternSize);

//过滤无效位姿(未检测到角点的图像)
cv::Mat filterInvalidPose(const vector<array<double, 6>>& poses, const vector<int>& IndexWithImg);

//计算相机位姿
cv::Mat computreCameraPose(cv::Mat& intrinsic_matrix, cv::Mat& distCoeffs, double& meanError, const vector<vector<cv::Point2f>>& chessboard_corners,
    const vector<int>& IndexWithImg, cv::Size pattern_size, double square_size, cv::Size imgSize, bool recalibrate);

// 计算重投影误差
double CalculateReprojectionError(const std::vector<cv::Point2f>& chessboard_corners, const std::vector<cv::Point3f>& objpoints, const cv::Mat& rvec,
    const cv::Mat& tvec, const cv::Mat& intrinsic_matrix, const cv::Mat& distCoeffs);

//手眼标定
cv::Mat calibrateHandEye(vector<cv::Mat>& vecHg, vector<cv::Mat>& vecHc, const cv::Size& pattern_size, double square_size, const vector<int>& IndexWithImg, const cv::Mat& trvecs,
    const cv::Mat& EEPose, HandEyeType handEyeType, RotationType rotaionType);

//计算误差
void calculateError(array<array<double, 6>, 3>& error, const vector<vector<array<double, 3>>>& pointcloud, const vector<array<double, 3 >> &rotateAngle, int validImgNum);

//R和T转RT矩阵
cv::Mat R_T2RT(cv::Mat& R, cv::Mat& T);

//RT转R和T矩阵
void RT2R_T(cv::Mat& RT, cv::Mat& R, cv::Mat& T);

//判断是否为旋转矩阵
bool isRotationMatrix(const cv::Mat& R);

/** @brief 欧拉角 -> 3*3 的R * @param eulerAngle 角度值 * @param seq 指定欧拉角xyz的排列顺序如："xyz" "zyx" */
cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq);

/** @brief 四元数转旋转矩阵 * @note 数据类型double； 四元数定义 q = w + x*i + y*j + z*k * @param q 四元数输入{w,x,y,z}向量 * @return 返回旋转矩阵3*3 */
cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q);

/** @brief ((四元数||欧拉角||旋转向量) && 转移向量) -> 4*4 的Rt * @param m 1*6 || 1*10的矩阵 -> 6 {x,y,z, rx,ry,rz} 10 {x,y,z, qw,qx,qy,qz, rx,ry,rz} * @param useQuaternion 如果是1*10的矩阵，判断是否使用四元数计算旋转矩阵 * @param seq 如果通过欧拉角计算旋转矩阵，需要指定欧拉角xyz的排列顺序如："xyz" "zyx" 为空表示旋转向量 */
cv::Mat attitudeVectorToMatrix(cv::Mat& m, bool useQuaternion, const std::string& seq);

// 将旋转矩阵转换为欧拉角（XYZ顺序）
cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat& R);

//函数用于分割字符串，处理可能的多种分隔符
vector<string> split(const string& line, const string& delimiters);

//角度转弧度
double deg2rad(double x);

//弧度转角度
double rad2deg(double x);

//角度归一化至[-pi, pi]
double normalizeAngle(double angle);

//mat2Array
template <std::size_t N, typename DataType>
array<array<double, N>, N> mat2Array(const cv::Mat& in);

//mat2Array
template <std::size_t N>
array<double, N> mat2Array(const cv::Mat& in, bool rowMajor = false);

//array2Mat
template <std::size_t N>
cv::Mat array2Mat(const std::array<std::array<double, N>, N>& arr);

//array2Mat
template <std::size_t N>
cv::Mat array2Mat(const std::array<double, N>& arr);

cv::Mat normalizeRotation(const cv::Mat& R_);

//保存矩阵到ini
bool saveMatrixToIni(const std::array<std::array<double, 4>, 4>& matrix, const std::string& fileName);

//保存矩阵到dat
bool saveMatrixToDat(const std::array<std::array<double, 4>, 4>& matrix, const std::string& fileName);

//判断文件后缀名
bool hasExtension(const std::string& filename, const std::string& extension);

#pragma endregion Internal Functions

HandEyeCalibration::HandEyeCalibration()
{
    squareSize = 0.0;
    recalibration = true;
    memset(patternSize.data(), 0, sizeof(int) * 2);
    Clear();
}

HandEyeCalibration::~HandEyeCalibration()
{
    for (auto &img : imgs)
    {
        delete[] img.image_data;  // 释放每个图像数据的内存
        img.image_data = nullptr;
        img.width = 0;
        img.height = 0;
    }
}

////读取位姿和标定板图像,需一一对应
//bool HandEyeCalibration::readPoseAndImageFiles(string poseLists, const vector<string>& imgFiles_, bool isDegree)
//{
//    //读取姿态
//    std::ifstream file(poseLists);
//    if (!file.is_open())
//    {
//        return false;
//    }
//
//    string line;
//    while (getline(file, line))
//    {
//        // 处理空行
//        if (line.empty())
//        {
//            continue;
//        }
//
//        // 使用空格、分号、逗号作为分隔符来分割每一行
//        vector<string> tokens = split(line, " ,;");
//
//        // 确保每行有六个值 (x, y, z, rx, ry, rz)
//        if (tokens.size() == 6)
//        {
//            double x = stof(tokens[0]);
//            double y = stof(tokens[1]);
//            double z = stof(tokens[2]);
//            double rx = stof(tokens[3]);
//            double ry = stof(tokens[4]);
//            double rz = stof(tokens[5]);
//            if (!isDegree)
//            {
//                poses.push_back({ x, y, z, rad2deg(rx), rad2deg(ry), rad2deg(rz) });
//            }
//            else
//            {
//                poses.push_back({ x, y, z, rx, ry, rz });
//            }
//        }
//        else
//        {
//            return false;
//        }
//    }
//    file.close();
//
//    //读取图像
//    for (string imgFile : imgFiles_)
//    {
//        cv::Mat image = cv::imread(imgFile, cv::IMREAD_GRAYSCALE);
//        if (!image.empty())
//        {
//            size_t imageSize = image.total() * image.elemSize();
//            Image img;
//            img.buffer = new uchar[imageSize];
//            img.width = image.cols;
//            img.height = image.rows;
//            memcpy(img.buffer, image.data, imageSize);
//            imgs.push_back(img);
//        }
//    }
//
//    return true;
//}
//
//bool HandEyeCalibration::readPoseAndImageFiles(string poseLists, string imgLists_, bool isDegree)
//{
//    //读取图像
//    std::ifstream file(imgLists_);
//    std::vector<std::string> imagePaths;
//
//    if (!file.is_open()) 
//    {
//        return false;
//    }
//
//    std::string line;
//    while (std::getline(file, line)) 
//    {
//        if (!line.empty()) 
//        {
//            cv::Mat image = cv::imread(line, cv::IMREAD_GRAYSCALE);
//            if (!image.empty())
//            {
//                size_t imageSize = image.total() * image.elemSize();
//                Image img;
//                img.buffer = new uchar[imageSize];
//                img.width = image.cols;
//                img.height = image.rows;
//                memcpy(img.buffer, image.data, imageSize);
//                imgs.push_back(img);
//            }
//        }
//    }
//    file.close();
//
//    //读取姿态
//    file.open(poseLists);
//    if (!file.is_open())
//    {
//        return false;
//    }
//
//    poses.clear();
//    while (getline(file, line))
//    {
//        // 处理空行
//        if (line.empty())
//        {
//            continue;
//        }
//
//        // 使用空格、分号、逗号作为分隔符来分割每一行
//        vector<string> tokens = split(line, " ,;");
//
//        // 确保每行有六个值 (x, y, z, rx, ry, rz)
//        if (tokens.size() == 6)
//        {
//            double x = stof(tokens[0]);
//            double y = stof(tokens[1]);
//            double z = stof(tokens[2]);
//            double rx = stof(tokens[3]);
//            double ry = stof(tokens[4]);
//            double rz = stof(tokens[5]);
//            if (!isDegree)
//            {
//                poses.push_back({ x, y, z, rad2deg(rx), rad2deg(ry), rad2deg(rz) });
//            }
//            else
//            {
//                poses.push_back({ x, y, z, rx, ry, rz });
//            }
//        }
//        else
//        {
//            return false;
//        }
//    }
//    file.close();
//
//    return true;
//}

bool HandEyeCalibration::readPose(string poseList, bool isDegree)
{
    //读取姿态
    std::ifstream file(poseList);
    if (!file.is_open())
    {
        return false;
    }

    poses.clear();

    string line;
    while (getline(file, line))
    {
        // 处理空行
        if (line.empty())
        {
            continue;
        }

        // 使用空格、分号、逗号作为分隔符来分割每一行
        vector<string> tokens = split(line, " ,;");

        // 确保每行有六个值 (x, y, z, rx, ry, rz)
        if (tokens.size() == 6)
        {
            double x = stof(tokens[0]);
            double y = stof(tokens[1]);
            double z = stof(tokens[2]);
            double rx = stof(tokens[3]);
            double ry = stof(tokens[4]);
            double rz = stof(tokens[5]);
            if (!isDegree)
            {
                poses.push_back({ x, y, z, rx, ry, rz });
            }
            else
            {
                poses.push_back({ x, y, z, deg2rad(rx), deg2rad(ry), deg2rad(rz) });
            }
        }
        else
        {
            return false;
        }
    }
    file.close();

    return true;
}

bool HandEyeCalibration::readImage(const vector<string>& imgFiles_)
{
    //读取图像
    imgs.clear();
    for (string imgFile : imgFiles_)
    {
        cv::Mat image = cv::imread(imgFile, cv::IMREAD_GRAYSCALE);
        if (!image.empty())
        {
            size_t imageSize = image.total() * image.elemSize();
            ImageDataInfo2D img;
            img.image_data = new uchar[imageSize];
            img.width = image.cols;
            img.height = image.rows;
            memcpy(img.image_data, image.data, imageSize);
            imgs.push_back(img);
        }
        else
        {
            return false;
        }
    }

    return true;
}

//读取标定数据(图像和位姿)
//isDegree为true表示角度,false表示弧度
bool HandEyeCalibration::readCalibrateData(const std::vector<CalibrateData>& calibDatas, bool isDegree)
{
    imgs.clear();
    poses.clear();

    imgs.reserve(calibDatas.size());
    poses.reserve(calibDatas.size());
    for (const CalibrateData &data : calibDatas)
    {
        if (data.img.image_data != nullptr)
        {
            int type = data.img.channel == 1 ? CV_8UC1 : CV_8UC3;
            cv::Mat imageCV(data.img.height, data.img.width, type, data.img.image_data);
            if (type == CV_8UC3)
            {
                cv::cvtColor(imageCV, imageCV, cv::COLOR_BGR2GRAY);
            }
            size_t imageSize = imageCV.total() * imageCV.elemSize();
            ImageDataInfo2D img;
            img.image_data = new uchar[imageSize];
            img.width = imageCV.cols;
            img.height = imageCV.rows;
            memcpy(img.image_data, imageCV.data, imageSize);
            imgs.push_back(img);
        }

        if (!isDegree)
        {
            poses.push_back({ data.pose.x, data.pose.y, data.pose.z, data.pose.rx, data.pose.ry, data.pose.rz });
        }
        else
        {
            poses.push_back({ data.pose.x, data.pose.y, data.pose.z, deg2rad(data.pose.rx), deg2rad(data.pose.ry), deg2rad(data.pose.rz) });
        }
    }

    return true;
}

//设置旋转方式
void HandEyeCalibration::setRotationType(RotationType type_)
{
    rotationType = type_;
}

//设置棋盘格参数
void HandEyeCalibration::setChessboardParams(array<int, 2> patternSize_, double squareSize_)
{
	patternSize = patternSize_;
	squareSize = squareSize_;
}

//设置相机内参和畸变系数,recalibration=true时重新进行相机内参标定
void HandEyeCalibration::setCameraIntrinsicAndDist(array<array<double, 3>, 3> cameraIntrinsic_, array<double, 5> distCoeffs_, bool recalibration_)
{
    cameraIntrinsic = cameraIntrinsic_;
    distCoeffs = distCoeffs_;
    recalibration = recalibration_;
}

//读取相机内参
bool HandEyeCalibration::ReadCameraIntrinsics(const char* intrinsicsPath, bool recalibrate)
{
    recalibration = recalibrate;
    if (!recalibrate)
    {
        // 打开 YML 文件
        cv::FileStorage fs(intrinsicsPath, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            return false;
        }

        // 读取相机矩阵
        cv::Mat camera_matrix;
        fs["cameraIntrinsics"] >> camera_matrix;

        // 读取畸变系数
        cv::Mat distortion_coefficients;
        fs["cameraDistCoeffs"] >> distortion_coefficients;

        fs.release();

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                cameraIntrinsic[i][j] = camera_matrix.at<double>(i, j);
            }
        }
        
        for (int i = 0; i < 5; ++i)
        {
            distCoeffs[i] = distortion_coefficients.at<double>(i, 0);
        }
    }
    
    return true;
}

// 残差类
class ResidualForPointCloud {
public:
    ResidualForPointCloud(const std::vector<std::vector<Eigen::Vector4d>>& pointcloud, const std::vector<Eigen::Matrix4d>& vecHg, const std::vector<Eigen::Matrix4d>& vecHc)
        : pointcloud_(pointcloud), vecHg_(vecHg), vecHc_(vecHc) {}

    template <typename T>
    bool operator()(const T* const X, T* residual) const {
        // 将四元数和平移分开
        Eigen::Matrix<T, 3, 1> translation(X[4], X[5], X[6]);
        Eigen::Quaternion<T> q(X[3], X[0], X[1], X[2]);
        q.normalize();

        // 将四元数转换为旋转矩阵
        Eigen::Matrix<T, 3, 3> R = q.toRotationMatrix();

        // 构建同质变换矩阵
        Eigen::Matrix<T, 4, 4> X_mat = Eigen::Matrix<T, 4, 4>::Identity();
        X_mat.block<3, 3>(0, 0) = R;
        X_mat.block<3, 1>(0, 3) = translation;

        // 对点云做变换
        std::vector<std::vector<Eigen::Matrix<T, 4, 1>>> pointcloudTransform;
        std::vector<std::array<T, 3>> angleTransform;
        pointcloudTransform.resize(pointcloud_.size());
        for (int i = 0; i < pointcloudTransform.size(); ++i)
        {
            pointcloudTransform[i].reserve(pointcloud_[i].size());

            Eigen::Matrix<T, 4, 4> gripperPoseMat = vecHg_[i].cast<T>();
            Eigen::Matrix<T, 4, 4> cameraPoseMat = vecHc_[i].cast<T>();

            for (int j = 0; j < pointcloud_[i].size(); ++j)
            {
                Eigen::Matrix<T, 4, 1> transformPoint = gripperPoseMat * X_mat * cameraPoseMat * pointcloud_[i][j].cast<T>();
                pointcloudTransform[i].push_back(transformPoint);
            }

            Eigen::Matrix<T, 4, 4> worldTransform = gripperPoseMat * X_mat * cameraPoseMat;
            Eigen::Matrix<T, 3, 3> R_world2base = worldTransform.block<3, 3>(0, 0);
            Eigen::Matrix<T, 3, 1> eulerAngle = R_world2base.eulerAngles(0, 1, 2);
            angleTransform.push_back({ eulerAngle[0], eulerAngle[1], eulerAngle[2] });
        }

        std::array<std::array<T, 3>, 3> error;
        T numChessboard = T(pointcloudTransform.size());
        T numCorners = T(pointcloudTransform[0].size());
        T x_rmsg = T(0), y_rmsg = T(0), z_rmsg = T(0);
        array<T, 3> coordMaxDiff = { T(0) };
        array<T, 3> coordMeanDiffg = { T(0) };
        array<T, 3> angleMaxDiff = { T(0) };
        array<T, 3> angleMeanDiff = { T(0) };
        for (int j = 0; j < pointcloudTransform[0].size(); ++j)
        {
            T x_mean = T(0), y_mean = T(0), z_mean = T(0);
            T x_max = T(-1e10), y_max = T(-1e10), z_max = T(-1e10);
            T x_min = T(1e10), y_min = T(1e10), z_min = T(1e10);
            for (int i = 0; i < pointcloudTransform.size(); ++i)
            {
                T x = T(pointcloudTransform[i][j][0]);
                T y = T(pointcloudTransform[i][j][1]);
                T z = T(pointcloudTransform[i][j][2]);
                x_mean += x;
                y_mean += y;
                z_mean += z;

                if (x > x_max)
                {
                    x_max = x;
                }

                if (y > y_max)
                {
                    y_max = y;
                }

                if (z > z_max)
                {
                    z_max = z;
                }

                if (x < x_min)
                {
                    x_min = x;
                }

                if (y < y_min)
                {
                    y_min = y;
                }

                if (z < z_min)
                {
                    z_min = z;
                }
            }
            x_mean /= numChessboard;
            y_mean /= numChessboard;
            z_mean /= numChessboard;

            coordMaxDiff[0] += x_max - x_min;
            coordMaxDiff[1] += y_max - y_min;
            coordMaxDiff[2] += z_max - z_min;

            array<T, 3> coordMeanDiff = { T(0) };
            T x_rms = T(0), y_rms = T(0), z_rms = T(0);
            for (int i = 0; i < numChessboard; ++i)
            {
                x_rms += pow((pointcloudTransform[i][j][0] - x_mean), 2);
                y_rms += pow((pointcloudTransform[i][j][1] - y_mean), 2);
                z_rms += pow((pointcloudTransform[i][j][2] - z_mean), 2);

                coordMeanDiff[0] += abs(pointcloudTransform[i][j][0] - x_mean);
                coordMeanDiff[1] += abs(pointcloudTransform[i][j][1] - y_mean);
                coordMeanDiff[2] += abs(pointcloudTransform[i][j][2] - z_mean);
            }

            coordMeanDiff[0] /= numChessboard;
            coordMeanDiff[1] /= numChessboard;
            coordMeanDiff[2] /= numChessboard;

            coordMeanDiffg[0] += coordMeanDiff[0];
            coordMeanDiffg[1] += coordMeanDiff[1];
            coordMeanDiffg[2] += coordMeanDiff[2];

            x_rms = sqrt(x_rms) / numChessboard;
            y_rms = sqrt(y_rms) / numChessboard;
            z_rms = sqrt(z_rms) / numChessboard;

            x_rmsg += x_rms;
            y_rmsg += y_rms;
            z_rmsg += z_rms;
        }

        coordMeanDiffg[0] /= numCorners;
        coordMeanDiffg[1] /= numCorners;
        coordMeanDiffg[2] /= numCorners;

        /*coordMaxDiff[0] /= numCorners;
        coordMaxDiff[1] /= numCorners;
        coordMaxDiff[2] /= numCorners;*/

        x_rmsg /= numCorners;
        y_rmsg /= numCorners;
        z_rmsg /= numCorners;

        T rx_mean = T(0), ry_mean = T(0), rz_mean = T(0);
        T rx_max = T(-1e10), ry_max = T(-1e10), rz_max = T(-1e10);
        T rx_min = T(1e10), ry_min = T(1e10), rz_min = T(1e10);
        for (int i = 0; i < angleTransform.size(); ++i)
        {
            T rx = angleTransform[i][0];
            T ry = angleTransform[i][1];
            T rz = angleTransform[i][2];
            rx_mean += angleTransform[i][0];
            ry_mean += angleTransform[i][1];
            rz_mean += angleTransform[i][2];
            if (rx > rx_max)
            {
                rx_max = rx;
            }

            if (ry > ry_max)
            {
                ry_max = ry;
            }

            if (rz > rz_max)
            {
                rz_max = rz;
            }

            if (rx < rx_min)
            {
                rx_min = rx;
            }

            if (ry < ry_min)
            {
                ry_min = ry;
            }

            if (rz < rz_min)
            {
                rz_min = rz;
            }
        }
        rx_mean /= T(angleTransform.size());
        ry_mean /= T(angleTransform.size());
        rz_mean /= T(angleTransform.size());

        angleMaxDiff[0] = rx_max - rx_min;
        angleMaxDiff[1] = ry_max - ry_min;
        angleMaxDiff[2] = rz_max - rz_min;

        T rx_rms = T(0), ry_rms = T(0), rz_rms = T(0);
        for (int i = 0; i < angleTransform.size(); ++i)
        {
            rx_rms += pow((angleTransform[i][0] - rx_mean), 2);
            ry_rms += pow((angleTransform[i][1] - ry_mean), 2);
            rz_rms += pow((angleTransform[i][2] - rz_mean), 2);
             
            angleMeanDiff[0] += abs(angleTransform[i][0] - rx_mean);
            angleMeanDiff[1] += abs(angleTransform[i][1] - ry_mean);
            angleMeanDiff[2] += abs(angleTransform[i][2] - rz_mean);
        }
        rx_rms = sqrt(rx_rms) / T(angleTransform.size());
        ry_rms = sqrt(ry_rms) / T(angleTransform.size());
        rz_rms = sqrt(rz_rms) / T(angleTransform.size());

        angleMeanDiff[0] /= T(angleTransform.size());
        angleMeanDiff[1] /= T(angleTransform.size());
        angleMeanDiff[2] /= T(angleTransform.size());

        residual[0] = coordMaxDiff[0];
        residual[1] = coordMaxDiff[1];
        residual[2] = coordMaxDiff[2];

        residual[3] = angleMaxDiff[0];
        residual[4] = angleMaxDiff[1];
        residual[5] = angleMaxDiff[2];

        return true;
    }

private:
    const std::vector<std::vector<Eigen::Vector4d>>& pointcloud_;
    const std::vector<Eigen::Matrix4d>& vecHg_;
    const std::vector<Eigen::Matrix4d>& vecHc_;
};

// 优化函数
void optimize(const std::vector<std::vector<Eigen::Vector4d>>& pointcloud, const std::vector<Eigen::Matrix4d>& vecHg, const std::vector<Eigen::Matrix4d>& vecHc, Eigen::Matrix4d& X) {
    ceres::Problem problem;

    // 提取四元数和平移
    double x[7];
    Eigen::Quaterniond q(X.block<3, 3>(0, 0));
    Eigen::Vector3d t = X.block<3, 1>(0, 3);

    // 初始化优化参数
    x[0] = q.x();
    x[1] = q.y();
    x[2] = q.z();
    x[3] = q.w();

    x[4] = X(0, 3);
    x[5] = X(1, 3);
    x[6] = X(2, 3);

    // 创建残差函数
    auto* residual = new ResidualForPointCloud(pointcloud, vecHg, vecHc);

    // 添加残差块
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ResidualForPointCloud, 6, 7>(residual),
        nullptr,
        x);

    // 设置求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_type = ceres::LINE_SEARCH;
    options.line_search_direction_type = ceres::LBFGS;
    options.max_num_iterations = 500;
    options.logging_type = ceres::SILENT;
    options.minimizer_progress_to_stdout = false;

    // 运行优化
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 更新 X
    Eigen::Quaterniond optimized_q(x[3], x[0], x[1], x[2]); // 注意顺序
    optimized_q.normalize();
    X.block<3, 3>(0, 0) = optimized_q.toRotationMatrix();
    X.block<3, 1>(0, 3) = Eigen::Vector3d(x[4], x[5], x[6]);

    // 将四元数转换为旋转矩阵
    //Eigen::Matrix3d rotation_matrix = X.block<3, 3>(0, 0);

    // 检查正交性 (R^T * R 应为单位矩阵)
    //Eigen::Matrix3d orthogonality_check = rotation_matrix.transpose() * rotation_matrix;

    // 检查行列式是否为1
    //double determinant = rotation_matrix.determinant();
}

//进行手眼标定
bool HandEyeCalibration::calibrate(HandEyeType handEyeType, bool useOptimize)
{
    Clear();
    try
    {
        vector<vector<cv::Point2f>> corners_;
        findChessboardCorners(corners_, validImgIndex, imgs, patternSize);
        cv::Mat EEPose = filterInvalidPose(poses, validImgIndex);
        cv::Mat cameraIntrinsic_ = array2Mat(cameraIntrinsic);
        cv::Mat distCoeffs_ = array2Mat(distCoeffs);
        cv::Size pattern_size(patternSize[0], patternSize[1]);
        cv::Size imgSize(imgs[0].width, imgs[0].height);
        cv::Mat trvecs = computreCameraPose(cameraIntrinsic_, distCoeffs_, meanReprojectionError, corners_, validImgIndex, pattern_size, squareSize, imgSize, recalibration);
        cameraIntrinsic = mat2Array<3, double>(cameraIntrinsic_);
        distCoeffs = mat2Array<5>(distCoeffs_);
        vector<cv::Mat> vecHg, vecHc;
        cv::Mat Hcg_ = calibrateHandEye(vecHg, vecHc, pattern_size, squareSize, validImgIndex, trvecs, EEPose, handEyeType, rotationType);
        if (useOptimize)
        {
            vector<Eigen::Matrix4d> vecHg_(vecHg.size());
            vector<Eigen::Matrix4d> vecHc_(vecHc.size());
            for (int i = 0; i < vecHg.size(); ++i)
            {
                cv::cv2eigen(vecHg[i], vecHg_[i]);
                cv::cv2eigen(vecHc[i], vecHc_[i]);
            }
            vector<vector<Eigen::Vector4d>> pointcloud(vecHc.size());
            for (int i = 0; i < vecHc.size(); ++i)
            {
                for (int j = 0; j < pattern_size.height; j++)
                {
                    for (int k = 0; k < pattern_size.width; k++)
                    {
                        pointcloud[i].push_back(Eigen::Vector4d(k * squareSize * 1.0, j * squareSize * 1.0, 0.0, 1.0));
                    }
                }
            }
            
            Eigen::Matrix4d X;
            cv::cv2eigen(Hcg_, X);
            optimize(pointcloud, vecHg_, vecHc_, X);

            //// 进行SVD分解
            //Eigen::JacobiSVD<Eigen::Matrix3d> svd(X.block(0, 0, 3, 3), Eigen::ComputeFullU | Eigen::ComputeFullV);
            //Eigen::Matrix3d U = svd.matrixU();
            //Eigen::Matrix3d V = svd.matrixV();

            //// 极分解得到的旋转矩阵
            //X.block(0, 0, 3, 3) = U * V.transpose();
            
            cv::eigen2cv(X, Hcg_);
        }

        for (int i = 0; i < validImgIndex.size(); ++i)
        {
            cv::Mat gripperPoseMat = vecHg[i];
            cv::Mat cameraPoseMat = vecHc[i];
            vector<array<double, 3>> chessboardTransformed;
            chessboardTransformed.reserve(pattern_size.height * pattern_size.width);
            for (int j = 0; j < pattern_size.height; j++)
            {
                for (int k = 0; k < pattern_size.width; k++)
                {
                    cv::Mat cheesePos{ double(k * squareSize), double(j * squareSize), 0.0, 1.0 };
                    cv::Mat worldPos;
                    cv::Mat cameraPos = cameraPoseMat * cheesePos;
                    worldPos = gripperPoseMat * Hcg_ * cameraPos;
                    double x = worldPos.at<double>(0, 0);
                    double y = worldPos.at<double>(1, 0);
                    double z = worldPos.at<double>(2, 0);
                    chessboardTransformed.push_back(array<double, 3>{ x, y, z });
                }
            }
            cornersTransformed.push_back(chessboardTransformed);

            cv::Mat worldTransform = gripperPoseMat * Hcg_ * cameraPoseMat;
            cv::Mat R_world2base = worldTransform(cv::Rect(0, 0, 3, 3));
            cv::Vec3f eulerAngle = rotationMatrixToEulerAngles(R_world2base);
            rotateAngleTransformed.push_back({ eulerAngle[0], eulerAngle[1], eulerAngle[2] });
        }

        //bool bIsRotationMatrix = isRotationMatrix(Hcg_);

        Hcg = mat2Array<4, double>(Hcg_);
        gripperPoseMatrix.reserve(vecHg.size());
        for (cv::Mat& Hg : vecHg)
        {
            gripperPoseMatrix.push_back(mat2Array<4, double>(Hg));
        }
        for (cv::Mat& Hc : vecHc)
        {
            cameraPoseMatrix.push_back(mat2Array<4, double>(Hc));
        }

        calculateError(handEyeCalibrateError, cornersTransformed, rotateAngleTransformed, validImgIndex.size());
    }
    catch (const cv::Exception& e)
    {
        errorMsg = e.err;
        return false;
    }
    catch (const std::exception& e)
    {
        errorMsg = "std error";
        return false;
    }

    return true;
}

//获取相机内参
array<array<double, 3>, 3> HandEyeCalibration::GetCameraIntrinsic() const
{
    return cameraIntrinsic;
}

//获取畸变系数
array<double, 5> HandEyeCalibration::GetDistCoeffs() const
{
    return distCoeffs;
}

//获取手眼矩阵
array<array<double, 4>, 4> HandEyeCalibration::GetHandEyeMatrix() const
{
    return Hcg;
}

//保存手眼矩阵
bool HandEyeCalibration::SaveHandEyeMatrix(const std::string& filename) const
{
    if (hasExtension(filename, "ini"))
    {
        saveMatrixToIni(Hcg, filename);
    }
    else if (hasExtension(filename, "dat"))
    {
        saveMatrixToDat(Hcg, filename);
    }
    else
    {
        return false;
    }
    return true;
}

//获取相机标定的重投影误差
double HandEyeCalibration::GetReprojectionError() const
{
    return meanReprojectionError;
}

//获取标定误差
array<double, 6> HandEyeCalibration::GetCalibrateError(CalibrateErrorType errorType) const
{
    switch (errorType)
    {
    case CalibrateErrorType::MAX_DIFF_ERROR:
        return handEyeCalibrateError[0];
    case CalibrateErrorType::MEAN_DIFF_ERROR:
        return handEyeCalibrateError[1];
    case CalibrateErrorType::RMSE:
        return handEyeCalibrateError[2];
    default:
        return handEyeCalibrateError[0];
    }
}

//获取有效棋盘格图像索引
const vector<int>& HandEyeCalibration::GetValidImgIndex() const
{
    return validImgIndex;
}

//保存转换后的棋盘格点云
bool HandEyeCalibration::savePointCloudTransformed(string outputDir) const
{
    _mkdir(outputDir.c_str());

    for (int i = 0; i < cornersTransformed.size(); ++i)
    {
        string filename = outputDir + "/chessboard_" + std::to_string(validImgIndex[i]) + ".txt";
        std::ofstream file(filename);
        if (file.fail())
        {
            return false;
        }

        for (int j = 0; j < cornersTransformed[i].size(); ++j)
        {
            file << cornersTransformed[i][j][0] << "," << cornersTransformed[i][j][1] << "," << cornersTransformed[i][j][2] << std::endl;
        }
        file.close();
    }
    return true;
}

//获取执行失败消息
string HandEyeCalibration::GetErrorMessage() const
{
    return errorMsg;
}

// 清除结果
void HandEyeCalibration::Clear()
{
    meanReprojectionError = 1e10;
    //memset(cameraIntrinsic.data(), 0, sizeof(double) * 9);
    //memset(distCoeffs.data(), 0, sizeof(double) * 5);
    memset(Hcg.data(), 0, sizeof(double) * 16);
    memset(rms.data(), 0, sizeof(double) * 6);
    /*for (int i = 0; i < 3; ++i)
    {
        cameraIntrinsic[i][i] = 1.0;
    }*/
    for (int i = 0; i < 4; ++i)
    {
        Hcg[i][i] = 1.0;
    }
    for (int i = 0; i < 3; ++i)
    {
        memset(handEyeCalibrateError[i].data(), 0, sizeof(double) * 6);
    }
    vector<int>().swap(validImgIndex);
    vector<array<array<double, 4>, 4>>().swap(gripperPoseMatrix);
    vector<array<array<double, 4>, 4>>().swap(cameraPoseMatrix);
    vector<vector<array<double, 3>>>().swap(cornersTransformed);
    vector<array<double, 3>>().swap(rotateAngleTransformed);
}

#pragma region Internal Funtions Implementation

//提取棋盘格角点
void findChessboardCorners(vector<vector<cv::Point2f>>& corners_, vector<int>& validImgIndex_, const vector<HandEyeCalibration::ImageDataInfo2D>& imgs, array<int, 2> patternSize)
{
    corners_.reserve(imgs.size());
    validImgIndex_.reserve(imgs.size());
    int i = 0;
    cv::Size pattern_size(patternSize[0], patternSize[1]);
    for (const auto& image : imgs)
    {
        cv::Mat gray(image.height, image.width, CV_8UC1, image.image_data);
        vector<cv::Point2f> corners;
        bool found = cv::findChessboardCornersSB(gray, pattern_size, corners, cv::CALIB_CB_ACCURACY);
        if (found)
        {
            //cv::Mat src = gray.clone();
            //cv::cvtColor(src, src, cv::COLOR_GRAY2BGR);
            //cv::drawChessboardCorners(src, pattern_size, corners, found);
            corners_.push_back(corners);
            validImgIndex_.push_back(i);
        }
        i++;
    }
}

//过滤无效位姿(未检测到角点的图像)
cv::Mat filterInvalidPose(const vector<array<double, 6>>& poses, const vector<int>& IndexWithImg)
{
    cv::Mat EEPose(IndexWithImg.size(), 6, CV_64F);
    for (int i = 0; i < IndexWithImg.size(); ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            EEPose.at<double>(i, j) = poses[IndexWithImg[i]][j];
        }
    }
    return EEPose;
}

//计算相机位姿
cv::Mat computreCameraPose(cv::Mat& intrinsic_matrix, cv::Mat& distCoeffs, double& meanError, const vector<vector<cv::Point2f>>& chessboard_corners,
    const vector<int>& IndexWithImg, cv::Size pattern_size, double square_size, cv::Size imgSize, bool recalibrate)
{
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat trvecs(IndexWithImg.size(), 6, CV_64F);

    std::vector<std::vector<cv::Point3f>> objpoints;
    for (size_t i = 0; i < IndexWithImg.size(); i++)
    {
        std::vector<cv::Point3f> objp;
        for (int j = 0; j < pattern_size.height; j++)
        {
            for (int k = 0; k < pattern_size.width; k++)
            {
                objp.push_back(cv::Point3f(k * square_size, j * square_size, 0));
            }
        }
        objpoints.push_back(objp);
    }

    if (recalibrate)
    {
        cv::calibrateCamera(objpoints, chessboard_corners, imgSize, intrinsic_matrix, distCoeffs, rvecs, tvecs);
    }

    int index = 0;
    meanError = 0.0;
    for (auto& corners : chessboard_corners)
    {
        cv::Mat rvec, tvec;
        cv::solvePnP(objpoints[index], corners, intrinsic_matrix, distCoeffs, rvec, tvec, false);
        meanError += CalculateReprojectionError(chessboard_corners[index], objpoints[index], rvec, tvec, intrinsic_matrix, distCoeffs);

        for (int j = 0; j < 3; ++j)
        {
            trvecs.at<double>(index, j) = tvec.at<double>(j, 0);
            trvecs.at<double>(index, j + 3) = rvec.at<double>(j, 0);
        }
        ++index;
    }
    meanError /= chessboard_corners.size();
    return trvecs;
}

// 计算重投影误差
double CalculateReprojectionError(const std::vector<cv::Point2f>& chessboard_corners, const std::vector<cv::Point3f>& objpoints, const cv::Mat& rvec,
    const cv::Mat& tvec, const cv::Mat& intrinsic_matrix, const cv::Mat& distCoeffs)
{
    std::vector<cv::Point2f> imgpoints_projected;
    cv::projectPoints(objpoints, rvec, tvec, intrinsic_matrix, distCoeffs, imgpoints_projected);

    double total_error = 0;
    for (size_t i = 0; i < objpoints.size(); ++i)
    {
        double error = cv::norm(imgpoints_projected[i] - chessboard_corners[i]);
        total_error += error;
    }

    double mean_error = total_error / chessboard_corners.size();
    return mean_error;
}

//手眼标定
cv::Mat calibrateHandEye(vector<cv::Mat>& vecHg, vector<cv::Mat>& vecHc, const cv::Size& pattern_size, double square_size, const vector<int>& IndexWithImg, const cv::Mat& trvecs,
    const cv::Mat& EEPose, HandEyeType handEyeType, RotationType rotaionType)
{
    //定义手眼标定矩阵
    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> t_gripper2base;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    cv::Mat R_cam2gripper = cv::Mat_<double>(3, 3);
    cv::Mat t_cam2gripper = cv::Mat_<double>(3, 1);

    size_t num_images = IndexWithImg.size();

    // 读取末端和标定板的姿态矩阵 4*4
    cv::Mat Hcg_;//定义相机到末端的变换矩阵
    cv::Mat tempR, tempT;

    for (size_t i = 0; i < num_images; i++)//计算标定板位姿
    {
        cv::Mat CalPosei = trvecs.row(i);
        cv::Mat tmp = attitudeVectorToMatrix(CalPosei, false, "");
        vecHc.push_back(tmp);
        RT2R_T(tmp, tempR, tempT);

        R_target2cam.push_back(tempR);
        t_target2cam.push_back(tempT);
    }

    std::vector<cv::Mat> vecHb;
    std::vector<cv::Mat> R_base2gripper;
    std::vector<cv::Mat> t_base2gripper;
    vecHb.reserve(num_images);
    R_base2gripper.reserve(num_images);
    t_base2gripper.reserve(num_images);

    string str = "";
    if (rotaionType == RotationType::EULER_ANGLE)
    {
        str = "xyz";
    }
    //str = "zyx";
    for (size_t i = 0; i < num_images; i++)//计算机械臂位姿
    {
        cv::Mat ToolPosei = EEPose.row(i);
        cv::Mat tmp = attitudeVectorToMatrix(ToolPosei, false, str); //转旋转矩阵
        if (handEyeType == HandEyeType::EYE_TO_HAND)
        {
            tmp = tmp.inv();
        }
        vecHg.push_back(tmp);

        RT2R_T(tmp, tempR, tempT);

        R_gripper2base.push_back(tempR);
        t_gripper2base.push_back(tempT);

        vecHb.push_back(tmp.inv());
        cv::Mat tmpInv = tmp.inv();
        RT2R_T(tmpInv, tempR, tempT);

        R_base2gripper.push_back(tempR);
        t_base2gripper.push_back(tempT);

    }
 
    //if ((int)method < 5)
    //{
    //    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_TSAI);
    //    Hcg_ = R_T2RT(R_cam2gripper, t_cam2gripper);
    //}
    //else
    //{

    /*Poses A;
    Poses B;

    for (int i = 0; i < vecHg.size(); ++i)
    {
        for (int j = i + 1; j < vecHg.size(); ++j)
        {
            cv::Mat A_mat = vecHg[j].inv() * vecHg[i];
            cv::Mat B_mat = vecHc[j] * vecHc[i].inv();

            Pose Atmp, Btmp;
            cv::cv2eigen(A_mat, Atmp);
            cv::cv2eigen(B_mat, Btmp);

            A.push_back(Atmp);
            B.push_back(Btmp);
        }
    }

    AndreffExtendedAXXBSolver solver(A, B);
    Pose X = solver.SolveX();
    X(3, 3) = 1.0;
    cv::eigen2cv(X, Hcg_);*/

    cv::Mat R_base2world, t_base2world, R_gripper2cam, t_gripper2cam;
    calibrateRobotWorldHandEye(R_target2cam, t_target2cam, R_base2gripper, t_base2gripper, R_base2world, t_base2world, R_gripper2cam, t_gripper2cam, cv::RobotWorldHandEyeCalibrationMethod::CALIB_ROBOT_WORLD_HAND_EYE_SHAH);
    cv::Mat Hgc;
    Hgc = R_T2RT(R_gripper2cam, t_gripper2cam);
    Hcg_ = Hgc.inv();
    //}
    
    return Hcg_;
}

//计算误差
void calculateError(array<array<double, 6>, 3>& error, const vector<vector<array<double, 3>>>& pointcloud, const vector<array<double, 3 >>& rotateAngle, int validImgNum)
{
    int numChessboard = pointcloud.size();
    int numCorners = pointcloud[0].size();
    double x_rmsg = 0, y_rmsg = 0, z_rmsg = 0;
    array<double, 3> coordMaxDiff = { 0 };
    array<double, 3> coordMeanDiffg = { 0 };
    array<double, 3> angleMaxDiff = { 0 };
    array<double, 3> angleMeanDiff = { 0 };
    for (int j = 0; j < numCorners; ++j)
    {
        double x_mean = 0, y_mean = 0, z_mean = 0;
        double x_max = -1e10, y_max = -1e10, z_max = -1e10;
        double x_min = 1e10, y_min = 1e10, z_min = 1e10;
        for (int i = 0; i < numChessboard; ++i)
        {
            double x = pointcloud[i][j][0];
            double y = pointcloud[i][j][1];
            double z = pointcloud[i][j][2];
            x_mean += x;
            y_mean += y;
            z_mean += z;

            if (x > x_max)
            {
                x_max = x;
            }

            if (y > y_max)
            {
                y_max = y;
            }

            if (z > z_max)
            {
                z_max = z;
            }

            if (x < x_min)
            {
                x_min = x;
            }

            if (y < y_min)
            {
                y_min = y;
            }

            if (z < z_min)
            {
                z_min = z;
            }
        }
        x_mean /= numChessboard;
        y_mean /= numChessboard;
        z_mean /= numChessboard;

        if (x_max - x_min > coordMaxDiff[0])
        {
            coordMaxDiff[0] = x_max - x_min;
        }
        if (y_max - y_min > coordMaxDiff[1])
        {
            coordMaxDiff[1] = y_max - y_min;
        }
        if (z_max - z_min > coordMaxDiff[2])
        {
            coordMaxDiff[2] = z_max - z_min;
        }

        array<double, 3> coordMeanDiff = { 0 };
        double x_rms = 0, y_rms = 0, z_rms = 0;
        for (int i = 0; i < numChessboard; ++i)
        {
            x_rms += pow((pointcloud[i][j][0] - x_mean), 2);
            y_rms += pow((pointcloud[i][j][1] - y_mean), 2);
            z_rms += pow((pointcloud[i][j][2] - z_mean), 2);

            coordMeanDiff[0] += abs(pointcloud[i][j][0] - x_mean);
            coordMeanDiff[1] += abs(pointcloud[i][j][1] - y_mean);
            coordMeanDiff[2] += abs(pointcloud[i][j][2] - z_mean);
        }

        coordMeanDiff[0] /= numChessboard;
        coordMeanDiff[1] /= numChessboard;
        coordMeanDiff[2] /= numChessboard;

        coordMeanDiffg[0] += coordMeanDiff[0];
        coordMeanDiffg[1] += coordMeanDiff[1];
        coordMeanDiffg[2] += coordMeanDiff[2];

        x_rms = sqrt(x_rms) / numChessboard;
        y_rms = sqrt(y_rms) / numChessboard;
        z_rms = sqrt(z_rms) / numChessboard;

        x_rmsg += x_rms;
        y_rmsg += y_rms;
        z_rmsg += z_rms;
    }

    coordMeanDiffg[0] /= numCorners;
    coordMeanDiffg[1] /= numCorners;
    coordMeanDiffg[2] /= numCorners;

    //coordMaxDiff[0] /= numCorners;
    //coordMaxDiff[1] /= numCorners;
    //coordMaxDiff[2] /= numCorners;

    x_rmsg /= numCorners;
    y_rmsg /= numCorners;
    z_rmsg /= numCorners;

    double rx_mean = 0, ry_mean = 0, rz_mean = 0;
    double rx_max = -1e10, ry_max = -1e10, rz_max = -1e10;
    double rx_min = 1e10, ry_min = 1e10, rz_min = 1e10;
    for (int i = 0; i < rotateAngle.size(); ++i)
    {
        double rx = rotateAngle[i][0];
        double ry = rotateAngle[i][1];
        double rz = rotateAngle[i][2];
        rx_mean += rotateAngle[i][0];
        ry_mean += rotateAngle[i][1];
        rz_mean += rotateAngle[i][2];
        if (rx > rx_max)
        {
            rx_max = rx;
        }

        if (ry > ry_max)
        {
            ry_max = ry;
        }

        if (rz > rz_max)
        {
            rz_max = rz;
        }

        if (rx < rx_min)
        {
            rx_min = rx;
        }

        if (ry < ry_min)
        {
            ry_min = ry;
        }

        if (rz < rz_min)
        {
            rz_min = rz;
        }
    }
    rx_mean /= rotateAngle.size();
    ry_mean /= rotateAngle.size();
    rz_mean /= rotateAngle.size();

    angleMaxDiff[0] = rx_max - rx_min;
    angleMaxDiff[1] = ry_max - ry_min;
    angleMaxDiff[2] = rz_max - rz_min;

    double rx_rms = 0, ry_rms = 0, rz_rms = 0;
    for (int i = 0; i < rotateAngle.size(); ++i)
    {
        rx_rms += pow((rotateAngle[i][0] - rx_mean), 2);
        ry_rms += pow((rotateAngle[i][1] - ry_mean), 2);
        rz_rms += pow((rotateAngle[i][2] - rz_mean), 2);

        angleMeanDiff[0] += abs(rotateAngle[i][0] - rx_mean);
        angleMeanDiff[1] += abs(rotateAngle[i][1] - ry_mean);
        angleMeanDiff[2] += abs(rotateAngle[i][2] - rz_mean);
    }
    rx_rms = sqrt(rx_rms) / rotateAngle.size();
    ry_rms = sqrt(ry_rms) / rotateAngle.size();
    rz_rms = sqrt(rz_rms) / rotateAngle.size();

    angleMeanDiff[0] /= rotateAngle.size();
    angleMeanDiff[1] /= rotateAngle.size();
    angleMeanDiff[2] /= rotateAngle.size();

    error[0] = { coordMaxDiff[0], coordMaxDiff[1], coordMaxDiff[2],  angleMaxDiff[0], angleMaxDiff[1], angleMaxDiff[2] };
    error[1] = { coordMeanDiffg[0], coordMeanDiffg[1], coordMeanDiffg[2],  angleMeanDiff[0], angleMeanDiff[1], angleMeanDiff[2] };
    error[2] = { x_rmsg, y_rmsg, z_rmsg, rx_rms, ry_rms, rz_rms };
#if 0
    int numChessboard = pointcloud.size();
    int numCorners = pointcloud[0].size();
    double x_rmsg = 0, y_rmsg = 0, z_rmsg = 0;
    array<double, 3> coordMaxDiff = { 0 };
    array<double, 3> coordMeanDiffg = { 0 };
    array<double, 3> angleMaxDiff = { 0 };
    array<double, 3> angleMeanDiff = { 0 };
    for (int j = 0; j < numCorners; ++j)
    {
        double x_mean = 0, y_mean = 0, z_mean = 0;
        double x_max = -1e10, y_max = -1e10, z_max = -1e10;
        double x_min = 1e10, y_min = 1e10, z_min = 1e10;
        for (int i = 0; i < numChessboard; ++i)
        {
            double x = pointcloud[i][j][0];
            double y = pointcloud[i][j][1];
            double z = pointcloud[i][j][2];
            x_mean += x;
            y_mean += y;
            z_mean += z;

            if (x > x_max)
            {
                x_max = x;
            }

            if (y > y_max)
            {
                y_max = y;
            }

            if (z > z_max)
            {
                z_max = z;
            }

            if (x < x_min)
            {
                x_min = x;
            }

            if (y < y_min)
            {
                y_min = y;
            }

            if (z < z_min)
            {
                z_min = z;
            }
        }
        x_mean /= numChessboard;
        y_mean /= numChessboard;
        z_mean /= numChessboard;
        coordMaxDiff[0] += x_max - x_min;
        coordMaxDiff[1] += y_max - y_min;
        coordMaxDiff[2] += z_max - z_min;
        
        array<double, 3> coordMeanDiff = { 0 };
        double x_rms = 0, y_rms = 0, z_rms = 0;
        for (int i = 0; i < numChessboard; ++i)
        {
            x_rms += pow((pointcloud[i][j][0] - x_mean), 2);
            y_rms += pow((pointcloud[i][j][1] - y_mean), 2);
            z_rms += pow((pointcloud[i][j][2] - z_mean), 2);

            coordMeanDiff[0] += abs(pointcloud[i][j][0] - x_mean);
            coordMeanDiff[1] += abs(pointcloud[i][j][1] - y_mean);
            coordMeanDiff[2] += abs(pointcloud[i][j][2] - z_mean);
        }

        coordMeanDiff[0] /= numChessboard;
        coordMeanDiff[1] /= numChessboard;
        coordMeanDiff[2] /= numChessboard;

        coordMeanDiffg[0] += coordMeanDiff[0];
        coordMeanDiffg[1] += coordMeanDiff[1];
        coordMeanDiffg[2] += coordMeanDiff[2];

        x_rms = sqrt(x_rms) / numChessboard;
        y_rms = sqrt(y_rms) / numChessboard;
        z_rms = sqrt(z_rms) / numChessboard;

        x_rmsg += x_rms;
        y_rmsg += y_rms;
        z_rmsg += z_rms;
    }

    coordMeanDiffg[0] /= numCorners;
    coordMeanDiffg[1] /= numCorners;
    coordMeanDiffg[2] /= numCorners;

    coordMaxDiff[0] /= numCorners;
    coordMaxDiff[1] /= numCorners;
    coordMaxDiff[2] /= numCorners;

    x_rmsg /= numCorners;
    y_rmsg /= numCorners;
    z_rmsg /= numCorners;
    
    double rx_mean = 0, ry_mean = 0, rz_mean = 0;
    double rx_max = -1e10, ry_max = -1e10, rz_max = -1e10;
    double rx_min = 1e10, ry_min = 1e10, rz_min = 1e10;
    for (int i = 0; i < rotateAngle.size(); ++i)
    {
        double rx = rotateAngle[i][0];
        double ry = rotateAngle[i][1];
        double rz = rotateAngle[i][2];
        rx_mean += rotateAngle[i][0];
        ry_mean += rotateAngle[i][1];
        rz_mean += rotateAngle[i][2];
        if (rx > rx_max)
        {
            rx_max = rx;
        }

        if (ry > ry_max)
        {
            ry_max = ry;
        }

        if (rz > rz_max)
        {
            rz_max = rz;
        }

        if (rx < rx_min)
        {
            rx_min = rx;
        }

        if (ry < ry_min)
        {
            ry_min = ry;
        }

        if (rz < rz_min)
        {
            rz_min = rz;
        }
    }
    rx_mean /= rotateAngle.size();
    ry_mean /= rotateAngle.size();
    rz_mean /= rotateAngle.size();

    angleMaxDiff[0] += rx_max - rx_min;
    angleMaxDiff[1] += ry_max - ry_min;
    angleMaxDiff[2] += rz_max - rz_min;

    double rx_rms = 0, ry_rms = 0, rz_rms = 0;
    for (int i = 0; i < rotateAngle.size(); ++i)
    {
        rx_rms += pow((rotateAngle[i][0] - rx_mean), 2);
        ry_rms += pow((rotateAngle[i][1] - ry_mean), 2);
        rz_rms += pow((rotateAngle[i][2] - rz_mean), 2);

        angleMeanDiff[0] += abs(rotateAngle[i][0] - rx_mean);
        angleMeanDiff[1] += abs(rotateAngle[i][1] - ry_mean);
        angleMeanDiff[2] += abs(rotateAngle[i][2] - rz_mean);
    }
    rx_rms = sqrt(rx_rms) / rotateAngle.size();
    ry_rms = sqrt(ry_rms) / rotateAngle.size();
    rz_rms = sqrt(rz_rms) / rotateAngle.size();

    angleMeanDiff[0] /= rotateAngle.size();
    angleMeanDiff[1] /= rotateAngle.size();
    angleMeanDiff[2] /= rotateAngle.size();

    error[0] = { coordMaxDiff[0], coordMaxDiff[1], coordMaxDiff[2],  angleMaxDiff[0], angleMaxDiff[1], angleMaxDiff[2] };
    error[1] = { coordMeanDiffg[0], coordMeanDiffg[1], coordMeanDiffg[2],  angleMeanDiff[0], angleMeanDiff[1], angleMeanDiff[2] };
    error[2] = { x_rmsg, y_rmsg, z_rmsg, rx_rms, ry_rms, rz_rms };
#endif
}

//R和T转RT矩阵
cv::Mat R_T2RT(cv::Mat& R, cv::Mat& T)
{

    cv::Mat RT;
    cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
        0.0, 0.0, 0.0);
    cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0), 1.0);

    cv::hconcat(R1, T1, RT);//C=A+B左右拼接
    return RT;
}

//RT转R和T矩阵
void RT2R_T(cv::Mat& RT, cv::Mat& R, cv::Mat& T)
{

    cv::Rect R_rect(0, 0, 3, 3);
    cv::Rect T_rect(3, 0, 1, 3);
    R = RT(R_rect);
    T = RT(T_rect);
}

//判断是否为旋转矩阵
bool isRotationMatrix(const cv::Mat& R)
{

    cv::Mat tmp33 = R({
    0,0,3,3 });
    cv::Mat shouldBeIdentity;

    shouldBeIdentity = tmp33.t() * tmp33;

    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

/** @brief 欧拉角 -> 3*3 的R * @param eulerAngle 角度值 * @param seq 指定欧拉角xyz的排列顺序如："xyz" "zyx" */
cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq)
{

    CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);

    //eulerAngle /= 180 / CV_PI;
    cv::Matx13d m(eulerAngle);
    auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
    auto xs = std::sin(rx), xc = std::cos(rx);
    auto ys = std::sin(ry), yc = std::cos(ry);
    auto zs = std::sin(rz), zc = std::cos(rz);

    cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
    cv::Mat rotY = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
    cv::Mat rotZ = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);

    cv::Mat rotMat;

    if (seq == "zyx")		rotMat = rotX * rotY * rotZ;
    else if (seq == "yzx")	rotMat = rotX * rotZ * rotY;
    else if (seq == "zxy")	rotMat = rotY * rotX * rotZ;
    else if (seq == "xzy")	rotMat = rotY * rotZ * rotX;
    else if (seq == "yxz")	rotMat = rotZ * rotX * rotY;
    else if (seq == "xyz")	rotMat = rotZ * rotY * rotX;
    else 
    {

        cv::error(cv::Error::StsAssert, "Euler angle sequence string is wrong.",
            __FUNCTION__, __FILE__, __LINE__);
    }

    if (!isRotationMatrix(rotMat)) 
    {

        cv::error(cv::Error::StsAssert, "Euler angle can not convert to rotated matrix",
            __FUNCTION__, __FILE__, __LINE__);
    }

    return rotMat;
    //cout << isRotationMatrix(rotMat) << endl;
}

/** @brief 四元数转旋转矩阵 * @note 数据类型double； 四元数定义 q = w + x*i + y*j + z*k * @param q 四元数输入{w,x,y,z}向量 * @return 返回旋转矩阵3*3 */
cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q)
{

    double w = q[0], x = q[1], y = q[2], z = q[3];

    double x2 = x * x, y2 = y * y, z2 = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;

    cv::Matx33d res{

        1 - 2 * (y2 + z2),	2 * (xy - wz),		2 * (xz + wy),
        2 * (xy + wz),		1 - 2 * (x2 + z2),	2 * (yz - wx),
        2 * (xz - wy),		2 * (yz + wx),		1 - 2 * (x2 + y2),
    };
    return cv::Mat(res);
}

/** @brief ((四元数||欧拉角||旋转向量) && 转移向量) -> 4*4 的Rt * @param m 1*6 || 1*10的矩阵 -> 6 {x,y,z, rx,ry,rz} 10 {x,y,z, qw,qx,qy,qz, rx,ry,rz} * @param useQuaternion 如果是1*10的矩阵，判断是否使用四元数计算旋转矩阵 * @param seq 如果通过欧拉角计算旋转矩阵，需要指定欧拉角xyz的排列顺序如："xyz" "zyx" 为空表示旋转向量 */
cv::Mat attitudeVectorToMatrix(cv::Mat& m, bool useQuaternion, const std::string& seq)
{

    CV_Assert(m.total() == 6 || m.total() == 10);
    if (m.cols == 1)
        m = m.t();
    cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);

    //如果使用四元数转换成旋转矩阵则读取m矩阵的第第四个成员，读4个数据
    if (useQuaternion)	// normalized vector, its norm should be 1.
    {
        cv::Vec4d quaternionVec = m({
    3, 0, 4, 1 });
        quaternionToRotatedMatrix(quaternionVec).copyTo(tmp({
    0, 0, 3, 3 }));
    }
    else
    {

        cv::Mat rotVec;
        if (m.total() == 6)
            rotVec = m({
    3, 0, 3, 1 });		//6
        else
            rotVec = m({
    7, 0, 3, 1 });		//10

        //如果seq为空表示传入的是旋转向量，否则"xyz"的组合表示欧拉角
        if (0 == seq.compare(""))
            cv::Rodrigues(rotVec, tmp({
    0, 0, 3, 3 }));
        else
            eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp({
    0, 0, 3, 3 }));
    }
    tmp({
    3, 0, 1, 3 }) = m({
    0, 0, 3, 1 }).t();

    return tmp;
}

// 将旋转矩阵转换为欧拉角（XYZ顺序）
cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat& R)
{
    // 确保是3x3矩阵
    CV_Assert(R.rows == 3 && R.cols == 3);

    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // 如果sy接近0，则可能存在奇异性

    double x, y, z;
    if (!singular)
    {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2)); // roll (X轴旋转)
        y = std::atan2(-R.at<double>(2, 0), sy);                // pitch (Y轴旋转)
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0)); // yaw (Z轴旋转)
    }
    else
    {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }

    auto rad2deg = [](double x)
    {
        return x / CV_PI * 180.0;
    };

    return cv::Vec3f(rad2deg(normalizeAngle(x)), rad2deg(normalizeAngle(y)), rad2deg(normalizeAngle(z))); // 返回欧拉角 (roll, pitch, yaw)
}

// 函数用于分割字符串，处理可能的多种分隔符
vector<string> split(const string& line, const string& delimiters)
{
    vector<string> tokens;
    size_t start = 0;
    size_t end = 0;

    while ((end = line.find_first_of(delimiters, start)) != string::npos) 
    {
        if (end > start) 
        {
            tokens.push_back(line.substr(start, end - start));
        }
        start = end + 1;
    }

    if (start < line.size()) 
    {
        tokens.push_back(line.substr(start));
    }

    return tokens;
}

//角度转弧度
double deg2rad(double x)
{
    return x * CV_PI / 180.0;
}

//弧度转角度
double rad2deg(double x)
{
    return x * 180.0 / CV_PI;
}

double normalizeAngle(double angle) 
{
    // 将角度归一化到 [0, 2π)
    angle = fmod(angle, 2 * CV_PI);
    // 调整到 [-π, π)
    if (angle < -CV_PI) 
    {
        angle += 2 * CV_PI;
    }
    else if (angle >= CV_PI) 
    {
        angle -= 2 * CV_PI;
    }
    return angle;
}

//mat2Array
template <std::size_t N, typename DataType>
array<array<double, N>, N> mat2Array(const cv::Mat& in)
{
    std::array<std::array<double, N>, N> arr;
    for (std::size_t i = 0; i < N; ++i) 
    {
        for (std::size_t j = 0; j < N; ++j) 
        {
            arr[i][j] = in.at<DataType>(i, j);
        }
    }
    return arr;
}

//mat2Array
template <std::size_t N>
array<double, N> mat2Array(const cv::Mat& in, bool rowMajor)
{
    std::array<double, N> arr;
    if (rowMajor)
    {
        for (std::size_t j = 0; j < N; ++j)
        {
            arr[j] = in.at<double>(j, 0);
        }
        return arr;
    }

    for (std::size_t j = 0; j < N; ++j)
    {
        arr[j] = in.at<double>(0, j);
    }
    return arr;
}

//array2Mat
template <std::size_t N>
cv::Mat array2Mat(const std::array<std::array<double, N>, N>& arr)
{
    cv::Mat mat(N, N, CV_64F);

    for (std::size_t i = 0; i < N; ++i) 
    {
        for (std::size_t j = 0; j < N; ++j) 
        {
            mat.at<double>(i, j) = arr[i][j];
        }
    }

    return mat;
}

//array2Mat
template <std::size_t N>
cv::Mat array2Mat(const std::array<double, N>& arr)
{
    cv::Mat mat(1, N, CV_64F);
    for (std::size_t j = 0; j < N; ++j)
    {
        mat.at<double>(0, j) = arr[j];
    }
    return mat;
}

cv::Mat normalizeRotation(const cv::Mat& R_)
{
    // Make R unit determinant
    cv::Mat R = R_.clone();
    double det = determinant(R);
    R = std::cbrt(std::copysign(1, det) / std::fabs(det)) * R;

    //// Make R orthogonal
    //cv::Mat w, u, vt;
    //SVDecomp(R, w, u, vt);
    //R = u * vt;

    //// Handle reflection case
    //if (determinant(R) < 0)
    //{
    //    cv::Matx33d diag(1.0, 0.0, 0.0,
    //        0.0, 1.0, 0.0,
    //        0.0, 0.0, -1.0);
    //    R = u * diag * vt;
    //}

    return R;
}

bool saveMatrixToIni(const std::array<std::array<double, 4>, 4>& matrix, const std::string& fileName)
{
    // 打开文件进行写入
    std::ofstream file(fileName);
    if (!file.is_open()) 
    {
        return false;
    }

    file << "[Matrix]" << std::endl;

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            // 写入矩阵元素，格式为 Element00=1.23
            file << "Element" << i << j << "=" << std::fixed << std::setprecision(6) << matrix[i][j] << std::endl;
        }
    }

    file.close();
    return true;
}

bool saveMatrixToDat(const std::array<std::array<double, 4>, 4>& matrix, const std::string& fileName)
{
    // 打开文件进行写入（以二进制方式）
    std::ofstream file(fileName, std::ios::binary);
    if (!file.is_open()) 
    {
        return false;
    }

    // 将矩阵元素写入文件
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            file.write(reinterpret_cast<const char*>(&matrix[i][j]), sizeof(matrix[i][j]));
        }
    }

    file.close();
    return true;
}

//判断文件后缀名
bool hasExtension(const std::string& filename, const std::string& extension)
{
    // 查找文件名最后一个点的位置
    size_t dotPos = filename.find_last_of('.');

    // 如果没有点，返回 false
    if (dotPos == std::string::npos) {
        return false;
    }

    // 获取文件后缀名，并比较大小写
    std::string fileExtension = filename.substr(dotPos + 1);
    return (fileExtension == extension);
}

#pragma endregion Internal Funtions Implementation