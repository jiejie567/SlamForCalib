/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "ImuTypes.h"
#include<iostream>

namespace ORB_SLAM3
{

namespace IMU
{

const float eps = 1e-4;

/** 
 * @brief 强制让R变成一个正交矩阵
 * @param R 待优化的旋转矩阵
 * @return 优化后的矩阵
 */
cv::Mat NormalizeRotation(const cv::Mat &R)
{
    cv::Mat U,w,Vt;
    cv::SVDecomp(R,w,U,Vt,cv::SVD::FULL_UV);
    return U*Vt;
}

/** 
 * @brief 计算反对称矩阵
 * @param v 3维向量
 * @return 反对称矩阵
 */
cv::Mat Skew(const cv::Mat &v)
{
    const float x = v.at<float>(0);
    const float y = v.at<float>(1);
    const float z = v.at<float>(2);
    return (cv::Mat_<float>(3,3) << 0, -z, y,
            z, 0, -x,
            -y,  x, 0);
}

/** 
 * @brief 计算SO3
 * @param xyz 李代数
 * @return SO3
 */
cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

/** 
 * @brief 计算SO3
 * @param xyz 李代数
 * @return SO3
 */
Eigen::Matrix<double,3,3> ExpSO3(const double &x, const double &y, const double &z)
{
    Eigen::Matrix<double,3,3> I = Eigen::MatrixXd::Identity(3,3);
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);
    Eigen::Matrix<double,3,3> W;
    W(0,0) = 0;
    W(0,1) = -z;
    W(0,2) = y;
    W(1,0) = z;
    W(1,1) = 0;
    W(1,2) = -x;
    W(2,0) = -y;
    W(2,1) = x;
    W(2,2) = 0;

    if(d<eps)
        return (I + W + 0.5*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0-cos(d))/d2);
}

/** 
 * @brief 计算SO3
 * @param v 李代数
 * @return SO3
 */
cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

/** 
 * @brief 计算so3
 * @param R SO3
 * @return so3
 */
cv::Mat LogSO3(const cv::Mat &R)
{
    const float tr = R.at<float>(0,0)+R.at<float>(1,1)+R.at<float>(2,2);
    cv::Mat w = (cv::Mat_<float>(3,1) <<(R.at<float>(2,1)-R.at<float>(1,2))/2,
                                        (R.at<float>(0,2)-R.at<float>(2,0))/2,
                                        (R.at<float>(1,0)-R.at<float>(0,1))/2);
    const float costheta = (tr-1.0f)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const float theta = acos(costheta);
    const float s = sin(theta);
    if(fabs(s)<eps)
        return w;
    else
        return theta*w/s;
}

/** 
 * @brief 计算右雅可比
 * @param xyz 李代数
 * @return Jr
 */
cv::Mat RightJacobianSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
    {
        return cv::Mat::eye(3,3,CV_32F);
    }
    else
    {
        return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

/** 
 * @brief 计算右雅可比
 * @param v so3
 * @return Jr
 */
cv::Mat RightJacobianSO3(const cv::Mat &v)
{
    return RightJacobianSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

/** 
 * @brief 计算右雅可比的逆
 * @param xyz so3
 * @return Jr^-1
 */
cv::Mat InverseRightJacobianSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
    {
        return cv::Mat::eye(3,3,CV_32F);
    }
    else
    {
        return I + W/2 + W*W*(1.0f/d2 - (1.0f+cos(d))/(2.0f*d*sin(d)));
    }
}

/** 
 * @brief 计算右雅可比的逆
 * @param v so3
 * @return Jr^-1
 */
cv::Mat InverseRightJacobianSO3(const cv::Mat &v)
{
    return InverseRightJacobianSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

/**
 * @brief                  计算旋转角度积分量
 * 
 * @param[in] angVel       陀螺仪数据
 * @param[in] imuBias      陀螺仪偏置
 * @param[in] time         两帧间的时间差
 */
IntegratedRotation::IntegratedRotation(const cv::Point3f &angVel, const Bias &imuBias, const float &time):
    deltaT(time)
{
    //得到考虑偏置后的角度旋转
    const float x = (angVel.x-imuBias.bwx)*time;
    const float y = (angVel.y-imuBias.bwy)*time;
    const float z = (angVel.z-imuBias.bwz)*time;

    cv::Mat I = cv::Mat::eye(3,3,CV_32F);

    //计算旋转矩阵的模值，后面用罗德里格公式计算旋转矩阵时会用到
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    //角度转成叉积的矩阵形式
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    // eps = 1e-4 是一个小量，根据罗德里格斯公式求极限，后面的高阶小量忽略掉得到此式
    if(d<eps)
    {
        //forster 经典预积分论文公式（4）
        deltaR = I + W;
        //小量时，右扰动 Jr = I
        rightJ = cv::Mat::eye(3,3,CV_32F);
    }
    else
    {
        //forster 经典预积分论文公式（3）
        deltaR = I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2;
        //forster 经典预积分论文公式（8）
        rightJ = I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

/** 
 * @brief 预积分类构造函数，根据输入的偏置初始化预积分参数
 * @param b_ 偏置
 * @param calib imu标定参数的类
 */
Preintegrated::Preintegrated(const Bias &b_, const Calib &calib)
{
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    Initialize(b_);
}

// Copy constructor
Preintegrated::Preintegrated(Preintegrated* pImuPre): dT(pImuPre->dT), C(pImuPre->C.clone()), Info(pImuPre->Info.clone()),
    Nga(pImuPre->Nga.clone()), NgaWalk(pImuPre->NgaWalk.clone()), b(pImuPre->b), dR(pImuPre->dR.clone()), dV(pImuPre->dV.clone()),
    dP(pImuPre->dP.clone()), JRg(pImuPre->JRg.clone()), JVg(pImuPre->JVg.clone()), JVa(pImuPre->JVa.clone()), JPg(pImuPre->JPg.clone()),
    JPa(pImuPre->JPa.clone()), avgA(pImuPre->avgA.clone()), avgW(pImuPre->avgW.clone()), bu(pImuPre->bu), db(pImuPre->db.clone()), mvMeasurements(pImuPre->mvMeasurements)
{

}

/** 
 * @brief 复制上一帧的预积分
 * @param pImuPre 上一帧的预积分
 */
void Preintegrated::CopyFrom(Preintegrated* pImuPre)
{
    std::cout << "Preintegrated: start clone" << std::endl;
    dT = pImuPre->dT;
    C = pImuPre->C.clone();
    Info = pImuPre->Info.clone();
    Nga = pImuPre->Nga.clone();
    NgaWalk = pImuPre->NgaWalk.clone();
    std::cout << "Preintegrated: first clone" << std::endl;
    b.CopyFrom(pImuPre->b);
    dR = pImuPre->dR.clone();
    dV = pImuPre->dV.clone();
    dP = pImuPre->dP.clone();
    // 旋转关于陀螺仪偏置变化的雅克比，以此类推
    JRg = pImuPre->JRg.clone();
    JVg = pImuPre->JVg.clone();
    JVa = pImuPre->JVa.clone();
    JPg = pImuPre->JPg.clone();
    JPa = pImuPre->JPa.clone();
    avgA = pImuPre->avgA.clone();
    avgW = pImuPre->avgW.clone();
    std::cout << "Preintegrated: second clone" << std::endl;
    bu.CopyFrom(pImuPre->bu);
    db = pImuPre->db.clone();
    std::cout << "Preintegrated: third clone" << std::endl;
    mvMeasurements = pImuPre->mvMeasurements;
    std::cout << "Preintegrated: end clone" << std::endl;
}

/** 
 * @brief 初始化预积分
 * @param b_ 偏置
 */
void Preintegrated::Initialize(const Bias &b_)
{
    dR = cv::Mat::eye(3, 3, CV_32F);
    dV = cv::Mat::zeros(3, 1, CV_32F);
    dP = cv::Mat::zeros(3, 1, CV_32F);
    JRg = cv::Mat::zeros(3, 3, CV_32F);
    JVg = cv::Mat::zeros(3, 3, CV_32F);
    JVa = cv::Mat::zeros(3, 3, CV_32F);
    JPg = cv::Mat::zeros(3, 3, CV_32F);
    JPa = cv::Mat::zeros(3, 3, CV_32F);
    C = cv::Mat::zeros(15, 15, CV_32F);
    Info = cv::Mat();
    db = cv::Mat::zeros(6, 1, CV_32F);
    b = b_;
    bu = b_;  // 更新后的偏置
    avgA = cv::Mat::zeros(3, 1, CV_32F);  // 平均加速度
    avgW = cv::Mat::zeros(3, 1, CV_32F);  // 平均角速度
    dT = 0.0f;
    mvMeasurements.clear();  // 存放imu数据及dt
}

/** 
 * @brief 根据新的偏置重新积分mvMeasurements里的数据 Optimizer::InertialOptimization调用
 */ 
void Preintegrated::Reintegrate()
{
    std::unique_lock<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements;
    Initialize(bu);
    for(size_t i=0;i<aux.size();i++)
        IntegrateNewMeasurement(aux[i].a,aux[i].w,aux[i].t);
}

/**
 * @brief 预积分计算，更新noise
 * 
 * @param[in] acceleration  加速度计数据
 * @param[in] angVel        陀螺仪数据
 * @param[in] dt            两帧之间时间差
 */
void Preintegrated::IntegrateNewMeasurement(const cv::Point3f &acceleration, const cv::Point3f &angVel, const float &dt)
{
    // 保存imu数据，利用中值积分的结果构造一个预积分类保存在mvMeasurements中
    mvMeasurements.push_back(integrable(acceleration,angVel,dt));

    // Position is updated firstly, as it depends on previously computed velocity and rotation.
    // Velocity is updated secondly, as it depends on previously computed rotation.
    // Rotation is the last to be updated.

    //Matrices to compute covariance
    // Step 1.构造协方差矩阵 参考Forster论文公式（62），邱笑晨的《预积分总结与公式推导》的P12页也有详细推导:η_ij = A * η_i,j-1 + B_j-1 * η_j-1
    // ? 位姿第一个被更新，速度第二（因为这两个只依赖前一帧计算的旋转矩阵和速度），后面再更新旋转角度
    // 噪声矩阵的传递矩阵，这部分用于计算i到j-1历史噪声或者协方差
    cv::Mat A = cv::Mat::eye(9,9,CV_32F);
    // 噪声矩阵的传递矩阵，这部分用于计算j-1新的噪声或协方差，这两个矩阵里面的数都是当前时刻的，计算主要是为了下一时刻使用
    cv::Mat B = cv::Mat::zeros(9,6,CV_32F);
    
    // 考虑偏置后的加速度、角速度
    cv::Mat acc = (cv::Mat_<float>(3,1) << acceleration.x-b.bax,acceleration.y-b.bay, acceleration.z-b.baz);
    cv::Mat accW = (cv::Mat_<float>(3,1) << angVel.x-b.bwx, angVel.y-b.bwy, angVel.z-b.bwz);

    // 记录平均加速度和角速度
    avgA = (dT*avgA + dR*acc*dt)/(dT+dt);
    avgW = (dT*avgW + accW*dt)/(dT+dt);
    
    // Update delta position dP and velocity dV (rely on no-updated delta rotation)
    // 根据没有更新的dR来更新dP与dV  eq.(38)
    dP = dP + dV*dt + 0.5f*dR*acc*dt*dt;	// 对应viorb论文的公式（2）的第三个，位移积分
    dV = dV + dR*acc*dt;					// 对应viorb论文的公式（2）的第二个，速度积分

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    // 根据η_ij = A * η_i,j-1 + B_j-1 * η_j-1中的Ａ矩阵和Ｂ矩阵对速度和位移进行更新
    cv::Mat Wacc = (cv::Mat_<float>(3,3) << 0, -acc.at<float>(2), acc.at<float>(1),
                                                   acc.at<float>(2), 0, -acc.at<float>(0),
                                                   -acc.at<float>(1), acc.at<float>(0), 0);
    A.rowRange(3,6).colRange(0,3) = -dR*dt*Wacc;
    A.rowRange(6,9).colRange(0,3) = -0.5f*dR*dt*dt*Wacc;
    A.rowRange(6,9).colRange(3,6) = cv::Mat::eye(3,3,CV_32F)*dt;
    B.rowRange(3,6).colRange(3,6) = dR*dt;
    B.rowRange(6,9).colRange(3,6) = 0.5f*dR*dt*dt;

    // Update position and velocity jacobians wrt bias correction
    // ? 更新bias雅克比,计算偏置的雅克比矩阵，pv 分别对ba与bg的偏导数,论文中没推这个值，邱笑晨那边也没有推导,
    // 但论文作者对forster论文公式的基础上做了变形，然后递归更新，参见 https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/212
    // 因为随着时间推移，不可能每次都重新计算雅克比矩阵，所以需要做J(k+1) = j(k) + (~)这类事，分解方式与AB矩阵相同
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JVg = JVg - dR*dt*Wacc*JRg;

    // Update delta rotation
    // Step 2. 构造函数，会根据更新后的bias进行角度积分
    IntegratedRotation dRi(angVel,b,dt);
    // 强行归一化使其符合旋转矩阵的格式
    dR = NormalizeRotation(dR*dRi.deltaR);

    // Compute rotation parts of matrices A and B
    // 补充AB矩阵
    A.rowRange(0,3).colRange(0,3) = dRi.deltaR.t();
    B.rowRange(0,3).colRange(0,3) = dRi.rightJ*dt;
    // 小量delta初始为0，更新后通常也为0，故省略了小量的更新
    // Update covariance
    // Step 3.更新协方差，frost经典预积分论文的第63个公式，推导了噪声（ηa, ηg）对dR dV dP 的影响
    C.rowRange(0,9).colRange(0,9) = A*C.rowRange(0,9).colRange(0,9)*A.t() + B*Nga*B.t(); 	// B矩阵为9*6矩阵 Nga 6*6对角矩阵，3个陀螺仪噪声的平方，3个加速度计噪声的平方
    // 这一部分最开始是0矩阵，随着积分次数增加，每次都加上随机游走，偏置的信息矩阵
    C.rowRange(9,15).colRange(9,15) = C.rowRange(9,15).colRange(9,15) + NgaWalk;	// NgaWalk 6*6 随机游走对角矩阵

    // Update rotation jacobian wrt bias correction
    // 计算偏置的雅克比矩阵，r对bg的导数，∂ΔRij/∂bg = (ΔRjj-1) * ∂ΔRij-1/∂bg - Jr(j-1)*t
    // 论文作者对forster论文公式的基础上做了变形，然后递归更新，参见 https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/212
    // ? 为什么先更新JPa、JPg、JVa、JVg最后更新JRg? 答：这里必须先更新dRi才能更新到这个值，但是为什么JPg和JVg依赖的上一个JRg值进行更新的？
    JRg = dRi.deltaR.t()*JRg - dRi.rightJ*dt;

    // Total integrated time
    // 更新总时间
    dT += dt;
}

/** 
 * @brief 融合两个预积分，发生在删除关键帧的时候，3帧变2帧，需要把两段预积分融合
 * @param pPrev 前面的预积分
 */
void Preintegrated::MergePrevious(Preintegrated* pPrev)
{
    if (pPrev==this)
        return;

    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);
    Bias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<integrable > aux1 = pPrev->mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements;

    Initialize(bav);
    for(size_t i=0;i<aux1.size();i++)
        IntegrateNewMeasurement(aux1[i].a,aux1[i].w,aux1[i].t);
    for(size_t i=0;i<aux2.size();i++)
        IntegrateNewMeasurement(aux2[i].a,aux2[i].w,aux2[i].t);

}

/** 
 * @brief 更新偏置
 * @param bu_ 偏置
 */
void Preintegrated::SetNewBias(const Bias &bu_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    bu = bu_;

    db.at<float>(0) = bu_.bwx-b.bwx;
    db.at<float>(1) = bu_.bwy-b.bwy;
    db.at<float>(2) = bu_.bwz-b.bwz;
    db.at<float>(3) = bu_.bax-b.bax;
    db.at<float>(4) = bu_.bay-b.bay;
    db.at<float>(5) = bu_.baz-b.baz;
}

/** 
 * @brief 获得当前偏置与输入偏置的改变量
 * @param b_ 偏置
 * @return 改变量
 */
IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    return IMU::Bias(b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz,b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
}

/** 
 * @brief 根据新的偏置计算新的dR
 * @param b_ 新的偏置
 * @return dR
 */
cv::Mat Preintegrated::GetDeltaRotation(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    // 计算偏置的变化量
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    // 考虑偏置后，dR对偏置线性化的近似求解,邱笑晨《预积分总结与公式推导》P13～P14
    // Forster论文公式（44）yP17也有结果（但没有推导），后面两个函数GetDeltaPosition和GetDeltaPosition也是基于此推导的
    return NormalizeRotation(dR*ExpSO3(JRg*dbg));
}

/** 
 * @brief 根据新的偏置计算新的dV
 * @param b_ 新的偏置
 * @return dV
 */
cv::Mat Preintegrated::GetDeltaVelocity(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    cv::Mat dba = (cv::Mat_<float>(3,1) << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz);
    // 考虑偏置后，dV对偏置线性化的近似求解,邱笑晨《预积分总结与公式推导》P13，JPg和JPa在预积分处理中更新 
    return dV + JVg*dbg + JVa*dba;
}

/** 
 * @brief 根据新的偏置计算新的dP
 * @param b_ 新的偏置
 * @return dP
 */
cv::Mat Preintegrated::GetDeltaPosition(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    cv::Mat dba = (cv::Mat_<float>(3,1) << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz);
    // 考虑偏置后，dP对偏置线性化的近似求解,邱笑晨《预积分总结与公式推导》P13，JPg和JPa在预积分处理中更新
    return dP + JPg*dbg + JPa*dba;
}

/** 
 * @brief 返回经过db(δba, δbg)更新后的dR,与上面是一个意思
 * @return dR
 */
cv::Mat Preintegrated::GetUpdatedDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return NormalizeRotation(dR*ExpSO3(JRg*db.rowRange(0,3)));
}

/** 
 * @brief 返回经过db(δba, δbg)更新后的dV,与上面是一个意思
 * @return dV
 */
cv::Mat Preintegrated::GetUpdatedDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV + JVg*db.rowRange(0,3) + JVa*db.rowRange(3,6);
}

/** 
 * @brief 返回经过db(δba, δbg)更新后的dP,与上面是一个意思
 * @return dP
 */
cv::Mat Preintegrated::GetUpdatedDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP + JPg*db.rowRange(0,3) + JPa*db.rowRange(3,6);
}

/** 
 * @brief 获取dR
 * @return dR
 */
cv::Mat Preintegrated::GetOriginalDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dR.clone();
}

/** 
 * @brief 获取dV
 * @return dV
 */
cv::Mat Preintegrated::GetOriginalDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV.clone();
}

/** 
 * @brief 获取dP
 * @return dP
 */
cv::Mat Preintegrated::GetOriginalDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP.clone();
}

/** 
 * @brief 获取b,更新前的偏置
 * @return b
 */
Bias Preintegrated::GetOriginalBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return b;
}

/** 
 * @brief 获取bu,更新后的偏置
 * @return bu
 */
Bias Preintegrated::GetUpdatedBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return bu;
}

/** 
 * @brief 获取db,更新前后的偏置差
 * @return db
 */
cv::Mat Preintegrated::GetDeltaBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return db.clone();
}

/** 
 * @brief 获取信息矩阵，没有用到，也就是C矩阵,其中9~15每个元素取倒数，
 * @return 信息矩阵
 */
Eigen::Matrix<double, 15, 15> Preintegrated::GetInformationMatrix()
{
    std::unique_lock<std::mutex> lock(mMutex);
    if(Info.empty())
    {
        Info = cv::Mat::zeros(15,15,CV_32F);
        Info.rowRange(0,9).colRange(0,9)=C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
        for(int i=9;i<15;i++)
            Info.at<float>(i,i)=1.0f/C.at<float>(i,i);
    }

    Eigen::Matrix<double,15,15> EI;
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            EI(i,j)=Info.at<float>(i,j);
    return EI;
}

/** 
 * @brief 赋值新的偏置
 * @param b 偏置
 */
void Bias::CopyFrom(Bias &b)
{
    bax = b.bax;
    bay = b.bay;
    baz = b.baz;
    bwx = b.bwx;
    bwy = b.bwy;
    bwz = b.bwz;
}

std::ostream& operator<< (std::ostream &out, const Bias &b)
{
    if(b.bwx>0)
        out << " ";
    out << b.bwx << ",";
    if(b.bwy>0)
        out << " ";
    out << b.bwy << ",";
    if(b.bwz>0)
        out << " ";
    out << b.bwz << ",";
    if(b.bax>0)
        out << " ";
    out << b.bax << ",";
    if(b.bay>0)
        out << " ";
    out << b.bay << ",";
    if(b.baz>0)
        out << " ";
    out << b.baz;

    return out;
}

/** 
 * @brief 设置参数
 * @param Tbc_ 位姿变换
 * @param ng 噪声
 * @param na 噪声
 * @param ngw 随机游走
 * @param naw 随机游走
 */
void Calib::Set(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
{
    Tbc = Tbc_.clone();
    Tcb = cv::Mat::eye(4,4,CV_32F);
    Tcb.rowRange(0,3).colRange(0,3) = Tbc.rowRange(0,3).colRange(0,3).t();
    Tcb.rowRange(0,3).col(3) = -Tbc.rowRange(0,3).colRange(0,3).t()*Tbc.rowRange(0,3).col(3);
	
	// 噪声协方差
    Cov = cv::Mat::eye(6,6,CV_32F);
    const float ng2 = ng*ng;
    const float na2 = na*na;
	
    Cov.at<float>(0,0) = ng2;
    Cov.at<float>(1,1) = ng2;
    Cov.at<float>(2,2) = ng2;
    Cov.at<float>(3,3) = na2;
    Cov.at<float>(4,4) = na2;
    Cov.at<float>(5,5) = na2;
	
	// 随机游走协方差
    CovWalk = cv::Mat::eye(6,6,CV_32F);
    const float ngw2 = ngw*ngw;
    const float naw2 = naw*naw;
    CovWalk.at<float>(0,0) = ngw2;
    CovWalk.at<float>(1,1) = ngw2;
    CovWalk.at<float>(2,2) = ngw2;
    CovWalk.at<float>(3,3) = naw2;
    CovWalk.at<float>(4,4) = naw2;
    CovWalk.at<float>(5,5) = naw2;
}

/** 
 * @brief imu标定参数的构造函数
 * @param calib imu标定参数
 */
Calib::Calib(const Calib &calib)
{
    Tbc = calib.Tbc.clone();
    Tcb = calib.Tcb.clone();
    Cov = calib.Cov.clone();
    CovWalk = calib.CovWalk.clone();
}

} //namespace IMU

} //namespace ORB_SLAM3
