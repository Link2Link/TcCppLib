//
// Created by Chenfei Wang on 7/5/2023.
//

#ifndef TCCPPLIB_CALC_HPP
#define TCCPPLIB_CALC_HPP

#include <TcEigen/Dense>
#include <vector>
#include <cmath>
#include <limits>

namespace robot
{
    // constexpr
    constexpr double M_PI = 3.14159265358979323846;
    constexpr double EPS = 1E-8;
    constexpr double NEAR_ZERO = 1E-6;
    constexpr double CHECK_BOUND = 1E-3;
    constexpr double EPS_SP = std::numeric_limits<float>::epsilon();
    constexpr double EPS_DP = std::numeric_limits<double>::epsilon();
    constexpr double MAX_SP = std::numeric_limits<float>::max();
    constexpr double MAX_DP = std::numeric_limits<double>::max();

    inline bool isInf(float val) {
        return (val == MAX_SP);
    }

    inline bool isInf(double val) {
        return (val == MAX_DP);
    }

    using Vec3 = Eigen::Matrix<double, 3, 1>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using VecX = Eigen::VectorXd;
    using Mat3 = Eigen::Matrix<double, 3, 3>;
    using Mat4 = Eigen::Matrix<double, 4, 4>;
    using MatX = Eigen::MatrixXd;

    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d O3 = Eigen::Matrix3d::Zero();
    const Eigen::Matrix4d O4 = Eigen::Matrix4d::Zero();

    inline bool NearZero(const double val)
    {
        return std::abs(val) < NEAR_ZERO;
    }

    VecX Normalize(VecX V)
    {
        V.normalize();
        return V;
    }

    inline Mat3 VecToso3(const Vec3 omg)
    {
        Mat3 m_ret;
        m_ret << 0, -omg(2), omg(1),
                omg(2), 0, -omg(0),
                -omg(1), omg(0), 0;
        return m_ret;
    }

    inline Vec3 so3ToVec(const Mat3 mat)
    {
        Vec3 v_ret;
        v_ret << mat(2, 1), mat(0, 2), mat(1, 0);
        return v_ret;
    }

    inline Eigen::Vector4d AxisAng3(const Vec3& expc3) {
        Eigen::Vector4d v_ret;
        v_ret << Normalize(expc3), expc3.norm();
        return v_ret;
    }

    Mat3 MatrixExp3(const Mat3& mat)
    {
        Vec3 omgtheta = so3ToVec(mat);
        if (NearZero(mat.norm()))
        {
            return Eigen::Matrix3d::Identity();
        }
        double theta = (AxisAng3(omgtheta))(3);
        Mat3 so3 = mat * (1 / theta);
        return I3 + std::sin(theta) * so3 + ((1 - std::cos(theta)) * (so3 * so3));
    }

    inline Mat3 MatrixExp3(const Vec3& vec)
    {
        Mat3 mat = VecToso3(vec);
        return MatrixExp3(mat);
    }

    Mat3 MatrixLog3(const Mat3& R)
    {
        double acosinput = (R.trace() - 1) / 2.0;
        auto m_ret = O3;
        if (acosinput >= 1)
            return m_ret;
        else if (acosinput <= -1) {
            Eigen::Vector3d omg;
            if (!NearZero(1 + R(2, 2)))
                omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
            else if (!NearZero(1 + R(1, 1)))
                omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
            else
                omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
            m_ret = VecToso3(M_PI * omg);
            return m_ret;
        }
        else {
            double theta = std::acos(acosinput);
            m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
            return m_ret;
        }
    }

    inline Vec3 MatrixLog3vec(const Mat3& R)
    {
        return so3ToVec(MatrixLog3(R));
    }

    Mat4 RpToTrans(const Mat3& R, const Vec3& p)
    {
        Mat4 m_ret;
        m_ret << R, p,
                0, 0, 0, 1;
        return m_ret;
    }

    std::vector<MatX> TransToRp(const Mat4& T) {
        std::vector<MatX> Rp_ret;
        Mat3 R_ret;
        // Get top left 3x3 corner
        R_ret = T.block<3, 3>(0, 0);

        Vec3 p_ret(T(0, 3), T(1, 3), T(2, 3));

        Rp_ret.push_back(R_ret);
        Rp_ret.push_back(p_ret);

        return Rp_ret;
    }

    Mat4 VecTose3(const Vec6& V) {
        // Separate angular (exponential representation) and linear velocities
        Vec3 exp(V(0), V(1), V(2));
        Vec3 linear(V(3), V(4), V(5));

        // Fill in values to the appropriate parts of the transformation matrix
        Mat4 m_ret;
        m_ret << VecToso3(exp), linear,
                0, 0, 0, 0;

        return m_ret;
    }

    Vec6 se3ToVec(const Mat4& T) {
        Vec6 m_ret;
        m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

        return m_ret;
    }

    Eigen::Matrix<double, 6, 6> Adjoint(const Mat4& T) {
        std::vector<Eigen::MatrixXd> R = TransToRp(T);
        Eigen::Matrix<double, 6, 6> ad_ret;
        ad_ret = Eigen::Matrix<double, 6, 6>::Zero();
        ad_ret << R[0], O3,
                VecToso3(R[1]) * R[0], R[0];
        return ad_ret;
    }

    Eigen::Matrix<double, 6, 6> ad(Vec6 V) {
        Mat3 omgmat = VecToso3(Vec3(V(0), V(1), V(2)));

        Eigen::Matrix<double, 6, 6> result;
        result.topLeftCorner<3, 3>() = omgmat;
        result.topRightCorner<3, 3>() = O3;
        result.bottomLeftCorner<3, 3>() = VecToso3(Vec3(V(3), V(4), V(5)));
        result.bottomRightCorner<3, 3>() = omgmat;
        return result;
    }

    Mat4 TransInv(const Mat4& transform) {
        auto rp = TransToRp(transform);
        Mat3 Rt = rp.at(0).transpose();
        Vec3 t = -(Rt * rp.at(1));
        Mat4 inv;
        inv = O4;
        inv.block(0, 0, 3, 3) = Rt;
        inv.block(0, 3, 3, 1) = t;
        inv(3, 3) = 1;
        return inv;
    }

    inline Mat3 RotInv(const Mat3& rotMatrix) {
        return rotMatrix.transpose();
    }


    Vec6 ScrewToAxis(Vec3 q, Vec3 s, double h) {
        Vec6 axis;
        axis.segment(0, 3) = s;
        axis.segment(3, 3) = q.cross(s) + (h * s);
        return axis;
    }

    Eigen::Vector<double, 7> AxisAng6(const Vec6& expc6) {
        Eigen::Vector<double, 7> v_ret(7);
        if (NearZero(expc6.norm()))
        {
            v_ret << expc6, expc6.norm();
            return v_ret;
        }
        double theta = Vec3(expc6(0), expc6(1), expc6(2)).norm();
        if (NearZero(theta))
            theta = Vec3(expc6(3), expc6(4), expc6(5)).norm();
        v_ret << expc6 / theta, theta;
        return v_ret;
    }


    Mat3 ProjectToSO3(const Mat3& M) {
        Eigen::JacobiSVD<Mat3> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Mat3 R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0)
            // In this case the result may be far from M; reverse sign of 3rd column
            R.col(2) *= -1;
        return R;
    }

    Mat4 ProjectToSE3(const Mat4& M) {
        Mat3 R = M.block<3, 3>(0, 0);
        Vec3 t = M.block<3, 1>(0, 3);
        Mat4 T = RpToTrans(ProjectToSO3(R), t);
        return T;
    }

    double DistanceToSO3(const Mat3& M) {
        if (M.determinant() > 0)
            return (M.transpose() * M - Eigen::Matrix3d::Identity()).norm();
        else
            return MAX_DP;
    }

    double DistanceToSE3(const Mat4& T) {
        Mat3 matR = T.block<3, 3>(0, 0);
        if (matR.determinant() > 0) {
            Mat4 m_ret;
            m_ret << matR.transpose()*matR, Eigen::Vector3d::Zero(3),
                    T.row(3);
            m_ret = m_ret - I4;
            return m_ret.norm();
        }
        else
            return MAX_DP;
    }

    inline bool isSO3(const Mat3& M) {
        return (DistanceToSO3(M) < CHECK_BOUND);
    }

    inline bool isSE3(const Mat4& T) {
        return (DistanceToSE3(T) < CHECK_BOUND);
    }

}




#endif //TCCPPLIB_CALC_HPP
