/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Utils/Geometry.h"

float cosRayParallax(const Eigen::Vector3f& a, const Eigen::Vector3f& b){
    return a.dot(b)/(a.norm()*b.norm());
}

void triangulate(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2,
                 const Sophus::SE3f &T1w, const Sophus::SE3f &T2w, Eigen::Vector3f &x3D){
    Sophus::SE3f T21 = T2w * T1w.inverse();
    Eigen::Vector3f m0 = T21.rotationMatrix() * xn1;
    Eigen::Vector3f m1 = xn2;

    Eigen::Vector3f t = T21.translation().normalized();
    Eigen::Matrix<float,3,2> M;
    M.col(0) = m0.normalized();
    M.col(1) = m1.normalized();

    Eigen::Matrix<float,2,3> A = M.transpose() * (Eigen::Matrix3f::Identity() - t*t.transpose());
    Eigen::JacobiSVD<Eigen::Matrix<float,2,3>> svd(A, Eigen::ComputeFullV);
    Eigen::Vector3f n = svd.matrixV().col(1);

    Eigen::Vector3f m0_ = m0 - (m0.dot(n)) * n;
    Eigen::Vector3f m1_ = m1 - (m1.dot(n)) * n;

    Eigen::Vector3f z = m1_.cross(m0_);
    float lambda0 = z.dot(T21.translation().cross(m1_))/(z.squaredNorm());
    Eigen::Vector3f p3D1 = T21.translation() + lambda0*m0_;

    float lambda1 = z.dot(T21.translation().cross(m0_))/(z.squaredNorm());
    Eigen::Vector3f test = lambda1 * m1_;

    x3D = T2w.inverse() * p3D1;
}

float squaredReprojectionError(cv::Point2f &p1, cv::Point2f &p2){
    float errx = p1.x - p2.x;
    float erry = p1.y - p2.y;

    return errx * errx + erry *erry;
}

Eigen::Matrix<float,3,3> computeEssentialMatrixFromPose(Sophus::SE3f& T12){
    EEigen::Matrix<float,3,3> E;

    // Extract Rotation matrix and translation vector from the pose
    Eigen::Matrix<float, 3, 3> R = T12.rotationMatrix();
    Eigen::Vector3f t = T12.translation();

    // Compute the skew-symmetric matrix representation of the translation vector
    Eigen::Matrix<float, 3, 3> t_x = Eigen::Matrix3f::Zero();
    t_x(0, 1) = -t(2); 
    t_x(1, 0) = t(2);
    t_x(0, 2) = t(1);
    t_x(2, 0) = -t(1);
    t_x(1, 2) = t(0); 
    t_x(2, 1) = t(1);

    // Compute the Essential Matrix 
    E = t_x * R;

    return E;
}