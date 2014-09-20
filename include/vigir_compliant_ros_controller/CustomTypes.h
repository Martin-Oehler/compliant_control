//=================================================================================================
// Copyright (c) 2013, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of TORC Robotics nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef CUSTOM_TYPES_H__
#define CUSTOM_TYPES_H__

// Define some basic types to replace ROS specific messages

// Define Eigen setup used by rbdl
#define EIGEN_DEFAULT_TO_ROW_MAJOR
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

namespace compliant_controller {

typedef Eigen::VectorXd             VectorNd;
typedef Eigen::MatrixXd             MatrixNd;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Vector3d             Vector3d;
typedef Eigen::Vector2d             Vector2d;
typedef Eigen::Quaternion<double>   Quatd;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian;

typedef struct CartState {
    CartState() {
        position = Vector6d::Zero();
        velocity = Vector6d::Zero();
    }
    Vector6d position;
    Vector6d velocity;
} CartState;

typedef struct Wrench
{
    Wrench() : force(0.0, 0.0, 0.0), torque(0.0, 0.0,0.0) {}
    Vector3d    force;
    Vector3d    torque;
} Wrench;

typedef struct IMU
{
    IMU() : orientation(1.0,0.0,0.0,0.0),angular_velocity(0.0,0.0,0.0),linear_acceleration(0.0,0.0,0.0){}

    Quatd    orientation;
    Vector3d angular_velocity;
    Vector3d linear_acceleration;

} IMU;

typedef struct Pose
{
    Pose() : position(0.0,0.0,0.0),orientation(1.0,0.0,0.0,0.0){}

    Vector3d position;
    Quatd    orientation;
} Pose;

typedef struct Transform
{
    Transform() :translation(0.0,0.0,0.0){ rotation.setIdentity();}

    Vector3d translation;
    Matrix3d rotation;

    Vector3d operator *(const Vector3d& rhs) const
    {
        return rotation*rhs + translation;
    }

} Transform;

typedef std::vector<Vector3d>  Polygon;
}
#endif


