/**
* This file is part of se2lam
*
* Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG (github.com/hbtang)
*/

#include "EdgeSE2XYZ.h"
#include <cmath>
#include "Thirdparty/g2o/g2o/types/isometry3d_mappings.h"

namespace g2o
{
using namespace std;
using namespace Eigen;

Vector2d project2d(const Vector3d& v)
{
    Vector2d res;
    res(0) = v(0)/v(2);
    res(1) = v(1)/v(2);
    return res;
}

Vector3d unproject2d(const Vector2d& v)
{
    Vector3d res;
    res(0) = v(0);
    res(1) = v(1);
    res(2) = 1;
    return res;
}

Eigen::Matrix3d d_inv_d_se2(const SE2& _se2)
{
    double c = std::cos(_se2.rotation().angle());
    double s = std::sin(_se2.rotation().angle());
    double x = _se2.translation()(0);
    double y = _se2.translation()(1);
    Matrix3d ret;
    ret << -c, -s, s*x - c*y, s, -c, c*x + s*y, 0, 0, -1;
    return ret;
}

g2o::SE3Quat SE2ToSE3(const g2o::SE2& _se2)
{
    SE3Quat ret;
    ret.setTranslation(Eigen::Vector3d(_se2.translation()(0), _se2.translation()(1), 0));
    ret.setRotation(Eigen::Quaterniond(AngleAxisd(_se2.rotation().angle(), Vector3d::UnitZ())));
    return ret;
}

g2o::SE2 SE3ToSE2(const SE3Quat &_se3)
{
    Eigen::Vector3d eulers = g2o::internal::toEuler(_se3.rotation().matrix());
    return g2o::SE2(_se3.translation()(0), _se3.translation()(1), eulers(2));
}


EdgeSE2XYZ::EdgeSE2XYZ()
{
}

EdgeSE2XYZ::~EdgeSE2XYZ()
{
}


bool EdgeSE2XYZ::read(std::istream &is)
{
    return true;
}

bool EdgeSE2XYZ::write(std::ostream &os) const
{
    return true;
}

Vector2d EdgeSE2XYZ::cam_project(const Vector3d & trans_xyz) const{
    Vector2d proj = project2d(trans_xyz);
    Vector2d res;
    res[0] = proj[0] * fx + cx;
    res[1] = proj[1] * fy + cy;
    return res;
}

void EdgeSE2XYZ::computeError()
{
    VertexSE2* v1 = static_cast<VertexSE2*>(_vertices[1]);
    VertexSBAPointXYZ* v2 = static_cast<VertexSBAPointXYZ*>(_vertices[0]);

    SE3Quat Tbw = SE2ToSE3(v1->estimate().inverse());
    SE3Quat Tcw = Tcb * Tbw;

    Vector3d lc = Tcw.map(v2->estimate());

    Vector2d obs(_measurement);
    _error = cam_project(lc) - obs;
    
}

bool EdgeSE2XYZ::isDepthPositive(){
    
    const VertexSE2* v1 = static_cast<VertexSE2*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<VertexSBAPointXYZ*>(_vertices[0]);

    return (SE2ToSE3(v1->estimate()).map(v2->estimate()))(2) > 0.0;
}

void EdgeSE2XYZ::linearizeOplus()
{
    VertexSE2* vj = static_cast<VertexSE2*>(_vertices[1]);
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);

    Vector3d vwb = vj->estimate().toVector();

    SE3Quat Tcw = Tcb * SE2ToSE3(vj->estimate().inverse());
    
    Matrix3d Rcw = Tcw.rotation().toRotationMatrix();
    Vector3d tcw = Tcw.translation();

    SE3Quat Twc(Tcw.inverse());
    Matrix3d Rwc = Twc.rotation().toRotationMatrix();
    Vector3d twc = Twc.translation();

    Vector3d pi(vwb[0], vwb[1], 0);

    Vector3d lw = vi->estimate();
    Vector3d lc = Tcw.map(lw);
    double zc = lc(2);
    double zc_inv = 1. / zc;
    double zc_inv2 = zc_inv * zc_inv;

    Matrix23d J_pi;
    J_pi << fx * zc_inv, 0, -fx*lc(0)*zc_inv2,
            0, fx * zc_inv, -fx*lc(1)*zc_inv2;

    Matrix23d J_pi_Rcw = J_pi * Rcw;

    _jacobianOplusXi.block<2,2>(0,0) = -J_pi_Rcw.block<2,2>(0,0);
    _jacobianOplusXi.block<2,1>(0,2) = (J_pi_Rcw * skew(lw-pi)).block<2,1>(0,2);

    _jacobianOplusXj = J_pi_Rcw;

}

// Only Pose - motion-only BA
EdgeSE2XYZOnlyPose::EdgeSE2XYZOnlyPose()
{
}

EdgeSE2XYZOnlyPose::~EdgeSE2XYZOnlyPose()
{
}

bool EdgeSE2XYZOnlyPose::read(std::istream &is)
{
    return true;
}

bool EdgeSE2XYZOnlyPose::write(std::ostream &os) const
{
    return true;
}

Vector2d EdgeSE2XYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
    Vector2d proj = project2d(trans_xyz);
    Vector2d res;
    res[0] = proj[0] * fx + cx;
    res[1] = proj[1] * fy + cy;
    return res;
}

void EdgeSE2XYZOnlyPose::computeError(){
    const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
    
    SE3Quat Tbw = SE2ToSE3(v1->estimate().inverse());
    SE3Quat Tcw = Tcb * Tbw;

    Vector2d obs(_measurement);
    Vector3d lc = Tcw.map(Xw);
    _error = cam_project(lc) - obs;
}

bool EdgeSE2XYZOnlyPose::isDepthPositive(){
    const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
    return (SE2ToSE3(v1->estimate().inverse()).map(Xw))(2) > 0.0;
}

void EdgeSE2XYZOnlyPose::linearizeOplus(){
    VertexSE2* vi = static_cast<VertexSE2 *>(_vertices[0]);
    
    Vector3d vwb = vi->estimate().toVector();

    SE3Quat Tcw = Tcb * SE2ToSE3(vi->estimate().inverse());
    
    Matrix3d Rcw = Tcw.rotation().toRotationMatrix();
    Vector3d tcw = Tcw.translation();

    SE3Quat Twc(Tcw.inverse());
    Matrix3d Rwc = Twc.rotation().toRotationMatrix();

    Vector3d pi(vwb[0], vwb[1], 0);

    Vector3d lw = Xw;
    Vector3d lc = Tcw.map(Xw);
    double zc = lc(2);
    double zc_inv = 1. / zc;
    double zc_inv2 = zc_inv * zc_inv;

    Matrix23d J_pi;
    J_pi << fx * zc_inv, 0, -fx*lc(0)*zc_inv2,
            0, fx * zc_inv, -fx*lc(1)*zc_inv2;

    Matrix23d J_pi_Rcw = J_pi * Rcw;

    _jacobianOplusXi.block<2,2>(0,0) = -J_pi_Rcw.block<2,2>(0,0);
    _jacobianOplusXi.block<2,1>(0,2) = (J_pi_Rcw * skew(lw-pi)).block<2,1>(0,2);

}

}
