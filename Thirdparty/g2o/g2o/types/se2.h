// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_SE2_H_
#define G2O_SE2_H_

#include "../stuff/misc.h"
#include "g2o_types_slam2d_api.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/eigen_types.h>
#include<opencv2/core/core.hpp>

using namespace Eigen;
namespace g2o {

  /**
   * \brief represent SE2
   */
  class G2O_TYPES_SLAM2D_API SE2 {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      SE2():_R(0),_t(0,0){}

      SE2(const Isometry2D& iso): _R(0), _t(iso.translation()){
        _R.fromRotationMatrix(iso.linear());
      }

      SE2(const Vector3D& v):_R(v[2]),_t(v[0],v[1]){}

      SE2(double x, double y, double theta):_R(theta),_t(x,y){}

      //! translational component
      inline const Vector2D& translation() const {return _t;}
      void setTranslation(const Vector2D& t_) {_t=t_;}

      //! rotational component
      inline const Eigen::Rotation2Dd& rotation() const {return _R;}
      void setRotation(const Eigen::Rotation2Dd& R_) {_R=R_;}

      //! concatenate two SE2 elements (motion composition)
      inline SE2 operator * (const SE2& tr2) const{
        SE2 result(*this);
        result *= tr2;
        return result;
      }

      //! motion composition operator
      inline SE2& operator *= (const SE2& tr2){
        _t+=_R*tr2._t;
        _R.angle()+=tr2._R.angle();
        _R.angle()=normalize_theta(_R.angle());
        return *this;
      }

      //! project a 2D vector
      inline Vector2D operator * (const Vector2D& v) const {
        return _t+_R*v;
      }

      inline SE2 operator +(const SE2& that) const{
          double c = std::cos(_R.angle());
          double s = std::sin(_R.angle());
          double _x = _t.x() + that._t.x()*c - that._t.y()*s;
          double _y = _t.y() + that._t.x()*s + that._t.y()*c;
          double _theta = normalize_theta(_R.angle() + that._R.angle());
          SE2 result(_x, _y, _theta);
          return result;
      }

      inline SE2 operator -(const SE2& that) const{
          double dx = _t.x() - that._t.x();
          double dy = _t.y() - that._t.y();
          double dth = normalize_theta(_R.angle() - that._R.angle());

          double c = std::cos(that._R.angle());
          double s = std::sin(that._R.angle());
          SE2 result(c*dx+s*dy, -s*dx+c*dy, dth);
          return result;
      }

      //! invert :-)
      inline SE2 inverse() const{
        SE2 ret;
        ret._R=_R.inverse();
        ret._R.angle()=normalize_theta(ret._R.angle());
#ifdef _MSC_VER
        ret._t=ret._R*(Vector2D(_t*-1.));
#else
        ret._t=ret._R*(_t*-1.);
#endif
        return ret;
      }

      inline double operator [](int i) const {
        assert (i>=0 && i<3);
        if (i<2)
          return _t(i);
        return _R.angle();
      }


      //! assign from a 3D vector (x, y, theta)
      inline void fromVector (const Vector3D& v){
        *this=SE2(v[0], v[1], v[2]);
      }

      //! convert to a 3D vector (x, y, theta)
      inline Vector3D toVector() const {
        return Vector3D(_t.x(), _t.y(), _R.angle());
      }

      inline Isometry2D toIsometry() const {
        Isometry2D iso = Isometry2D::Identity();
        iso.linear() = _R.toRotationMatrix();
        iso.translation() = _t;
        return iso;
      }

      inline cv::Mat toCvSE3() const
      {
        float theta = _R.angle();
        float x = _t.x();
        float y = _t.y();
        float c = cos(theta);
        float s = sin(theta);

        return (cv::Mat_<float>(4,4) <<
                c,-s, 0, x,
                s, c, 0, y,
                0, 0, 1, 0,
                0, 0, 0, 1);
      }
      static inline g2o::SE2& fromCvSE3(const cv::Mat &mat)
      {
        float yaw = std::atan2(mat.at<float>(1,0), mat.at<float>(0,0));
        double theta = normalize_theta(yaw);
        double x = mat.at<float>(0,3);
        double y = mat.at<float>(1,3);
        SE2 result = SE2(x,y,theta);
        return result;
      }

    protected:
      Eigen::Rotation2Dd _R;
      Vector2D _t;
  };

} // end namespace

#endif
