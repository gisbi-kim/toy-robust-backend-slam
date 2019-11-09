#include "ceres_error.h"

OdometryResidue::OdometryResidue
(double dx, double dy, double dtheta)
{
    this->dx = dx;
    this->dy = dy;
    this->dtheta = dtheta;

    // make a_Tcap_b
    {
      double cos_t = cos( this->dtheta );
      double sin_t = sin( this->dtheta );
      a_Tcap_b(0,0) = cos_t;
      a_Tcap_b(0,1) = -sin_t;
      a_Tcap_b(1,0) = sin_t;
      a_Tcap_b(1,1) = cos_t;
      a_Tcap_b(0,2) = this->dx;
      a_Tcap_b(1,2) = this->dy;
      a_Tcap_b(2,0) = 0.0;
      a_Tcap_b(2,1) = 0.0;
      a_Tcap_b(2,2) = 1.0;
  }
} // constructor, OdometryResidue


static ceres::CostFunction* 
OdometryResidue::Create
(const double dx, const double dy, const double dtheta)
{
    return 
    ( 
      new ceres::AutoDiffCostFunction<OdometryResidue, 3, 3, 3>
      (
        new OdometryResidue(dx, dy, dtheta)
      )
    );
} // Create, OdometryResidue


template <typename T>
bool 
OdometryResidue::operator() 
(const T* const P1, const T* const P2, T* e) const
{
    // Convert P1 to T1 ^w_T_a
    Eigen::Matrix<T,3,3> w_T_a;
    {
      T cos_t = T(cos( P1[2] ));
      T sin_t = T(sin( P1[2] ));
      w_T_a(0,0) = cos_t;
      w_T_a(0,1) = -sin_t;
      w_T_a(1,0) = sin_t;
      w_T_a(1,1) = cos_t;
      w_T_a(0,2) = P1[0];
      w_T_a(1,2) = P1[1];
      w_T_a(2,0) = T(0.0);
      w_T_a(2,1) = T(0.0);
      w_T_a(2,2) = T(1.0);
    }

    // Convert P2 to T2 ^w_T_a
    Eigen::Matrix<T,3,3> w_T_b;
    {
      T cos_t = cos( P2[2] );
      T sin_t = sin( P2[2] );
      w_T_b(0,0) = cos_t;
      w_T_b(0,1) = -sin_t;
      w_T_b(1,0) = sin_t;
      w_T_b(1,1) = cos_t;
      w_T_b(0,2) = P2[0];
      w_T_b(1,2) = P2[1];
      w_T_b(2,0) = T(0.0);
      w_T_b(2,1) = T(0.0);
      w_T_b(2,2) = T(1.0);
    }

    // cast from double to T
    Eigen::Matrix<T, 3, 3> T_a_Tcap_b;
    T_a_Tcap_b <<   T(a_Tcap_b(0,0)), T(a_Tcap_b(0,1)),T(a_Tcap_b(0,2)),
                    T(a_Tcap_b(1,0)), T(a_Tcap_b(1,1)),T(a_Tcap_b(1,2)),
                    T(a_Tcap_b(2,0)), T(a_Tcap_b(2,1)),T(a_Tcap_b(2,2));

    // now we have :: w_T_a, w_T_b and a_Tcap_b
    // compute pose difference
    Eigen::Matrix<T,3,3> diff = T_a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

    e[0] = diff(0,2);
    e[1] = diff(1,2);
    e[2] = asin( diff(1,0) );

    return true;
} // (), OdometryResidue


DCSClosureResidue::DCSClosureResidue 
(double dx, double dy, double dtheta )
{
    this->dx = dx;
    this->dy = dy;
    this->dtheta = dtheta;

    // make a_Tcap_b
    {
      double cos_t = cos( this->dtheta );
      double sin_t = sin( this->dtheta );
      a_Tcap_b(0,0) = cos_t;
      a_Tcap_b(0,1) = -sin_t;
      a_Tcap_b(1,0) = sin_t;
      a_Tcap_b(1,1) = cos_t;
      a_Tcap_b(0,2) = this->dx;
      a_Tcap_b(1,2) = this->dy;
      a_Tcap_b(2,0) = 0.0;
      a_Tcap_b(2,1) = 0.0;
      a_Tcap_b(2,2) = 1.0;
  }
}  // constructor, DCSClosureResidue


static ceres::CostFunction* 
DCSClosureResidue::Create
(const double dx, const double dy, const double dtheta)
{
  return 
  (
    new ceres::AutoDiffCostFunction<DCSClosureResidue, 3, 3, 3>
    (
      new DCSClosureResidue(dx, dy, dtheta)
    )
  );
} // Create, DCSClosureResidue


template <typename T> 
bool 
DCSClosureResidue::operator() 
(const T* const P1, const T* const P2, T* e) const
{
    // Convert P1 to T1 ^w_T_a
    Eigen::Matrix<T,3,3> w_T_a;
    {
      T cos_t = T(cos( P1[2] ));
      T sin_t = T(sin( P1[2] ));
      w_T_a(0,0) = cos_t;
      w_T_a(0,1) = -sin_t;
      w_T_a(1,0) = sin_t;
      w_T_a(1,1) = cos_t;
      w_T_a(0,2) = P1[0];
      w_T_a(1,2) = P1[1];
      w_T_a(2,0) = T(0.0);
      w_T_a(2,1) = T(0.0);
      w_T_a(2,2) = T(1.0);
    }

    // Convert P2 to T2 ^w_T_a
    Eigen::Matrix<T,3,3> w_T_b;
    {
        T cos_t = cos( P2[2] );
        T sin_t = sin( P2[2] );
        w_T_b(0,0) = cos_t;
        w_T_b(0,1) = -sin_t;
        w_T_b(1,0) = sin_t;
        w_T_b(1,1) = cos_t;
        w_T_b(0,2) = P2[0];
        w_T_b(1,2) = P2[1];
        w_T_b(2,0) = T(0.0);
        w_T_b(2,1) = T(0.0);
        w_T_b(2,2) = T(1.0);
    }

    // cast from double to T
    Eigen::Matrix<T, 3, 3> T_a_Tcap_b;
    T_a_Tcap_b <<   T(a_Tcap_b(0,0)), T(a_Tcap_b(0,1)),T(a_Tcap_b(0,2)),
                    T(a_Tcap_b(1,0)), T(a_Tcap_b(1,1)),T(a_Tcap_b(1,2)),
                    T(a_Tcap_b(2,0)), T(a_Tcap_b(2,1)),T(a_Tcap_b(2,2));

    // now we have :: w_T_a, w_T_b and a_Tcap_b
    // compute pose difference
    Eigen::Matrix<T,3,3> diff = T_a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

    // ... as far as here, same as the original residual

    // refer: http://www2.informatik.uni-freiburg.de/~spinello/agarwalICRA13.pdf
    float const_upper_bound = 0.5; // USER PARAMETER, notation phi at the original paper 
    T res = diff(0,2)*diff(0,2) + diff(1,2)*diff(1,2); // original error term for each loop closing constraint
    T psi_org = sqrt( T(2.0) * T(const_upper_bound) / ( T(const_upper_bound) + res ) );
    T psi = std::min( T(1.0), psi_org ) ;

    // if psi is constant (i.e., 1), this goes the original (not robust one) 
    e[0] = psi * diff(0,2);
    e[1] = psi * diff(1,2);
    e[2] = psi * asin( diff(1,0) );

    return true;
} // (), DCSClosureResidue