#ifndef CERES_ERROR_H
#define CERES_ERROR_H

#include <algorithm>  
#include <vector>
#include <Eigen/Dense>
#include <ceres/ceres.h>

struct OdometryResidue
{
    OdometryResidue(double dx, double dy, double dtheta);
    static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta);
    
    // residual
    template <typename T>
    bool operator()(const T* const P1, const T* const P2, T* e) const;

    // var
    double dx;
    double dy;
    double dtheta;
    Eigen::Matrix<double,3,3> a_Tcap_b;

};


// DCS residual (Dynamic Covariance Scaling)
// - (Template body should be defined in a header) 
struct DCSClosureResidue
{
    // Observation for the edge
    DCSClosureResidue(double dx, double dy, double dtheta )
    {
        this->dx = dx;
        this->dy = dy;
        this->dtheta = dtheta;
        this->s_cap = drand48() * .1 + .9;

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
    }

    // Define the residue for each edge. P1 and P2 are 3-vectors representing state of the node ie. x,y,theta
    template <typename T>
    bool operator()(const T* const P1, const T* const P2, T* e) const
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

        // psi - scalar (covariance term. See the paper on DCS for derivation)
        // T psi = T(1.0) / (T(1.0) + exp( T(-2.0)*s[0] ));
        // T psi = max( T(0.0), min( T(1.0), s[0] ) );

        T res = diff(0,2)*diff(0,2) + diff(1,2)*diff(1,2); // + asin( diff(1,0) )*asin( diff(1,0) );
        // T psi_org = T(.3) * T(s_cap) / ( T(1.0) + res ) ;
        T psi_org = sqrt( T(2.0) * T( .5 ) / ( T(.5) + res ) );
        // e[0] = psi ;
        // e[1] = T(0.0);
        // e[2] = T(0.0);
        // return true;
        T psi = std::min( T(1.0), psi_org ) ;

        e[0] = psi*diff(0,2);
        e[1] = psi*diff(1,2);
        e[2] = psi*asin( diff(1,0) );

        return true;
    }

    double dx;
    double dy;
    double dtheta;
    double s_cap;
    Eigen::Matrix<double,3,3> a_Tcap_b;

    static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta){
        return (new ceres::AutoDiffCostFunction<DCSClosureResidue, 3, 3, 3>(
            new DCSClosureResidue(dx, dy, dtheta)));
    };

};


#endif // CERES_ERROR_H