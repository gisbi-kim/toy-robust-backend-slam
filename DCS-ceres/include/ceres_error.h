#ifndef CERES_ERROR_H
#define CERES_ERROR_H

#include <algorithm>  
#include <vector>
#include <Eigen/Dense>
#include <ceres/ceres.h>

struct OdometryResidue
{
public:
    OdometryResidue(double dx, double dy, double dtheta);
    template <typename T> bool operator()(const T* const P1, const T* const P2, T* e) const;
    static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta);

public:
    double dx;
    double dy;
    double dtheta;
    Eigen::Matrix<double,3,3> a_Tcap_b;
};


struct DCSClosureResidue
{
public:
    DCSClosureResidue (double dx, double dy, double dtheta );
    template <typename T> bool operator() (const T* const P1, const T* const P2, T* e) const;
    static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta);

public:
    double dx;
    double dy;
    double dtheta;
    Eigen::Matrix<double,3,3> a_Tcap_b;
};

#endif // CERES_ERROR_H