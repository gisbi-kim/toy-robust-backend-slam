#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "ceres_error.h"
#include "g2o_util.h"
#include "graph.h"

using std::cout;
using std::endl;
using std::string;

string BASE_PATH = std::string( "../data");
string SAVE_PATH = std::string( "../save");

/*
DATASET_NAME_WITHOUGH_DOTG2O: {INTEL, M3500, M3500b, M3500c, CSAIL, FR079, FRH, M10000}

 * How to use
 * $ ./main DATASET_NAME_WITHOUGH_DOTG2O NUM_OUTLIER_LOOPS DSC_ON
 * - e.g., $ ./main INTEL 50 1 # USING DCS
 * - e.g., $ ./main INTEL 50 0 # NOT USING DCS
*/
auto main(int argc, char *argv[]) ->int
{
    // @ random seed change for tests
    std::srand((unsigned int) time(0)); // for random bogus add 

    // @ Read g2o file and add noise edges
    string fname{argv[1]};
    string fpath = BASE_PATH + "/" + fname + ".g2o";
    cout << "Start Reading PoseGraph\n";
    ReadG2O g2o_manager( fpath );

    int num_bogus_loops{atoi(argv[2])};
    g2o_manager.add_random_C(num_bogus_loops); // adding bogus (false) loops

    bool DCS_ON{atoi(argv[3])};

    g2o_manager.writePoseGraph_nodes(SAVE_PATH+"/init_nodes.txt");
    g2o_manager.writePoseGraph_edges(SAVE_PATH+"/init_edges.txt");
    cout << "total nodes : "<< g2o_manager.nNodes.size() << endl;
    cout << "total nEdgesOdometry : "<< g2o_manager.nEdgesOdometry.size() << endl;
    cout << "total nEdgesClosure : "<< g2o_manager.nEdgesClosure.size() << endl;
    cout << "total nEdgesBogus : "<< g2o_manager.nEdgesBogus.size() << endl;

    // @ Make a cost function 
    ceres::Problem problem;
    ceres::LossFunction * loss_function = NULL;
    loss_function = new ceres::HuberLoss(0.01); // robust kernel

    // @ 1. Odometry Constraints
    for( int i=0 ; i<g2o_manager.nEdgesOdometry.size() ; i++ )
    {
        Edge* ed = g2o_manager.nEdgesOdometry[i];
        ceres::CostFunction * cost_function = OdometryResidue::Create( ed->x, ed->y, ed->theta );
        problem.AddResidualBlock( cost_function, loss_function, ed->a->p, ed->b->p );
    }

    // @ 2. Loop Closure Constaints 
    // - see DCSClosureResidue for DCS
    for( int i=0 ; i<g2o_manager.nEdgesClosure.size() ; i++ )
    {   // for clean edges
        Edge* ed = g2o_manager.nEdgesClosure[i];
        ceres::CostFunction * cost_function;
        if (DCS_ON) cost_function = DCSClosureResidue::Create( ed->x, ed->y, ed->theta );
        if (! DCS_ON) cost_function = OdometryResidue::Create( ed->x, ed->y, ed->theta );
        problem.AddResidualBlock( cost_function, loss_function, ed->a->p, ed->b->p );
    } 
    for( int i=0 ; i<g2o_manager.nEdgesBogus.size() ; i++ )
    {   // for bogus edges
        Edge* ed = g2o_manager.nEdgesBogus[i];
        ceres::CostFunction * cost_function;
        if (DCS_ON) cost_function = DCSClosureResidue::Create( ed->x, ed->y, ed->theta );
        if (! DCS_ON) cost_function = OdometryResidue::Create( ed->x, ed->y, ed->theta );
        problem.AddResidualBlock( cost_function, loss_function, ed->a->p, ed->b->p );
    }

    // @ Solve 
    problem.SetParameterBlockConstant(g2o_manager.nNodes[0]->p); // i.e., 1st pose be origin
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    // options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    // options.preconditioner_type = ceres::SCHUR_JACOBI;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    // @ Write Pose Graph file after Optimization
    g2o_manager.writePoseGraph_nodes(SAVE_PATH+"/opt_nodes.txt");
    g2o_manager.writePoseGraph_edges(SAVE_PATH+"/opt_edges.txt");

} // End main 
