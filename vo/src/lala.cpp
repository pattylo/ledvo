/*
    This file is part of LEDVO - learning dynamic factor for visual odometry

    LEDVO is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LEDVO is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with LEDVO.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * \file lala.cpp
 * \date 06/11/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief trial cpp file to test GTSAM
 */

#include "sfmdata.h"

#include "include/tools/essential.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <vector>

#include <gtsam/nonlinear/FixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/inference/VariableIndex.h>


 #include <gtsam/nonlinear/BatchFixedLagSmoother.h>
 #include <gtsam/nonlinear/FixedLagSmoother.h>

 

#include <torch/torch.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    torch::Tensor tensor = torch::rand({2, 3});
  std::cout << tensor << std::endl;

    using namespace gtsam;
    using namespace std;

    // Define the camera calibration parameters
    Cal3_S2::shared_ptr K;
    
    K = std::make_shared<Cal3_S2>(50.0, 50.0, 0.0, 50.0, 50.0);
    // (new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

    

    // Define the camera observation noise model
    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

    // Create the set of ground-truth landmarks
    vector<Point3> points = createPoints();

    // Create the set of ground-truth poses
    vector<Pose3> poses = createPoses();

    // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
    // and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
    // and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
    // will approach the batch result.
    // ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    
    // ISAM2 isam(parameters);

    // // Create a Factor Graph and Values to hold the new data
    // NonlinearFactorGraph graph;
    // Values initialEstimate;

    // // Loop over the different poses, adding the observations to iSAM incrementally
    // for (size_t i = 0; i < poses.size(); ++i) 
    // {
    //     // Add factors for each landmark observation
    //     for (size_t j = 0; j < points.size(); ++j) 
    //     {
    //         PinholeCamera<Cal3_S2> camera(poses[i], *K);
    //         // SimpleCamera camera(poses[i], *K);

    //         Point2 measurement = camera.project(points[j]);

    //         graph.push_back(
    //             GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
    //                 measurement, 
    //                 measurementNoise, 
    //                 Symbol('x', i), 
    //                 Symbol('l', j), 
    //                 K
    //             )
    //         );                
    //     }

    //     cout<<endl;

    //     // Add an initial guess for the current pose
    //     // Intentionally initialize the variables off from the ground truth
    //     initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));


    //     // If this is the first iteration, add a prior on the first pose to set the coordinate frame
    //     // and a prior on the first landmark to set the scale
    //     // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
    //     // adding it to iSAM.
    //     if( i == 0) 
    //     {
    //         // Add a prior on pose x0
    //         noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    //         graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));

    //         // Add a prior on landmark l0
    //         noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    //         graph.push_back(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph

    //         // Add initial guesses to all observed landmarks
    //         // Intentionally initialize the variables off from the ground truth
    //         for (size_t j = 0; j < points.size(); ++j)
    //             initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));

    //     } 
    //     else 
    //     {
    //         // Update iSAM with the new factors
    //         // parameters.ve
    //         std::cout<<"here is the graph size: "<<isam.size()<<std::endl;

    //         isam.update(graph, initialEstimate);
    //         std::cout<<"before error: "<<isam.getFactorsUnsafe().error(isam.calculateEstimate())<<std::endl;

    //         while(isam.getFactorsUnsafe().error(isam.calculateEstimate()) > 1e-4)
    //         {
    //             std::cout<<isam.getFactorsUnsafe().size()<<std::endl;
    //             isam.update();                
    //         }
                
    //         std::cout<<"after error: "<<isam.getFactorsUnsafe().error(isam.calculateEstimate())<<std::endl;
    //         // isam.update()
    //         // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
    //         // If accuracy is desired at the expense of time, update(*) can be called additional times
    //         // to perform multiple optimizer iterations every step.
    //         // isam.
            
    //         Values currentEstimate = isam.calculateEstimate();
            
    //         // cout << "****************************************************" << endl;
    //         // cout << "Frame " << i << ": " << endl;
    //         // currentEstimate.print("Current estimate: ");
    //         cout<<endl<<endl;

    //         // Clear the factor graph and values for the next iteration
    //         graph.resize(0);
    //         initialEstimate.clear();
    //     }
    // }

    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    // double lag = 1.0;
    // gtsam::IncrementalFixedLagSmoother slw(lag, parameters);
    // initialEstimate.clear();

    // bool slw_start = false;
    // Values new_estimates;
    // Values lm_estimates;

    // FixedLagSmoother::KeyTimestampMap newTimestamps_slw;
    
    // // loop over the each pose at time t;
    // for(size_t k = 0; k < poses.size(); k++)
    // {

    // // initial estimate and registration
        
    //     // register xk
    //     new_estimates.insert(Symbol('x', k), poses[k].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
        
    //     for(size_t j = 0; j < points.size(); ++j)
    //     {
    //         if(!lm_estimates.exists(Symbol('l',j)))
    //         {
    //             new_estimates.insert<Point3>(
    //                 Symbol('l',j),
    //                 points[j] + Point3(-0.25, 0.20, 0.15)
    //             );

    //             lm_estimates.insert<Point3>(
    //                 Symbol('l',j),
    //                 points[j] + Point3(-0.25, 0.20, 0.15)
    //             );
    //         }
    //     }


    // // local graph factor
    //     // prior factor for x0
    //     if(k == 0)
    //     {
    //         // Add a prior on pose x0
    //         noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    //         graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));
    //         newTimestamps_slw.emplace(Symbol('x',0), k*0.5);

    //         // // Add a prior on landmark l0
    //         noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    //         graph.push_back(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph
    //         // graph.
    //         newTimestamps_slw.emplace(Symbol('l',0), k*0.5);
    //     }

    //     // xk with landmarks factor
    //     for(size_t j = 0; j < points.size(); ++j)
    //     {
    //         PinholeCamera<Cal3_S2> camera(poses[k], *K);
    //         // this will be the points that
    //         Point2 measurement = camera.project(points[j]);

    //         graph.push_back(
    //             GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
    //                 measurement, 
    //                 measurementNoise, 
    //                 Symbol('x', k), 
    //                 Symbol('l', j), 
    //                 K
    //             )
    //         );

    //         newTimestamps_slw.emplace(Symbol('x',k), k * 0.5);
    //         newTimestamps_slw.emplace(Symbol('l',j), k * 0.5);
    //     }

    //     if(k > 0)
    //     {
    //         std::cout<<"time: "<<k * 0.5<<std::endl;

            

    //         slw.update(
    //             graph,
    //             new_estimates,
    //             newTimestamps_slw
    //         );

    //         std::cout<<"before error: "<<slw.getFactors().error(slw.calculateEstimate())<<std::endl;
            
    //         while(slw.getFactors().error(slw.calculateEstimate()) > 1e-4)
    //         {
    //             std::cout<<slw.getFactors().size()<<std::endl;
    //             std::cout<<slw.getISAM2().getFactorsUnsafe().size()<<std::endl;;
    //             std::cout<<slw.calculateEstimate().size()<<std::endl;

                
                
    //             slw.update();                
    //         }

    //         std::cout<<"before error: "<<slw.getFactors().error(slw.calculateEstimate())<<std::endl;

            
    //         // slw.calculateEstimate().error();
            
    //         // for(auto what : slw.timestamps())
    //         //     std::cout<<what.second<<std::endl;

    //         std::cout<<slw.timestamps().size()<<std::endl;

    //         graph.resize(0);
    //         new_estimates.clear();
    //         newTimestamps_slw.clear();
    //     }
    // }
    // std::cout<<"END"<<std::endl;

    // Values currentEstimation = slw.calculateEstimate();
    // currentEstimation.print();

    // std::cout<<currentEstimation.at<Pose3>(Symbol('x', currentEstimation.size() - 2));


    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    LevenbergMarquardtParams LMparameters;

    // LMparameters.absoluteErrorTol = 1e-8;
    // LMparameters.relativeErrorTol = 1e-8;
    // LMparameters.maxIterations = 500;
    LMparameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    LMparameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    double lag = 2.5;
    BatchFixedLagSmoother batchsmoother(lag, LMparameters);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Values landmarkValues;
    FixedLagSmoother::KeyTimestampMap newTimestamps;

    for(size_t k = 0; k < poses.size(); k++)
    {
        for(size_t j = 0; j < points.size(); ++j)
        {
            PinholeCamera<Cal3_S2> camera(poses[k], *K);
            // this will be the points that
            Point2 measurement = camera.project(points[j]);

            newFactors.push_back(
                GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
                    measurement, 
                    measurementNoise, 
                    Symbol('x', k), 
                    Symbol('l', j), 
                    K
                )
            );

            if(!newValues.exists(Symbol('x',k)))
            {
                newValues.insert(
                    Symbol('x', k), 
                    poses[k].compose(
                        Pose3(
                            Rot3::Rodrigues(-0.1, 0.2, 0.25), 
                            Point3(0.05, -0.10, 0.20)
                        )
                    )
                );
            }

            if(!landmarkValues.exists(Symbol('l',j)))
            {
                landmarkValues.insert<Point3>(
                    Symbol('l',j),
                    points[j] + Point3(-0.25, 0.20, 0.15)
                );

                newValues.insert<Point3>(
                    Symbol('l',j),
                    points[j] + Point3(-0.25, 0.20, 0.15)
                );
            }

            newTimestamps[Symbol('x',k)] = k * 0.5;
            newTimestamps[Symbol('l',j)] = k * 0.5;
            
        }

        std::cout<<"lala\n"<<std::endl;

        if(k == 0)
        {
            noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
            newFactors.addPrior(Symbol('x', 0), poses[0], poseNoise);
            // newValues.insert(Symbol('x', 0), poses[0].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
            newTimestamps[Symbol('x', 0)] = 0.0;
            // newFactors.

            noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
            newFactors.addPrior(Symbol('l',0), points[0], pointNoise);
            // newValues.insert<Point3>(Symbol('l',0), points[0] + Point3(-0.25, 0.20, 0.15));
            newTimestamps[Symbol('l', 0)] = 0.0;

            std::cout<<"init!"<<std::endl;
        }
        else
        {
            std::cout<<"time: "<<k * 0.5<<std::endl;
            // newValues.print();

            

            FixedLagSmoother::Result lala = batchsmoother.update(
                newFactors,
                newValues,
                newTimestamps
            );
            
            

            batchsmoother.calculateEstimate<Pose3>(Symbol('x',k)).print("Batch Estimate:");
            // batchsmoother.calculateEstimate<Pose3>(Symbol('x',k)).tr
            batchsmoother.calculateEstimate().print();
            
            std::cout<<"current factor size: "<<batchsmoother.getFactors().size()<<std::endl;;
            std::cout<<"current batch size: "<<batchsmoother.calculateEstimate().size()<<std::endl;
            std::cout<<lala.getError()<<std::endl;
            // lala.

            newFactors.resize(0);
            newValues.clear();
            newTimestamps.clear();

        }

        

        std::cout<<"1 iteration"<<std::endl<<std::endl<<std::endl;

    }

    batchsmoother.calculateEstimate().print();





    return 0;
}