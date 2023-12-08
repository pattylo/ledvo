/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMMdata.h
 * @brief   Simple example for the structure-from-motion problems
 * @author  Duy-Nguyen Ta
 */

/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 */

// As this is a full 3D problem, we will use Pose3 variables to represent the camera
// positions and Point3 variables (x, y, z) to represent the landmark coordinates.
// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
// We will also need a camera object to hold calibration information and perform projections.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <gtsam/geometry/Cal3_S2.h>


// We will also need a camera object to hold calibration information and perform projections.
#include <gtsam/geometry/PinholeCamera.h>

/* ************************************************************************* */
std::vector<gtsam::Point3> createPoints() {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

  return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses() {

  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;
  double radius = 30.0;
  gtsam::Point3 up(0, 0, 1);
  gtsam::Point3 target(0, 0, 0);

  for (int i = 0; i < 16; ++i) {
    double theta = i * 2 * M_PI / 8;
    gtsam::Point3 position = gtsam::Point3(radius * cos(theta), radius * sin(theta), 0.0);
    gtsam::PinholeCamera camera = gtsam::PinholeCamera<gtsam::Cal3_S2>::Lookat(position, target, up);

    poses.push_back(camera.pose());
  }

  return poses;
}
/* ************************************************************************* */