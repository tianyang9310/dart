/*************************************************************************
    > File Name: world_setup.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 10:49:26 AM EDT
 ************************************************************************/

#ifndef WORLD_SETUP_H
#define WORLD_SETUP_H 

#include "dart/dart.h"
namespace toyexample{

using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;



// parameter setting
const double default_density = 2e3;   // different density for cube and world_setup ?
const double floor_length    = 0.4;
const double floor_width     = 0.2;   // half of floor_length
const double floor_height    = 0.01;
const double wall_length     = 0.4;   // floor_length
const double wall_thickness  = 0.01;  // floor_height
const double wall_height     = 0.05;  // 1/8 * floor_length

const double obstacle_radius = 0.05;
const double obstacle_height = 0.025; // half of wall_height
const double obstacle_2_wall = 2.5*wall_thickness;

const double cube_length     = 0.005;


SkeletonPtr createFloor();

BodyNodePtr addWall(SkeletonPtr world_setup, BodyNodePtr parent, int wall_index);

BodyNodePtr addObstacle(SkeletonPtr world_setup, BodyNodePtr parent, int obstacle_index);

SkeletonPtr createCube();


} // namespace toyexample
#endif
