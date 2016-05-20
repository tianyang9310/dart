#include <iostream>
#include "dart/dart.h"
#include "MyWindow.h"
#include "WorldSetup.cpp"

using namespace dart;
using namespace dynamics;
using namespace simulation;
using namespace utils;

int main(int argc, char* argv[])
{
	WorldPtr mWorld = std::make_shared<World>();
	WorldSetup(mWorld);
  // create a window and link it to the world
  MyWindow window;
  window.setWorld(mWorld);

//  std::cout << "space bar: simulation on/off" << std::endl;
//  std::cout << "'p': playback/stop" << std::endl;
//  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
//  std::cout << "'v': visualization on/off" << std::endl;
//  std::cout << "'1'--'4': programmed interaction" << std::endl;
//  std::cout << "'w': move forward" << std::endl;
//  std::cout << "'s': stop" << std::endl;
//  std::cout << "'x': move backward" << std::endl;
//  std::cout << "'a': rotate steering wheels to left" << std::endl;
//  std::cout << "'d': rotate steering wheels to right" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Vehicle");
  glutMainLoop();

  return 0;
}
