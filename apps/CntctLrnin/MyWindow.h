#ifndef CNTCTLRNIN_MYWINDOW_H
#define CNTCTLRNIN_MYWINDOW_H

#include "dart/dart.h"
#include "MyDantzigLCPSolver.h"
#include "Controller.h"

class MyWindow : public dart::gui::SimWindow
{
public:
    MyWindow(dart::simulation::WorldPtr world);
    virtual ~MyWindow();
    
    
    void addExtForce();
    
    void timeStepping() override;
    void keyboard(unsigned char _key, int _x, int _y) override;
    void drawSkels() override;
    
private:
    std::unique_ptr<Controller> mController;
    std::unique_ptr<dart::collision::CollisionDetector> mCollisionDetector;
};

#endif  // CNTCTLRNIN_MYWINDOW_H
