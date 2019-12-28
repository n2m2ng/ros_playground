#include <crane_x7_control_kyun/hardware_interface.cpp>
#include <crane_x7_control_kyun/dxlport_control.cpp>

main()
{
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);

  while (true)
  {
     robot.read();
     cm.update(robot.get_time(), robot.get_period());
     robot.write();
     sleep();
  }
}

catkin_package
(
  CATKIN_DEPENDS
)
