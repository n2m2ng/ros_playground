#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
 {
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("base_to_2nd", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_base_to_2nd);

   hardware_interface::JointStateHandle state_handle_b("2nd_to_3rd", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_2nd_to_3rd);

   hardware_interface::JointStateHandle state_handle_b("3rd_to_4th", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_3rd_to_4th);

   hardware_interface::JointStateHandle state_handle_b("4th_to_5th", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_4th_to_5th);

   hardware_interface::JointStateHandle state_handle_b("5th_to_6th", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_5th_to_6th);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("base_to_2nd"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_base_to_2nd);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("2nd_to_3rd"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_2nd_to_3rd);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("3rd_to_4th"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_3rd_to_4th);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("4th_to_5th"), &cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_4th_to_5th);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("5th_to_6th"), &cmd[4]);
   jnt_pos_interface.registerHandle(pos_handle_5th_to_6th);

   registerInterface(&jnt_pos_interface);
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};
