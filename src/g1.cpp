#include "g1.h"
#include "config.h"

#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>
#include <RBDyn/parsers/urdf.h>

namespace mc_robots
{

inline static std::string g1Variant(const std::string & variant)
{
  std::string fullName = "g1_" + variant;
  mc_rtc::log::info("G1RobotModule uses the G1 variant: '{}'", fullName);
  return fullName;
}

G1RobotModule::G1RobotModule(const std::string & variant)
: RobotModule(mc_rtc::G1_DESCRIPTION_PATH, 
              g1Variant(variant),
              std::string(mc_rtc::G1_DESCRIPTION_PATH) + "/urdf/" + g1Variant(variant) + ".urdf")
{
  mc_rtc::log::success("G1RobotModule loaded with name: {}", name);
  rsdf_dir = std::string(mc_rtc::G1_DESCRIPTION_PATH) + "/rsdf";

  mc_rtc::log::success("G1RobotModule using URDF \"{}\"", urdf_path);
  mc_rtc::log::success("G1RobotModule using path \"{}\" for rsdf", rsdf_dir);

  // True if the robot has a fixed base, false otherwise
  bool fixed = false;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));

  _ref_joint_order = {
      "left_hip_pitch_joint",      "left_hip_roll_joint",       "left_hip_yaw_joint",
      "left_knee_joint",           "left_ankle_pitch_joint",    "left_ankle_roll_joint",
      "right_hip_pitch_joint",     "right_hip_roll_joint",      "right_hip_yaw_joint",
      "right_knee_joint",          "right_ankle_pitch_joint",   "right_ankle_roll_joint",
      "waist_yaw_joint"};

  if(variant == "29dof")
  {
    _ref_joint_order.push_back("waist_roll_joint");
    _ref_joint_order.push_back("waist_pitch_joint");
  }

  _ref_joint_order.insert(_ref_joint_order.end(), {
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",   "left_elbow_joint",          "left_wrist_roll_joint"});

  if(variant == "29dof")
  {
    _ref_joint_order.push_back("left_wrist_pitch_joint");
    _ref_joint_order.push_back("left_wrist_yaw_joint");
  }

  _ref_joint_order.insert(_ref_joint_order.end(), {
      "right_shoulder_pitch_joint","right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_joint",         "right_wrist_roll_joint"});

  if(variant == "29dof")
  {
    _ref_joint_order.push_back("right_wrist_pitch_joint");
    _ref_joint_order.push_back("right_wrist_yaw_joint");
  }

  using namespace mc_rtc::constants;
  _stance["left_hip_yaw_joint"] = {0.0};
  _stance["left_hip_roll_joint"] = {0.0};
  _stance["left_hip_pitch_joint"] = {-0.312};
  _stance["left_knee_joint"] = {0.669};
  _stance["left_ankle_pitch_joint"] = {-0.363};
  _stance["left_ankle_roll_joint"] = {0.0};
  _stance["right_hip_yaw_joint"] = {0.0};
  _stance["right_hip_roll_joint"] = {0.0};
  _stance["right_hip_pitch_joint"] = {-0.312};
  _stance["right_knee_joint"] = {0.669};
  _stance["right_ankle_pitch_joint"] = {-0.363};
  _stance["right_ankle_roll_joint"] = {0.0};
  _stance["waist_yaw_joint"] = {0.0};
  _stance["left_shoulder_pitch_joint"] = {0.2};
  _stance["left_shoulder_roll_joint"] = {0.2};
  _stance["left_shoulder_yaw_joint"] = {0.0};
  _stance["left_elbow_joint"] = {0.6};
  _stance["left_wrist_roll_joint"] = {0.0};
  _stance["right_shoulder_pitch_joint"] = {0.2};
  _stance["right_shoulder_roll_joint"] = {-0.2};
  _stance["right_shoulder_yaw_joint"] = {0.0};
  _stance["right_elbow_joint"] = {0.6};
  _stance["right_wrist_roll_joint"] = {0.0};

  if(variant == "29dof")
  {
    _stance["waist_roll_joint"] = {0.0};
    _stance["waist_pitch_joint"] = {0.0};
    _stance["left_wrist_pitch_joint"] = {0.0};
    _stance["left_wrist_yaw_joint"] = {0.0};
    _stance["right_wrist_pitch_joint"] = {0.0};
    _stance["right_wrist_yaw_joint"] = {0.0};
  }

  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.76}};

  // Add JointSensors for temperature/current logging
  for(size_t i = 0; i < _ref_joint_order.size(); ++i)
  {
    if(mb.jointIndexByName().count(_ref_joint_order[i]) != 0)
    {
      _jointSensors.push_back(mc_rbdyn::JointSensor(_ref_joint_order[i]));
    }
  }

  // Sensors
  _bodySensors.emplace_back("Accelerometer", "torso_link",
                            sva::PTransformd(Eigen::Vector3d(-0.03959, -0.00224, 0.13792)));
  _bodySensors.emplace_back("FloatingBase", "pelvis", sva::PTransformd::Identity());

  _minimalSelfCollisions = {mc_rbdyn::Collision("torso_link", "left_shoulder_yaw_link", 0.02, 0.001, 0.),
                            mc_rbdyn::Collision("torso_link", "right_shoulder_yaw_link", 0.02, 0.001, 0.),
                            mc_rbdyn::Collision("torso_link", "left_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("torso_link", "right_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis_contour_link", "left_shoulder_yaw_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis_contour_link", "right_shoulder_yaw_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis_contour_link", "left_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis_contour_link", "right_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("left_hip_pitch_link", "right_hip_pitch_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_knee_link", "right_knee_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_pitch_link", "right_ankle_pitch_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_roll_link_0", "right_ankle_roll_link_0", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_pitch_link", "right_knee_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("right_ankle_pitch_link", "left_knee_link", 0.02, 0.01, 0.)};
  _commonSelfCollisions = _minimalSelfCollisions;
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"G1", "G1_23dof", "G1_29dof"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("G1")
    if(n == "G1" || n == "G1_23dof")
    {
      return new mc_robots::G1RobotModule("23dof");
    }
    else if(n == "G1_29dof")
    {
      return new mc_robots::G1RobotModule("29dof");
    }
    else
    {
      mc_rtc::log::error("G1 module cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
