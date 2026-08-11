#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

using namespace std::chrono_literals;

mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

double target_omega[4]  = {0.0, 0.0, 0.0, 0.0};
double current_omega[4] = {0.0, 0.0, 0.0, 0.0};

// Motor constants from x500 sdf
static const double kMotorConstant          = 8.54858e-06;
static const double kMomentConstant         = 0.016;
static const double kTimeConstantUp         = 0.0125;
static const double kTimeConstantDown       = 0.025;
static const double kMaxRotVelocity         = 1000.0;
static const double kRotorDragCoefficient   = 8.06428e-05;
static const double kRollingMomentCoefficient = 1e-06;
static const double kRotorVelocitySlowdownSim = 10.0;

// Motor spin and torque directions
static const double kSpinDir[4]  = { 1.0,  1.0, -1.0, -1.0 };
static const double kReactDir[4] = { -1.0, -1.0,  1.0,  1.0 };

// Mouse vars
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

void multicopter_motor_callback(const mjModel* m, mjData* d) {
  // Clear forces to avoid accum
  mju_zero(d->qfrc_applied, m->nv);

  // Check for base_link
  double dt = m->opt.timestep;
  int base_id = mj_name2id(m, mjOBJ_BODY, "base_link");
  if (base_id < 0) return;

  Eigen::Vector3d wind_vel_world(0.0, 0.0, 0.0);

  for (int i = 0; i < 4; i++) {
    // Check for rotor
    std::string rotor_name = "rotor_" + std::to_string(i);
    std::string joint_name = rotor_name + "_joint";
    int rotor_id = mj_name2id(m, mjOBJ_BODY, rotor_name.c_str());
    int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
    if (rotor_id < 0 || joint_id < 0) continue;

    int dof_idx = m->jnt_dofadr[joint_id];

    // First order filter from original plugin
    double refMotorInput = target_omega[i];
    if (refMotorInput < 0.0) refMotorInput = 0.0;
    if (refMotorInput > kMaxRotVelocity) refMotorInput = kMaxRotVelocity;
    double cur = current_omega[i];
    double tau = (refMotorInput > cur) ? kTimeConstantUp : kTimeConstantDown;
    double alpha = std::exp(-dt / tau);
    double refMotorRotVel = alpha * cur + (1.0 - alpha) * refMotorInput;
    current_omega[i] = refMotorRotVel;

    // Velocity sign correct
    double motorRotVel = d->qvel[dof_idx];
    double realMotorVelocity = motorRotVel * kRotorVelocitySlowdownSim;
    int realMotorVelocitySign = (realMotorVelocity > 0) - (realMotorVelocity < 0);

    // 3. World Kinematics
    mjtNum rotor_vel[6];
    mj_objectVelocity(m, d, mjOBJ_BODY, rotor_id, rotor_vel, 0);
    Eigen::Vector3d body_lin_vel_world(rotor_vel[3], rotor_vel[4], rotor_vel[5]);

    mjtNum axis_local[3] = {0, 0, 1};
    mjtNum axis_world_raw[3];
    mju_mulMatVec3(axis_world_raw, d->xmat + 9 * rotor_id, axis_local);
    Eigen::Vector3d axis_world(axis_world_raw[0], axis_world_raw[1], axis_world_raw[2]);
    axis_world.normalize();

    // Compute thrust
    double thrust = kSpinDir[i] * realMotorVelocitySign * (realMotorVelocity * realMotorVelocity) * kMotorConstant;
    Eigen::Vector3d thrust_force_world = thrust * axis_world;

    // Get wind efffects
    Eigen::Vector3d relative_wind_vel = body_lin_vel_world - wind_vel_world;
    Eigen::Vector3d v_perp = relative_wind_vel - (relative_wind_vel.dot(axis_world) * axis_world);

    Eigen::Vector3d air_drag = -std::abs(realMotorVelocity) * kRotorDragCoefficient * v_perp;
    Eigen::Vector3d rolling_moment = -std::abs(realMotorVelocity) * kRollingMomentCoefficient * v_perp;

    // Compute on-rotor force and torque
    Eigen::Vector3d rotor_force = thrust_force_world + air_drag;
    mjtNum force_rotor_global[3] = {rotor_force.x(), rotor_force.y(), rotor_force.z()};
    mjtNum torque_rotor_global[3] = {0, 0, 0};
    mjtNum point_rotor_global[3] = {d->xpos[3 * rotor_id], d->xpos[3 * rotor_id + 1], d->xpos[3 * rotor_id + 2]};

    mj_applyFT(m, d, force_rotor_global, torque_rotor_global, point_rotor_global, rotor_id, d->qfrc_applied);

    Eigen::Vector3d drag_torque_world = -kSpinDir[i] * thrust * kMomentConstant * axis_world;
    Eigen::Vector3d parent_torque = drag_torque_world + rolling_moment;

    mjtNum force_parent_global[3] = {0, 0, 0};
    mjtNum torque_parent_global[3] = {parent_torque.x(), parent_torque.y(), parent_torque.z()};
    mjtNum point_parent_global[3] = {d->xpos[3 * base_id], d->xpos[3 * base_id + 1], d->xpos[3 * base_id + 2]};

    mj_applyFT(m, d, force_parent_global, torque_parent_global, point_parent_global, base_id, d->qfrc_applied);

    // Joint velocity command
    double desired_qvel = (kSpinDir[i] * refMotorRotVel) / kRotorVelocitySlowdownSim;
    double current_qvel = d->qvel[dof_idx];
    double rotor_inertia = 2.649858234714004e-05;

    double required_torque = rotor_inertia * (desired_qvel - current_qvel) / dt;
    d->qfrc_applied[dof_idx] += required_torque;
  }
}

// UI callbacks (from MuJoCo sample code)
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
  (void)window; (void)scancode; (void)mods;
}
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  glfwGetCursorPos(window, &lastx, &lasty);
  (void)mods; (void)button; (void)act;
}
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  if (!button_left && !button_middle && !button_right) return;
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos; lasty = ypos;
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
  (void)width;
}
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
  (void)window; (void)xoffset;
}

class MujocoNode : public rclcpp::Node {
public:
  MujocoNode() : Node("mujoco_node") {
    // Model param
    this->declare_parameter<std::string>("mujoco_model", "mujoco/x500.xml");
    mujoco_model = this->get_parameter("mujoco_model").as_string();

    char error[1000] = "Could not load XML model";
    sim_model = mj_loadXML(mujoco_model.c_str(), nullptr, error, 1000);
    if (!sim_model) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load Mujoco model: %s", error);
      throw std::runtime_error("MuJoCo load failed");
    } else {
      RCLCPP_INFO(this->get_logger(), "Loaded Mujoco model: %s", mujoco_model.c_str());

    }

    sim_data = mj_makeData(sim_model);

    motor_subscriber   = this->create_subscription<actuator_msgs::msg::Actuators>("/x500_1/command/motor_speed", 10, std::bind(&MujocoNode::motor_callback, this, std::placeholders::_1));
    sim_pose_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/control_1/odometry", 10);
    tf_broadcaster     = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }
  ~MujocoNode() {
    if (sim_data)  mj_deleteData(sim_data);
    if (sim_model) mj_deleteModel(sim_model);
  }
  mjModel* get_model() { return sim_model; }
  mjData* get_data()   { return sim_data;  }

  void publish_sim_state() {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.pose.pose.position.x    = sim_data->qpos[0];
    odom_msg.pose.pose.position.y    = sim_data->qpos[1];
    odom_msg.pose.pose.position.z    = sim_data->qpos[2];
    odom_msg.pose.pose.orientation.w = sim_data->qpos[3];
    odom_msg.pose.pose.orientation.x = sim_data->qpos[4];
    odom_msg.pose.pose.orientation.y = sim_data->qpos[5];
    odom_msg.pose.pose.orientation.z = sim_data->qpos[6];

    odom_msg.twist.twist.linear.x  = sim_data->qvel[0];
    odom_msg.twist.twist.linear.y  = sim_data->qvel[1];
    odom_msg.twist.twist.linear.z  = sim_data->qvel[2];
    odom_msg.twist.twist.angular.x = sim_data->qvel[3];
    odom_msg.twist.twist.angular.y = sim_data->qvel[4];
    odom_msg.twist.twist.angular.z = sim_data->qvel[5];

    sim_pose_publisher->publish(odom_msg);
  }

private:

  void motor_callback(const actuator_msgs::msg::Actuators::SharedPtr msg) {
    int num_motors = 0;

    if (!msg->velocity.empty()) {
      num_motors = std::min(4, (int)msg->velocity.size());
      for (int i = 0; i < num_motors; i++) {
        target_omega[i] = msg->velocity[i];
      }
    } else if (!msg->normalized.empty()) {
      num_motors = std::min(4, (int)msg->normalized.size());
      for (int i = 0; i < num_motors; i++) {
        target_omega[i] = msg->normalized[i] * kMaxRotVelocity;
      }
    }
  }

  std::string mujoco_model;
  mjModel* sim_model = nullptr;
  mjData* sim_data  = nullptr;
  rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr motor_subscriber;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          sim_pose_publisher;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster;

};

// ==========================================================
// Main Loop
// ==========================================================
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MujocoNode>();
  m = node->get_model();
  d = node->get_data();

  // Attach the aerodynamic plugin logic globally to MuJoCo
  mjcb_control = multicopter_motor_callback;

  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }
  
  glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
  GLFWwindow* window = glfwCreateWindow(1200, 900, "ROS2 MuJoCo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // Capture time to enforce fps
  auto frame_start_time = std::chrono::steady_clock::now();

  while (!glfwWindowShouldClose(window) && rclcpp::ok()) {

    // Spin mujoco and ros
    rclcpp::spin_some(node);

    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0/60.0) {
      mj_step(m, d);
    }

    node->publish_sim_state();

    // Update UI
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();

    // Sleep until to regulate fps
    frame_start_time += std::chrono::milliseconds(16);
    std::this_thread::sleep_until(frame_start_time);
  }

  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
