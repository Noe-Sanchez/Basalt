#include <chrono>
#include <iostream>
#include <functional>
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

// Mouse vars
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// We use pointers sequentially to access the correct slice by name, according to drone_id 
class Multirotor {
  public:
    Multirotor(const mjModel* model, mjData* data, int _rotor_count, int _drone_id, std::string _drone_name) {
      std::cout << "Initializing Multirotor: " << _drone_name << std::endl;
      std::cout.flush();

      m = model;
      d = data;
      rotor_count = _rotor_count;
      drone_id    = _drone_id;
      drone_name  = _drone_name;

      std::cout << "Looking for " << drone_name + std::string("kMotorConstant") << " in model..." << std::endl;
      std::cout.flush();

      // Load model param
      int param_id   = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kMotorConstant")).c_str());
      int address    = m->numeric_adr[param_id];
      kMotorConstant = m->numeric_data[address];

      std::cout << "Loaded first parameter for " << drone_name << ":" << std::endl;
      std::cout.flush();

      param_id        = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kMomentConstant")).c_str()); 
      address         = m->numeric_adr[param_id];
      kMomentConstant = m->numeric_data[address];

      param_id        = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kTimeConstantUp")).c_str()); 
      address         = m->numeric_adr[param_id];
      kTimeConstantUp = m->numeric_data[address];

      param_id          = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kTimeConstantDown")).c_str()); 
      address           = m->numeric_adr[param_id];
      kTimeConstantDown = m->numeric_data[address];

      param_id        = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kMaxRotVelocity")).c_str()); 
      address         = m->numeric_adr[param_id];
      kMaxRotVelocity = m->numeric_data[address];

      param_id              = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kRotorDragCoefficient")).c_str()); 
      address               = m->numeric_adr[param_id];
      kRotorDragCoefficient = m->numeric_data[address];

      param_id                  = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kRollingMomentCoefficient")).c_str());
      address                   = m->numeric_adr[param_id];
      kRollingMomentCoefficient = m->numeric_data[address];

      param_id                  = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kRotorVelocitySlowdownSim")).c_str()); 
      address                   = m->numeric_adr[param_id];
      kRotorVelocitySlowdownSim = m->numeric_data[address];

      std::cout << "Loaded parameters for " << drone_name << ", " << kTimeConstantUp << std::endl; 
      std::cout.flush();

      target_omega.resize(rotor_count,  0.0);
      current_omega.resize(rotor_count, 0.0);
      kSpinDir.resize(rotor_count,      0.0); 
      kReactDir.resize(rotor_count,     0.0); 

      param_id = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kSpinDir")).c_str());
      address  = m->numeric_adr[param_id];
      for (int i = 0; i < rotor_count; i++) {
        kSpinDir[i] = m->numeric_data[address + i];
      }

      param_id = mj_name2id(m, mjOBJ_NUMERIC, (drone_name + std::string("kReactDir")).c_str()); 
      address  = m->numeric_adr[param_id];
      for (int i = 0; i < rotor_count; i++) {
	kReactDir[i] = m->numeric_data[address + i];
      }

      // Search for the first joint and first body of this drone
      std::string base_link_name = drone_name + std::string("base_link");
      std::string rotor_name     = drone_name + std::string("rotor_0_joint");
      joint_idx = -1;
      body_idx  = -1;
      for (int i = 0; i < m->nbody; i++) {
        std::string body_name = m->names + m->name_bodyadr[i];
	if (body_name == base_link_name) {
	  body_idx = i;
	  break;
	}
      }
      for (int i = 0; i < m->njnt; i++) {
        std::string joint_name = m->names + m->name_jntadr[i];
	if (joint_name == rotor_name) {
	  joint_idx = i;
	  break;
	}
      }
      //std::cout << "Drone: " << drone_name << ", Body index: " << body_idx << ", Joint index: " << joint_idx << std::endl;
      std::cout.flush();

    }

    // Adapted to handle only the variables of a single drone at a time
    void multicopter_motor_physics(){
      
      // Clear rotor_count forces starting from the joint_idx position
      //mju_zero(d->qfrc_applied + joint_idx, rotor_count);

      // Check for base_link
      double dt = m->opt.timestep;
      //int base_id = mj_name2id(m, mjOBJ_BODY, "base_link");
      //if (base_id < 0) return;

      Eigen::Vector3d wind_vel_world(0.0, 0.0, 0.0);

      for (int i = 0; i < rotor_count; i++) { 
        // Check for rotor
        //std::string rotor_name = "rotor_" + std::to_string(i);
        //std::string joint_name = rotor_name + "_joint";
        //int rotor_id = mj_name2id(m, mjOBJ_BODY,  rotor_name.c_str());
        //int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
	int rotor_id = body_idx + i + 1;
	int joint_id = joint_idx + i;

        //std::cout << "Drone: " << drone_name << ", Rotor: " << i << ", Rotor ID: " << rotor_id << ", Joint ID: " << joint_id << std::endl;

        if (rotor_id < 0 || joint_id < 0) continue;

        int dof_idx = m->jnt_dofadr[joint_id];

	//std::cout << "Drone: " << drone_name << ", Rotor: " << i << ", Rotor ID: " << rotor_id << ", Joint ID: " << joint_id << ", DOF index: " << dof_idx << std::endl;

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
        Eigen::Vector3d v_perp            = relative_wind_vel - (relative_wind_vel.dot(axis_world) * axis_world);

        Eigen::Vector3d air_drag       = -std::abs(realMotorVelocity) * kRotorDragCoefficient * v_perp;
        Eigen::Vector3d rolling_moment = -std::abs(realMotorVelocity) * kRollingMomentCoefficient * v_perp;

        // Compute on-rotor force and torque
        Eigen::Vector3d rotor_force   = thrust_force_world + air_drag;
        mjtNum force_rotor_global[3]  = {rotor_force.x(), rotor_force.y(), rotor_force.z()};
        mjtNum torque_rotor_global[3] = {0, 0, 0};
        mjtNum point_rotor_global[3]  = {d->xpos[3 * rotor_id], d->xpos[3 * rotor_id + 1], d->xpos[3 * rotor_id + 2]};

        mj_applyFT(m, d, force_rotor_global, torque_rotor_global, point_rotor_global, rotor_id, d->qfrc_applied);
	
        Eigen::Vector3d drag_torque_world = -kSpinDir[i] * thrust * kMomentConstant * axis_world;
        Eigen::Vector3d parent_torque     = drag_torque_world + rolling_moment;

        mjtNum force_parent_global[3]  = {0, 0, 0};
        mjtNum torque_parent_global[3] = {parent_torque.x(), parent_torque.y(), parent_torque.z()};
        //mjtNum point_parent_global[3]  = {d->xpos[3 * base_id], d->xpos[3 * base_id + 1], d->xpos[3 * base_id + 2]};
        mjtNum point_parent_global[3]  = {d->xpos[3 * body_idx], d->xpos[3 * body_idx + 1], d->xpos[3 * body_idx + 2]};

        //mj_applyFT(m, d, force_parent_global, torque_parent_global, point_parent_global, base_id, d->qfrc_applied);
        mj_applyFT(m, d, force_parent_global, torque_parent_global, point_parent_global, body_idx, d->qfrc_applied);

        // Joint velocity command
        double desired_qvel  = (kSpinDir[i] * refMotorRotVel) / kRotorVelocitySlowdownSim;
        double current_qvel  = d->qvel[dof_idx];
        double rotor_inertia = 2.649858234714004e-05;

        double required_torque    = rotor_inertia * (desired_qvel - current_qvel) / dt;
        d->qfrc_applied[dof_idx] += required_torque;
      }
    } 

    // Cached indices for faster dynamics
    int joint_idx;
    int body_idx;
    std::string    drone_name;
    int            rotor_count;
    std::vector<double> target_omega;
    std::vector<double> current_omega;
    double kMaxRotVelocity;

  private:
    // For sim
    const mjModel* m;
    mjData*        d;
    int            drone_id;

    // From model
    double kMotorConstant;
    double kMomentConstant;
    double kTimeConstantUp;
    double kTimeConstantDown;
    double kRotorDragCoefficient;
    double kRollingMomentCoefficient;
    double kRotorVelocitySlowdownSim;

    std::vector<double> kSpinDir;
    std::vector<double> kReactDir;
};

// Pointer to drones array, to be used in the callback
std::vector<Multirotor>* drones_ptr = nullptr;

/*void multicopter_motor_callback(const mjModel* m, mjData* d) {
  // Clear forces to avoid accum
  mju_zero(d->qfrc_applied, m->nv);

  // Check for base_link
  double dt   = m->opt.timestep;
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

    mjtNum force_parent_global[3]  = {0, 0, 0};
    mjtNum torque_parent_global[3] = {parent_torque.x(), parent_torque.y(), parent_torque.z()};
    mjtNum point_parent_global[3]  = {d->xpos[3 * body_idx], d->xpos[3 * body_idx + 1], d->xpos[3 * body_idx + 2]};

    //mj_applyFT(m, d, force_parent_global, torque_parent_global, point_parent_global, base_id, d->qfrc_applied);
    mj_applyFT(m, d, force_parent_global, torque_parent_global, point_parent_global, body_idx, d->qfrc_applied);

    // Joint velocity command
    double desired_qvel = (kSpinDir[i] * refMotorRotVel) / kRotorVelocitySlowdownSim;
    double current_qvel = d->qvel[dof_idx];
    double rotor_inertia = 2.649858234714004e-05;

    double required_torque = rotor_inertia * (desired_qvel - current_qvel) / dt;
    d->qfrc_applied[dof_idx] += required_torque;
  }
}*/

// UI callbacks (from MuJoCo sample code)
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
  (void)window; (void)scancode; (void)mods;
}
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
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

// Vector pointer to the drones array
void simulation_step_callback(const mjModel* m, mjData* d){
  // Clear forces to avoid accum
  mju_zero(d->qfrc_applied, m->nv);
  
  for (long unsigned int i = 0; i < drones_ptr->size(); i++) {
    (*drones_ptr)[i].multicopter_motor_physics();
  }

  // Consume pointers to avoid errors
  (void)m;
  (void)d;

}

class MujocoNode : public rclcpp::Node {
public:
  MujocoNode() : Node("mujoco_node") {
    // Model param
    //this->declare_parameter<std::string>("mujoco_model", "mujoco/x500.xml");
    //mujoco_model = this->get_parameter("mujoco_model").as_string();

    /*char error[1000] = "Could not load XML model";
    sim_model = mj_loadXML(mujoco_model.c_str(), nullptr, error, 1000);
    if (!sim_model) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load Mujoco model: %s", error);
      throw std::runtime_error("MuJoCo load failed");
    } else {
      RCLCPP_INFO(this->get_logger(), "Loaded Mujoco model: %s", mujoco_model.c_str());

    }*/

    // Hardcode model composition for now
    // Replicate this python code in cpp
    // world = mj.MjSpec.from_file("mujoco/world.xml")

    // # Compose the world model with the other xml files
    // drone1 = mj.MjSpec.from_file("mujoco/x500_standalone.xml")
    // drone2 = mj.MjSpec.from_file("mujoco/x500_standalone.xml")
    // 
    // # Compose here
    // world_site = world.worldbody.add_site(pos=[0, 0, 0], name="world_attachment_site")
    // for i in range(2):
    //   world.attach(child=drone1 if i == 0 else drone2, prefix=f"/x500_{i+1}/", site=world_site)
    // 
    // m = world.compile()
    // d = mj.MjData(m)
    this->declare_parameter<int>("num_drones", 2);
    num_drones = this->get_parameter("num_drones").as_int();

    drones.reserve(num_drones);

    std::cout << "Creating dynamic publishers and subscribers for " << num_drones << " drones." << std::endl;
    std::cout.flush();

    for (int i = 0; i < num_drones; i++) {
      std::string drone_name = "/x500_" + std::to_string(i + 1) + std::string("/");
      
      // Dynamically create publishers and subscribers
      std::function<void(const actuator_msgs::msg::Actuators::SharedPtr)> drone_proto = std::bind(&MujocoNode::motor_callback, this, std::placeholders::_1, i);
      drone_actuator_subscriptions.push_back(this->create_subscription<actuator_msgs::msg::Actuators>(drone_name + "command/motor_speed", 10, drone_proto));

      std::cout << "Created subscription for drone: " << drone_name << std::endl;
      std::cout.flush();

      odom_publishers.push_back(this->create_publisher<nav_msgs::msg::Odometry>("/model" + drone_name + "odometry", 10));

      std::cout << "Created publisher for drone: " << drone_name << std::endl;
      std::cout.flush();
    
    }

    std::cout << "Dynamic publishers and subscribers created." << std::endl;
    std::cout.flush();
 
    drones_ptr = &drones;

    // Load spec, not model
    auto world  = mj_parseXML("mujoco/world.xml", nullptr, nullptr, 0);
    auto drone1 = mj_parseXML("mujoco/x500_standalone.xml", nullptr, nullptr, 0);
    auto drone2 = mj_parseXML("mujoco/x500_standalone.xml", nullptr, nullptr, 0);

    std::cout << "MuJoCo XML models parsed." << std::endl;
    std::cout.flush();

    // Compose here (add site)
    auto worldbody  = mjs_findBody(world, "world");
    /*auto world_site = mjs_addSite(worldbody, nullptr);
    world_site->pos[0]  = 0.0;
    world_site->pos[1]  = 0.0;
    world_site->pos[2]  = 0.0;
    world_site->size[0] = 0.01;
    world_site->size[1] = 0.01;
    world_site->size[2] = 0.01;

    std::cout << "World site added to MuJoCo model." << std::endl;
    std::cout.flush();*/

    //auto drone1_worldbody = mjs_findBody(drone1, "world");
    //auto drone2_worldbody = mjs_findBody(drone2, "world");

    //mjs_attach(world_site->element, drone1->element, "/x500_1/", nullptr);
    //mjs_attach(world_site->element, drone2->element, "/x500_2/", nullptr);
    
    //mjs_attach(world_site->element, drone1_worldbody->element, "/x500_1/", "");
    //mjs_attach(world_site->element, drone2_worldbody->element, "/x500_2/", "");
    
    //mjs_attach(worldbody->element, drone1_worldbody->element, "/x500_1/", "");
    //mjs_attach(worldbody->element, drone2_worldbody->element, "/x500_2/", "");
    
    mjs_attach(worldbody->element, drone1->element, "/x500_1/", "");
    mjs_attach(worldbody->element, drone2->element, "/x500_2/", "");

    std::cout << "Drones attached to world site in MuJoCo model." << std::endl;

    sim_model = mj_compile(world, nullptr);
    sim_data  = mj_makeData(sim_model);

    std::cout << "MuJoCo model compiled and data created." << std::endl;

    for (int i = 0; i < num_drones; i++) {
      std::string drone_name = "/x500_" + std::to_string(i + 1) + std::string("/");
      drones.push_back(Multirotor(sim_model, sim_data, 4, i, drone_name));

      /*// Change drone position to avoid collision
      sim_data->xpos[3 * drones[i].body_idx]     = 0.0;
      sim_data->xpos[3 * drones[i].body_idx + 1] = 1.0 * i; // Offset in y-axis
      sim_data->xpos[3 * drones[i].body_idx + 2] = 0.5; // Height*/
      
      // Change drone position to avoid collision by modifying the free joint's state
      std::string free_joint_name = drone_name + "base_free_joint";
      int jnt_id = mj_name2id(sim_model, mjOBJ_JOINT, free_joint_name.c_str());
      
      if (jnt_id >= 0) {
        int qpos_adr = sim_model->jnt_qposadr[jnt_id];
        
        // A free joint has 7 elements in qpos: [x, y, z, qw, qx, qy, qz]
        sim_data->qpos[qpos_adr]     = 0.0;
        sim_data->qpos[qpos_adr + 1] = 1.0 * i; // Offset in y-axis
        sim_data->qpos[qpos_adr + 2] = 0.5;     // Height
      } else {
        std::cerr << "Warning: Could not find free joint for " << drone_name << std::endl;
      }

    }

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    std::cout << "MujocoNode initialized with " << num_drones << " drones." << std::endl;
    // Flush to print now
    std::cout.flush();
  }
  ~MujocoNode() {
    if (sim_data)  mj_deleteData(sim_data);
    if (sim_model) mj_deleteModel(sim_model);
  }
  mjModel* get_model() { return sim_model; }
  mjData*  get_data()  { return sim_data;  }

  void publish_sim_state() {
    nav_msgs::msg::Odometry odom_msg;

    for (long unsigned int i = 0; i < drones.size(); i++) {
      Multirotor& drone = drones[i];
      
      odom_msg.header.stamp    = this->now();
      odom_msg.header.frame_id = "world";
      odom_msg.child_frame_id  = drone.drone_name + "/base_link";
      
      odom_msg.pose.pose.position.x = sim_data->xpos[3 * drone.body_idx]; 
      odom_msg.pose.pose.position.y = sim_data->xpos[3 * drone.body_idx + 1];
      odom_msg.pose.pose.position.z = sim_data->xpos[3 * drone.body_idx + 2];
      
      // Orientation
      mjtNum quat[4];
      mju_mat2Quat(quat, sim_data->xmat + 9 * drone.body_idx);
      odom_msg.pose.pose.orientation.x = quat[1];
      odom_msg.pose.pose.orientation.y = quat[2];
      odom_msg.pose.pose.orientation.z = quat[3];
      odom_msg.pose.pose.orientation.w = quat[0];

      int qvel_adr = sim_model->jnt_dofadr[drone.joint_idx - 1]; // Offset back to the free joint
      odom_msg.twist.twist.linear.x  = sim_data->qvel[qvel_adr];
      odom_msg.twist.twist.linear.y  = sim_data->qvel[qvel_adr + 1];
      odom_msg.twist.twist.linear.z  = sim_data->qvel[qvel_adr + 2];
      odom_msg.twist.twist.angular.x = sim_data->qvel[qvel_adr + 3];
      odom_msg.twist.twist.angular.y = sim_data->qvel[qvel_adr + 4];
      odom_msg.twist.twist.angular.z = sim_data->qvel[qvel_adr + 5];
      
      // Publish the message
      odom_publishers[i]->publish(odom_msg);
      
      // Broadcast TF
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp    = this->now();
      tf_msg.header.frame_id = "world";
      tf_msg.child_frame_id  = drone.drone_name + "/base_link";
      tf_msg.transform.translation.x = sim_data->xpos[3 * drone.body_idx];
      tf_msg.transform.translation.y = sim_data->xpos[3 * drone.body_idx + 1];
      tf_msg.transform.translation.z = sim_data->xpos[3 * drone.body_idx + 2];
      tf_msg.transform.rotation.x    = quat[1];
      tf_msg.transform.rotation.y    = quat[2];
      tf_msg.transform.rotation.z    = quat[3];
      tf_msg.transform.rotation.w    = quat[0];
      
      tf_broadcaster->sendTransform(tf_msg);
    }
  }

private:

  void motor_callback(const actuator_msgs::msg::Actuators::SharedPtr msg, int drone_id) {
    /*int num_motors = 0;

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
    }*/

    for (long unsigned int i = 0; i < (long unsigned int)drones[drone_id].rotor_count; i++) {
      if (i < msg->velocity.size()) {
        drones[drone_id].target_omega[i] = msg->velocity[i];
      } else if (i < msg->normalized.size()) {
        drones[drone_id].target_omega[i] = msg->normalized[i] * drones[drone_id].kMaxRotVelocity;
      } else {
        drones[drone_id].target_omega[i] = 0.0;
      }
    }
  }

  std::string mujoco_model;
  mjModel* sim_model = nullptr;
  mjData* sim_data  = nullptr;
  std::vector<Multirotor> drones;
  int num_drones;

  std::vector<rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr> drone_actuator_subscriptions;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_publishers;

  std::shared_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster;

};

// ==========================================================
// Main Loop
// ==========================================================
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  std::cout << "Starting MuJoCo ROS2 Node..." << std::endl;
  std::cout.flush();

  auto node = std::make_shared<MujocoNode>();
  m = node->get_model();
  d = node->get_data();

  // Attach the aerodynamic plugin logic globally to MuJoCo
  mjcb_control = simulation_step_callback;

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
