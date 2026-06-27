#include <cstdio>
#include <stack>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>

#include "moveit_hanoi/srv/execute_move.hpp"

#include <filesystem>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_hanoi");
namespace mtc = moveit::task_constructor;
namespace fs = std::filesystem;

struct taskInfo
{
  bool isTaskValid;
  int sourcePeg;
};

class HanoiTaskNode
{
public:
  HanoiTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  
  bool doTask(int disk, int target_peg, bool go_home_after);
  void setupPlanningScene();
  taskInfo isValidMove(int disk, int target_peg);
  double getDiskRadius(int disk) const;

  std::string getTopContactElement(int peg);
  std::string getSecondFromTopContactElement(int peg);

private:
  mtc::Task createTask(int disk, int source_peg, int target_peg, bool go_home_after);
  rclcpp::Service<moveit_hanoi::srv::ExecuteMove>::SharedPtr execute_move_service_;
  
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  std::stack<int> peg1_stack;
  std::stack<int> peg2_stack;
  std::stack<int> peg3_stack;

  double disk_thickness;
  double peg_height;
  double platform_height_;

  // Peg positions
  geometry_msgs::msg::Pose peg1_pose;
  geometry_msgs::msg::Pose peg2_pose;
  geometry_msgs::msg::Pose peg3_pose;  

  int num_disks_;
  int disk_max_rad_;
  int disk_min_rad_;

};

HanoiTaskNode::HanoiTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("hanoi_mtc_executor", options) }
{

  // Disk thickness
  disk_thickness = 0.025;
  peg_height = 0.15;
  platform_height_ = 0.025;

  disk_max_rad_ = 0.035;
  disk_min_rad_ = 0.02;

  // Initialize peg positions
  peg1_pose.position.x = 0.55;
  peg1_pose.position.y = -0.25;
  peg1_pose.position.z = 0.075 + platform_height_;
  peg1_pose.orientation.w = 1.0;

  peg2_pose.position.x = 0.55;
  peg2_pose.position.y = 0;
  peg2_pose.position.z = 0.075 + platform_height_;
  peg2_pose.orientation.w = 1.0;

  peg3_pose.position.x = 0.55;
  peg3_pose.position.y = 0.25;
  peg3_pose.position.z = 0.075 + platform_height_;
  peg3_pose.orientation.w = 1.0;
  
  // Create service
  execute_move_service_ = 
    node_->create_service<moveit_hanoi::srv::ExecuteMove>(
      "execute_move",
      [this](
        const std::shared_ptr<moveit_hanoi::srv::ExecuteMove::Request> request,
        std::shared_ptr<moveit_hanoi::srv::ExecuteMove::Response> response)
      {
        RCLCPP_INFO(LOGGER, 
          "Recieved move request: disk=%d, target_peg=%d",
          request->disk,
          request->target_peg);

        bool ok = this->doTask(request->disk, request->target_peg, request->go_home_after);

        response->success = ok;
        response->message = ok ? "Move executed successfully" : "Move failed";
      });

}

double HanoiTaskNode::getDiskRadius(int disk) const
{
    if (num_disks_ == 1)
        return disk_max_rad_;

    double step =
        (disk_max_rad_ - disk_min_rad_) /
        static_cast<double>(num_disks_ - 1);

    return disk_max_rad_ - (disk - 1) * step;
}

std::string HanoiTaskNode::getTopContactElement(int peg)
{
    switch (peg)
    {
        case 1:
            if (peg1_stack.empty())
                return "platform";
            return "disk" + std::to_string(peg1_stack.top());

        case 2:
            if (peg2_stack.empty())
                return "platform";
            return "disk" + std::to_string(peg2_stack.top());

        case 3:
            if (peg3_stack.empty())
                return "platform";
            return "disk" + std::to_string(peg3_stack.top());

        default:
            throw std::runtime_error("Invalid peg number");
    }
}

std::string HanoiTaskNode::getSecondFromTopContactElement(int peg)
{
  std::stack<int> temp;

  switch (peg) {
    case 1: temp = peg1_stack; break;
    case 2: temp = peg2_stack; break;
    case 3: temp = peg3_stack; break;
    default: throw std::runtime_error("Invalid peg");
  }

  if (temp.empty()) {
    throw std::runtime_error("Source peg is empty");
  }

  temp.pop();  // remove disk being picked

  if (temp.empty()) {
    return "platform";
  }

  return "disk" + std::to_string(temp.top());
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr HanoiTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// Helper for adding mesh objects
moveit_msgs::msg::CollisionObject makeMeshObject(
  const std::string& id,
  const std::string& mesh_uri,
  const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = "world";

  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_uri);
  shape_msgs::msg::Mesh mesh_msg;
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);
  mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  object.meshes.push_back(mesh_msg);
  object.mesh_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  delete mesh;
  return object;
}

void HanoiTaskNode::setupPlanningScene()
{

  moveit::planning_interface::PlanningSceneInterface psi;

  // Platform
  moveit_msgs::msg::CollisionObject platform;
  platform.id = "platform";
  platform.header.frame_id = "world";
  platform.primitives.resize(1);
  platform.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  platform.primitives[0].dimensions = { 0.085, 0.575, platform_height_ };

  geometry_msgs::msg::Pose platform_pose;
  platform_pose = peg2_pose;
  platform_pose.position.z = platform_height_/2;
  platform.pose = platform_pose;
  psi.applyCollisionObject(platform);

  // Peg 1
  moveit_msgs::msg::CollisionObject peg1;
  peg1.id = "peg1";
  peg1.header.frame_id = "world";
  peg1.primitives.resize(1);
  peg1.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  peg1.primitives[0].dimensions = { 0.15, 0.01 };
  peg1.pose = peg1_pose;

  // Peg 2
  moveit_msgs::msg::CollisionObject peg2;
  peg2.id = "peg2";
  peg2.header.frame_id = "world";
  peg2.primitives.resize(1);
  peg2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  peg2.primitives[0].dimensions = { 0.15, 0.01 };
  peg2.pose = peg2_pose;

  // Peg 3
  moveit_msgs::msg::CollisionObject peg3;
  peg3.id = "peg3";
  peg3.header.frame_id = "world";
  peg3.primitives.resize(1);
  peg3.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  peg3.primitives[0].dimensions = { 0.15, 0.01 };
  peg3.pose = peg3_pose;

  RCLCPP_INFO(
    LOGGER,
    "peg3_pose: x=%f y=%f z=%f w=%f",
    peg3_pose.position.x,
    peg3_pose.position.y,
    peg3_pose.position.z,
    peg3_pose.orientation.w
  );

  // Apply peg collision objects to planning scene
  
  std::vector<moveit_msgs::msg::CollisionObject> peg_objects;
  peg_objects.push_back(peg1);
  peg_objects.push_back(peg2);
  peg_objects.push_back(peg3);
  psi.applyCollisionObjects(peg_objects);

  std::vector<moveit_msgs::msg::CollisionObject> disk_objects;
  std::vector<moveit_msgs::msg::ObjectColor> disk_colors;
  
  std::string package_name = "moveit_hanoi";
  fs::path mesh_dir = ament_index_cpp::get_package_share_directory(package_name) + "/meshes";

  std::vector<fs::path> mesh_files;

  for (const auto& entry : fs::directory_iterator(mesh_dir))
  {
    if (entry.path().extension() == ".stl")
    {
      mesh_files.push_back(entry.path());
    }
  }

  std::sort(mesh_files.begin(), mesh_files.end());

  // Derive disk poses: all disks start on peg1
  for (size_t i{}; i < mesh_files.size(); ++i) {
    std::string filename = mesh_files[i].filename().string();

    geometry_msgs::msg::Pose disk_pose = peg1_pose;
    disk_pose.position.z = (static_cast<double>(i) + 0.5) * disk_thickness + platform_height_;
    
    // Update disk positions in peg data structure
    int disk_id = static_cast<int>(i) + 1;
    peg1_stack.push(disk_id);

    RCLCPP_INFO(LOGGER, "Added disk %d", disk_id);

    std::string object_id = mesh_files[i].stem().string();
    std::string mesh_uri = "package://" + package_name + "/meshes/" + filename;

    auto disk = makeMeshObject(object_id, mesh_uri, disk_pose);
    
    moveit_msgs::msg::ObjectColor color;
    color.id = disk.id;  // must match collision object id

    if (disk_id == 1) {
      color.color.r = 0.9;
      color.color.g = 0.1;
      color.color.b = 0.1;
      color.color.a = 1.0;
    } else if (disk_id == 2) {
      color.color.r = 0.1;
      color.color.g = 0.1;
      color.color.b = 0.9;
      color.color.a = 1.0;
    } else if (disk_id == 3 ){
      color.color.r = 0.1;
      color.color.g = 0.9;
      color.color.b = 0.1;
      color.color.a = 1.0;
    } else if (disk_id == 4) {
      color.color.r = 0.9;
      color.color.g = 0.9;
      color.color.b = 0.1;
      color.color.a = 1.0;
    } else {
      color.color.r = 0.5;
      color.color.g = 0.5;
      color.color.b = 0.5;
      color.color.a = 1.0;
    }

    disk_objects.push_back(disk);
    disk_colors.push_back(color);

  }

  num_disks_ = static_cast<int>(mesh_files.size());

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.world.collision_objects = disk_objects;
  scene.object_colors = disk_colors;

  psi.applyPlanningScene(scene);

  // Add disks to planning scene
  psi.applyCollisionObjects(disk_objects);

}

taskInfo HanoiTaskNode::isValidMove(int disk, int target_peg)
{
  taskInfo result;
  result.isTaskValid = false;
  result.sourcePeg = -1;

  if (target_peg > 3 || target_peg < 1)
  {
    RCLCPP_ERROR(LOGGER, "Invalid peg. Peg %d does not exist", target_peg);
    return result;
  }

  std::vector<int> top_disks { -1, -1, -1 };

  if (!peg1_stack.empty()) top_disks[0] = peg1_stack.top();
  if (!peg2_stack.empty()) top_disks[1] = peg2_stack.top();
  if (!peg3_stack.empty()) top_disks[2] = peg3_stack.top();

  for (std::size_t i{}; i < top_disks.size(); ++i) {
    
    // Check if disk is already on the target peg
    if (i == static_cast<std::size_t>(target_peg - 1)) {
      if (top_disks[i] == disk) {
        RCLCPP_ERROR(LOGGER, "INVALID ACTION: Disk %d is already on peg %d", disk, target_peg);
        return result;
      }
    }
    // Check if disk is graspable
    else {
      if (top_disks[i] == disk) {
        result.isTaskValid = true;
        result.sourcePeg = static_cast<int>(i) + 1;
        return result;
      }
    }
  }

  //Disk not found
  RCLCPP_ERROR(LOGGER, "Couldn't find disk %d", disk);
  return result;
}

bool HanoiTaskNode::doTask(int disk, int target_peg, bool go_home_after)
{
  taskInfo currentTaskInfo = isValidMove(disk, target_peg);
  if (!currentTaskInfo.isTaskValid) {
    return false;
  }

  int source_peg = currentTaskInfo.sourcePeg;
  task_ = createTask(disk, source_peg, target_peg, go_home_after);

  try
  {
    task_.init();
  }
  catch(mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  if (!task_.plan(20))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed.");
    return false;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed.");
    return false;
  }

  // Update source peg stack
  switch(source_peg)
  {
    case 1: peg1_stack.pop(); break;
    case 2: peg2_stack.pop(); break;
    case 3: peg3_stack.pop(); break;
  }

  // Update target peg stack
  switch(target_peg)
  {
    case 1: peg1_stack.push(disk); break;
    case 2: peg2_stack.push(disk); break;
    case 3: peg3_stack.push(disk); break;
  }

  return true;
}

mtc::Task HanoiTaskNode::createTask(int disk, int source_peg, int target_peg, bool go_home_after)
{

  mtc::Task task;
  task.stages()->setName("move disk" + std::to_string(disk));
  std::string diskName = "disk" + std::to_string(disk);

  RCLCPP_INFO(LOGGER, "Creating pick task for diskName = %s", diskName.c_str());
  
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  // mtc::Stage* allow_collision_stage = nullptr;

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  // sampling_planner->setProperty("max_velocity_scaling_factor", 0.1);
  // sampling_planner->setProperty("max_acceleration_scaling_factor", 0.1);
  
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.03);

  auto stage_open_hand = 
    std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // {
  //   auto stage =
  //     std::make_unique<mtc::stages::ModifyPlanningScene>("allow disk hand collision debug");

  //   auto hand_links =
  //     task.getRobotModel()
  //       ->getJointModelGroup(hand_group_name)
  //       ->getLinkModelNamesWithCollisionGeometry();

  //   stage->allowCollisions("disk1", hand_links, true);
  //   stage->allowCollisions(diskName, hand_links, true);
  //   stage->allowCollisions("peg1", hand_links, true);
  //   allow_collision_stage = stage.get();
  //   task.add(std::move(stage));
  // }

  auto stage_move_to_pick = 
    std::make_unique<mtc::stages::Connect>("move to pick",
    mtc::stages::Connect::GroupPlannerVector { { arm_group_name, sampling_planner }});
  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // Forward attach object_stage to place pose generator
  mtc::Stage* attach_object_stage = nullptr;
  // mtc::Stage* allow_place_collision_stage = nullptr;

  auto hand_links =
    task.getRobotModel()
      ->getJointModelGroup(hand_group_name)
      ->getLinkModelNamesWithCollisionGeometry();
  std::string source_peg_name = "peg" + std::to_string(source_peg);
  std::string target_peg_name = "peg" + std::to_string(target_peg);
  std::string contact_elem_name = getTopContactElement(target_peg);
  std::string source_contact_elem_name = getSecondFromTopContactElement(source_peg);
  double place_clearance = 0.05;

  // Pick
  {

    auto grasp = std::make_unique<mtc::SerialContainer>("pick_object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
      { "eef", "group", "ik_frame" });
    
    {
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);
    
      // Move the arm forward
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }


    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp-pose");
      stage->setPreGraspPose("open");
      stage->setObject(diskName);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);
      // stage->setMonitoredStage(allow_collision_stage);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaternion q = Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      // grasp_frame_transform.translation().z() = 0.06 + 0.035*1.5;
      // grasp_frame_transform.translation().z() = 0.13 + 0.02;
      grasp_frame_transform.translation().z() = 0.12 + getDiskRadius(disk);
      // grasp_frame_transform.translation().x() = -disk_thickness/2;

      // Compute IK
      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }
    {
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, object)");
      stage->allowCollisions(diskName, hand_links, true);
      stage->allowCollisions(diskName, source_contact_elem_name, true);
      stage->allowCollisions(source_peg_name, hand_links, true);
      stage->allowCollisions(source_peg_name, diskName, true);

      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(diskName, hand_frame);
      // stage->allowCollisions("disk1", "disk2", true);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    {
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      
      int source_peg_length = -1;
      double raise_height{};
      switch(source_peg)
      {
        case 1: source_peg_length = peg1_stack.size(); break;
        case 2: source_peg_length = peg2_stack.size(); break;
        case 3: source_peg_length = peg3_stack.size(); break;
      }
      if (source_peg_length != -1) {
        raise_height = place_clearance + (peg_height - (disk_thickness/2 + std::max(source_peg_length - 1, 0)*disk_thickness));
      }
      else {
        RCLCPP_ERROR(LOGGER, "Cant find target peg length.");
      }
      
      
      
      // stage->setMinMaxDistance(0.3, 0.3);  
      stage->setMinMaxDistance(raise_height, raise_height);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    {
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand, source_peg)");
      
      stage->allowCollisions(diskName, source_contact_elem_name, false);
      stage->allowCollisions(source_peg_name, hand_links, false);
      stage->allowCollisions(source_peg_name, diskName, false);

      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));

  }
  // Place
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move_to_place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                { hand_group_name, interpolation_planner} });
    stage_move_to_place->setTimeout(20.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    {
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow place collisions");
      
      stage->properties().configureInitFrom(mtc::Stage::PARENT);

      stage->allowCollisions(contact_elem_name, diskName, true);
      stage->allowCollisions(contact_elem_name, hand_links, true);
      stage->allowCollisions(target_peg_name, hand_links, true);
      stage->allowCollisions(target_peg_name, diskName, true);
      // allow_place_collision_stage = stage.get();
      // stage->allowCollisions(contact_elem_name, hand_links, true);
      // stage->allowCollisions("peg1", hand_links, true);
      
      place->insert(std::move(stage));
    }
    {
      auto alternatives = std::make_unique<mtc::Alternatives>("place pose alternatives");
      
      place->properties().exposeTo(alternatives->properties(), { "eef", "group", "ik_frame" });
      alternatives->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      for (double yaw: {0.0, M_PI/2, M_PI, -M_PI/2}) {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(diskName);

        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "world";
        
        switch (target_peg)
        {
          case 1: target_pose_msg.pose = peg1_pose; break;
          case 2: target_pose_msg.pose = peg2_pose; break;
          case 3: target_pose_msg.pose = peg3_pose; break;
          default: RCLCPP_ERROR(LOGGER, "Unexpected target peg %d", target_peg);
        }

        target_pose_msg.pose.position.z += (peg_height/2 + place_clearance);
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        target_pose_msg.pose.orientation = tf2::toMsg(q);

        stage->setPose(target_pose_msg);
        stage->setMonitoredStage(attach_object_stage); 
      
        // Compute IK
        auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(diskName);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        
        alternatives->insert(std::move(wrapper));
      }
      place->insert(std::move(alternatives));
    }

    {
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow place collisions");

      stage->allowCollisions(contact_elem_name, diskName, true);
      stage->allowCollisions(target_peg_name, hand_links, true);
      stage->allowCollisions(target_peg_name, diskName, true);
      // stage->allowCollisions(contact_elem_name, hand_links, true);
      // stage->allowCollisions("peg1", hand_links, true);
      
      place->insert(std::move(stage));
    }

    // Get lower_height
    int target_peg_length = -1;
    double lower_height{};
    switch(target_peg)
    {
      case 1: target_peg_length = peg1_stack.size(); break;
      case 2: target_peg_length = peg2_stack.size(); break;
      case 3: target_peg_length = peg3_stack.size(); break;
    }
    if (target_peg_length != -1) {
      lower_height = peg_height + 0.3 - (disk_thickness/2 + target_peg_length*disk_thickness);
      lower_height = place_clearance + peg_height - (disk_thickness/2 + target_peg_length*disk_thickness);
    }
    else {
      RCLCPP_ERROR(LOGGER, "Cant find target peg length.");
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      
      // Get current height of stack on target peg

      
      // lower_height = place_clearance;

      
      stage->setMinMaxDistance(lower_height, lower_height);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lower_object");

      // Set downward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));

    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(diskName, hand_frame);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("raise arm", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      
      // Get current height of stack on target peg

      
      // lower_height = place_clearance;

      
      stage->setMinMaxDistance(lower_height, lower_height);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "raise arm");

      // Set downward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));

    }
    {
      std::string target_peg_name = "peg" + std::to_string(target_peg);
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      auto hand_links = task.getRobotModel()
                        ->getJointModelGroup(hand_group_name)
                        ->getLinkModelNamesWithCollisionGeometry();
      stage->allowCollisions(diskName, hand_links, false);
      stage->allowCollisions(target_peg_name, hand_links, false);
      place->insert(std::move(stage));
    }
    

    task.add(std::move(place));
  } 
  
  if (go_home_after)
  {
    // Go home
    auto stage = std::make_unique<mtc::stages::MoveTo>("go home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
  
  return task;

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto hanoi_task_node = std::make_shared<HanoiTaskNode>(options);
  
  // Setup planning scene
  hanoi_task_node->setupPlanningScene();
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Planning scene ready. Waiting for service requests...");
  
  rclcpp::executors::MultiThreadedExecutor executor;

  // auto spin_thread = std::make_unique<std::thread>([&executor, &hanoi_task_node](){
  //   executor.add_node(hanoi_task_node->getNodeBaseInterface());
  //   executor.spin();
  //   executor.remove_node(hanoi_task_node->getNodeBaseInterface());
  // });

  executor.add_node(hanoi_task_node->getNodeBaseInterface());
  executor.spin();

  executor.remove_node(hanoi_task_node->getNodeBaseInterface());

  rclcpp::shutdown();
  return 0;
}
