#include <cstdio>
#include <stack>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_hanoi");
namespace mtc = moveit::task_constructor;

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
  // void doTask(
  //   const std::string& disk_name,
  //   const std::string& source_peg,
  //   const std::string& target_peg
  // );
  bool doTask(int disk, int target_peg);
  void setupPlanningScene(int num_disks);
  //bool isValidMove(int disk, int target_peg);
  taskInfo isValidMove(int disk, int target_peg);

private:
  mtc::Task createTask(int disk, int source_peg, int target_peg);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  std::stack<int> peg1_stack;
  std::stack<int> peg2_stack;
  std::stack<int> peg3_stack;

  int disk_thickness;
  int peg_height;

  // Peg positions
  geometry_msgs::msg::Pose peg1_pose;
  geometry_msgs::msg::Pose peg2_pose;
  geometry_msgs::msg::Pose peg3_pose;  

};

HanoiTaskNode::HanoiTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("hanoi_node", options) }
{

  // Disk thickness
  disk_thickness = 0.02;
  peg_height = 0.15;

  // Initialize peg positions
  peg1_pose.position.x = 0.5;
  peg1_pose.position.y = -0.2;
  peg1_pose.position.z = 0.075;
  peg1_pose.orientation.w = 1.0;

  peg2_pose.position.x = 0.5;
  peg2_pose.position.y = 0;
  peg2_pose.position.z = 0.075;
  peg2_pose.orientation.w = 1.0;

  peg3_pose.position.x = 0.5;
  peg3_pose.position.y = 0.2;
  peg3_pose.position.z = 0.075;
  peg3_pose.orientation.w = 1.0;
  
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

void HanoiTaskNode::setupPlanningScene(int num_disks)
{
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

  moveit::planning_interface::PlanningSceneInterface psi;

  // Disk 1 and 2
  // Add support for more disks later

  geometry_msgs::msg::Pose disk1_pose;
  disk1_pose = peg1_pose;
  // disk1_pose.position.x = 0.5;
  // disk1_pose.position.y = -0.2;
  disk1_pose.position.z = 0.01;
  // disk1_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose disk2_pose;
  disk2_pose = peg1_pose;
  // disk2_pose.position.x = 0.5;
  // disk2_pose.position.y = -0.2;
  disk2_pose.position.z = 0.03;
  // disk2_pose.orientation.w = 1.0;

  auto disk1 = makeMeshObject(
    "disk1",
    "package://moveit_hanoi/meshes/disk1.stl",
    disk1_pose);
  
  auto disk2 = makeMeshObject(
    "disk2",
    "package://moveit_hanoi/meshes/disk2.stl",
    disk2_pose);
  
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(peg1);
  objects.push_back(peg2);
  objects.push_back(peg3);
  objects.push_back(disk1);
  objects.push_back(disk2);

  psi.applyCollisionObjects(objects);

  // Update disk positions in peg data structures
  peg1_stack.push(1);
  peg1_stack.push(2);

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
    if (i == target_peg - 1) {
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

bool HanoiTaskNode::doTask(int disk, int target_peg)
{
  taskInfo currentTaskInfo = isValidMove(disk, target_peg);
  if (!currentTaskInfo.isTaskValid) {
    return false;
  }

  int source_peg = currentTaskInfo.sourcePeg;
  task_ = createTask(disk, source_peg, target_peg);

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

  return true;
}

mtc::Task HanoiTaskNode::createTask(int disk, int source_peg, int target_peg)
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

  mtc::Stage* allow_collision_stage = nullptr;

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto stage_open_hand = 
    std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  {
    auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow disk1 hand collision debug");

    auto hand_links =
      task.getRobotModel()
        ->getJointModelGroup(hand_group_name)
        ->getLinkModelNamesWithCollisionGeometry();

    stage->allowCollisions("disk1", hand_links, true);
    stage->allowCollisions(diskName, hand_links, true);
    stage->allowCollisions("peg1", hand_links, true);
    allow_collision_stage = stage.get();
    task.add(std::move(stage));
  }

  auto stage_move_to_pick = 
    std::make_unique<mtc::stages::Connect>("move to pick",
    mtc::stages::Connect::GroupPlannerVector { { arm_group_name, sampling_planner }});
  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // Forward attach object_stage to place pose generator
  mtc::Stage* attach_object_stage = nullptr;

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
      // stage->setMonitoredStage(current_state_ptr);
      stage->setMonitoredStage(allow_collision_stage);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaternion q = Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      // grasp_frame_transform.translation().z() = 0.06 + 0.035*1.5;
      grasp_frame_transform.translation().z() = 0.08 + 0.025;
      grasp_frame_transform.translation().x() = -0.01;

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
      stage->allowCollisions(diskName,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(), true);
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
      stage->allowCollisions("disk1", "disk2", true);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    {
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.2, 0.3);  
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }
  // Place
  {


  }

  return task;

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto hanoi_task_node = std::make_shared<HanoiTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &hanoi_task_node](){
    executor.add_node(hanoi_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(hanoi_task_node->getNodeBaseInterface());
  });

  hanoi_task_node->setupPlanningScene(2);
  
  rclcpp::sleep_for(std::chrono::seconds(1));
  // Trying out a task
  hanoi_task_node->doTask(2, 3);

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
