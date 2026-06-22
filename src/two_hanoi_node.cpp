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
  bool isValidMove(int disk, int target_peg);

private:
  mtc::Task createTask(int disk, int target_peg);
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
  psi.applyCollisionObject(peg1);
  psi.applyCollisionObject(peg2);
  psi.applyCollisionObject(peg3);

  // Disk 1 and 2
  // Add support for more disks later

  geometry_msgs::msg::Pose disk1_pose;
  disk1_pose.position.x = 0.5;
  disk1_pose.position.y = -0.2;
  disk1_pose.position.z = 0.01;
  disk1_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose disk2_pose;
  disk2_pose.position.x = 0.5;
  disk2_pose.position.y = -0.2;
  disk2_pose.position.z = 0.03;
  disk2_pose.orientation.w = 1.0;

  auto disk1 = makeMeshObject(
    "disk1",
    "package://moveit_hanoi/meshes/disk1.stl",
    disk1_pose);
  
  auto disk2 = makeMeshObject(
    "disk2",
    "package://moveit_hanoi/meshes/disk2.stl",
    disk2_pose);
  
  psi.applyCollisionObjects({ disk1, disk2 });

  // Update disk positions in peg data structures
  peg1_stack.push(1);
  peg1_stack.push(2);

}

bool HanoiTaskNode::isValidMove(int disk, int target_peg)
{
  if (target_peg > 3 || target_peg < 1)
  {
    RCLCPP_ERROR(LOGGER, "Invalid peg. Peg %d does not exist", target_peg);
    return false;
  }

  std::vector<int> top_disks { -1, -1, -1 };

  if (!peg1_stack.empty()) top_disks[0] = peg1_stack.top();
  if (!peg2_stack.empty()) top_disks[1] = peg2_stack.top();
  if (!peg3_stack.empty()) top_disks[2] = peg3_stack.top();

  for (int i{}; i < top_disks.size(); ++i) {
    
    // Check if disk is already on the target peg
    if (i == target_peg - 1) {
      if (top_disks[i] == disk) {
        RCLCPP_ERROR(LOGGER, "INVALID ACTION: Disk %d is already on peg %d", disk, target_peg);
        return false;
      }
    }
    // Check if disk is graspable
    else {
      if (top_disks[i] == disk) {
        return true;
      }
    }
  }

  //Disk wasnt found
  RCLCPP_ERROR(LOGGER, "Couldn't find disk %d", disk);
  return false;
}

bool HanoiTaskNode::doTask(int disk, int target_peg)
{

  if (!isValidMove(disk, target_peg)) {
    return false;
  }

  task_ = createTask(disk, target_peg);

  try
  {
    task_.init();
  }
  catch(mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  if (!task_.plan(5))
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

mtc::Task HanoiTaskNode::createTask(int disk, int target_peg)
{
  
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

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
