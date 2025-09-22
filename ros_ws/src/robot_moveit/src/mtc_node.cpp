#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <string>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene(const double radius, const double height, const std::string& id);
  void removeObject(const std::string &id);

  private:
  moveit::planning_interface::PlanningSceneInterface psi_;
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene(const double height, const double radius, const std::string& id)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { height, radius };

  geometry_msgs::msg::Pose pose;
  pose.position.x = -0.2;
  pose.position.y = 1.6;
  pose.position.z = 0.175;
  pose.orientation.w = 1.0;
  object.pose = pose;

  psi_.applyCollisionObject(object);
  RCLCPP_INFO(LOGGER, "Added an object into the world");
}

void MTCTaskNode::removeObject(const std::string &id) {
    std::vector<std::string> object_ids;
    object_ids.push_back(id);
    psi_.removeCollisionObjects(object_ids);
    RCLCPP_INFO(LOGGER, "Removed an object into the world");
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& gripper_group_name = "gripper";
  const auto& gripper_frame = "end_effector";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", gripper_group_name);
  task.setProperty("ik_frame", gripper_frame);

  mtc::Stage* current_state_ptr = nullptr;

  // ------------------ Current state ------------------
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // ------------------ Solvers ------------------
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // ---- 1. Open gripper ----
  auto stage_open_gripper =
      std::make_unique<mtc::stages::MoveTo>("open_gripper", interpolation_planner);
  stage_open_gripper->setGroup(gripper_group_name);
  stage_open_gripper->setGoal("open");
  task.add(std::move(stage_open_gripper));

  // --- 2. Move to Pick ---
  auto stage_pick = std::make_unique<mtc::stages::MoveTo>(
      "move to pick", interpolation_planner);
  stage_pick->setGroup(arm_group_name);
  stage_pick->setGoal("pick");
  task.add(std::move(stage_pick));

  // ---- 3. Close the gripper ----
  auto stage_close = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage_close->setGroup(gripper_group_name);
  stage_close->setGoal("close");
  task.add(std::move(stage_close));

  // ---- 4. Attach object ----
  auto stage_attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage_attach->attachObject("object", gripper_frame);
  task.add(std::move(stage_attach));

  // --- 5. Move to Home ---
  auto stage_home = std::make_unique<mtc::stages::MoveTo>(
      "move to home", interpolation_planner);
  stage_home->setGroup(arm_group_name);
  stage_home->setGoal("home");
  task.add(std::move(stage_home));

  //---- 6. Move to place ----
  auto stage_place = std::make_unique<mtc::stages::MoveTo>(
      "place", interpolation_planner);
  stage_place->setGroup(arm_group_name);
  stage_place->setGoal("place");
  task.add(std::move(stage_place));

  // ---- 7. Detach object ----
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject("object", gripper_frame);
  task.add(std::move(stage));

  // ---- 7a. Open gripper ----
  auto stage_open_gripper2 =
      std::make_unique<mtc::stages::MoveTo>("open_gripper", interpolation_planner);
  stage_open_gripper2->setGroup(gripper_group_name);
  stage_open_gripper2->setGoal("open");
  task.add(std::move(stage_open_gripper2));

  // ---- 8. Go to home ----
  auto stage_home2 = std::make_unique<mtc::stages::MoveTo>(
      "move to home2", interpolation_planner);
  stage_home2->setGroup(arm_group_name);
  stage_home2->setGoal("home");
  task.add(std::move(stage_home2));

  // ---- 9. Close gripper ----
  auto stage_h = std::make_unique<mtc::stages::MoveTo>("close gripper home", interpolation_planner);
  stage_h->setGroup(gripper_group_name);
  stage_h->setGoal("home");
  task.add(std::move(stage_h));


  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene(0.35, 0.08, "object");
  mtc_task_node->doTask();
//   mtc_task_node->removeObject("object");

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}