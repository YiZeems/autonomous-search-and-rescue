// Mission-level Behavior Tree runner — BehaviorTree.CPP v3 (assignment-compliant).
//
// Loads bt_xml/mission.xml and ticks it at `tick_rate_hz` until the tree
// returns SUCCESS. Custom leaf nodes are wired to the project topics:
//
//   WaitForMap          (Condition)  SUCCESS once a /map message has arrived
//   CoverageReached     (Condition)  SUCCESS when /coverage >= {threshold}
//   VictimsFound        (Condition)  SUCCESS when /victims_map has >= {min_count} poses
//   MissionLog          (Action)     logs {message} once, returns SUCCESS
//   PublishMissionDone  (Action)     latches std_msgs/Bool true on /mission_done
//
// A ZMQ publisher (port 1666) exposes the live tree to Groot ("Monitor" mode).
// The XML is also directly loadable in Groot/Groot2 for editing.

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

// Shared mission state, updated by ROS subscriptions, read by BT leaves.
struct MissionContext
{
  std::atomic<bool> map_received{false};
  std::atomic<float> coverage{0.0f};
  std::atomic<int> victim_count{0};
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub;
};

class WaitForMap : public BT::ConditionNode
{
public:
  WaitForMap(const std::string & name, const BT::NodeConfiguration & cfg,
             std::shared_ptr<MissionContext> ctx)
  : BT::ConditionNode(name, cfg), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return ctx_->map_received ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<MissionContext> ctx_;
};

class CoverageReached : public BT::ConditionNode
{
public:
  CoverageReached(const std::string & name, const BT::NodeConfiguration & cfg,
                  std::shared_ptr<MissionContext> ctx)
  : BT::ConditionNode(name, cfg), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("threshold", 0.90, "coverage ratio in [0,1]")};
  }

  BT::NodeStatus tick() override
  {
    double threshold = 0.90;
    getInput("threshold", threshold);
    return ctx_->coverage >= threshold ? BT::NodeStatus::SUCCESS
                                       : BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<MissionContext> ctx_;
};

class VictimsFound : public BT::ConditionNode
{
public:
  VictimsFound(const std::string & name, const BT::NodeConfiguration & cfg,
               std::shared_ptr<MissionContext> ctx)
  : BT::ConditionNode(name, cfg), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("min_count", 0, "minimum victims registered")};
  }

  BT::NodeStatus tick() override
  {
    int min_count = 0;
    getInput("min_count", min_count);
    return ctx_->victim_count >= min_count ? BT::NodeStatus::SUCCESS
                                           : BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<MissionContext> ctx_;
};

class MissionLog : public BT::SyncActionNode
{
public:
  MissionLog(const std::string & name, const BT::NodeConfiguration & cfg,
             std::shared_ptr<MissionContext> ctx)
  : BT::SyncActionNode(name, cfg), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override
  {
    std::string message{"(no message)"};
    getInput("message", message);
    RCLCPP_INFO(ctx_->node->get_logger(), "[BT] %s", message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<MissionContext> ctx_;
};

class PublishMissionDone : public BT::SyncActionNode
{
public:
  PublishMissionDone(const std::string & name, const BT::NodeConfiguration & cfg,
                     std::shared_ptr<MissionContext> ctx)
  : BT::SyncActionNode(name, cfg), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    ctx_->done_pub->publish(msg);
    RCLCPP_INFO(ctx_->node->get_logger(), "[BT] mission done published on /mission_done");
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<MissionContext> ctx_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_runner");

  node->declare_parameter<std::string>("bt_xml", "");
  node->declare_parameter<double>("tick_rate_hz", 1.0);
  node->declare_parameter<bool>("groot_zmq", true);

  const std::string bt_xml = node->get_parameter("bt_xml").as_string();
  const double tick_rate = node->get_parameter("tick_rate_hz").as_double();
  if (bt_xml.empty()) {
    RCLCPP_FATAL(node->get_logger(), "parameter 'bt_xml' is required (path to mission XML)");
    return 1;
  }

  auto ctx = std::make_shared<MissionContext>();
  ctx->node = node;
  // Latched so late subscribers (result exporter, demo scripts) still get it.
  ctx->done_pub = node->create_publisher<std_msgs::msg::Bool>(
    "/mission_done", rclcpp::QoS(1).transient_local());

  auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local(),
    [ctx](nav_msgs::msg::OccupancyGrid::ConstSharedPtr) {
      if (!ctx->map_received) {
        RCLCPP_INFO(ctx->node->get_logger(), "[BT] /map received — SLAM alive");
      }
      ctx->map_received = true;
    });
  auto cov_sub = node->create_subscription<std_msgs::msg::Float32>(
    "/coverage", 10,
    [ctx](std_msgs::msg::Float32::ConstSharedPtr m) { ctx->coverage = m->data; });
  auto victims_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "/victims_map", 10,
    [ctx](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
      ctx->victim_count = static_cast<int>(m->poses.size());
    });

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<WaitForMap>(
    "WaitForMap",
    [ctx](const std::string & n, const BT::NodeConfiguration & c) {
      return std::make_unique<WaitForMap>(n, c, ctx);
    });
  factory.registerBuilder<CoverageReached>(
    "CoverageReached",
    [ctx](const std::string & n, const BT::NodeConfiguration & c) {
      return std::make_unique<CoverageReached>(n, c, ctx);
    });
  factory.registerBuilder<VictimsFound>(
    "VictimsFound",
    [ctx](const std::string & n, const BT::NodeConfiguration & c) {
      return std::make_unique<VictimsFound>(n, c, ctx);
    });
  factory.registerBuilder<MissionLog>(
    "MissionLog",
    [ctx](const std::string & n, const BT::NodeConfiguration & c) {
      return std::make_unique<MissionLog>(n, c, ctx);
    });
  factory.registerBuilder<PublishMissionDone>(
    "PublishMissionDone",
    [ctx](const std::string & n, const BT::NodeConfiguration & c) {
      return std::make_unique<PublishMissionDone>(n, c, ctx);
    });

  auto tree = factory.createTreeFromFile(bt_xml);

  std::unique_ptr<BT::PublisherZMQ> groot;
  if (node->get_parameter("groot_zmq").as_bool()) {
    try {
      groot = std::make_unique<BT::PublisherZMQ>(tree);  // Groot Monitor, port 1666
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Groot ZMQ publisher unavailable: %s", e.what());
    }
  }

  RCLCPP_INFO(node->get_logger(),
              "BT runner started — xml=%s tick=%.1f Hz", bt_xml.c_str(), tick_rate);

  // Persistent executor: repeatedly calling rclcpp::spin_some(node) builds a
  // throwaway executor every tick, which churns the waitset and can miss the
  // latched (TRANSIENT_LOCAL) /map sample. A single long-lived executor keeps
  // the subscription state and delivers callbacks reliably.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  rclcpp::Rate rate(tick_rate);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int ticks = 0;
  // Heartbeat every ~10 s so a long mapping phase doesn't look hung.
  const int heartbeat = std::max(1, static_cast<int>(tick_rate * 10.0));
  while (rclcpp::ok() && status != BT::NodeStatus::SUCCESS) {
    exec.spin_some();
    status = tree.tickRoot();
    if (++ticks % heartbeat == 0) {
      RCLCPP_INFO(node->get_logger(),
                  "[BT] mission running — map=%s coverage=%.1f%% victims=%d",
                  ctx->map_received ? "yes" : "no",
                  ctx->coverage * 100.0f, ctx->victim_count.load());
    }
    rate.sleep();
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "[BT] mission tree returned SUCCESS — idling (Ctrl-C to quit)");
    // Keep the node alive so /mission_done stays latched and Groot stays connected.
    exec.spin();
  }

  rclcpp::shutdown();
  return 0;
}
