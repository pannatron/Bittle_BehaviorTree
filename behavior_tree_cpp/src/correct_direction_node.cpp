#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <chrono>

class CorrectDirection : public BT::CoroActionNode
{
public:
    CorrectDirection(const std::string& name, const BT::NodeConfiguration& config)
        : BT::CoroActionNode(name, config), aruco_angle_(0.0), direction_received_(false)
    {
        auto node_name = name + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        node_ = rclcpp::Node::make_shared(node_name);

        direction_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/aruco_direction", 10,
            std::bind(&CorrectDirection::directionCallback, this, std::placeholders::_1));

        angle_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/aruco_angle", 10,
            std::bind(&CorrectDirection::angleCallback, this, std::placeholders::_1));

        command_pub_ = node_->create_publisher<std_msgs::msg::String>("command", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        while (rclcpp::ok())
        {
            if (!direction_received_)
            {
                RCLCPP_INFO(node_->get_logger(), "Waiting for direction data...");
                rclcpp::spin_some(node_);
                setStatusRunningAndYield();
                return BT::NodeStatus::RUNNING;
            }
            else
            {
                if (!aruco_direction_)
                {
                    auto message = std_msgs::msg::String();
                    if (aruco_angle_ > 0)
                    {
                        message.data = "kvtR";
                    }
                    else if (aruco_angle_ < 0)
                    {
                        message.data = "kvtL";
                    }
                    RCLCPP_INFO(node_->get_logger(), "Publishing: '%s'", message.data.c_str());
                    command_pub_->publish(message);
                    rclcpp::spin_some(node_);
                    setStatusRunningAndYield();
                    return BT::NodeStatus::RUNNING;
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "Direction is correct.");
                    return BT::NodeStatus::SUCCESS;
                }
            }
        }
        return BT::NodeStatus::FAILURE;
    }

    void halt() override
    {
        // No operation for halt
    }

private:
    void directionCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        aruco_direction_ = msg->data;
        direction_received_ = true;
    }

    void angleCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        aruco_angle_ = msg->data;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr direction_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    bool aruco_direction_;
    float aruco_angle_;
    bool direction_received_;
};

class MoveForward : public BT::CoroActionNode
{
public:
    MoveForward(const std::string& name, const BT::NodeConfiguration& config)
        : BT::CoroActionNode(name, config), distance_(0.0)
    {
        auto node_name = name + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        node_ = rclcpp::Node::make_shared(node_name);

        distance_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/aruco_distance", 10,
            std::bind(&MoveForward::distanceCallback, this, std::placeholders::_1));

        command_pub_ = node_->create_publisher<std_msgs::msg::String>("command", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        rclcpp::spin_some(node_);  // Spin to process incoming messages
        RCLCPP_INFO(rclcpp::get_logger("MoveForward"), "Distance: %f", distance_);

        auto message = std_msgs::msg::String();
        if (distance_ > 100)
        {
            message.data = "kwkF";
            RCLCPP_INFO(rclcpp::get_logger("MoveForward"), "Publishing: '%s'", message.data.c_str());
            command_pub_->publish(message);
            return BT::NodeStatus::RUNNING;

        }
        else
        {
            message.data = "kbalace";
            command_pub_->publish(message);
            return BT::NodeStatus::SUCCESS;

        }

        return BT::NodeStatus::RUNNING;
    }

private:
    void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        distance_ = msg->data;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    float distance_;
};

class AdjustAngle : public BT::SyncActionNode
{
public:
    AdjustAngle(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(rclcpp::get_logger("AdjustAngle"), "Adjusting angle...");
        return BT::NodeStatus::SUCCESS;
    }
};

class IsAttackRange : public BT::ConditionNode
{
public:
    IsAttackRange(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        bool in_range = true; // replace with actual check
        RCLCPP_INFO(rclcpp::get_logger("IsAttackRange"), "Is in attack range: %s", in_range ? "true" : "false");
        return in_range ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class Attack : public BT::SyncActionNode
{
public:
    Attack(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(rclcpp::get_logger("Attack"), "Attacking...");
        return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CorrectDirection>("CorrectDirection");
    factory.registerNodeType<MoveForward>("MoveForward");
    factory.registerNodeType<AdjustAngle>("AdjustAngle");
    factory.registerNodeType<IsAttackRange>("IsAttackRange");
    factory.registerNodeType<Attack>("Attack");

    auto tree = factory.createTreeFromFile("/home/borot/Desktop/Bittle_BehaviorTree/src/behavior_tree_cpp/trees/tree.xml");

    // Monitor the execution using a ZMQ Publisher
    
    std::unique_ptr<BT::PublisherZMQ> publisher_zmq;

    publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree);

    rclcpp::Rate rate(10);

    while (rclcpp::ok())
    {
        BT::NodeStatus status = tree.tickRoot();
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_INFO(rclcpp::get_logger("main"), "Tree execution finished with status: %s", status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
            tree.haltTree();
            tree = factory.createTreeFromFile("/home/borot/Desktop/Bittle_BehaviorTree/src/behavior_tree_cpp/trees/tree.xml");
            publisher_zmq.reset();
            publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree);
        }
        rclcpp::spin_some(rclcpp::Node::make_shared("spin_node"));
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}