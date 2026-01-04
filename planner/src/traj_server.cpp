#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/poly_traj.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gcopter/trajectory.hpp>
#include <gcopter/funs.hpp>
#include <gcopter/flatness.hpp>

using namespace Eigen;
using namespace gcopter;
using namespace flatness;

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_cmd_pub;

quadrotor_msgs::msg::PositionCommand cmd;
std_msgs::msg::Bool cmdb;

bool receive_traj_ = false;
bool hasTrigger = false;
boost::shared_ptr<Trajectory<5>> traj_;
double traj_duration_;
rclcpp::Time start_time_;
Eigen::Vector3d last_pos_;
Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero()), snp(Eigen::Vector3d::Zero());

// yaw control
Eigen::Vector3d last_yaws_, yaws_;

void polyTrajCallback(const quadrotor_msgs::msg::PolyTraj::SharedPtr msg)
{
  if ((int)msg->order != 5)
  {
    RCLCPP_ERROR(node_->get_logger(), "[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * ((int)msg->order + 1) != msg->coef_x.size())
  {
    RCLCPP_ERROR(node_->get_logger(), "[traj_server] WRONG trajectory parameters");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<typename Piece<5>::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new Trajectory<5>(dura, cMats));
  traj_duration_ = traj_->getTotalDuration();
  std::cout << "uav has receive traj!" << std::endl;
  receive_traj_ = true;
}

Eigen::Vector3d calculate_yaw()
{
  Eigen::Vector3d yaws;
  yaws.setZero();
  return yaws;
}

void publish_quad_cmd()
{

  cmd.header.stamp = node_->now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.jerk.x = jer(0);
  cmd.jerk.y = jer(1);
  cmd.jerk.z = jer(2);
  cmd.snap.x = snp(0);
  cmd.snap.y = snp(1);
  cmd.snap.z = snp(2);

  cmd.yaw = yaws_(0);
  cmd.yaw_dot = yaws_(1);
  cmd.yaw_acc = yaws_(2);
  cmd_pub->publish(cmd);

  // last_pos_ = p;
}


void triggerCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if ((!receive_traj_))
    return;

  hasTrigger = true;
  RCLCPP_WARN(node_->get_logger(), "hasTrigger !");
  start_time_ = node_->now();
}

void cmdCallback()
{
  if (!hasTrigger)
  {
    return;
  }
  // cout << "ABCD"<< endl;
  rclcpp::Time time_now = node_->now();

  double t_cur = (time_now - start_time_).seconds();

  std::pair<double, double> yaw_yawdot(0, 0);
  (void)yaw_yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);
    snp = traj_->getSnp(t_cur);

    /*** calculate yaw ***/
    yaws_ = calculate_yaw();
    last_yaws_ = yaws_;
    // last_pos_ = pos;

    // publish
    publish_quad_cmd();
  }
  else if (t_cur > traj_duration_)
  {
    cmdb.data = true;
    stop_cmd_pub->publish(cmdb);
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node_ = rclcpp::Node::make_shared("traj_server");

  auto poly_traj_load_sub = node_->create_subscription<quadrotor_msgs::msg::PolyTraj>(
      "planning/trajectory", 10, polyTrajCallback);
  auto trigger_sub = node_->create_subscription<std_msgs::msg::Empty>(
      "planning/trigger", 10, triggerCallback);

  cmd_pub = node_->create_publisher<quadrotor_msgs::msg::PositionCommand>("/pos_cmd", 50);
  stop_cmd_pub = node_->create_publisher<std_msgs::msg::Bool>("planning/stop", 50);

  auto cmd_timer = node_->create_wall_timer(std::chrono::milliseconds(1), cmdCallback);

  (void)poly_traj_load_sub;
  (void)trigger_sub;
  (void)cmd_timer;

  last_yaws_.setZero();

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node_->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node_);

  return 0;
}
