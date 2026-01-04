#include <algorithm>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/poly_traj.hpp>
#include <quadrotor_msgs/msg/position_command_new.hpp>
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
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommandNew>::SharedPtr cmd_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_cmd_pub;

quadrotor_msgs::msg::PositionCommandNew cmd;
std_msgs::msg::Bool cmdb;

bool receive_traj_load_ = false;
bool receive_traj_cable_ = false;
bool hasTriggerFlight = false;
bool hasTriggerTakeOff = false;
bool hasTriggerSwitchControl = false;
boost::shared_ptr<Trajectory<7>> traj_load_;
boost::shared_ptr<Trajectory<7>> traj_cable_;
double traj_duration_, transient_time_;
double len, initHeight, startHeight, initYaw, takeOffVel;
rclcpp::Time start_time_, start_takeoff_time_;
int drone_id_, drone_num_;
int flag;
Eigen::Vector3d last_pos_;
Eigen::Vector3d posL(Eigen::Vector3d::Zero()), velL(Eigen::Vector3d::Zero()), accL(Eigen::Vector3d::Zero()), jerL(Eigen::Vector3d::Zero()), snpL(Eigen::Vector3d::Zero()), posQ(Eigen::Vector3d::Zero()), velQ(Eigen::Vector3d::Zero()), accQ(Eigen::Vector3d::Zero()), jerQ(Eigen::Vector3d::Zero()), snpQ(Eigen::Vector3d::Zero()), angT(Eigen::Vector3d::Zero()), dAngT(Eigen::Vector3d::Zero()), d2AngT(Eigen::Vector3d::Zero()), d3AngT(Eigen::Vector3d::Zero()), d4AngT(Eigen::Vector3d::Zero());

// yaw control
Eigen::Vector3d last_yaws_, yaws_, init_pos;
std::vector<double> inifinAngT, initLoadPos;
// double kk = 0.0;

void polyTrajCallback(const quadrotor_msgs::msg::PolyTraj::SharedPtr msg)
{
  if (((int)msg->type == 0 && (int)msg->drone_id != 0) || ((int)msg->type == 1 && (int)msg->drone_id != drone_id_))
  {
    RCLCPP_ERROR(node_->get_logger(), "[traj_server] received drone_id != drone_id ");
    return;
  }
  if ((int)msg->order != 7)
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
  std::vector<typename Piece<7>::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i8 = i * 8;
    cMats[i].row(0) << msg->coef_x[i8 + 0], msg->coef_x[i8 + 1], msg->coef_x[i8 + 2],
        msg->coef_x[i8 + 3], msg->coef_x[i8 + 4], msg->coef_x[i8 + 5], msg->coef_x[i8 + 6], msg->coef_x[i8 + 7];
    cMats[i].row(1) << msg->coef_y[i8 + 0], msg->coef_y[i8 + 1], msg->coef_y[i8 + 2],
        msg->coef_y[i8 + 3], msg->coef_y[i8 + 4], msg->coef_y[i8 + 5], msg->coef_y[i8 + 6], msg->coef_y[i8 + 7];
    cMats[i].row(2) << msg->coef_z[i8 + 0], msg->coef_z[i8 + 1], msg->coef_z[i8 + 2],
        msg->coef_z[i8 + 3], msg->coef_z[i8 + 4], msg->coef_z[i8 + 5], msg->coef_z[i8 + 6], msg->coef_z[i8 + 7];

    dura[i] = msg->duration[i];
  }

  if ((int)msg->type == 0)
  {
    traj_load_.reset(new Trajectory<7>(dura, cMats));
    traj_duration_ = traj_load_->getTotalDuration();
    std::cout << "uav" << drone_id_ << "\t have receive load traj!" << std::endl;
    receive_traj_load_ = true;
  }
  else if ((int)msg->type == 1)
  {
    traj_cable_.reset(new Trajectory<7>(dura, cMats));
    std::cout << "uav" << drone_id_ << "\t have receive cable traj!" << std::endl;
    receive_traj_cable_ = true;
  }
}

Eigen::Vector3d calculate_yaw()
{
  Eigen::Vector3d yaws;
  yaws.setZero();
  return yaws;
}

void publish_quad_cmd(const int &flag)
{

  cmd.header.stamp = node_->now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommandNew::TRAJECTORY_STATUS_READY;
  cmd.flag = flag;
  cmd.position.x = posL(0);
  cmd.position.y = posL(1);
  cmd.position.z = posL(2);
  cmd.velocity.x = velL(0);
  cmd.velocity.y = velL(1);
  cmd.velocity.z = velL(2);
  cmd.acceleration.x = accL(0);
  cmd.acceleration.y = accL(1);
  cmd.acceleration.z = accL(2);
  cmd.jerk.x = jerL(0);
  cmd.jerk.y = jerL(1);
  cmd.jerk.z = jerL(2);
  cmd.snap.x = snpL(0);
  cmd.snap.y = snpL(1);
  cmd.snap.z = snpL(2);
  cmd.ang_t.x = angT(0);
  cmd.ang_t.y = angT(1);
  cmd.ang_t.z = angT(2);
  cmd.d_ang_t.x = dAngT(0);
  cmd.d_ang_t.y = dAngT(1);
  cmd.d_ang_t.z = dAngT(2);
  cmd.d2_ang_t.x = d2AngT(0);
  cmd.d2_ang_t.y = d2AngT(1);
  cmd.d2_ang_t.z = d2AngT(2);
  cmd.d3_ang_t.x = d3AngT(0);
  cmd.d3_ang_t.y = d3AngT(1);
  cmd.d3_ang_t.z = d3AngT(2);
  cmd.d4_ang_t.x = d4AngT(0);
  cmd.d4_ang_t.y = d4AngT(1);
  cmd.d4_ang_t.z = d4AngT(2);

  cmd.yaw = yaws_(0);
  cmd.yaw_dot = yaws_(1);
  cmd.yaw_acc = yaws_(2);
  cmd_pub->publish(cmd);

  // last_pos_ = p;
}

void triggerCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if ((!receive_traj_load_) || (!receive_traj_cable_))
    return;

  hasTriggerFlight = true;
  hasTriggerSwitchControl = false;
  hasTriggerTakeOff = false;
  RCLCPP_WARN(node_->get_logger(), "hasTriggerFlight !");
  start_time_ = node_->now();
}

void triggerSwitchControlCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  // if ((!receive_traj_load_) || (!receive_traj_cable_))
  //   return;

  hasTriggerSwitchControl = true;
  hasTriggerTakeOff = false;
  RCLCPP_WARN(node_->get_logger(), "hasTriggerSwitchControl !");
  start_time_ = node_->now();
}

void triggerTakeOffCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  hasTriggerTakeOff = true;
  RCLCPP_WARN(node_->get_logger(), "hasTakeOffTrigger !");
  start_takeoff_time_ = node_->now();
}

void cmdCallback()
{
  if (!hasTriggerFlight && !hasTriggerTakeOff && !hasTriggerSwitchControl)
  {
    return;
  }
  // cout << "ABCD"<< endl;
  rclcpp::Time time_now = node_->now();

  if (hasTriggerTakeOff)
  {
    double t_cur = (time_now - start_takeoff_time_).seconds();
    posL.x() = init_pos.x();
    posL.y() = init_pos.y();
    velL.x() = 0.0;
    velL.y() = 0.0;
    if (t_cur <= transient_time_)
    {
      posL.z() = init_pos.z();
      velL.z() = 0.0;
    }
    else if (t_cur <= transient_time_ + (startHeight - init_pos.z()) / takeOffVel)
    {
      posL.z() = std::min(init_pos.z() + takeOffVel * (t_cur - transient_time_), startHeight);
      velL.z() = takeOffVel;
    }
    else
    {
      posL.z() = startHeight;
      velL.z() = 0.0;
    }

    accL = Eigen::Vector3d::Zero();
    jerL = Eigen::Vector3d::Zero();
    snpL = Eigen::Vector3d::Zero();
    angT = Eigen::Vector3d::Zero();
    dAngT = Eigen::Vector3d::Zero();
    d2AngT = Eigen::Vector3d::Zero();
    d3AngT = Eigen::Vector3d::Zero();
    d4AngT = Eigen::Vector3d::Zero();
    yaws_ = calculate_yaw();
    flag = 0;
    publish_quad_cmd(flag);
    // cout << "aaaa" << endl;

  }
  else if (hasTriggerSwitchControl)
  {
    posL.x() = init_pos.x();
    posL.y() = init_pos.y();
    posL.z() = startHeight;
    velL = Eigen::Vector3d::Zero();
    accL = Eigen::Vector3d::Zero();
    jerL = Eigen::Vector3d::Zero();
    snpL = Eigen::Vector3d::Zero();
    angT = Eigen::Vector3d::Zero();
    dAngT = Eigen::Vector3d::Zero();
    d2AngT = Eigen::Vector3d::Zero();
    d3AngT = Eigen::Vector3d::Zero();
    d4AngT = Eigen::Vector3d::Zero();
    yaws_ = calculate_yaw();
    flag = 1;
    publish_quad_cmd(flag);
  }
  else if (hasTriggerFlight)
  {
    double t_cur = (time_now - start_time_).seconds();
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
      posL = traj_load_->getPos(t_cur);
      velL = traj_load_->getVel(t_cur);
      accL = traj_load_->getAcc(t_cur);
      jerL = traj_load_->getJer(t_cur);
      snpL = traj_load_->getSnp(t_cur);

      posQ = traj_cable_->getPos(t_cur);
      velQ = traj_cable_->getVel(t_cur);
      accQ = traj_cable_->getAcc(t_cur);
      jerQ = traj_cable_->getJer(t_cur);
      snpQ = traj_cable_->getSnp(t_cur);

      angT = posQ - posL;
      dAngT = velQ - velL;
      d2AngT = accQ - accL;
      d3AngT = jerQ - jerL;
      d4AngT = snpQ - snpL;

      /*** calculate yaw ***/
      yaws_ = calculate_yaw();
      last_yaws_ = yaws_;
      // last_pos_ = pos;

      // publish
      flag = 2;
      publish_quad_cmd(flag);
    }
    else if (t_cur > traj_duration_)
    {
      cmdb.data = true;
      stop_cmd_pub->publish(cmdb);
    }
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node_ = rclcpp::Node::make_shared("traj_server_transport");

  drone_id_ = node_->declare_parameter<int>("drone_id", 1);
  drone_num_ = node_->declare_parameter<int>("drone_num", 1);
  len = node_->declare_parameter<double>("len", 1.0);
  initHeight = node_->declare_parameter<double>("init_height", 0.0);
  startHeight = node_->declare_parameter<double>("start_height", 0.0);
  initYaw = node_->declare_parameter<double>("init_yaw", 0.0);
  takeOffVel = node_->declare_parameter<double>("takeoff_vel", 0.0);
  transient_time_ = node_->declare_parameter<double>("transient_time", 0.0);
  inifinAngT = node_->declare_parameter<std::vector<double>>("inifinAngT", {});
  initLoadPos = node_->declare_parameter<std::vector<double>>("initLoadPos", {});

  cmd_pub = node_->create_publisher<quadrotor_msgs::msg::PositionCommandNew>("/pos_cmd", 50);
  stop_cmd_pub = node_->create_publisher<std_msgs::msg::Bool>("planning/stop", 50);

  auto poly_traj_load_sub = node_->create_subscription<quadrotor_msgs::msg::PolyTraj>(
      "planning/trajectoryLoad", 10, polyTrajCallback);
  auto poly_traj_cable_sub = node_->create_subscription<quadrotor_msgs::msg::PolyTraj>(
      "planning/trajectoryCable", 10, polyTrajCallback);
  auto trigger_sub = node_->create_subscription<std_msgs::msg::Empty>(
      "planning/triggerFlight", 10, triggerCallback);
  auto trigger_takeoff_sub = node_->create_subscription<std_msgs::msg::Empty>(
      "planning/triggerTakeOff", 10, triggerTakeOffCallback);
  auto trigger_switch_control_sub = node_->create_subscription<std_msgs::msg::Empty>(
      "planning/triggerSwitchControl", 10, triggerSwitchControlCallback);

  auto cmd_timer = node_->create_wall_timer(std::chrono::milliseconds(3), cmdCallback);

  (void)poly_traj_load_sub;
  (void)poly_traj_cable_sub;
  (void)trigger_sub;
  (void)trigger_takeoff_sub;
  (void)trigger_switch_control_sub;
  (void)cmd_timer;

  last_yaws_.setZero();

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node_->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node_);

  return 0;
}
