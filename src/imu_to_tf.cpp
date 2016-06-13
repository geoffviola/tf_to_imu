#include <cstdlib>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using std::string;

static char const* const IMU_TOPIC_NAME = "imu/data";

double acceleration_to_position(double a_m_s2, double dt_s);

double acceleration_to_position(double const a_m_s2, double const dt_s)
{
  return 0.5 * a_m_s2 * dt_s * dt_s;
}

geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 const &lhs,
                                 geometry_msgs::Vector3 const &rhs)
{
  geometry_msgs::Vector3 output;
  output.x = lhs.x + rhs.x;
  output.y = lhs.y + rhs.y;
  output.z = lhs.z + rhs.z;
  return output;
}

geometry_msgs::Vector3 operator*(geometry_msgs::Vector3 const &lhs,
                                 double const scalar)
{
  geometry_msgs::Vector3 output;
  output.x = lhs.x * scalar;
  output.y = lhs.y * scalar;
  output.z = lhs.z * scalar;
  return output;
}

geometry_msgs::Quaternion operator-(geometry_msgs::Quaternion const& rhs)
{
  double const sum_squared = rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z +
      rhs.w * rhs.w;
  geometry_msgs::Quaternion output;
  output.x = -rhs.x / sum_squared;
  output.y = -rhs.y / sum_squared;
  output.z = -rhs.z / sum_squared;
  output.w = rhs.w / sum_squared;
  return output;
}

geometry_msgs::Vector3& operator+=(geometry_msgs::Vector3& lhs,
                                   geometry_msgs::Vector3 const& rhs)
{
  lhs.x = lhs.x + rhs.x;
  lhs.y = lhs.y + rhs.y;
  lhs.z = lhs.z + rhs.z;
  return lhs;
}

geometry_msgs::Vector3 operator*(double const scalar,
                                 geometry_msgs::Vector3 const &rhs)
{
  return rhs * scalar;
}

geometry_msgs::Vector3 quat_rotate(geometry_msgs::Vector3 const& point,
    geometry_msgs::Quaternion const& quat)
{
  tf2::Vector3 const tf2_point(point.x, point.y, point.z);
  tf2::Quaternion const tf2_quat(quat.x, quat.y, quat.z, quat.w);
  tf2::Vector3 const tf2_output = tf2::quatRotate(tf2_quat, tf2_point);
  geometry_msgs::Vector3 output;
  output.x = tf2_output.getX();
  output.y = tf2_output.getY();
  output.z = tf2_output.getZ();
  return output;
}

class ImuToTf
{
public:
  ImuToTf(ros::NodeHandle *const node_handle_in, string const &parent_frame_in,
          string const& child_frame_in)
      : node_handle(node_handle_in)
      , imu_sub(
      node_handle->subscribe(IMU_TOPIC_NAME, 10, &ImuToTf::imuCallbackFirst, this))
      , parent_frame(parent_frame_in)
      , child_frame(child_frame_in)
  {
  }

private:
  void imuCallbackFirst(sensor_msgs::Imu::ConstPtr imu_msg)
  {
    this->prev_transform.header = imu_msg->header;
    this->prev_transform.transform.rotation = imu_msg->orientation;
    this->prev_transform.transform.translation.x = 0.0;
    this->prev_transform.transform.translation.y = 0.0;
    this->prev_transform.transform.translation.z = 0.0;
    this->imu_sub = this->node_handle->subscribe(
        IMU_TOPIC_NAME, 10, &ImuToTf::imuCallback, this);
  }

  void imuCallback(sensor_msgs::Imu::ConstPtr imu_msg)
  {
    geometry_msgs::TransformStamped transform;
    transform.header.seq = imu_msg->header.seq;
    transform.header.stamp = imu_msg->header.stamp;
    transform.header.frame_id = this->parent_frame;
    transform.child_frame_id = this->child_frame;
    double const dt_s = imu_msg->header.stamp.toSec() -
                        this->prev_transform.header.stamp.toSec();
    if (dt_s <= 0.0)
    {
      ROS_ERROR("dts = %f", dt_s);
    }

    ROS_INFO("%f, %f, %f; %f, %f, %f; %f",
        this->prev_transform.transform.translation.x,
        this->prev_transform.transform.translation.y,
        this->prev_transform.transform.translation.z,
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z,
        dt_s);

    transform.transform.translation =
        0.5 * imu_msg->linear_acceleration * dt_s * dt_s;
    transform.transform.translation.z -= 0.5 * 9.81 * dt_s * dt_s;
    transform.transform.translation = quat_rotate(
        transform.transform.translation, imu_msg->orientation);
    transform.transform.translation +=
        this->prev_transform.transform.translation;

    transform.transform.rotation = imu_msg->orientation;
    // ROS_INFO("%f, %f, %f; %f, %f, %f, %f",
    //      transform.transform.translation.x,
    //      transform.transform.translation.y,
    //      transform.transform.translation.z,
    //      imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);

    this->transform_broadcaster.sendTransform(transform);

    this->prev_transform = transform;
  }

  ros::NodeHandle *node_handle;
  geometry_msgs::TransformStamped prev_transform;
  ros::Subscriber imu_sub;
  tf2_ros::TransformBroadcaster transform_broadcaster;
  string parent_frame;
  string child_frame;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "imu_to_tf");

  ros::NodeHandle p_node_handle("~");
  string const parent_frame = p_node_handle.param("parent_frame",
                                                 string("parent_frame"));
  string const child_frame = p_node_handle.param("child_frame",
                                                 string("child_frame"));

  ros::NodeHandle node_handle;
  ImuToTf imu_to_tf(&node_handle, parent_frame, child_frame);
  ros::spin();

  return EXIT_SUCCESS;
}
