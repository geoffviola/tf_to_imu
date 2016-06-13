#include <cstdlib>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using std::string;

static char const* const IMU_TOPIC_NAME = "imu/data";

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

geometry_msgs::Vector3 operator/(geometry_msgs::Vector3 const &lhs,
                                 double const scalar)
{
  geometry_msgs::Vector3 output;
  output.x = lhs.x / scalar;
  output.y = lhs.y / scalar;
  output.z = lhs.z / scalar;
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
    this->prev_velocities.x = 0.0;
    this->prev_velocities.y = 0.0;
    this->prev_velocities.z = 0.0;
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

    geometry_msgs::Vector3 const delta_velocities_bf =
        imu_msg->linear_acceleration * dt_s;
    geometry_msgs::Vector3 delta_velocities_if = quat_rotate(
        delta_velocities_bf, imu_msg->orientation);
    delta_velocities_if.z -= 9.81 * dt_s;
    geometry_msgs::Vector3 velocities = this->prev_velocities + 
        delta_velocities_if;
    transform.transform.translation = 0.5 * delta_velocities_if * dt_s;
    transform.transform.translation += velocities * dt_s;
    transform.transform.translation +=
        this->prev_transform.transform.translation;

    transform.transform.rotation = imu_msg->orientation;

    this->transform_broadcaster.sendTransform(transform);

    this->prev_transform = transform;
    this->prev_velocities = velocities;
  }

  ros::NodeHandle *node_handle;
  geometry_msgs::TransformStamped prev_transform;
  ros::Subscriber imu_sub;
  tf2_ros::TransformBroadcaster transform_broadcaster;
  string parent_frame;
  string child_frame;
  geometry_msgs::Vector3 prev_velocities;
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
