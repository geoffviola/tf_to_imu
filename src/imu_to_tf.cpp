#include <cstdlib>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

using std::string;

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
                                 double scalar)
{
  geometry_msgs::Vector3 output;
  output.x = lhs.x * scalar;
  output.y = lhs.y * scalar;
  output.z = lhs.z * scalar;
  return output;
}

geometry_msgs::Vector3 operator*(double scalar,
                                 geometry_msgs::Vector3 const &rhs)
{
  geometry_msgs::Vector3 output;
  output.x = rhs.x * scalar;
  output.y = rhs.y * scalar;
  output.z = rhs.z * scalar;
  return output;
}

class ImuToTf
{
public:
  ImuToTf(ros::NodeHandle *const node_handle, string const &child_frame_in)
      : imu_sub(
      node_handle->subscribe("imu/data", 10, &ImuToTf::imuCallback, this))
        , child_frame(child_frame_in)
  {
    prev_transform.transform.translation.x = 0.0;
    prev_transform.transform.translation.y = 0.0;
    prev_transform.transform.translation.z = 0.0;
    prev_transform.transform.rotation.x = 0.0;
    prev_transform.transform.rotation.y = 0.0;
    prev_transform.transform.rotation.z = 0.0;
    prev_transform.transform.rotation.w = 1.0;
  }

private:
  void imuCallback(sensor_msgs::Imu::ConstPtr imu_msg)
  {
    geometry_msgs::TransformStamped transform;
    transform.header = imu_msg->header;
    transform.child_frame_id = this->child_frame;
    double const dt_s = imu_msg->header.stamp.toSec() -
                        this->prev_transform.header.stamp.toSec();

    transform.transform.translation =
        this->prev_transform.transform.translation +
        0.5 * imu_msg->linear_acceleration * dt_s * dt_s;
    transform.transform.translation.z -= 0.5 * 9.81 * dt_s * dt_s;

    transform.transform.rotation = imu_msg->orientation;

    this->transform_broadcaster.sendTransform(transform);

    this->prev_transform = transform;
  }

  geometry_msgs::TransformStamped prev_transform;
  ros::Subscriber imu_sub;
  tf2_ros::TransformBroadcaster transform_broadcaster;
  string child_frame;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "imu_to_tf");

  ros::NodeHandle p_node_handle("~");
  string const child_frame = p_node_handle.param("child_frame",
                                                 string("child_frame"));

  ros::NodeHandle node_handle;
  ImuToTf imu_to_tf(&node_handle, child_frame);
  ros::spin();

  return EXIT_SUCCESS;
}
