#include <cstdlib>
#include <ros/init.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <tuple>
#include <tf2/LinearMath/Quaternion.h>

using std::tuple;
using std::get;
using std::string;

tuple<bool, geometry_msgs::TransformStamped>
    maybe_get_transform(string const &parent_frame, string const &child_frame,
                        const tf2_ros::Buffer &buffer);

geometry_msgs::TransformStamped get_transform(string const &parent_frame,
                                              string const &child_frame,
                                              const tf2_ros::Buffer &buffer);

tuple<bool, geometry_msgs::TransformStamped>
maybe_get_transform(string const &parent_frame, string const &child_frame,
                    const tf2_ros::Buffer &buffer)
{
  bool got_transform = false;
  geometry_msgs::TransformStamped current_transformation;
  try
  {
    current_transformation = buffer.lookupTransform(parent_frame,
                                                    child_frame,
                                                    ros::Time::now(),
                                                    ros::Duration(1.0));
    got_transform = true;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("error: %s", ex.what());
  }
  return tuple<bool, geometry_msgs::TransformStamped>(got_transform,
                                                      current_transformation);
}

geometry_msgs::TransformStamped get_transform(string const &parent_frame,
                                              string const &child_frame,
                                              const tf2_ros::Buffer &buffer)
{
  tuple<bool, geometry_msgs::TransformStamped> prev_transform;
  get<0>(prev_transform) = false;
  while (ros::ok() && !get<0>(prev_transform))
  {
    prev_transform = maybe_get_transform(parent_frame, child_frame, buffer);
  }
  return get<1>(prev_transform);
}

geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 const &lhs,
                                 geometry_msgs::Vector3 const &rhs)
{
  geometry_msgs::Vector3 output;
  output.x = lhs.x - rhs.x;
  output.y = lhs.y - rhs.y;
  output.z = lhs.z - rhs.z;
  return output;
}

geometry_msgs::Vector3 operator/(geometry_msgs::Vector3 const &lhs,
                                 double scalar)
{
  geometry_msgs::Vector3 output;
  output.x = lhs.x / scalar;
  output.y = lhs.y / scalar;
  output.z = lhs.z / scalar;
  return output;
}

geometry_msgs::Vector3 invert_quat_rotate(geometry_msgs::Vector3 const& point,
    geometry_msgs::Quaternion const& quat)
{
  tf2::Vector3 const tf2_point(point.x, point.y, point.z);
  tf2::Quaternion const tf2_quat(quat.x, quat.y, quat.z, quat.w);
  tf2::Vector3 const tf2_output = tf2::quatRotate(tf2_quat.inverse(), tf2_point);
  geometry_msgs::Vector3 output;
  output.x = tf2_output.getX();
  output.y = tf2_output.getY();
  output.z = tf2_output.getZ();
  return output;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf_to_imu");

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tf2_listener(buffer);
  ros::NodeHandle node_handle;

  ros::NodeHandle p_node_handle("~");
  string const parent_frame = p_node_handle.param("parent_frame",
                                                  string("parent_frame"));
  string const child_frame = p_node_handle.param("child_frame",
                                                 string("child_frame"));

  ros::Publisher imu_pub(
      node_handle.advertise<sensor_msgs::Imu>("/imu/data", 5));

  geometry_msgs::TransformStamped prev_prev_transform, prev_transform;
  do
  {
    prev_prev_transform = get_transform(parent_frame, child_frame, buffer);
    prev_transform = get_transform(parent_frame, child_frame, buffer);
  } while (ros::ok() && prev_transform.header.stamp.toSec() -
                        prev_prev_transform.header.stamp.toSec() <= 0.0);

  ros::Rate rate(50);

  while (ros::ok())
  {
    geometry_msgs::TransformStamped current_transform(
        get_transform(parent_frame, child_frame, buffer));
    sensor_msgs::Imu imu_msg;
    imu_msg.header = current_transform.header;
    imu_msg.orientation = current_transform.transform.rotation;
    double const delta_time_s = current_transform.header.stamp.toSec() -
                                prev_transform.header.stamp.toSec();
    double const prev_delta_time_s = prev_transform.header.stamp.toSec() -
                                     prev_prev_transform.header.stamp.toSec();
    if (delta_time_s > 0.0)
    {
      geometry_msgs::Vector3 const delta_translation =
          current_transform.transform.translation -
          prev_transform.transform.translation;
      geometry_msgs::Vector3 const prev_delta_translation =
          prev_transform.transform.translation -
          prev_prev_transform.transform.translation;
      geometry_msgs::Vector3 const average_velocity_mps =
          delta_translation / delta_time_s;
      geometry_msgs::Vector3 const prev_average_velocity_mps =
          prev_delta_translation / prev_delta_time_s;
      geometry_msgs::Vector3 accelerations_inertial_frame =
          (average_velocity_mps - prev_average_velocity_mps) / delta_time_s;
      accelerations_inertial_frame.z += 9.81;
      imu_msg.linear_acceleration = invert_quat_rotate(
          accelerations_inertial_frame,
          current_transform.transform.rotation);
      imu_pub.publish(imu_msg);
      prev_prev_transform = prev_transform;
      prev_transform = current_transform;
    }
    rate.sleep();
  }

  return EXIT_SUCCESS;
}
