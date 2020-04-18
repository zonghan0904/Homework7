#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;
  tf::Transform tf_A_B, tf_A_C, tf_C_B;
  Eigen::Quaterniond q(1, 0, 0, 0);
  Eigen::Vector3d v_A_B(1, 0, 0), v_A_C(0, 1, 0);

  tf::Vector3 tf_v_A_B(v_A_B(0), v_A_B(1), v_A_B(2));
  tf::Quaternion tf_q_A_B(q.x(), q.y(), q.z(), q.w());

  tf_A_B.setOrigin(tf_v_A_B);
  tf_A_B.setRotation(tf_q_A_B);

  tf::Vector3 tf_v_A_C(v_A_C(0), v_A_C(1), v_A_C(2));
  tf::Quaternion tf_q_A_C(q.x(), q.y(), q.z(), q.w());

  tf_A_C.setOrigin(tf_v_A_C);
  tf_A_C.setRotation(tf_q_A_C);

  tf_C_B = tf_A_C.inverse() * tf_A_B;

  br.sendTransform(tf::StampedTransform(tf_A_B, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "A", // paranet frame ID
                                        "B")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_A_C,
                                        ros::Time::now(),
                                        "A",
                                        "C"));

  br.sendTransform(tf::StampedTransform(tf_C_B,
                                        ros::Time::now(),
                                        "C",
                                        "B"));

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ncrl_tf_learning");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
