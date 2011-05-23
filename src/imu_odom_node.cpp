/** @file
*
* @brief IMU simple odometer
*
* This is a ROS simple node that cumulates IMU readings to produce odometry
* information. Orientation and linear velocity are computed integrating
* angular speeds and accelerations respectively. Covariances are
* accumulated too. It also sets the transforms between a base frame and the
* current computed pose.
*
* @par Subscribes
*
*  - @b data topic (sensor_msgs/Imu) IMU data
*
* @par Advertises
*
* - @b imu_odom topic (nav_msgs/Odometry) Pose and twist computed from IMU's data
*
* @par Parameters
*
* - @b frame_id absolute frame
* - @b child_frame_id IMU's relative frame*
*/


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

class ImuOdom
{
public:
  ImuOdom();

  void initParams();

  void advertiseOutputTopics();

  void subscribeInputTopics();

private:
  ros::NodeHandle node_;
  ros::NodeHandle priv_;

  // subscribers and publishers
  ros::Subscriber imu_subs_;
  ros::Publisher odom_publ_;
  nav_msgs::Odometry odom_msg_;

  // frames
  std::string frame_id_;
  std::string child_frame_id_;

  // transforms
  tf::TransformBroadcaster odom_broadcaster_;
  tf::StampedTransform odom_tf_;

  // integrated magnitudes in the initial frame
  tf::Vector3 lin_vel_;
  tf::Quaternion ori_quat_;

  // time
  ros::Time last_time_, current_time_;
  
  // float precision
  double epsilon_;

  // odometer initialized
  bool odom_ready_;

  // standard gravity constant
  static const double G_STD_ = 9.80665;

  bool initializeOdom(const sensor_msgs::ImuConstPtr& data);
  void updateOdom(const sensor_msgs::ImuConstPtr& data);
  void dataReceivedCallback(const sensor_msgs::ImuConstPtr& data);

};


ImuOdom::ImuOdom()
: odom_ready_(false)
{}

void ImuOdom::initParams()
{
  // tf frame parameters
  priv_.param<std::string>("odometer_frame_id",frame_id_,"imu_odom");
  priv_.param<std::string>("imu_frame_id",child_frame_id_,"imu");
  priv_.param("epsilon",epsilon_,1e-3);

  odom_msg_.header.frame_id = frame_id_;
  odom_msg_.child_frame_id = child_frame_id_;
  odom_tf_.frame_id_ = frame_id_;
  odom_tf_.child_frame_id_ = child_frame_id_;

  lin_vel_ = tf::Vector3(0.0, 0.0, 0.0);
  ori_quat_ = tf::createIdentityQuaternion();
  current_time_ = ros::Time::now();
}

void ImuOdom::advertiseOutputTopics()
{
  ROS_INFO_STREAM("[imu_odom] Advertising output topics");
  odom_publ_ = node_.advertise<nav_msgs::Odometry>("imu_odom", 1);
}

void ImuOdom::subscribeInputTopics()
{
  ROS_INFO_STREAM("[imu_odom] Subscribing to IMU topic");
  imu_subs_ = node_.subscribe<sensor_msgs::Imu>("data", 1,
                                                &ImuOdom::dataReceivedCallback,
                                                this);
}

bool ImuOdom::initializeOdom(const sensor_msgs::ImuConstPtr& data)
{
  tf::Vector3 curr_g;
  tf::vector3MsgToTF(data->linear_acceleration, curr_g);
  double gx = data->linear_acceleration.x;
  double gy = data->linear_acceleration.y;
  double gz = data->linear_acceleration.z;
  double g_length = sqrt(gx*gx + gy*gy + gz*gz);
  if ( abs(g_length - G_STD_) < epsilon_ )
  {
    ROS_WARN_STREAM("Could not initialize imu data integration "
                    "from a non static condition (imu acceleration is "
                    << g_length << " m/s^2)");
    return false;
  }
  lin_vel_ = tf::Vector3(0.0, 0.0, 0.0);
  ori_quat_.setEuler(0.0,atan2(gz,gx),atan2(gy,gz));
  ROS_INFO_STREAM("Euler is : " << ori_quat_.x() << " " << ori_quat_.y() << " " << ori_quat_.z() );
  ori_quat_.setRPY(0.0,asin(gx/g_length),atan2(gy,gz));
  ROS_INFO_STREAM("RPY is : " << ori_quat_.x() << " " << ori_quat_.y() << " " << ori_quat_.z() );
  current_time_ = data->header.stamp;
  return true;
}

void ImuOdom::updateOdom(const sensor_msgs::ImuConstPtr& data)
{
  // update time values
  last_time_ = current_time_;
  current_time_ = data->header.stamp;
  double dt = (current_time_ - last_time_).toSec();

  // update velocity in body-fixed frame
  tf::Vector3 lin_acc;
  tf::vector3MsgToTF(data->linear_acceleration, lin_acc);
  lin_acc -= odom_tf_*tf::Vector3(0,0,G_STD_); // correct gravity
  lin_vel_ += dt * lin_acc;
  
  // update orientation in initial frame
  // (according to p. 15 of Guidance and Control of Ocean Vehicles by Fossen)
  tf::Vector3 ang_vel;
  tf::vector3MsgToTF(data->angular_velocity, ang_vel);
  ori_quat_ += (ori_quat_*ang_vel) * (dt*0.5);
  ori_quat_.normalize();
  
  // update transform with new orientation (do it now to use it just below)
  odom_tf_.stamp_ = current_time_;
  odom_tf_.setRotation(ori_quat_);

  // TODO check linear velocity frame and covariances
  
  // update odometry message
  odom_msg_.header.stamp = current_time_;
  odom_msg_.twist.twist.angular = data->angular_velocity;
  tf::vector3TFToMsg(lin_vel_, odom_msg_.twist.twist.linear);
  tf::quaternionTFToMsg(ori_quat_, odom_msg_.pose.pose.orientation);
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
    {
      odom_msg_.pose.covariance [   6*i+j]  = (i==j) ? -1.0 : 0.0;
      odom_msg_.pose.covariance [21+6*i+j] += data->orientation_covariance[3*i+j]*dt*dt;
      odom_msg_.twist.covariance[   6*i+j] += data->linear_acceleration_covariance[3*i+j]*dt*dt;
      odom_msg_.twist.covariance[21+6*i+j]  = data->angular_velocity_covariance[3*i+j];
    }
}

void ImuOdom::dataReceivedCallback(const sensor_msgs::ImuConstPtr& data)
{
  
  if (odom_ready_)
  {

    // update pose and twist from new sample
    updateOdom(data);
    // send pose and transform
    odom_broadcaster_.sendTransform(odom_tf_);
    odom_publ_.publish(odom_msg_);
  }
  else
  {
    // initialize the odometer from a static condition at first message arrival
    odom_ready_ = initializeOdom(data);
  }
}


int main(int argc, char* argv[])
{
  ros::init(argc,argv,"imu_odom_node");

  ImuOdom imu_odom;

  imu_odom.initParams();

  imu_odom.advertiseOutputTopics();

  imu_odom.subscribeInputTopics();

  ros::spin();
}
