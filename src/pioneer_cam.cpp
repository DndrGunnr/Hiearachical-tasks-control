#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <algorithm>
#include <visp/vpSubMatrix.h>
#include <opencv2/highgui.hpp>

using namespace std;


PioneerCam::PioneerCam() : loop_(10), it_(nh_)
{
  // joint setpoint publisher
  joint_pub_  =nh_.advertise<sensor_msgs::JointState>("/joint_setpoint", 1);
  joint_setpoint_.name = {"Pioneer_p3dx_leftMotor", "Pioneer_p3dx_rightMotor", "camera_pan", "camera_tilt"};
  joint_setpoint_.velocity.resize(4);

  // wheels
  radius_ = .0975;
  base_ = .331;
  w_max_ = 4;

  // color detector & camera calibration
  color_detector.detectColor(0,255,0);
  color_detector.setContourDisplay(255,0,0);
  color_detector.fitCircle();
  color_detector.setCamera(640,480,60);

  // joints subscriber
  joint_ok_ = false;
  joint_sub_ = nh_.subscribe("/joint_states", 1, &PioneerCam::readJointState, this);
  joint_names_ = {"camera_pan", "camera_tilt"};
  q_.resize(joint_names_.size());


  // image transport
  im_ok_ = false;
  s_im_.resize(2);
  //cv::startWindowThread();
  im_sub_ = it_.subscribe("/image", 1, &PioneerCam::readImage, this);
  std::cout << "Pioneer init ok" << std::endl;
  sphere_sub_ = nh_.subscribe("/sphere", 1, &PioneerCam::readSpherePose, this);

  // pose
  target_ok_ = false;
  target_sub_ = nh_.subscribe("/target", 1, &PioneerCam::readTargetPose, this);

}


void PioneerCam::sendVelocity(const vpColVector &v)
{
  if(v.size() != 4)
  {
    std::cout << "PioneerCam::setVelocity: v should be length 4, is " << v.size() << std::endl;
    return;
  }

  // change (v,w) to left and right wheel velocities
  joint_setpoint_.velocity[0] = (v[0]-base_*v[1])/radius_;
  joint_setpoint_.velocity[1] = (v[0]+base_*v[1])/radius_;
  double a = std::max(vpMath::abs(joint_setpoint_.velocity[0])/w_max_, vpMath::abs(joint_setpoint_.velocity[1])/w_max_);
  if(a<1)
    a = 1;

  // scale wheel velocities if activated
  if(vpMath::abs(joint_setpoint_.velocity[0])/w_max_ > 1.01)
    cout << "*** Left wheel velocity above limit (" << joint_setpoint_.velocity[0] << ") ***" << endl;
  if(vpMath::abs(joint_setpoint_.velocity[1])/w_max_ > 1.01)
    cout << "*** Right wheel velocity above limit (" << joint_setpoint_.velocity[1] << ") ***" << endl;

  joint_setpoint_.velocity[0] *= 1./a;
  joint_setpoint_.velocity[1] *= 1./a;

  // copy camera joints velocity
  joint_setpoint_.velocity[2] = v[2];
  joint_setpoint_.velocity[3] = v[3];

  joint_pub_.publish(joint_setpoint_);
}


vpMatrix PioneerCam::camJacobian(const vpColVector &_q) const
{
  // autogenerated from pioneer.urdf
  vpMatrix J(6,4);
  const double c1 = cos(_q[0]);
  const double c2 = cos(_q[1]);
  const double s1 = sin(_q[0]);
  const double s2 = sin(_q[1]);

  J[0][0] = s1;
  J[0][1] = -0.155485*c1 - 0.035*c2;
  J[0][2] = -0.035*c2;
  //J[0][3] = 0;
  J[1][0] = s2*c1;
  //J[1][1] = 0;
  //J[1][2] = 0;
  J[1][3] = -0.035;
  J[2][0] = c1*c2;
  //J[2][1] = 0;
  //J[2][2] = 0;
  //J[2][3] = 0;
  //J[3][0] = 0;
  //J[3][1] = 0;
  //J[3][2] = 0;
  J[3][3] = 1.;
  //J[4][0] = 0;
  J[4][1] = -c2;
  J[4][2] = -c2;
  //J[4][3] = 0;
  //J[5][0] = 0;
  J[5][1] = s2;
  J[5][2] = s2;
  //J[5][3] = 0;

  return J;
}


void PioneerCam::readJointState(const sensor_msgs::JointStateConstPtr &_msg)
{
  joint_ok_ = true;
  for(unsigned int j=0;j<joint_names_.size();++j)
  {
    for(unsigned int i=0;i<_msg->name.size();++i)
    {
      if(_msg->name[i] == joint_names_[j])
      {
        q_[j] = _msg->position[i];
        break;
      }
    }
  }
}

void PioneerCam::readImage(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat im_out;
  color_detector.process(msg, im_out);
  cv::imshow("view", im_out);
  cv::waitKey(1);
}


void PioneerCam::readTargetPose(const geometry_msgs::PoseConstPtr &msg)
{
  target_ok_ = true;
  target_pose_.x = msg->position.x;
  target_pose_.y = msg->position.y;
  target_pose_.theta = 2*atan2(msg->orientation.z, msg->orientation.w);
}

void PioneerCam::readSpherePose(const geometry_msgs::PoseConstPtr &msg)
{
  im_ok_ = true;
  s_im_[1] = msg->position.y/msg->position.z;
  if(msg->position.z > 0)
    s_im_[0] = msg->position.x/msg->position.z;
  else if(msg->position.x != 0)
    s_im_[0] =  1.2*std::abs(msg->position.x)*color_detector.xLim()/msg->position.x;
}
