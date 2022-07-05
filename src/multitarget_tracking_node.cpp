#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "multitarget_tracking_node.h"
#include "ros_multitarget_tracking/Gaussian2D.h"
#include "ros_multitarget_tracking/GaussianMixture2D.h"

// Initialize Gaussian Mixture component & belief
GaussianMixtureComponent2D InitialGMComp;


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  // Initialize node
  ros::init(argc, argv, "multitarget_tracker");
  ros::NodeHandle n;

  // With nodehandle created, populate initial belief GM with node's launch parameters
  n.param<float> ("mu_x", InitialGMComp.muX, 0.0);
  n.param<float> ("mu_y", InitialGMComp.muY, 0.0);
  n.param<float> ("sigma_x", InitialGMComp.sigmaX, 0.5);
  n.param<float> ("sigma_y", InitialGMComp.sigmaY, 0.5);
  n.param<float> ("weight", InitialGMComp.weight, 1.0);
  Belief.push_back(InitialGMComp);

  // Initialize publisher and messages
  ros::Publisher state_pub = n.advertise<ros_multitarget_tracking::GaussianMixture2D>("multitarget_state", 1000);
  ros_multitarget_tracking::GaussianMixture2D BeliefMsg;
  ros_multitarget_tracking::Gaussian2D GaussianMsg;

  ros::Rate loop_rate(10);

  // Main loop
  while (ros::ok())
  {

    // Populate state belief message with current Gaussians
    BeliefMsg.models.clear();
  
    for (auto model : Belief) {
      GaussianMsg.weight = model.weight;
      GaussianMsg.mu_x = model.muX;
      GaussianMsg.mu_y = model.muY;
      GaussianMsg.sigma_x = model.sigmaX;
      GaussianMsg.sigma_y = model.sigmaY;
      BeliefMsg.models.push_back(GaussianMsg);

    } // for model : Belief

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    state_pub.publish(BeliefMsg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}