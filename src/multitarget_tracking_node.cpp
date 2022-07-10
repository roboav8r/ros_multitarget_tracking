#include <sstream>

//#include <Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "multitarget_tracking_node.h"
#include "ros_multitarget_tracking/Gaussian2D.h"
#include "ros_multitarget_tracking/GaussianMixture2D.h"

// Debug independent assert statement
#define ASSERT(x) if (not x) throw(std::exception())

// Main loop
int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "multitarget_tracker");
  ros::NodeHandle n;

  // Get initial, user-supplied parameters from YAML file
  n.getParam("mtt_node/x0", xInitial);
  ASSERT(xInitial.getType() == XmlRpc::XmlRpcValue::TypeArray);
  int NumGMs = xInitial.size();
  std::cout << "Initializing state with " << NumGMs << " Gaussian Mixtures \n";
  ROS_INFO("Initializing state with %i Gaussian Mixtures", NumGMs);

  // Initialize tracker object with size = number of state vectors
  GmPhdFilter gmPhd(4);

  // Initialize GM component for populating the initial state belief
  GaussianMixtureComponent gmComp;

  // Populate tracker object's initial estimate with user-supplied value of x0
  for (int ii=0; ii < xInitial.size() ; ii++) {
    // Store each GM component from file in a loop variable
    XmlRpc::XmlRpcValue gm = xInitial[ii];

    // Assign gaussian mixture values from input file to temporary variable gmComp
    gmComp.weight = (double)gm["weight"];
    for (int jj = 0; jj < gm["mu"].size(); jj++) {
      gmComp.state(jj) = gm["mu"][jj];
      gmComp.variance(jj) = gm["sigma"][jj];
    };

    std::cout << "Got GM component with weight = " << gmComp.weight << ", state = " << gmComp.state << ", and variance = " << gmComp.variance << "\n"; 

    // Add gmComp to the tracker's initial state belief vector
    gmPhd.belief.GaussianMixtures.push_back(gmComp);

  }

  // Initialize publisher and messages
  // ros::Publisher state_pub = n.advertise<ros_multitarget_tracking::GaussianMixture2D>("multitarget_state", 1000);
  // ros_multitarget_tracking::GaussianMixture2D BeliefMsg;
  // ros_multitarget_tracking::Gaussian2D GaussianMsg;

  ros::Rate loop_rate(10);

  // Main loop
  while (ros::ok())
  {

    // Populate state belief message with current Gaussians
    // BeliefMsg.models.clear();
  
    // for (auto model : Belief) {
    //   GaussianMsg.weight = model.weight;
    //   GaussianMsg.mu_x = model.muX;
    //   GaussianMsg.mu_y = model.muY;
    //   GaussianMsg.sigma_x = model.sigmaX;
    //   GaussianMsg.sigma_y = model.sigmaY;
    //   BeliefMsg.models.push_back(GaussianMsg);

    // } // for model : Belief

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    // state_pub.publish(BeliefMsg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}