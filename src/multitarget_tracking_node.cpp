#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "multitarget_tracking_node.h"
#include "ros_multitarget_tracking/GaussianModel.h"
#include "ros_multitarget_tracking/GaussianMixture.h"

// Debug independent assert statement
#define ASSERT(x) if (not x) throw(std::exception())

// Define helper methods
void PublishState (ros::Publisher& statePub, ros_multitarget_tracking::GaussianMixture& belMsg, ros_multitarget_tracking::GaussianModel& gaussMsg, GmPhdFilter& tracker)
{
  // Populate belief message header
  belMsg.header.stamp = tracker.belief.Timestamp;

  // Clear, then populate state belief message with current Gaussians
  belMsg.models.clear();

  for (auto& model : tracker.belief.GaussianMixtures) {
    gaussMsg.weight = model.weight;
    gaussMsg.mu.resize(model.state.size());
    gaussMsg.sigma.resize(model.state.size());
    for (int ii=0; ii< model.state.size(); ii++) {
      gaussMsg.mu[ii] = model.state(ii);
      gaussMsg.sigma[ii] = model.variance(ii);
    };

    // Add gaussian mixture message to belief message
    belMsg.models.push_back(gaussMsg);

  } // for model : Belief

  // Publish current state estimate to ROS
  statePub.publish(belMsg);
};

void GmPhdFilter::PredictExistingTargets(){
  // Get current time and compute time since last state estimate
  ros::Time predictTime = ros::Time::now();
  ros::Duration deltaT = predictTime - this->belief.Timestamp;

  // Debug information
  ROS_DEBUG("Prediction time interval was %f seconds", deltaT.toSec());

  // Update state estimate time
  this->belief.Timestamp = predictTime;

  // Propagate state forward
};


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
      gmComp.state(jj) = (double)gm["mu"][jj];
      gmComp.variance(jj) = (double)gm["sigma"][jj];
    };

    std::cout << "Got GM component with weight = " << gmComp.weight << ", state = " << gmComp.state << ", and variance = " << gmComp.variance << "\n"; 

    // Add gmComp to the tracker's initial state belief vector
    gmPhd.belief.GaussianMixtures.push_back(gmComp);

  }
  
  // Add timestamp to current belief
  gmPhd.belief.Timestamp = ros::Time::now();


  // Initialize publisher and messages
  ros::Publisher state_pub = n.advertise<ros_multitarget_tracking::GaussianMixture>("multitarget_state", 1000);
  ros_multitarget_tracking::GaussianMixture beliefMsg;
  ros_multitarget_tracking::GaussianModel gmMsg;


  ros::Rate loop_rate(10);

  // Main loop
  while (ros::ok())
  {

    // Predict/propagate last state forward in time
    gmPhd.PredictExistingTargets();

    // Publish current state
    PublishState (state_pub, beliefMsg, gmMsg, gmPhd);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}