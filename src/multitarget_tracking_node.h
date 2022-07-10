#ifndef MULTITARGET_TRACKING_NODE_H_
#define MULTITARGET_TRACKING_NODE_H_

#include<vector>

#include<Eigen/Dense>

#include <XmlRpcValue.h>


using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// n-Dimensional Gaussian Mixture component consisting of weight and first moments mu & sigma
struct GaussianMixtureComponent {
    float weight; // Weight of component
    VectorXd state = VectorXd(4); // n-dimensional state vector
    VectorXd variance = VectorXd(4); // n-dimensional variance vector
};

// A timestamped vector of weighted Gaussian Mixture components forms a mixture/belief of the current state
struct StateEstimate {

    std::vector<GaussianMixtureComponent> GaussianMixtures;
};


// Create class for tracker objects
class GmPhdFilter {

public:
    // Constructor
    GmPhdFilter(int numStateVars) 
        : nStateVars(numStateVars), 
        processTransition{MatrixXd::Zero(numStateVars,numStateVars)}, 
        processCovariance{MatrixXd::Zero(numStateVars,numStateVars)}
        //belief(numStateVars)
    {
        std::cout << "Created GM-PHD filter for system with " << this->nStateVars << " state variables \n";
        ROS_INFO("Created GM-PHD filter for system with %i state variables", this->nStateVars);
    };


    int nStateVars{4}; //Number of dimensions in state vector
    
    StateEstimate belief; // Current state estimate/Belief

private:
    MatrixXd processTransition{4,4};
    MatrixXd processCovariance{4,4};
};

// Initialize variable to store ROS parameter values
XmlRpc::XmlRpcValue xInitial;

#endif  // MULTITARGET_TRACKING_NODE_H_