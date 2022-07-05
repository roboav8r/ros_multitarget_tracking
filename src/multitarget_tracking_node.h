#ifndef MULTITARGET_TRACKING_NODE_H_
#define MULTITARGET_TRACKING_NODE_H_

#include<vector>

// 2D Gaussian Mixture component consisting of weight and first moments mu & sigma
struct GaussianMixtureComponent2D {
    float weight; // Weight of component
    float muX; // Mean of GM in x-direction
    float muY; // Mean of GM in y-direction
    float sigmaX; // Variance of GM in x-direction
    float sigmaY; // Variance of GM in y-direction
};

// A vector of weighted Gaussian components forms a mixture/belief of the current state
std::vector<GaussianMixtureComponent2D> Belief;

#endif  // MULTITARGET_TRACKING_NODE_H_