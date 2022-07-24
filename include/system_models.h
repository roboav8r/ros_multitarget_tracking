#ifndef SYSTEM_MODELS_H_
#define SYSTEM_MODELS_H_

#include <Eigen/Dense>

using Eigen::MatrixXd;

class LinearMotionModel2D {

    public:
        LinearMotionModel2D(double& dt);
        LinearMotionModel2D(double& dt, double& process_variance);

        void SetProcessTransition(double& dt);
        void SetProcessCovariance(double& dt);
        void SetProcessVariance(double& sigma_proc);

        MatrixXd* GetProcessTransition();
        MatrixXd* GetProcessCovariance();

    private:
        MatrixXd ProcessTransition_{4,4};
        MatrixXd ProcessCovariance_{4,4};
        double ProcessVariance_;

}; // LinearSystemsModel

#endif  // SYSTEM_MODELS_H_