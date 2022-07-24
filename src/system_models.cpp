#include <stdexcept>
#include <Eigen/Dense>

using Eigen::MatrixXd;

class LinearMotionModel2D {

    public:
        LinearMotionModel2D(double& dt) : ProcessVariance_(0.25) {SetProcessCovariance(dt); SetProcessTransition(dt);}
        LinearMotionModel2D(double& dt, double& process_variance) {
            SetProcessVariance(process_variance);
            SetProcessCovariance(dt); 
            SetProcessTransition(dt);
        }

        void SetProcessTransition(double& dt){
            if (dt > 0) {
                ProcessTransition_ = MatrixXd::Ones(4,4);
                ProcessTransition_(0,2) = dt;
                ProcessTransition_(1,3) = dt;
            } else {
                throw std::invalid_argument("Timestep must be positive");
            }
        }

        void SetProcessCovariance(double& dt){
            if (dt > 0) {
                ProcessCovariance_ << 0.25*pow(dt,4),   0,              0.50*pow(dt,2), 0,
                                      0,                0.25*pow(dt,4), 0,              0.50*pow(dt,2),
                                      0.50*pow(dt,2),   0,              1,              0,
                                      0,                0.50*pow(dt,2), 0,              1;
                ProcessCovariance_ *= pow(ProcessVariance_,2);
            } else {
                throw std::invalid_argument("Timestep must be positive");
            }
        }

        void SetProcessVariance(double& sigma_proc){
            if (sigma_proc > 0) {
                ProcessVariance_ = sigma_proc;
            } else {
                throw std::invalid_argument("Process variance must be positive");
            }
        }

        MatrixXd* GetProcessTransition() {return &ProcessTransition_;}
        MatrixXd* GetProcessCovariance() {return &ProcessCovariance_;}

    private:
        MatrixXd ProcessTransition_= MatrixXd::Ones(4,4);;
        MatrixXd ProcessCovariance_= MatrixXd::Ones(4,4);;
        double ProcessVariance_;

}; // LinearSystemsModel