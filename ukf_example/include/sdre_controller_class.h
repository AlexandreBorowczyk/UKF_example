#ifndef SDRECONTROLLER_H_
#define SDRECONTROLLER_H_

#include <eigen3/Eigen/Dense>

class SdreController {
    public:
        explicit SdreController(
                const double & cart_mass,
                const double & pendulum1_mass,
                const double & pendulum2_mass,
                const double & pendulum1_length,
                const double & pendulum2_length);

        ~SdreController();

        double ComputeCommand(
                const Eigen::Matrix<double,6,1> & X,
                const double & current_time);



    private:

        void ComputeCoef();
        Eigen::Matrix<double,6,6> SolveDare(
                const Eigen::Matrix<double,6,6> & Phi,
                const Eigen::Matrix<double,6,1> & Gamma,
                const Eigen::Matrix<double,6,6> & Q,
                const double & R);


        // Computed coef.
        Eigen::Matrix<double,3,3> d_;
        Eigen::Matrix<double,1,3> f_;


        // Controller parameter
        Eigen::Matrix<double,6,6> Q_;
        double R_;

    public : EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif  // SDRECONTROLLER_H_
