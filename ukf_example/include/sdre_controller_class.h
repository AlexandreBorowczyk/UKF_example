#ifndef SDRECONTROLLER_H_
#define SDRECONTROLLER_H_

#include <eigen3/Eigen/Dense>

class SdreController {
    public:
        explicit SdreController(
                const double & cart_mass,
                const double & pendulum1_mass,
                const double & pendulum2_mass,
                const double & pendulum1_length_,
                const double & pendulum2_length_);

        ~SdreController();

        double ComputeCommand(
                const Eigen::Matrix<double,6,1> & X,
                const double & iteration_time);



    private:

        void ComputeCoef();

        // System physical param.
        double cart_mass_;
        double pendulum1_mass_;
        double pendulum2_mass_;
        double pendulum1_length_;
        double pendulum2_length_;

        // Computed coef.
        Eigen::Matrix<double,6,6> d_;
        Eigen::Matrix<double,1,3> f_;


        // Controller parameter
        Eigen::Matrix<double,6,6> Q_;
        double R_;

    public : EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif  // SDRECONTROLLER_H_
