#ifndef SDRECONTROLLER_H_
#define SDRECONTROLLER_H_

#include <eigen3/Eigen/Dense>

class SdreController {
    public:
        explicit SdreController();
        virtual ~SdreController();

        void OnInit() ;

        void OnUpdate();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        private:


};


#endif  // SDRECONTROLLER_H_
