//
// Created by steve on 18-1-27.
//

#ifndef COMPLEXITYPOSITIONING_LIGHTFILTER_H
#define COMPLEXITYPOSITIONING_LIGHTFILTER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace LightFilter {
    template<int StateNumber,
            int InputNumber,
            int MeasurementNumber,
            typename T>
    class LightFilter {
    public:
        using StateType =  Eigen::Matrix<T, StateNumber, 1>;
        using StateProbabilityType=Eigen::Matrix<T, StateNumber, StateNumber>;
        using InputType=Eigen::Matrix<T, InputNumber, 1>;
        using MeasurementType=Eigen::Matrix<T, MeasurementNumber, 1>;

        using StateTransMatrixType=Eigen::Matrix<T, StateNumber, StateNumber>;
        using InputGainMatrixType=Eigen::Matrix<T, StateNumber, InputNumber>;
        using OutputGainMatrixType=Eigen::Matrix<T, MeasurementNumber, StateNumber>;

        using ProcessNoiseMatrixType=Eigen::Matrix<T, StateNumber, StateNumber>;
        using MeasurementNoiseMatrixType=Eigen::Matrix<T, MeasurementNumber, MeasurementNumber>;
        using KMatrixType=Eigen::Matrix<T, StateNumber, MeasurementNumber>;

        /**
         * Initial Kalman Filter
         * @param process_noise_vec process noise vector
         * @param measurement_noise_vec measurement noise vector
         * @param initial_probability_vec state probability vector
         */
        LightFilter(const ProcessNoiseMatrixType &process_noise_vec,
                    const MeasurementNoiseMatrixType &measurement_noise_vec,
                    const StateProbabilityType &initial_probability_vec) :
                state_(StateType::Identity()),
                Q_(ProcessNoiseMatrixType::Identity() * process_noise_vec),
                R_(MeasurementNoiseMatrixType::Identity() * measurement_noise_vec),
                state_probability_(StateProbabilityType::Identity() * initial_probability_vec),
                dX_(StateType::Ones()) {


        }

//        LightFilter() {}


        /**
         *
         * @param input
         * @return
         */
        virtual bool StateTransaction(const InputType &input) {

            return true;
        }


        /**
         *  measurement state
         * @param m  measurement state.
         * @return
         */
        virtual bool MeasurementState(const MeasurementType &m) {
            return true;
        }


        /**
         * Setter and getter for A,B&C.
         */
        const StateTransMatrixType &getA_() const;

        void setA_(const StateTransMatrixType &A_);

        const InputGainMatrixType &getB_() const;

        void setB_(const InputGainMatrixType &B_);

        const OutputGainMatrixType &getH_() const;

        void setH_(const OutputGainMatrixType &H_);


    protected:
        /**
         * X_i=A*X_{i-1}+B*u_i+w_i
         * z_i=H*X_i+v_i
         * w_i \in Q
         * v_i \in R
         */
        StateTransMatrixType A_ = StateTransMatrixType::Identity();
        InputGainMatrixType B_ = InputGainMatrixType::Identity();
        OutputGainMatrixType H_ = OutputGainMatrixType::Identity();

        ProcessNoiseMatrixType Q_;
        MeasurementNoiseMatrixType R_;

        KMatrixType K_ = KMatrixType::Identity();

        StateType dX_;
        StateType state_;
        StateProbabilityType state_probability_;
        InputType input_;
        MeasurementType m_;

    };


    template<int MeasurementNum, typename T>
    class ImuEkf :
            public LightFilter<9, 6, MeasurementNum, T> {

    public:


        ImuEkf(const typename LightFilter<9, 6, MeasurementNum, T>::ProcessNoiseMatrixType
               &process_noise_vec,
               const typename LightFilter<9, 6, MeasurementNum, T>::MeasurementNoiseMatrixType
               &measurement_noise_vec,
               const typename LightFilter<9, 6, MeasurementNum, T>::StateProbabilityType
               &initial_probability_vec) :
                LightFilter<9, 6, MeasurementNum, T>(process_noise_vec, measurement_noise_vec,
                                                     initial_probability_vec) {
            this->dX_.setZero();
        }


        bool StateTransaction(const typename LightFilter<9, 6, MeasurementNum, T>::InputType &input) {
            return true;

        }

        bool MeasurementState(const typename LightFilter<9, 6, MeasurementNum, T>::MeasurementType &m) {
            return true;
        }

        /**
         *  initial function
         * @param u server times of measurements
         * @param pos3d pose(x,y,z) of initial state.
         * @param initial_heading
         * @return
         */
        bool Initialization(Eigen::MatrixXd u,
                            Eigen::Vector3d pos3d = Eigen::Vector3d(0, 0, 0),
                            double initial_heading = 0.0) {
            this->state_.block(0, 0, 3, 1) = pos3d;
            long double f_u(0.0), f_v(0.0), f_w(0.0);

            f_u = u.col(0).mean();
            f_v = u.col(1).mean();
            f_w = u.col(2).mean();

            double roll = std::atan(f_v / f_w);
            double pitch = -std::asin(f_u / std::sqrt(f_u * f_u + f_v * f_v + f_w * f_w));

            auto C = Ang2RotMatrix(pitch, roll, initial_heading);
            Eigen::Vector3d acc(f_u, f_v, f_w);

            std::cout << "acc src:" << acc.transpose() << std::endl;
            std::cout << "acc after:" << C * acc << std::endl;
            if (std::isnan(C.sum())) {
                std::cout << "error in initial navigation equation" << std::endl;
            }

        }


        double getTime_inteval_() const {
            return time_inteval_;
        }

        void setTime_inteval_(double time_inteval_) {
            ImuEkf::time_inteval_ = time_inteval_;
        }

        double getGravity_() const {
            return gravity_;
        }

        void setGravity_(double gravity_) {
            ImuEkf::gravity_ = gravity_;
        }

    protected:

        inline Eigen::Matrix3d Ang2RotMatrix(Eigen::Vector3d ang) {
            double cr(cos(ang(0)));
            double sr(sin(ang(0)));

            double cp(cos(ang(1)));
            double sp(sin(ang(1)));

            double cy(cos(ang(2)));
            double sy(sin(ang(2)));

            Eigen::Matrix3d R3;
            R3 << cy * cp, sy * cp, -sp,
                    -sy * cr + cy * sp * sr, cy * cr + sy * sp * sr, cp * sr,
                    sy * sr + cy * sp * cr, -cy * sr + sy * sp * cr, cp * cr;

            return R3;

        }

        inline Eigen::Matrix3d Ang2RotMatrix(double pitch, double roll, double yaw) {
            double cp = std::cos(pitch);
            double sp = std::sin(pitch);

            double cr = std::cos(roll);
            double sr = std::sin(roll);

            double cy = std::cos(yaw);
            double sy = std::sin(yaw);

            Eigen::Matrix3d C;
            C << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy + sr * sy),
                    cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
                    -sp, sr * cp, cr * cp;
            return C;
        }


        double time_inteval_ = 0.05;
        double gravity_ = 9.81;
//        Eigen::Quaterniond


    };
}


#endif //COMPLEXITYPOSITIONING_LIGHTFILTER_H
