#ifndef MYLAW_H
#define MYLAW_H

#include "../ParamSim.h"
#include <Matrix.h>
#include <Layout.h>
#include <Tab.h>
#include <GridLayout.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <Vector3D.h>
#include <Vector3DSpinBox.h>
#include <SpinBox.h>
#include <GroupBox.h>
#include <Quaternion.h>
#include <Euler.h>
#include <ControlLaw.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include "../Observer/KFC/KFC.h"
#include "../Observer/UDE/UDE.h"
#include "../Observer/Luenberger/Luenberger.h"
#include "../Observer/SlidingMode/SlidingMode.h"
#include "../Observer/SuperTwist/SuperTwist.h"


using namespace std;
using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
    namespace gui {
        class LayoutPosition;
    }
}

namespace flair {
namespace filter {
    class MyLaw : public ControlLaw,
    public Observer::ObserverBase
    {
        public:

        enum class ObserverMode_t { UDE, Luenberger, SuperTwist, SlidingMode };
        ObserverMode_t observerMode;
        Observer::KFC kalmanFilter;

        Observer::UDE ude;
        Observer::Luenberger luenberger;
        Observer::SlidingMode slidingMode;
        Observer::SuperTwist superTwist;


        string something2stream;
        bool isDisturbanceActive; // Flag for disturbance activation
        bool isKalmanActive;
        float activation_delay;   // Delay time for disturbance activation 
        float this_time;


    ///////////////////////////
    // LAYOUT GAINS AND PARAMS
    ///////////////////////////

        // Layout Translational Control inputs
            Vector3DSpinBox *kp_rot_layout;
            Vector3DSpinBox *kd_rot_1_layout;
            Vector3DSpinBox *kd_rot_2_layout;
            DoubleSpinBox *sat_rot_layout;

        // Layout Rotational Control inputs
            Vector3DSpinBox *kp_trans_layout;
            Vector3DSpinBox *kd_trans_1_layout;
            Vector3DSpinBox *kd_trans_2_layout;
			DoubleSpinBox *sat_trans_layout;

        // Layout drone properties
            DoubleSpinBox *mass_layout;
            
        // Layout UDE Gain
            Vector3DSpinBox *omega_gains_trans;
            Vector3DSpinBox *omega_gains_rot;

        // Motor constant
            DoubleSpinBox *motorConst;

    ///////////////////////////
    // GAINS AND PARAMS
    ///////////////////////////

        // Translational Control inputs
            Eigen::Vector3f kp_rot;
            Eigen::Vector3f kd_rot_1;
            Eigen::Vector3f kd_rot_2;
            float sat_rot;

        // Rotational Control inputs
            Eigen::Vector3f kp_trans;
            Eigen::Vector3f kd_trans_1;
            Eigen::Vector3f kd_trans_2;
            float sat_trans;
            
    ///////////////////////////
    // TARGET VARS
    ///////////////////////////

            Eigen::Quaternionf eq;  // Error quaternion
            Eigen::Quaternionf qz;  // z rotation quaternion
            Eigen::Vector3f p_d;    // Desire position

    ///////////////////////////
    // CONTROL INPUTS
    ///////////////////////////

            float Fu;
            Eigen::Vector3f Tauu;
            float motorK;

    ///////////////////////////
    // PERTURBATIONS
    ///////////////////////////

        // Perturbations
            Eigen::Vector3f perturbation_trans;
            Eigen::Vector3f perturbation_rot;

            Eigen::Vector3f rejectionPercent;
            
        // Custom time conditions
            bool firstUpdate;

    ///////////////////////////
    // TIME
    ///////////////////////////
            
            std::chrono::high_resolution_clock::time_point previous_chrono_time;

    ///////////////////////////
    // CSV HANDLERS
    ///////////////////////////

            std::ofstream controlInputFileCSV;
            std::ofstream controlDebugFileCSV;
            std::ofstream translationOutputFileCSV;
            std::ofstream rotationOutputFileCSV;

            std::string controlFilePath;
            std::string debugFilePath;
            std::string translationFilePath;     
            std::string rotationFilePath;     
        
    ///////////////////////////
    // DYNAMIC VARIABLES MATRIX
    ///////////////////////////

            Matrix *stateM, *dataexp; // Description Matrix

            
    ///////////////////////////
    // CONSTRUCTOR
    ///////////////////////////

            ~MyLaw(void);
            MyLaw(const LayoutPosition* position,std::string name);
            
    ///////////////////////////
    // FLAIR STATE MACHINE CONTROL HANDLERS
    ///////////////////////////
    
            void UpdateFrom(const io_data *data);
            void UseDefaultPlot(const LayoutPosition* position);

    ///////////////////////////
    // SETTERS
    ///////////////////////////

            void SetTarget(Vector3Df, Vector3Df, Quaternion);
            void SetPerturbation(Vector3Df, Vector3Df);
            void SetRejectionPercent(Vector3Df);

    ///////////////////////////
    // MA CONTROL ALGO
    /////////////////////////// 
            
            void CalculateControl(const Eigen::MatrixXf&, Eigen::MatrixXf&, float);
            
    ///////////////////////////
    // UPDATE DYNAMIC VARS
    /////////////////////////// 
            
            void UpdateDynamics(Vector3Df p, Vector3Df pd, Quaternion q,Vector3Df w);
            void Reset(void); // No esta bien implementada aun
            
    ///////////////////////////
    // CSV FUNcS
    /////////////////////////// 

            void SaveStateCSV(Eigen::Vector3f &p, Eigen::Vector3f &dp,Eigen::Vector3f &ddp, Eigen::Vector3f &domega, Eigen::Vector3f &omega, Eigen::Quaternionf &dq, Eigen::Quaternionf &q, float &dt);
            void SaveControlCSV();
            void SaveDebugCSV(float, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Quaternionf&, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f&);
            
    ///////////////////////////
    // QUATERNION FUNCTIONS
    ///////////////////////////

            Eigen::Quaternionf ExpQuaternion(const Eigen::Quaternionf&);
            Eigen::Vector3f RotVec(const Eigen::Quaternionf&);
            Eigen::Quaternionf LogQuaternion(const Eigen::Quaternionf&);

    };
}
}

#endif //MYLAW_H
































