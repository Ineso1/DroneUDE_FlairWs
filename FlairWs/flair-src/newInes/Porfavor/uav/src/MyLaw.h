#ifndef MYLAW_H
#define MYLAW_H

#include "DefineHandler.h"
#include <Matrix.h>
#include <Layout.h>
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

#include "StateSpace.h"

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
    class MyLaw : public ControlLaw, public Observer::StateSpace {
        public:

        string something2debug;

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
            Vector3DSpinBox *J_layout;
            
        // Layout UDE Gain
            Vector3DSpinBox *omega_gains_trans;

        // Layout Translational Control inputs
            Eigen::Vector3f kp_rot;
            Eigen::Vector3f kd_rot_1;
            Eigen::Vector3f kd_rot_2;
            float sat_rot;

        // Layout Rotational Control inputs
            Eigen::Vector3f kp_trans;
            Eigen::Vector3f kd_trans_1;
            Eigen::Vector3f kd_trans_2;
            float sat_trans;
            
        // Desire position and Control Inputs

            Eigen::Quaternionf eq;
            Eigen::Vector3f p_d;
            float Fu;
            Eigen::Vector3f Tauu;
            
        // SM conditions
            float previous_time;
            bool first_update;
            float timeT;
            bool first=true;

        // CSV vars
            std::ofstream controlOutputFile;
            std::ofstream debugOutputFile;

            std::string controlFilePath;
            std::string debugFilePath;
        
        // Description Matrix
            Matrix *stateM;
            
        // My methods
            ~MyLaw(void);
            MyLaw(const LayoutPosition* position,std::string name);
            void UpdateFrom(const io_data *data);
            void UseDefaultPlot(const LayoutPosition* position);
            void Reset(void);

            void CalculateControl(const Eigen::MatrixXf&, Eigen::MatrixXf&, float);
            Eigen::Quaternionf ExpQuaternion(const Eigen::Quaternionf&);
            Eigen::Vector3f RotVec(const Eigen::Quaternionf&);
            Eigen::Quaternionf LogQuaternion(const Eigen::Quaternionf&);
            void SaveControlDataToCSV();
            void SaveAllDataToCSV(float, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Quaternionf&, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f&);
            void SetTarget(Vector3Df, Vector3Df);
            void UpdateDynamics(Vector3Df p, Vector3Df pd, Quaternion q,Vector3Df w);

        private:
    };
}
}

#endif //MYLAW_H
































