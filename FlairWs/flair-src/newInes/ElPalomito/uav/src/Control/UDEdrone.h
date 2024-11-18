#ifndef UDEDRONE_H
#define UDEDRONE_H

#include "../ParamSim.h"
#include <UavStateMachine.h>
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <PidThrust.h>
#include "MyLaw.h"
#include <iostream>

using namespace std;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;



#ifdef PARAMSIM_H
    const float POSITION_THRESHOLD = POSITION_THRESHOLD_AUTOMATIC_MODE;
    enum CustomTargets {
        Points3Target,
        Points5Target
    };
    /*
    flair::core::Vector3Df Points3_arr[] = {
        flair::core::Vector3Df(0, 0, 2.5),
        flair::core::Vector3Df(1.5, 0, 2.5),
        flair::core::Vector3Df(1.5, -1.5, 2.5),
    };

    flair::core::Vector3Df Points5_arr[] = {
        flair::core::Vector3Df(-2, 2, 2.5),
        flair::core::Vector3Df(0, 0, 3),
        flair::core::Vector3Df(-2, -2, 2.5),
        flair::core::Vector3Df(2, -2, 2.5),
        flair::core::Vector3Df(2, 2, 2.5)
    };
    */
#else
    const float POSITION_THRESHOLD = 0.1;
#endif

class UDEdrone : public UavStateMachine {
    public:
        UDEdrone(TargetController *controller);
        Uav* uav;

    ///////////////////////////
    // Perturbation Toogle    
    ///////////////////////////
        bool perturbation = false;

    ///////////////////////////
    // Target
    ///////////////////////////
        int currentTargetIndex = 0;
        flair::core::Vector3Df currentTarget;
        flair::core::Vector3Df targetPositions[3] = {
            flair::core::Vector3Df(0, 0, 2.5),
            flair::core::Vector3Df(1.5, 0, 2.5),
            flair::core::Vector3Df(1.5, -1.5, 2.5),
        };
        flair::core::Vector3Df posHold;
        float yawHold;

    ///////////////////////////
    // Feedback objects
    ///////////////////////////
        flair::core::Vector3Df vrpnPosition;
    	flair::core::Quaternion vrpnQuaternion;
        bool vrpnLost;

        ~UDEdrone();

    private:

	    enum class BehaviourMode_t { Default, PositionHold, Trajectory };
        BehaviourMode_t behaviourMode;

    protected:

    ///////////////////////////
    // UI Buttons function handler
    ///////////////////////////
        void StartTrajectory(void);     // Start Button
        void StopTrajectory(void);      // Stop Button  
        void PositionHold(void);        // Position Hold button, not implemented 
        void PositionChange(void);      // Toggle target position button              

    ///////////////////////////
    // State Machine funcs (dont move for the moment :v )
    ///////////////////////////
        void SignalEvent(Event_t event) override;   // Sets Behaviour Mode
        void ExtraSecurityCheck(void) override;     // State Machine vrpn objects check
        void ExtraCheckPushButton(void) override;   // State Machine buttons check
        void ExtraCheckJoystick(void) override;     // State Machine Joystick check
        
    ///////////////////////////
    // My control handlers
    ///////////////////////////
        void ApplyControl(void);                                            // Control Calcs 
        float ComputeCustomThrust(void) override;                           // Set custom thrust
        void ComputeCustomTorques(flair::core::Euler &torques) override;    // Set custom torques
        void CoordFrameCorrection(Vector3Df&, Vector3Df&, Vector3Df&,Vector3Df&);   // Frame correction
        
        
        MyLaw * myLaw;
        PushButton *startTrajectory;
        PushButton *stopTrajectory; 
        PushButton *positionHold;
        PushButton *positionChange;
        PushButton *togglePerturbation;
        MetaVrpnObject *refVrpn;
        MetaVrpnObject *uavVrpn;
        flair::core::AhrsData *customReferenceOrientation;
        flair::core::AhrsData *customOrientation;
};

#endif // UDEDRONE




