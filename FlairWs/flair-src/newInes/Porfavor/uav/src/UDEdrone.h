#ifndef UDEDRONE_H
#define UDEDRONE_H

#include "DefineHandler.h"
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

const float POSITION_THRESHOLD = 0.1; // Threshold for position change

using namespace std;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

typedef struct{
    flair::core::Vector3Df error;
    flair::core::Vector3Df integralError;
    flair::core::Vector3Df derivativeError;
} uavPosError_t;

typedef enum{
    Default,
    PositionHold,
    Trajectory
} BehaviourMode_t;

class UDEdrone : public UavStateMachine {
    public:
        UDEdrone(TargetController *controller);
        Uav* uav;
        float Fu;
        flair::core::Vector3Df Tauu;

        float time = 0;
        float cont = 0;
        int currentTargetIndex = 0;

        flair::core::Vector3Df currentTarget;

        flair::core::Vector3Df vrpnPosition;
    	flair::core::Quaternion vrpnQuaternion;

        flair::core::Vector3Df targetPositions[5] = {
            flair::core::Vector3Df(-2, 2, 2.5),
            flair::core::Vector3Df(0, 0, 3),
            flair::core::Vector3Df(-2, -2, 2.5),
            flair::core::Vector3Df(2, -2, 2.5),
            flair::core::Vector3Df(2, 2, 2.5)
        };
        ~UDEdrone();

    private:

	    enum class BehaviourMode_t {
            Default,
            PositionHold,
            Trajectory
        };

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        float yawHold;
        flair::core::Vector3Df posHold;

    protected:

        void StartTrajectory(void); //Aqui bien
        void StopTrajectory(void); //Aqui bien
        void PositionHold(void); //Aqui bien
        void SignalEvent(Event_t event) override; //Aqui bien

        void ExtraSecurityCheck(void) override; //Aqui bien
        void ExtraCheckPushButton(void) override; //Aqui bien
        void ExtraCheckJoystick(void) override; //Aqui bien
        void ApplyControl(void);
        float ComputeCustomThrust(void) override; // No se esta override checa virtual
        void ComputeCustomTorques(flair::core::Euler &torques) override;
        
        void CoordFrameCorrection(Vector3Df&, Vector3Df&, Vector3Df&,Vector3Df&);
        void PositionChange();
        
        MyLaw * myLaw;
        PushButton *startTrajectory;
        PushButton *stopTrajectory; 
        PushButton *positionHold;
        PushButton *positionChange;
        MetaVrpnObject *refVrpn;
        MetaVrpnObject *uavVrpn;
        flair::core::AhrsData *customReferenceOrientation;
        flair::core::AhrsData *customOrientation;
};

#endif // UDEDRONE




