// DroneBase.h
#ifndef DRONEBASE_H
#define DRONEBASE_H 

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
#include <Vector3DSpinBox.h>
#include <DoubleSpinBox.h>
#include <iostream>

using namespace std;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;
using namespace flair::core;

class DroneBase : public UavStateMachine {
public:
    Uav* uav;
    bool vrpnLost;
    DroneBase(TargetController *controller);
    virtual ~DroneBase();

protected:
    enum class BehaviourMode_t { Default, PositionHold, Trajectory };
    BehaviourMode_t behaviourMode;

    // UI Buttons
    Tab *customLawTab;
    GridLayout* execLayout;
    PushButton *startTrajectory;
    PushButton *stopTrajectory; 
    PushButton *positionHold;
    PushButton *positionChange;
    PushButton *togglePerturbation;
    Vector3DSpinBox *targetPosition_layout;
    DoubleSpinBox *yawAngle_layout;
    Vector3DSpinBox *rejectionPercent_layout;
    PushButton *rejectPerturbation;
    Vector3DSpinBox *perturbation_layout;
    MetaVrpnObject *uavVrpn;

    // Methods
    void StopTrajectory(void);
    void StartTrajectory(void);
    void PositionHold(void);

    virtual void HandleDisturbanceToggle(void);
    virtual void ApplyControl(void);
    virtual void PositionChange(void);
    virtual void RejectDisturbance(void);

    // State Machine Functions
    void SignalEvent(Event_t event) override;
    void ExtraSecurityCheck(void) override;
    void ExtraCheckPushButton(void) override;
    void ExtraCheckJoystick(void) override;
};

#endif // DRONEBASE_H