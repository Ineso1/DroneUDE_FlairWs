// Drone.h
#ifndef DRONE_H
#define DRONE_H

#include "DroneBase.h"
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

class Drone : public DroneBase {
public:
    Drone(TargetController *controller);
    virtual ~Drone();

protected:
    // Perturbation Toggle
    enum class ObserverMode_t { UDE, Luenberger, SuperTwist, SlidingMode };
    enum class AlgorithmBehaviourMode_t {PositionPoint, TrajectoryFollow};

    bool perturbation;
    bool kalman;

    // Feedback objects
    flair::core::Vector3Df vrpnPosition;
    flair::core::Quaternion vrpnQuaternion;
    flair::core::Vector3Df currentTarget;
    float yawAngle;
    Quaternion initQuaternion;

    // Control Law
    MyLaw *myLaw;

    // Control functions
    void HandleDisturbanceToggle(void) override;
    void ApplyControl(void) override;
    void PositionChange() override;
    void RejectDisturbance(void) override;
    void ApplyKalman(void) override;

    float ComputeCustomThrust() override;
    void ComputeCustomTorques(flair::core::Euler &torques) override;
    void CoordFrameCorrection(Vector3Df &uav_p, Vector3Df &uav_dp, Vector3Df &w, Vector3Df &aim_p);
    void MixOrientation();

private:
    Quaternion mixQuaternion;
    Vector3Df mixAngSpeed;
    Quaternion qI;
    bool first_up = true;
};

#endif // DRONE_H
