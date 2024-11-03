#include "UDEdrone.h"

using namespace flair::core;

UDEdrone::UDEdrone(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    uav = GetUav();
    Fu = 0;
    time = 0;
    cont = 0;
    myLaw = new MyLaw(setupLawTab->At(1,0),"MyLaw");

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
        refVrpn=new MetaVrpnObject("target",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        refVrpn = new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        refVrpn=new MetaVrpnObject("target");
    }
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    getFrameworkManager()->AddDeviceToLog(refVrpn);
    getFrameworkManager()->AddDeviceToLog(myLaw);
    vrpnclient->Start();

    startTrajectory = new PushButton(GetButtonsLayout()->NewRow(),"Start con tokyo");
    stopTrajectory = new PushButton(GetButtonsLayout()->LastRowLastCol(),"Stop calmalas aguas");
    positionHold = new PushButton(GetButtonsLayout()->LastRowLastCol(),"position estate quieto");
    positionChange = new PushButton(GetButtonsLayout()->LastRowLastCol(),"otra position");
    
    customReferenceOrientation = new AhrsData(this,"reference");
    AddDataToControlLawLog(customReferenceOrientation);
	
    customOrientation = new AhrsData(this,"orientation");

    currentTarget = Vector3Df(1.5,1.5,2);
}

UDEdrone::~UDEdrone() {
}

void UDEdrone::CoordFrameCorrection(Vector3Df &uav_p, Vector3Df &uav_dp, Vector3Df &w, Vector3Df &aim_p){
    // looks like the body frame is rotated somehow XD and the frame is rotated in x axis upside down
    Vector3Df correction_uav_p(uav_p.y, -uav_p.x,-uav_p.z);
    Vector3Df correction_aim_p(aim_p.y, -aim_p.x, aim_p.z);
    uav_p = correction_uav_p;
    aim_p = correction_aim_p;
    w.x *= -1;  // so we can handle positive rot_kd
    uav_dp.z *= -1;
}

void UDEdrone::ApplyControl(){
    Vector3Df ref_p;
    Vector3Df uav_p,uav_dp; 
    Vector3Df aim_p, aim_dp;
    Quaternion q = GetCurrentQuaternion();
    Vector3Df w;
    float yaw_ref;

    aim_p = currentTarget;
    aim_dp = Vector3Df(0,0,0);
    yawHold = 0;

    refVrpn->GetPosition(ref_p);
    uavVrpn->GetPosition(uav_p);
    uavVrpn->GetSpeed(uav_dp);
    GetDefaultOrientation()->GetQuaternionAndAngularRates(q, w);
    
    //Get position respect 2 the reference ninja
    uav_p = uav_p - ref_p;
    CoordFrameCorrection(uav_p, uav_dp, w, aim_p);

    #ifdef PosLog
        std::ostringstream logStream;
        logStream   << "\n" << "uav x: " << uav_p.x << "\n"
                            << "uav y: " << uav_p.y << "\n"
                            << "uav z: " << uav_p.z << "\n"
                    << "\n" << "tar x: " << aim_p.x << "\n"
                            << "tar y: " << aim_p.y << "\n"
                            << "tar z: " << aim_p.z << "\n"
                    << "\n" << "q0   : " << q.q0 << "\n"
                            << "q1   : " << q.q1 << "\n"
                            << "q2   : " << q.q2 << "\n"
                            << "q3   : " << q.q3 << "\n"
                    << "\n" << "w x: " << w.x << "\n"
                            << "w y: " << w.y << "\n"
                            << "w z: " << w.z << "\n";
        Thread::Info(logStream.str().c_str());
    #endif

    myLaw->SetTarget(aim_p, aim_dp);
    myLaw->UpdateDynamics(uav_p, uav_dp, q, w);
    myLaw->Update(GetTime());

    #ifdef AutomaticSwitch
        // Check if the drone is within the threshold distance of the target
        float distanceToTarget = std::abs(uav_p.GetNorm() - aim_p.GetNorm());
        if (distanceToTarget < POSITION_THRESHOLD) {
            PositionChange(); // Automatically toggle to the next position
        }
    #endif
}

void UDEdrone::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::EnteringControlLoop:
        behaviourMode=BehaviourMode_t::Trajectory;
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void UDEdrone::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Trajectory) || (behaviourMode==BehaviourMode_t::PositionHold))) {
        if (!refVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void UDEdrone::ExtraCheckPushButton(void) {
    if(startTrajectory->Clicked()) {
        StartTrajectory();
        Thread::Info("\n\n\nUDEdrone: Con Tokyo Honda y Kawasaky \n\n\n\n");
    }
    if(stopTrajectory->Clicked()) {
        StopTrajectory();
        Thread::Info("\n\n\nUDEdrone: Calama las aguas \n\n\n\n");
    }
    if(positionHold->Clicked()) {
        PositionHold();
        Thread::Info("\n\n\nUDEdrone: Estate ahi \n\n\n\n");
    }
    if(positionChange->Clicked()) {
        cont = 1;
        Thread::Info("\n\n\nApoco si jalo?? JAJAJ \n\n\n\n");
        PositionChange();
    }
}

void UDEdrone::ExtraCheckJoystick(void) {
    //R1 and Circle
    if(GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
        StartTrajectory();
    }

    //R1 and Cross
    if(GetTargetController()->ButtonClicked(5) && GetTargetController()->IsButtonPressed(9)) {
        StopTrajectory();
    }
    
    //R1 and Square
    if(GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
        PositionHold();
    }
}

void UDEdrone::ComputeCustomTorques(Euler &torques) {
    ApplyControl();
    float roll = myLaw->Output(0);
    float pitch = myLaw->Output(1);
    float yaw = myLaw->Output(2);
    //Just 2 be sure nothing happen here
    torques.pitch = std::isnan(pitch) ? 0.0f : -pitch/10;
    torques.roll = std::isnan(roll) ? 0.0f : roll/10;
    torques.yaw = std::isnan(yaw) ? 0.0f : -yaw/10;
}

float UDEdrone::ComputeCustomThrust() {
    
    #ifdef MyLawDebugLog
    string error = myLaw->something2debug;
    std::ostringstream logStream;
    logStream << error << "\n";
    Thread::Info(logStream.str().c_str());
    #endif
    
    float thrust = myLaw->Output(3);

    if (std::isnan(thrust)) {
        thrust = 0.0f;
    }
    // inverted frame :v aero standard or something like that
    return -thrust/10;
}

void UDEdrone::StartTrajectory(void) {
    if( behaviourMode == BehaviourMode_t::Trajectory) {
        Thread::Warn("UDEdrone: already in this mode\n");
    }
    if (SetOrientationMode(OrientationMode_t::Manual)) {
        Thread::Info("esta vaina: start \n");
    } else {
        Thread::Warn("esta vaina: could not start otra vez \n");
        return;
    }

    if (SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Info("UDEdrone: Custom Thrust Mode Set\n");
    } else {
        Thread::Warn("UDEdrone: Failed to set Custom Thrust Mode\n");
    }

    if (SetTorqueMode(TorqueMode_t::Custom)) {
        Thread::Info("UDEdrone: Custom Torque Mode Set\n");
    } else {
        Thread::Warn("UDEdrone: Failed to set Custom Torque Mode\n");
    }

    cont=1;
    time=0;
    ApplyControl();
}

void UDEdrone::StopTrajectory(void) {
    if( behaviourMode!=BehaviourMode_t::Trajectory) {
        Thread::Warn("UDEdrone: not in Trajectory mode\n");
        return;
    }
    Thread::Info("UDEdrone: StopTrajectory\n");
}

void UDEdrone::PositionHold(void) {
    if( behaviourMode==BehaviourMode_t::PositionHold) {
        Thread::Warn("UDEdrone: already in vrpn position hold mode\n");
        return;
    }
	uavVrpn->GetQuaternion(vrpnQuaternion);
    yawHold=vrpnQuaternion.ToEuler().yaw;
    uavVrpn->GetPosition(vrpnPosition);
    posHold = vrpnPosition;
    myLaw->Reset();
    behaviourMode=BehaviourMode_t::PositionHold;
    SetOrientationMode(OrientationMode_t::Custom);
    SetThrustMode(ThrustMode_t::Custom);
    SetTorqueMode(TorqueMode_t::Custom);
    Thread::Info("UDEdrone: holding position\n");
}

void UDEdrone::PositionChange() {
    const int numTargets = sizeof(targetPositions) / sizeof(targetPositions[0]);
    currentTargetIndex = (currentTargetIndex + 1) % numTargets;
    Thread::Info("\nPosition changed to next point in matrix.\n");
    currentTarget = targetPositions[currentTargetIndex];
}
