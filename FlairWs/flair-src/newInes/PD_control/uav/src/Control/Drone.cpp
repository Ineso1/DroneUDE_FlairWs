// Drone.cpp
#include "Drone.h"

Drone::Drone(TargetController *controller) : DroneBase(controller) {
    perturbation = false;
    myLaw = new MyLaw(execLayout->At(1, 0), "MyLaw");
    getFrameworkManager()->AddDeviceToLog(myLaw);

    initQuaternion =  GetCurrentQuaternion();

    // Set current position target
    flair::core::Vector3Df uav_p;
    uavVrpn->GetPosition(uav_p);
    currentTarget = Vector3Df(uav_p.x,uav_p.y,1);
}

Drone::~Drone() {
    delete myLaw;
}

void Drone::PositionChange(void) {
    currentTarget = flair::core::Vector3Df(targetPosition_layout->Value().x, targetPosition_layout->Value().y, targetPosition_layout->Value().z);
}

void Drone::HandleDisturbanceToggle() {
    perturbation = !perturbation;
    if (perturbation) {
        Vector3Df perturbationVec = flair::core::Vector3Df(perturbation_layout->Value().x, perturbation_layout->Value().y, perturbation_layout->Value().z);
        myLaw->SetPerturbation(perturbationVec, Vector3Df(0, 0, 0));
    } else {
        myLaw->SetPerturbation(Vector3Df(0, 0, 0), Vector3Df(0, 0, 0));
    }

    #ifdef PERTURBANCE_LOG
        std::ostringstream logStream;
        logStream << "\n\nTranslational Perturbation: (\t" 
                << myLaw->perturbation_trans.x() << ",\t"
                << myLaw->perturbation_trans.y() << ",\t"
                << myLaw->perturbation_trans.z() << "\t)\n"
                << "Rotational Perturbation: (\t"
                << myLaw->perturbation_rot.x() << ",\t"
                << myLaw->perturbation_rot.y() << ",\t"
                << myLaw->perturbation_rot.z() << "\t)\n\n";
        
        Thread::Info(logStream.str().c_str());
    #endif
    Thread::Info("\n\n\nDisturbancia aplicada \n\n\n\n");
}

void Drone::RejectDisturbance() {
    myLaw->isDisturbanceActive = !myLaw->isDisturbanceActive;
}

void Drone::ApplyControl() {
    Vector3Df ref_p(0, 0, 0);
    Vector3Df uav_p, uav_dp; 
    Vector3Df aim_p, aim_dp;
    Vector3Df rejectionPercent = flair::core::Vector3Df(rejectionPercent_layout->Value().x, rejectionPercent_layout->Value().y, rejectionPercent_layout->Value().z);
    Quaternion q = GetCurrentQuaternion();
    Vector3Df w;
    yawAngle = yawAngle_layout->Value();
    float yawAngleInRadians = yawAngle * M_PI / 180.0;
    Quaternion aim_yaw(std::cos(yawAngleInRadians / 2), 0, 0, std::sin(yawAngleInRadians / 2));
    aim_yaw.Normalize();

    aim_p = currentTarget;
    aim_dp = Vector3Df(0, 0, 0);

    uavVrpn->GetPosition(uav_p);
    uavVrpn->GetSpeed(uav_dp);

    MixOrientation();
    q = mixQuaternion;
    w = mixAngSpeed;

    uav_p = uav_p - ref_p;
    Quaternion qze = aim_yaw.GetConjugate() * q;
    Vector3Df thetaze = 2 * qze.GetLogarithm();
    float zsign = 1;
    if (thetaze.GetNorm() >= 3.1416) {
        zsign = -1;
    }
    aim_yaw = zsign * aim_yaw;

    CoordFrameCorrection(uav_p, uav_dp, w, aim_p);

    myLaw->SetRejectionPercent(rejectionPercent);
    myLaw->SetTarget(aim_p, aim_dp, aim_yaw);
    myLaw->UpdateDynamics(uav_p, uav_dp, q, w);
    myLaw->Update(GetTime());
}

float Drone::ComputeCustomThrust() {
    float thrust = myLaw->Output(3); 
    return std::isnan(thrust) ? 0.0f : -thrust;
}

void Drone::ComputeCustomTorques(Euler &torques) {
    ApplyControl();
    float roll = myLaw->Output(0);
    float pitch = myLaw->Output(1);
    float yaw = myLaw->Output(2);
    torques.roll = std::isnan(roll) ? 0.0f : roll;
    torques.pitch = std::isnan(pitch) ? 0.0f : -pitch;
    torques.yaw = std::isnan(yaw) ? 0.0f : -yaw;
}

void Drone::CoordFrameCorrection(Vector3Df &uav_p, Vector3Df &uav_dp, Vector3Df &w, Vector3Df &aim_p) {
    Vector3Df correction_uav_p(-uav_p.x, -uav_p.y, -uav_p.z);
    Vector3Df correction_uav_dp(-uav_dp.x, -uav_dp.y, -uav_dp.z);
    Vector3Df correction_aim_p(-aim_p.x, -aim_p.y, aim_p.z);
    uav_p = correction_uav_p;
    uav_dp = correction_uav_dp;
    aim_p = correction_aim_p;
    w.x *= -1;
}

void Drone::MixOrientation() {
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    if (first_up) {
        Quaternion vrpnQuaternion;
        uavVrpn->GetQuaternion(vrpnQuaternion);
        Euler vrpnEuler;
        vrpnQuaternion.ToEuler(vrpnEuler);
        vrpnQuaternion.q0 = std::cos(vrpnEuler.yaw / 2);
        vrpnQuaternion.q1 = 0;
        vrpnQuaternion.q2 = 0;
        vrpnQuaternion.q3 = std::sin(vrpnEuler.yaw / 2);
        vrpnQuaternion.Normalize();
        qI = vrpnQuaternion * ahrsQuaternion.GetConjugate();
        first_up = false;
    }

    mixQuaternion = qI * ahrsQuaternion;
    mixQuaternion.Normalize();
    mixAngSpeed = ahrsAngularSpeed;
}
