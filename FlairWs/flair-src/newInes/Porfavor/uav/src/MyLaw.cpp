#include "MyLaw.h"

namespace flair
{
namespace filter
{
	
MyLaw::MyLaw(const LayoutPosition* position, string name) : ControlLaw(position->getLayout(), name, 4) {
    
    /************************
    Dynamic varibles
    ************************/ 
    input = new Matrix(this,23,1,floatType,name);
    MatrixDescriptor* desc = new MatrixDescriptor(23,1);
    desc->SetElementName(0,0,"q.q0");
    desc->SetElementName(1,0,"q.q1");
    desc->SetElementName(2,0,"q.q2");
    desc->SetElementName(3,0,"q.q3");
    desc->SetElementName(4,0,"dq.q0");
    desc->SetElementName(5,0,"dq.q1");
    desc->SetElementName(6,0,"dq.q2");
    desc->SetElementName(7,0,"dq.q3");
    desc->SetElementName(8,0,"w.x");
    desc->SetElementName(9,0,"w.y");
    desc->SetElementName(10,0,"w.z");
    desc->SetElementName(11,0,"dw.x");
    desc->SetElementName(12,0,"dw.y");
    desc->SetElementName(13,0,"dw.z");
    desc->SetElementName(14,0,"p.x");
    desc->SetElementName(15,0,"p.y");
    desc->SetElementName(16,0,"p.z");
    desc->SetElementName(17,0,"dp.x");
    desc->SetElementName(18,0,"dp.y");
    desc->SetElementName(19,0,"dp.z");
    desc->SetElementName(20,0,"ddp.x");
    desc->SetElementName(21,0,"ddp.y");
    desc->SetElementName(22,0,"ddp.z");
    stateM = new Matrix(this,desc,floatType,name);
    AddDataToLog(stateM);
    Reset();
    GroupBox* reglages_groupbox = new GroupBox(position,name);

    /************************
    TRANSLATIONAL PARAMS LAYOUT
    ************************/
    kp_rot_layout = new Vector3DSpinBox(reglages_groupbox->NewRow(),"kp_rot,.",-100,100,0.0001,6,Vector3Df(-3.000000,3.000000,3.000000));
    kd_rot_1_layout = new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kd_rot_1,",-100,100,0.0001,5,Vector3Df(0.500000,0.500000,0.500000));
    kd_rot_2_layout = new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kd_rot_2,",-100,100,0.0001,5,Vector3Df(0.010000,0.010000,0.010000));
    sat_rot_layout = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sat_rot:",0,200,0.1,2,10);

    /************************
    TRANSLATIONAL PARAMS LAYOUT
    ************************/
    kp_trans_layout = new Vector3DSpinBox(reglages_groupbox->NewRow(),"kpp_trans",-100,100,0.0000001,10,Vector3Df(0.0000000200,0.0000000200,0.0000050000));
    kd_trans_1_layout = new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kd_trans_1,",-100,100,0.0001,10,Vector3Df(0.0000000100,0.0000000100,0.0000500000));
    kd_trans_2_layout = new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kd_trans_2,",-100,100,0.0001,10,Vector3Df(0.0000000010,0.0000000010,0.0010000000));
    sat_trans_layout = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sat_trans:",0,200,0.1,2,10);
    omega_gains_trans = new Vector3DSpinBox(reglages_groupbox->NewRow(),"omegaUDE_trans",0,100,0.01);
    
    /***********************
    WEIGHT LAYOUT
    ************************/
    mass_layout = new DoubleSpinBox(reglages_groupbox->NewRow(),"Massa que mas aplauda",0.1,10,0.001,3,0.405);
    J_layout = new Vector3DSpinBox(reglages_groupbox->NewRow(),"Inertia",0.1,10,0.1,2,Vector3Df(0.01,0.01,0.02));

    /***********************
    Quaternion errorc
    ************************/
    eq = Eigen::Quaternionf(1,0,0,0);

    /***********************
    CSV write instances
    ************************/
    controlFilePath = CONTROL_FILE_PATH;
    debugFilePath = DEBUG_FILE_PATH;

    #ifdef SaveControlData2CSV
        controlOutputFile.open(controlFilePath, std::ios::trunc);
        if (controlOutputFile.is_open()) {
            controlOutputFile << "Tauu_x,Tauu_y,Tauu_z,Fu\n";
        }
        controlOutputFile.close();
    #endif
    #ifdef SaveMyLawDebug2CSV
        controlOutputFile.open(debugFilePath, std::ios::trunc);
        if (controlOutputFile.is_open()) {
            controlOutputFile << "dt,u_thrust_x,u_thrust_y,u_thrust_z,"
                            << "uz_uvec_x,uz_uvec_y,uz_uvec_z,"
                            << "ep_x,ep_y,ep_z,"
                            << "eq_w,eq_x,eq_y,eq_z,"
                            << "debugRot_x,debugRot_y,debugRot_z,"
                            << "eomega_x,eomega_y,eomega_z,"
                            << "u_torque_x,u_torque_y,u_torque_z\n";
        }
        controlOutputFile.close();
    #endif
}

MyLaw::~MyLaw(void) {
    if (controlOutputFile.is_open()) {
        controlOutputFile.close();
    }
}

void MyLaw::UseDefaultPlot(const gui::LayoutPosition* position) {
}

/***********************
MY CONTROL CHIDO >:v
************************/
void MyLaw::UpdateFrom(const io_data *data) {


    // Observer gain
    #ifdef STATESPACE_H
        Observer::StateSpace::J = Eigen::DiagonalMatrix<float, 3>(J_layout->Value().x, J_layout->Value().y, J_layout->Value().z);

        Observer::StateSpace::Omega_UDE_trans = (Eigen::Matrix3f() << 
                                           omega_gains_trans->Value().x, 0.0f, 0.0f,
                                           0.0f, omega_gains_trans->Value().y, 0.0f,
                                           0.0f, 0.0f, omega_gains_trans->Value().z).finished();

    #endif
    
    // Control parse
    float dt,Fth;
    dt = (float)(data->DataTime()-previous_time);
    input->GetMutex();
    input->GetMutex();
    Eigen::MatrixXf stateM(23, 1); 
    Eigen::MatrixXf outputMatrix(4, 1); 

    stateM(0, 0) = input->ValueNoMutex(0, 0);
    stateM(1, 0) = input->ValueNoMutex(1, 0);
    stateM(2, 0) = input->ValueNoMutex(2, 0);
    stateM(3, 0) = input->ValueNoMutex(3, 0);
    stateM(4, 0) = input->ValueNoMutex(4, 0);
    stateM(5, 0) = input->ValueNoMutex(5, 0);
    stateM(6, 0) = input->ValueNoMutex(6, 0);
    stateM(7, 0) = input->ValueNoMutex(7, 0);
    stateM(8, 0) = input->ValueNoMutex(8, 0);
    stateM(9, 0) = input->ValueNoMutex(9, 0);
    stateM(10, 0) = input->ValueNoMutex(10, 0);
    stateM(11, 0) = input->ValueNoMutex(11, 0);
    stateM(12, 0) = input->ValueNoMutex(12, 0);
    stateM(13, 0) = input->ValueNoMutex(13, 0);
    stateM(14, 0) = input->ValueNoMutex(14, 0);
    stateM(15, 0) = input->ValueNoMutex(15, 0);
    stateM(16, 0) = input->ValueNoMutex(16, 0);
    stateM(17, 0) = input->ValueNoMutex(17, 0);
    stateM(18, 0) = input->ValueNoMutex(18, 0);
    stateM(19, 0) = input->ValueNoMutex(19, 0);
    stateM(20, 0) = input->ValueNoMutex(20, 0);
    stateM(21, 0) = input->ValueNoMutex(21, 0);
    stateM(22, 0) = input->ValueNoMutex(22, 0);
    input->ReleaseMutex();

    kp_rot = Eigen::Vector3f(kp_rot_layout->Value().x, kp_rot_layout->Value().y, kp_rot_layout->Value().z);
    kd_rot_1 = Eigen::Vector3f(kd_rot_1_layout->Value().x, kd_rot_1_layout->Value().y, kd_rot_1_layout->Value().z);
    kd_rot_2 = Eigen::Vector3f(kd_rot_2_layout->Value().x, kd_rot_2_layout->Value().y, kd_rot_2_layout->Value().z);
    sat_rot = sat_rot_layout->Value();
    kp_trans = Eigen::Vector3f(kp_trans_layout->Value().x, kp_trans_layout->Value().y, kp_trans_layout->Value().z);
    kd_trans_1 = Eigen::Vector3f(kd_trans_1_layout->Value().x, kd_trans_1_layout->Value().y, kd_trans_1_layout->Value().z);
    kd_trans_2 = Eigen::Vector3f(kd_trans_2_layout->Value().x, kd_trans_2_layout->Value().y, kd_trans_2_layout->Value().z);
    sat_trans = sat_trans_layout->Value();

    CalculateControl(stateM, outputMatrix, dt);

    /**************************************
        OUTPUT Torque, thustForce, dq
    **************************************/

    output->SetValue(0,0,outputMatrix(0, 0));
    output->SetValue(1,0,outputMatrix(1, 0));
    output->SetValue(2,0,outputMatrix(2, 0));
    output->SetValue(3,0,outputMatrix(3, 0));
    output->SetDataTime(data->DataTime());
    // previous_time = data->DataTime();
    previous_time = dt;

    ProcessUpdate(output);
}


void MyLaw::Reset(void) {
    p_d.x() = 0;
    p_d.y() = 0;
    p_d.z() = 0;
}


void MyLaw::CalculateControl(const Eigen::MatrixXf& stateM, Eigen::MatrixXf& outputMatrix, float dt) {
    // Translational and Rotational state extraction
    mass = mass_layout->Value();
    Eigen::Quaternionf q(stateM(0, 0), stateM(1, 0), stateM(2, 0), stateM(3, 0));
    Eigen::Quaternionf dq(stateM(4, 0), stateM(5, 0), stateM(6, 0), stateM(7, 0));
    Eigen::Vector3f w(stateM(8, 0), stateM(9, 0), stateM(10, 0));
    Eigen::Vector3f dw(stateM(11, 0), stateM(12, 0), stateM(13, 0));
    Eigen::Vector3f p(stateM(14, 0), stateM(15, 0), stateM(16, 0));
    Eigen::Vector3f dp(stateM(17, 0), stateM(18, 0), stateM(19, 0));
    Eigen::Vector3f ddp(stateM(20, 0), stateM(21, 0), stateM(22, 0));

    Eigen::Vector3f w_estimation_trans = Observer::StateSpace::EstimateDisturbanceUDE_trans(p, dp, dt);
    Eigen::Vector3f w_estimation_rot = Observer::StateSpace::EstimateDisturbanceUDE_rot(q, w, dt);

    // Errors
    Eigen::Vector3f ep = p_d.array() - p.array();


    // add to debug string XD
    std::ostringstream debugDeste;
    // Translational Control
    u_thrust = (kp_trans.array() * ep.array()) - (kd_trans_1.array() * dp.array());
    if (u_thrust.norm() != 0) {
        u_thrust = u_thrust.normalized();
    } else {
        u_thrust = Eigen::Vector3f::Zero();
    }
    if (u_thrust.norm() != 0) {
        u_thrust = sat_trans * tanh(u_thrust.norm() / sat_trans) * u_thrust.normalized();
    }
    u_thrust += Eigen::Vector3f(0, 0, g * mass) - (kd_trans_2.array() * dp.array()).matrix();
    Fu = u_thrust.norm();

    // Rotation Control
    Eigen::Vector3f uz_uvec;
    if (u_thrust.norm() != 0) {
        uz_uvec = u_thrust.normalized();
    } else {
        uz_uvec = Eigen::Vector3f::UnitZ();
    }   
    float dot_product = Eigen::Vector3f::UnitZ().dot(uz_uvec);
    Eigen::Vector3f cross_product = Eigen::Vector3f::UnitZ().cross(uz_uvec);
    Eigen::Quaternionf q_temp(dot_product, cross_product.x(), cross_product.y(), cross_product.z());
    Eigen::Quaternionf q_d = ExpQuaternion(Eigen::Quaternionf(0.5f * LogQuaternion(q_temp).coeffs()));    
    q_d.normalize();
    Eigen::Quaternionf eq_prev = eq;
    eq = q * q_d.conjugate();   
    eq.normalize(); 
    // Uncomment if needed to calculate
    // Eigen::Vector3f eomega = RotVec(eq * eq_prev.conjugate())/dt;
    Eigen::Vector3f eomega = w; // If enable to mesure, then do it
    if (!eomega.array().isFinite().all()) {
        eomega = Eigen::Vector3f::Zero();
    }
    u_torque = -kp_rot.array() * RotVec(eq).array() - kd_rot_1.array() * eomega.array();    
    if (u_torque.norm() != 0) {
        u_torque = sat_rot * tanh(u_torque.norm() / sat_rot) * u_torque.normalized().array() - kd_rot_2.array() * eomega.array();
    }
    else {
        u_torque = Eigen::Vector3f::Zero();
    }
    Eigen::Vector3f debugRot = RotVec(eq);
    #ifdef SaveMyLawDebug2CSV
        SaveAllDataToCSV(dt, u_thrust, uz_uvec, ep, eq, debugRot, eomega, u_torque);
    #endif

    #ifdef MyLawDebugLog
        std::ostringstream errorStream;
        errorStream << "\ndt : (\t" << dt << ")\n";
        errorStream << "\ntrhust (thrust): (\t" << u_thrust.x() << ",\t" << u_thrust.y() << ",\t" << u_thrust.z() << ")\n";
        errorStream << "\nuz (uz error): (\t" << uz_uvec.x() << ",\t" << uz_uvec.y() << ",\t" << uz_uvec.z() << ")\n";
        errorStream << "\nep (position error): (\t" << ep.x() << ",\t" << ep.y() << ",\t" << ep.z() << ")\n";
        errorStream << "\neq (quaternion error): (\t" << eq.w() << ",\t" << eq.x() << ",\t" << eq.y() << ",\t" << eq.z() << ")\n";
        errorStream << "\nrotVec (rot vec): (\t" << debugRot.x() << ",\t" << debugRot.y() << ",\t" << debugRot.z() << ")\n";
        errorStream << "\neomega (eomega vec): (\t" << eomega.x() << ",\t" << eomega.y() << ",\t" << eomega.z() << ")\n";
        errorStream << "\nu_torque (torque vec): (\t" << u_torque.x() << ",\t" << u_torque.y() << ",\t" << u_torque.z() << ")\n";
        something2debug = errorStream.str();
    #endif

    Tauu = J * u_torque;
    if (!Tauu.array().isFinite().all()) {
        Tauu = Eigen::Vector3f::Zero();
    }

    outputMatrix(0, 0) = Tauu.x();
    outputMatrix(1, 0) = Tauu.y();
    outputMatrix(2, 0) = Tauu.z();
    outputMatrix(3, 0) = Fu;
    #ifdef SaveControlData2CSV
        SaveControlDataToCSV();
    #endif
}


Eigen::Quaternionf MyLaw::ExpQuaternion(const Eigen::Quaternionf& q) {
    float w = q.w();
    Eigen::Vector3f v(q.x(), q.y(), q.z());
    float v_norm = v.norm();
    Eigen::Quaternionf exp_q;
    exp_q.w() = std::exp(w) * std::cos(v_norm); 
    if (v_norm != 0) {
        Eigen::Vector3f exp_v = std::exp(w) * (std::sin(v_norm) / v_norm) * v;
        exp_q.vec() = exp_v; 
    } else {
        exp_q.vec() = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
    return exp_q.normalized();
}

Eigen::Quaternionf MyLaw::LogQuaternion(const Eigen::Quaternionf& q) {
    Eigen::Quaternionf normalized_q = q.normalized();
    float w = normalized_q.w();
    Eigen::Vector3f v(normalized_q.x(), normalized_q.y(), normalized_q.z()); 
    float v_norm = v.norm();
    float theta = std::acos(w);
    Eigen::Quaternionf log_q;
    log_q.w() = 0.0f;
    if (v_norm != 0) {
        Eigen::Vector3f log_v = (theta / v_norm) * v;
        log_q.vec() = log_v;
    } else {
        log_q.vec() = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
    if (!log_q.vec().array().isFinite().all()) {
        log_q.vec() = Eigen::Vector3f::Zero();
    }
    return log_q;
}

Eigen::Vector3f MyLaw::RotVec(const Eigen::Quaternionf &quat) {
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
    double norm = std::sqrt(x * x + y * y + z * z);
    if (norm < 1e-6) {
        return Eigen::Vector3f::Zero();
    }
    double theta = 2 * std::atan2(norm, w);
    return theta * Eigen::Vector3f(x / norm, y / norm, z / norm);
}

void MyLaw::SaveControlDataToCSV() {
    controlOutputFile.open(controlFilePath, std::ios::app);
    if (controlOutputFile.is_open()) {
        controlOutputFile << Tauu.x() << "," << Tauu.y() << "," << Tauu.z() << "," << Fu << "\n";
    } else {
        std::cerr << "Failed to open control_data.csv for writing.\n";
    }
    controlOutputFile.close();
}

void MyLaw::SaveAllDataToCSV(float dt, 
                             const Eigen::Vector3f& u_thrust, 
                             const Eigen::Vector3f& uz_uvec, 
                             const Eigen::Vector3f& ep, 
                             const Eigen::Quaternionf& eq, 
                             const Eigen::Vector3f& debugRot, 
                             const Eigen::Vector3f& eomega, 
                             const Eigen::Vector3f& u_torque) {
    controlOutputFile.open(debugFilePath, std::ios::app);
    if (controlOutputFile.is_open()) {
        controlOutputFile << dt << ","
                          << u_thrust.x() << "," << u_thrust.y() << "," << u_thrust.z() << ","
                          << uz_uvec.x() << "," << uz_uvec.y() << "," << uz_uvec.z() << ","
                          << ep.x() << "," << ep.y() << "," << ep.z() << ","
                          << eq.w() << "," << eq.x() << "," << eq.y() << "," << eq.z() << ","
                          << debugRot.x() << "," << debugRot.y() << "," << debugRot.z() << ","
                          << eomega.x() << "," << eomega.y() << "," << eomega.z() << ","
                          << u_torque.x() << "," << u_torque.y() << "," << u_torque.z() << "\n";
    } else {
        std::cerr << "Failed to open CSV file for writing.\n";
    }
    controlOutputFile.close();
}


void MyLaw::SetTarget(Vector3Df target_pos, Vector3Df target_vel){
    p_d = Eigen::Vector3f(target_pos.x, target_pos.y, target_pos.z);
}

void MyLaw::UpdateDynamics(Vector3Df p, Vector3Df dp, Quaternion q,Vector3Df w){
    input->SetValue(0,0,q.q0);
    input->SetValue(1,0,q.q1);
    input->SetValue(2,0,q.q2);
    input->SetValue(3,0,q.q3);
    input->SetValue(8,0,w.x);
    input->SetValue(9,0,w.y);
    input->SetValue(10,0,w.z);
    input->SetValue(14,0,p.x);
    input->SetValue(15,0,p.y);
    input->SetValue(16,0,p.z);
    input->SetValue(17,0,dp.x);
    input->SetValue(18,0,dp.y);
    input->SetValue(19,0,dp.z);
};

} // end namespace filter
} // end namespace flair


