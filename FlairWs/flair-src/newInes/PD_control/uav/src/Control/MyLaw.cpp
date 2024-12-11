#include "MyLaw.h"

namespace flair
{
namespace filter
{
	
MyLaw::MyLaw(const LayoutPosition* position, string name) : ControlLaw(position->getLayout(), name, 4) {
       
    previous_chrono_time = std::chrono::high_resolution_clock::now();
    firstUpdate = true;
    isDisturbanceActive = false;
    activation_delay = 0.0f;
    this_time = 0;
    rejectionPercent = Eigen::Vector3f(0,0,0);

    /************************
    Mutex for data access.
    ************************/
    input = new Matrix(this,23,1,floatType,name);

    /************************
    Label descriptor for data saved.
    ************************/
    MatrixDescriptor* desc = new MatrixDescriptor(23,1);
    desc->SetElementName(0,0,"q0");
    desc->SetElementName(1,0,"q1");
    desc->SetElementName(2,0,"q2");
    desc->SetElementName(3,0,"q3");
    desc->SetElementName(4,0,"wx");
    desc->SetElementName(5,0,"wy");
    desc->SetElementName(6,0,"wz");
    desc->SetElementName(7,0,"px");
    desc->SetElementName(8,0,"py");
    desc->SetElementName(9,0,"pz");
    desc->SetElementName(10,0,"u_roll");
    desc->SetElementName(11,0,"u_pitch");
    desc->SetElementName(12,0,"u_yaw");
    desc->SetElementName(13,0,"thrust");
    desc->SetElementName(14,0,"ecx");
    desc->SetElementName(15,0,"ecy");
    desc->SetElementName(16,0,"ecz");
    desc->SetElementName(17,0,"udeTx");
    desc->SetElementName(18,0,"udeTy");
    desc->SetElementName(19,0,"udeTz");
    desc->SetElementName(20,0,"udeRx");
    desc->SetElementName(21,0,"udeRy");
    desc->SetElementName(22,0,"udeRz");
    dataexp = new Matrix(this,desc,floatType,name);
    delete desc;
    AddDataToLog(dataexp);
    Reset();

    GroupBox* reglages_groupbox = new GroupBox(position,name);

    /************************
    TRANSLATIONAL PARAMS LAYOUT
    ************************/
    kp_rot_layout = new Vector3DSpinBox(reglages_groupbox->NewRow(),"kp_rot,.",-100,100,0.0001,6,Vector3Df(-30.000000,30.000000,30.000000));
    kd_rot_1_layout = new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kd_rot_1,",-100,100,0.0001,5,Vector3Df(15.000000,15.000000,15.000000));
    kd_rot_2_layout = new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kd_rot_2,",-100,100,0.0001,5,Vector3Df(5.00000,5.00000,5.00000));
    sat_rot_layout = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sat_rot:",0,200,0.1,2,10);
    omega_gains_rot = new Vector3DSpinBox(reglages_groupbox->NewRow(),"omegaUDE_rot",0,100,0.01,3,Vector3Df(80.0,80.0,80.0));


    /************************
    TRANSLATIONAL PARAMS LAYOUT
    ************************/
    kp_trans_layout = new Vector3DSpinBox(reglages_groupbox->NewRow(),"kpp_trans",-100,100,0.0000001,10,Vector3Df(0.1000000000,0.1000000000,0.1000000000));
    kd_trans_1_layout = new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kd_trans_1,",-100,100,0.0001,10,Vector3Df(0.5000000000,0.5000000000,0.5000000000));
    kd_trans_2_layout = new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kd_trans_2,",-100,100,0.0001,10,Vector3Df(0.0100000000,0.0100000000,0.0100000000));
    sat_trans_layout = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sat_trans:",0,200,0.1,2,10);
    omega_gains_trans = new Vector3DSpinBox(reglages_groupbox->NewRow(),"omegaUDE_trans",0,100,0.01,3,Vector3Df(60.0,60.0,60.0));
    
    /***********************
    WEIGHT LAYOUT
    ************************/
    mass_layout = new DoubleSpinBox(reglages_groupbox->NewRow(),"Massa que mas aplauda",0.1,10,0.001,3,0.405);
    motorConst = new DoubleSpinBox(reglages_groupbox->NewRow(),"Motor const",0,20,0.0001,10,10);

    Observer::ObserverBase::J = Eigen::Matrix3f();
    Observer::ObserverBase::J <<    2098e-6, 63.577538e-6, -2.002648e-6,
                                    63.577538e-6, 2102e-6, 0.286186e-6,
                                    -2.002648e-6, 0.286186e-6, 4068e-6;

    /***********************
    Quaternion errorc
    ************************/
    eq = Eigen::Quaternionf(1,0,0,0);
    perturbation_trans = Eigen::Vector3f(0,0,0);
    perturbation_rot = Eigen::Vector3f(0,0,0);

    /***********************
    CSV write instances
    ************************/
    #ifdef PARAMSIM_H

        #ifdef SAVE_CONTROL_INPUT_CSV
            controlFilePath = CONTROL_INPUT_FILE_PATH_CSV;
            controlInputFileCSV.open(controlFilePath, std::ios::trunc);
            if (controlInputFileCSV.is_open()) {
                controlInputFileCSV << "Tauu_x,Tauu_y,Tauu_z,Fu\n";
            }
            controlInputFileCSV.close();
        #endif
        #ifdef SAVE_DEBUG_CSV
            debugFilePath = DEBUG_FILE_PATH_CSV;
            controlDebugFileCSV.open(debugFilePath, std::ios::trunc);
            if (controlDebugFileCSV.is_open()) {
                controlDebugFileCSV << "dt,u_thrust_x_1,u_thrust_y_1,u_thrust_z_1,"
                                << "u_thrust_x_2,u_thrust_y_2,u_thrust_z_2,"
                                << "u_thrust_x,u_thrust_y,u_thrust_z,"
                                << "uz_uvec_x,uz_uvec_y,uz_uvec_z,"
                                << "ep_x,ep_y,ep_z,"
                                << "eq_w,eq_x,eq_y,eq_z,"
                                << "debugRot_x,debugRot_y,debugRot_z,"
                                << "eomega_x,eomega_y,eomega_z,"
                                << "u_torque_x,u_torque_y,u_torque_z\n";
            }
            controlDebugFileCSV.close();
        #endif
        #ifdef SAVE_REAL_STATE_SPACE_CSV
            translationFilePath = TRANSLATION_FILE_PATH_CSV;
            rotationFilePath = ROTATION_FILE_PATH_CSV;
            translationOutputFileCSV.open(translationFilePath, std::ios::trunc);
            rotationOutputFileCSV.open(rotationFilePath, std::ios::trunc);
            translationOutputFileCSV    << "p_x,p_y,p_z,dp_x,dp_y,dp_z,ddp_x,ddp_y,ddp_z,dt\n";
            rotationOutputFileCSV       << "domega_x,domega_y,domega_z,omega_x,omega_y,omega_z,"
                                        << "dq_w,dq_x,dq_y,dq_z,q_w,q_x,q_y,q_z,dt\n";
        #endif
    #endif
}

MyLaw::~MyLaw(void) {
    if (translationOutputFileCSV.is_open()) {
        translationOutputFileCSV.close();
    }
    if (rotationOutputFileCSV.is_open()) {
        rotationOutputFileCSV.close();
    }
    if (controlInputFileCSV.is_open()) {
        controlInputFileCSV.close();
    }
    if (controlDebugFileCSV.is_open()) {
        controlDebugFileCSV.close();
    }
}

void MyLaw::UseDefaultPlot(const gui::LayoutPosition* position) {
}

/***********************
MY CONTROL CHIDO >:v
************************/
void MyLaw::UpdateFrom(const io_data *data) {
    if(firstUpdate){
        previous_chrono_time = std::chrono::high_resolution_clock::now();
        std::ostringstream startLogStream;
        startLogStream << "\nStart\n";
        something2stream = startLogStream.str();
    }
     
    #if OBSERVER_TYPE == UDE_OBSERVER
        Observer::UDE::Omega_UDE_trans = (Eigen::Matrix3f() << 
                                            omega_gains_trans->Value().x, 0.0f, 0.0f,
                                            0.0f, omega_gains_trans->Value().y, 0.0f,
                                            0.0f, 0.0f, omega_gains_trans->Value().z).finished();

        Observer::UDE::Omega_UDE_rot = (Eigen::Matrix3f() << 
                                            omega_gains_rot->Value().x, 0.0f, 0.0f,
                                            0.0f, omega_gains_rot->Value().y, 0.0f,
                                            0.0f, 0.0f, omega_gains_rot->Value().z).finished();
    #endif

    motorK = motorConst->Value();
    
    // dt Calc
    float dt,Fth;
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> alt_dt = current_time - previous_chrono_time;
    dt = alt_dt.count();
    previous_chrono_time = current_time;

    #ifdef ACTIVATE_DISTURBANCE_REJECTION_BY_TIME
        if (!isDisturbanceActive) {
            this_time += dt;
            std::cout << "time " << dt << "\n";
            if (this_time >= activation_delay) {
                isDisturbanceActive = true;
                std::cout << "Disturbance estimator activated after " << activation_delay << " seconds\n";
            }
        }
    #endif

    #ifdef DT_LOG
        std::ostringstream dtLogStream;
        dtLogStream << "\nCalculated dt (original): " << dt << " seconds\n";
        something2stream = dtLogStream.str();
    #endif

    /**************************************
        Flait parse to Eigen
    **************************************/

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

    /**************************************
        Layout 2 variable
    **************************************/

    kp_rot = Eigen::Vector3f(kp_rot_layout->Value().x, kp_rot_layout->Value().y, kp_rot_layout->Value().z);
    kd_rot_1 = Eigen::Vector3f(kd_rot_1_layout->Value().x, kd_rot_1_layout->Value().y, kd_rot_1_layout->Value().z);
    kd_rot_2 = Eigen::Vector3f(kd_rot_2_layout->Value().x, kd_rot_2_layout->Value().y, kd_rot_2_layout->Value().z);
    sat_rot = sat_rot_layout->Value();
    kp_trans = Eigen::Vector3f(kp_trans_layout->Value().x, kp_trans_layout->Value().y, kp_trans_layout->Value().z);
    kd_trans_1 = Eigen::Vector3f(kd_trans_1_layout->Value().x, kd_trans_1_layout->Value().y, kd_trans_1_layout->Value().z);
    kd_trans_2 = Eigen::Vector3f(kd_trans_2_layout->Value().x, kd_trans_2_layout->Value().y, kd_trans_2_layout->Value().z);
    sat_trans = sat_trans_layout->Value();

    /**************************************
        Control
    **************************************/

    CalculateControl(stateM, outputMatrix, dt);

    /**************************************
        OUTPUT Torque, thustForce, dq
    **************************************/

    output->SetValue(0,0,outputMatrix(0, 0));
    output->SetValue(1,0,outputMatrix(1, 0));
    output->SetValue(2,0,outputMatrix(2, 0));
    output->SetValue(3,0,outputMatrix(3, 0));
    ProcessUpdate(output);
    firstUpdate = false;
}

void MyLaw::Reset(void) {
    p_d.x() = 0;
    p_d.y() = 0;
    p_d.z() = 0;
}

void MyLaw::CalculateControl(const Eigen::MatrixXf& stateM, Eigen::MatrixXf& outputMatrix, float dt) {
    
    /**************************************
        State extraction
    **************************************/
    
    Observer::ObserverBase::mass = mass_layout->Value();
    Eigen::Quaternionf q(stateM(0, 0), stateM(1, 0), stateM(2, 0), stateM(3, 0));
    Eigen::Quaternionf dq(stateM(4, 0), stateM(5, 0), stateM(6, 0), stateM(7, 0));
    Eigen::Vector3f w(stateM(8, 0), stateM(9, 0), stateM(10, 0));
    Eigen::Vector3f dw(stateM(11, 0), stateM(12, 0), stateM(13, 0));
    Eigen::Vector3f p(stateM(14, 0), stateM(15, 0), stateM(16, 0));
    Eigen::Vector3f dp(stateM(17, 0), stateM(18, 0), stateM(19, 0));
    Eigen::Vector3f ddp(stateM(20, 0), stateM(21, 0), stateM(22, 0));

    if(firstUpdate){
        dt = 0;
    }

    /**************************************
        Errors
    **************************************/
    
    //Eigen::Vector3f ep = p_d.array() - p.array();


    //Convercion de coords inerciales para el error del body frame
    Eigen::Vector3f ep_inertial = p_d - p; // Position error in inertial frame
    Eigen::Vector3f edp_inertial = dp;
    Eigen::Vector3f ep_body = q.conjugate() * ep_inertial; // Transform error to body frame
    Eigen::Vector3f edp_body = q.conjugate() * edp_inertial;

    Eigen::Vector3f ep = ep_body;
    dp = edp_body;


    /**************************************
        Translational Control
    **************************************/

    u_thrust = (kp_trans.array() * ep.array()) - (kd_trans_1.array() * dp.array());
    #if defined(SAVE_DEBUG_CSV) || defined(MY_LAW_DEBUG_LOG)
        std::ostringstream errorStream;
        Eigen::Vector3f u_thrust_1 = u_thrust;
        errorStream << "\nFirst u_thrust (thrust): (\t" << u_thrust_1.x() << ",\t" << u_thrust_1.y() << ",\t" << u_thrust_1.z() << ")\n";
    #endif

    if (u_thrust.norm() != 0) {
        u_thrust = sat_trans * tanh(u_thrust.norm() / sat_trans) * u_thrust.normalized();
    }
    
    #if defined(SAVE_DEBUG_CSV) || defined(MY_LAW_DEBUG_LOG)
        Eigen::Vector3f u_thrust_2 = u_thrust;
        errorStream << "\nSecond u_thrust (thrust): (\t" << u_thrust_2.x() << ",\t" << u_thrust_2.y() << ",\t" << u_thrust_2.z() << ")\n";
    #endif

    float perturbed_Fu;

    u_thrust += Eigen::Vector3f(0, 0, g * mass) - (kd_trans_2.array() * dp.array()).matrix();

    #ifdef PERTURBANCE_LOG
        Eigen::Vector3f rej = compensation;
        errorStream << "\nrejection error: (\t" << rej.x() << ",\t" << rej.y() << ",\t" << rej.z() << ")\n";
    #endif

    Fu = u_thrust.norm();

    /**************************************
        Rotational Control
    **************************************/

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

    // Define CALC_EOMEGA if we dont have the w measurement
    #ifdef CALC_EOMEGA
        Eigen::Vector3f eomega = RotVec(eq * eq_prev.conjugate())/dt;
    #else
        Eigen::Vector3f eomega = w; // If enable to mesure, then do it
    #endif
    if (!eomega.array().isFinite().all()) {
        eomega = Eigen::Vector3f::Zero();
    }
    u_torque = -kp_rot.array() * RotVec(eq).array() - kd_rot_1.array() * eomega.array();    
    if (u_torque.norm() != 0) {
        u_torque = (sat_rot * tanh(u_torque.norm() / sat_rot) * u_torque.normalized().array() - kd_rot_2.array() * eomega.array());
    }
    else {
        u_torque = Eigen::Vector3f::Zero();
    }
    Eigen::Vector3f debugRot = RotVec(eq);

    #ifdef SAVE_DEBUG_CSV
        SaveDebugCSV(dt, u_thrust_1, u_thrust_2, u_thrust, uz_uvec, ep, eq, debugRot, eomega, u_torque);
    #endif

    #ifdef MY_LAW_DEBUG_LOG
        //std::ostringstream errorStream;
        errorStream << "\ndt : (\t" << dt << ")\n";
        errorStream << "\ntrhust (thrust): (\t" << u_thrust.x() << ",\t" << u_thrust.y() << ",\t" << u_thrust.z() << ")\n";
        errorStream << "\nuz (uz error): (\t" << uz_uvec.x() << ",\t" << uz_uvec.y() << ",\t" << uz_uvec.z() << ")\n";
        errorStream << "\nep (position error): (\t" << ep.x() << ",\t" << ep.y() << ",\t" << ep.z() << ")\n";
        errorStream << "\neq (quaternion error): (\t" << eq.w() << ",\t" << eq.x() << ",\t" << eq.y() << ",\t" << eq.z() << ")\n";
        errorStream << "\nrotVec (rot vec): (\t" << debugRot.x() << ",\t" << debugRot.y() << ",\t" << debugRot.z() << ")\n";
        errorStream << "\neomega (eomega vec): (\t" << eomega.x() << ",\t" << eomega.y() << ",\t" << eomega.z() << ")\n";
        errorStream << "\nu_torque (torque vec): (\t" << u_torque.x() << ",\t" << u_torque.y() << ",\t" << u_torque.z() << ")\n";
        
        something2stream = errorStream.str();
    #endif

    #ifdef ERROR_LOG
        std::ostringstream errorPQstream;
        errorPQstream << "\nep (position error): (\t" << ep.x() << ",\t" << ep.y() << ",\t" << ep.z() << ")\n";
        errorPQstream << "\neq (quaternion error): (\t" << eq.w() << ",\t" << eq.x() << ",\t" << eq.y() << ",\t" << eq.z() << ")\n";
    #endif

    Tauu = J * u_torque;
    if (!Tauu.array().isFinite().all()) {
        Tauu = Eigen::Vector3f::Zero();
    }

    Fu = Fu / motorK;
    Tauu = Tauu / motorK;

    // Data to be saved using FlAir
    dataexp->GetMutex();
    dataexp->SetValue(0,0, q.w());
    dataexp->SetValue(1,0, q.x());
    dataexp->SetValue(2,0, q.y());
    dataexp->SetValue(3,0, q.z());
    dataexp->SetValue(4,0, w(0));
    dataexp->SetValue(5,0, w(1));
    dataexp->SetValue(6,0, w(2));
    dataexp->SetValue(7,0, p(0));
    dataexp->SetValue(8,0, p(1));
    dataexp->SetValue(9,0, p(2));
    dataexp->SetValue(10,0, Tauu.x());
    dataexp->SetValue(11,0, Tauu.y());
    dataexp->SetValue(12,0, Tauu.z());
    dataexp->SetValue(13,0, Fu);
    dataexp->SetValue(14,0, ep(0));
    dataexp->SetValue(15,0, ep(1));
    dataexp->SetValue(16,0, ep(2));
    dataexp->SetValue(17,0, 0);
    dataexp->SetValue(18,0, 0);
    dataexp->SetValue(19,0, 0);
    dataexp->SetValue(20,0, 0);
    dataexp->SetValue(21,0, 0);
    dataexp->SetValue(22,0, 0);
    dataexp->ReleaseMutex();

    outputMatrix(0, 0) = Tauu.x();
    outputMatrix(1, 0) = Tauu.y();
    outputMatrix(2, 0) = Tauu.z();
    outputMatrix(3, 0) = Fu; // Cambiar a Fu si se quiere el valor sin perturbaciones
    #ifdef SAVE_CONTROL_INPUT_CSV
        SaveControlCSV();
    #endif

    #ifdef SAVE_REAL_STATE_SPACE_CSV
        SaveStateCSV(p, dp, ddp, dw, w, dq, q, dt); 
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

void MyLaw::SaveControlCSV() {
    controlInputFileCSV.open(controlFilePath, std::ios::app);
    if (controlInputFileCSV.is_open()) {
        controlInputFileCSV << Tauu.x() << "," << Tauu.y() << "," << Tauu.z() << "," << Fu << "\n";
    } else {
        std::cerr << "Failed to open control_data.csv for writing.\n";
    }
    controlInputFileCSV.close();
}

void MyLaw::SaveDebugCSV(float dt, 
                             const Eigen::Vector3f& u_thrust_1,
                             const Eigen::Vector3f& u_thrust_2,
                             const Eigen::Vector3f& u_thrust, 
                             const Eigen::Vector3f& uz_uvec, 
                             const Eigen::Vector3f& ep, 
                             const Eigen::Quaternionf& eq, 
                             const Eigen::Vector3f& debugRot, 
                             const Eigen::Vector3f& eomega, 
                             const Eigen::Vector3f& u_torque) {
    controlDebugFileCSV.open(debugFilePath, std::ios::app);
    if (controlDebugFileCSV.is_open()) {
        controlDebugFileCSV << dt << ","
                            << u_thrust_1.x() << "," << u_thrust_1.y() << "," << u_thrust_1.z() << ","
                            << u_thrust_2.x() << "," << u_thrust_2.y() << "," << u_thrust_2.z() << ","
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
    controlDebugFileCSV.close();
}

void MyLaw::SaveStateCSV(Eigen::Vector3f &p, Eigen::Vector3f &dp,Eigen::Vector3f &ddp, Eigen::Vector3f &domega, Eigen::Vector3f &omega, Eigen::Quaternionf &dq, Eigen::Quaternionf &q, float &dt){
    if (translationOutputFileCSV.is_open()) {
        translationOutputFileCSV    << std::fixed << std::setprecision(6)
                                    << p.x() << "," << p.y() << "," << p.z() << ","
                                    << dp.x() << "," << dp.y() << "," << dp.z() << ","
                                    << ddp.x() << "," << ddp.y() << "," << ddp.z() << "," << dt << "\n";
    } else {
        std::cerr << "Error opening RealStateSpace_trans.csv\n";
    }
    if (rotationOutputFileCSV.is_open()) {
        rotationOutputFileCSV   << domega.x() << "," << domega.y() << "," << domega.z() << ","
                                << omega.x() << "," << omega.y() << "," << omega.z() << ","
                                << dq.w() << "," << dq.x() << "," << dq.y() << "," << dq.z() << ","
                                << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "," << dt << "\n";
    } else {
        std::cerr << "Error opening RealStateSpace_rot.csv\n";
    }
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

void MyLaw::SetTarget(Vector3Df target_pos, Vector3Df target_vel){
    p_d = Eigen::Vector3f(target_pos.x, target_pos.y, target_pos.z);
}

void MyLaw::SetPerturbation(Vector3Df p_trans, Vector3Df p_rot){
    perturbation_trans = Eigen::Vector3f(p_trans.x, p_trans.y, p_trans.z);
    perturbation_rot = Eigen::Vector3f(p_rot.x, p_rot.y, p_rot.z);
}

void MyLaw::SetRejectionPercent(Vector3Df rejection){
    rejectionPercent = Eigen::Vector3f(rejection.x, rejection.y, rejection.z);
}

} // end nasmespace filter
} // end namespace flair


