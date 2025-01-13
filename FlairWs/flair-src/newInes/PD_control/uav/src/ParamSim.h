#ifndef PARAMSIM_H
#define PARAMSIM_H

#include <string>

//#define AutomaticSwitch                       // Enable automatic switching
#define POSITION_THRESHOLD_AUTOMATIC_MODE 0.1f  // Threshold for position change

//#define ACTIVATE_DISTURBANCE_REJECTION_BY_TIME

#define targetSelect 0

#define UDE_THRESHOLD 0.95f

#define UDE_OBSERVER 1
#define LUENBERGER_OBSERVER 2
#define SUPERTWIST_OBSERVER 3
#define SLIDINGMODE_OBSERVER 4

// #define OBSERVER_TYPE SLIDINGMODE_OBSERVER
#define OBSERVER_TYPE SUPERTWIST_OBSERVER
// #define OBSERVER_TYPE UDE_OBSERVER
// #define OBSERVER_TYPE LUENBERGER_OBSERVER


/*********************
    LOG
*********************/
//#define POSE_LOG
//#define DT_LOG
#define MY_LAW_DEBUG_LOG
//#define ERROR_LOG
//#define PERTURBANCE_LOG 

/*********************
    CSV
*********************/
//#define SAVE_CONTROL_INPUT_CSV
#define CONTROL_INPUT_FILE_PATH_CSV "/home/nessy/Documents/SimDataCSV/ControlData.csv"

//#define SAVE_DEBUG_CSV
#define DEBUG_FILE_PATH_CSV         "/home/nessy/Documents/SimDataCSV/debugData.csv"

// #define SAVE_REAL_STATE_SPACE_CSV
#define TRANSLATION_FILE_PATH_CSV   "/home/nessy/Documents/SimDataCSV/RealStateSpace_trans.csv"
#define ROTATION_FILE_PATH_CSV      "/home/nessy/Documents/SimDataCSV/RealStateSpace_rot.csv"

// #define SAVE_STATE_ESTIMATION_CSV

#define DISTURBANCE_TRANSLATIONAL_FILE_PATH "/home/nessy/Documents/SimDataCSV/TranslationalEstimation.csv"
#define DISTURBANCE_ROTATIONAL_FILE_PATH    "/home/nessy/Documents/SimDataCSV/RotationalEstimation.csv"

// #define SAVE_UDE_DEBUG_CSV
#define SAVE_UDE_TRANS_DEBUG_FILE_PATH_CSV  "/home/nessy/Documents/SimDataCSV/DebugUDE_trans.csv"
#define SAVE_UDE_ROT_DEBUG_FILE_PATH_CSV    "/home/nessy/Documents/SimDataCSV/DebugUDE_rot.csv"

#define COMPARE_STATE__REAL_OBSERVER

#ifdef COMPARE_STATE__REAL_OBSERVER
    #ifndef SAVE_REAL_STATE_SPACE_CSV
        #define SAVE_REAL_STATE_SPACE_CSV
    #endif
    #ifndef SAVE_STATE_ESTIMATION_CSV
        #define SAVE_STATE_ESTIMATION_CSV
    #endif
#endif


#endif //PARAMSIM_H