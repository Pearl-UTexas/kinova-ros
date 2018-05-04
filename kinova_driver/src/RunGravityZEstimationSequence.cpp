#include <ros/ros.h>
#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <kinova/KinovaTypes.h>
#include "kinova/Kinova.API.USBCommandLayerUbuntu.h"
#include "kinova/Kinova.API.USBCommLayerUbuntu.h"
// #include "kinova_driver/kinova_api.h"
#include <unistd.h>
#include <typeinfo>

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "loadArmParameters");
    ros::NodeHandle n;

    cout << endl << endl;
    cout << "==========================================================" << endl;
    cout << "=== Estimating Gravity Parameters for Jaco2 7-dof arm  ===" << endl;
    cout << "==========================================================" << endl;

    cout << "!!!! ! W A R N I N G !!!!!" << endl;
    cout << "PLEASE MAKE SURE YOU PROVIDE ENOUGH SPACE FOR RUNNING ESTIMATION SEQUENCE" << endl;

    int result;
    int programResult = 0;
    int devicesCount;

    //Handle for the library's command layer.
    void * commandLayer_handle;

    //Function pointers to the functions we need
    int(*MyInitAPI)();
    int(*MyCloseAPI)();
    int(*MyGetAngularCommand)(AngularPosition &);
    int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int(*MySetActiveDevice)(KinovaDevice device);
    int(*MyRunGravityZEstimationSequence7DOF)(ROBOT_TYPE type, float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF]);
    int(*MySetGravityOptimalParameter)(float Command[GRAVITY_PARAM_SIZE]);
    int(*MySetGravityType)(GRAVITY_TYPE Type);

    int(*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &Response);

    //We load the library (Under Windows, use the function LoadLibrary)

    commandLayer_handle = dlopen("USBCommandLayerUbuntu.so",RTLD_NOW | RTLD_GLOBAL);

    //We load the functions from the library
    MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
    MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
    MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
    MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
    MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");

    MyRunGravityZEstimationSequence7DOF = (int(*)(ROBOT_TYPE type, float OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence7DOF");

    MySetGravityOptimalParameter = (int(*)(float Command[GRAVITY_PARAM_SIZE]))  dlsym(commandLayer_handle, "SetGravityOptimalZParam");
    MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");


    if ( (MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyRunGravityZEstimationSequence7DOF == NULL) ||(MySetGravityOptimalParameter == NULL) || (MySetGravityType == NULL))
    {
            cout << "* * *  Error during initialization!  * * *" << endl;
            cout << "Failure in getting: " << endl;

            if(MyInitAPI == NULL) cout << "MyInitAPI failed" << endl;
            if(MyCloseAPI == NULL) cout << "MyCloseAPI failed" << endl;
            if(MyGetAngularCommand == NULL) cout << "MyGetAngularCommand failed" << endl;
            if(MyRunGravityZEstimationSequence7DOF == NULL) cout << "MyRunGravityZEstimationSequence failed" << endl;
            if(MySetGravityOptimalParameter == NULL) cout << "MySetGravityOptimalParameter failed" << endl;
            if(MySetGravityType == NULL) cout << "MySetGravityType failed" << endl;

            programResult = 0;
    }
    else
    {
        cout << "Initialization completed." << endl << endl;
        result = (*MyInitAPI)();

        KinovaDevice list[MAX_KINOVA_DEVICE];

        devicesCount = MyGetDevices(list, result);
        if (devicesCount == 0)
        {
         cout << "\n WARNING : The robot is off or is not in the loop!" << endl;
         return 0;
        }

         cout << "Found robot on the USB bus (" << list[0].SerialNumber << ") (" << list[0].DeviceType << ")" << endl;

        int resultComm;
        AngularPosition DataCommand;    // Get the angular command to test the communication with the robot
        resultComm = MyGetAngularCommand(DataCommand);
        // cout << "Communication result :" << resultComm << endl;
        // If the API is initialized and the communication with the robot is working
        if (result == 1 && resultComm == 1)
        {
            ROBOT_TYPE type = SPHERICAL_7DOF_SERVICE; 
            float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF];

            // Run identification sequence
            cout << "Running gravity parameters estimation trajectory..." << endl;  
            MyRunGravityZEstimationSequence7DOF (type, OptimalzParam);

            MySetGravityOptimalParameter(OptimalzParam);    // informs the robot on the new optimal gravity parameters
            MySetGravityType(OPTIMAL); //sets the gravity compensation mode to Optimal
            cout << "Setting Optimal gravity Parameters for the Arm" << endl;
        }

        cout << endl << "C L O S I N G A P I" << endl;
        result = (*MyCloseAPI)();
        programResult = 1;
    }
    dlclose(commandLayer_handle);
    return programResult;
}

