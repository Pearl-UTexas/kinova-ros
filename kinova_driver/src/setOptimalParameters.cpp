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

    // cout << endl << endl;
    // cout << "========================================================" << endl;
    // cout << "=====  Setting Gravity Parameters for Jaco2 arm   =====" << endl;
    // cout << "========================================================" << endl;

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
    int(*MySetGravityOptimalParameter)(float Command[GRAVITY_PARAM_SIZE]);
    int(*MySetGravityType)(GRAVITY_TYPE Type);


    //We load the library (Under Windows, use the function LoadLibrary)

    commandLayer_handle = dlopen("USBCommandLayerUbuntu.so",RTLD_NOW | RTLD_GLOBAL);

    //We load the functions from the library
    MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
    MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
    MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
    MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
    MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
    MySetGravityOptimalParameter = (int(*)(float Command[GRAVITY_PARAM_SIZE]))  dlsym(commandLayer_handle, "SetGravityOptimalZParam");
    MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");


    if ( (MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MySetGravityOptimalParameter == NULL) || (MySetGravityType == NULL))
    {
            cout << "* * *  Error during initialization!  * * *" << endl;
            cout << "Failure in getting: " << endl;

            if(MyInitAPI == NULL) cout << "MyInitAPI" << endl;
            if(MyCloseAPI == NULL) cout << "MyCloseAPI" << endl;
            if(MyGetAngularCommand == NULL) cout << "MyGetAngularCommand" << endl;
            if(MySetGravityOptimalParameter == NULL) cout << "MySetGravityOptimalParameter" << endl;
            if(MySetGravityType == NULL) cout << "MySetGravityType" << endl;

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

         cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ") (" << list[0].DeviceType << ")" << endl;
        
         cout << "Found a robot on the USB bus (" << list[1].SerialNumber << ") (" << list[1].DeviceType << ")" << endl;

        float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF] = { 0.00473463, -1.53159, -0.000864168, -0.0278615, -0.0176949, -0.653584, -0.00116566, -4.73937e-05, 0.000662961, -0.216932, -0.00324248, -0.00245626, 0.31686, 0.0953809, 0.176447, 0.454518, 0.092448, 0.107609, 0.201394};

        float OptimalzParam2[OPTIMAL_Z_PARAM_SIZE_7DOF] = {0.00371122, -1.51495, -0.00665266, -0.0195639, -0.0117675, -0.641425, 0.000537852, -0.00239706, -0.000355449, -0.212329, -0.00338902, 0.00291464, 0.278495, 0.108816, -0.142982, 0.520992, -0.0460984, 0.056346, -0.0303139};

        /* Setting right arm */
        if( *list[0].SerialNumber == 'PJ00900006153340003')
            MySetActiveDevice(list[0]);  //Setting right arm as the active device.
        else
            MySetActiveDevice(list[1]);  //Setting right arm as the active device.

        int resultComm;
        AngularPosition DataCommand;    // Get the angular command to test the communication with the robot
        resultComm = MyGetAngularCommand(DataCommand);
        // cout << "Communication result :" << resultComm << endl;
        // If the API is initialized and the communication with the robot is working
        if (result == 1 && resultComm == 1)
        {
            ROBOT_TYPE type = SPHERICAL_7DOF_SERVICE; 
            MySetGravityOptimalParameter(OptimalzParam);    // informs the robot on the new optimal gravity parameters
            MySetGravityType(OPTIMAL); //sets the gravity compensation mode to Optimal
    		cout << "Setting Optimal gravity Parameters for Right Arm" << endl;
	    }

        /* Setting Left arm */
        if( *list[0].SerialNumber == 'PJ00900006153340003')
            MySetActiveDevice(list[1]);  //Setting Left arm as the active device.
        else
            MySetActiveDevice(list[0]);  //Setting Left arm as the active device.

        resultComm = MyGetAngularCommand(DataCommand);
        // cout << "Communication result :" << resultComm << endl;
        // If the API is initialized and the communication with the robot is working
        if (result == 1 && resultComm == 1)
        {
            ROBOT_TYPE type = SPHERICAL_7DOF_SERVICE; 
            MySetGravityOptimalParameter(OptimalzParam2);    // informs the robot on the new optimal gravity parameters
            MySetGravityType(OPTIMAL); //sets the gravity compensation mode to Optimal
            cout << "Setting Optimal gravity Parameters for Left Arm" << endl;
        }

        result = (*MyCloseAPI)();
        programResult = 1;
    }
    dlclose(commandLayer_handle);
    return programResult;
}

