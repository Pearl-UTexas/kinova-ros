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

        float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF] = { 0.0220081, -1.53608, -0.00392239, -0.0259511, -0.0234186, -0.64607, -0.00157998, 0.00082352, 0.00391767, -0.215765, -0.00372691, -0.00416097, 0.290655, -0.147011, -0.0229545, 0.195504, 0.0157291, 0.144705, -0.221752};

        float OptimalzParam2[OPTIMAL_Z_PARAM_SIZE_7DOF] = {-0.00608284, -1.52057, -0.00408724, -0.0249677, -0.0153228, -0.651196, 0.000869198, -0.00018281, 0.00109552, -0.212481, -9.0997e-05, 0.000194872, 0.441752, -0.0661224, 0.222764, 0.485318, 0.0817648, 0.0880216, -0.0456601};

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

