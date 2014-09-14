// Note for the Hackathon: This was based off of the sample hello-myo project. We inserted our
// code into various segments of the program to get the results we desired. Not all of the
// code was written by us.

//Edited code by Dheeraj Nalluri and Brennen Ferguson

// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <ctime>
#include <windows.h>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>
#include <myo/libmyo.h>

double scaleMove(double weighted, double initial, double dz)
{
	double result = 0;
	if((weighted - initial) > 0)
	{
		result = (weighted - initial - dz);
		if(result > 12)
		{
			result = scaleMove(weighted - 18, initial, dz);
		}
	}
	else if((weighted - initial) < 0)
	{
		result = (weighted - initial + dz);
		if(result < -12)
		{
			result = scaleMove(weighted + 18, initial, dz);
		}
	}
	return result;
}

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : onArm(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose(myo::Pose::rest), calibrated(false)
    {
    	roll_i = pitch_i = yaw_i = 0;
    	whichArm = myo::Arm::armUnknown;
    	activeKeys = 0;
    }

    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        roll_i = 0;
        pitch_i = 0;
        yaw_i = 0;
        onArm = false;
        calibrated = false;
    }

    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;

        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(2.0f * (quat.w() * quat.y() - quat.z() * quat.x()));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

        // Convert the floating point angles in radians to a scale from 0 to 20.
        roll_w = static_cast<double>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<double>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<double>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);

        if(!calibrated)
        {
        	roll_i = roll_w;
        	pitch_i = pitch_w;
        	yaw_i = yaw_w;
        }
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        if (pose == myo::Pose::thumbToPinky && pitch_w > 2 && pitch_w < 16)
        {
        	if(!calibrated)
        	{
        		calibrated = true;
        		currentPose = pose;
        	}
        	else
        	{
        		//Right Mouse Down
        		INPUT Input = {0};
        		Input.type = INPUT_MOUSE;
        		Input.mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;

        		SendInput(1, &Input, sizeof(Input));

        		activeKeys |= (1<<1);
        	}
        }
        else if(!calibrated)
        {
        	return;
        }
        else if(pose == myo::Pose::fist)
        {
        	// Vibrate the Myo whenever we've detected that the user has made a fist.
        	myo->vibrate(myo::Myo::vibrationShort);
        	//Left Mouse Down
        	INPUT Input = {0};
        	Input.type = INPUT_MOUSE;
        	Input.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;

        	SendInput(1, &Input, sizeof(Input));

        	activeKeys |= (1<<0);
        }
        else if(pose == myo::Pose::rest)
        {
        	if (activeKeys & (1<<1))
        	{
        		//Right Mouse Up
        		INPUT Input = {0};
        		Input.type = INPUT_MOUSE;
        		Input.mi.dwFlags = MOUSEEVENTF_RIGHTUP;

        		SendInput(1, &Input, sizeof(Input));

        		activeKeys &= ~(1<<1);
        	}
        	if (activeKeys & (1<<0))
        	{
        		//Left Mouse Up
        		INPUT Input = {0};
        		Input.type = INPUT_MOUSE;
        		Input.mi.dwFlags = MOUSEEVENTF_LEFTUP;

        		SendInput(1, &Input, sizeof(Input));

        		activeKeys &= ~(1<<0);
        	}
        }

        currentPose = pose;
    }

    // onArmRecognized() is called whenever Myo has recognized a setup gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmRecognized(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        onArm = true;
        whichArm = arm;
    }

    // onArmLost() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmLost(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
        calibrated = false;
    }

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';

        // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
        /*std::cout << '[' << roll_w << ']'
                  << '[' << pitch_w << ']'
                  << '[' << yaw_w << ']';*/

        if (onArm) {
            // Print out the currently recognized pose and which arm Myo is being worn on.

            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            std::string poseString = currentPose.toString();

            std::cout << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                      << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        } else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << "[?]" << '[' << std::string(14, ' ') << ']';
        }

        std::cout << std::flush;
    }

    // These values are set by onArmRecognized() and onArmLost() above.
    bool onArm;
    myo::Arm whichArm;

    // These values are set by onOrientationData() and onPose() above.
    double roll_w, pitch_w, yaw_w;
    double roll_i, pitch_i, yaw_i;
    bool calibrated;
    int activeKeys;
    myo::Pose currentPose;
};

int main(int argc, char** argv)
{
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {

    // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
    // publishing your application. The Hub provides access to one or more Myos.
    myo::Hub hub("com.pickernickel.hackisu2014.MyoApp");


    std::cout << "Attempting to find a Myo..." << std::endl;

    // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
    // immediately.
    // waitForAnyMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
    // if that fails, the function will return a null pointer.
    myo::Myo* myo = hub.waitForMyo(10000);

    // If waitForAnyMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
    if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }

    // We've found a Myo.
    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector;

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

    //Stores target mouse position. Makes movement smoother and avoids truncation of speed.
    double mx = 0,my = 0;

    // Finally we enter our main loop.
    while (1) {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
        hub.run(1000/20);

        // After processing events, we call the print() member function we defined above to print out the values we've
        // obtained from any events that have occurred.
        collector.print();

        if(!collector.calibrated)
        {
        	continue;
        }
        else if(GetAsyncKeyState(VK_ESCAPE))
        {
        	collector.calibrated = false;
        }
        else if(collector.currentPose == myo::Pose::waveIn)
        {
           	//Scroll Down
           	INPUT Input = {0};
           	Input.type = INPUT_MOUSE;
           	Input.mi.dwFlags = MOUSEEVENTF_WHEEL;
           	Input.mi.mouseData = -40;
           	SendInput(1, &Input, sizeof(Input));
        }
        else if(collector.currentPose == myo::Pose::waveOut)
        {
            //Scroll Up
            INPUT Input = {0};
            Input.type = INPUT_MOUSE;
            Input.mi.dwFlags = MOUSEEVENTF_WHEEL;
            Input.mi.mouseData = 40;
            SendInput(1, &Input, sizeof(Input));
        }
        if(abs(collector.pitch_w - collector.pitch_i) > .5)
        {
        	//Move Mouse Y
        	INPUT Input = {0};
        	Input.type = INPUT_MOUSE;
        	Input.mi.dwFlags = MOUSEEVENTF_MOVE;
        	my += scaleMove(collector.pitch_w, collector.pitch_i, .5) * 4;
        	Input.mi.dy = floor(my);
        	my -= floor(my);
        	SendInput(1, &Input, sizeof(Input));
        }
        if(abs(collector.yaw_w - collector.yaw_i) > .25)
        {
        	//Move Mouse X
        	INPUT Input = {0};
        	Input.type = INPUT_MOUSE;
        	Input.mi.dwFlags = MOUSEEVENTF_MOVE;
        	mx += scaleMove(collector.yaw_w, collector.yaw_i, .25) * -8;
        	Input.mi.dx = floor(mx);
        	mx -= floor(mx);
        	SendInput(1, &Input, sizeof(Input));
        }

    }

    // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
