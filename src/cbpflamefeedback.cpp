#include <iostream>
#include <cstdio>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>
#include <vector>
#include <typeinfo>
#include <bitset>

#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <wiringPi.h>

#include "config.h"

using namespace std;
using namespace std::chrono;

// Some globals for ease of use and communication between threads
bool debug; // Will be se from config file
bool igniterActive = false;
int cbpGasGPIO = 0;
int fbGasGPIO = 0;
int fbIgniterGPIO = 0;
int fbFlameSensorGPIO = 0;
int igniterPulseDuration = 0;
int igniterTimeBetweenPulses = 0;
int noFlameShutdownTime = 0;

// This is the signal handler to turn off GPIOs when CTRL-C is pressed
void safeShutdown(int sig) {
   digitalWrite(fbGasGPIO, 0);
   digitalWrite(fbIgniterGPIO, 0);
   if ( debug ) cout << "Caught CTRL-C. Shutting down GPIOs and exiting.\n";
   exit(0);
}

// This thread runs all the time but does nothing but sleep unless the
// global variable "igniterActive" is true. When active it will cycle
// the igniter on and off based on settings from the config file.
void igniterThread() {
   bool igniterWasActive = false;

   while ( true ) {
      while ( igniterActive ) {
         if ( !igniterWasActive ) {
            igniterWasActive = true;
            if ( debug ) cout << "Turning on igniter.\n" << std::flush;
         }
         digitalWrite(fbIgniterGPIO, 1);
         this_thread::sleep_for(milliseconds(igniterPulseDuration));
         digitalWrite(fbIgniterGPIO, 0);
         this_thread::sleep_for(milliseconds(igniterTimeBetweenPulses));
      }
      if ( igniterWasActive ) {
         igniterWasActive = false;
         if ( debug ) cout << "Turning off igniter.\n" << std::flush;
      }
      this_thread::sleep_for(milliseconds(100));
   }
}

// This thread runs all the time and watches for changes to the state
// of the GPIO used by CraftBeerPi. When that GPIO is high the thread
// will attempt to ignite the flame, and monitor flame status.
void monitorThread() {
   // burnerEnabled is true when the system is trying to light the
   // burner and keep it lit. burnerEnabled is set to false if the
   // burner fails to ignite either initially or if the flame goes
   // out and reignition fails.
   bool burnerEnabled = false;
   // ignitionFailed is set to true when the igniter is on for the
   // time specified in the config file. When it is set to true
   // a clock point is set in the future based on the config file
   // setting from noFlameShutdownTime.
   bool ignitionFailed = false;
   // This variable only exists so that a proper debug message can
   // be printed if the CraftBeerPi gas control GPIO goes low while
   // the loop is waiting after an ignition/reignition failure.
   bool cbpGasEnabled = false;

   steady_clock::time_point ignitionRetryClock = steady_clock::now();
   steady_clock::time_point ignitionFailedClock = steady_clock::now();
   if ( debug ) cout << "Starting monitoring thread...\n" << std::flush;
   while ( true ) {
      while ( digitalRead(cbpGasGPIO) == 1 ) {
         // If we are in an ignition failed state (ignitionFailed is true and
         // current time is less than the ignitionRetryClock time) then skip
         // this iteration of the loop.
         if ( !(ignitionFailed && (steady_clock::now() < ignitionRetryClock)) ) {
            // Set cbpGasEnabled to true if this is our first time through after
            // the CraftBeerPi gas GPIO goes high
            if ( !cbpGasEnabled ) cbpGasEnabled = true;
            // If the burner isn't enabled we need to set all the "turn burner on"
            // initial states and registering the clock point for monitoring ignition failure
            if ( !burnerEnabled ) {
               if ( debug ) cout << "Turning on gas and starting igniter\n" << std::flush;
               ignitionFailed = false;
               burnerEnabled = true;
               digitalWrite(fbGasGPIO, 1);
               igniterActive = true;
               ignitionFailedClock = steady_clock::now() + milliseconds(noFlameShutdownTime);
            }
            // If the flame sensor is high we turn off the igniter and reset ignitionFailed
            if ( digitalRead(fbFlameSensorGPIO) == 1 ) {
               if ( igniterActive ) {
                  igniterActive = false;
                  if ( debug ) cout << "Flame detected! Entering stable burning state.\n" << std::flush;
               }
               ignitionFailed = false;
            } else {
               // If the igniter is active while no flame is seen and we've passed the ignition
               // failed clock, turn off for the time configured by noFlameShutdownTime
               if ( igniterActive && (steady_clock::now() >= ignitionFailedClock) ) {
                  if ( debug ) cout << "Ignition Failed!!! Trying again in " << noFlameShutdownTime << " milliseconds.\n" << std::flush;
                  ignitionFailed = true;
                  igniterActive = false; // stop the igniter
                  digitalWrite(fbGasGPIO, 0); // turn off the gas
                  burnerEnabled = false;
                  ignitionRetryClock = steady_clock::now() + milliseconds(noFlameShutdownTime);
               // If the igniter isn't active and there's no flame while we are past the
               // ignitionFailedClock then the flame has gone out and we need to reignite.
               } else if ( !igniterActive && (steady_clock::now() > ignitionFailedClock) ) {
                  if ( debug ) cout << "Flame went out!!! Trying to reignite flame.\n" << std::flush;
                  igniterActive = true;
                  ignitionFailedClock = steady_clock::now() + milliseconds(noFlameShutdownTime);
               }
            }
         }
         this_thread::sleep_for(milliseconds(100));
      }
      if ( cbpGasEnabled ) {
         if ( debug ) cout << "BrewPi turned off gas.\n" << std::flush;
         cbpGasEnabled = false;
      }
      if ( burnerEnabled ) {
         burnerEnabled = false;
         ignitionFailed = false;
         igniterActive = false;
         digitalWrite(fbGasGPIO, 0);
      }
      this_thread::sleep_for(milliseconds(100));
   }
}

int main (int argc, const char* argv[], char* envp[]) {
   // Trap CTRL-C so we can cleanly turn GPIOs off before exiting
   signal(SIGINT, safeShutdown);

   //  Time to read in the config file.
   //  A config file may be specified on the command line
   //  as the first argument. If no config file is specified
   //  the program will try to use the file "/etc/cbpflamefeedback.config".
   //  If that file does not exist the program will exit.
   string settingsFile;
   if ( argc == 2 ) {
      settingsFile = argv[1];
      if ( FILE *file = fopen(settingsFile.c_str(), "r")) {
         //cout << "Found config file \"" << settingsFile << "\"\n";
      } else {
         cout << "Error: Config file \"" << settingsFile << "\" not found!\n";
         exit(1);
      }
   } else {
      settingsFile = "/etc/cbpflamefeedback.config";
      if ( FILE *file = fopen(settingsFile.c_str(), "r")) {} else {
         cout << "Error: Default config file \"" << settingsFile << "\" not found!\n";
         exit(1);
      }
   }

   // Set the value of some globals based on settings in the config file
   Config config(settingsFile, envp);
   debug = config.pBool("debug");
   cbpGasGPIO = config.pInt("CBPGasGPIO");
   fbGasGPIO = config.pInt("FBGasGPIO");
   fbIgniterGPIO = config.pInt("FBIgniterGPIO");
   fbFlameSensorGPIO = config.pInt("FBFlameSensorGPIO");
   igniterPulseDuration = config.pInt("igniterPulseDuration");
   igniterTimeBetweenPulses = config.pInt("igniterTimeBetweenPulses");
   noFlameShutdownTime = config.pInt("noFlameShutdownTime");
   if ( debug ) {
      cout << "Using config file \"" << settingsFile << "\"\n";
      cout << "CBPGasGPIO: " << cbpGasGPIO << "\n";
      cout << "FBGasGPIO: " << fbGasGPIO << "\n";
      cout << "FBIgniterGPIO: " << fbIgniterGPIO << "\n";
      cout << "FBFlameSensorGPIO: " << fbFlameSensorGPIO << "\n";
      cout << "igniterPulseDuration: " << igniterPulseDuration << "\n";
      cout << "igniterTimeBetweenPulses: " << igniterTimeBetweenPulses << "\n";
      cout << "noFlameShutdownTime: " << noFlameShutdownTime << "\n";
      cout << "\n";
   }

   // Setup our pins
   wiringPiSetup();
   pinMode(fbGasGPIO, OUTPUT);
   pinMode(fbIgniterGPIO, OUTPUT);
   pinMode(fbFlameSensorGPIO, INPUT);
   digitalWrite(fbGasGPIO, 0);
   digitalWrite(fbIgniterGPIO, 0);

   // Start the threads
   thread igniterT(igniterThread);
   thread monitorT(monitorThread);
   igniterT.join();
   monitorT.join();

   return 0;
}
