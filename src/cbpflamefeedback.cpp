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

void igniterThread() {
   while ( true ) {
      while ( igniterActive ) {
         digitalWrite(fbIgniterGPIO, 1);
         this_thread::sleep_for(milliseconds(igniterPulseDuration));
         digitalWrite(fbIgniterGPIO, 0);
         this_thread::sleep_for(milliseconds(igniterTimeBetweenPulses));
      }
      this_thread::sleep_for(milliseconds(100));
   }
}

void monitorThread() {
   bool burnerEnabled = false;
   bool ignitionFailed = false;
   bool cbpGasEnabled = false;
   steady_clock::time_point ignitionRetryClock = steady_clock::now();
   steady_clock::time_point ignitionFailedClock = steady_clock::now();
   if ( debug ) cout << "Starting monitoring thread...\n" << std::flush;
   while ( true ) {
      while ( digitalRead(cbpGasGPIO) == 1 ) {
         this_thread::sleep_for(milliseconds(100));
         if ( !cbpGasEnabled ) cbpGasEnabled = true;
         if ( !burnerEnabled ) {
            if ( ignitionFailed && (steady_clock::now() < ignitionRetryClock) ) continue;
            if ( debug ) cout << "Turning on gas and starting igniter\n" << std::flush;
            ignitionFailed = false;
            burnerEnabled = true;
            digitalWrite(fbGasGPIO, 1);
            igniterActive = true;
            ignitionFailedClock = steady_clock::now() + milliseconds(noFlameShutdownTime);
         }
         if ( digitalRead(fbFlameSensorGPIO) == 1 ) {
            igniterActive = false;
            ignitionFailed = false;
         } else {
            if ( igniterActive && (steady_clock::now() >= ignitionFailedClock) ) {
               if ( debug ) cout << "Ignition Failed!!! Trying again in " << noFlameShutdownTime << " milliseconds.\n" << std::flush;
               ignitionFailed = true;
               igniterActive = false; // stop the igniter
               digitalWrite(fbGasGPIO, 0); // turn off the gas
               burnerEnabled = false;
               ignitionRetryClock = steady_clock::now() + milliseconds(noFlameShutdownTime);
            } else if ( !igniterActive && (steady_clock::now() > ignitionFailedClock) ) {
               if ( debug ) cout << "Flame went out!!! Trying to reignite flame.\n" << std::flush;
               igniterActive = true;
               ignitionFailedClock = steady_clock::now() + milliseconds(noFlameShutdownTime);
            }
         }
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
      settingsFile = "/etc/fbflamefeedback.config";
      if ( FILE *file = fopen(settingsFile.c_str(), "r")) {} else {
         cout << "Error: Default config file \"" << settingsFile << "\" not found!\n";
         exit(1);
      }
   }

   Config config(settingsFile, envp);
   debug = config.pBool("debug");
   if ( debug ) {
      cout << "Using config file \"" << settingsFile << "\"\n";
      cout << "CBPGasGPIO: " << config.pInt("CBPGasGPIO") << "\n";
      cout << "FBGasGPIO: " << config.pInt("FBGasGPIO") << "\n";
      cout << "FBIgniterGPIO: " << config.pInt("FBIgniterGPIO") << "\n";
      cout << "FBFlameSensorGPIO: " << config.pInt("FBFlameSensorGPIO") << "\n";
      cout << "igniterPulseDuration: " << config.pInt("igniterPulseDuration") << "\n";
      cout << "igniterTimeBetweenPulses: " << config.pInt("igniterTimeBetweenPulses") << "\n";
      cout << "noFlameShutdownTime: " << config.pInt("noFlameShutdownTime") << "\n";
      cout << "\n";
   }
   cbpGasGPIO = config.pInt("CBPGasGPIO");
   fbGasGPIO = config.pInt("FBGasGPIO");
   fbIgniterGPIO = config.pInt("FBIgniterGPIO");
   fbFlameSensorGPIO = config.pInt("FBFlameSensorGPIO");
   igniterPulseDuration = config.pInt("igniterPulseDuration");
   igniterTimeBetweenPulses = config.pInt("igniterTimeBetweenPulses");
   noFlameShutdownTime = config.pInt("noFlameShutdownTime");

   // Setup our pins
   wiringPiSetup();
   pinMode(fbGasGPIO, OUTPUT);
   pinMode(fbIgniterGPIO, OUTPUT);
   pinMode(fbFlameSensorGPIO, INPUT);
   digitalWrite(fbGasGPIO, 0);
   digitalWrite(fbIgniterGPIO, 0);

   thread igniterT(igniterThread);
   thread monitorT(monitorThread);
   igniterT.join();
   monitorT.join();

   return 0;
}
