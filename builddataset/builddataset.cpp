/*
Adept MobileRobots Robotics Interface for Applications(ARIA)
Copyright(C) 2004, 2005 ActivMedia Robotics LLC
Copyright(C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
Copyright(C) 2011, 2012, 2013 Adept Technology

This program is free software; you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 - 1307  USA

If you wish to redistribute ARIA under different terms, contact
Adept MobileRobots for information about a commercial version of ARIA at
robots@mobilerobots.com or
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1 - 603 - 881 - 7960
*/

#include "Aria.h"

/*
* Modify this program.
* For help using the ARIA library, see the ARIA README:
*   file://C:/Program%20Files/MobileRobots/Aria/README.txt
* and the ARIA API Reference Manual at:
*   file://C:/Program%20Files/MobileRobots/Aria/docs/index.html
* and the example programs at:
*   file://C:/Program%20Files/MobileRobots/Aria/examples
*/

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if (!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "builddataset: Could not connect to the robot.");
    if (parser.checkHelpAndWarnUnparsed())
    {
      // -help not given
      Aria::logOptions();
      Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1); // exit with error code
  }

  ArLog::log(ArLog::Normal, "builddataset: Connected to robot.");

  robot.enableMotors();

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);

  // Print out some data from the SIP.  We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  robot.lock();
  ArLog::log(ArLog::Normal, "builddataset: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "builddataset: Sleeping for 3 seconds...");
  ArUtil::sleep(3000);


  ArLog::log(ArLog::Normal, "simpleConnect: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "builddataset: Exiting.");
  Aria::exit(0);
  return 0;
}
