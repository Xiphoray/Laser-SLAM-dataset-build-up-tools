#include <iostream>
#include "Aria.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include<fstream>
#include<typeinfo>
#include<windows.h>

#include <math.h>
#include <string>
#include <thread>
#include <stdio.h>
#include <stdlib.h>

#include "CImg.h"
#include "low.h"
using namespace cimg_library;
using namespace std;


//
// Globals
//


// The current commands being given to the robot for movement. 
// Not used when the robot is reading data from a log file.
double RotationSpeed, TranslationSpeed;

bool CarActionFlag = true;
bool IsCarAction = false;

// 新一轮收集
bool isnewround = false;
// 数据集序列
int DataNumber = 0;
// 修改结果全地图地图即时显示
CImg<unsigned char> Allmapimgwait;
//激光数据图
CImg<unsigned char> Laserimg;
unsigned char black[] = { 0 };
unsigned char white[] = { 255 };
//
//
// InitializeRobot
//
// Calls the routines in 'ThisRobot.c' to initialize the necessary hardware and software.
// Each call has an opportunity to return a value of -1, which indicates that the initialization of that
// part of the robot has failed. In that event, Initialize robot itself will return a general error of -1.
//
//
int InitializeRobot(ArArgumentParser *parser, ArRobot *robot, ArRobotConnector *robotConnector, ArLaserConnector *laserConnector, int argc, char *argv[]) {
	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.

	if (!(*robotConnector).connectRobot())
	{
		ArLog::log(ArLog::Terse, "lasersExample: Could not connect to the robot.");
		if ((*parser).checkHelpAndWarnUnparsed())
		{
			fprintf(stderr, "Start up initialization of the robot has failed.\n");
			// -help not given
			Aria::logOptions();
			return -1;
		}
	}
	fprintf(stderr, "\n  ");
	if (!Aria::parseArgs())
	{
		Aria::logOptions();
		return -1;
	}
	fprintf(stderr, "\n  ");
	ArLog::log(ArLog::Normal, "Connected to robot.");
	// Start the robot processing cycle running in the background.
	// True parameter means that if the connection is lost, then the 
	// run loop ends.

	fprintf(stderr, "\nConnecting Laser.\n");
	(*robot).runAsync(true);

	// Connect to laser(s) as defined in parameter files.
	// (Some flags are available as arguments to connectLasers() to control error behavior and to control which lasers are put in the list of lasers stored by ArRobot. See docs for details.)
	if (!(*laserConnector).connectLasers())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured lasers. Exiting.");
		return -1;
	}

	// Allow some time to read laser data
	ArUtil::sleep(500);
	ArLog::log(ArLog::Normal, "Connected to all lasers.");
	return 0;
}


static string  getCurrentTimeStr()
{
	time_t t = time(NULL);
	char ch[64] = { 0 };
	strftime(ch, sizeof(ch) - 1, "%Y%m%d%H%M%S", localtime(&t));     //年-月-日 时-分-秒
	return ch;
}



//
//
// SlamThread
//
// Slam线程
//
//
class SlamThread : public ArASyncTask
{
	ArCondition SlamCondition;
	ArMutex myMutex;
	ArRobot *robot;

public:
	/* Construtor. Initialize counter. */
	SlamThread()
	{
		SlamCondition.setLogName("SlamThreadCondition");
	}
	/* This method is called in the new thread when launched. The void* parameter
	 * and return value are platform implementation-specific and can be ignored.
	 * This method will run in a loop, incrementing the counter each second, but
	 * locking the mutex to prevent conflicting access by other threads.
	 * If it reaches a value divisible by ten, signal our condition variable.
	 */
	void* runThread(void*)
	{
		
		myMutex.lock();
		char sysCall[128];
		myMutex.unlock();
		InitLowSlam(robot);
		ArLog::log(ArLog::Normal, "InitLowSlam finished");
		int rx, ry, xtest, ytest, X, Y, rX, rY;
		double cosrad , sinrad ;
		CImg<unsigned char> allmaprotation;
		CImg<unsigned char> allmaptranslation;
		// Run until the thread is requested to end by another thread.
		while (this->getRunningWithLock())
		{
			myMutex.lock();

			if (CarActionFlag) {
				(*robot).stop();
				CarActionFlag = false;
				LowSlam(robot);
				allmaprotation.assign(Allmapimgwait);
				cosrad = (double)std::cos(-odometry.theta);
				sinrad = (double)std::sin(-odometry.theta);
				
				cimg_forXY(allmaprotation, x, y) {//+ odometry.b * (double)std::cos(-odometry.lastsumtheta)- odometry.b * (double)std::sin(-odometry.lastsumtheta)
					const double
						cX = x - ALLMAP_WIDTH/2 , cY = y - ALLMAP_HEIGHT/2 ,
						fX = ALLMAP_WIDTH / 2 + cX * cosrad - cY * sinrad,
						fY = ALLMAP_HEIGHT / 2 + cX * sinrad + cY * cosrad;
					X = cimg::mod((int)fX, allmaprotation.width());
					Y = cimg::mod((int)fY, allmaprotation.height());
					ytest = (int)(odometry.x * (double)std::cos(odometry.lastsumtheta) + odometry.y * (double)std::sin(odometry.lastsumtheta));
					xtest = (int)(-odometry.x * (double)std::sin(odometry.lastsumtheta) + odometry.y * (double)std::cos(odometry.lastsumtheta));
					if ((y + ytest >= 0) && (y + ytest < allmaprotation.height()))
						ry = y + ytest;
					else
						ry = y;

					if ((x + xtest < allmaprotation.width()) && (x + xtest >= 0))
						rx = x + xtest;
					else
						rx = x;
					cimg_forC(allmaprotation, c) Allmapimgwait(rx, ry, c) = allmaprotation(X, Y, c);
				}
				Laserimg.fill(255);
				cimg_forXY(Laserimg, x, y) {
					if (rawmap[x][y] == 0) {
						Laserimg(x, y, 0) = 0;
						Allmapimgwait((int)((ALLMAP_WIDTH - MAP_WIDTH) / 2) + x, (int)((ALLMAP_HEIGHT - MAP_HEIGHT) / 2) + y, 0) = 0;
					}
					
				}
			}
			else {
				CarActionFlag = true;
				string input;
				int h, w, h1, w1;
				while (1) {
					cin >> input;
					if (input == "q") {
						break;
					}
					else if (input == "l") {

						cin >> input;
						w = atoi(input.c_str());
						cin >> input;
						h = atoi(input.c_str());
						cin >> input;
						w1 = atoi(input.c_str());
						cin >> input;
						h1 = atoi(input.c_str());
						Allmapimgwait.draw_line(w, h, w1, h1, black);
					}
					else {
						w = atoi(input.c_str());
						cin >> input;
						h = atoi(input.c_str());
						if (Allmapimgwait(w, h, 0) == 255)
							Allmapimgwait(w, h, 0) = 0;
						else
							Allmapimgwait(w, h, 0) = 255;

					}
					ArLog::log(ArLog::Normal, "---");
				}

				sprintf(sysCall, "result/Allmap-%s.bmp", getCurrentTimeStr().c_str());
				Allmapimgwait.save(sysCall);
				int i;
				ofstream fout(csv_addr, ios::app);
				fout << DataNumber;
				fout << SWPARATOR_BIG;
				for (i = 0; i < SENSE_NUMBER; i++) {
					fout << sense[i].distance;
					fout << SWPARATOR_BIG;
				}
				fout << odometry.x << SWPARATOR_BIG;
				fout << odometry.y << SWPARATOR_BIG;
				fout << odometry.theta << SWPARATOR_BIG;
				fout << odometry.lastsumtheta << SWPARATOR_BIG;
				fout << sysCall << endl;
				fout.close();
				ArLog::log(ArLog::Normal, "%d turn is finished", DataNumber);

				DataNumber++;
				
				(*robot).clearDirectMotion();
				ArUtil::sleep(3000);
			}
			myMutex.unlock();
			
		}
		ArLog::log(ArLog::Normal, "Slam thread: requested stop running, ending thread.");
		return NULL;
	}
	/* Other threads can call this to wait for a condition eventually
	 * signalled by this thread. (So note that in this example program, this
	 * function is not executed within "Example thread", but is executed in the main thread.)
	 */
	void waitOnCondition()
	{
		SlamCondition.wait();
		ArLog::log(ArLog::Normal, " %s ArCondition object was signalled, done waiting for it.", SlamCondition.getLogName());
	}
	/* Get the counter. Not threadsafe, you must lock the mutex during access. */
	ArRobot *getRobot() { return robot; }
	/* Set the countner. Not threadsafe, you must lock the mutex during access. */
	void setRobot(ArRobot *ctr) { robot = ctr; }
	/* Lock the mutex object.  */
	void lockMutex() { myMutex.lock(); }
	/* Unlock the mutex object. */
	void unlockMutex() { myMutex.unlock(); }
};



//
//
// AllmapimgDisplayThread
//
// 地图修改显示线程
//
//
class AllmapimgDisplayThread : public ArASyncTask
{
	ArCondition AllmapDisplayCondition;
	ArMutex myMutex;

public:
	/* Construtor. Initialize counter. */
	AllmapimgDisplayThread()
	{
		AllmapDisplayCondition.setLogName("AllmapimgDisplayThreadCondition");
	}
	void* runThread(void*)
	{

		int argc = 0;
		char **argv = NULL;
		CImgDisplay Allmap_disp;
		// Run until the thread is requested to end by another thread.
		while (this->getRunningWithLock())
		{
			Allmapimgwait.display(Allmap_disp, true);

		}
		ArLog::log(ArLog::Normal, "AllmapimgDisplay thread: requested stop running, ending thread.");
		return NULL;
	}
	/* Other threads can call this to wait for a condition eventually
	 * signalled by this thread. (So note that in this example program, this
	 * function is not executed within "Example thread", but is executed in the main thread.)
	 */
	void waitOnCondition()
	{
		AllmapDisplayCondition.wait();
		ArLog::log(ArLog::Normal, " %s ArCondition object was signalled, done waiting for it.", AllmapDisplayCondition.getLogName());
	}

	/* Lock the mutex object.  */
	void lockMutex() { myMutex.lock(); }
	/* Unlock the mutex object. */
	void unlockMutex() { myMutex.unlock(); }
};



//
//
// LaserDisplayThread
//
// 激光数据图显示线程
//
//
class LaserDisplayThread : public ArASyncTask
{
	ArCondition LaserDisplayCondition;
	ArMutex myMutex;

public:
	/* Construtor. Initialize counter. */
	LaserDisplayThread()
	{
		LaserDisplayCondition.setLogName("LaserDisplayThread");
	}
	void* runThread(void*)
	{

		int argc = 0;
		char **argv = NULL;
		CImgDisplay Laser_disp;
		// Run until the thread is requested to end by another thread.
		while (this->getRunningWithLock())
		{
			Laserimg.display(Laser_disp, true);

		}
		ArLog::log(ArLog::Normal, "LaserDisplay thread: requested stop running, ending thread.");
		return NULL;
	}
	/* Other threads can call this to wait for a condition eventually
	 * signalled by this thread. (So note that in this example program, this
	 * function is not executed within "Example thread", but is executed in the main thread.)
	 */
	void waitOnCondition()
	{
		LaserDisplayCondition.wait();
		ArLog::log(ArLog::Normal, " %s ArCondition object was signalled, done waiting for it.", LaserDisplayCondition.getLogName());
	}

	/* Lock the mutex object.  */
	void lockMutex() { myMutex.lock(); }
	/* Unlock the mutex object. */
	void unlockMutex() { myMutex.unlock(); }
};




//
//
// ImgThread
//
// 地图图像显示线程
//
//
bool imgflash = false;
class ImgThread : public ArASyncTask
{
	ArCondition ImgCondition;
	ArMutex myMutex;

public:
	/* Construtor. Initialize counter. */
	ImgThread()
	{
		ImgCondition.setLogName("ImgThreadCondition");
	}
	/* This method is called in the new thread when launched. The void* parameter
	 * and return value are platform implementation-specific and can be ignored.
	 * This method will run in a loop, incrementing the counter each second, but
	 * locking the mutex to prevent conflicting access by other threads.
	 * If it reaches a value divisible by ten, signal our condition variable.
	 */
	void* runThread(void*)
	{

		int argc = 0;
		char **argv = NULL;
		Showmap showmap;
		// Run until the thread is requested to end by another thread.
		while (this->getRunningWithLock())
		{
			if (imgflash) {
				showmap.Todisplay();
				imgflash = false;
				fprintf(stderr, "Map dumped to file\n");
			}
			
			cimg::wait(2000);

		}
		ArLog::log(ArLog::Normal, "Img thread: requested stop running, ending thread.");
		return NULL;
	}
	/* Other threads can call this to wait for a condition eventually
	 * signalled by this thread. (So note that in this example program, this
	 * function is not executed within "Example thread", but is executed in the main thread.)
	 */
	void waitOnCondition()
	{
		ImgCondition.wait();
		ArLog::log(ArLog::Normal, " %s ArCondition object was signalled, done waiting for it.", ImgCondition.getLogName());
	}

	/* Lock the mutex object.  */
	void lockMutex() { myMutex.lock(); }
	/* Unlock the mutex object. */
	void unlockMutex() { myMutex.unlock(); }
};


//
//
// 启动
//
//
int main(int argc, char **argv)
{
	char sysCall[128];
	DataNumber = 0;
	Allmapimgwait.assign(ALLMAP_HEIGHT, ALLMAP_WIDTH, 1, 1);
	cimg_forXY(Allmapimgwait, x, y) {
		Allmapimgwait(x, y, 0) = 255;	
	}
	Laserimg.assign(MAP_HEIGHT, MAP_WIDTH, 1, 1);


	Aria::init();
	ArRobot robot;

	SlamThread slamThread;
	//ImgThread imgThread;
	LaserDisplayThread laserdisplaythread;
	AllmapimgDisplayThread allmapimgdisplaythread;

	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	ArLaserConnector laserConnector(&parser, &robot, &robotConnector);



	fprintf(stderr, "********** Localization Example *************\n");
	if (InitializeRobot(&parser, &robot, &robotConnector, &laserConnector, argc, argv) == -1) {
		Aria::exit(1);
		return -1;
	}

	fprintf(stderr, "********** World Initialization ***********\n");


	if (robot.isConnected())
	{
		slamThread.setRobot(&robot);
	}

	// Trigger argument parsing
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}

	robot.runAsync(true);
	IsCarAction = true;
	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();

	//robot.comInt(ArCommands::SOUNDTOG, 0);
	// add a set of actions that combine together to effect the wander behavior
	ArActionStallRecover recover;
	ArActionBumpers bumpers;
	ArActionAvoidFront avoidFrontNear("Avoid Front Near", 112, 0);
	ArActionAvoidFront avoidFrontFar;
	ArActionConstantVelocity constantVelocity("Constant Velocity", 500);
	robot.addAction(&recover, 40);\
	robot.addAction(&bumpers, 37);
	robot.addAction(&avoidFrontNear, 25);
	robot.addAction(&avoidFrontFar, 25);
	robot.addAction(&constantVelocity, 12);

	
	
	slamThread.runAsync();
	laserdisplaythread.runAsync();
	allmapimgdisplaythread.runAsync();

	//imgThread.runAsync();
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	Aria::exit(0);
	return 0;
}

