#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <algorithm>
#include <string>
#include <iostream>
#include <math.h>
#include <unistd.h>

// Convert degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

#define WAIT_PERSON       0.5 //Places where people start walking
#define WALKING_PERSON    1.0 //Places where people start walking
#define CHECK_POINT1      2.0 //1st Section
#define FRONT_OF_ELEVATOR 2.5 //Waiting area of the elevator
#define ELEVATOR          3.0 //Waiting area of the elevator
#define ELEVATOR_CLEAR    4.0 //2nd Section
#define CROWD             5.0 //Waiting area after you exit the crowd
#define CROWD_END         6.0 //3rd Section
#define	GRASP			  7.0
#define END              99.0 //End location

struct coordinate{
	double x[255];
	double y[255];
	double z[255];
	double flag[255];
};

char robotName[] = "man_000";
char avatarName[] = "robot_000";
int graspNum = 0;

using namespace std;

class MyController : public Controller {
public:
	void onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void onRecvMsg(RecvMsgEvent &evt);
	void onCollision(CollisionEvent &evt);

private:
	SimObj *my;
	SimObj *avatar;
	std::vector<std::string> m_entities;

	coordinate node;

	bool start;
	bool end;
	bool stop;
	bool task_end;
	bool srpr_msg;

	double dx,dy,dz;
	int i;

	bool walking;

	FILE* fp;
	FILE* fp2;
	FILE* fp3;
	FILE* fp_log;
	string file_name;
	std::vector<std::string> object;
	float stepWidth;
	int sleeptime;
	const static int SIZE = 30;
	int motionNum;

	float HEIGHT[SIZE];
	float HEAD_JOINT1[SIZE];
	float LARM_JOINT1[SIZE]; // left shoulder
	float LARM_JOINT3[SIZE]; // left elbow
	float LARM_JOINT4[SIZE];
	float RARM_JOINT1[SIZE]; // right shoulder
	float RARM_JOINT3[SIZE]; // right elbow
	float RARM_JOINT4[SIZE];
	float WAIST_JOINT0[SIZE];

	int srprNum;
	float s_HEIGHT[SIZE];
	float s_HEAD_JOINT1[SIZE];
	float s_LARM_JOINT1[SIZE]; // left shoulder
	float s_LARM_JOINT3[SIZE]; // left elbow
	float s_LARM_JOINT4[SIZE];
	float s_RARM_JOINT1[SIZE]; // right shoulder
	float s_RARM_JOINT3[SIZE]; // right elbow
	float s_RARM_JOINT4[SIZE];
	float s_WAIST_JOINT0[SIZE];




	// condition flag for grasping trash
	bool grasp;
	std::string graspObjectName;
	SimObj *moveObject;

	int cnt_action;
	int count;
	int step;

	double interval;

	std::string m_lastReachedPoint;	// Point reached just before

	void initCondition();
};

void MyController::onInit(InitEvent &evt)
{
	my = getObj(myname());
	avatar = getObj(avatarName);

	// Put arms down
	my->setJointAngle("LARM_JOINT7", DEG2RAD(-90));
	my->setJointAngle("RARM_JOINT7", DEG2RAD(90));

	stepWidth = 20;


	if((fp = fopen("motion.txt", "r")) == NULL) {
		printf("File motion.txt does not exist.\n");
		exit(0);
		}
	else{
		fscanf(fp, "%d", &motionNum);
		fscanf(fp, "%d", &sleeptime);
		for(int i=0; i<motionNum; i++){
			fscanf(fp, "%f %f %f %f %f", 
					   &HEIGHT[i],
					   &LARM_JOINT1[i],
					   &LARM_JOINT3[i],
					   &RARM_JOINT1[i],
					   &RARM_JOINT3[i]);}
		}

	interval = sleeptime / 1000000;

	//object.push_back("curry");
	//object.push_back("spoon");
	//object.push_back("chopstick");
	//object.push_back("cup_brown");
	//object.push_back("end");
	object.push_back("spagetti");
	object.push_back("fork");
	object.push_back("knife");
	object.push_back("cup_white");
	object.push_back("end");

	m_lastReachedPoint = "TP4";

	fp_log = fopen("avatar.csv","w");
	fprintf(fp_log, "rx,ry,rz,ax,ay,az\n");

	cnt_action = 0;
	task_end = false;
}

void MyController::initCondition()
{
	start = false;
	end = false;
	stop = true;
	grasp = false;

	i=0;

	walking = false;

	count = 0;
	step = 0;

	double x=0;
	double y=0;
	double z=0;
	double flag=0; //Check point

	dx=0;
	dy=0;
	dz=0;

	for(int k=0; k<256; k++){
		node.x[k] = 0.0;
		node.y[k] = 0.0;
		node.z[k] = 0.0;
		node.flag[k] = 0.0;
	}

	if((fp = fopen(file_name.c_str(), "r")) == NULL) {
		LOG_MSG(("File does not exist."));
		exit(0);
	}
	while(fscanf(fp, "%lf,%lf,%lf,%lf", &x,&y,&z,&flag) != EOF) {
		node.x[i]=x;
		node.y[i]=y;
		node.z[i]=z;
		node.flag[i]=flag;
		i++;
	}


	fclose(fp);
	i=0;

}

double MyController::onAction(ActionEvent &evt)
{
	Vector3d pos;
	Vector3d pos_avatar;
	double angle;
	Vector3d ownPosition;

	my->getPosition(pos);
	avatar->getPosition(pos_avatar);

	cnt_action++;
	if(cnt_action == 10){
		fprintf(fp_log, "%lf,%lf,%lf,",pos.x(),pos.y(),pos.z());
		fprintf(fp_log, "%lf,%lf,%lf\n",pos_avatar.x(),pos_avatar.y(),pos_avatar.z());
		cnt_action = 0;
	}

	if(!end && start){
	
		// when you're not walking
		if(!walking){
			dx=(node.x[i]-pos.x());
			dz=(node.z[i]-pos.z());
			angle = atan2(dx,dz);

			my->setAxisAndAngle(0,1.0, 0, angle);

			double r = sqrt(pow(dx,2)+pow(dz,2));

			step = 2 * (int)r / stepWidth;

			dx /= step*motionNum;
			dz /= step*motionNum;

			walking = true;
		}

		// when you are walking
		else if(!stop){
			double addx = 0;
			double addz = 0;
		
			// IT WAS HERE FIRST!!!!
			
			if(count%2){
				for(int j=0; j<motionNum; j++){
					addx += dx;
					addz += dz;
					usleep(sleeptime);
					my->setPosition(pos.x()+addx, HEIGHT[i], pos.z()+addz);

					my->setJointAngle("LARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(LARM_JOINT3[j]));
					my->setJointAngle("RARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(RARM_JOINT3[j]));
				}
				if(grasp){
					my->getPartsPosition(ownPosition,"RARM_LINK7");
					moveObject->setPosition(ownPosition.x(),ownPosition.y(),ownPosition.z() );
				}
			}
			
			else{
				for(int j=0; j<motionNum; j++){
					addx += dx;
					addz += dz;
					usleep(sleeptime);
					my->setPosition(pos.x()+addx, HEIGHT[i], pos.z()+addz);
					my->setJointAngle("RARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(-LARM_JOINT3[j]));
					my->setJointAngle("LARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(-RARM_JOINT3[j]));
				}
			}
			count++;

			// arrived at the target point
			if(step==count){
				count = 0;
				step = 0;
				walking = false;
				i++;
			}
		}



		double checkPoint = node.flag[i-1];

		if(checkPoint==GRASP && !grasp){
			//grasp
			broadcastMsg("Grasp object");

			grasp = true;
			moveObject = getObj(object[0].c_str());
			graspObjectName = object[0];
			//sleep(1);
		}
		// 終了点に着いた
		if(checkPoint==END){
			//release
			if(graspObjectName == "curry" || graspObjectName == "spagetti"){//T4
				//broadcastMsg("Spagetti is set");
				printf("spagetti is set\n");
				moveObject->setPosition( 20.0, 68.5, 55.0 );
			}
			else if(graspObjectName == "spoon" || graspObjectName == "fork"){//T2
				broadcastMsg("Fork is set");
				moveObject->setPosition( 0.0, 68.5, 10.0 );
			}
			else if(graspObjectName == "chopstick" || graspObjectName == "knife"){//T1
				broadcastMsg("Knife is set");
				moveObject->setPosition( 0.0, 68.5, 100.0 );
			}
			else if(graspObjectName == "cup_brown" || graspObjectName == "cup_white"){//T3
				broadcastMsg("Cup_white is set");
				moveObject->setPosition( -20.0, 68.5, 55.0 );
			}

			graspNum++;
			// broadcast what was grasped
			std::cout<< "Number of grasps: " << graspNum <<endl;

			grasp = false;
			object.erase(object.begin());
			end = true;
			sleep(1);
		}
	}
	else if(task_end){
	}
	else{
		string str;
		str.append("I will take ");
		str.append(object[0]);
		str.append("\nPlease take ");
		str.append(object[1]);

		broadcastMsg(str.c_str());
		if(object[0] == "curry"){ // move toward curry
			if(m_lastReachedPoint == "TP1"){ // from TP1
				file_name = "path/curry_t1.txt";
			}
			else if(m_lastReachedPoint == "TP2"){ // from TP2
				file_name = "path/curry_t2.txt";
			}
			else if(m_lastReachedPoint == "TP3"){ // from TP3
				file_name = "path/curry_t3.txt";
			}
			else if(m_lastReachedPoint == "TP4"){ // from TP4
				file_name = "path/curry_t4.txt";
			}
			count = 0;
			initCondition();
			start = true;
			stop = false;
			m_lastReachedPoint = "TP4";
		}
		else if(object[0] == "spagetti"){ // move toward spagetti
			if(m_lastReachedPoint == "TP1"){ // from TP1
				file_name = "path/spagetti_t1.txt";
			}
			else if(m_lastReachedPoint == "TP2"){ // from TP2
				file_name = "path/spagetti_t2.txt";
			}
			else if(m_lastReachedPoint == "TP3"){ // from TP3
				file_name = "path/spagetti_t3.txt";
			}
			else if(m_lastReachedPoint == "TP4"){ // from TP4
				file_name = "path/spagetti_t4.txt";
			}
			count = 0;
			initCondition();
			start = true;
			stop = false;
			m_lastReachedPoint = "TP4";
		}
		else if(object[0] == "spoon"){ // move toward spoon
			if(m_lastReachedPoint == "TP1"){ // from TP1
				file_name = "path/spoon_t1.txt";
			}
			else if(m_lastReachedPoint == "TP2"){ // from TP2
				file_name = "path/spoon_t2.txt";
			}
			else if(m_lastReachedPoint == "TP3"){ // from TP3
				file_name = "path/spoon_t3.txt";
			}
			else if(m_lastReachedPoint == "TP4"){ // from TP4
				file_name = "path/spoon_t4.txt";
			}
			count = 0;
			initCondition();
			start = true;
			stop = false;
			m_lastReachedPoint = "TP2";
		}
		else if(object[0] == "fork"){ // move toward fork
			if(m_lastReachedPoint == "TP1"){ // from TP1
				file_name = "path/fork_t1.txt";
			}
			else if(m_lastReachedPoint == "TP2"){ // from TP2
				file_name = "path/fork_t2.txt";
			}
			else if(m_lastReachedPoint == "TP3"){ // from TP3
				file_name = "path/fork_t3.txt";
			}
			else if(m_lastReachedPoint == "TP4"){ // from TP4
				file_name = "path/fork_t4.txt";
			}
			count = 0;
			initCondition();
			start = true;
			stop = false;
			m_lastReachedPoint = "TP2";
		}
		else if(object[0] == "chopstick"){ // move toward chopstick
			if(m_lastReachedPoint == "TP1"){ // from TP1
				file_name = "path/chopstick_t1.txt";
			}
			else if(m_lastReachedPoint == "TP2"){ // from TP2
				file_name = "path/chopstick_t2.txt";
			}
			else if(m_lastReachedPoint == "TP3"){ // from TP3
				file_name = "path/chopstick_t3.txt";
			}
			else if(m_lastReachedPoint == "TP4"){ // from TP4
				file_name = "path/chopstick_t4.txt";
			}
			count = 0;
			initCondition();
			start = true;
			stop = false;
			m_lastReachedPoint = "TP1";
		}
		else if(object[0] == "knife"){ // move toward knife
			if(m_lastReachedPoint == "TP1"){ // from TP1
				file_name = "path/knife_t1.txt";
			}
			else if(m_lastReachedPoint == "TP2"){ // from TP2
				file_name = "path/knife_t2.txt";
			}
			else if(m_lastReachedPoint == "TP3"){ // from TP3
				file_name = "path/knife_t3.txt";
			}
			else if(m_lastReachedPoint == "TP4"){ // from TP4
				file_name = "path/knife_t4.txt";
			}
			count = 0;
			initCondition();
			start = true;
			stop = false;
			m_lastReachedPoint = "TP1";
		}
		else if(object[0] == "cup_brown"){ // move toward cup_brown
			if(m_lastReachedPoint == "TP1"){ // from TP1
				file_name = "path/cup_brown_t1.txt";
			}
			else if(m_lastReachedPoint == "TP2"){ // from TP2
				file_name = "path/cup_brown_t2.txt";
			}
			else if(m_lastReachedPoint == "TP3"){ // from TP3
				file_name = "path/cup_brown_t3.txt";
			}
			else if(m_lastReachedPoint == "TP4"){ // from TP4
				file_name = "path/cup_brown_t4.txt";
			}
			count = 0;
			initCondition();
			start = true;
			stop = false;
			m_lastReachedPoint = "TP3";
		}
		else if(object[0] == "cup_white"){ // move toward cup_white
			if(m_lastReachedPoint == "TP1"){ // from TP1
				file_name = "path/cup_white_t1.txt";
			}
			else if(m_lastReachedPoint == "TP2"){ // from TP2
				file_name = "path/cup_white_t2.txt";
			}
			else if(m_lastReachedPoint == "TP3"){ // from TP3
				file_name = "path/cup_white_t3.txt";
			}
			else if(m_lastReachedPoint == "TP4"){ // from TP4
				file_name = "path/cup_white_t4.txt";
			}
			count = 0;
			initCondition();
			start = true;
			stop = false;
			m_lastReachedPoint = "TP3";
		}
		else if(object[0] == "end"){ // end task
			broadcastMsg("Task end");
			task_end = true;
		}
	}

	return 0.1;//interval;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();

	//broadcastMsg(msg.c_str());
	LOG_MSG(("receive: %s",msg.c_str()));

	// object confirmation
	for(int i=0; i<object.size(); i++){
		if(msg == object[i]){
			object.erase(object.begin()+i);
		}

  //get an handle to agent
  SimObj *my = getObj(myname());

  // string message is received
  std::string value(evt.getMsg());

}

}

void MyController::onCollision(CollisionEvent &evt)
{
}  //can be deleted

extern "C" Controller * createController() {
	return new MyController;
}