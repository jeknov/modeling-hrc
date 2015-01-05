#include <string>
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <map>
#include <time.h>
#include <sys/time.h>
#include "quatanion.h"

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define RAD2DEG(RAD) ( (RAD) * 180.0 / (PI) )

using namespace std;

typedef map<string, double> JMap;

class ActionLogger : public Controller
{
public:
	double onAction(ActionEvent &evt);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	void writeActionLog(std::string msg);
	void readActionLog();
	void playActionLog(double time);
	void strSplit(string msg, string separator);
	double gettimeofday_sec();
	void moveByKinect();
	void moveByOrs();
	std::string getLocalTime();
	//string FloatToString(float x);


private:
	//初期位置                                                                    
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;
	//データ数（関節数）最大値                                                    
	int m_maxsize;

	BaseService *m_kinect;

	Vector3d m_pos;
	Rotation m_rotation;
	string body_str;
	int count;
	ofstream ofs;
	stringstream ss;

	bool play;
	bool write;
	bool playLog;
	bool writeInit;
	double time_init;
	double time_start;
	double time_current;
	double time_target;
	int logIndex;
	int logSize;
	vector<string> actionLog;
	string logName;

	int ors_count;
	int xtion_count;
	double time_ors;
	double time_xtion;
	double time_pre_target;
	double time_diff;

	// 前回送信したyaw, pitch roll
	double pyaw, ppitch, proll;

	string headStr;
	string bodyStr;

	std::string name_man;

	ofstream waist;
	ofstream arm;
	ofstream neck;
	ofstream log_ors;
	ofstream log_xtion;
	//ofstream log_voice;
	double logTime;
};

double ActionLogger::gettimeofday_sec(){
	struct timeval t;
	gettimeofday(&t, NULL);
	return (double)t.tv_sec + (double)t.tv_usec * 1e-6;
}

std::string ActionLogger::getLocalTime(){
	time_t timer;
	struct tm *local;

	timer = time(NULL);
	local = localtime(&timer);
	double year = local->tm_year + 1900;
	double month = local->tm_mon + 1;
	double day = local->tm_mday;
	double hour = local->tm_hour;
	double minute = local->tm_min;
	double second = local->tm_sec;

	ss << year << std::setw(2) << std::setfill('0') << month << std::setw(2) << std::setfill('0') << day << std::setw(2) << std::setfill('0') << hour << std::setw(2) << std::setfill('0') << minute << std::setw(2) << std::setfill('0') << second;

	string name = name_man + "_actionLog_";
	name += ss.str() + ".txt";

	ss.str("");
	ss.clear(stringstream::goodbit);

	return name;
}

void ActionLogger::strSplit(string msg, string separator)
{
	int strPos1 = 0;
	int strPos2;
	std::string head;
	std::string body;

	strPos2 = msg.find_first_of(separator, strPos1);
	head.assign(msg, strPos1, strPos2 - strPos1);
	body.assign(msg, strPos2 + 1, msg.length() - strPos2);
	headStr = head;
	bodyStr = body;
}

void ActionLogger::onInit(InitEvent &evt)
{
	play = false;
	write = false;
	writeInit = false;
	
	time_start = 0;
	time_current = 0;
	time_target = 0;
	logIndex = 0;
	logSize = 0;

	name_man = "man_001";

	body_str = "";

	SimObj *my = getObj(name_man.c_str());

	// 初期位置取得                                                            
	Vector3d pos;
	my->getPosition(pos);
	m_posx = pos.x();
	m_posy = pos.y();
	m_posz = pos.z();

	// 初期姿勢（回転）取得                                                                                           
	Rotation rot;
	my->getRotation(rot);
	double qw = rot.qw();
	double qy = rot.qy();

	m_yrot = acos(fabs(qw)) * 2;
	if (qw*qy > 0)
		m_yrot = -1 * m_yrot;
	LOG_MSG(("m_posx:%f", m_posx));
	LOG_MSG(("m_posy:%f", m_posy));
	LOG_MSG(("m_posz:%f", m_posz));
	LOG_MSG(("m_yrot:%f", m_yrot));

	m_range = 0.1;
	m_maxsize = 15;

	pyaw = ppitch = proll = 0.0;

	time_init = gettimeofday_sec();

	ors_count = 0;
	xtion_count = 0;
	time_ors = 0;
	time_xtion = 0;
	time_pre_target = 0;
	time_diff = 0;
	
	logName = "man_001_actionLog_20140924175040.txt";

	//ofstream clear("man_001_waist.csv", ios::trunc);
	//waist.open("man_001_waist.csv", ios::app);
	ofstream clear("man_001_neck.csv", ios::trunc);
	neck.open("man_001_neck.csv", ios::app);
	ofstream clear2("man_001_arm.csv", ios::trunc);
	arm.open("man_001_arm.csv", ios::app);
	ofstream clear3("man_001_ors.csv", ios::trunc);
	log_ors.open("man_001_ors.csv", ios::app);
	ofstream clear4("man_001_xtion.csv", ios::trunc);
	log_xtion.open("man_001_xtion.csv", ios::app);
	//ofstream clear5("man_001_voice.csv", ios::trunc);
	//log_voice.open("man_001_voice.csv", ios::app);
}

void ActionLogger::writeActionLog(std::string msg)
{
	if (writeInit == true){
		writeInit = false;
		logName = getLocalTime();
		ofstream clear(logName.c_str(), ios::trunc);
		ofs.open(logName.c_str(), ios::app);
	}

	//ログとして記録する情報
	//時間
	time_current = gettimeofday_sec() - time_start;
	ss << "TIME:" << time_current << " ";
	ss << msg;

	body_str += ss.str();

	ofs << body_str << std::endl;

	body_str = "";
	ss.str("");
	ss.clear(stringstream::goodbit);

}

void ActionLogger::readActionLog()
{
	ifstream ifs(logName.c_str());
	while (ifs){
		string str;
		getline(ifs, str);
		actionLog.push_back(str);
	}
	logSize = actionLog.size() - 1;
}

void ActionLogger::moveByKinect()
{
	//自分自身の取得                                                              
	SimObj *my = getObj(name_man.c_str());

	int i = 0;
	while (true)
	{
		i++;
		if (i == m_maxsize + 1) break;
		strSplit(bodyStr, ":");

		//体の位置                                                            
		if (headStr == "POSITION")
		{
			strSplit(bodyStr, ",");
			double x = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double y = atof(headStr.c_str());
			strSplit(bodyStr, " ");
			double z = atof(headStr.c_str());
			//キネクト座標からSIGVerse座標への変換                            
			double gx = cos(m_yrot)*x - sin(m_yrot)*z;
			double gz = sin(m_yrot)*x + cos(m_yrot)*z;
			my->setPosition(m_posx + gx, m_posy + y, m_posz + gz);
			continue;
		}

		//体全体の回転                                                        
		else if (headStr == "WAIST")
		{
			strSplit(bodyStr, ",");
			double w = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double x = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double y = atof(headStr.c_str());
			strSplit(bodyStr, " ");
			double z = atof(headStr.c_str());
			my->setJointQuaternion("ROOT_JOINT0", w, x, y, z);

			//double t_yrot = acos(fabs(w)) * 2;
			//if (w*y > 0)
			//	t_yrot = -1 * t_yrot;
			//waist << logTime << "," << t_yrot << "," << pyaw << "," << t_yrot + pyaw << endl;
			continue;
		}

		else if (headStr == "END")
		{
			break;
		}



		//関節の回転                                                          
		else
		{
			string type = headStr.c_str();

			strSplit(bodyStr, ",");
			double w = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double x = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double y = atof(headStr.c_str());
			strSplit(bodyStr, " ");
			double z = atof(headStr.c_str());

			if (type == "RARM_JOINT2"){
				double q[4];
				q[0] = w;
				q[1] = x;
				q[2] = y;
				q[3] = z;
				//肩のロール軸回転を計算
				double aroll_rad = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
				double aroll_deg = RAD2DEG(aroll_rad);
				//両手を下した状態を0度，腕を上げる回転方向を正としてテキスト出力
				arm << logTime << "," << 90 - aroll_deg << std::endl;
			}

			double angle = acos(w) * 2;
			double tmp = sin(angle / 2);
			double vx = x / tmp;
			double vy = y / tmp;
			double vz = z / tmp;
			double len = sqrt(vx*vx + vy*vy + vz*vz);
			if (len < (1 - m_range) || (1 + m_range) < len) continue;
			if (type != "HEAD_JOINT1"){
				my->setJointQuaternion(type.c_str(), w, x, y, z);
			}
			continue;
		}
	}

}

void ActionLogger::moveByOrs()
{
	//自分自身の取得                                                              
	SimObj *my = getObj(name_man.c_str());

	double w, x, y, z;

	strSplit(bodyStr, ",");
	w = atof(headStr.c_str());

	strSplit(bodyStr, ",");
	x = atof(headStr.c_str());

	strSplit(bodyStr, ",");
	y = atof(headStr.c_str());

	strSplit(bodyStr, ",");
	z = atof(headStr.c_str());

	my->setJointQuaternion("HEAD_JOINT0", w, x, y, z);
}

void ActionLogger::playActionLog(double time)
{
	//メッセージ取得
	if (logSize > logIndex){
		string all_msg = actionLog[logIndex];

		strSplit(all_msg, ":");
		if (headStr == "TIME")
		{
			strSplit(bodyStr, " ");
			logTime = atof(headStr.c_str());
			time_target =  logTime + time_start - time_init;

			if (time >= time_target)
			{
				time_diff = time_target - time_pre_target;
				
				//対象の取得
				SimObj *my = getObj(name_man.c_str());
				strSplit(bodyStr, " ");
				if (headStr == "XTION_DATA"){
					log_xtion << all_msg << std::endl;
					moveByKinect();
					time_xtion += time_diff;
					xtion_count++;
				}
				else if (headStr == "ORS_DATA"){
					log_ors << all_msg << std::endl;
					moveByOrs();
					time_ors += time_diff;
					ors_count++;
				}
				//else if (headStr == "VOICE_DATA"){
				//	log_voice << all_msg << std::endl;
				//	sendMsg("VoiceReco_Service1", bodyStr);

				//}
				logIndex++;
				time_pre_target = time_target;
			}
		}
	}
	else
	{
		LOG_MSG(("Play End  :%4f", time));
		LOG_MSG(("Play Time :%4f", time - (time_start - time_init)));
		LOG_MSG(("XTION TIME:%4f", time_xtion / xtion_count));
		LOG_MSG(("ORS TIME  :%4f\n", time_ors / ors_count));
		sendMsg("SIGViewer", "Play End\n");
		sendMsg(name_man, "PlayEnd");
		sendMsg("logger", "PlayEnd");
		//sendMsg("VoiceReco_Service", "再生終了");
		play = false;
		logIndex = 0;
		time_pre_target = 0;
		time_xtion = 0;
		xtion_count = 0;
		time_ors = 0;
		ors_count = 0;
		actionLog.clear();
	}
}

double ActionLogger::onAction(ActionEvent &evt)
{
	if (write == true){
		play = false;
		//writeActionLog(evt.time() - time_start);
	}
	else if (play == true){
		write = false;
		time_current = gettimeofday_sec() - time_init;
		playActionLog(time_current);
	}

	return 0.001;
}

void ActionLogger::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();

	//メッセージ取得                                                              
	string all_msg = evt.getMsg();
	
	if (all_msg == "rec"){
		if (write != true){
			sendMsg("voiceLog1", "rec");
			sendMsg(name_man, "rec");
			sendMsg("SIGViewer", "Rec Start\n");
			write = true;
			writeInit = true;
			play = false;
			time_start = gettimeofday_sec();
			LOG_MSG(("Rec Start :%4f", time_start - time_init));
		}
	}
	else if (all_msg == "stop"){
		sendMsg("voiceLog1", "stop");
		sendMsg(name_man, "stop");
		if (write == true){
			double time = gettimeofday_sec() - time_init;
			sendMsg("SIGViewer", "Rec Stop\n");
			LOG_MSG(("Rec Stop  :%4f", time));
			LOG_MSG(("Rec Time  :%4f\n", time - (time_start - time_init)));
		}
		if (play == true){
			sendMsg("SIGViewer", "Play Stop\n");
		}
		write = false;
		play = false;
	}
	else if (all_msg == "play"){
		if (play != true){
			sendMsg("voiceLog1", "play");
			sendMsg(name_man, "play");
			sendMsg("SIGViewer", "Play Start\n");
			play = true;
			write = false;
			time_start = gettimeofday_sec();
			LOG_MSG(("logFile:%s", logName.c_str()));
			LOG_MSG(("Play Start:%4f", time_start - time_init));
			readActionLog();
		}
	}
	else if (write == true){
		writeActionLog(all_msg);
	}
	
}

extern "C"  Controller * createController()
{
	return new ActionLogger;
}
