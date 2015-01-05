#include <string>
#include <sstream>
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include <ViewImage.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define RAD2DEG(RAD) ( (RAD) * 180.0 / (PI) )

using namespace std;

class AgentController : public Controller
{
public:
	double onAction(ActionEvent &evt);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	void strSplit(string msg, string separator);
	void moveByKinect();
	void moveByOrs();

private:
	//初期位置                                                                    
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;
	//データ数（関節数）最大値                                                    
	int m_maxsize;

	BaseService *m_kinect;
	BaseService *m_ors;

	// 前回送信したyaw, pitch roll
	double pyaw, ppitch, proll;

	// 体全体の角度
	double m_qw, m_qy, m_qx, m_qz;

	bool chk_srv;
	bool play;
	bool rec;

	//strSplitで分割した文字列を格納する変数
	string headStr;
	string bodyStr;

	int count;
};

/*　strSplit
 *　文字列をseparatorで区切り
 *　separatorより前をheadStr
 *　separatorより後をbodyStr
 *　にそれぞれ格納
 */
void AgentController::strSplit(string msg, string separator)
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

void AgentController::onInit(InitEvent &evt)
{
	sendMsg("SIGORS1", "start");
	// 初期化          
	m_kinect = NULL;
	SimObj *my = getObj(myname());

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
	double qx = rot.qx();
	double qy = rot.qy();
	double qz = rot.qz();

	m_yrot = acos(fabs(qw))*2;
	if(qw*qy > 0)
	m_yrot = -1*m_yrot;

	m_range = 0.1;
	m_maxsize = 15;

	// 体全体の向き
	m_qw = qw;
	m_qx = qx;
	m_qy = qy;
	m_qz = qz;

	pyaw = ppitch = proll = 0.0;

	count = 0;

	play = false;
	rec = false;
}

double AgentController::onAction(ActionEvent &evt)
{
  //// サービスが使用可能か定期的にチェックする
  //bool available = checkService("SIGXTION1");
  //bool ch_ors = checkService("SIGORS1");

  //if (ch_ors && m_ors == NULL) {
	 // m_ors = connectToService("SIGORS1");
  //}
  //else if (!ch_ors && m_ors != NULL){
	 // m_ors = NULL;
  //}

  //// 使用可能
  //if(available && m_kinect == NULL){
  //  // サービスに接続
  //  m_kinect = connectToService("SIGXTION1");
  //  
  //}
  //// 使用不可能
  //else if (!available && m_kinect != NULL){
  //  m_kinect = NULL;
  //}
  
  return 1.0;
}

//SIG_KINECTからのデータを元にアバターを操作(位置，体全体の向き，首の関節以外の関節)
void AgentController::moveByKinect()
{
	//自分自身の取得                                                              
	SimObj *my = getObj(myname());

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
			m_qw = w;
			m_qx = x;
			m_qy = y;
			m_qz = z;
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

//SIG_ORSからのデータを元にアバターを操作（首の関節）
void AgentController::moveByOrs()
{
	//自分自身の取得                                                              
	SimObj *my = getObj(myname());

	//double yaw, pitch, roll;

	//strSplit(bodyStr, ",");
	//yaw = atof(headStr.c_str());

	//strSplit(bodyStr, ",");
	//pitch = atof(headStr.c_str());

	//strSplit(bodyStr, ",");
	//roll = atof(headStr.c_str());

	//if (yaw == pyaw && pitch == ppitch && roll == proll)  return;
	//else {
	//	pyaw = yaw;
	//	ppitch = pitch;
	//	proll = roll;
	//}

	//dQuaternion qyaw;
	//dQuaternion qpitch;
	//dQuaternion qroll;

	//qyaw[0] = cos(-yaw / 2.0);
	//qyaw[1] = 0.0;
	//qyaw[2] = sin(-yaw / 2.0);
	//qyaw[3] = 0.0;

	//qpitch[0] = cos(-pitch / 2.0);
	//qpitch[1] = sin(-pitch / 2.0);
	//qpitch[2] = 0.0;
	//qpitch[3] = 0.0;

	//qroll[0] = cos(-roll / 2.0);
	//qroll[1] = 0.0;
	//qroll[2] = 0.0;
	//qroll[3] = sin(-roll / 2.0);
	//dQuaternion tmpQ1;
	//dQuaternion tmpQ2;

	//dQMultiply0(tmpQ1, qyaw, qpitch);
	//dQMultiply0(tmpQ2, tmpQ1, qroll);

	//dQuaternion bodyQ;
	//bodyQ[0] = m_qw;
	//bodyQ[1] = m_qx;
	//bodyQ[2] = m_qy;
	//bodyQ[3] = m_qz;

	//dQuaternion tmpQ3;
	//dQMultiply1(tmpQ3, bodyQ, tmpQ2);

	//my->setJointQuaternion("HEAD_JOINT0", tmpQ3[0], tmpQ3[1], -tmpQ3[2], tmpQ3[3]);

	double w, x, y, z;

	strSplit(bodyStr, ",");
	w = atof(headStr.c_str());

	strSplit(bodyStr, ",");
	x = atof(headStr.c_str());

	strSplit(bodyStr, ",");
	y = atof(headStr.c_str());

	strSplit(bodyStr, ",");
	z = atof(headStr.c_str());

	dQuaternion neckQ;
	neckQ[0] = w;
	neckQ[1] = x;
	neckQ[2] = y;
	neckQ[3] = z;

	dQuaternion bodyQ;
	bodyQ[0] = m_qw;
	bodyQ[1] = m_qx;
	bodyQ[2] = m_qy;
	bodyQ[3] = m_qz;

	dQuaternion tmpQ3;
	dQMultiply1(tmpQ3, bodyQ, neckQ);

	w = tmpQ3[0];
	x = -tmpQ3[1];
	y = tmpQ3[2];
	z = -tmpQ3[3];

	my->setJointQuaternion("HEAD_JOINT0", w, x, y, z);
	if (rec == true){
		std::stringstream ss;
		ss << "ORS_DATA " << w << "," << x << "," << y << "," << z << " END:";
		sendMsg("logger1", ss.str());
	}

	//double n_yrot = acos(fabs(y)) * 2;
	//if (w * (-y) > 0)
	//	n_yrot = -1 * n_yrot;
	//LOG_MSG(("neck:%.2f", RAD2DEG(n_yrot)));
	////LOG_MSG(("neck:%.4f", y));

	//double b_yrot = acos(fabs(m_qy)) * 2;
	//if (m_qw * (-m_qy) > 0)
	//	b_yrot = -1 * b_yrot;
	//LOG_MSG(("body:%.2f\n", RAD2DEG(b_yrot)));
	////LOG_MSG(("body:%.4f",m_qw * m_qy));
	//
	//double rad = n_yrot - b_yrot;
	//LOG_MSG((" def:%.2f\n", RAD2DEG(rad)));

	//my->setJointQuaternion("HEAD_JOINT0", w, x, y - m_qy, z);
	//my->setJointQuaternion("HEAD_JOINT0", w - m_qw, x, y - m_qy, z);


	//my->setJointQuaternion("HEAD_JOINT0", tmpQ2[0], tmpQ2[1], -tmpQ2[2], tmpQ2[3]);
	//my->setJointQuaternion("HEAD_JOINT0", tmpQ2[0], tmpQ2[1] - m_qx, -tmpQ2[2] - m_qy, tmpQ2[3] - m_qz);

}

void AgentController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();

	//メッセージ取得                                                              
	string all_msg = evt.getMsg();

	strSplit(all_msg, " ");

	if (headStr == "play"){
		if (sender == "logger1"){
			play = true;
		}
	}
	else if (headStr == "PlayEnd" || headStr == "stop"){
		if (sender == "logger1"){
			play = false;
		}
	}
	else if (headStr == "rec"){
		if (sender == "logger1"){
			rec = true;
		}
	}
	if (play != true){
		if (headStr == "ORS_DATA")
		{
			moveByOrs();
		}
		else if (headStr == "XTION_DATA")
		{
			moveByKinect();
		}
	}
}

extern "C"  Controller * createController ()
{
  return new AgentController;
}
