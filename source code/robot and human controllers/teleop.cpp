#include <string>
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include <ViewImage.h>
#include <math.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

using namespace std;

class AgentController : public Controller
{
public:
  double onAction(ActionEvent &evt);
  void onRecvMsg(RecvMsgEvent &evt);
  void onInit(InitEvent &evt);
  void onCollision(CollisionEvent &evt);

private:
  double m_posx, m_posy, m_posz;
  double m_yrot;
  double m_range;
  int m_maxsize;     // maximum number of joints

  //flag for grasping
  string grasp_obj;

  //parts to be grasped
  const char* grasp_parts;

  //flag for collision
  bool Colli;
 Vector3d pos;
  BaseService *m_kinect;
};

void AgentController::onInit(InitEvent &evt)
{
  Colli = false;

  //grasping by right hand
  grasp_parts = "RARM_LINK7";

  m_kinect = NULL;
  SimObj *my = getObj(myname());

  // Get initial position

  my->getPosition(pos);
  m_posx = pos.x();
  m_posy = pos.y();
  m_posz = pos.z();



  // Get initial rotation
  Rotation rot;
  my->getRotation(rot);
  double qw = rot.qw();
  double qy = rot.qy();

  m_yrot = acos(fabs(qw))*2;
  if(qw*qy > 0)
    m_yrot = -1*m_yrot;

  m_range = 0.1;
  m_maxsize = 15;
}

double AgentController::onAction(ActionEvent &evt)
{
  // Frequent check whether service is available or not
  bool available = checkService("SIGKINECT");

  // If service is available
  if(available && m_kinect == NULL){
    // Connect to the service
    m_kinect = connectToService("SIGKINECT");
  }
  // If service is not working
  else if (!available && m_kinect != NULL){
    m_kinect = NULL;
  }
  return 1.0;
}

void AgentController::onRecvMsg(RecvMsgEvent &evt)
{
  char * msgR = (char*)evt.getMsg();
 // LOG_MSG(("msgR : %s", msgR.c_str()));

  std::string sender = evt.getSender();

  SimObj *my = getObj(myname());
  // std::cout << "MsgR " <<  msgR << endl;

  // Receiving message
 // all_msg = (char*)evt.getMsg();
//char * all_msg = (char*)msgR.c_str();
  char *msg = strtok(msgR," ");
  std::cout << "Msg :" <<  msgR << endl;
  if(strcmp(msg,"KINECT_DATA") == 0)
    {
         std::cout << "Kienct OK " << endl;
      int i = 0;
      while(true)
        {
          i++;
          if(i == m_maxsize+1) break;
          char *type = strtok(NULL,":");

          // Position of the avatar
          if(strcmp(type,"POSITION") == 0)
            {
              double x = atof(strtok(NULL,","));
              double y = atof(strtok(NULL,","));
              double z = atof(strtok(NULL," "));
              // Coordinate transform from Kinect coordinate to SIGVerse coordinate
              double gx = cos(m_yrot)*x - sin(m_yrot)*z;
              double gz = sin(m_yrot)*x + cos(m_yrot)*z;
            //  Vector3d m_pos;
              my->getPosition(pos);
              my->setPosition(pos.x(),pos.y(),pos.z());

              continue;
            }

          // Orientation of the whole body
          else if(strcmp(type,"WAIST") == 0)
            {
              double w = atof(strtok(NULL,","));
              double x = atof(strtok(NULL,","));
              double y = atof(strtok(NULL,","));
              double z = atof(strtok(NULL," "));
              my->setJointQuaternion("ROOT_JOINT0",w,x,y,z);
              continue;
            }

          else if(strcmp(type,"END") == 0)  break;

          // Rotation of joints
          else
            {
              double w = atof(strtok(NULL,","));
              double x = atof(strtok(NULL,","));
              double y = atof(strtok(NULL,","));
              double z = atof(strtok(NULL," "));
              double angle = acos(w)*2;
              double tmp = sin(angle/2);
              double vx = x/tmp;
              double vy = y/tmp;
              double vz = z/tmp;
              double len = sqrt(vx*vx+vy*vy+vz*vz);
              if(len < (1 - m_range) || (1 + m_range) < len) continue;
              my->setJointQuaternion(type,w,x,y,z);
              continue;
            }
}
}
  // The agent will rotate when a message "rotationL" is received.
  else if (strcmp(msgR, "rotationL") == 0){

    // Set rotational angle
    int dy = 3;

    // Get rotation value (quaternion) around y-axis
    Rotation rot;
    my->getRotation(rot);

    // calculate radian value from quaternion
    double theta = 2*asin(rot.qy());

    // Rotate the body
    double y = theta + DEG2RAD(dy);
    if( y >= PI) {
      y = y - 2 * PI;
    }
    my->setAxisAndAngle(0, 1.0, 0, y);
  }

  // The agent will rotate when a message "rotationR" is received.
  else if (strcmp(msgR, "rotationR") == 0){

    // Set rotational angle
    int dy = -3;

    // Get rotation value (quaternion) around y-axis
    Rotation rot;
    my->getRotation(rot);

    // calculate radian value from quaternion
    double theta = 2*asin(rot.qy());

    // Rotate the body
    double y = theta + DEG2RAD(dy);
    if( y >= PI) {
      y = y - 2 * PI;
    }
    my->setAxisAndAngle(0, 1.0, 0, y);
  }

  // The agent will go forward when a message "move" is come
 else if (strcmp(msgR, "move") == 0){

    // Get current position
std::cout << "I m moving " << endl;
    my->getPosition(pos);
//std::cout << "I m moving " << endl;
    // Get rotation value (quaternion) around y-axis
    Rotation rot;
    my->getRotation(rot);

    // calculate joint value from quaternion
    double theta = 2*asin(rot.qy());

    // Initialize displacement in the movement
    double dx = 0.0;
    double dz = 0.0;

    // Set velocity
    double vel = 5;

    // Set orientation of the motion
    dx = sin(theta) * vel;
    dz = cos(theta) * vel;

    // Execute the motion
    my->setPosition( pos.x() + dx, pos.y() , pos.z() + dz );
  }

  // The agent will go forward when a message "back" is come
 else if (strcmp(msgR, "back") == 0){

    // Get current position

    my->getPosition(pos);

    // Get rotation value (quaternion) around y-axis
    Rotation rot;
    my->getRotation(rot);

    // calculate joint value from quaternion
    double theta = 2*asin(rot.qy());

    // Initialize displacement in the movement
    double dx = 0.0;
    double dz = 0.0;

    // Set velocity
    double vel = 15;

    // Set orientation of the motion
    dx = sin(theta) * vel;
    dz = cos(theta) * vel;

    // Execute the motion
    my->setPosition( pos.x() - dx, pos.y() , pos.z() - dz );
  }


  else if (strcmp(msgR, "regrasp") == 0){

    Colli = false;
  }



  // Release
  if (strcmp(msgR, "release") == 0){
    // Get part of hand
    CParts * parts = my->getParts(grasp_parts);

    // Execution of release
    parts->releaseObj();

  }


}


// Eventhandler for the collision
void AgentController::onCollision(CollisionEvent &evt)
{

  if (Colli == false)
  {
    typedef CollisionEvent::WithC C;

    // Get the name of entity which is touched by the agent
    const std::vector<std::string> & with = evt.getWith();

    // Get the name of link parts of the agent
    const std::vector<std::string> & mparts = evt.getMyParts();

    // Loop for every entity which is collided
    for(int i = 0; i < with.size(); i++)
    {

      // If the right hand or right arm is touched
      if(mparts[i] == "RARM_LINK7" || mparts[i] == "RARM_LINK4")
      {

  SimObj *my = getObj(myname());

  CParts * parts = my->getParts(grasp_parts);

  // Execution of grasping
  parts->graspObj(with[i]);

        Colli = true;
      }
    }
  }
}

extern "C"  Controller * createController ()
{
  return new AgentController;
}
