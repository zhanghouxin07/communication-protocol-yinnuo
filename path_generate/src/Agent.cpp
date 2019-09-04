#include "Agent.h"
#include "Dubins.h"

extern vector<double> waypoint[3];
extern int minradius;
AgentController::AgentController(size_t _velocity, double _wheelbase, double _minRadius, string _name):
  m_velocity(_velocity), m_name(_name), m_wheelbase(fabs(_wheelbase)), m_minRadius(fabs(_minRadius)) {}

  void AgentController::SetGoal(AgentState _goal){
    m_goal = _goal;
    Dubins d;
    cout<<"min radius  " <<minradius << "wheelbase  "<<m_wheelbase<<endl;
    m_nextTrajectory = d.DubinsShortestPath(minradius, m_wheelbase, m_myState, _goal);
  }

bool
AgentController::Update(){
  if (m_nextTrajectory.controls.empty())
    return false;
  vector<Control>::iterator nextC = m_nextTrajectory.controls.begin();
  for (size_t i=0;i<m_velocity; ++i){
    while(nextC->timesteps < 1.0 && nextC->timesteps <= 0.0){
      nextC = m_nextTrajectory.controls.erase(nextC);
      if (nextC == m_nextTrajectory.controls.end())
        return false;
    }
    nextC->timesteps--;

    //update stuff
    //update position
    m_myState.pos.first += DELTA*cos(m_myState.theta);
    m_myState.pos.second += DELTA*sin(m_myState.theta);

    //get turning radius
    double turningRadius = 0.0;
    bool straightLine = true;

    if (abs(nextC->steeringAngle) > 1e-5){
      turningRadius = m_wheelbase / sin(nextC->steeringAngle);
      straightLine = false;
    }

    if(!straightLine){
      m_myState.theta += (DELTA)/turningRadius;
      if (m_myState.theta > PI)
        m_myState.theta -= 2.0*PI;
      else if (m_myState.theta < -PI)
        m_myState.theta += 2.0*PI;
    }
  }
  static int counter = 0;
  counter++;
  //cout << "Agent state after update: "  << m_myState << " "<<counter<<endl;
  waypoint[0].push_back(m_myState.pos.first);
  waypoint[1].push_back(m_myState.pos.second);
  waypoint[2].push_back(m_myState.theta);
  return true;
}//end Update function
