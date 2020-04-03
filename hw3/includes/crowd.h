#ifndef __CROWD_H__
#define __CROWD_H__

#include "agent.h"
#include "common.h"

namespace NSCrowd
{
  using namespace NSShape;

  class Crowd
  {
  public:
    Crowd(std::vector<Agent>& agents, std::vector<Vertex>& targets)
      : _agents(agents), _targets(targets) {};

    std::vector<Agent>& agents() { return this->_agents; };
    std::vector<Vertex>& targets() { return this->_targets; };

    void genPath(std::vector<glm::vec3> &waypoints,
                 std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                 const std::vector<Obstacle> &obs,
                 const Rectangle &frame,
                 std::string mode = "PRM", std::string algorithm = "AStar");
    void genAgentPathByPRM(const std::vector<glm::vec3> &waypoints,
                           const std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                           const std::vector<Obstacle> &obs,
                           unsigned int, unsigned int, std::string);
    void genAgentPathByRRT(std::vector<glm::vec3> &waypoints,
                           std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                           const std::vector<Obstacle> &obs,
                           const Rectangle &frame,
                           float sampleProb, float strie,
                           unsigned int maxTimes, float threshold,
                           bool rrtstar, unsigned int index);
    void simulate(float nearestDist, std::vector<Obstacle> &obs,
                  Rectangle &frame,
                  std::string mode="boid", bool smooth = true);
  private:
    std::vector<Agent>& _agents;
    std::vector<Vertex>& _targets;
  };
}

#endif
