#include "crowd.h"

namespace NSCrowd
{
  void Crowd::genPath(std::vector<glm::vec3> &waypoints,
                      std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                      const std::vector<Obstacle> &obs,
                      const Rectangle &frame,
                      std::string mode, std::string algorithm)
  {
    if (_targets.size() < _agents.size()) {
      unsigned int padSize = _agents.size() - _targets.size();
      for (unsigned int i = 0; i < padSize; ++i)
        _targets.push_back(_targets.back());
    }

    for (auto &agent:_agents)
      agent.clearPath();

    if (mode == "RRT" || mode == "RRTStar") {
      for (unsigned int i = 0; i < _agents.size(); ++i) {
        waypoints.clear();
        graph.clear();
        waypoints.push_back(_agents[i].center().pos());
        graph.push_back(std::vector<std::pair<unsigned int, float>>{});

        if (mode == "RRT")
          genAgentPathByRRT(waypoints, graph, obs, frame, 0.5f, 0.3f, 5000, 0.2f, false, i);
        else
          genAgentPathByRRT(waypoints, graph, obs, frame, 0.5f, 0.3f, 1500, 0.2f, true, i);
      }
    } else {
      std::vector<unsigned int> targetIndex;
      for (auto &tar: _targets) {
        float minWayToCornerDist = FLT_MAX;
        unsigned int curTargetIndex = 0;
        for (unsigned int i = 0; i < waypoints.size(); ++i) {
          float curDist = glm::distance(waypoints[i], tar.pos());
          if (curDist < minWayToCornerDist) {
            minWayToCornerDist = curDist;
            curTargetIndex = i;
          }
        }
        targetIndex.push_back(curTargetIndex);
      }

      for (unsigned int i = 0; i < _agents.size(); ++i)
        genAgentPathByPRM(waypoints, graph, obs, targetIndex[i], i, algorithm);
    }

  }

  void Crowd::genAgentPathByPRM(const std::vector<glm::vec3> &waypoints,
                                const std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                                const std::vector<Obstacle> &obs,
                                unsigned int target, unsigned int i, std::string algorithm)
  {
    _agents[i].findPathByPRM(waypoints, graph, obs, target, algorithm);
  }

  void Crowd::genAgentPathByRRT(std::vector<glm::vec3> &waypoints,
                                std::vector<std::vector<std::pair<unsigned int, float>>> &graph, 
                                const std::vector<Obstacle> &obs,
                                const Rectangle &frame, float sampleProb, float stride,
                                unsigned int maxTimes, float threshold,
                                bool rrtstar, unsigned int i) {
    _agents[i].findPathByRRT(waypoints, graph, obs, frame, _targets[i].pos(), sampleProb, stride, maxTimes, threshold, rrtstar);
  }

  void Crowd::simulate(float nearestDist,
                       std::vector<Obstacle> &obs,
                       Rectangle &frame,
                       std::string mode,
                       bool smooth)
  {
    if (mode == "orca") {
      ;
    } else {
      for (auto &agent: _agents) {
        std::vector<Agent> nnAgents;
        std::vector<Obstacle> nnObstacles;
        std::vector<std::vector<glm::vec3>> nnEdges;

        // Brute Force - locate nearest objects
        nearestShapeBF(nearestDist, agent, _agents, obs, frame,
                       nnAgents, nnObstacles, nnEdges);

        // Smooth path - regenerate next target
        if (smooth)
          agent.smoothPath(obs);

        // Update by Boids
        agent.updateByBoids(_agents, nnObstacles, nnEdges);
      }

    }
  }
}
