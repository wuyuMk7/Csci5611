#include "agent.h"
#include "common.h"

namespace NSShape
{
  float tH = 5, maxAvoidF = 100.0f;
  // Code from Prof. Guy
  glm::vec3 avoidForce(Agent &agent, Circle *tar)
  {
    float t = timeToCollision(agent, tar);

    if (t == FLT_MAX)
      return glm::vec3(0.0f, 0.0f, 0.0f);

    glm::vec3 force(0.0f, 0.0f, 0.0f);
    force = agent.center().pos() + agent.center().vel() * t -
      tar->center().pos() - tar->center().vel() * t;
    if (force.x != 0.0f && force.y != 0.0f)
      force = glm::normalize(force);

    float forceMag = 0.0f;
    if (t >= 0 && t <= tH)
      forceMag = (tH - t) / (t + 0.001f);
    //std::cout << t << ", " << forceMag << std::endl;
    if (forceMag > maxAvoidF)
      forceMag = maxAvoidF;
    force = forceMag * force;

    return force;
  }

  unsigned int Circle::renderPoints(std::vector<Vertex> &points)
  {
    double angle = 2 * M_PI / _count;
    Vertex center = _vertices.front(), cur = center;

    for (unsigned int i = 0; i < _count; ++i) {
      cur.pos(center.pos().x + _radius * cos(i * angle),
              center.pos().y + _radius * sin(i * angle),
              center.pos().z);
      points.push_back(cur);

      cur.pos(center.pos().x + _radius * cos((i+1) * angle),
              center.pos().y + _radius * sin((i+1) * angle),
              center.pos().z);
      points.push_back(cur);
    }

    return this->renderSize();
  }

  unsigned int Circle::boundingBox(std::vector<Vertex> &points)
  {
    Vertex center = _vertices.front(), cur = center;
    std::vector<std::pair<float, float>> boundingPoints{
        std::pair<float, float>{center.pos().x - _radius,
                                center.pos().y + _radius},
        std::pair<float, float>{center.pos().x + _radius,
                                center.pos().y + _radius},
        std::pair<float, float>{center.pos().x + _radius,
                                center.pos().y - _radius},
        std::pair<float, float>{center.pos().x - _radius,
                                center.pos().y - _radius}
    };

    for (auto &point : boundingPoints) {
      cur.pos(point.first, point.second, center.pos().z);
      points.push_back(cur);
    }

    return 4;
  }

  unsigned int Polygon::renderPoints(std::vector<Vertex> &points)
  {
    for (unsigned int i = 0;i < _vertices.size(); ++i) {
      points.push_back(_vertices[i]);
      i == _vertices.size() - 1 ? points.push_back(_vertices[0]): points.push_back(_vertices[i+1]);
    }

    return this->renderSize();
  }

  unsigned int Polygon::boundingBox(std::vector<Vertex> &points) {
    Vertex first = _vertices.front(), cur = first;
    float x[2] = { first.pos().x, first.pos().x },
      y[2] = { first.pos().y, first.pos().y },
      z[2] = { first.pos().z, first.pos().z };

    for (auto &vertex: _vertices) {
      if (vertex.pos().x < x[0])
        x[0] = vertex.pos().x;
      if (vertex.pos().x > x[1])
        x[1] = vertex.pos().x;
      if (vertex.pos().y < y[0])
        y[0] = vertex.pos().y;
      if (vertex.pos().y > y[1])
        y[1] = vertex.pos().y;
      if (vertex.pos().z < z[0])
        z[0] = vertex.pos().z;
      if (vertex.pos().z > z[1])
        z[1] = vertex.pos().z;
    }

    if (z[0] == z[1]) {
      for (unsigned int i = 0;i < 2; ++i) {
        for (unsigned int j = 0;j < 2; ++j) {
          cur.pos(x[i], y[j], z[0]);
          points.push_back(cur);
        }
      }

      return 4;
    } else {
      for (unsigned int i = 0; i < 2; ++i) {
        for (unsigned int j = 0; j < 2; ++j) {
          for (unsigned int k = 0;j < 2; ++k) {
            cur.pos(x[i], y[j], z[k]);
            points.push_back(cur);
          }
        }
      }
      return 8;
    }
  }

  void Agent::updateByBoids(std::vector<Agent> &agents,
                            std::vector<Obstacle> &obs,
                            std::vector<std::vector<glm::vec3>> &nnEdges)
  {
    glm::vec3 curPos = this->center().pos();
    float dt = 0.01, targetSpeed = 5.0f;
    _vertices[0].force(0.0f, 0.0f, 0.0f);

    // Settings
    float separationDist = radius() * 1.5f;
    /*
    float separationWeight = 3.0f, alignmentWeight = 2.0f, cohesionWeight = 2.0f,
      obWeight = 3.0f, edgeWeight = 1.0f, steerWeight = 5.0f;
    */
    float separationWeight = 3.0f, alignmentWeight = 1.0f,
          cohesionWeight = 1.0f, obWeight = 1.5f, edgeWeight = 1.0f,
          steerWeight = 5.0f;

    glm::vec3 separationForce(0.0f, 0.0f, 0.0f), cohesionForce(0.0f, 0.0f, 0.0f),
      alignmentForce(0.0f, 0.0f, 0.0f), finalForce(0.0f, 0.0f, 0.0f), finalVel(0.0f, 0.0f, 0.0f);
    float seCount = 0.0f, alCount = 0.0f, coCount = 0.0f, colCount = 0.0f;

    glm::vec3 colAvoidVel(0.0f, 0.0f, 0.0f);
    for (auto &agent: agents) {
      glm::vec3 relPos = center().pos() - agent.center().pos();
      float curDist = glm::length(relPos) - 2 * radius();

      // Avoid Collision
      // TODO: Maybe remove that in the future
      if (curDist <= 0 && curDist != -2 * radius()) {
        Vertex newCenter(center());
        glm::vec3 newPos =
            agent.center().pos() + 2.0f * radius() * glm::normalize(relPos);
        newCenter.pos(newPos.x, newPos.y, newPos.z);
        center(newCenter);
      }

      // Separation
      //if (curDist > 0.01f && curDist <= separationDist ) {
      if (curDist > 0.01f) {
        separationForce += avoidForce(*this, &agent);
      }

      // Alignment && Cohesion
      if (curDist > 0) {
        alignmentForce += agent.center().vel();
        alCount += 1;

        cohesionForce += agent.center().pos();
        coCount += 1;
      }
    }

    if (alCount > 0 && glm::length(alignmentForce) > 0) {
      glm::vec3 alDirection = glm::normalize(alignmentForce);
      glm::vec3 curDirection = center().vel();
      if (glm::length(curDirection) > 0) {
        curDirection = glm::normalize(curDirection);
      }
      alignmentForce = (glm::length(alignmentForce) / alCount * (alDirection - curDirection));
    }


    if (coCount > 0 && glm::length(cohesionForce) > 0) {
      glm::vec3 coDirection = glm::normalize(cohesionForce);
      glm::vec3 curDirection = center().pos();
      if (glm::length(curDirection) > 0) {
        curDirection = glm::normalize(curDirection);
      }
      cohesionForce = (glm::length(cohesionForce - curDirection) / coCount * (coDirection - curDirection));
    }

    glm::vec3 frameForce(0.0f, 0.0f, 0.0f), frameVel(0.0f, 0.0f, 0.0f);
    for (auto &edge: nnEdges) {
      float curDist = glm::length(glm::cross(center().pos() - edge[0], glm::normalize(edge[1] - edge[0])));
      glm::vec3 towardsN = glm::normalize(edge[3]-edge[2]);
      //std::cout << glm::to_string(edge[0]) << " ," << glm::to_string(edge[1]) << ", " << curDist << std::endl;
      if (curDist <= radius()) {
        glm::vec3 vV = glm::dot(center().vel(), towardsN) * towardsN;

        Vertex newCenter(center());
        glm::vec3 proj = glm::dot(center().pos() - edge[0], glm::normalize(edge[1] - edge[0])) * glm::normalize(edge[1] - edge[0]);
        glm::vec3 newPos = proj - radius() * towardsN + edge[0];
        newCenter.pos(newPos.x, newPos.y, newPos.z);
        center(newCenter);

        frameVel += (- 1.5f * vV);
      } else {
        if (checkVelSegIntersection(center().pos(), center().vel(), edge[0],
                                    edge[1], towardsN)) {
          frameForce += (glm::length(center().vel()) *
                         glm::normalize(edge[2] - edge[3])) *
                        edgeWeight;
        }
      }
    }
    finalForce += frameForce;
    finalVel += frameVel;

    // Obstacle avoidance
    glm::vec3 obForce(0.0f, 0.0f, 0.0f);
    for (auto &ob: obs) {
      obForce += avoidForce(*this, &ob);
    }

    // TODO: generate this force by inspecting next target! (_path.back())
    // Normal
    glm::vec3 tarForce(0.0f, 0.0f, 0.0f);
    if (glm::distance(center().pos(), _path.back().pos()) <= 0.5f && _path.size() > 1) {
      _path.pop_back();
    }
    //else {
      //if (!_path.empty())
        //tarForce += (_path.back().pos() - center().pos()) / (0.05f + glm::length(center().vel()));
      //tarForce += (glm::normalize(_path.back().pos() - center().pos()) * targetSpeed - center().vel());
    //}

    tarForce +=
        (glm::normalize(_path.back().pos() - center().pos()) * targetSpeed -
         center().vel());
    // Smooth
    //finalForce += tarForce;

    finalForce = separationForce * separationWeight + 
      + cohesionForce * cohesionWeight + alignmentForce * alignmentWeight
      + obForce * obWeight + tarForce;
    finalVel = glm::vec3(0.0f, 0.0f, 0.0f);

    /*
    std::cout << glm::to_string(frameForce) << " " << glm::to_string(obForce)
              << glm::to_string(tarForce) << std::endl;
    */
    //finalForce = separationForce + alignmentForce + cohesionForce;
    update(dt, finalForce, finalVel);
  }

  void Agent::findPathByPRM(const std::vector<glm::vec3> &waypoints,
                            const std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                            const std::vector<Obstacle> &obs,
                            unsigned int target,
                            const std::string mode)
  {
    unsigned int src = 0;
    float minDist = FLT_MAX;
    for (unsigned int i = 0; i < waypoints.size(); ++i) {
      float curDist = glm::distance(center().pos(), waypoints[i]);
      if (curDist < minDist && !pathObIntersection(center().pos(), waypoints[i], obs)) {
        src = i;
        minDist = curDist;
      }
    }

    std::vector<std::unordered_map<unsigned int, float>> vertDist(waypoints.size());
    for (unsigned int i = 0;i < waypoints.size(); ++i) {
      vertDist[i] = std::unordered_map<unsigned int, float>{};
      for (auto &edge: graph[i])
        vertDist[i][edge.first] = edge.second;
    }

    using DistType = std::pair<float, unsigned int>;
    std::vector<unsigned int> route(waypoints.size(), waypoints.size());
    std::vector<float> distance(waypoints.size(), FLT_MAX);
    std::priority_queue<DistType, std::vector<DistType>, std::greater<DistType>> openList;
    std::unordered_set<unsigned int> closedList;

    distance[src] = 0.0f;
    openList.push(std::make_pair(0.0f, src));

    double duration;
    std::clock_t start = std::clock();
    unsigned int statVertCount = 0;
    unsigned int statEdgeCount = 0;

    bool end = false;
    while (!openList.empty() && !end) {
      unsigned int curIndex = openList.top().second;
      openList.pop();

      if (closedList.find(curIndex) == closedList.end())
        closedList.insert(curIndex);
      else
        continue;

      ++statVertCount;
      //std::cout << curIndex << std::endl;

      if (curIndex == target) {
        //std::cout << "Break! Target: " << target << std::endl;
        break;
      }

      for (auto &edge : graph[curIndex]) {
        ++statEdgeCount;

        if (distance[curIndex] + vertDist[curIndex][edge.first] < distance[edge.first]) {
          route[edge.first] = curIndex;
          distance[edge.first] = distance[curIndex] + vertDist[curIndex][edge.first];

          if (mode == "AStar")
            openList.push(std::make_pair(distance[edge.first] + glm::distance(waypoints[edge.first], waypoints[target]), edge.first));
          else
            openList.push(std::make_pair(distance[edge.first], edge.first));

          // std::cout << "** cur: " << curIndex << ", next: "<< edge.first
          //<< ", dist: " << routeDist[edge.first] << std::endl;
        }

          //if (edge.first == target) {
          //end = true;
          //break;
          //}
      }
    }

    duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "Path finding with " << mode << ": "
              << duration << " seconds to find the path." << std::endl;
    std::cout << statVertCount << " vertices and "
              << statEdgeCount << " edges have been checked." << std::endl;

    // Move path to agent's path vector
    _path.clear();
    unsigned int index = target;
    if (route[target] != waypoints.size()) {
      while (index != src) {
        _path.push_back(waypoints[index]);
        index = route[index];
      }
      _path.push_back(waypoints[index]);
    } else {
      //_path.push_back(waypoints[src]);
      //_path.push_back(waypoints[src]);
    }
  }



  // stride - move by stride / threshold - dist to goal for termination
  void Agent::findPathByRRT(std::vector<glm::vec3> &waypoints,
                            std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                            const std::vector<Obstacle> &obs,
                            const Rectangle &frame, glm::vec3 target,
                            float sampleProb, float stride,
                            unsigned int maxTimes, float threshold,
                            bool rrtstar)
  {
    if (waypoints.size() < 1)
      return;

    float startX = frame.vertices()[0].pos().x + radius(),
      endX = frame.vertices()[1].pos().x - radius(),
      startY = frame.vertices()[3].pos().y + radius(),
      endY = frame.vertices()[0].pos().y - radius();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> disX(startX, endX), disY(startY, endY), disSample(0, 1);
    glm::vec3 sample(0.0f, 0.0f, 0.0f);
    std::vector<unsigned int> route;

    // For RRT*
    bool RRTStar = rrtstar;
    float nearestRadius = 3 * stride;
    std::vector<float> cost;

    for (unsigned int i = 0;i < waypoints.size(); ++i) {
      cost.push_back(0.0f);
      route.push_back(0);
    }

    bool targetFound = false;
    for (unsigned int i = 0;i < maxTimes; ++i) {
      // Sample
      if (disSample(gen) < sampleProb) {
        sample.x = disX(gen);
        sample.y = disY(gen);
      } else {
        sample.x = target.x;
        sample.y = target.y;
      }

      // Find nearest point in the current tree
      unsigned int index = nearestOnePointBF(sample, waypoints);
      glm::vec3 newPoint = stride * glm::normalize(sample - waypoints[index]) + waypoints[index];

      // Restrict new point to stay in the frame
      if (newPoint.x > endX)
        newPoint = abs(endX) * glm::normalize(newPoint);
      if (newPoint.x < startX)
        newPoint = abs(startX) * glm::normalize(newPoint);
      if (newPoint.y > endY)
        newPoint = abs(endY) * glm::normalize(newPoint);
      if (newPoint.y < startY)
        newPoint = abs(startY) * glm::normalize(newPoint);

      // RRT Star
      if (RRTStar){
        // Reselect Parent
        std::vector<std::pair<unsigned int, float>> neighbors;
        for (unsigned int j = 0; j < waypoints.size(); ++j) {
          float curDist = glm::distance(waypoints[j], newPoint);
          if (curDist <= nearestRadius && !pathObIntersection(waypoints[j], newPoint, obs))
            neighbors.push_back(std::pair<unsigned int, float>{ j, curDist });
        }

        if (!neighbors.empty()) {
          unsigned int newParent = index;
          float minCost = FLT_MAX;
          float curDist = 0.0f;
          for (unsigned int j = 0; j < neighbors.size(); ++j) {
            float curCost = cost[neighbors[j].first] + neighbors[j].second;
            if (curCost < minCost) {
              minCost = curCost;
              newParent = neighbors[j].first;
              curDist = neighbors[j].second;
            }
          }

          waypoints.push_back(newPoint);
          route.push_back(newParent);
          cost.push_back(minCost);

          // Add new edge for the new point
          graph[newParent].push_back(
              std::pair<unsigned int, float>{waypoints.size() - 1, curDist});
          graph.push_back(
              std::vector<std::pair<unsigned int, float>>{{newParent, curDist}});

          // Rewire
          for (unsigned int j = 0; j < neighbors.size(); ++j) {
            if (cost[neighbors[j].first] > minCost + neighbors[j].second) {

              // Remove old edge from the graph
              int tobeRemoved = -1;
              for (;tobeRemoved < graph[route[neighbors[j].first]].size(); ++tobeRemoved)
                if (graph[route[neighbors[j].first]][tobeRemoved].first == neighbors[j].first)
                  break;
              if (tobeRemoved >= 0)
                graph[route[neighbors[j].first]].erase(graph[route[neighbors[j].first]].begin() + tobeRemoved);

              tobeRemoved = -1;
              for (;tobeRemoved < graph[neighbors[j].first].size(); ++tobeRemoved)
                if (graph[neighbors[j].first][tobeRemoved].first == route[neighbors[j].first])
                  break;
              if (tobeRemoved >= 0)
                graph[neighbors[j].first].erase(graph[neighbors[j].first].begin() + tobeRemoved);

              // Add new edge to the graph
              graph[neighbors[j].first].push_back(std::pair<unsigned int, float>{
                  waypoints.size() - 1, neighbors[j].second});
              graph[waypoints.size() - 1].push_back(std::pair<unsigned int, float>{
                  neighbors[j].first, neighbors[j].second});

              cost[neighbors[j].first] = neighbors[j].second + minCost;
              route[neighbors[j].first] = route.size() - 1;
            }
          }
        }
      } else {
        // Normal RRT
        if (!pathObIntersection(waypoints[index], newPoint, obs)) {
          float curDist = glm::distance(waypoints[index], newPoint);
          waypoints.push_back(newPoint);
          route.push_back(index);
          graph[index].push_back(
              std::pair<unsigned int, float>{waypoints.size() - 1, curDist});
          graph.push_back(
              std::vector<std::pair<unsigned int, float>>{{index, curDist}});

          if (glm::distance(target, newPoint) <= threshold) {
            targetFound = true;
            break;
          }
        }
      }
    }

    _path.clear();
    //_path.push_back(Vertex(target));

    unsigned int index = 0;
    if (targetFound) {
      index = waypoints.size() - 1;
    } else {
      float minDist = FLT_MAX;
      for (unsigned int i = 0;i < waypoints.size(); ++i) {
        if (glm::distance(target, waypoints[i]) < minDist) {
          minDist = glm::distance(target, waypoints[i]);
          index = i;
        }
      }
    }

    while(index > 0) {
      _path.push_back(Vertex(waypoints[index]));
      index = route[index];
    }
    _path.push_back(Vertex(waypoints.front()));
  }

  // true for at least one intersection, false for no intersection
  bool Agent::pathObIntersection(glm::vec3 start, glm::vec3 end,
                                 const std::vector<Obstacle> &obs)
  {
    for (auto &ob: obs) 
      if (checkSegCirIntersection(start, end, ob.center().pos(), radius() + ob.radius()))
        return true;
    return false;
  }

  void Agent::smoothPath(const std::vector<Obstacle> &obs)
  {
    std::vector<Vertex>::iterator it = _path.begin();
    for (; it != _path.end(); ++it) {
      if (!pathObIntersection(center().pos(), it->pos(), obs)) {
        ++it;
        break;
      }
    }

    if (it != _path.end())
      _path.erase(it, _path.end());
  }

}
