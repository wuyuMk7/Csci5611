#include "agent.h"
#include "common.h"

namespace NSShape
{
  float tH = 10, maxAvoidF = 300.0f;
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
    std::cout << forceMag << std::endl;
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
    float separationWeight = 3.0f, alignmentWeight = 2.0f,
          cohesionWeight = 3.0f, obWeight = 3.0f, edgeWeight = 1.0f,
          steerWeight = 5.0f;

    glm::vec3 separationForce(0.0f, 0.0f, 0.0f), cohesionForce(0.0f, 0.0f, 0.0f),
      alignmentForce(0.0f, 0.0f, 0.0f), finalForce(0.0f, 0.0f, 0.0f), finalVel(0.0f, 0.0f, 0.0f);
    float seCount = 0.0f, alCount = 0.0f, coCount = 0.0f, colCount = 0.0f;

    glm::vec3 colAvoidVel(0.0f, 0.0f, 0.0f);
    for (auto &agent: agents) {
      glm::vec3 relPos = center().pos() - agent.center().pos();
      float curDist = glm::length(relPos) - 2 * radius();

      /*
      if (curDist <= 0 && curDist != -2 * radius()) {
        //colAvoidVel += (center().vel() + 1.0f * glm::dot(center().vel(), glm::normalize(relPos)) * glm::normalize(relPos));
        //colCount += 1;

        Vertex newCenter(center());
        glm::vec3 newPos = agent.center().pos() + 2.0f * radius() * glm::normalize(relPos);
        newCenter.pos(newPos.x, newPos.y, newPos.z);
        center(newCenter);
      }
      */

      // Separation
      if (curDist > 0.01f && curDist <= separationDist ) {
        //separationForce += (glm::normalize(relPos) / (curDist));
        //seCount += 1;
        separationForce += avoidForce(*this, &agent) * separationWeight;
      }

      // Alignment && Cohesion
      if (curDist > 0) {
        alignmentForce += agent.center().vel();
        alCount += 1;

        cohesionForce += agent.center().pos();
        coCount += 1;
      }
    }

    // Handler
    if (seCount > 0 && glm::length(separationForce) > 0) {
      //separationForce = separationForce / seCount * separationWeight;
      separationForce = separationForce;
    }
    finalForce += separationForce;

    if (alCount > 0 && glm::length(alignmentForce) > 0) {
      glm::vec3 alDirection = glm::normalize(alignmentForce);
      glm::vec3 curDirection = center().vel();
      if (glm::length(curDirection) > 0) {
        curDirection = glm::normalize(curDirection);
      }
      alignmentForce = (glm::length(alignmentForce) / alCount * (alDirection - curDirection)) * alignmentWeight;
    }
    finalForce += alignmentForce;


    if (coCount > 0 && glm::length(cohesionForce) > 0) {
      glm::vec3 coDirection = glm::normalize(cohesionForce);
      glm::vec3 curDirection = center().pos();
      if (glm::length(curDirection) > 0) {
        curDirection = glm::normalize(curDirection);
      }
      cohesionForce = (glm::length(cohesionForce - curDirection) / coCount * (coDirection - curDirection)) * cohesionWeight;
    }
    finalForce += cohesionForce;

    // TODO: divded by colCount
    if (colCount > 0 && glm::length(colAvoidVel) > 0) {
      finalVel += (glm::length(colAvoidVel) / colCount) * glm::normalize(colAvoidVel);
      //finalForce += (glm::length(colAvoidVel) / colCount) * glm::normalize(colAvoidVel);
    }

    /*
      std::cout << glm::to_string(diff) << " "
          << glm::to_string(glm::normalize(diff)) << " " << curDist
          << " " << glm::to_string(force) << std::endl;
    */
    glm::vec3 frameForce(0.0f, 0.0f, 0.0f), frameVel(0.0f, 0.0f, 0.0f);
    for (auto &edge: nnEdges) {
      float curDist = glm::length(glm::cross(center().pos() - edge[0], glm::normalize(edge[1] - edge[0])));
      glm::vec3 towardsN = glm::normalize(edge[3]-edge[2]);
      //std::cout << glm::to_string(edge[0]) << " ," << glm::to_string(edge[1]) << ", " << curDist << std::endl;
      if (curDist <= radius()) {
        glm::vec3 vV = glm::dot(center().vel(), towardsN) * towardsN;
        /*
        if (curDist > 0)
          frameForce += (-towardsN / curDist) * edgeWeight;
        else
          frameForce += (-towardsN * 100.0f) * edgeWeight;
        */

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

    //std::cout << glm::to_string(ob.center().pos())
    //std::cout << glm::to_string(center().pos()) << std::endl;
    //std::cout << radius() << " " << ob.radius() << " " << pureDist << std::endl;

    glm::vec3 obForce(0.0f, 0.0f, 0.0f), obVel(0.0f, 0.0f, 0.0f);
    for (auto &ob: obs) {
      obForce += avoidForce(*this, &ob);
      continue;
      glm::vec3 connDirection = ob.center().pos() - center().pos();
      float curDist = glm::length(connDirection);
      float pureDist = curDist - radius() - ob.radius();

      // TODO: Check POS and VEL here. The velocity may not be correct.
      if (pureDist <= 0) {
        obVel += (-1.5f * center().vel());
        continue;
      }

      if (glm::dot(center().vel(), connDirection) > 0) {
        glm::vec3 proj = (glm::dot(connDirection, glm::normalize(center().vel()))) * glm::normalize(center().vel());
        //float distToProj = glm::length(glm::cross(connDirection, proj));
        float distToProj = glm::length(proj - connDirection);
        if (distToProj <= radius() + ob.radius()) {
          glm::vec3 newDirection = glm::normalize(proj - connDirection);
          if (distToProj == 0) {
            if (center().vel().x == 0)
              newDirection = glm::vec3(-1.0f, 0.0f, 0.0f);
            else if (center().vel().y == 0)
              newDirection = glm::vec3(0.0f, 1.0f, 0.0f);
            else
              newDirection = glm::normalize(glm::vec3(1.0f, -(center().vel().x / center().vel().y) , 0.0f));
          }

          obForce += (glm::length(center().vel()) / pureDist * newDirection) * obWeight;
          //obVel += (center().vel() * newDirection) * 2.0f;
          //obVel += (glm::length(center().vel()) * newDirection);
        }
      }
    }
    finalForce += obForce;
    //finalVel += obVel;

    // TODO: generate this force by inspecting next target! (_path.back())
    // finalForce += glm::vec3(9.0f, 9.0f, 0.0f);
    // finalForce += (glm::vec3(9.0f, 9.0f, 0.0f) - center().pos()) / (1.0f +
    // glm::length(center().vel()));
    // Normal
    glm::vec3 tarForce(0.0f, 0.0f, 0.0f);
    if (glm::distance(center().pos(), _path.back().pos()) <= 0.2f && _path.size() > 1) {
      _path.pop_back();
    } else {
      //if (!_path.empty())
        //tarForce += (_path.back().pos() - center().pos()) / (0.05f + glm::length(center().vel()));
      tarForce += steerWeight * (glm::normalize(_path.back().pos() - center().pos()) * targetSpeed - center().vel());
    }
    // Smooth
    //finalForce += tarForce;

    finalForce = separationForce + cohesionForce + alignmentForce + obForce + tarForce;
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
                            unsigned int target,
                            const std::string mode)
  {
    struct F
    {
      unsigned int vert;
      float dist;

      F(unsigned int vert, float dist): vert(vert), dist(dist) {};
      bool operator < (F d) const
      {
        return this->dist < d.dist;
      }
    };

    // Regardless of the obstacles. Just select the nearest waypoint as the starting point.
    unsigned int src = 0;
    float minDist = FLT_MAX;
    for (unsigned int i = 0; i < waypoints.size(); ++i) {
      float curDist = glm::distance(center().pos(), waypoints[i]);
      if (curDist < minDist) {
        src = i;
        minDist = curDist;
      }
    }

    float *routeDist = new float[waypoints.size()], *route = new float[waypoints.size()];
    for (unsigned int i = 0;i < waypoints.size(); ++i) {
      routeDist[i] = FLT_MAX;
      route[i] = waypoints.size();
    }
    routeDist[src] = 0;

    std::priority_queue<F> toBeSearched;
    toBeSearched.push(F(src, 0));

    bool end = false;
    while (!toBeSearched.empty() && !end) {
      unsigned int curIndex = toBeSearched.top().vert;
      toBeSearched.pop();

      if (curIndex == target)
        break;

      for (auto &edge : graph[curIndex]) {
        if (routeDist[edge.first] == FLT_MAX)
        //toBeSearched.push(F(edge.first, edge.second + h[edge.first]));
        // F(n) = G(n) + H(n)
          toBeSearched.push(F(edge.first, edge.second + glm::distance(waypoints[edge.first], waypoints[target])));

        if (routeDist[curIndex] + edge.second < routeDist[edge.first]) {
          route[edge.first] = curIndex;
          routeDist[edge.first] = routeDist[curIndex] + edge.second;
        //std::cout << "** cur: " << curIndex << ", next: "<< edge.first
        //<< ", dist: " << routeDist[edge.first] << std::endl;
        }

        /*
        if (edge.first == target) {
          end = true;
          break;
        }
        */
      }
    }

    // Move path to agent's path vector
    _path.clear();
    unsigned int index = target;
    while(index != src) {
      _path.push_back(waypoints[index]);
      index = route[index];
    }
    _path.push_back(waypoints[index]);

    delete[] route;
    delete[] routeDist;
  }
}
