#include "common.h"

using namespace NSShape;

// Nearset Neighbors
void nearestPointsBF(float distance, glm::vec3 &curPoint,
                     std::vector<glm::vec3> &points, std::vector<glm::vec3> &result)
{
  for (auto &point: points)
    if (glm::distance(curPoint, point) <= distance)
      result.push_back(point);
}

void nearestShapeBF(float distance, Agent &curAgent, std::vector<Agent> &agents,
                    std::vector<Obstacle> &obs, Rectangle &frame,
                    std::vector<Agent> &resAgents,
                    std::vector<Obstacle> &resObs,
                    std::vector<std::vector<glm::vec3>> &resEdges)
{
  for (auto &agent: agents)
    if (glm::distance(agent.center().pos(), curAgent.center().pos())
        - curAgent.radius() - agent.radius() <= distance)
      resAgents.push_back(agent);

  for (auto &ob: obs)
    if (glm::distance(ob.center().pos(), curAgent.center().pos())
        - 2 * curAgent.radius() - ob.radius() <= distance)
      resObs.push_back(ob);

  std::vector<Vertex> vertices = frame.vertices();
  for (unsigned int i = 0;i < 4; ++i) {
    float dist = (i % 2 == 0) ?
      abs(curAgent.center().pos().y - vertices[i].pos().y) :
      abs(curAgent.center().pos().x - vertices[i].pos().x);

    if (dist - curAgent.radius() <= distance) {
      resEdges.push_back(std::vector{
        vertices[i].pos(), vertices[(i+1)%4].pos(),
        vertices[(i+3)%4].pos(), vertices[i].pos()
      });
    }
  }
}

// true for intersect, false for no intersect (parallel)
bool checkSegCirIntersection(const glm::vec3 &src, const glm::vec3 &tar,
                             const glm::vec3 &obCen, float maskR)
{
  /*
  std::cout<<glm::to_string(src) << " " << glm::to_string(tar) << ", "
           <<glm::to_string(obCen) << ", " << maskR << std::endl;
  */
  if (glm::distance(src, obCen) <= maskR || glm::distance(tar, obCen) <= maskR)
    return true;
  if (glm::distance(src, tar) == 0.0f)
    return false;
  //std::cout << glm::to_string(src) << " " << glm::to_string(tar) << " " << 1 << std::endl;
  if (abs(glm::determinant(glm::dmat2(obCen - src, glm::normalize(tar - src)))) > maskR) {
    return false;
  } else {
    // If the projection point is not on the segment, then add it to the graph edges collection
    glm::vec3 proj = glm::dot(obCen - src, glm::normalize(tar - src)) * glm::normalize(tar - src);

    /*
    std::cout << glm::to_string(src) << " " << glm::to_string(tar) << " " << 2
              << std::endl;
    */

    // TODO: Alternative - | proj - (obCen - src) | > maskR (x Incorrect )

    // Suppose that all points are outside of the circle
    // Cannot handle cases that point is in the circle


    if (glm::dot(proj, tar - src) < 0 ||
        glm::dot(proj, tar - src) > glm::dot(tar - src, tar - src)) {
      //float dist = glm::distance(tar, src);
      //std::cout << glm::dot(proj, tar-src) << " " << glm::dot(tar - src, tar - src) << std::endl;
      return false;
    }

    //if (glm::distance(proj, obCen - src) > maskR)
    //return false;

    //if (glm::distance(proj + src, obCen) > maskR)
    //return false;

    /*
    std::cout << glm::to_string(src) << " " << glm::to_string(tar) << " " << 3
              << std::endl;
    */
  }
  return true;
}

bool checkSegSegIntersection(const glm::vec3 &s1_0, const glm::vec3 &s1_1,
                             const glm::vec3 &s2_0, const glm::vec3 &s2_1,
                             glm::vec3 &intersect)
{
  // TODO: k == 0
  float k1 = (s1_1.y - s1_0.y) / (s1_1.x - s1_0.x), b1 = s1_0.y - k1 * s1_0.x,
    k2 = (s2_1.y - s2_0.y) / (s2_1.x - s2_0.x), b2 = s2_0.y - k2 * s2_0.x;

  if (k1 == k2 || k1 == -1 * k2) {
    return false;
  } else {
    float int_x = (b1 - b2) / (k1 - k2), int_y = k1 * int_x + b1;
    intersect = glm::vec3(int_x, int_y, 0.0f);

    if (glm::dot(intersect - s1_0, s1_1 - s1_0) > 0 &&
        glm::dot(intersect - s1_0, s1_1 - s1_0) < glm::dot(s1_1 - s1_0, s1_1 - s1_0) &&
        glm::dot(intersect - s2_0, s2_1 - s2_0) > 0 &&
        glm::dot(intersect - s2_0, s2_1 - s2_0) < glm::dot(s2_1 - s2_0, s2_1 - s2_0)) {
      return true;
    }
  }

  return false;
}

// segment_n represents the direction leading to collision
bool checkVelSegIntersection(const glm::vec3 &src, const glm::vec3 &vel,
                             const glm::vec3 &s2_0, const glm::vec3 &s2_1,
                             const glm::vec3 &segment_n){
  if (glm::length(vel) == 0)
    return false;

  glm::vec3 e_vel = glm::normalize(vel);
  float dot = glm::dot(e_vel, glm::normalize(s2_1 - s2_0));

  // Vel parallel with the segment
  if (dot == 1 || dot == -1)
    return false;

  // Agent moving towards the wall
  if (glm::dot(e_vel, segment_n) <= 0)
    return false;

  // Cross (e2_0 x e_vel) dot (e2_1 x src) > 0 - src between e2_0 and e2_1
  glm::vec3 e2_0(s2_0.x - src.x, s2_0.y - src.y, 0.0f), e2_1(s2_1.x - src.x, s2_1.y - src.y, 0.0f);
  e2_0 = glm::normalize(e2_0);
  e2_1 = glm::normalize(e2_1);

  // Vel parallel (same direction) with the e2_0 or e2_1 but not parallel with the segment
  if (glm::dot(e_vel, e2_0) == 1 || glm::dot(e_vel, e2_1) == 1)
    return true;

  glm::vec3 e2_0_cross = glm::cross(e2_0, e_vel), e2_1_cross = glm::cross(e2_1, e_vel);
  float dot_cross = glm::dot(e2_0_cross, e2_1_cross);
  if (dot_cross > 0) {
    return false;
  } else {
    return true;
  }
}

// Code from book (code from Prof. Guy)
float timeToCollision(Agent &src, Circle *tar)
{
  // tau = (b +/- sqrt(b*b - a*c)) / a (b = (posb - posa)(va - vb))
  float tau = 0.0f;

  float r = src.radius() + tar->radius();
  glm::vec3 w = tar->center().pos() - src.center().pos(),
    v = src.center().vel() - tar->center().vel();

  float c = glm::dot(w, w) - r * r;
  if (c < 0)
    return 0.0f;

  float a = glm::dot(v, v), b = glm::dot(w, v);
  float discr = b * b - a * c;

  if (discr <= 0)
    return FLT_MAX;
  tau = (b - sqrt(discr)) / a;
  if (tau < 0)
    return FLT_MAX;
  return tau;
}

//
// Nearest point
//

unsigned int nearestOnePointBF(glm::vec3 curPoint, const std::vector<glm::vec3> &points)
{
  unsigned int ret = 0;
  float dist = FLT_MAX;

  for (unsigned int i = 0; i < points.size(); ++i) {
    float curDist = glm::distance(curPoint, points[i]);
    if (curDist < dist) {
      dist = curDist;
      ret = i;
    }
  }

  return ret;
}
