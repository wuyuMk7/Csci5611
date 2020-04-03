#ifndef __COMMON_H__
#define __COMMON_H__

#include "glm/glm.hpp"
#include "agent.h"

void nearestPointsBF(float distance, glm::vec3 &curPoint,
                     std::vector<glm::vec3> &points, std::vector<glm::vec3> &result);
void nearestShapeBF(float distance, NSShape::Agent &curAgent,
                    std::vector<NSShape::Agent> &agents,
                    std::vector<NSShape::Obstacle> &obs,
                    NSShape::Rectangle &frame,
                    std::vector<NSShape::Agent> &resAgents,
                    std::vector<NSShape::Obstacle> &resObs,
                    std::vector<std::vector<glm::vec3>> &resEdges);
bool checkSegCirIntersection(const glm::vec3 &, const glm::vec3 &,
                             const glm::vec3 &, float);
bool checkSegSegIntersection(const glm::vec3 &, const glm::vec3 &,
                             const glm::vec3 &, const glm::vec3 &,
                             const glm::vec3 &);
bool checkVelSegIntersection(const glm::vec3 &src, const glm::vec3 &vel,
                             const glm::vec3 &s2_0, const glm::vec3 &s2_1,
                             const glm::vec3 &segment_n);
float timeToCollision(NSShape::Agent &src, NSShape::Circle *tar);

unsigned int nearestOnePointBF(glm::vec3, const std::vector<glm::vec3> &);

#endif
