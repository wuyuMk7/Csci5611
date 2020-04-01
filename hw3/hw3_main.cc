#ifdef __APPLE__
#include "glad.h"
#else
#include <GL/glew.h>
#endif

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <stdio.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "hw3.h"

using namespace NSCamera;
using namespace NSShape;
using namespace NSCrowd;

enum VBOIndex { VI_agents, VI_bg, VI_p_rts, VI_r_rts, VI_pts };

void framebuffer_size_cb(GLFWwindow *window, int width, int height);
void loadShader(const char *vertexShaderName, const char *fragmentShaderName,
                unsigned int &vertShader, unsigned int &fragShader,
                unsigned int &shaderProg);
void processInput(GLFWwindow *window);
void scrollCallback(GLFWwindow *window, double xOffset, double yOffset);
void mouseMoveCallback(GLFWwindow *window, double xPos, double yPos);
//void mouseButtonCallback(GLFWwindow *window, int, int, int);
void keyCallback(GLFWwindow*, int, int, int, int);

void render();

unsigned int genSamplePoints(float*, float, float, float, float, float, float, float,  unsigned int);
void genGraph(std::vector<std::pair<float, float>>&,
              std::vector<std::vector<std::pair<unsigned int, float>>>&,
              float, float, float);
void genRoute(std::vector<std::pair<float, float>>&, 
              std::vector<std::vector<std::pair<unsigned int, float>>>&,
              std::vector<unsigned int>&);

// Configuration
const float screenWidth = 800, screenHeight = 600;
bool firstRun = true, mouseBeingPressed = false, running = false;
float mouseLastX = 0.0, mouseLastY = 0.0, mouseCurX = 0.0, mouseCurY = 0.0;
float lastFrameTime = 0.0f, deltaFrameTime = 0.0f, frameRate = 0.0f;
Camera camera(glm::vec3(0.0f, 0.0f, 30.0f));
const unsigned int vertAttrLen = 9;
//Camera camera(glm::vec3(1.0f, 1.0f, 6.0f), glm::vec3(0.0f, 1.0f, 0.0f), -100.0f, -15.0f);

const float sampleNeighborRadius = 20.0f;
const unsigned int frameVertCount = 4, wayPointCount = 3;

unsigned int totalVertCount = 0, totalDrawVertCount = 0;

// Shapes
const float agentR = 0.5f, agentX = -9.0f, agentY = -9.0f,
  targetX = 9.0f, targetY = 9.0f,
  obR = 2, obX = 0, obY = 0,
  maskR = agentR + obR, maskX = obX, maskY = obY;
const float agentSpeed = 0.05f;
//float agentX = -9, agentY = -9;

// Settings
bool path_PRM = true; // true for PRM and false for RRT.
int nearest_neighbor_dist = 20.0f;

// Shapes
float rtRoutes[2700000];
unsigned int maxRtRtPts = 300000;

std::vector<Agent> agents;
std::vector<Obstacle> obstacles;
std::vector<Vertex> targets{ Vertex(glm::vec3(targetX, targetY, 0.0f)) };
Rectangle frame(Vertex(glm::vec3(-10.0f, 10.0f, 0.0f)), Vertex(glm::vec3(10.0f, -10.0f, 0.0f)));


/************ New ***********/

unsigned int drawAgents(std::vector<Agent> &, std::vector<Vertex> &);
unsigned int reDrawAgent(float*, std::vector<Vertex> &);
unsigned int drawObstacles(std::vector<Obstacle> &, std::vector<Vertex> &);
unsigned int convertBGVerts(float **bgPoints,
                            std::vector<Vertex> &frameVerts,
                            std::vector<Vertex> &obstacleVerts);
unsigned int convertAgentVerts(float **agentPoints,
                               std::vector<Vertex> &agentVerts);
unsigned int convertPredRtVerts(float **predRtPoints,
                                std::vector<Vertex> &predRtVerts);
unsigned int convertRealRtVerts(float **realRtPoints,
                                std::vector<Vertex> &realRtVerts);
unsigned int convertPtVerts(float **ptPoints,
                            std::vector<glm::vec3> &waypoints);

unsigned int sampleWaypoints(std::vector<glm::vec3> &waypoints,
                             const std::vector<Obstacle> &obs,
                             const Rectangle &frame,
                             float agentR, unsigned int count);
bool checkSegCirIntersection(const glm::vec3 &, const glm::vec3 &,
                             const glm::vec3 &, float);
bool checkSegSegIntersection(const glm::vec3 &, const glm::vec3 &,
                             const glm::vec3 &, const glm::vec3 &,
                             const glm::vec3 &);
void updateGraph(std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                 std::vector<glm::vec3> &waypoints, const std::vector<glm::vec3> &newpoints,
                 const std::vector<Obstacle> &obs, const Rectangle &frame, float agentR);
unsigned int drawGraphEdges(std::vector<Vertex> &edges,
                            std::vector<glm::vec3> &graphVerts,
                            std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                            glm::vec3 edgeColor = glm::vec3(0.0f, 0.0f, 0.0f));

unsigned int convertBGVerts(float **bgPoints, std::vector<Vertex> &frameVerts,
                            std::vector<Vertex> &obstacleVerts)
{
  unsigned int size = frameVerts.size() + obstacleVerts.size();
  unsigned int ptr = 0;
  *bgPoints = new float[size * vertAttrLen];

  for (unsigned int i = 0; i < frameVerts.size(); ++i)
    ptr += frameVerts[i].flat((*bgPoints) + ptr);

  for (unsigned int i = 0; i < obstacleVerts.size(); ++i)
    ptr += obstacleVerts[i].flat((*bgPoints) + ptr);

  return size;
}

unsigned int convertAgentVerts(float **agentPoints,
                               std::vector<Vertex> &agentVerts)
{
  unsigned int size = agentVerts.size();
  *agentPoints = new float[size * vertAttrLen];

  for (unsigned int i = 0, ptr = 0; i < agentVerts.size(); ++i)
    ptr += agentVerts[i].flat((*agentPoints) + ptr);

  return size;
}

unsigned int convertPredRtVerts(float **predRtPoints,
                                std::vector<Vertex> &predRtVerts)
{
  unsigned int size = predRtVerts.size();
  *predRtPoints = new float[size * vertAttrLen];

  for (unsigned int i = 0, ptr = 0; i < predRtVerts.size(); ++i)
    ptr += predRtVerts[i].flat((*predRtPoints) + ptr);

  return size;
}

unsigned int convertRealRtVerts(float **realRtPoints,
                                std::vector<Vertex> &realRtVerts)
{
  return 0;
}

unsigned int convertPtVerts(float **ptPoints,
                            std::vector<glm::vec3> &waypoints)
{
  unsigned int size = waypoints.size();
  *ptPoints = new float[size * vertAttrLen];

  unsigned int ptr = 0;
  Vertex curWayPtVert(glm::vec3(0.0f, 0.0f, 0.0f));
  for (auto &waypoint: waypoints) {
    curWayPtVert.pos(waypoint.x, waypoint.y, waypoint.z);
    ptr += curWayPtVert.flat((*ptPoints) + ptr);
  }

  return size;
}

unsigned int drawAgents(std::vector<Agent> &agents,
                        std::vector<Vertex> &points)
{
  unsigned int size = 0;
  for (auto &agent : agents) {
    size += agent.draw(points);
  }

  return size;
}

unsigned int drawObstacles(std::vector<Obstacle> &obstacles,
                           std::vector<Vertex> &points)
{
  unsigned int size = 0;
  for (auto &ob: obstacles) {
    size += ob.draw(points);
  }

  return size;
}

unsigned int reDrawAgent(float *verts, std::vector<Vertex> &agentPoints)
{
  for (unsigned int i = 0, j = 0; i < agentPoints.size();++i)
    j += agentPoints[i].flat(verts + j);

  return agentPoints.size();
}

unsigned int sampleWaypoints(std::vector<glm::vec3> & waypoints,
                             const std::vector<Obstacle> &obs,
                             const Rectangle &frame,
                             float agentR, unsigned int count){
  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<Vertex> frameVerts = frame.vertices();
  std::uniform_real_distribution<float> disX(frameVerts[0].pos().x + agentR, frameVerts[2].pos().x - agentR), disY(frameVerts[0].pos().y - agentR, frameVerts[2].pos().y + agentR);

  glm::vec3 mask(0.0f, 0.0f, 0.0f), sample(0.0f, 0.0f, 0.0f);
  float maskR = 0.0f;

  for (unsigned int i = 0; i < count; ++i) {
    sample.x = disX(gen);
    sample.y = disY(gen);

    // TODO: may update the code to drop points in the obstacle area instead of moving them out
    // Ensure obstacles intialized with large gap between each other ()
    for (auto &ob: obs) {
      // TODO: check the point with bounding box instead of using propertities from a fixed shape
      mask.x = ob.center().pos().x;
      mask.y = ob.center().pos().y;
      maskR = ob.radius() + agentR;

      if (sample.x >= mask.x - maskR && sample.x <= mask.x + maskR &&
          sample.y >= mask.y - maskR && sample.y <= mask.y + maskR) {
        sample = mask + glm::normalize(sample - mask) * (maskR + 0.01f);
      }

    }
    waypoints.push_back(sample);
  }

  return waypoints.size();
}

bool checkSegCirIntersection(const glm::vec3 &src, const glm::vec3 &tar,
                             const glm::vec3 &obCen, float maskR)
{
  /*
  std::cout<<glm::to_string(src) << " " << glm::to_string(tar) << ", "
           <<glm::to_string(obCen) << ", " << maskR << std::endl;
  */

  if (abs(glm::determinant(glm::dmat2(obCen - src, glm::normalize(tar - src)))) > maskR) {
    return false;
  } else {
    // If the projection point is not on the segment, then add it to the graph edges collection
    glm::vec3 proj = glm::dot(obCen - src, glm::normalize(tar - src)) * glm::normalize(tar - src);

    // TODO: Alternative - | proj - (obCen - src) | > maskR
    if (glm::dot(proj, tar - src) < 0 ||
        glm::dot(proj, tar - src) > glm::dot(tar - src, tar - src)) {
      //float dist = glm::distance(tar, src);
      return false;
    }
  }
  return true;
}

bool checkSegSegIntersection(const glm::vec3 &s1_0, const glm::vec3 &s1_1,
                             const glm::vec3 &s2_0, const glm::vec3 &s2_1,
                             glm::vec3 &intersect)
{
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



void updateGraph(std::vector<std::vector<std::pair<unsigned int, float>>> & graph,
                 std::vector<glm::vec3> & waypoints,
                 const std::vector<glm::vec3> &newpoints,
                 const std::vector<Obstacle> &obs, const Rectangle &frame,
                 float agentR) {
  unsigned int oldSize = waypoints.size(), index = oldSize;

  float dist = 0.0f;
  for (auto &newpoint: newpoints) {
    waypoints.push_back(newpoint);
    graph.push_back(std::vector<std::pair<unsigned int, float>>{});
    /* Brute Force */
    for (unsigned int i = 0;i < waypoints.size(); ++i) {
      dist = glm::distance(newpoint, waypoints[i]);
      if (dist > 0 && dist <= nearest_neighbor_dist) {
        for (auto &ob: obs) {
          if (!checkSegCirIntersection(newpoint, waypoints[i],
                                       ob.center().pos(), ob.radius() + agentR)) {
            graph[index].push_back(std::pair<unsigned int, float>{i, dist});
            graph[i].push_back(std::pair<unsigned int, float>{index, dist});
          }
        }
      }
    }
    /**************/
    ++index;
  }
}

unsigned int drawGraphEdges(std::vector<Vertex>& edges,
                               std::vector<glm::vec3> &graphVerts,
                               std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                               glm::vec3 edgeColor)
{
  for (unsigned int i = 0;i < graph.size(); ++i) {
    for (auto &pair: graph[i]) {
      if (pair.first > i) {
        edges.push_back(Vertex(graphVerts[i], edgeColor, 20.0f, 1.0f));
        edges.push_back(Vertex(graphVerts[pair.first], edgeColor, 20.0f, 1.0f));
      }
    }
  }

  return edges.size();
}

/******************n*********/

int main(int argc, char* argv[])
{
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  GLFWwindow* window = glfwCreateWindow(800, 600, "Homework 3 Checkin (Press ENTER to start/pause)", NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
  glfwSetCursorPosCallback(window, mouseMoveCallback);
  //glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetScrollCallback(window, scrollCallback);
  glfwSetKeyCallback(window, keyCallback);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glfwSetFramebufferSizeCallback(window, framebuffer_size_cb);
  glEnable(GL_DEPTH_TEST);

  // Shapes & Routes
  float curAgentX = agentX, curAgentY = agentY, curTarX = targetX, curTarY = targetY;
  agents.push_back(Agent(Vertex(glm::vec3(curAgentX, curAgentY, 0.0f)), agentR));
  //agents.push_back(Agent(Vertex(glm::vec3(curAgentX + 0.5f, curAgentY + 0.5f, 0.0f)), agentR));
  //agents.push_back(Agent(Vertex(glm::vec3(curAgentX+1.0f, curAgentY+1.0f, 0.0f)), agentR));
  obstacles.push_back(Obstacle(Vertex(glm::vec3(obX, obY, 0.0f)), obR));
  //obstacles.push_back(Obstacle(Vertex(glm::vec3(obX+4.0f, obY+4.0f, 0.0f)), obR));

  Crowd crowd(agents, targets);
  // Generate sample points for roadmap
  std::vector<glm::vec3> waypointsVec;
  sampleWaypoints(waypointsVec, obstacles, frame, agentR, wayPointCount);

  std::vector<glm::vec3> graphVerts, tmpGraphVerts(waypointsVec);
  std::vector<std::vector<std::pair<unsigned int, float>>> graph;
  updateGraph(graph, graphVerts, waypointsVec, obstacles, frame, agentR);

  /*
  for (unsigned int i = 0;i < graph.size(); ++i) {
    std::cout << glm::to_string(graphVerts[i]) << ": " << std::endl;
    for (auto &pair: graph[i]) {
      std::cout << glm::to_string(graphVerts[pair.first]) << " " << pair.second << std::endl;
    }
    std::cout << std::endl;
  }
  */

  /*
  float *samplePoints = new float[samplePointCount * 3];
  genSamplePoints(samplePoints, agentX, agentY, targetX, targetY,
                  maskX, maskY, maskR, samplePointCount);

  std::vector<std::pair<float, float>> sampleVector;
  sampleVector.push_back(std::pair<float, float>{ curAgentX, curAgentY });
  for (unsigned int i = 0;i < samplePointCount; ++i) 
    sampleVector.push_back(std::pair<float, float>{ samplePoints[i*3], samplePoints[i*3+1]});
  sampleVector.push_back(std::pair<float, float>{ targetX, targetY });

  std::vector<std::vector<std::pair<unsigned int, float>>> graphVector(sampleVector.size(), std::vector<std::pair<unsigned int, float>>());
  genGraph(sampleVector, graphVector, maskX, maskY, maskR);

  std::vector<unsigned int> shortestRoute(sampleVector.size());
  std::stack<unsigned int> routeStack;
  genRoute(sampleVector, graphVector, shortestRoute);

  //std::cout << "size: " << sampleVector.size() << std::endl;
  routeStack.push(sampleVector.size() - 1);
  for (unsigned int routeVert = shortestRoute.back(); routeVert != 0;) {
    routeStack.push(routeVert);
    //std::cout << routeVert << ", pos x: " << sampleVector[routeVert].first
    //<< ", y: " << sampleVector[routeVert].second << std::endl;
    routeVert = shortestRoute[routeVert];
  }
  curTarX = sampleVector[routeStack.top()].first;
  curTarY = sampleVector[routeStack.top()].second;
  routeStack.pop();
  */

  // Generate geometry vertices and points for opengl
  unsigned int bgVertCount = 0, bgVertArrCount = 0,
    agentVertCount = 0, agentVertArrCount = 0,
    predVertCount = 0, predVertArrCount = 0,
    realVertCount = 0, realVertArrCount = 0,
    pointVertCount = 0, pointVertArrCount = 0;
  float *bgPoints, *agentPoints, *predPoints, *realPoints, *ptPoints;

  unsigned int obVertCount = 0, frameVertCount = 0, graphEdgeVertCount;
  std::vector<Vertex> agentVerts, frameVerts, obVerts, predRtVerts, realRtVerts, ptVerts,
    graphEdgeVerts;
  frameVertCount = frame.renderPoints(frameVerts);
  agentVertCount = drawAgents(agents, agentVerts);
  obVertCount = drawObstacles(obstacles, obVerts);

  // TODO: Merge predverts and graphverts! (when displaying the graph)
  graphEdgeVertCount = drawGraphEdges(graphEdgeVerts, graphVerts, graph);

  bgVertCount = convertBGVerts(&bgPoints, frameVerts, obVerts);
  bgVertArrCount = bgVertCount * vertAttrLen;
  agentVertCount = convertAgentVerts(&agentPoints, agentVerts);
  agentVertArrCount = agentVertCount * vertAttrLen;
  // TODO: Use predVerts instead (in the following statement)
  predVertCount = convertPredRtVerts(&predPoints, graphEdgeVerts);
  predVertArrCount = predVertCount * vertAttrLen;
  pointVertCount = convertPtVerts(&ptPoints, waypointsVec);
  pointVertArrCount = pointVertCount * vertAttrLen;

  // Buffers
  unsigned int VBOs[5];
  glGenBuffers(5, VBOs);
  glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_agents]);
  glBufferData(GL_ARRAY_BUFFER, agentVertArrCount * sizeof(float), agentPoints, GL_STREAM_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_bg]);
  glBufferData(GL_ARRAY_BUFFER, bgVertArrCount * sizeof(float), bgPoints, GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_p_rts]);
  glBufferData(GL_ARRAY_BUFFER, predVertArrCount * sizeof(float), predPoints, GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_r_rts]);
  glBufferData(GL_ARRAY_BUFFER, realVertArrCount * sizeof(float), realPoints, GL_STREAM_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_pts]);
  glBufferData(GL_ARRAY_BUFFER, pointVertArrCount * sizeof(float), ptPoints, GL_STATIC_DRAW);

  unsigned int VAOs[5];
  glGenVertexArrays(5, VAOs);
  for (unsigned int i = 0;i < 5; ++i) {
    glBindVertexArray(VAOs[i]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[i]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertAttrLen * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertAttrLen * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, vertAttrLen * sizeof(float),
                          (void *)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, vertAttrLen * sizeof(float),
                          (void *)(7 * sizeof(float)));
    glEnableVertexAttribArray(3);
  }
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // Shader & Camera Settings
  int success = 0;
  char infoLog[512];
  unsigned int vertexShader, fragmentShader, shaderProgram;
  loadShader("hw3.vert", "hw3.frag", vertexShader, fragmentShader, shaderProgram);
  glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
    std::cout << "ShaderProgram failed:" << infoLog << std::endl;
  }

  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  GLint modelID = glGetUniformLocation(shaderProgram, "model");
  GLint viewID = glGetUniformLocation(shaderProgram, "view");
  GLint projectionID = glGetUniformLocation(shaderProgram, "projection");
  glm::mat4 model(1.0f), view(1.0f), projection(1.0f);

  //glEnable(GL_LINE_WIDTH);
  glEnable(GL_PROGRAM_POINT_SIZE);
  while(!glfwWindowShouldClose(window)) {
    float currentFrameTime = glfwGetTime();
    deltaFrameTime = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;
    frameRate = 1 / deltaFrameTime;
    //std::cout << "Frame rate: " << frameRate << std::endl;
    camera.speed(2.5f);

    processInput(window);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shaderProgram);

    // Set model view projection matrix
    view = camera.view();
    projection = glm::perspective(glm::radians(camera.zoom()),
                                  (float)screenWidth / (float)screenHeight,
                                  0.1f, 100.0f);
    glUniformMatrix4fv(viewID, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projectionID, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(modelID, 1, GL_FALSE, glm::value_ptr(model));

    // Re-draw Shapes & Render
    // agentDrawVertCount = drawAgent(agentVertices, curAgentX, curAgentY, agentR, agentVertCount);
    // agentDrawVertCount = reDrawAgent(vertices, agentVertices, agentDrawVertCount);

    //agentPoints.clear();
    //agentDrawVertCount = drawAgents(agents, agentPoints);
    //agentDrawVertCount = reDrawAgent(vertices, agentPoints);
    //void *buf_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    //memcpy(buf_ptr, vertices, vertAttrLen * agentDrawVertCount * sizeof(float));
    //glUnmapBuffer(GL_ARRAY_BUFFER);

    // Render
    glBindVertexArray(VAOs[0]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_agents]);
    glDrawArrays(GL_LINES, 0, agentVertCount);

    glBindVertexArray(VAOs[1]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_bg]);
    glDrawArrays(GL_LINES, 0, bgVertCount);

    glBindVertexArray(VAOs[2]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_p_rts]);
    glDrawArrays(GL_LINES, 0, predVertCount);

    glBindVertexArray(VAOs[3]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_r_rts]);
    glDrawArrays(GL_LINES, 0, realVertCount);

    glBindVertexArray(VAOs[4]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_pts]);
    glDrawArrays(GL_POINTS, 0, pointVertCount);

    //glDrawArrays(GL_LINES, 0, objectLinesRenderCount);
    /*
    glDrawArrays(GL_LINES, 0, objectLinesRenderCount);
    glDrawArrays(GL_POINTS, objectLinesRenderCount, graphVertsRenderCount);
    glDrawArrays(GL_LINES, objectLinesRenderCount + graphVertsRenderCount, graphLinesRenderCount);
    */

    // Compute Locations
    if (running)
      //computePhysics(&curAgentX, &curAgentY, &curTarX, &curTarY, agentSpeed, routeStack, sampleVector);
      ;

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  //delete[] samplePoints;


  if (bgPoints) delete[] bgPoints;
  if (agentPoints) delete[] agentPoints;
  if (predPoints) delete[] predPoints;
  if (realPoints) delete[] realPoints;
  if (ptPoints) delete[] ptPoints;

  glfwTerminate();

  return 0;
}

void framebuffer_size_cb(GLFWwindow *window, int width, int height)
{
  glViewport(0, 0, width, height);
}

void processInput(GLFWwindow *window)
{
  /* WSAD control */
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    camera.moveForward(deltaFrameTime);
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    camera.moveBackward(deltaFrameTime);
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    camera.moveLeft(deltaFrameTime);
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    camera.moveRight(deltaFrameTime);

  // Camera control - to replace mouse movement control
  if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS)
    camera.pitchAndYaw(1.0, 0.0);
  if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS)
    camera.pitchAndYaw(-1.0, 0.0);
  if (glfwGetKey(window, GLFW_KEY_9) == GLFW_PRESS)
    camera.pitchAndYaw(0.0, 1.0);
  if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS)
    camera.pitchAndYaw(0.0, -1.0);
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
  if (key == GLFW_KEY_ENTER && action  == GLFW_PRESS)
    running = (running ? false : true);
}


void scrollCallback(GLFWwindow *window, double xOffset, double yOffset)
{
  camera.zoomInOut(yOffset);
}

void mouseMoveCallback(GLFWwindow *window, double xPos, double yPos)
{
  /*
  if (firstRun) {
    mouseLastX = xPos;
    mouseLastY = yPos;
    firstRun = false;
  }

  double xOffset = xPos - mouseLastX, yOffset = mouseLastY - yPos;
  mouseLastX = xPos;
  mouseLastY = yPos;

  camera.pitchAndYaw(xOffset, yOffset);
  */
  mouseCurX = xPos;
  mouseCurY = yPos;
}

void computePhysics(float *curX, float *curY, float *tarX, float *tarY,
                    float speed, std::stack<unsigned int> &route,
                    std::vector<std::pair<float, float>> &verts)
{
  glm::vec2 cur(*curX, *curY), tar(*tarX, *tarY);

  if (*curX < targetX && *curY < targetY) {
    cur += speed * glm::normalize(tar - cur);
    *curX = cur.x;
    *curY = cur.y;

    if (*curX >= *tarX && *curY >= *tarY) {
      *curX = *tarX;
      *curY = *tarY;
      if (!route.empty()) {
        *tarX = verts[route.top()].first;
        *tarY = verts[route.top()].second;
        route.pop();
      }
    }
  }
}

unsigned int genSamplePoints(float* samples,
                          float startX, float startY,
                          float endX, float endY,
                          float maskX, float maskY, float maskR,
                          unsigned int count)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> disX(startX, endX), disY(startY, endY);
  glm::vec2 mask(maskX, maskY), sample(0.0f, 0.0f);

  for (unsigned int i = 0;i < count; ++i) {
    samples[i*3] = disX(gen);
    samples[i*3+1] = disY(gen);

    if (samples[i*3] >= maskX - maskR && samples[i*3] <= maskX + maskR &&
        samples[i*3+1] >= maskY - maskR && samples[i*3+1] <= maskY + maskR) {
      sample.x = samples[i*3];
      sample.y = samples[i*3+1];

      sample = (sample - mask) / glm::distance(sample, mask) * (maskR + 0.01f);
      samples[i*3] = sample.x;
      samples[i*3+1] = sample.y;
    }
  }

  return count;
}

void genGraph(std::vector<std::pair<float, float>>& samples,
              std::vector<std::vector<std::pair<unsigned int, float>>>& graph,
              float maskX, float maskY, float maskR)
{
  glm::vec2 mask(maskX, maskY), src(0.0f, 0.0f), tar(0.0f, 0.0f), proj(0.0f, 0.0f);

  for (unsigned int i = 0;i < samples.size(); ++i) {
    src.x = samples[i].first;
    src.y = samples[i].second;
    for (unsigned int j = i+1;j < samples.size(); ++j) {
      //if (j == i) continue;

      tar.x = samples[j].first;
      tar.y = samples[j].second;

      /*
      std::cout << "src x, src y, tar x, tar y: " << glm::to_string(src) << " "
                << glm::to_string(tar) << glm::to_string(mask) << std::endl;
      std::cout << glm::to_string(glm::dmat2(mask-src, tar-src)) << std::endl;
      std::cout << glm::determinant(glm::dmat2(glm::normalize(mask - src), tar - src)) << std::endl;
      */
      float dist = glm::distance(tar, src);
      if (dist > sampleNeighborRadius)
        continue;

      if (abs(glm::determinant(glm::dmat2(mask - src, glm::normalize(tar - src)))) > maskR) {
        //float dist = glm::distance(tar, src);
        graph[i].push_back(std::pair<unsigned int, float>{j, dist});
        graph[j].push_back(std::pair<unsigned int, float>{i, dist});

        /*
        std::cout << "src x, src y, tar x, tar y: " << glm::to_string(src)
                  << " " << glm::to_string(tar) << glm::to_string(mask)
                  << std::endl;
        */

      } else {
        // If the projection point is not on the segment, then add it to the graph edges collection
        proj = glm::dot(mask - src, glm::normalize(tar - src)) * glm::normalize(tar - src);

        if (glm::dot(proj, tar - src) < 0 ||
            glm::dot(proj, tar - src) > glm::dot(tar - src, tar - src)) {
          //float dist = glm::distance(tar, src);
          graph[i].push_back(std::pair<unsigned int, float>{j, dist});
          graph[j].push_back(std::pair<unsigned int, float>{i, dist});
        }
      }
    }
  }
}

void genRoute(std::vector<std::pair<float, float>> &verts, 
              std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
              std::vector<unsigned int> &route)
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

  auto distLam = [](const std::pair<float, float> &p1,
                    const std::pair<float, float> &p2) {
    float xDist = p1.first - p2.first, yDist = p1.second - p2.second;
    return sqrt(xDist * xDist + yDist * yDist);
  };

  //float *h = new float[verts.size()],
  float *routeDist = new float[verts.size()];
  for (unsigned int i = 0;i < verts.size(); ++i) {
    //h[i] = distLam(verts[i], verts.back());
    //h[i] = 0;
    routeDist[i] = FLT_MAX;
    route[i] = verts.size();
  }
  routeDist[0] = 0;

  std::priority_queue<F> toBeSearched;
  toBeSearched.push(F(0, 0));

  bool end = false;
  while (!toBeSearched.empty() && !end) {
    unsigned int curIndex = toBeSearched.top().vert;
    toBeSearched.pop();

    for (auto &edge : graph[curIndex]) {
      if (routeDist[edge.first] == FLT_MAX)
        //toBeSearched.push(F(edge.first, edge.second + h[edge.first]));
        // F(n) = G(n) + H(n)
        toBeSearched.push(F(edge.first, edge.second + distLam(verts[edge.first], verts.back())));

      if (routeDist[curIndex] + edge.second < routeDist[edge.first]) {
        route[edge.first] = curIndex;
        routeDist[edge.first] = routeDist[curIndex] + edge.second;
        //std::cout << "** cur: " << curIndex << ", next: "<< edge.first
        //<< ", dist: " << routeDist[edge.first] << std::endl;
      }

      if (edge.second == verts.size() - 1) {
        end = true;
        break;
      }
    }
  }

  delete[] routeDist;
}

void loadShader(
  const char* vertexShaderName, const char* fragmentShaderName,
  unsigned int& vertShader, unsigned int& fragShader, unsigned int& shaderProg
)
{
  std::ifstream vertexShaderFStream(vertexShaderName),
    fragmentShaderFStream(fragmentShaderName);
  std::stringstream vertexShaderStream, fragmentShaderStream;

  vertexShaderStream << vertexShaderFStream.rdbuf();
  fragmentShaderStream << fragmentShaderFStream.rdbuf();

  const string *vertexStr = new string(vertexShaderStream.str());
  const string *fragStr = new string(fragmentShaderStream.str());
  // const char *vertexShaderSource = vertexShaderStream.str().c_str();
  // const char *fragmentShaderSource = fragmentShaderStream.str().c_str();
  const char *vertexShaderSource = vertexStr->c_str();
  const char *fragmentShaderSource = fragStr->c_str();

  //std::cout << vertexShaderSource;
  //std::cout << fragmentShaderSource;

  vertShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertShader, 1, &vertexShaderSource, NULL);
  glCompileShader(vertShader);

  fragShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragShader, 1, &fragmentShaderSource, NULL);
  glCompileShader(fragShader);

  shaderProg = glCreateProgram();
  glAttachShader(shaderProg, vertShader);
  glAttachShader(shaderProg, fragShader);
  glLinkProgram(shaderProg);

  delete vertexStr;
  delete fragStr;
}
