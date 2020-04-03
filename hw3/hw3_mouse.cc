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
void mouseButtonCallback(GLFWwindow *window, int, int, int);
void keyCallback(GLFWwindow*, int, int, int, int);

// Configuration
const float screenWidth = 800, screenHeight = 600;
bool firstRun = true, mouseBeingPressed = false, running = false;
float mouseLastX = 0.0, mouseLastY = 0.0, mouseCurX = 0.0, mouseCurY = 0.0;
float lastFrameTime = 0.0f, deltaFrameTime = 0.0f, frameRate = 0.0f;
Camera camera(glm::vec3(0.0f, 0.0f, 30.0f));
const unsigned int vertAttrLen = 9;

const float sampleNeighborRadius = 8.0f;
const unsigned int frameVertCount = 4, wayPointCount = 200;

unsigned int totalVertCount = 0, totalDrawVertCount = 0;

// Shapes
const float agentR = 0.2f, agentX = -9.0f, agentY = -9.0f,
  targetX = 9.0f, targetY = 9.0f,
  obR = 2, obX = 0, obY = 0,
  maskR = agentR + obR, maskX = obX, maskY = obY;
const float agentSpeed = 0.05f;
//float agentX = -9, agentY = -9;

// Settings
bool path_PRM = true; // true for PRM and false for RRT.

// Scene 1 for normal PRM Boid
// Scene 2 for PRM with mouse based target change
// Scene 3 for PRM + target change + obstacle
// Scene 4 for A* with normal ones (time & times to reach dest)
// Scene 5 for RRT
// Scene 6 for RRT*
// Scene 7 for smoothing path with RRT
unsigned int scene = 1;
bool scenePrep = false;
bool replan = false;
glm::vec3 replanToTarget(0.0f, 0.0f, 0.0f);

float nearest_neighbor_dist = 20.0f;
unsigned int maximumAgentCount = 25;

// Shapes
float rtRoutes[2700000];
unsigned int maxRtRtPts = 300000;

std::vector<Agent> agents;
std::vector<Obstacle> obstacles;
std::vector<Vertex> targets;
Crowd crowd(agents, targets);

Rectangle frame(Vertex(glm::vec3(-10.0f, 10.0f, 0.0f)), Vertex(glm::vec3(10.0f, -10.0f, 0.0f)));

/************ New ***********/

void initializeAgents(std::vector<Agent> &);
void initializeObstacles(std::vector<Obstacle> &);


unsigned int drawAgents(std::vector<Agent> &, std::vector<Vertex> &);
unsigned int reDrawAgent(float*, std::vector<Vertex> &);
unsigned int drawObstacles(std::vector<Obstacle> &, std::vector<Vertex> &);
unsigned int convertBGVerts(float **bgPoints,
                            std::vector<Vertex> &frameVerts,
                            std::vector<Vertex> &obstacleVerts);
unsigned int convertAgentVerts(float **agentPoints,
                               std::vector<Vertex> &agentVerts);
unsigned int convertPredRtVerts(float **predRtPoints,
                                std::vector<Vertex> &predRtVerts,
                                std::vector<Agent> &agents);
unsigned int convertRealRtVerts(float **realRtPoints,
                                std::vector<Vertex> &realRtVerts);
unsigned int convertPtVerts(float **ptPoints,
                            std::vector<glm::vec3> &waypoints);

unsigned int sampleWaypoints(std::vector<glm::vec3> &waypoints,
                             const std::vector<Obstacle> &obs,
                             const Rectangle &frame,
                             float agentR, unsigned int count);
void updateGraph(std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                 std::vector<glm::vec3> &waypoints, const std::vector<glm::vec3> &newpoints,
                 const std::vector<Obstacle> &obs, const Rectangle &frame, float agentR);
unsigned int drawGraphEdges(std::vector<Vertex> &edges,
                            std::vector<glm::vec3> &graphVerts,
                            std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                            glm::vec3 edgeColor = glm::vec3(0.0f, 0.0f, 0.0f));

int main(int argc, char* argv[])
{
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "Homework 3 (Press ENTER to start/pause)", NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
  glfwSetCursorPosCallback(window, mouseMoveCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetScrollCallback(window, scrollCallback);
  glfwSetKeyCallback(window, keyCallback);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glfwSetFramebufferSizeCallback(window, framebuffer_size_cb);
  glEnable(GL_DEPTH_TEST);

  // Shapes & Routes
  initializeAgents(agents);

  obstacles.push_back(Obstacle(Vertex(glm::vec3(0.0f, 3.5f, 0.0f)), obR));
  obstacles.push_back(Obstacle(Vertex(glm::vec3(0.0f, -3.5f, 0.0f)), obR));
  obstacles.push_back(Obstacle(Vertex(glm::vec3(5.0f, 0.0f, 0.0f)), obR));
  obstacles.push_back(Obstacle(Vertex(glm::vec3(5.0f, 5.0f, 0.0f)), obR - 1.0f));
  obstacles.push_back(Obstacle(Vertex(glm::vec3(5.0f, -5.0f, 0.0f)), obR - 1.0f));
  obstacles.clear();

  obstacles.push_back(Obstacle(Vertex(glm::vec3(0.0f, 0.0f, 0.0f)), obR));

  targets.push_back(Vertex(glm::vec3(7.0f, 7.0f, 0.0f)));

  //Crowd crowd(agents, targets);

  // Generate sample points for roadmap
  std::vector<glm::vec3> waypointsVec;
  for (auto &agent:crowd.agents())
    waypointsVec.push_back(agent.center().pos());

  sampleWaypoints(waypointsVec, obstacles, frame, agentR, wayPointCount);

  std::vector<glm::vec3> graphVerts, tmpGraphVerts(waypointsVec);
  std::vector<std::vector<std::pair<unsigned int, float>>> graph;
  updateGraph(graph, graphVerts, waypointsVec, obstacles, frame, agentR);

  //crowd.genPath(graphVerts, graph, obstacles, frame, "RRT");
  crowd.genPath(graphVerts, graph, obstacles, frame, "PRM");

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
  predVertCount = convertPredRtVerts(&predPoints, graphEdgeVerts, agents);
  predVertArrCount = predVertCount * vertAttrLen;
  //pointVertCount = convertPtVerts(&ptPoints, waypointsVec);
  pointVertCount = convertPtVerts(&ptPoints, graphVerts);
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
    if (replan) {
      replan = false;
      targets.clear();
      targets.push_back(Vertex(replanToTarget));
      for (auto &agent: agents)
        agent.clearPath();
      crowd.genPath(graphVerts, graph, obstacles, frame, "PRM");
    }

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

    // Render
    glBindVertexArray(VAOs[0]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_agents]);
    glDrawArrays(GL_LINES, 0, agentVertCount);

    glBindVertexArray(VAOs[1]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_bg]);
    glDrawArrays(GL_LINES, 0, bgVertCount);

    /*
    glBindVertexArray(VAOs[2]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_p_rts]);
    glDrawArrays(GL_LINES, 0, predVertCount);

    glBindVertexArray(VAOs[3]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_r_rts]);
    glDrawArrays(GL_LINES, 0, realVertCount);

    glBindVertexArray(VAOs[4]);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_pts]);
    glDrawArrays(GL_POINTS, 0, pointVertCount);
    */

    // Compute Locations
    if (running)
      crowd.simulate(3 * agentR, obstacles, frame, "boid", true);

    // Re-draw Shapes & Render
    agentVerts.clear();
    agentVertCount = drawAgents(agents, agentVerts);
    agentVertCount = reDrawAgent(agentPoints, agentVerts);

    glBindBuffer(GL_ARRAY_BUFFER, VBOs[VI_agents]);
    void *buf_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    memcpy(buf_ptr, agentPoints, vertAttrLen * agentVertCount * sizeof(float));
    glUnmapBuffer(GL_ARRAY_BUFFER);

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

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    double xPos, yPos;
    glfwGetCursorPos(window, &xPos, &yPos);
    std::cout << xPos << ", " << yPos << std::endl;

    /*
    // Camera settings
    glm::mat4 model(1.0f), view(1.0f), projection(1.0f);
    view = camera.view();
    projection = glm::perspective(glm::radians(camera.zoom()),
                                  (float)screenWidth / (float)screenHeight,
                                  0.1f, 100.0f);
    */

    // TODO: Check scene here
    /*
    glm::vec3 mouseSpacePos(0.0f, 0.0f, 0.0f);
    mouseSpacePos.x = (xPos - screenWidth / 2) / 24.0f;
    mouseSpacePos.y = (screenHeight / 2 - yPos) / 24.0f;
    */

    replan = true;
    replanToTarget.x = (xPos - screenWidth / 2)/ 24.0f;
    replanToTarget.y = (screenHeight / 2 - yPos) / 24.0f;
    replanToTarget.z = 0;

    //std::cout << glm::to_string(mouseSpacePos) << std::endl;
    /*
    glm::vec4 result = projection * view * model * mouseSpacePos;
    std::cout << glm::to_string(result) << std::endl;
    */
  }
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
// Set initial velocity here!
void initializeAgents(std::vector<Agent> &agents)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<Vertex> frameVerts = frame.vertices();
  std::uniform_real_distribution<float> disX(frameVerts[0].pos().x + agentR,
                                             frameVerts[0].pos().x + 30 * agentR),
      disY(frameVerts[3].pos().y + agentR, frameVerts[3].pos().y + 30 * agentR);

  for (unsigned int i = 0; i < maximumAgentCount - 5;) {
    Agent agent(Vertex(glm::vec3(disX(gen), disY(gen), 0.0f)), agentR);
    //agent.vertices()[0].vel(-disX(gen)/10, -disY(gen)/10, 0.0f);
    bool insert = true;
    for (auto &cur: agents) {
      if (glm::distance(agent.center().pos(), cur.center().pos()) <= 2 * agentR) {
        insert = false;
        break;
      }
    }
    if (insert) {
      ++i;
      agents.push_back(agent);
    }
  }
}

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

  for (unsigned int i = 0;i < size * vertAttrLen; ++i)
    (*agentPoints)[i] = 0.0f;

  for (unsigned int i = 0, ptr = 0; i < agentVerts.size(); ++i)
    ptr += agentVerts[i].flat((*agentPoints) + ptr);

  return size;
}

unsigned int convertPredRtVerts(float **predRtPoints,
                                std::vector<Vertex> &predRtVerts,
                                std::vector<Agent> &agents)
{
  std::vector<Vertex> pathVerts;
  unsigned int i = 0;
  for (auto &agent: agents) {
    //std::cout << "Agent " << i++ << " : " << glm::to_string(agent.center().pos()) << std::endl;
    for (unsigned int i = 0;i < agent.path().size() - 1; ++i) {
      //std::cout << glm::to_string(agent.path()[i].pos()) << std::endl;
      pathVerts.push_back(agent.path()[i]);
      pathVerts.push_back(agent.path()[i+1]);
    }
  }

  unsigned int size = predRtVerts.size() + pathVerts.size();
  //unsigned int size = predRtVerts.size();
  //unsigned int size = pathVerts.size();
  *predRtPoints = new float[size * vertAttrLen];

  unsigned int ptr = 0;

  for (unsigned int i = 0; i < pathVerts.size(); ++i) {
    ptr += pathVerts[i].flat((*predRtPoints) + ptr);
  }

  for (unsigned int i = 0;i < predRtVerts.size(); ++i) {
    Vertex curV(predRtVerts[i]);
    curV.color(0.8f, 0.8f, 0.8f);
    ptr += curV.flat((*predRtPoints) + ptr);
  }


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

unsigned int reDrawAgent(float *agentPoints, std::vector<Vertex> &agentVerts)
{
  for (unsigned int i = 0, ptr = 0; i < agentVerts.size();++i)
    ptr += agentVerts[i].flat(agentPoints + ptr);

  return agentVerts.size();
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
        bool noIntersection = true;
        for (auto &ob: obs) {
          if (checkSegCirIntersection(newpoint, waypoints[i],
                                       ob.center().pos(), ob.radius() + agentR)) {
            noIntersection = false;
            break;
          }
        }
        if (noIntersection) {
          graph[index].push_back(std::pair<unsigned int, float>{i, dist});
          graph[i].push_back(std::pair<unsigned int, float>{index, dist});
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
