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

void framebuffer_size_cb(GLFWwindow *window, int width, int height);
void loadShader(const char *vertexShaderName, const char *fragmentShaderName,
                unsigned int &vertShader, unsigned int &fragShader,
                unsigned int &shaderProg);
void processInput(GLFWwindow *window);
void scrollCallback(GLFWwindow *window, double xOffset, double yOffset);
void mouseMoveCallback(GLFWwindow *window, double xPos, double yPos);
//void mouseButtonCallback(GLFWwindow *window, int, int, int);
void keyCallback(GLFWwindow*, int, int, int, int);

void convertVertices(Vertex**, float *);
void render();
unsigned int drawCircle(float *, float, float, float, unsigned int);
unsigned int convertVertices(
  float **, float *, unsigned int, float *, unsigned int,
  float *, unsigned int, float*, unsigned int,
  std::vector<std::pair<float, float>>&,
  std::vector<std::vector<std::pair<unsigned int, float>>>&,
  std::vector<unsigned int> &);
unsigned int reDrawAgent(float *, float *, unsigned int);
//void computePhysics(Vertex**, float*, float, unsigned int, unsigned int, unsigned int);
void computePhysics(float*, float*, float*, float*, float,
                    std::stack<unsigned int>&, std::vector<std::pair<float, float>>&);

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

const float sampleNeighborRadius = 5.0f;
const unsigned int frameVertCount = 4, obVertCount = 360,
  agentVertCount = 360, maskVertCount = 360, samplePointCount = 200;
//unsigned int totalVertCount = 2 * (frameVertCount + obVertCount + agentVertCount) + samplePointCount, totalDrawVertCount = vertAttrLen * totalVertCount;
unsigned int totalVertCount = 0, totalDrawVertCount = 0;

// Shapes
const float agentR = 0.5f, agentX = -9.0f, agentY = -9.0f,
  targetX = 9.0f, targetY = 9.0f,
  obR = 2, obX = 0, obY = 0,
  maskR = agentR + obR, maskX = obX, maskY = obY;
const float agentSpeed = 0.05f;
//float agentX = -9, agentY = -9;

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
  //float *frameVertices = new float[frameVertCount * 2];
  float curAgentX = agentX, curAgentY = agentY, curTarX = targetX, curTarY = targetY;
  float *agentVertices = new float[agentVertCount * 3 * 2];
  float *obVertices = new float[obVertCount * 3 * 2];
  float frameVertices[] = {
      // Frame
      -10.0f, 10.0f,  0.0f,
      10.0f,  10.0f,  0.0f,

      10.0f,  10.0f,  0.0f,
      10.0f,  -10.0f, 0.0f,

      10.0f,  -10.0f, 0.0f,
      -10.0f, -10.0f, 0.0f,

      -10.0f, -10.0f, 0.0f,
      -10.0f, 10.0f,  0.0f,
  };

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

  //float *vertices = new float[totalDrawVertCount];
  //for (unsigned int i = 0; i < totalDrawVertCount; ++i)
  //vertices[i] = 0.0f;

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

  float *vertices = NULL;
  unsigned int agentDrawVertCount, obDrawVertCount;
  agentDrawVertCount = drawCircle(agentVertices, agentX, agentY, agentR, agentVertCount);
  obDrawVertCount = drawCircle(obVertices, obX, obY, obR, obVertCount);
  totalVertCount = convertVertices(&vertices,
                                   agentVertices, agentDrawVertCount,
                                   obVertices, obDrawVertCount,
                                   frameVertices, frameVertCount * 2,
                                   samplePoints, samplePointCount,
                                   sampleVector, graphVector,
                                   shortestRoute);
  totalDrawVertCount = vertAttrLen * totalVertCount;

  unsigned int objectLinesRenderCount = agentDrawVertCount + obDrawVertCount + frameVertCount * 2,
    graphVertsRenderCount = sampleVector.size(),
    graphLinesRenderCount = totalVertCount - objectLinesRenderCount - graphVertsRenderCount;

  // Buffers
  unsigned int VAO, VBO;
  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, totalDrawVertCount * sizeof(float), vertices, GL_STREAM_DRAW);

  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertAttrLen * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                        vertAttrLen * sizeof(float), (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE,
                        vertAttrLen * sizeof(float), (void *)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE,
                        vertAttrLen * sizeof(float), (void *)(7 * sizeof(float)));
  glEnableVertexAttribArray(3);

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
    std::cout << "Frame rate: " << frameRate << std::endl;
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
    agentDrawVertCount = drawCircle(agentVertices, curAgentX, curAgentY, agentR, agentVertCount);
    agentDrawVertCount = reDrawAgent(vertices, agentVertices, agentDrawVertCount);
    void *buf_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    memcpy(buf_ptr, vertices, vertAttrLen * agentDrawVertCount * sizeof(float));
    glUnmapBuffer(GL_ARRAY_BUFFER);

    /*
    for (size_t i = 0;i < 368; ++i)
      std::cout << vertices[i*9] << " " << vertices[i*9+1] << " " << vertices[i*9+2] << std::endl;
    exit(0);
    */

    // Render
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, objectLinesRenderCount);
    glDrawArrays(GL_POINTS, objectLinesRenderCount, graphVertsRenderCount);
    glDrawArrays(GL_LINES, objectLinesRenderCount + graphVertsRenderCount, graphLinesRenderCount);

    // Compute Locations
    if (running)
      computePhysics(&curAgentX, &curAgentY, &curTarX, &curTarY, agentSpeed, routeStack, sampleVector);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  delete[] agentVertices;
  delete[] obVertices;
  delete[] samplePoints;
  delete[] vertices;

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

unsigned int drawCircle(float *verts, float x, float y, float r, unsigned int count)
{
  double angle = 2 * M_PI / count;

  for (unsigned int i = 0; i < count; ++i) {
    verts[6*i] = x + r * cos(i * angle);
    verts[6*i+1] = y + r * sin(i * angle);
    verts[6*i+2] = 0.0f;

    verts[6*i+3] = x + r * cos((i+1) * angle);
    verts[6*i+4] = y + r * sin((i+1) * angle);
    verts[6*i+5] = 0.0f;
  }

  return 2 * count;
}

unsigned int convertVertices(
  float **verts,
  float *agent, unsigned int agentCount, 
  float *ob, unsigned int obCount, 
  float *frame, unsigned int frameCount,
  float *samples, unsigned int sampleCount,
  std::vector<std::pair<float, float>> &sampleVec,
  std::vector<std::vector<std::pair<unsigned int, float>>> &graphVec,
  std::vector<unsigned int> &shortestRoute
)
{
  std::vector<std::vector<float>> tmpVerts;

  std::vector<std::pair<float, float>> graphEdgeVerts;
  for (unsigned int i = 0; i < graphVec.size(); ++i) {
    for (unsigned int k = 0; k < graphVec[i].size(); ++k) {
      if (graphVec[i][k].first > i) {
        graphEdgeVerts.push_back(sampleVec[i]);
        graphEdgeVerts.push_back(sampleVec[graphVec[i][k].first]);
      }
    }
  }

  for (unsigned int i = 0;i < agentCount; ++i) {
    std::vector<float> vert{ agent[i*3], agent[i*3+1], agent[i*3+2],
                              1.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f };
    tmpVerts.push_back(vert);
  }

  for (unsigned int i = 0; i < obCount; ++i) {
    std::vector<float> vert{ ob[i*3], ob[i*3+1], ob[i*3+2],
                             0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f };
    tmpVerts.push_back(vert);
  }

  for (unsigned int i = 0; i < frameCount; ++i) {
    std::vector<float> vert{ frame[i*3], frame[i*3+1], frame[i*3+2],
                             0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f };
    tmpVerts.push_back(vert);
  }

  for (auto &sampleVert : sampleVec) {
    std::vector<float> vert{ sampleVert.first, sampleVert.second, 0.0f,
                             0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f };
    tmpVerts.push_back(vert);
  }

  unsigned int curIndex = shortestRoute.size() - 1;
  while(curIndex != 0) {
    std::vector<float> vert1{ sampleVec[curIndex].first, sampleVec[curIndex].second, 0.0f,
                             1.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f };
    tmpVerts.push_back(vert1);

    curIndex = shortestRoute[curIndex];
    std::vector<float> vert2{ sampleVec[curIndex].first, sampleVec[curIndex].second, 0.0f,
                              1.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f };
    tmpVerts.push_back(vert2);
  }

  for (auto &edgeVert : graphEdgeVerts) {
    std::vector<float> vert{ edgeVert.first, edgeVert.second, 0.0f,
                              0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f };
    tmpVerts.push_back(vert);
  }

  unsigned int size = tmpVerts.size();
  *verts = new float[size * vertAttrLen];
  for (unsigned int i = 0;i < size; ++i) {
    (*verts)[i * vertAttrLen + 0] = tmpVerts[i][0];
    (*verts)[i * vertAttrLen + 1] = tmpVerts[i][1];
    (*verts)[i * vertAttrLen + 2] = tmpVerts[i][2];
    (*verts)[i * vertAttrLen + 3] = tmpVerts[i][3];
    (*verts)[i * vertAttrLen + 4] = tmpVerts[i][4];
    (*verts)[i * vertAttrLen + 5] = tmpVerts[i][5];
    (*verts)[i * vertAttrLen + 6] = tmpVerts[i][6];
    (*verts)[i * vertAttrLen + 7] = tmpVerts[i][7];
    (*verts)[i * vertAttrLen + 8] = tmpVerts[i][8];
  }

  return size;
}

unsigned int reDrawAgent(float* verts, float* agent, unsigned int agentCount)
{
  for (unsigned int i = 0; i < agentCount; ++i) {
    verts[i * vertAttrLen] = agent[i * 3];
    verts[i * vertAttrLen + 1] = agent[i * 3 + 1];
    verts[i * vertAttrLen + 2] = agent[i * 3 + 2];
    verts[i * vertAttrLen + 6] = 10.0f;
  }

  return agentCount;
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

  //delete[] h;
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
