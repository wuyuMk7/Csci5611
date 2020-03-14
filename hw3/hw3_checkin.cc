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

void convertVertices(Vertex**, float *);
void render();
unsigned int drawCircle(float *, float, float, float, unsigned int);
unsigned int convertVertices(
  float *, float *, unsigned int, float *, unsigned int,
  float *, unsigned int, float*, unsigned int);
unsigned int reDrawAgent(float*, float*, unsigned int);
void computePhysics(Vertex**, float*, float, unsigned int, unsigned int, unsigned int);

unsigned int genSamplePoints(float*, float, float, float, float, float, float, float,  unsigned int);

// Configuration
const float screenWidth = 800, screenHeight = 600;
bool firstRun = true, mouseBeingPressed = false;
float mouseLastX = 0.0, mouseLastY = 0.0, mouseCurX = 0.0, mouseCurY = 0.0;
float lastFrameTime = 0.0f, deltaFrameTime = 0.0f, frameRate = 0.0f;
Camera camera(glm::vec3(0.0f, 0.0f, 30.0f));
const unsigned int vertAttrLen = 9;
//Camera camera(glm::vec3(1.0f, 1.0f, 6.0f), glm::vec3(0.0f, 1.0f, 0.0f), -100.0f, -15.0f);

const unsigned int frameVertCount = 4, obVertCount = 360,
  agentVertCount = 360, maskVertCount = 360, samplePointCount = 30;
const unsigned int totalVertCount = 2 * (frameVertCount + obVertCount + agentVertCount) + samplePointCount, totalDrawVertCount = vertAttrLen * totalVertCount;

// Shapes
const float agentR = 0.5f, agentX = -9.0f, agentY = -9.0f, targetX = 9.0f, targetY = 9.0f,
  obR = 2, obX = 0, obY = 0,
  maskR = agentR + obR, maskX = obX, maskY = obY;
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

  GLFWwindow* window = glfwCreateWindow(800, 600, "Homework 3 Checkin", NULL, NULL);
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

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glfwSetFramebufferSizeCallback(window, framebuffer_size_cb);
  glEnable(GL_DEPTH_TEST);

  // Shapes & Routes
  //float *frameVertices = new float[frameVertCount * 2];
  float *agentVertices = new float[agentVertCount * 3 * 2];
  float curAgentX = agentX, curAgentY = agentY;

  float *obVertices = new float[obVertCount * 3 * 2];

  /*
  float frameVertices[] = {
    // Frame
    -10.0f, 10.0f,  0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f,
    10.0f,  10.0f,  0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f,

    10.0f,  10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f,
    10.0f, -10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f,

    10.0f, -10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f,
    -10.0f, -10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f,

    -10.0f, -10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f,
    -10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f,
  };
  */

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

  float *vertices = new float[totalDrawVertCount];
  for (unsigned int i = 0; i < totalDrawVertCount; ++i)
    vertices[i] = 0.0f;

  /*
  // Obstacle
  obX, obY, 0.0f, 0.0f, 0.0f, 0.0f, obR, 0.0f, 0.0f,
  // Agent
  agentX, agentY, 0.0f, 0.0f, 0.0f, 0.0f, agentR, 0.0f, 0.0f,
  */

  unsigned int VAO, VBO;
  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, totalDrawVertCount * sizeof(float), vertices, GL_STATIC_DRAW);

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

  // Shader Settings
  int success = 0;
  char infoLog[512];
  unsigned int vertexShader, fragmentShader, shaderProgram;
  loadShader("hw3.vert", "hw3.frag", vertexShader, fragmentShader, shaderProgram);
  glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
    std::cout << "ShaderProgram failed:" << infoLog << std::endl;
  }
  GLint modelID = glGetUniformLocation(shaderProgram, "model");
  GLint viewID = glGetUniformLocation(shaderProgram, "view");
  GLint projectionID = glGetUniformLocation(shaderProgram, "projection");

  glm::mat4 model(1.0f), view(1.0f), projection(1.0f);

  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

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

    // Rendering
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

    // Draw Shapes
    unsigned int agentDrawVertCount, obDrawVertCount;
    agentDrawVertCount = drawCircle(agentVertices, agentX, agentY, agentR, agentVertCount);
    obDrawVertCount = drawCircle(obVertices, obX, obY, obR, obVertCount);
    convertVertices(vertices,
                    agentVertices, agentDrawVertCount,
                    obVertices, obDrawVertCount,
                    frameVertices, frameVertCount * 2,
                    samplePoints, samplePointCount);

    void *buf_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    memcpy(buf_ptr, vertices, totalDrawVertCount * sizeof(float));
    glUnmapBuffer(GL_ARRAY_BUFFER);

    /*
    for (size_t i = 0;i < 368; ++i)
      std::cout << vertices[i*9] << " " << vertices[i*9+1] << " " << vertices[i*9+2] << std::endl;
    exit(0);
    */

    // Render
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, totalVertCount - samplePointCount);
    glDrawArrays(GL_POINTS, totalVertCount - samplePointCount, samplePointCount);

    // Compute Locations


    /*
    for (size_t count = 0; count < 50; ++count) {
      computePhysics(clothVertices, vertices, deltaFrameTime / 50,
                     clothVertexCount * clothVertexStepLen, clothWidth,
                     clothHeight);
    }
    */

    /*
    void *buf_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for (int i = 0;i < sizeof(vertices) / sizeof(float); ++i)
      std::cout << *((float *)buf_ptr + i) << std::endl;
    glUnmapBuffer(GL_ARRAY_BUFFER);
    exit(0);
    */

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
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);

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
  float *verts,
  float *agent, unsigned int agentCount, 
  float *ob, unsigned int obCount, 
  float *frame, unsigned int frameCount,
  float *samples, unsigned int sampleCount
)
{
  unsigned int size = vertAttrLen * (agentCount + obCount + frameCount);

  for (unsigned int i = 0;i < agentCount; ++i) {
    verts[i * vertAttrLen] = agent[i * 3];
    verts[i * vertAttrLen + 1] = agent[i * 3 + 1];
    verts[i * vertAttrLen + 2] = agent[i * 3 + 2];
    verts[i * vertAttrLen + 6] = 10.0f;
  }

  for (unsigned int i = 0, j = agentCount; i < obCount; ++i, ++j) {
    verts[j * vertAttrLen] = ob[i * 3];
    verts[j * vertAttrLen + 1] = ob[i * 3 + 1];
    verts[j * vertAttrLen + 2] = ob[i * 3 + 2];
    verts[j * vertAttrLen + 6] = 10.0f;
  }

  for (unsigned int i = 0, j = agentCount + obCount; i < frameCount; ++i, ++j) {
    verts[j * vertAttrLen] = frame[i * 3];
    verts[j * vertAttrLen + 1] = frame[i * 3 + 1];
    verts[j * vertAttrLen + 2] = frame[i * 3 + 2];
    verts[j * vertAttrLen + 6] = 10.0f;
  }

  for (unsigned int i = 0, j = agentCount + obCount + frameCount; i < sampleCount; ++i, ++j) {
    verts[j * vertAttrLen] = samples[i * 3];
    verts[j * vertAttrLen + 1] = samples[i * 3 + 1];
    verts[j * vertAttrLen + 2] = samples[i * 3 + 2];
    verts[j * vertAttrLen + 6] = 10.0f;
  }

  /*
  for (unsigned int i = 0;i < frameCount; ++i) {
    verts[i * vertAttrLen] = frame[i * 3];
    verts[i * vertAttrLen + 1] = frame[i * 3 + 1];
    verts[i * vertAttrLen + 2] = frame[i * 3 + 2];
    verts[i * vertAttrLen + 6] = 10.0f;
  }

  for (unsigned int i = 0, j = frameCount; i < obCount; ++i, ++j) {
    verts[j * vertAttrLen] = ob[i * 3];
    verts[j * vertAttrLen + 1] = ob[i * 3 + 1];
    verts[j * vertAttrLen + 2] = ob[i * 3 + 2];
    verts[j * vertAttrLen + 6] = 10.0f;
  }

  for (unsigned int i = 0, j = frameCount + obCount; i < agentCount; ++i, ++j) {
    verts[j * vertAttrLen] = agent[i * 3];
    verts[j * vertAttrLen + 1] = agent[i * 3 + 1];
    verts[j * vertAttrLen + 2] = agent[i * 3 + 2];
    verts[j * vertAttrLen + 6] = 10.0f;
  }
  */

  return size;
}

void computePhysics(Vertex **clothVertices, float* vertices, float dt,
                    unsigned int shaderVLength, unsigned int clothWidth, unsigned int clothHeight)
{
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

      sample = (sample - mask) / glm::distance(sample, mask) * (maskR + 0.001f);
      samples[i*3] = sample.x;
      samples[i*3+1] = sample.y;
    }
  }

  return count;
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

  std::cout << vertexShaderSource;
  std::cout << fragmentShaderSource;

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
