#ifdef __APPLE__
#include <glad/glad.h>
#else
#include <GL/glew.h>
#endif

#include <stdio.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "camera.h"

using namespace NSCamera;

void Camera::zoomInOut(double zoom)
{
  if (_zoom >= 1.0f && _zoom <= 45.0f)
    _zoom -= zoom;
  if (_zoom <= 1.0f)
    _zoom = 1.0f;
  if (_zoom >= 45.0f)
    _zoom = 45.0f;
}

void Camera::pitchAndYaw(double xOffset, double yOffset)
{
  xOffset *= _sensitivity;
  yOffset *= _sensitivity;

  _yaw += xOffset;
  _pitch += yOffset;

  if (_pitch > 89.0f)
    _pitch = 89.0f;
  if (_pitch < -89.0f)
    _pitch = -89.0f;

  updateVectors();
}

void Camera::updateVectors()
{
  // Front vector
  _front.x = cos(glm::radians(_pitch)) * cos(glm::radians(_yaw));
  _front.y = sin(glm::radians(_pitch));
  _front.z = cos(glm::radians(_pitch)) * sin(glm::radians(_yaw));
  _front = glm::normalize(_front);

  // Right vector
  _right = glm::normalize(glm::cross(_front, _globalUp));

  // Up vector
  _up = glm::normalize(glm::cross(_right, _front));
}

// #define __TEST__

// Configuration


#ifdef __TEST__

using std::string;

void framebuffer_size_cb(GLFWwindow *window, int width, int height);
void loadShader(const char *vertexShaderName, const char *fragmentShaderName,
                unsigned int &vertShader, unsigned int &fragShader,
                unsigned int &shaderProg);
void processInput(GLFWwindow *window);
void scrollCallback(GLFWwindow *window, double xOffset, double yOffset);
void mouseMoveCallback(GLFWwindow *window, double xPos, double yPos);

const float screenWidth = 800, screenHeight = 600;
bool firstRun = true;
float mouseLastX= 0.0, mouseLastY = 0.0;
float lastFrameTime = 0.0f, deltaFrameTime = 0.0f, frameRate = 0.0f;
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));

int main(int argc, char *argv[]) {
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "Camera Test", NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  glfwSetCursorPosCallback(window, mouseMoveCallback);
  glfwSetScrollCallback(window, scrollCallback);
  
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glfwSetFramebufferSizeCallback(window, framebuffer_size_cb);

  glEnable(GL_DEPTH_TEST);

  float vertices[] = {
    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
    0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
    0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
    0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
    0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
    0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
    0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
    -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

    -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
    -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
    -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

    0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
    0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
    0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
    0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
    0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
    0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
    0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
    0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
    0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
    0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
    0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
    0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
    -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
  };

  glm::vec3 cubePositions[] = {
      glm::vec3(0.0f, 0.0f, 0.0f),    glm::vec3(2.0f, 5.0f, -15.0f),
      glm::vec3(-1.5f, -2.2f, -2.5f), glm::vec3(-3.8f, -2.0f, -12.3f),
      glm::vec3(2.4f, -0.4f, -3.5f),  glm::vec3(-1.7f, 3.0f, -7.5f),
      glm::vec3(1.3f, -2.0f, -2.5f),  glm::vec3(1.5f, 2.0f, -2.5f),
      glm::vec3(1.5f, 0.2f, -1.5f),   glm::vec3(-1.3f, 1.0f, -1.5f)};

  unsigned int VAO, VBO;
  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // Shader Settings
  int success = 0;
  char infoLog[512];
  unsigned int vertexShader, fragmentShader, shaderProgram;
  loadShader("camera_test.vert", "camera_test.frag", vertexShader, fragmentShader, shaderProgram);
  glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
    std::cout << "ShaderProgram failed:" << infoLog << std::endl;
  }
  // glDeleteShader(vertexShader);
  // glDeleteShader(fragmentShader);

  GLint modelID = glGetUniformLocation(shaderProgram, "model");
  GLint viewID = glGetUniformLocation(shaderProgram, "view");
  GLint projectionID = glGetUniformLocation(shaderProgram, "projection");

  // Camera Settings
  glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
  glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
  glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

  glm::mat4 model(1.0f), view(1.0f), projection(1.0f);

  glUseProgram(shaderProgram);

  //glEnable(GL_LINE_WIDTH);
  //glEnable(GL_PROGRAM_POINT_SIZE);
  while(!glfwWindowShouldClose(window)) {
    float currentFrameTime = glfwGetTime();
    deltaFrameTime = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;
    frameRate = 1 / deltaFrameTime;
    //std::cout << "Frame rate: " << frameRate << std::endl;
    camera.speed(2.5f);

    processInput(window);

    // Rendering
    glClearColor(0.8f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shaderProgram);

    // Set view and projection matrix
    view = camera.view();
    projection = glm::perspective(glm::radians(camera.zoom()),
                                  (float)screenWidth / (float)screenHeight,
                                  0.1f, 100.0f);
    glUniformMatrix4fv(viewID, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projectionID, 1, GL_FALSE, glm::value_ptr(projection));

    // glUniformMatrix4fv(viewID, 1, GL_FALSE, &view[0][0]);

    for (unsigned int i = 0;i < 10; ++i) {
      model = glm::mat4(1.0f);
      model = glm::translate(model, cubePositions[i]);
      float angle = 20.0f * i;
      model = glm::rotate(model, glm::radians(angle), glm::vec3(1.0f, 0.3f, 0.5f));
      glUniformMatrix4fv(modelID, 1, GL_FALSE, glm::value_ptr(model));
      glDrawArrays(GL_TRIANGLES, 0, 36);
    }

    //computePhysics(vertices, shaderProgram, sizeof(vertices));

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);

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
}

void scrollCallback(GLFWwindow *window, double xOffset, double yOffset)
{
  camera.zoomInOut(yOffset);
}

void mouseMoveCallback(GLFWwindow *window, double xPos, double yPos)
{
  if (firstRun) {
    mouseLastX = xPos;
    mouseLastY = yPos;
    firstRun = false;
  }

  double xOffset = xPos - mouseLastX, yOffset = mouseLastY - yPos;
  mouseLastX = xPos;
  mouseLastY = yPos;

  camera.pitchAndYaw(xOffset, yOffset);
}

void computePhysics(GLfloat* vertices, unsigned int shaderProgram, unsigned int length)
{
  float timeValue = glfwGetTime();
  for (GLuint i = 0;i < 9 * 3; i += 9) {
    vertices[i] += sin(timeValue);
   }
  void *buf_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
  memcpy(buf_ptr, vertices, length);
  glUnmapBuffer(GL_ARRAY_BUFFER);
  /*
  float timeValue = glfwGetTime();
  std::cout << timeValue << std::endl;
  float xOffset = glGetUniformLocation(shaderProgram, "xOffset");
  glUniform1f(xOffset, sin(timeValue));
  */
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

#endif
