#ifdef __APPLE__
#include <glad/glad.h>
#else
#include <GL/glew.h>
#endif

#include <stdio.h>
#include <SDL2/SDL.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

#include "hw2.h"

#define STRING_COUNT 7
#define STRING_SEGMENT_COUNT 30
#define AUDIO_BUFFER_LEN 3000000

using namespace NSCamera;

void framebuffer_size_cb(GLFWwindow *window, int width, int height);
void loadShader(const char *vertexShaderName, const char *fragmentShaderName,
                unsigned int &vertShader, unsigned int &fragShader,
                unsigned int &shaderProg);
void processInput(GLFWwindow *window);
void scrollCallback(GLFWwindow *window, double xOffset, double yOffset);
void mouseMoveCallback(GLFWwindow *window, double xPos, double yPos);

void processAudioInput(GLFWwindow *window, std::vector<std::vector<Vertex>>&);
void pluck(std::vector<std::vector<Vertex>>&, unsigned int, float);
void convertVertices(std::vector<std::vector<Vertex>> &, float *);
void computePhysics(std::vector<std::vector<Vertex>> &, float *, float dt);
void audio_callback(void*, Uint8*, int);

// Configuration
const float screenWidth = 800, screenHeight = 600;
bool firstRun = true;
float mouseLastX = 0.0, mouseLastY = 0.0;
float lastFrameTime = 0.0f, deltaFrameTime = 0.0f, frameRate = 0.0f;
Camera camera(glm::vec3(0.0f, 0.0f, 20.0f));
unsigned int propertyCount = 9;
//Camera camera(glm::vec3(2.0f, 2.0f, 5.0f), glm::vec3(0.0f, 1.0f, 0.0f), -110.0f, -20.0f);

// String settings
const unsigned int stringCount = STRING_COUNT, stringSegmentCount = STRING_SEGMENT_COUNT;

// Physics factors settings
const float restLen = 0.5, mass = 1.0f, tension = 10.0f;
float springK[STRING_COUNT] = { 1500000.0f, 100000.0f, 300000.0f, 400000.0f, 100.0f, 1000.0f, 10000.0f}, dampingK[STRING_COUNT] = { 10.f, 100.f, 10.f, 10.f, 10.f, 10.f, 10.f };

// Voice settings
const unsigned int micPos = stringSegmentCount / 2, samplesPerSecond = 48000;
double toneVolume = 2000;
static unsigned int lastS = 0, lastP = 0, recentPluck = 100;
float soundBuffer[AUDIO_BUFFER_LEN];

int main(int argc, char* argv[])
{
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  GLFWwindow* window = glfwCreateWindow(800, 600, "Homework 2", NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
  //glfwSetCursorPosCallback(window, mouseMoveCallback);
  glfwSetScrollCallback(window, scrollCallback);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glfwSetFramebufferSizeCallback(window, framebuffer_size_cb);
  glEnable(GL_DEPTH_TEST);

  SDL_Init(SDL_INIT_AUDIO);
  SDL_AudioSpec desiredSpec, obtainSpec;
  desiredSpec.freq = samplesPerSecond;
  // desiredSpec.format = AUDIO_S16SYS;
  desiredSpec.format = AUDIO_S16SYS;
  desiredSpec.channels = 1;
  desiredSpec.samples = 2048;
  desiredSpec.callback = audio_callback;
  SDL_OpenAudio(&desiredSpec, &obtainSpec);
  //SDL_PauseAudio(0);

  std::vector<std::vector<Vertex>> strings(stringCount, std::vector<Vertex>(stringSegmentCount+1));
  float stringStep = 10.0f / stringCount, segmentStep = restLen;
  stringStep = 2.0f;
  for (size_t i = 0;i < stringCount; ++i)
    for (size_t j = 0; j <= stringSegmentCount; ++j) {
      strings[i][j].pos() =
          glm::vec3(-7.5f + j * segmentStep, 5.0f - stringStep * i, 0.0f);
      //strings[i][j].vel() = glm::vec3(0.0f, 0.0f, 0.0f);
      //strings[i][j].color() = glm::vec3(0.0f, 0.0f, 0.0f);
      //strings[i][j].radius(20.0f);
      /*
      std::cout << "i: " << i << ", j: " << j << std::endl;
      std::cout << strings[i][j].radius() << ", " << strings[i][j].mass()
                << std::endl;
      */
      //std::cout << "pos: " << glm::to_string(strings[i][j].pos()) << std::endl;
      //std::cout << "vel: " << glm::to_string(strings[i][j].vel()) << std::endl;
    }
      //Vertex v(glm::vec3(-5.0f + j * segmentStep, 5.0f - stringStep * i, 0.0f),
      //       glm::vec3(0.0f, 0.0f, 0.0f), 20.0f, 1.0f);


  float *vertices = new float[stringCount * (stringSegmentCount + 1) * propertyCount];
  convertVertices(strings, vertices);

  unsigned int VAO, VBO;
  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, stringCount * (stringSegmentCount + 1) * propertyCount * sizeof(float), vertices, GL_STREAM_DRAW);

  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, propertyCount * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                        propertyCount * sizeof(float), (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE,
                        propertyCount * sizeof(float), (void *)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);

  // Shader Settings
  int success = 0;
  char infoLog[512];
  unsigned int vertexShader, fragmentShader, shaderProgram;
  loadShader("audio.vert", "audio.frag", vertexShader, fragmentShader, shaderProgram);
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
  SDL_PauseAudio(0);
  while(!glfwWindowShouldClose(window)) {
    //SDL_PauseAudio(1);
    float currentFrameTime = glfwGetTime();
    deltaFrameTime = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;
    frameRate = 1 / deltaFrameTime;
    std::cout << "Frame rate: " << frameRate << std::endl;
    camera.speed(2.5f);

    processInput(window);
    processAudioInput(window, strings);

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

    glBindVertexArray(VAO);
    //glDrawArrays(GL_TRIANGLES, 0, clothVertexCount);
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    //glDrawElements(GL_TRIANGLES, clothVertexCount, GL_UNSIGNED_INT, 0);
    //glLineWidth(10);
    //glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
    //glDrawElements(GL_POINTS, 6, GL_UNSIGNED_INT, 0);
    //glBindVertexArray(VAO);
    //glDrawArrays(GL_LINE_STRIP, 0, 220);

    for (size_t count = 0; count < 2; ++count) {
      // for (size_t count = 0; count < stringCount; ++count) {
      glDrawArrays(GL_LINE_STRIP, count * (stringSegmentCount + 1),
                   (stringSegmentCount + 1));
    }

    //glDrawArrays(GL_TRIANGLE_STRIP, 0, 3);
    //glDrawArrays(GL_POINTS, 0, clothVertexCount);

    SDL_LockAudio();
    for (size_t count = 0; count < 1000; ++count) {
      //computePhysics(strings, vertices, deltaFrameTime / 1000);
      computePhysics(strings, vertices, 0.00001);
      if (lastS < AUDIO_BUFFER_LEN && recentPluck < stringCount) {
        soundBuffer[lastS++] = 0.5 * strings[recentPluck][micPos].pos().y +
                               0.25 * strings[recentPluck][micPos - 1].pos().y +
                               0.25 * strings[recentPluck][micPos + 1].pos().y;
      }
    }
    //recentPluck = 100;
    SDL_UnlockAudio();
    convertVertices(strings, vertices);

    void *buf_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    memcpy(buf_ptr, vertices, stringCount * (stringSegmentCount + 1) * propertyCount * sizeof(float));
    glUnmapBuffer(GL_ARRAY_BUFFER);
 
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

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

void processAudioInput(GLFWwindow *window, std::vector<std::vector<Vertex>>& strings)
{
  if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
    pluck(strings, 0, 0.3);
    recentPluck = 0;
  }
  if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
    pluck(strings, 1, 0.3);
    recentPluck = 1;
  }
  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
    springK[0] += 5000;
  }
}

void pluck(std::vector<std::vector<Vertex>> &strings, unsigned int stringIndex,
           float strength) {
  float d = strength * restLen;
  strings[stringIndex][micPos].pos() += glm::vec3(0.0f, d, 0.0f);
  strings[stringIndex][micPos-1].pos() += glm::vec3(0.0f, d/2, 0.0f);
  strings[stringIndex][micPos+1].pos() += glm::vec3(0.0f, d/2, 0.0f);
  strings[stringIndex][micPos-2].pos() += glm::vec3(0.0f, d/2, 0.0f);
  strings[stringIndex][micPos+2].pos() += glm::vec3(0.0f, d/2, 0.0f);
}

void convertVertices(std::vector<std::vector<Vertex>> &strings, float *vertices)
{
  size_t curVerticesIndex = 0;
  for (size_t i = 0;i < stringCount; ++i) {
    for (size_t j = 0;j <= stringSegmentCount; ++j) {
      strings[i][j].flat(&vertices[curVerticesIndex]);
      curVerticesIndex += propertyCount;
    }
  }
}

void computePhysics(std::vector<std::vector<Vertex>> &strings, float *vertices, float dt)
{
  for (size_t i = 0;i < stringCount; ++i)
    for (size_t j = 0;j <= stringSegmentCount; ++j)
      strings[i][j].force() = glm::vec3(0.0f, 0.0f, 0.0f);

  for (size_t i = 0;i < stringCount; ++i) {
    for (size_t j = 0;j < stringSegmentCount; ++j) {
      glm::vec3 posOri = glm::normalize(strings[i][j+1].pos() -
                                        strings[i][j].pos());
      float posMag = glm::distance(strings[i][j+1].pos(),
                                   strings[i][j].pos()),
        velMagDif = glm::dot(posOri, strings[i][j].vel()) -
                    glm::dot(posOri, strings[i][j+1].vel()),
        curSpringForce = -springK[i] * (restLen - posMag) - dampingK[i] * velMagDif;

      /*
      std::cout << i << "-" << j << std::endl;
      std::cout << glm::to_string(strings[i][j+1].pos()) << std::endl;
      std::cout << posMag << " " << curSpringForce << std::endl;

      if (velMagDif > 1 || posMag > 1 || curSpringForce > 1 || isnan(velMagDif) || isnan(curSpringForce) || isnan(posMag)) {
        std::cout << "v1 pos " << glm::to_string(strings[i][j].pos()) << std::endl  
                  << "v1 vel " << glm::to_string(strings[i][j].pos()) << std::endl;
        std::cout << "v2 pos " << glm::to_string(strings[i][j + 1].vel())
                  << std::endl
                  << "v2 vel " << glm::to_string(strings[i][j + 1].vel())
                  << std::endl;
        std::cout << "posMag: " << posMag << std::endl
                  << "vel: " << velMagDif << ", force: " << curSpringForce << std::endl
                  << "restLen - posMag: " << restLen - posMag << std::endl;
        exit(0);
      }
      */

      if (j != 0)
        strings[i][j].force() += curSpringForce * posOri;
      if (j != stringSegmentCount - 1)
        strings[i][j+1].force() -= curSpringForce * posOri;
    }
  }

  for (size_t i = 0;i < stringCount; ++i) 
    for (size_t j = 1;j < stringSegmentCount; ++j)
      strings[i][j].update(dt);
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

void audio_callback(void *b, Uint8 *stream, int len) {
  short *soundStream = (short *)stream;
  int i;
  static int lastP = 0;


  /*
    for (i = 0; i < len / 2; ++i) {
      int j = lastP++ / (24000/128);
      soundStream[i] = (j % 2) ? toneVolume : -toneVolume;
    }
  */

  for (i = 0; i < len && (lastP < lastS); ++i) {
    if (lastS > AUDIO_BUFFER_LEN)
      return;

    double amp = toneVolume * 2 * soundBuffer[++lastP];
    amp = std::clamp(amp, -3 * toneVolume, 3 * toneVolume);
    soundStream[i] = amp;
    //std::cout << lastP << " " << lastS << " " << amp << std::endl;
  }

  for (; i < len; ++i) {
    soundStream[i] = soundStream[i-1];
    //std::cout << 123 << std::endl;
  }
}
