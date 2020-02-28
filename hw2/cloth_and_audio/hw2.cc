#ifdef __APPLE__
#include <glad/glad.h>
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

#include "hw2.h"

using namespace NSCamera;

void framebuffer_size_cb(GLFWwindow *window, int width, int height);
void loadShader(const char *vertexShaderName, const char *fragmentShaderName,
                unsigned int &vertShader, unsigned int &fragShader,
                unsigned int &shaderProg);
void processInput(GLFWwindow *window);
void scrollCallback(GLFWwindow *window, double xOffset, double yOffset);
void mouseMoveCallback(GLFWwindow *window, double xPos, double yPos);
//void mouseButtonCallback(GLFWwindow *window, int, int, int);

void updateCube();
void convertVertices(Vertex**, float *);
void computePhysics(Vertex**, float*, float, unsigned int, unsigned int, unsigned int);

// Configuration
const float screenWidth = 800, screenHeight = 600;
bool firstRun = true, mouseBeingPressed = false;
float mouseLastX = 0.0, mouseLastY = 0.0, mouseCurX = 0.0, mouseCurY = 0.0;
float lastFrameTime = 0.0f, deltaFrameTime = 0.0f, frameRate = 0.0f;
//Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
Camera camera(glm::vec3(1.0f, 1.0f, 6.0f), glm::vec3(0.0f, 1.0f, 0.0f), -100.0f, -15.0f);

// Cloth configuration
const unsigned int clothWidth = 31, clothHeight = 31;
const unsigned int clothVertexCount = clothWidth * (2 * clothHeight - 2), clothVertexStepLen = 9;
const bool fixedCloth = true;
bool bendingEnabled = true, dragEnabled = true;

// Physics factors settings
const float restLen = 1.6 / (clothWidth - 1), springK = 500.0f, dampingK = 5.0f, mass = 0.1f;

const float gv = 2.0f;
const glm::vec3 gvDirection = glm::vec3(0.0f, -1.0f, 0.0f);

const glm::vec3 airVelocity = glm::vec3(1.0f, 2.0f, 4.1f);
// 10, 0.95
const float airDensity = 15, dragCoefficient = 0.95;

// Cube vertices
float cubeEdgeLength = 0.6;
float cubelx = 0, cuberx = cubelx + cubeEdgeLength,
  cubely = -0.6, cubery = cubely + cubeEdgeLength,
  cubelz = 1.1, cuberz = cubelz + cubeEdgeLength;
glm::vec3 cubeVel = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 cubeVertices[8] = {
  glm::vec3(cubelx, cubery, cubelz), glm::vec3(cubelx, cubery, cuberz),
  glm::vec3(cuberx, cubery, cuberz), glm::vec3(cuberx, cubery, cubelz),
  glm::vec3(cubelx, cubely, cubelz), glm::vec3(cubelx, cubely, cuberz),
  glm::vec3(cuberx, cubely, cuberz), glm::vec3(cuberx, cubely, cubelz),
};
glm::vec3 cubeNormals[6] = {
  glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f),
  glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f),
  glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f)
};
const float cubeColors[] = { 0.5f, 0.5f, 0.5f };
unsigned int cubeRenderSeq[] = { 1, 2, 5, 6, 7, 2, 3, 1, 0, 5, 4, 7, 0, 3 };
unsigned int cubeVerticesLength = sizeof(cubeRenderSeq) / sizeof(unsigned int);
const float collisionThreshold = 0.01f, bounceRate = 0.75;

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

  unsigned int flagTexture;
  glGenTextures(1, &flagTexture);
  glBindTexture(GL_TEXTURE_2D, flagTexture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  int flagTextureWidth, flagTextureHeight, flagTextureNRChannels;
  unsigned char *flagTextureGoldy =
      stbi_load("flag_goldy.png", &flagTextureWidth, &flagTextureHeight,
                &flagTextureNRChannels, 0);
  if (flagTextureGoldy) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, flagTextureWidth, flagTextureHeight,
                 0, GL_RGB, GL_UNSIGNED_BYTE, flagTextureGoldy);
    glGenerateMipmap(GL_TEXTURE_2D);
  } else {
    std::cout << "Loading texture failed." << std::endl;
  }

  stbi_image_free(flagTextureGoldy);

  /*
  Vertex *clothVertices = new Vertex[clothVertexCount];

  for (int i = 0; i < clothHeight - 1; ++i) {
    for (int j = 0; j < clothWidth; ++j) {
      clothVertices[i * clothWidth + j * 2] =
        Vertex(glm::vec3(-0.8 + j * xStep, 0.8 - i * yStep, 0.0f),
               glm::vec3(1.0f, 0.0f, 1.0f), 20.0f, 1.0f);
      clothVertices[i * clothWidth + j * 2 + 1] =
        Vertex(glm::vec3(-0.8 + j * xStep, 0.8 - (i + 1) * yStep, 0.0f),
               glm::vec3(1.0f, 0.0f, 1.0f), 20.0f, 1.0f);
    }
  }
  */

  Vertex **clothVertices = new Vertex*[clothHeight];
  float texXStep = 1.0 / (clothWidth - 1), texYStep = 1.0 / (clothHeight - 1);
  float xStep = 1.6 / (clothWidth - 1), yStep = 1.6 / (clothHeight - 1);
  glm::vec3 clothStripColor[2] = {
  //glm::vec3(0.21f, 0.81f, 0.37f),
  //glm::vec3(0.07f, 0.26f, 0.12f)

      glm::vec3(1.0f, 1.0f, 1.0f),
      glm::vec3(1.0f, 1.0f, 1.0f)
  };
  for (size_t i = 0;i < clothHeight; ++i) {
    clothVertices[i] = new Vertex[clothWidth];
    for (size_t j = 0;j < clothWidth; ++j) {
      clothVertices[i][j] =
        //Vertex(glm::vec3(-0.8 + i * yStep, 1.0f, -0.8 + j * xStep),
        Vertex(glm::vec3(-0.8 + j * xStep, 0.8 - i * yStep, 1.0f),
        //Vertex(glm::vec3(1.0f, 0.8 + i * yStep, -0.8 + j * xStep),
          // Vertex(glm::vec3(-0.8 + j * xStep, 0.5f, 0.8 - i * yStep),
        //Vertex(glm::vec3(-0.8 + j * xStep, 0.8 - i * yStep, 0.0f),
          clothStripColor[i%2], 20.0f, mass);
      clothVertices[i][j].texture() = glm::vec2(j * texXStep, i * texYStep);
    }
    //std::cout << clothVertices[i][clothWidth-2].pos().x << std::endl;
    //std::cout << clothVertices[i][clothHeight-2].pos().y << std::endl;
  }

  float *vertices = new float[(clothVertexCount + cubeVerticesLength) * clothVertexStepLen];
  convertVertices(clothVertices, vertices);
  /*
  size_t curVerticesIndex = 0;
  for (size_t i = 0;i < clothHeight - 1; ++i) {
    for (size_t j = 0;j < clothWidth; ++j) {
      clothVertices[i][j].flat(&vertices[curVerticesIndex]);
      curVerticesIndex += clothVertexStepLen;
      clothVertices[i+1][j].flat(&vertices[curVerticesIndex]);
      curVerticesIndex += clothVertexStepLen;
    }
  }
  */

  /*
  GLfloat vertices[] = {
    0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, 30.0f,
    0.5f,  -0.5f, -0.0f, 1.0f,  0.0f, 0.0f, 0.0f, 1.0f,  30.0f,
    -0.5f, -0.5f, -0.0f, 1.0f, 0.0f, 0.0f, 0.0f,  1.0f,  30.0f,
    -0.5f, 0.5f,  0.0f, 1.0f, 0.0f, 0.0f,  0.0f,  1.0f,  30.0f
  };
  */

  /*
  GLfloat vertices[] = {
    0.5f,  0.0f,  0.5f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, 30.0f,
    0.5f,  -0.0f, -0.5f, 1.0f,  0.0f, 0.0f, 0.0f, 1.0f,  30.0f,
    -0.5f, -0.0f, -0.5f, 1.0f, 0.0f, 0.0f, 0.0f,  1.0f,  30.0f,
    -0.5f, 0.0f,  0.5f, 1.0f, 0.0f, 0.0f,  0.0f,  1.0f,  30.0f
  };
  */

  unsigned int indices[] = {
    0, 1, 3,
    1, 2, 3
  };

  unsigned int VAO, VBO;
  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, (clothVertexCount + cubeVerticesLength) * clothVertexStepLen * sizeof(float), vertices, GL_STREAM_DRAW);

  unsigned int EBO;
  glGenBuffers(1, &EBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, clothVertexStepLen * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                        clothVertexStepLen * sizeof(float), (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE,
                        clothVertexStepLen * sizeof(float), (void *)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE,
                        clothVertexStepLen * sizeof(float), (void *)(7 * sizeof(float)));
  glEnableVertexAttribArray(3);

  // Shader Settings
  int success = 0;
  char infoLog[512];
  unsigned int vertexShader, fragmentShader, shaderProgram;
  loadShader("hw2.vert", "hw2.frag", vertexShader, fragmentShader, shaderProgram);
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

    glBindVertexArray(VAO);
    //glDrawArrays(GL_TRIANGLES, 0, clothVertexCount);
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    //glDrawElements(GL_TRIANGLES, clothVertexCount, GL_UNSIGNED_INT, 0);
    //glLineWidth(10);
    //glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
    //glDrawElements(GL_POINTS, 6, GL_UNSIGNED_INT, 0);
    //glBindVertexArray(VAO);
    //glDrawArrays(GL_LINE_STRIP, 0, 220);

    unsigned int drawStep = clothWidth * 2;
    for (size_t count = 0; count < clothHeight - 1; ++count)
      glDrawArrays(GL_TRIANGLE_STRIP, count * drawStep, drawStep);

    //glDrawArrays(GL_TRIANGLE_STRIP, 0, clothVertexCount);
    glDrawArrays(GL_TRIANGLE_STRIP, clothVertexCount, cubeVerticesLength);

    //glDrawArrays(GL_TRIANGLE_STRIP, 0, 3);
    //glDrawArrays(GL_POINTS, 0, clothVertexCount);

    /*
    for (size_t pi = 0;pi < 7; ++pi)
      std::cout << vertices[pi+7] << " ";
    std::cout << std::endl;
    */
    for (size_t count = 0; count < 50; ++count) {
      computePhysics(clothVertices, vertices, deltaFrameTime / 50,
                     clothVertexCount * clothVertexStepLen, clothWidth,
                     clothHeight);
    }
    convertVertices(clothVertices, vertices);
    void *buf_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    memcpy(buf_ptr, vertices, (clothVertexCount + cubeVerticesLength) * clothVertexStepLen * sizeof(float));
    //std::cout << *((float*)(buf_ptr)+7) <<std::endl;
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

  cubeVel = glm::vec3(0.0f, 0.0f, 0.0f);
  /* Cube control */
  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
    cubelz -= 0.01f;
    cuberz -= 0.01f;
    cubeVel = glm::vec3(0.0f, 0.0f, -0.02f / deltaFrameTime);
    updateCube();
  }
  if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
    cubelz += 0.01f;
    cuberz += 0.01f;
    cubeVel = glm::vec3(0.0f, 0.0f, 0.02f / deltaFrameTime);
    updateCube();
  }
  if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
    cubelx -= 0.01f;
    cuberx -= 0.01f;
    cubeVel = glm::vec3(-0.02f / deltaFrameTime, 0.0f, 0.0f);
    updateCube();
  }
  if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
    cubelx += 0.01f;
    cuberx += 0.01f;
    cubeVel = glm::vec3(0.02f / deltaFrameTime, 0.0f, 0.0f);
    updateCube();
  }

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

void updateCube()
{
  cubeVertices[0] = glm::vec3(cubelx, cubery, cubelz);
  cubeVertices[1] = glm::vec3(cubelx, cubery, cuberz);
  cubeVertices[2] = glm::vec3(cuberx, cubery, cuberz);
  cubeVertices[3] = glm::vec3(cuberx, cubery, cubelz);
  cubeVertices[4] = glm::vec3(cubelx, cubely, cubelz);
  cubeVertices[5] = glm::vec3(cubelx, cubely, cuberz);
  cubeVertices[6] = glm::vec3(cuberx, cubely, cuberz);
  cubeVertices[7] = glm::vec3(cuberx, cubely, cubelz);
}

void convertVertices(Vertex **clothVertices, float *vertices)
{
  size_t curVerticesIndex = 0;
  for (size_t i = 0;i < clothHeight - 1; ++i) {
    for (size_t j = 0;j < clothWidth; ++j) {
      // clothVertices[i][j].flat(&vertices[curVerticesIndex]);
      clothVertices[i][j].flat(&vertices[curVerticesIndex]);
      curVerticesIndex += clothVertexStepLen;
      clothVertices[i + 1][j].flat(&vertices[curVerticesIndex]);
      // clothVertices[i + 1][j].flat(&vertices[curVerticesIndex]);
      curVerticesIndex += clothVertexStepLen;
    }
  }

  for (size_t i = 0;i < cubeVerticesLength; ++i) {
    vertices[curVerticesIndex++] = cubeVertices[cubeRenderSeq[i]].x;
    vertices[curVerticesIndex++] = cubeVertices[cubeRenderSeq[i]].y;
    vertices[curVerticesIndex++] = cubeVertices[cubeRenderSeq[i]].z;
    vertices[curVerticesIndex++] = cubeColors[0];
    vertices[curVerticesIndex++] = cubeColors[1];
    vertices[curVerticesIndex++] = cubeColors[2];
    vertices[curVerticesIndex++] = 20.0f;
    vertices[curVerticesIndex++] = 0.0f;
    vertices[curVerticesIndex++] = 0.0f;
  }
}

void computePhysics(Vertex **clothVertices, float* vertices, float dt,
                    unsigned int shaderVLength, unsigned int clothWidth, unsigned int clothHeight)
{
  float dragTermsValue = -0.5 * airDensity * dragCoefficient;

  for (size_t i = 0; i < clothHeight; ++i) {
    for (size_t j = 0; j < clothWidth; ++j) {
      clothVertices[i][j].force() = glm::vec3(0.0f, 0.0f, 0.0f);
    }
  }

  for (size_t i = 0; i < clothHeight; ++i) {
    for (size_t j = 0; j < clothWidth; ++j) {

      if (i != 0 || !fixedCloth)
        clothVertices[i][j].force() += gv * mass * gvDirection;

      // Cloth x-axis force update
      if (i < clothHeight - 1) {
        /*
        std::cout << "bpos: " << glm::to_string(clothVertices[i][j].pos())
                  << " " << glm::to_string(clothVertices[i+1][j].pos())
                  << std::endl;
        */

        glm::vec3 posOri = glm::normalize(clothVertices[i + 1][j].pos() -
                                          clothVertices[i][j].pos());
        float posMag = glm::distance(clothVertices[i][j].pos(),
                                     clothVertices[i+1][j].pos()),
              velMagDif = glm::dot(posOri, clothVertices[i][j].vel()) -
                          glm::dot(posOri, clothVertices[i + 1][j].vel()),
              curSpringForce =
                  -springK * (restLen - posMag) - dampingK * velMagDif;

        //std::cout << "posOri:" << glm::to_string(posOri) << std::endl;
        //if (abs((abs(springK * (restLen-posMag)) - gv*mass)) < 0.01)
        //std::cout << "Pos now: " << glm::to_string(clothVertices[i+1][j].pos()) << std::endl;
        //std::cout << posMag << std::endl;
        //std::cout << "S" << -springK*(restLen-posMag) << std::endl;
        //std::cout << dampingK * velMagDif << std::endl;
        if (i != 0 || !fixedCloth)
          clothVertices[i][j].force() += curSpringForce * posOri;
        clothVertices[i+1][j].force() -= curSpringForce * posOri;

        /*

        std::cout << "apos: " << glm::to_string(clothVertices[i][j].pos())
                  << " " << glm::to_string(clothVertices[i+1][j].pos())
                  << std::endl;
        */
      }

      // Cloth z-axis force update
      if (j < clothWidth - 1) {
        glm::vec3 posOri = glm::normalize(clothVertices[i][j+1].pos() -
                                          clothVertices[i][j].pos());
        float posMag = glm::distance(clothVertices[i][j+1].pos(),
                                     clothVertices[i][j].pos()),
              velMagDif = glm::dot(posOri, clothVertices[i][j].vel()) -
                          glm::dot(posOri, clothVertices[i][j+1].vel()),
              curSpringForce =
                  -springK * (restLen - posMag) - dampingK * velMagDif;

        if (i != 0 || !fixedCloth)
          clothVertices[i][j].force() += curSpringForce * posOri;
        clothVertices[i][j+1].force() -= curSpringForce * posOri;
      }

      // Bending & 2nd order bending (skip nodes)
      if (bendingEnabled) {
        if (i < clothHeight - 1 && j > 0) {
          glm::vec3 posOri = glm::normalize(clothVertices[i + 1][j-1].pos() -
                                            clothVertices[i][j].pos());
          float posMag = glm::distance(clothVertices[i + 1][j-1].pos(),
                                       clothVertices[i][j].pos()),
            velMagDif = glm::dot(posOri, clothVertices[i][j].vel()) -
            glm::dot(posOri, clothVertices[i + 1][j-1].vel()),
            curSpringForce = -springK * (1.414 * restLen - posMag) - dampingK * velMagDif;

          if (i != 0 || !fixedCloth)
            clothVertices[i][j].force() += curSpringForce * posOri;
          clothVertices[i + 1][j-1].force() -= curSpringForce * posOri;
        }
        if (i < clothHeight - 1 && j < clothWidth - 1) {
          glm::vec3 posOri = glm::normalize(clothVertices[i + 1][j+1].pos() -
                                            clothVertices[i][j].pos());
          float posMag = glm::distance(clothVertices[i + 1][j+1].pos(),
                                       clothVertices[i][j].pos()),
            velMagDif = glm::dot(posOri, clothVertices[i][j].vel()) -
            glm::dot(posOri, clothVertices[i + 1][j + 1].vel()),
            curSpringForce = -springK * (1.414 * restLen - posMag) - dampingK * velMagDif;

          if (i != 0 || !fixedCloth)
            clothVertices[i][j].force() += curSpringForce * posOri;
          clothVertices[i + 1][j + 1].force() -= curSpringForce * posOri;
        }

        // 2nd order
        if (i % 2 == 0 && j % 2 == 0) {
          if (i < clothHeight - 2) {
            glm::vec3 posOri = glm::normalize(clothVertices[i + 2][j].pos() -
                                              clothVertices[i][j].pos());
            float posMag = glm::distance(clothVertices[i + 2][j].pos(),
                                         clothVertices[i][j].pos()),
              velMagDif = glm::dot(posOri, clothVertices[i][j].vel()) -
              glm::dot(posOri, clothVertices[i + 2][j].vel()),
              curSpringForce =
              -springK * (2 * restLen - posMag) - dampingK * velMagDif;

            if (i != 0 || !fixedCloth)
              clothVertices[i][j].force() += curSpringForce * posOri;
            clothVertices[i + 2][j].force() -= curSpringForce * posOri;
          }

          if (j < clothHeight - 2) {
            glm::vec3 posOri = glm::normalize(clothVertices[i][j + 2].pos() -
                                              clothVertices[i][j].pos());
            float posMag = glm::distance(clothVertices[i][j + 2].pos(),
                                         clothVertices[i][j].pos()),
              velMagDif = glm::dot(posOri, clothVertices[i][j].vel()) -
              glm::dot(posOri, clothVertices[i][j + 2].vel()),
              curSpringForce =
              -springK * (2 * restLen - posMag) - dampingK * velMagDif;

            if (i != 0 || !fixedCloth)
              clothVertices[i][j].force() += curSpringForce * posOri;
            clothVertices[i][j + 2].force() -= curSpringForce * posOri;
          }

          if (i < clothHeight - 2 && j < clothHeight - 2) {
            glm::vec3 posOri = glm::normalize(clothVertices[i+2][j + 2].pos() -
                                              clothVertices[i][j].pos());
            float posMag = glm::distance(clothVertices[i+2][j + 2].pos(),
                                         clothVertices[i][j].pos()),
              velMagDif = glm::dot(posOri, clothVertices[i][j].vel()) -
              glm::dot(posOri, clothVertices[i+2][j + 2].vel()),
              curSpringForce =
              -springK * (2.828 * restLen - posMag) - dampingK * velMagDif;

            if (i != 0 || !fixedCloth)
              clothVertices[i][j].force() += curSpringForce * posOri;
            clothVertices[i+2][j + 2].force() -= curSpringForce * posOri;
          }
          if (i < clothHeight - 2 && j >= 2) {
            glm::vec3 posOri = glm::normalize(clothVertices[i + 2][j -2].pos() -
                                              clothVertices[i][j].pos());
            float posMag = glm::distance(clothVertices[i + 2][j -2].pos(),
                                         clothVertices[i][j].pos()),
              velMagDif = glm::dot(posOri, clothVertices[i][j].vel()) -
              glm::dot(posOri, clothVertices[i + 2][j -2].vel()),
              curSpringForce =
              -springK * (2.828 * restLen - posMag) - dampingK * velMagDif;

            if (i != 0 || !fixedCloth)
              clothVertices[i][j].force() += curSpringForce * posOri;
            clothVertices[i + 2][j -2].force() -= curSpringForce * posOri;
          }
        }
      }

      // Drag
      if (dragEnabled) {
        glm::vec3 relativeVelocity, normalVec, dragForce;
        if (i < clothHeight - 1 && j < clothWidth - 1) {
          Vertex *v1, *v2, *v3;
          if (j % 2 == 0) {
            v1 = &clothVertices[i][j];
            v2 = &clothVertices[i + 1][j];
            v3 = &clothVertices[i][j + 1];
          } else {
            v1 = &clothVertices[i][j];
            v2 = &clothVertices[i + 1][j - 1];
            v3 = &clothVertices[i + 1][j];
          }

          relativeVelocity = (v1->vel() + v2->vel() + v3->vel()) / 3.0f - airVelocity;
          normalVec = glm::cross(v2->pos() - v1->pos(), v3->pos() - v1->pos());

          dragForce = dragTermsValue * glm::length(relativeVelocity)
                * glm::dot(relativeVelocity, normalVec) / (2 * glm::length(normalVec)) * normalVec;

          v2->force() += dragForce / 3.0f;
          if (j % 2 == 0 && (i != 0 || !fixedCloth)) {
            v1->force() += dragForce / 3.0f;
            v3->force() += dragForce / 3.0f;
          }
          if (j % 2 == 1) {
            v3->force() += dragForce / 3.0f;
            if (i != 0 || !fixedCloth)
              v1->force() += dragForce / 3.0f;
          }
        }
      }
    }
  }

  for (size_t i = 0; i < clothHeight; ++i) {
    for (size_t j = 0; j < clothWidth; ++j) {
      //clothVertices[i][j].update(dt);

      // Collision Detection
      // Check if x y z in the cube

      float curDistance = 0.0f;
      for (size_t k = 0; k < 6; ++k) {
        float curX = clothVertices[i][j].pos().x,
              curY = clothVertices[i][j].pos().y,
              curZ = clothVertices[i][j].pos().z;

        if (curX >= cubelx && curX <= cuberx && curY >= cubely && curY <= cubery &&
            curZ >= cubelz && curZ <= cuberz) {
          curDistance = glm::dot(clothVertices[i][j].pos() - cubeVertices[k],
                                 cubeNormals[k]);

          if (abs(curDistance) <= collisionThreshold) {
            clothVertices[i][j].pos() -= curDistance * cubeNormals[k];
            clothVertices[i][j].pos() += 0.05f * cubeNormals[k];
            glm::vec3 incomingVel = clothVertices[i][j].vel() - cubeVel,
              incomingF = clothVertices[i][j].force();
            if (glm::dot(incomingVel, cubeNormals[k]) < 0) {
              clothVertices[i][j].vel() =
                incomingVel - 2 * bounceRate *
                                    glm::dot(incomingVel, cubeNormals[k]) *
                                    cubeNormals[k];
            }
            if (glm::dot(incomingF, cubeNormals[k]) < 0) {
              clothVertices[i][j].force() =
                (incomingF - glm::dot(incomingF, cubeNormals[k]) * cubeNormals[k]);
            }
          }
        }
      }

      clothVertices[i][j].update(dt);
    }
  }

  //convertVertices(clothVertices, vertices);
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

