#include <GL/glew.h>     
#include <GLFW/glfw3.h>  

#include <glm/glm.hpp>           
#include <glm/gtc/type_ptr.hpp>  

#include <opencv2/calib3d.hpp> 
#include <iostream>

#include <pose.hpp>

namespace {
const float CUBE_SIDE = 100.f;                         
const float SHEET_W = 297.f, SHEET_H = 210.f;         
const glm::vec3 CUBE_COLOR(0.2f, 0.5f, 1.f);            
const glm::vec3 EDGE_COLOR(0.f, 0.f, 0.f);             
const glm::vec3 SHEET_COLOR(1.f, 1.f, 1.f);           
const glm::vec3 BACKGROUND(0.25f, 0.25f, 0.3f);        

const char* vertexSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;  
uniform mat4 uMVP;                 
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* fragmentSrc = R"(
#version 330 core
uniform vec3 uColor;  
out vec4 FragColor;
void main() {
  FragColor = vec4(uColor, 1.0);
}
)";


GLFWwindow* window = nullptr;
GLuint program = 0;           
GLint mvpLoc = -1, colorLoc = -1; 
GLuint vao = 0, vbo = 0;     
GLuint ebo = 0;               
glm::mat4 proj(1.f);           
glm::mat4 lastModelView(1.f); 
bool hasPose = false;          

const int N_CUBE_TRIS = 36, N_CUBE_EDGES = 24 ;
const int N_SHEET_TRIS = 6;

GLuint compileShader(GLenum type, const char* src) {
  GLuint s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[512];
    glGetShaderInfoLog(s, 512, nullptr, log);
    std::cerr << "Erreur shader :\n" << log << std::endl;
  }
  return s;
}

glm::mat4 projectionFromK(const cv::Mat& K, int W, int H, float near, float far) {
  float fx = (float)K.at<double>(0, 0), fy = (float)K.at<double>(1, 1);
  float cx = (float)K.at<double>(0, 2), cy = (float)K.at<double>(1, 2);
  glm::mat4 P(0.f);              
  P[0][0] = 2.f * fx / W;      
  P[2][0] = 2.f * cx / W - 1.f; 
  P[1][1] = -2.f * fy / H;     
  P[2][1] = 1.f - 2.f * cy / H;  
  P[2][2] = (far + near) / (far - near);  
  P[3][2] = -2.f * far * near / (far - near);
  P[2][3] = 1.f;              
  return P;
}

glm::mat4 modelViewFromPose(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R); 
  glm::mat4 M(1.f);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) M[c][r] = (float)R.at<double>(r, c); 
    M[3][r] = (float)tvec.at<double>(r);                            
  }
  return M;
}
}  

bool glCubeInit(const CameraParams& cam, cv::Size size) {
  if (!glfwInit()) {
    std::cerr << "Impossible d'initialiser GLFW" << std::endl;
    return false;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);  
  window = glfwCreateWindow(size.width, size.height, "Cube OpenGL", nullptr, nullptr);
  if (!window) {
    std::cerr << "Impossible de creer la fenetre OpenGL" << std::endl;
    glfwTerminate();
    return false;
  }
  glfwMakeContextCurrent(window);  
  glfwSwapInterval(0);             

  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Impossible d'initialiser GLEW" << std::endl;
    return false;
  }

  const float h = CUBE_SIDE / 2, s = CUBE_SIDE;
  const float sw = SHEET_W / 2, sh = SHEET_H / 2;
  float vertices[] = {
      -h, -h, 0,  h, -h, 0,  h, h, 0,  -h, h, 0,  
      -h, -h, s,  h, -h, s,  h, h, s,  -h, h, s,  
      -sw, -sh, -0.5f,  sw, -sh, -0.5f,  sw, sh, -0.5f,  -sw, sh, -0.5f, 
  };

  unsigned int indices[] = {
      0, 1, 2,  2, 3, 0,  
      4, 5, 6,  6, 7, 4,  
      0, 4, 7,  7, 3, 0,  
      1, 5, 6,  6, 2, 1,  
      0, 1, 5,  5, 4, 0,  
      3, 2, 6,  6, 7, 3,  
      0, 1,  1, 2,  2, 3,  3, 0, 
      4, 5,  5, 6,  6, 7,  7, 4, 
      0, 4,  1, 5,  2, 6,  3, 7,  
      8, 9, 10,  10, 11, 8,
  };

  glGenVertexArrays(1, &vao);  
  glGenBuffers(1, &vbo);      
  glGenBuffers(1, &ebo);       
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  glBindVertexArray(0);

  GLuint vs = compileShader(GL_VERTEX_SHADER, vertexSrc);
  GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);
  program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glDeleteShader(vs);
  glDeleteShader(fs);
  mvpLoc = glGetUniformLocation(program, "uMVP");
  colorLoc = glGetUniformLocation(program, "uColor");

  glEnable(GL_DEPTH_TEST);          
  glEnable(GL_POLYGON_OFFSET_FILL); 
  glPolygonOffset(1.f, 1.f);        
  proj = projectionFromK(cam.K, size.width, size.height, 10.f, 10000.f);  
  return true;
}

bool glCubeDraw(bool found, const cv::Mat& rvec, const cv::Mat& tvec) {
  if (!window) return false;
  glfwMakeContextCurrent(window);

  if (found) {
    lastModelView = modelViewFromPose(rvec, tvec);
    hasPose = true;
  }

  int fbW, fbH;
  glfwGetFramebufferSize(window, &fbW, &fbH);  
  glViewport(0, 0, fbW, fbH);
  glClearColor(BACKGROUND.r, BACKGROUND.g, BACKGROUND.b, 1.f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (hasPose) {
    glm::mat4 mvp = proj * lastModelView;  
    glUseProgram(program);
    glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, glm::value_ptr(mvp));
    glBindVertexArray(vao);

    const size_t U = sizeof(unsigned int);

    glUniform3fv(colorLoc, 1, glm::value_ptr(SHEET_COLOR));
    glDrawElements(GL_TRIANGLES, N_SHEET_TRIS, GL_UNSIGNED_INT, (void*)((N_CUBE_TRIS + N_CUBE_EDGES) * U));

    glUniform3fv(colorLoc, 1, glm::value_ptr(CUBE_COLOR));   
    glDrawElements(GL_TRIANGLES, N_CUBE_TRIS, GL_UNSIGNED_INT, (void*)0);

    glUniform3fv(colorLoc, 1, glm::value_ptr(EDGE_COLOR));  
    glDrawElements(GL_LINES, N_CUBE_EDGES, GL_UNSIGNED_INT, (void*)(N_CUBE_TRIS * U));
  }

  glfwSwapBuffers(window); 
  glfwPollEvents();     

  return !(glfwWindowShouldClose(window) || glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS ||
           glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS);
}

void glCubeClose() {
  if (!window) return;
  glfwMakeContextCurrent(window);
  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
  glDeleteBuffers(1, &ebo);
  glDeleteProgram(program);
  glfwDestroyWindow(window);
  glfwTerminate();
  window = nullptr;
}
