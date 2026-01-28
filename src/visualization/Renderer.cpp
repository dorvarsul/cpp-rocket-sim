#include "Renderer.h"
#include <iostream>

const char *vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;
    
    uniform mat4 view;
    uniform mat4 projection;
    
    out vec3 Color;
    
    void main() {
        gl_Position = projection * view * vec4(aPos, 1.0);
        Color = aColor;
    }
)";

const char *fragmentShaderSource = R"(
    #version 330 core
    in vec3 Color;
    out vec4 FragColor;
    
    void main() {
        FragColor = vec4(Color, 1.0);
    }
)";

Renderer::Renderer()
    : m_shaderProgram(0), m_vao(0), m_vbo(0), m_gridVao(0), m_gridVbo(0) {}

Renderer::~Renderer() {
  glDeleteVertexArrays(1, &m_vao);
  glDeleteBuffers(1, &m_vbo);
  glDeleteVertexArrays(1, &m_gridVao);
  glDeleteBuffers(1, &m_gridVbo);
  glDeleteProgram(m_shaderProgram);
}

void Renderer::init() {
  m_shaderProgram = createShader(vertexShaderSource, fragmentShaderSource);

  // Setup Trajectory buffers
  glGenVertexArrays(1, &m_vao);
  glGenBuffers(1, &m_vbo);

  // Setup Grid buffers
  glGenVertexArrays(1, &m_gridVao);
  glGenBuffers(1, &m_gridVbo);

  // Create grid data (lines)
  // 1km x 1km grid
  int size = 2000;
  int step = 100;
  for (int i = -size; i <= size; i += step) {
    // Lines parallel to X (East)
    m_gridVertices.push_back((float)-size);
    m_gridVertices.push_back((float)i);
    m_gridVertices.push_back(0.0f); // Pos
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f); // Color

    m_gridVertices.push_back((float)size);
    m_gridVertices.push_back((float)i);
    m_gridVertices.push_back(0.0f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);

    // Lines parallel to Y (North)
    m_gridVertices.push_back((float)i);
    m_gridVertices.push_back((float)-size);
    m_gridVertices.push_back(0.0f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);

    m_gridVertices.push_back((float)i);
    m_gridVertices.push_back((float)size);
    m_gridVertices.push_back(0.0f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
  }

  // Add Axis lines
  // X (Red)
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(100);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);

  // Y (Green)
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(100);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);

  // Z (Blue)
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(100);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);

  glBindVertexArray(m_gridVao);
  glBindBuffer(GL_ARRAY_BUFFER, m_gridVbo);
  glBufferData(GL_ARRAY_BUFFER, m_gridVertices.size() * sizeof(float),
               m_gridVertices.data(), GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
}

void Renderer::render(const Eigen::Matrix4f &view, const Eigen::Matrix4f &proj,
                      const std::vector<std::vector<StateVector>> &trajectories,
                      float cameraDistance) {
  // Check for pre-existing errors
  GLenum err;
  while ((err = glGetError()) != GL_NO_ERROR) {
    std::cerr << "OpenGL Error BEFORE render: " << err << std::endl;
  }

  if (m_shaderProgram == 0) {
    std::cerr << "Error: Shader Program is 0!" << std::endl;
    return;
  }

  glUseProgram(m_shaderProgram);

  // Reset State that ImGui or others might have changed
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);

  GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");
  GLint projLoc = glGetUniformLocation(m_shaderProgram, "projection");

  if (viewLoc == -1 || projLoc == -1) {
    std::cerr
        << "Warning: Uniform 'view' or 'projection' not found (optimized out?)"
        << std::endl;
  }

  glUniformMatrix4fv(viewLoc, 1, GL_FALSE, view.data());
  glUniformMatrix4fv(projLoc, 1, GL_FALSE, proj.data());

  // Fixed grid: always 20x20 cells, but cells get bigger as you zoom out
  static const int GRID_LINES = 10; // 10 lines per direction = 10x10 cells

  // Cell size scales with camera distance
  int cellSize =
      std::max(100, (int)(cameraDistance / 5.0)); // Cells grow with distance
  int gridSize = cellSize * GRID_LINES;           // Total grid extent

  // Regenerate grid if camera distance changed significantly (cell size
  // changed)
  static int lastCellSize = 0;
  if (std::abs(cellSize - lastCellSize) > cellSize * 0.3f) {
    lastCellSize = cellSize;
    m_gridVertices.clear();

    for (int i = -GRID_LINES; i <= GRID_LINES; i++) {
      float pos = i * cellSize;

      // Lines parallel to X (East)
      m_gridVertices.push_back((float)-gridSize);
      m_gridVertices.push_back(pos);
      m_gridVertices.push_back(0.0f);
      m_gridVertices.push_back(0.4f); // Subtle gray
      m_gridVertices.push_back(0.4f);
      m_gridVertices.push_back(0.4f);

      m_gridVertices.push_back((float)gridSize);
      m_gridVertices.push_back(pos);
      m_gridVertices.push_back(0.0f);
      m_gridVertices.push_back(0.4f);
      m_gridVertices.push_back(0.4f);
      m_gridVertices.push_back(0.4f);

      // Lines parallel to Y (North)
      m_gridVertices.push_back(pos);
      m_gridVertices.push_back((float)-gridSize);
      m_gridVertices.push_back(0.0f);
      m_gridVertices.push_back(0.4f);
      m_gridVertices.push_back(0.4f);
      m_gridVertices.push_back(0.4f);

      m_gridVertices.push_back(pos);
      m_gridVertices.push_back((float)gridSize);
      m_gridVertices.push_back(0.0f);
      m_gridVertices.push_back(0.4f);
      m_gridVertices.push_back(0.4f);
      m_gridVertices.push_back(0.4f);
    }

    // Add axis lines (scaled appropriately)
    float axisLength = gridSize / 10.0f;
    if (axisLength < 100)
      axisLength = 100;

    // X (Red)
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(1);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(axisLength);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(1);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);

    // Y (Green)
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(1);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(axisLength);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(1);
    m_gridVertices.push_back(0);

    // Z (Blue)
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(1);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(axisLength);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(0);
    m_gridVertices.push_back(1);

    // Update GPU buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVbo);
    glBufferData(GL_ARRAY_BUFFER, m_gridVertices.size() * sizeof(float),
                 m_gridVertices.data(), GL_DYNAMIC_DRAW);
  }

  // Draw Grid - subtle reference lines
  float lineWidth = 1.0f; // Subtle, constant width
  glLineWidth(lineWidth);
  glBindVertexArray(m_gridVao);
  glDrawArrays(GL_LINES, 0, m_gridVertices.size() / 6);

  // Draw Trajectories with colors matching ImPlot
  // ImPlot default color palette (matching the 2D graphs)
  static const float colors[][3] = {
      {0.0f, 0.7490196228f, 1.0f}, // Cyan
      {1.0f, 0.0f, 0.0f},          // Red
      {0.4980392158f, 1.0f, 0.0f}, // Green
      {1.0f, 1.0f, 0.0f},          // Yellow
      {0.0f, 0.4980392158f, 1.0f}, // Blue
      {1.0f, 0.0f, 1.0f},          // Magenta
      {0.0f, 1.0f, 1.0f},          // Cyan
      {1.0f, 0.5019607842f, 0.0f}, // Orange
  };
  static const int numColors = 8;

  std::vector<float> trajVertices;
  int trajIndex = 0;
  for (const auto &history : trajectories) {
    if (history.empty()) {
      trajIndex++;
      continue;
    }

    // Get color for this trajectory
    const float *color = colors[trajIndex % numColors];

    for (const auto &state : history) {
      trajVertices.push_back((float)state.position.x());
      trajVertices.push_back((float)state.position.y());
      trajVertices.push_back((float)state.position.z());

      // Use trajectory-specific color
      trajVertices.push_back(color[0]);
      trajVertices.push_back(color[1]);
      trajVertices.push_back(color[2]);
    }
    trajIndex++;
  }

  if (!trajVertices.empty()) {
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, trajVertices.size() * sizeof(float),
                 trajVertices.data(), GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Draw trajectories with thin lines
    int offset = 0;
    glLineWidth(1.8f); // Thin lines
    glPointSize(4.0f); // Small points

    for (const auto &history : trajectories) {
      if (history.empty())
        continue;
      // Draw line
      glDrawArrays(GL_LINE_STRIP, offset, history.size());
      // Draw points
      glDrawArrays(GL_POINTS, offset, history.size());

      offset += history.size();
    }
  }
}

GLuint Renderer::createShader(const char *vertexSource,
                              const char *fragmentSource) {
  GLint success;
  char infoLog[512];

  GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexSource, NULL);
  glCompileShader(vertexShader);

  // Check for compile errors
  glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }

  GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
  glCompileShader(fragmentShader);

  // Check for compile errors
  glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }

  GLuint shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);

  // Check for link errors
  glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
              << infoLog << std::endl;
  }

  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  return shaderProgram;
}
