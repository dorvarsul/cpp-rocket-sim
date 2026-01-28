#include "Camera.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <numbers>

Camera::Camera() {}

void Camera::update() {
  // Placeholder for physics-based camera movement or smoothing
}

void Camera::processInput(GLFWwindow *window) {
  if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    if (!m_isDragging) {
      m_isDragging = true;
      m_lastX = xpos;
      m_lastY = ypos;
    }

    double xoffset = xpos - m_lastX;
    double yoffset =
        m_lastY - ypos; // reversed since y-coordinates go from bottom to top

    m_lastX = xpos;
    m_lastY = ypos;

    float sensitivity = 0.3f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    m_yaw -= xoffset;
    m_pitch += yoffset;

    if (m_pitch > 89.0f)
      m_pitch = 89.0f;
    if (m_pitch < -89.0f)
      m_pitch = -89.0f;
  } else {
    m_isDragging = false;
  }

  // Scroll handling for zoom should be done via callback in main,
  // but for simplicity we can poll keys or just let user implement scroll
  // callback. Let's add simple key zoom for now.
  if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS)
    m_distance -= 1.0f;
  if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS)
    m_distance += 1.0f;
  if (m_distance < 1.0f)
    m_distance = 1.0f;
}

Eigen::Matrix4f Camera::getViewMatrix() const {
  // Convert spherical to cartesian
  float yawRad = m_yaw * (std::numbers::pi_v<float> / 180.0f);
  float pitchRad = m_pitch * (std::numbers::pi_v<float> / 180.0f);

  Eigen::Vector3f pos;
  pos.x() = m_distance * cos(pitchRad) * sin(yawRad);
  pos.y() =
      m_distance * cos(pitchRad) * cos(yawRad); // Swapped for ENU (Y is North)
  pos.z() = m_distance * sin(pitchRad);

  pos += m_target;

  // LookAt matrix
  Eigen::Vector3f forward = (m_target - pos).normalized();
  Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
  Eigen::Vector3f right = forward.cross(up).normalized();
  Eigen::Vector3f newUp = right.cross(forward);

  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view.block<1, 3>(0, 0) = right.transpose();
  view.block<1, 3>(1, 0) = newUp.transpose();
  view.block<1, 3>(2, 0) = -forward.transpose();
  view(0, 3) = -right.dot(pos);
  view(1, 3) = -newUp.dot(pos);
  view(2, 3) = forward.dot(pos);

  return view;
}

Eigen::Matrix4f Camera::getProjectionMatrix(float aspectRatio) const {
  float fov = 45.0f * (std::numbers::pi_v<float> / 180.0f);
  float near = 0.1f;
  float far = 200000.0f; // 200km - enough for long-range rockets

  float tanHalfFovy = tan(fov / 2.0f);

  Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();
  projection(0, 0) = 1.0f / (aspectRatio * tanHalfFovy);
  projection(1, 1) = 1.0f / (tanHalfFovy);
  projection(2, 2) = -(far + near) / (far - near);
  projection(2, 3) = -(2.0f * far * near) / (far - near);
  projection(3, 2) = -1.0f;

  return projection;
}
