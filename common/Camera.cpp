#include "Camera.h"
#include "glm/gtc/matrix_transform.hpp"

ProjectionCamera::ProjectionCamera()
    : aspectRatio(800.0f / 600.0f), fovy(0.4f), near(0.1f), far(1000.0f) {}
ProjectionCamera::ProjectionCamera(const float &aspectRatio, const float &fovy,
                                   const float &near, const float &far)
    : aspectRatio(aspectRatio), fovy(fovy), near(near), far(far) {}
glm::mat4 ProjectionCamera::getProjectionMatrix() {
  return glm::perspective(fovy, aspectRatio, near, far);
}
