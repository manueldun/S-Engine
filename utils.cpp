#include "utils.h"
#include "glm/fwd.hpp"
#include "glm/geometric.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <span>
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"

std::vector<char> readFile(const std::string &filename) {
  std::ifstream file(filename, std::ios::ate | std::ios::binary);

  if (!file.is_open()) {
    throw std::runtime_error("failed to open file!");
  }

  size_t fileSize{(size_t)file.tellg()};
  std::vector<char> buffer(fileSize);

  file.seekg(0);
  file.read(buffer.data(), fileSize);
  file.close();

  return buffer;
}

glm::mat3 star(const glm::vec3 &a) {
  return glm::mat3(glm::vec3(0.0f, -a.z, a.y), glm::vec3(a.z, 0.0f, -a.x),
                   glm::vec3(-a.y, a.x, 0.0f));
}
/* Determines if the normal of a triangle points towards a point following the
 * right hand ordering to determinate the triangle normal direction
 */
bool isTowardsPlaneNormal(const std::span<const glm::vec3> &trianglePlane,
                          const glm::vec3 &point) {

  const glm::vec3 triangleEdge1 = trianglePlane[1] - trianglePlane[0];
  const glm::vec3 triangleEdge2 = trianglePlane[2] - trianglePlane[0];
  const glm::vec3 normal = glm::cross(triangleEdge1, triangleEdge2);
  const float dotProduct = glm::dot(normal, point - trianglePlane[0]);
  return dotProduct > 0.0f;
}

tinygltf::Model loadGltfFile(const std::string &path) {
  tinygltf::Model model;
  tinygltf::TinyGLTF loader;
  std::string err;
  std::string warn;

  bool ret = loader.LoadASCIIFromFile(&model, &err, &warn, path);

  if (!warn.empty()) {
    std::cout << "Warn: " << warn << std::endl;
  }

  if (!ret) {
    throw std::runtime_error("Error loading gltf file");
  }

  if (!err.empty()) {
    std::cout << "Error: " << err << std::endl;
  }
  return model;
}

const std::span<const glm::vec3>
getVertexData(const tinygltf::Model &model, const std::string &attributeName) {

  const tinygltf::Mesh &mesh = model.meshes.at(0);
  assert(mesh.primitives.size() < 2);
  assert(mesh.primitives.size() > 0);
  assert(mesh.primitives[0].attributes.count(attributeName));

  const std::span<const tinygltf::Primitive> primitiveSpan(mesh.primitives);
  const int &primitiveIndex = primitiveSpan[0].attributes.at(attributeName);
  const tinygltf::Accessor &accessor = model.accessors[primitiveIndex];
  const int bufferViewIndex = accessor.bufferView;
  const tinygltf::BufferView &bufferView = model.bufferViews[bufferViewIndex];
  const int bufferIndex = bufferView.buffer;
  const std::span<const tinygltf::Buffer> bufferSpans(model.buffers);
  const std::span<const unsigned char> bufferSpan(
      bufferSpans[bufferIndex].data.data(),
      accessor.byteOffset + bufferView.byteOffset);
  assert(accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT);
  const size_t componentBytes = 4;
  size_t numberOfComponents = 0;
  switch (accessor.type) {
  case TINYGLTF_TYPE_SCALAR:
    numberOfComponents = 1;
    break;
  case TINYGLTF_TYPE_VEC2:
    numberOfComponents = 2;
    break;
  case TINYGLTF_TYPE_VEC3:
    numberOfComponents = 3;
    break;
  case TINYGLTF_TYPE_VEC4:
    numberOfComponents = 4;
    break;
  default:
    std::cerr << "accessor type not supported" << std::endl;
    assert(false);
  }
  const size_t numberOfVertices =
      bufferView.byteLength / componentBytes / numberOfComponents;
  const glm::vec3 *vec3ptr =
      reinterpret_cast<const glm::vec3 *>(bufferSpan.data());
  return std::span<const glm::vec3>(vec3ptr, numberOfVertices);
}
IndexDataSpan::IndexDataSpan(const std::span<const uint8_t> &indexSpan,
                             const uint8_t &bytesperInt)
    : m_indexSpan(indexSpan), m_bytesPerInt(bytesperInt) {}

const std::span<const uint8_t> IndexDataSpan::getIndexSpan() const {
  return m_indexSpan;
}
constexpr int getByteWidthFromComponentType(const int &componentType) {

  switch (componentType) {
  case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
    return 1;
    break;
  }
  case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
    return 2;
    break;
  }
  case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
    return 4;
    break;
  }
  default: {
    return 0;
  }
  }
}
const uint8_t IndexDataSpan::getBytesPerInt() const { return m_bytesPerInt; }
std::vector<IndexDataSpan> getIndexSpans(const tinygltf::Model &model) {

  std::vector<IndexDataSpan> bufferSpans;

  for (const tinygltf::Node &node : model.nodes) {
    const tinygltf::Mesh &mesh = model.meshes[node.mesh];
    assert(mesh.primitives.size() < 2);
    if (mesh.primitives.size() > 0) {

      const int &accesorIndex = mesh.primitives[0].indices;
      const tinygltf::Accessor &accessor = model.accessors[accesorIndex];
      const int &bufferViewIndex = accessor.bufferView;
      const tinygltf::BufferView &bufferView =
          model.bufferViews[bufferViewIndex];
      const int &bufferIndex = bufferView.buffer;
      const tinygltf::Buffer &buffer = model.buffers[bufferIndex];

      const uint8_t *intPtr = reinterpret_cast<const uint8_t *>(
          buffer.data.data() + bufferView.byteOffset + accessor.byteOffset);
      std::span<const uint8_t> bufferSpan(intPtr, bufferView.byteLength);

      bufferSpans.push_back(IndexDataSpan(
          bufferSpan, getByteWidthFromComponentType(accessor.componentType)));
      assert(accessor.type == TINYGLTF_TYPE_SCALAR);
    }
  }

  return bufferSpans;
}

template <typename T>
std::span<T> getindexSpans(const IndexDataSpan &indexSpan) {
  switch (indexSpan.getBytesPerInt()) {
  case 1: {
    return indexSpan.getIndexSpan();
  } break;
  case 2: {
    const uint16_t *ptr =
        reinterpret_cast<const uint16_t *>(indexSpan.getIndexSpan().data());
    const std::span<const uint16_t> span =
        std::span(ptr, ptr + indexSpan.getIndexSpan().size() / 2);
    return span;
  } break;
  case 4: {
    const uint32_t *ptr =
        reinterpret_cast<const uint32_t *>(indexSpan.getIndexSpan().data());
    const std::span<const uint32_t> span =
        std::span(ptr, ptr + indexSpan.getIndexSpan().size() / 4);
    return span;
  } break;
  }
}

const glm::vec3 getCenterOfMass(const std::span<const glm::vec3> &vertices,
                                const IndexDataSpan &dataSpan,
                                const bool &verbose) {

  switch (dataSpan.getBytesPerInt()) {
  case 1: {
    return getCenterOfMass<const uint8_t>(vertices, dataSpan.getIndexSpan(),
                                          verbose);
  } break;
  case 2: {
    const uint16_t *intPtr =
        reinterpret_cast<const uint16_t *>(dataSpan.getIndexSpan().data());
    size_t numberOfIndices =
        dataSpan.getIndexSpan().size() / dataSpan.getBytesPerInt();
    std::span<const uint16_t> span16(intPtr, numberOfIndices);
    return getCenterOfMass<const uint16_t>(vertices, span16, verbose);
    break;
  }
  case 4: {
    const uint32_t *intPtr =
        reinterpret_cast<const uint32_t *>(dataSpan.getIndexSpan().data());
    auto numberOfIndices =
        dataSpan.getIndexSpan().size() / dataSpan.getBytesPerInt();
    std::span<const uint32_t> span32(intPtr, numberOfIndices);
    return getCenterOfMass<const uint32_t>(vertices, span32, verbose);
  } break;
  default:
    assert(false);
  }
}

const glm::mat3 getInertiaTensor(const std::span<const glm::vec3> &vertices,
                                 const IndexDataSpan &dataSpan,
                                 const bool &verbose) {

  switch (dataSpan.getBytesPerInt()) {
  case 1: {
    return getInertiaTensor<const uint8_t>(vertices, dataSpan.getIndexSpan(),
                                           verbose);
  } break;
  case 2: {
    const uint16_t *intPtr =
        reinterpret_cast<const uint16_t *>(dataSpan.getIndexSpan().data());
    size_t numberOfIndices =
        dataSpan.getIndexSpan().size() / dataSpan.getBytesPerInt();
    std::span<const uint16_t> span16(intPtr, numberOfIndices);
    return getInertiaTensor<const uint16_t>(vertices, span16, verbose);
    break;
  }
  case 3: {
    const uint32_t *intPtr =
        reinterpret_cast<const uint32_t *>(dataSpan.getIndexSpan().data());
    auto numberOfIndices =
        dataSpan.getIndexSpan().size() / dataSpan.getBytesPerInt();
    std::span<const uint32_t> span32(intPtr, intPtr + numberOfIndices);
    return getInertiaTensor<const uint32_t>(vertices, span32, verbose);
  } break;
  default:
    assert(false);
  }
}

template <typename T>
const glm::vec3 getCenterOfMass(const std::span<const glm::vec3> &vertices,
                                const std::span<T> &indices,
                                const bool &verbose) {

  unsigned int numOfVertices = indices.size();
  glm::vec3 centerOfMass = glm::vec3(0.0f);
  float totalArea = 0.0f;

  if (verbose) {
    std::cout << "numOfTriangles: " << numOfVertices << std::endl;
  }
  for (size_t vertexIndex = 0; vertexIndex < numOfVertices; vertexIndex += 3) {
    if (verbose) {
      std::cout << "vertexIndex: " << vertexIndex << std::endl;
      std::cout << "indices" << std::endl;
      std::cout << indices[vertexIndex] << std::endl;
      std::cout << indices[vertexIndex + 1] << std::endl;
      std::cout << indices[vertexIndex + 2] << std::endl;
    }

    const glm::vec3 &vertex1 = vertices[indices[vertexIndex]];
    if (verbose) {
      std::cout << "vec1" << std::endl;

      std::cout << vertex1.x << " " << vertex1.y << " " << vertex1.z
                << std::endl;
    }
    const glm::vec3 &vertex2 = vertices[indices[vertexIndex + 1]];
    if (verbose) {
      std::cout << "vec2" << std::endl;
      std::cout << vertex2.x << " " << vertex2.y << " " << vertex2.z
                << std::endl;
    }
    const glm::vec3 &vertex3 = vertices[indices[vertexIndex + 2]];
    if (verbose) {
      std::cout << "vec3" << std::endl;
      std::cout << vertex3.x << " " << vertex3.y << " " << vertex3.z
                << std::endl;
    }

    const glm::vec3 triangleCentroid = (vertex1 + vertex2 + vertex3) / 3.0f;

    if (verbose) {
      std::cout << "centroid" << std::endl;
      std::cout << triangleCentroid.x << " " << triangleCentroid.y << " "
                << triangleCentroid.z << std::endl;
    }
    glm::vec3 a{vertex2 - vertex1};
    glm::vec3 b{vertex3 - vertex1};
    glm::vec3 crossVector{glm::cross(a, b)};
    float area = crossVector.length() / 2.0f;
    totalArea += area;
    centerOfMass += triangleCentroid * area;
    if (verbose) {
      std::cout << "Acumulated center of mass:" << std::endl;
      std::cout << centerOfMass.x << " " << centerOfMass.y << " "
                << centerOfMass.z << std::endl;
    }
  }
  return centerOfMass / totalArea;
}

template <typename T>
const glm::mat3 getInertiaTensor(const std::span<const glm::vec3> &vertices,
                                 const std::span<T> &indices,
                                 const bool &verbose) {

  glm::mat3 acumulatedInertialTensor = glm::mat3(0.0f);

  uint32_t numsOfTetrahedrons = 0;
  for (size_t vertexIndex = 0; vertexIndex < indices.size(); vertexIndex += 3) {

    glm::vec3 vertex1 = glm::vec3(0.0f);
    glm::vec3 vertex2 = vertices[indices[vertexIndex]];
    glm::vec3 vertex3 = vertices[indices[vertexIndex + 1]];
    glm::vec3 vertex4 = vertices[indices[vertexIndex + 2]];

    glm::mat3 jacobian = glm::mat3(vertex2, vertex3, vertex4);
    float det = glm::determinant(jacobian);

    if (verbose) {
      std::cout << "First Vertex: " << std::endl;
      std::cout << vertex2.x << "\t" << vertex2.y << "\t" << vertex2.z
                << std::endl;
      std::cout << "Second Vertex: " << std::endl;
      std::cout << vertex3.x << "\t" << vertex3.y << "\t" << vertex3.z
                << std::endl;
      std::cout << "Third Vertex: " << std::endl;
      std::cout << vertex4.x << "\t" << vertex4.y << "\t" << vertex4.z
                << std::endl;

      std::cout << "Jacobian:" << std::endl;
      std::cout << jacobian[0][0] << "\t" << jacobian[0][1] << "\t"
                << jacobian[0][2] << "\t" << std::endl;

      std::cout << jacobian[1][0] << "\t" << jacobian[1][1] << "\t"
                << jacobian[1][2] << "\t" << std::endl;

      std::cout << jacobian[2][0] << "\t" << jacobian[2][1] << "\t"
                << jacobian[2][2] << "\t" << std::endl;
      std::cout << "Determinant of the jacobian : " << det << std::endl;
    }

    float abcx = vertex1.x * vertex1.x + vertex1.x * vertex2.x +
                 vertex2.x * vertex2.x + vertex1.x * vertex3.x +
                 vertex2.x * vertex3.x + vertex3.x * vertex3.x +
                 vertex1.x * vertex4.x + vertex2.x * vertex4.x +
                 vertex3.x * vertex4.x + vertex4.x * vertex4.x;

    float abcy = vertex1.y * vertex1.y + vertex1.y * vertex2.y +
                 vertex2.y * vertex2.y + vertex1.y * vertex3.y +
                 vertex2.y * vertex3.y + vertex3.y * vertex3.y +
                 vertex1.y * vertex4.y + vertex2.y * vertex4.y +
                 vertex3.y * vertex4.y + vertex4.y * vertex4.y;

    float abcz = vertex1.z * vertex1.z + vertex1.z * vertex2.z +
                 vertex2.z * vertex2.z + vertex1.z * vertex3.z +
                 vertex2.z * vertex3.z + vertex3.z * vertex3.z +
                 vertex1.z * vertex4.z + vertex2.z * vertex4.z +
                 vertex3.z * vertex4.z + vertex4.z * vertex4.z;
    float a = (det * (abcy + abcz) / 60.0f) / (det / 6.0);
    float b = (det * (abcx + abcz) / 60.0f) / (det / 6.0);
    float c = (det * (abcx + abcy) / 60.0f) / (det / 6.0);

    float abcxp =
        vertex2.y * vertex1.z + vertex3.y * vertex1.z + vertex4.y * vertex1.z +
        vertex1.y * vertex2.z + vertex3.y * vertex2.z + vertex4.y * vertex2.z +
        vertex1.y * vertex3.z + vertex2.y * vertex3.z + vertex4.y * vertex3.z +
        vertex1.y * vertex4.z + vertex2.y * vertex4.z + vertex3.y * vertex4.z +
        2.0f * vertex1.y * vertex1.z + 2.0f * vertex2.y * vertex2.z +
        2.0f * vertex3.y * vertex3.z + 2.0f * vertex4.y * vertex4.z;

    float abcyp =
        vertex2.x * vertex1.z + vertex3.x * vertex1.z + vertex4.x * vertex1.z +
        vertex1.x * vertex2.z + vertex3.x * vertex2.z + vertex4.x * vertex2.z +
        vertex1.x * vertex3.z + vertex2.x * vertex3.z + vertex4.x * vertex3.z +
        vertex1.x * vertex4.z + vertex2.x * vertex4.z + vertex3.x * vertex4.z +
        2.0f * vertex1.x * vertex1.z + 2.0f * vertex2.x * vertex2.z +
        2.0f * vertex3.x * vertex3.z + 2.0f * vertex4.x * vertex4.z;

    float abczp =
        vertex2.x * vertex1.y + vertex3.x * vertex1.y + vertex4.x * vertex1.y +
        vertex1.x * vertex2.y + vertex3.x * vertex2.y + vertex4.x * vertex2.y +
        vertex1.x * vertex3.y + vertex2.x * vertex3.y + vertex4.x * vertex3.y +
        vertex1.x * vertex4.y + vertex2.x * vertex4.y + vertex3.x * vertex4.y +
        2.0f * vertex1.x * vertex1.y + 2.0f * vertex2.x * vertex2.y +
        2.0f * vertex3.x * vertex3.y + 2.0f * vertex4.x * vertex4.y;

    float ap = (det * abcxp / 120.0f) / (det / 6.0);
    float bp = (det * abcyp / 120.0f) / (det / 6.0);
    float cp = (det * abczp / 120.0f) / (det / 6.0);

    float tetrahedronInertiaTensorArray[] = {a,   -bp, -cp, -bp, b,
                                             -ap, -cp, -ap, c};
    glm::mat3 tetrahedronInertiaTensor =
        glm::make_mat3(tetrahedronInertiaTensorArray);
    if (verbose) {

      std::cout << "Acumulated tetrahedron inertial tensor:" << std::endl;
      std::cout << tetrahedronInertiaTensor[0][0] << "\t"
                << tetrahedronInertiaTensor[0][1] << "\t"
                << tetrahedronInertiaTensor[0][2] << "\t" << std::endl;

      std::cout << tetrahedronInertiaTensor[1][0] << "\t"
                << tetrahedronInertiaTensor[1][1] << "\t"
                << tetrahedronInertiaTensor[1][2] << "\t" << std::endl;

      std::cout << tetrahedronInertiaTensor[2][0] << "\t"
                << tetrahedronInertiaTensor[2][1] << "\t"
                << tetrahedronInertiaTensor[2][2] << "\t" << std::endl;
    }
    acumulatedInertialTensor += tetrahedronInertiaTensor;
    numsOfTetrahedrons++;
    if (verbose)
      std::cout << "Number of tetrahedrons: " << numsOfTetrahedrons
                << std::endl;
  }
  return acumulatedInertialTensor / static_cast<float>(numsOfTetrahedrons);
}
const bool doCollideVertexBased(const std::span<const glm::vec3> vertices1,
                                const std::span<const glm::vec3> vertices2) {
  for (size_t i = 0; i < vertices1.size() - 3; i++) {
    const std::span<const glm::vec3> vertexPlane = vertices1.subspan(i, 3);
    bool allOnOneSide = true;
    bool isTowards1 = isTowardsPlaneNormal(vertexPlane, vertices1[0]);
    for (const glm::vec3 &vertex : vertices1.subspan(1)) {
      if (isTowards1 != isTowardsPlaneNormal(vertexPlane, vertex)) {
        allOnOneSide = false;
      }
    }
    if (allOnOneSide) {
      allOnOneSide = true;
      bool isTowards2 = !isTowards1;
      for (const glm::vec3 &vertex : vertices2) {
        if (isTowards2 != isTowardsPlaneNormal(vertexPlane, vertex)) {
          return false;
        }
      }
    }
  }
  return true;
}

const bool
doCollideTriangleMeshBased(const std::span<const glm::vec3> vertices1,
                           const std::span<const glm::vec3> vertices2) {
  assert(vertices1.size() % 3 == 0);
  assert(vertices2.size() % 3 == 0);
  for (size_t i = 0; i < vertices1.size() - 3; i += 3) {
    const std::span<const glm::vec3> vertexPlane = vertices1.subspan(i, 3);
    bool isTowards = true;
    for (const glm::vec3 &vertex : vertices2) {
      if (!isTowardsPlaneNormal(vertexPlane, vertex)) {
        isTowards = false;
      }
    }
    if (isTowards) {
      isTowards = false;
      for (size_t j = 0; j < vertices2.size(); j += 3) {
        if (i != j && !isTowardsPlaneNormal(vertexPlane, vertices1[j])) {
          return true;
        }
      }
      return false;
    }
  }
  return true;
}
