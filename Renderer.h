#pragma once
#include "stb_image.h"
#include "glm/fwd.hpp"
#include "imgui_impl_vulkan.h"
#include "tiny_gltf.h"
#include <vulkan/vulkan_core.h>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <vk_mem_alloc.h>

#include <optional>
#include <string>
#include <vector>

static void check_vk_result(VkResult err) {
  if (err == 0)
    return;
  fprintf(stderr, "[vulkan] Error: VkResult = %d\n", err);
  if (err < 0)
    abort();
}

struct UniformBufferObject {
  glm::mat4 model;
  glm::mat4 view;
  glm::mat4 projection;
};

struct Vertex {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 texCoord;
  static std::vector<VkVertexInputBindingDescription> getBindingDescriptions();
  static std::vector<VkVertexInputAttributeDescription>
  getAttributeDescriptions();
};

struct BufferData {
  VkBuffer indexBuffer;
  VmaAllocation indexAllocation;

  VkBuffer positionBuffer;
  VmaAllocation positionAllocation;

  VkBuffer texCoordBuffer;
  VmaAllocation texCoordAllocation;

  VkBuffer normalBuffer;
  VmaAllocation normalAllocation;

  VkBuffer tangentbuffer;
  VmaAllocation tangentAllocation;
};

class MemoryData {
public:
  friend class Renderer;
  MemoryData() = delete;
  MemoryData(const VkBuffer &buffer, const VmaAllocation &allocation);

private:
  const VkBuffer m_buffer;
  const VmaAllocation m_allocation;
};

class Texture {
public:
  friend class Renderer;
  Texture() = delete;
  Texture(const VkImage &image, const VkImageView &imageView,
          const VmaAllocation &allocation, const VkSampler &sampler,
          const std::vector<VkDescriptorSet> &descriptorSet);
  void clean(const VkDevice &device, const VmaAllocator &allocator) const;

private:
  const VkImage m_image;
  const VkImageView m_imageView;
  const VmaAllocation m_allocation;
  const VkSampler m_sampler;
  const std::vector<VkDescriptorSet> m_descriptorSet;
};

class Drawble {
public:
  friend class Renderer;
  Drawble() = delete;
  Drawble(const BufferData &bufferData, const VkPipeline &pipeline,
          const uint32_t indexOffset, const uint32_t count,
          const VkIndexType &indexType, const uint32_t &indexToTexture,
          const uint32_t &indexToDescriptor);

private:
  const BufferData m_bufferData;
  const VkPipeline m_pipeline;
  const uint32_t m_indexOffset;
  const uint32_t m_count;
  const VkIndexType m_indexType;
  const uint32_t m_indexToTexture;
  const uint32_t m_indexToDescriptor;
};

class Node {
public:
  friend class Renderer;
  Node() = delete;
  Node(const std::string &name, const std::vector<Drawble> &drawbles,
       const glm::mat4 &initialMatrix);
  glm::mat4 getInitialMatrix() const;
  void setMatrix(const glm::mat4 &matrix);

private:
  glm::mat4 m_matrix;
  const std::string m_name;
  const std::vector<Drawble> m_drawbles;
};

const std::vector<Vertex> vertices = {
    {{-0.5f, -0.5f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
    {{0.5f, -0.5f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
    {{0.5f, 0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
    {{-0.5f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 1.0f}},

    {{-0.5f, -0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
    {{0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
    {{0.5f, 0.5f, -0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
    {{-0.5f, 0.5f, -0.5f}, {1.0f, 1.0f, 1.0f}, {0.0f, 1.0f}}};

const std::vector<uint16_t> indices = {0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4};

VkResult CreateDebugUtilsMessengerEXT(
    const VkInstance &instance,
    const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
    const VkAllocationCallbacks *pAllocator,
    VkDebugUtilsMessengerEXT *pDebugMessenger);

void DestroyDebugUtilsMessengerEXT(
    const VkInstance &instance, const VkDebugUtilsMessengerEXT &debugMessenger,
    const VkAllocationCallbacks *pAllocator);

struct QueueFamilyIndices {
  std::optional<uint32_t> graphicsFamily;
  std::optional<uint32_t> presentFamily;
  bool isComplete() const;
};

struct SwapChainSupportDetails {
  VkSurfaceCapabilitiesKHR capabilities;
  std::vector<VkSurfaceFormatKHR> formats;
  std::vector<VkPresentModeKHR> presentModes;
};

class RenderObject;
class Renderer {
public:
  friend class RenderObject;
  Renderer();
  RenderObject loadModel(const tinygltf::Model &model);
  bool shouldExit();
  void draw(RenderObject *const fmrenderObject);
  void drawLabel(const std::string *label);
  void endFrame();
  void destroy();

private:
  void init();

  void initWindow();

  void initVulkan();

  void createSurface();
  void createInstance();
  bool checkValidationLayerSupport() const;
  void pickPhysicalDevice();
  bool isDeviceSuitable(VkPhysicalDevice device);
  bool checkDeviceExtensionSupport(
      const VkPhysicalDevice &device); // delete this coment

  void createLogicalDevice();
  void createVMA();
  void createSwapChain();
  VkSurfaceFormatKHR chooseSwapSurfaceFormat(
      const std::vector<VkSurfaceFormatKHR> &availableFormats);
  VkPresentModeKHR chooseSwapPresentMode(
      const std::vector<VkPresentModeKHR> &availablePresentModes);
  VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities);
  QueueFamilyIndices findQueueFamilies(const VkPhysicalDevice &device);
  SwapChainSupportDetails querySwapChainSupport(const VkPhysicalDevice &device);

  void createImageViews();
  VkImageView createImageView(const VkImage &image, const VkFormat &format,
                              const VkImageAspectFlags &aspectFlags);
  void createRenderPass();
  VkFormat findDepthFormat();
  VkFormat findSupportedFormat(const std::vector<VkFormat> &candidates,
                               const VkImageTiling &tiling,
                               const VkFormatFeatureFlags &features);
  void createDepthResources();
  void createFramebuffers();
  void createCommandPool();
  void createTextureImage();
  void createImage(const uint32_t &width, const uint32_t &height,
                   const VkFormat &format, const VkImageTiling &tiling,
                   const VkImageUsageFlags &usage,
                   const VmaMemoryUsage &memoryUsage,
                   const VmaAllocationCreateFlags &allocationFlags,
                   VkImage &image, VmaAllocation &allocation);
  void createTextureImageView();
  void createTextureSampler();
  void createVertexBuffer();
  void copyBuffer(const VkBuffer &srcBuffer, const VkBuffer &dstBuffer,
                  const VkDeviceSize &size);
  VkCommandBuffer beginSingleTimeCommands();
  void transitionImageLayout(const VkImage &image, const VkFormat &format,
                             const VkImageLayout &oldLayout,
                             const VkImageLayout &newLayout);
  void copyBufferImage(const VkBuffer &buffer, const VkImage &image,
                       const uint32_t &width, const uint32_t &height);
  void endSingleTimeCommands(VkCommandBuffer &commandBuffer);
  void createDescriptorPool();
  void createDescriptorSetLayouts();
  void allocateUboDescriptorSets();
  void createIndexBuffer();
  void createCommandBuffer();
  void createSyncObjects();

  void initImGui();

  static void framebufferResizeCallback(GLFWwindow *window, int width,
                                        int height);

  std::vector<const char *> getRequiredExtensions();
  void setupDebugMessenger();
  static VKAPI_ATTR VkBool32 VKAPI_CALL
  debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                VkDebugUtilsMessageTypeFlagsEXT messageType,
                const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
                void *pUserData);

  VkPipeline
  createGraphicPipeline(const std::string &vertexShaderPath,
                        const std::string &fragmentShaderPath,
                        const std::vector<VkVertexInputBindingDescription>
                            &vertexInpitBindingDescription,
                        const std::vector<VkVertexInputAttributeDescription>
                            &vertexInputAttributeDescription);
  VkShaderModule createShaderModule(const std::vector<char> &code);

  std::vector<VkDescriptorSet>
  allocateTextureDescriptorSet(const VkImageView &imageview,
                               const VkSampler &sampler);
  void createBuffer(const VkDeviceSize &size, const VkBufferUsageFlags &usage,
                    const VmaMemoryUsage &memoryUsage,
                    const VmaAllocationCreateFlags &allocationFlags,
                    VkBuffer &buffer, VmaAllocation &allocation);
  void drawFrame();
  void updateUniformBuffer(const uint32_t &currentImage);
  void cleanupSwapChain();
  void recreateSwapChain();
  void cleanup();

  void setMatrix(const uint32_t &index, const glm::mat4 &matrix);
  void resizeDescriptorSets();
  bool hasStencilComponent(VkFormat format);

  GLFWwindow *m_window;
  VkSurfaceKHR m_surface;
  VkInstance m_instance;
  VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
  const std::vector<const char *> m_deviceExtensions = {
      VK_KHR_SWAPCHAIN_EXTENSION_NAME,
      VK_KHR_SHADER_NON_SEMANTIC_INFO_EXTENSION_NAME};
  VkDevice m_device;
  VkQueue m_queue;
  VkQueue m_presentQueue;
  QueueFamilyIndices m_queueFamilyIndices;
  VmaAllocator m_vmaAllocator;
  VkFormat m_swapChainImageFormat;
  std::vector<VkImage> m_swapChainImages;
  std::vector<VkImageView> m_swapChainImageViews;
  uint32_t m_imageCount;
  VkRenderPass m_renderPass;

  VkImage m_depthImage;
  VmaAllocation m_depthAllocation;
  VkSwapchainKHR m_swapchain;
  std::vector<VkFramebuffer> m_swapChainFramebuffers;
  VkImageView m_depthImageView;
  VkCommandPool m_commandPool;
  VkExtent2D swapChainExtent;
  VkImage m_textureImage;
  VmaAllocation m_textureAllocation;
  VkImageView m_textureImageView;
  VkSampler m_textureSampler;
  VmaAllocation m_vertexAllocation;
  VmaAllocation m_indexAllocation;
  VkBuffer m_vertexBuffer;
  VkDescriptorPool m_firstDescriptorPool;
  VkDescriptorSetLayout m_uboDescriptorSetLayout;
  VkDescriptorSetLayout m_textureDescriptorSetLayout;
  std::vector<VkDescriptorSet> m_uboDescriptorSets;
  VkBuffer m_indexBuffer;
  std::vector<VkCommandBuffer> m_commandBuffers;
  std::vector<VkSemaphore> m_imageAvailableSemaphores;
  std::vector<VkSemaphore> m_renderFinishedSemaphores;
  std::vector<VkFence> m_inFlightFences;
  bool m_framebufferResized = false;

  std::vector<const std::string *> m_guiLabels;

  const u_int32_t WIDTH = 800;
  const u_int32_t HEIGHT = 600;
  const std::vector<const char *> validationLayers = {
      "VK_LAYER_KHRONOS_validation"};

#ifdef NDEBUG
  const bool enableValidationLayers = false;
#else
  const bool enableValidationLayers = true;
#endif
  std::vector<const char *> extensions;
  VkDebugUtilsMessengerEXT m_debugMessenger;

  VkPipeline m_pipeline;
  VkPipelineLayout m_pipelineLayout;
  uint32_t m_currentFrame = 0;
  std::vector<Texture> m_loadedTextures;

  std::vector<VkBuffer> m_uboBuffer;
  std::vector<VmaAllocation> m_uboAllocation;
  std::vector<void *> m_uniformBufferMapped;
  void createUniformBuffers();
  uint32_t findMemoryType(const uint32_t &typeFilter,
                          const VkMemoryPropertyFlags &properties);
  ImGui_ImplVulkanH_Window m_imguiWindow;
  std::vector<BufferData> m_loadedBufferData;

  std::vector<glm::mat4> m_modelMatrices;
  std::vector<Node *> m_pNodeToDraw;
};

class RenderObject {
public:
  friend class Renderer;
  RenderObject() = delete;
  RenderObject(const std::vector<Node> &nodes, const tinygltf::Model &model);
  void setMatrix(const glm::mat4 &matrix, const uint32_t index);
  RenderObject(const RenderObject &other) = default;
  glm::vec3 getCenterOfMass(const bool &verbose) const;
  glm::mat4 getInertiaTensor(const bool &verbose) const;

private:
  std::vector<Node> m_nodes;
  tinygltf::Model m_model;
};
