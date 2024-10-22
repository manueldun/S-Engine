#pragma once
#include "glm/fwd.hpp"
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
          const std::vector<VkDescriptorSet> descriptorSet);
  void clean(const VkDevice &device, const VmaAllocator &allocator);

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
  std::string getName();

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
  Node(const std::string name, const std::vector<Drawble> drawbles,
       const glm::mat4 initialMatrix);
  const glm::mat4 getInitialMatrix();
  void setMatrix(const glm::mat4 &matrix);

private:
  glm::mat4 m_matrix;
  const std::string m_name;
  std::vector<Drawble> m_drawbles;
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

std::vector<char> readFile(const std::string &filename);

VkResult CreateDebugUtilsMessengerEXT(
    VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
    const VkAllocationCallbacks *pAllocator,
    VkDebugUtilsMessengerEXT *pDebugMessenger);

void DestroyDebugUtilsMessengerEXT(VkInstance instance,
                                   VkDebugUtilsMessengerEXT debugMessenger,
                                   const VkAllocationCallbacks *pAllocator);

struct QueueFamilyIndices {
  std::optional<uint32_t> graphicsFamily;
  std::optional<uint32_t> presentFamily;
  bool isComplete();
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
  RenderObject loadGLTF(std::string path);
  void init();
  void loop();
  void destroy();
  bool shouldExit();
  void draw(RenderObject* renderObject);
  void endFrame();

private:
  void initWindow();
  static void framebufferResizeCallback(GLFWwindow *window, int width,
                                        int height);
  const u_int32_t WIDTH = 800;
  const u_int32_t HEIGHT = 600;
  GLFWwindow *window;
  const std::vector<const char *> validationLayers = {
      "VK_LAYER_KHRONOS_validation"};
#ifdef NDEBUG
  const bool enableValidationLayers = false;
#else
  const bool enableValidationLayers = true;
#endif
  bool checkValidationLayerSupport();
  void initVulkan();
  std::vector<const char *> getRequiredExtensions();
  VkDebugUtilsMessengerEXT debugMessenger;
  static VKAPI_ATTR VkBool32 VKAPI_CALL
  debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                VkDebugUtilsMessageTypeFlagsEXT messageType,
                const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
                void *pUserData);
  std::vector<const char *> extensions;
  void createInstance();
  void setupDebugMessenger();
  VkSurfaceKHR surface;
  void createSurface();
  void pickPhysicalDevice();
  const std::vector<const char *> deviceExtensions = {
      VK_KHR_SWAPCHAIN_EXTENSION_NAME,
      VK_KHR_SHADER_NON_SEMANTIC_INFO_EXTENSION_NAME};
  bool isDeviceSuitable(VkPhysicalDevice device);
  VkSurfaceFormatKHR chooseSwapSurfaceFormat(
      const std::vector<VkSurfaceFormatKHR> &availableFormats);
  VkPresentModeKHR chooseSwapPresentMode(
      const std::vector<VkPresentModeKHR> &availablePresentModes);
  VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities);
  bool checkDeviceExtensionSupport(VkPhysicalDevice device);
  QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device);
  SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);
  VkSwapchainKHR swapchain;
  std::vector<VkImage> swapChainImages;
  VkFormat swapChainImageFormat;
  VkExtent2D swapChainExtent;
  void createSwapChain();
  void createImageViews();
  VkShaderModule createShaderModule(const std::vector<char> &code);
  void createRenderPass();
  VkDescriptorPool m_firstDescriptorPool;
  void createDescriptorPool();
  VkDescriptorSetLayout m_uboDescriptorSetLayout;
  VkDescriptorSetLayout m_textureDescriptorSetLayout;
  void createDescriptorSetLayouts();
  std::vector<VkDescriptorSet> m_uboDescriptorSets;
  void allocateUboDescriptorSets();
  std::vector<VkDescriptorSet>
  allocateTextureDescriptorSet(VkImageView imageview, VkSampler sampler);
  VkPipeline pipeline;
  VkPipeline loadedPipeline;
  VkPipelineLayout pipelineLayout;
  std::vector<VkImageView> swapChainImageViews;
  VkRenderPass renderPass;
  VkPipeline
  createGraphicPipeline(std::string vertexShaderPath,
                        std::string fragmentShaderPath,
                        const std::vector<VkVertexInputBindingDescription>
                            &vertexInpitBindingDescription,
                        const std::vector<VkVertexInputAttributeDescription>
                            &vertexInputAttributeDescription);
  std::vector<VkFramebuffer> swapChainFramebuffers;
  void createFramebuffers();
  VkCommandPool commandPool;
  void createCommandPool();
  VkImage depthImage;
  VmaAllocation depthAllocation;
  VkImageView depthImageView;
  void createDepthResources();
  VkFormat findSupportedFormat(const std::vector<VkFormat> &candidates,
                               VkImageTiling tiling,
                               VkFormatFeatureFlags features);
  VkFormat findDepthFormat();
  bool hasStencilComponent(VkFormat format);
  VkImage textureImage;
  VmaAllocation textureAllocation;
  void createTextureImage();
  VkImageView textureImageView;
  void createTextureImageView();
  VkImageView createImageView(VkImage image, VkFormat format,
                              VkImageAspectFlags aspectFlags);
  void createImage(uint32_t width, uint32_t height, VkFormat format,
                   VkImageTiling tiling, VkImageUsageFlags usage,
                   VmaMemoryUsage memoryUsage,
                   VmaAllocationCreateFlags allocationFlags, VkImage &image,
                   VmaAllocation &allocation);
  VkBuffer vertexBuffer;
  VmaAllocation vertexAllocation;
  VkCommandBuffer beginSingleTimeCommands();
  void endSingleTimeCommands(VkCommandBuffer commandBuffer);
  void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
  void transitionImageLayout(VkImage image, VkFormat format,
                             VkImageLayout oldLayout, VkImageLayout newLayout);
  void copyBufferImage(VkBuffer buffer, VkImage image, uint32_t width,
                       uint32_t height);
  VkSampler textureSampler;
  void createTextureSampler();
  void createVertexBuffer();
  VkBuffer indexBuffer;
  VmaAllocation indexAllocation;
  void createIndexBuffer();
  std::vector<VkBuffer> m_uboBuffer;
  std::vector<VmaAllocation> m_uboAllocation;
  std::vector<void *> m_uniformBufferMapped;
  void createUniformBuffers();
  void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                    VmaMemoryUsage memoryUsage,
                    VmaAllocationCreateFlags allocationFlags, VkBuffer &buffer,
                    VmaAllocation &allocation);
  uint32_t findMemoryType(uint32_t typeFilter,
                          VkMemoryPropertyFlags properties);
  std::vector<VkCommandBuffer> commandBuffers;
  void createCommandBuffer();
  std::vector<VkSemaphore> imageAvailableSemaphores;
  std::vector<VkSemaphore> renderFinishedSemaphores;
  std::vector<VkFence> inFlightFences;
  void createSyncObjects();
  uint32_t m_currentFrame = 0;
  bool framebufferResized = false;
  void updateUniformBuffer(uint32_t currentImage);
  void drawFrame();
  void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageindex);
  void recordSceneCommandBuffer(const VkCommandBuffer &commandBuffer,
                                const uint32_t imageIndex);
  VkInstance instance;
  VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
  VkDevice device;
  VkQueue queue;
  VkQueue presentQueue;
  void createLogicalDevice();
  VmaAllocator vmaAllocator;
  void createVMA();
  const size_t MAX_FRAMES_IN_FLIGHT = 2;
  void cleanupSwapChain();
  void recreateSwapChain();
  void cleanup();
  std::vector<BufferData> m_loadedBufferData;
  std::vector<Texture> m_loadedTextures;
  std::vector<glm::mat4> m_modelMatrices;
  uint32_t addModelMatrix(const glm::mat4 matrix);
  void setMatrix(const uint32_t index, const glm::mat4 matrix);
  void resizeDescriptorSets();
  std::vector<Node *> m_pNodeToDraw;
};
class RenderObject {
public:
  friend class Renderer;
  RenderObject() = delete;
  RenderObject(const std::vector<Node> &nodes);
  void setMatrix(const glm::mat4 &matrix, const uint32_t index);
  RenderObject(const RenderObject &other) = default;


private:
  std::vector<Node> m_nodes;
};
