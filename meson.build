project('vulkan-project','c','cpp')

cmake = import('cmake')

glfw_subproject = cmake.subproject('glfw')
glm_subproject = cmake.subproject('glm')
vma_subproject = cmake.subproject('VulkanMemoryAllocator')

glfw_dependency = glfw_subproject.dependency('glfw')
glm_dependency = glm_subproject.dependency('glm')
vma_dependency = vma_subproject.dependency('VulkanMemoryAllocator')
vulkan_dependency = dependency('vulkan')

subdir('shaders')
subdir('textures')


executable('vulkanProgram',['main.cpp', 'Renderer.cpp',
    'Physics.cpp', 'utils.cpp', 'imgui/backends/imgui_impl_vulkan.cpp',
    'imgui/backends/imgui_impl_glfw.cpp', 'imgui/imgui.cpp',
    './imgui/imgui_demo.cpp', './imgui/imgui_draw.cpp',
    './imgui/imgui_tables.cpp', './imgui/imgui_widgets.cpp',
    vertex_textured_shader_custom_target, fragment_textured_shader_custom_target,
    vertex_solid_shader_custom_target,fragment_solid_shader_custom_target],
    dependencies:[glfw_dependency,glm_dependency,vulkan_dependency,
      vma_dependency],
    include_directories: ['imgui','imgui/backends', 'imgui/backends/vulkan'],
    override_options : ['cpp_std=c++20'])
