project('vulkan-project','c','cpp')

cmake = import('cmake')

glfw_subproject = cmake.subproject('glfw')
glm_subproject = cmake.subproject('glm')


glfw_dependency = glfw_subproject.dependency('glfw')
glm_dependency = glm_subproject.dependency('glm')
vulkan_dependency = dependency('vulkan')

subdir('shaders')
subdir('textures')


executable('vulkanProgram',['main.cpp','Renderer.h','Renderer.cpp','./include_stb/stb_image.h',vertex_shader_custom_target,fragment_shader_custom_target],
    dependencies:[glfw_dependency,glm_dependency,vulkan_dependency],
    override_options : ['cpp_std=c++20'])
