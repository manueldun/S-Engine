texture_vertex_shader_file = 'texturedVert.spv'
vertex_textured_shader_custom_target  = custom_target(
    'texturedColorVertexShader',
    input: 'texturedShader.vert',
    output: texture_vertex_shader_file,
    command: ['glslang','-V','-o','@OUTPUT@','-gVS','@INPUT@']
)
add_project_arguments('-DTEXTURED_VERTEX_SHADER_NAME="'+texture_vertex_shader_file+'"', language:'cpp')

texture_fragment_shader_file = 'texturedFrag.spv'
fragment_textured_shader_custom_target =  custom_target(
    'texturedColorFragmentShader',
    input: 'texturedShader.frag',
    output: texture_fragment_shader_file,
    command: ['glslang','-V','-o','@OUTPUT@','-gVS','@INPUT@'])
add_project_arguments('-DTEXTURED_FRAGMENT_SHADER_NAME="'+texture_fragment_shader_file+'"', language:'cpp')

solid_vertex_shader_file = 'solidVert.spv'
vertex_solid_shader_custom_target =  custom_target(
    'solidColorVertexShader',
    input: 'solidColor.vert',
    output: solid_vertex_shader_file,
    command:['glslang','-V','-o','@OUTPUT@','-gVS','@INPUT@'])
add_project_arguments('-DSOLID_VERTEX_SHADER_NAME="'+solid_vertex_shader_file+'"', language:'cpp')

solid_fragment_shader_file = 'solidFrag.spv'
fragment_solid_shader_custom_target =  custom_target(
    'solidColorFragmentShader',
    input: 'solidColor.frag',
    output: solid_fragment_shader_file,
    command: ['glslang','-V','-o','@OUTPUT@','-gVS','@INPUT@'])
add_project_arguments('-DSOLID_FRAGMENT_SHADER_NAME="'+solid_fragment_shader_file+'"', language:'cpp')
