#version 150

uniform mat4 u_modelviewMatrix;
uniform mat4 u_projMatrix;

in vec3 v_position;
in vec3 v_color;
in vec3 v_normal;


out vec4 f_color;
out vec4 f_normal;
out vec4 f_light;

void main()
{
    f_color = vec4(v_color, 1.0);
    f_normal = transpose(inverse(u_modelviewMatrix)) * vec4(v_normal, 0.0);
    f_light = vec4(0.0, 0.0, 1.0, 0.0);

    gl_Position = u_projMatrix * u_modelviewMatrix * vec4(v_position, 1.0);
}
