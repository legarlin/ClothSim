#version 150

in vec4 f_color;
in vec4 f_normal;
in vec4 f_light;

out vec4 out_Color;
void main()
{
    vec4 diffuseColor = f_color;//material color
    
    float diffuseTerm = clamp(abs(dot(f_normal, f_light)), 0.0, 1.0);

    out_Color = diffuseTerm * diffuseColor;
}
