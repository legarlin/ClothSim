#include <iostream>
#include "openGL_headers.h"
#include "math_headers.h"
#include "cloth_sim.h"
#include "scene.h"
#include "stb_image_write.h"

int window_width = 1024;
int window_height = 768;

//----------State Control----------//
bool pause = false;
bool record = false;
bool flip_draw_mode = false;

//----------Time----------//
double now, lastTime;
float delta_t = 0.0f;
int frame_num = 0;

//----------Mouse Control----------//
int mouse_old_x, mouse_old_y;
unsigned char button_mask = 0x00;

//----------OpenGL Render Control----------//
GLuint m_uniform_location[2];
GLuint m_vert_handle, m_frag_handle, m_shaderprog_handle;

//----------Camera Control----------//
float eye_distance = 20.0f;
float head = 45.0f, pitch = 45.0f;
glm::vec3 cam_pos, up(0.0f, 1.0f, 0.0f), lookat(0.0f, 4.0f, 0.0f);

//----------functions----------//
// declare
void initShader(const char* vert_path, const char* frag_path);
void cleanupShader();
void mouseClick(int button, int action);
void mouseMotion(int x, int y);
void keypress(int key, int action);
void aimCamera(void);
void drawAxes(void);
void grabScreen(void);

void activate_shaderprog(GLuint shaderprog);
void deactivate_shaderprog(GLuint shaderprog);

// courtesy of Swiftless
char* textFileRead(const char* fileName);
void printLinkInfoLog(int prog);
void printShaderInfoLog(int shader);

// define

void initShader(const char* vert_path, const char* frag_path)
{
    // create shaders and shader program
    m_vert_handle = glCreateShader(GL_VERTEX_SHADER);
    m_frag_handle = glCreateShader(GL_FRAGMENT_SHADER);
    m_shaderprog_handle = glCreateProgram();

    // load shader source from file
    const char* vert_source = textFileRead(vert_path);
    const char* frag_source = textFileRead(frag_path);
    glShaderSource(m_vert_handle, 1, &vert_source, NULL);
    glShaderSource(m_frag_handle, 1, &frag_source, NULL);
    glCompileShader(m_vert_handle);
    glCompileShader(m_frag_handle);

    // compile shader source
    GLint compiled;
    glGetShaderiv(m_vert_handle, GL_COMPILE_STATUS, &compiled);
    if(!compiled)
        printShaderInfoLog(m_vert_handle);
    glGetShaderiv(m_frag_handle, GL_COMPILE_STATUS, &compiled);
    if(!compiled)
        printShaderInfoLog(m_frag_handle);

    // TODO: customize this part if you want to modify the glsl shader.
    // bind attribute locations for the shaders
    // 0 for position, 1 for color, 2 for normal.
    glBindAttribLocation(m_shaderprog_handle, 0, "v_position");
    glBindAttribLocation(m_shaderprog_handle, 1, "v_color");
    glBindAttribLocation(m_shaderprog_handle, 2, "v_normal");

    // attach shader to the shader program
    glAttachShader(m_shaderprog_handle, m_vert_handle);
    glAttachShader(m_shaderprog_handle, m_frag_handle);
    glLinkProgram(m_shaderprog_handle);
    GLint linked;
    glGetProgramiv(m_shaderprog_handle, GL_LINK_STATUS, &linked);
    if(!linked)
        printLinkInfoLog(m_shaderprog_handle);

    // TODO: customize this part if you want to modify the glsl shader.
    // query uniform locations from openGL.
    m_uniform_location[0] = glGetUniformLocation(m_shaderprog_handle, "u_modelviewMatrix");
    m_uniform_location[1] = glGetUniformLocation(m_shaderprog_handle, "u_projMatrix");

    // activate the shader program.
    glUseProgram(m_shaderprog_handle);
}

void cleanupShader()
{
    glDetachShader(m_shaderprog_handle, m_vert_handle);
    glDetachShader(m_shaderprog_handle, m_frag_handle);
    glDeleteShader(m_vert_handle);
    glDeleteShader(m_frag_handle);
    glDeleteProgram(m_shaderprog_handle);
}

void mouseClick(int button, int action)
{// left: 0. right: 1. middle: 2.
    if(action == GLFW_PRESS)
    {
        button_mask |= 0x01 << button;
        //if (button == GLFW_MOUSE_BUTTON_LEFT)
        //else if(button == GLFW_MOUSE_BUTTON_RIGHT)
    }
    else if(action == GLFW_RELEASE)
    {
        unsigned char mask_not = ~button_mask;
        mask_not |= 0x01 << button;
        button_mask = ~mask_not;
    }
}

void mouseMotion(int x, int y)
{
    float dx, dy;
    dx = (float)(x - mouse_old_x);
    dy = (float)(y - mouse_old_y);

    if (button_mask & 0x01) 
    {// left button
        head += dy * 0.2f;
        pitch += dx * 0.2f;
    } 
    else if (button_mask & 0x02) 
    {// right button
        eye_distance -= dy * 0.01f;
    }
    else if (button_mask & 0x04)
    {// middle button
        glm::vec3 vdir(lookat - cam_pos);
        glm::vec3 u(glm::normalize(glm::cross(vdir, up)));
        glm::vec3 v(glm::normalize(glm::cross(u, vdir)));

        lookat += 0.01f * (dy * v - dx * u);
    }

    mouse_old_x = x;
    mouse_old_y = y;
}

void keypress(int key, int action)
{
    if(glfwGetKey(key) == GLFW_PRESS)
    {
        switch(key) 
        {
        case GLFW_KEY_SPACE:
            pause = !pause;
            break;
        case 'f':
        case 'F':
            flip_draw_mode = !flip_draw_mode;
            break;
        case 'r':
        case 'R':
            record = !record;
            break;
        }
    }
}

void drawAxes(void)
{
    glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
    glDisable(GL_LIGHTING);
    //draw axis.
    GLfloat modelview[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, &viewport[0]);
    GLint width = viewport[2] / 16;
    GLint height = viewport[3] / 16;
    glViewport(0, 0, width, height);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    //get the camera position and up vector from the modelview matrix.
    double campos[3] = {0.0 + 2.0f * modelview[2], 0.0 + 2.0f * modelview[6], 0.0 + 2.0f * modelview[10]};
    double up[3] = {modelview[1], modelview[5], modelview[9]};
    //set up the view matrix.
    gluLookAt(campos[0], campos[1], campos[2], 
        0.0, 0.0, 0.0,
        up[0], up[1], up[2]);

    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();

    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glPopMatrix();
    glPopAttrib();
}

void aimCamera(void)
{
    float r_head = glm::radians(head), r_pitch = glm::radians(pitch);
    cam_pos.x = lookat.x + eye_distance * glm::cos(r_head) * glm::cos(r_pitch);
    cam_pos.y = lookat.y + eye_distance * glm::sin(r_head);
    cam_pos.z = lookat.z + eye_distance * glm::cos(r_head) * glm::sin(r_pitch);

    glMatrixMode(GL_MODELVIEW);
    up = glm::vec3(0.0f, (glm::cos(r_head) > 0.0f) ? 1.0f : -1.0f, 0.0f);
    glm::mat4 modelview = glm::lookAt(cam_pos, lookat, up);
    glLoadMatrixf(&modelview[0][0]);
    
    glMatrixMode(GL_PROJECTION);
    glm::mat4 projection = glm::perspective(60.0f, static_cast<float>(window_width) / static_cast<float>(window_height), 0.1f, 100.0f);
    glLoadMatrixf(&projection[0][0]);

    activate_shaderprog(m_shaderprog_handle);
    glUniformMatrix4fv(m_uniform_location[0], 1, false, &modelview[0][0]);
    glUniformMatrix4fv(m_uniform_location[1], 1, false, &projection[0][0]);
}

void grabScreen(void)
{
    unsigned char* bitmapData = new unsigned char[3 * window_width * window_height];

    for (int i=0; i < window_height; i++) 
    {
        glReadPixels(0, i, window_width, 1, GL_RGB, GL_UNSIGNED_BYTE, 
            bitmapData + (window_width * 3 * ((window_height - 1) - i)));
    }

    char anim_filename[2048];
    sprintf_s(anim_filename, 2048, "output/PBD_no_self_intersect_%04d.png", frame_num);

    stbi_write_png(anim_filename, window_width, window_height, 3, bitmapData, window_width * 3);

    delete [] bitmapData;
}

void activate_shaderprog(GLuint shaderprog)
{
    GLint current_prog;
    glGetIntegerv(GL_CURRENT_PROGRAM, &current_prog);
    if(current_prog != (GLint)shaderprog)
        glUseProgram(shaderprog);
}

void deactivate_shaderprog(GLuint shaderprog)
{
    GLint current_prog;
    glGetIntegerv(GL_CURRENT_PROGRAM, &current_prog);
    if(current_prog == (GLint)shaderprog)
        glUseProgram(0);
}

int main(int argc, char** argv)
{
    bool run = GL_TRUE;

    if(!glfwInit())
    {
        exit(EXIT_FAILURE);
    }

    if(!glfwOpenWindow(window_width, window_height, 8, 8, 8, 8, 24, 0, GLFW_WINDOW))
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glewInit();
    if (!glewIsSupported( "GL_VERSION_2_0 " 
        "GL_ARB_pixel_buffer_object"
        )) {
            fprintf( stderr, "ERROR: Support for necessary OpenGL extensions missing.");
            fflush( stderr);
            return false;
    }

    initShader("./Shader/vert.glsl", "./Shader/frag.glsl");

    glfwSetKeyCallback(keypress);
    glfwSetMouseButtonCallback(mouseClick);
    glfwSetMousePosCallback(mouseMotion);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glViewport(0, 0, window_width, window_height);

    VBO vbo_handle;
    Scene scene("../Scene/test_scene.xml");
    // TODO: change here if you want to use a smaller iteration number.
    ClothSim cloth_sim(10);
    // TODO: change here if you want to modify the dimension.
    cloth_sim.initialize(20, 20, glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(5.0f, 10.0f, 5.0f));

    lastTime = glfwGetTime();
    while(run)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        aimCamera();

        if(!pause)
            cloth_sim.update(&scene, 0.006325f);
        if(flip_draw_mode)
        {
            cloth_sim.flip_draw_mode();
            flip_draw_mode = false;
        }
        activate_shaderprog(m_shaderprog_handle);
        cloth_sim.draw(vbo_handle);
        scene.draw(vbo_handle);
        deactivate_shaderprog(m_shaderprog_handle);

        drawAxes();
        if(!pause && record)
            grabScreen();
        frame_num++;

        now = glfwGetTime();
        char fpsInfo[256];
        sprintf(fpsInfo, "%f", 1.0f / (now - lastTime));
        lastTime = now;
        glfwSetWindowTitle(fpsInfo);

        glfwSwapBuffers();

        run = !glfwGetKey(GLFW_KEY_ESC) && glfwGetWindowParam(GLFW_OPENED);
    }
    
    cleanupShader();
    glfwTerminate();
    exit(EXIT_SUCCESS);
}

// helper function to read shader source and put it in a char array
// thanks to Swiftless
char* textFileRead(const char* fileName) 
{
    char* text;

    if (fileName != NULL) {
        FILE *file = fopen(fileName, "rt");

        if (file != NULL) {
            fseek(file, 0, SEEK_END);
            int count = ftell(file);
            rewind(file);

            if (count > 0) {
                text = (char*)malloc(sizeof(char) * (count + 1));
                count = fread(text, sizeof(char), count, file);
                text[count] = '\0';	//cap off the string with a terminal symbol, fixed by Cory
            }
            fclose(file);
        }
    }
    return text;
}

void printLinkInfoLog(int prog) 
{
    int infoLogLen = 0;
    int charsWritten = 0;
    GLchar *infoLog;

    glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &infoLogLen);

    // should additionally check for OpenGL errors here

    if (infoLogLen > 0)
    {
        infoLog = new GLchar[infoLogLen];
        // error check for fail to allocate memory omitted
        glGetProgramInfoLog(prog,infoLogLen, &charsWritten, infoLog);
        std::cout << "InfoLog:" << std::endl << infoLog << std::endl;
        delete [] infoLog;
    }
}

void printShaderInfoLog(int shader)
{
    int infoLogLen = 0;
    int charsWritten = 0;
    GLchar *infoLog;

    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLen);

    // should additionally check for OpenGL errors here

    if (infoLogLen > 0)
    {
        infoLog = new GLchar[infoLogLen];
        // error check for fail to allocate memory omitted
        glGetShaderInfoLog(shader,infoLogLen, &charsWritten, infoLog);
        std::cout << "InfoLog:" << std::endl << infoLog << std::endl;
        delete [] infoLog;
    }

    // should additionally check for OpenGL errors here
}