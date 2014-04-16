#ifndef OPENGL_HEADERS_INCLUDED
#define OPENGL_HEADERS_INCLUDED
#include "GL/glew.h"
#include "GL/glfw.h"

struct VBO
{
    VBO()
    {
        if(!glIsBuffer(m_vbo))
            glGenBuffers(1, &m_vbo);
        if(!glIsBuffer(m_cbo))
            glGenBuffers(1, &m_cbo);
        if(!glIsBuffer(m_nbo))
            glGenBuffers(1, &m_nbo);
        if(!glIsBuffer(m_ibo))
            glGenBuffers(1, &m_ibo);
    }

    virtual ~VBO()
    {
        if(glIsBuffer(m_vbo))
            glDeleteBuffers(1, &m_vbo);
        if(glIsBuffer(m_cbo))
            glDeleteBuffers(1, &m_cbo);
        if(glIsBuffer(m_nbo))
            glDeleteBuffers(1, &m_nbo);
        if(glIsBuffer(m_ibo))
            glDeleteBuffers(1, &m_ibo);
    }
    // vertex, color, normal, index
    GLuint m_vbo, m_cbo, m_nbo, m_ibo;
};
#endif