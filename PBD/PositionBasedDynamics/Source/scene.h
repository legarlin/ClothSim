// Cloth Simulation using Position Based Dynamics
// Courtesy of Aline Normoyle
// Copyright 2013 Xing Du

#ifndef SCENE_H_INCLUDED
#define SCENE_H_INCLUDED

#include "openGL_headers.h"
#include "math_headers.h"
#include "glm.hpp"
#include <tinyxml.h>
#include <vector>

class Scene;
class XMLSceneVisitor;

class Scene
{
public:
    Scene(const char* file_name);
    virtual ~Scene();

    void load_from_file(const char* file_name);
    virtual void draw(const VBO& vbo);
public:
    // TODO: add more primitives here.
    enum PrimitiveType {PLANE, SPHERE};
    class Primitive
    {
    public:
        Primitive(const PrimitiveType& t) : m_type(t) {init_visualization();};
        Primitive(const Primitive& other) : 
            m_type(other.m_type), 
            m_positions(other.m_positions), m_colors(other.m_colors), m_normals(other.m_normals),
            m_indices(other.m_indices)
        {
            ;
        }
        virtual ~Primitive()
        {
            m_positions.clear();
            m_colors.clear();
            m_normals.clear();
            m_indices.clear();
        }

        PrimitiveType type() const
        {
            return m_type;
        }

        virtual void draw(const VBO& vbos) const
        {
            ;
        }

        virtual bool line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
        {
            return false;
        }
    protected:
        virtual void init_visualization() {};
    protected:
        PrimitiveType m_type;
        std::vector<glm::vec3> m_positions, m_colors, m_normals;
        std::vector<unsigned short> m_indices;
    };

    class Plane : public Primitive
    {
    public:
        Plane() : Primitive(PLANE), m_normal(glm::vec3(0.0f, 1.0f, 0.0f)), m_value(0.0f) {init_visualization();};
        Plane(const glm::vec3 norm, float value) : Primitive(PLANE), m_normal(norm), m_value(value) {init_visualization();};
        Plane(const Plane& other) : Primitive(other), m_normal(other.m_normal), m_value(other.m_value) {};
        virtual ~Plane() {};

        virtual void draw(const VBO& vbos) const;
        virtual bool line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const;
        
    protected:
        virtual void init_visualization();
    protected:
        glm::vec3 m_normal;
        float m_value;
    };

    class Sphere : public Primitive
    {
    public:
        Sphere() : Primitive(SPHERE), m_center(glm::vec3(0.0f)), m_radius(0.0f) {init_visualization();};
        Sphere(const glm::vec3 pos, float radius) : Primitive(SPHERE), m_center(pos), m_radius(radius) {init_visualization();};
        Sphere(const Sphere& other) : Primitive(other), m_center(other.m_center), m_radius(other.m_radius) {};
        virtual ~Sphere() {};

        virtual void draw(const VBO& vbos) const;
        virtual bool line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const;
    protected:
        virtual void init_visualization();
    protected:
        glm::vec3 m_center;
        float m_radius;
    };

public:
    void insert_primitive(Primitive* const new_primitive);
    bool line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const;
protected:
    std::vector<Primitive*> m_primitives;
};

class XMLSceneVisitor : public TiXmlVisitor
{
public:
    XMLSceneVisitor(Scene* const scene);
    XMLSceneVisitor(const XMLSceneVisitor& other);
    virtual ~XMLSceneVisitor();

    virtual bool VisitEnter(const TiXmlElement& element, const TiXmlAttribute* attribute);
    virtual bool VisitExit( const TiXmlElement& element);

protected:
    Scene* const m_scene;
    Scene::Primitive* m_current;
};



#endif