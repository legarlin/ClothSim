// Cloth Simulation using Position Based Dynamics
// Copyright 2013 Xing Du

#ifndef CLOTH_SIM_H_INCLUDED
#define CLOTH_SIM_H_INCLUDED


#include "openGL_headers.h"
#include "math_headers.h"
#include "particlelist.h"
#include "constraint.h"
#include "scene.h"
#include <vector>

class ClothSim
{
public:
    ClothSim();
    ClothSim(unsigned int n);
    virtual ~ClothSim();


    void initialize(unsigned int dim_x, unsigned int dim_z, const glm::vec3& cloth_min, const glm::vec3& cloth_max);
    void update(const Scene* const scene, float dt);
    void draw(const VBO& vbos);
    void flip_draw_mode()
    {
        m_draw_wire = !m_draw_wire;
    }
protected:
    struct Edge
    {
        unsigned int m_v1, m_v2;
        unsigned int m_tri1, m_tri2;
    };
protected:
    unsigned int m_dimx, m_dimz;
    float m_thick;
    unsigned int m_solver_iterations;
    // vertices and estimated position.
    ParticleList m_vertices;
    // internal and external constraints.
    std::vector<Constraint*> m_constraints_int;
    std::vector<CollisionConstraint> m_constraints_ext;
    std::vector<SelfCollisionConstraint> m_self_collision;
    //std::vector<Constraint*> m_constraints_ext;
    // for visualize the cloth.
    bool m_draw_wire;
    std::vector<glm::vec3> m_normals;
    std::vector<glm::vec3> m_colors;
    std::vector<unsigned int> m_triangle_list;
    // for generating constraints.
    std::vector<Edge> m_edge_list;
private:
    // generate edge list from the geometry representation.
    void generate_edge_list();
    // generate all the internal constraints based on the edge list. 
    void generate_internal_constraints();
    // update the normal per frame for visualization.
    void compute_normal();
    // apply external force to the system.
    void apply_external_force(const glm::vec3& force, float dt);
    // damp velocity for all vertices.
    void damp_velocity(float k_damp);
    // compute predicted position based on velocity.
    void compute_predicted_position(float dt);
    // collision detection, generating external constraints. need to generate constraints per frame.
    void collision_detection(const Scene* const scene);
    // self collision.
    void self_collision_detection();
    // resolve all the constraints, both internal and external.
    void resolve_constriants();
    // update the position and velocity.
    void integration(float dt);
    void update_velocity(float friction, float restitution);
    void clean_collision_constraints();
};

#endif