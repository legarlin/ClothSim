// Cloth Simulation using Position Based Dynamics
// Copyright 2013 Xing Du

#ifndef PARTICLE_LIST_H_INCLUDED
#define PARTICLE_LIST_H_INCLUDED

#include "math_headers.h"
#include <vector>
#include <cassert>

// Structure of Array
struct ParticleList
{
    ParticleList() {};
    virtual ~ParticleList()
    {
        m_pos.clear();
        m_predicted_pos.clear();
        m_vel.clear();
        m_lock_pos.clear();
        m_inv_mass.clear();
        m_size = 0;
    }

    void resize(unsigned int size)
    {
        m_pos.clear();
        m_predicted_pos.clear();
        m_vel.clear();
        m_lock_pos.clear();
        m_inv_mass.clear();

        m_pos.resize(size);
        m_predicted_pos.resize(size);
        m_vel.resize(size);
        m_lock_pos.resize(size, false);
        m_inv_mass.resize(size, 1.0f);

        m_size = size;
    }
    // these two functions are used for attach a vertex to a certain point.
    void lock_pos(unsigned int n)
    {
        assert(n < m_size);
        m_lock_pos[n] = true;
    }
    void unlock_pos(unsigned int n)
    {
        assert(n < m_size);
        m_lock_pos[n] = false;
    }
    void unlock_pos_all()
    {
        for(std::vector<bool>::iterator i = m_lock_pos.begin(); i != m_lock_pos.end(); ++i)
        {
            *i = false;
        }
    }
    // inverse mass accessor.
    float inv_mass(unsigned int n) const
    {
        assert(n < m_size);
        if(m_lock_pos[n])
            return 0.0f;
        else
            return m_inv_mass[n];
    }
    void set_inv_mass(unsigned int n, float inv_mass)
    {
        assert(n < m_size);
        m_inv_mass[n] = inv_mass;
    }
    // position accessor.
    const glm::vec3& pos(unsigned int n) const
    {
        assert(n < m_size);
        return m_pos[n];
    }
    glm::vec3& pos(unsigned int n)
    {
        assert(n < m_size);
        return m_pos[n];
    }
    // position accessor.
    const glm::vec3& predicted_pos(unsigned int n) const
    {
        assert(n < m_size);
        return m_predicted_pos[n];
    }
    glm::vec3& predicted_pos(unsigned int n)
    {
        assert(n < m_size);
        return m_predicted_pos[n];
    }
    // velocity accessor.
    const glm::vec3& vel(unsigned int n) const
    {
        assert(n < m_size);
        return m_vel[n];
    }
    glm::vec3& vel(unsigned int n)
    {
        assert(n < m_size);
        return m_vel[n];
    }
    // size accessor
    unsigned int size() const
    {
        return m_size;
    }

protected:
    // number of vertices.
    unsigned int m_size;
    // position of all vertices.
    std::vector<glm::vec3> m_pos;
    // predicted position.
    std::vector<glm::vec3> m_predicted_pos;
    // velocity of a vertex
    std::vector<glm::vec3> m_vel;
    // used for fixing vertex to a certain point
    std::vector<bool> m_lock_pos;
    // weight for resolving the constraints.
    std::vector<float> m_inv_mass;
};
#endif