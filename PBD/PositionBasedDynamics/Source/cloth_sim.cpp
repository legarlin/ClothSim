// Cloth Simulation using Position Based Dynamics
// Copyright 2013 Xing Du

#include "cloth_sim.h"

ClothSim::ClothSim() : 
    m_dimx(0), m_dimz(0),
    m_thick(0.05f),
    m_solver_iterations(10),
    m_draw_wire(false)
{
    ;
}

ClothSim::ClothSim(unsigned int n) : 
    m_dimx(0), m_dimz(0),
    m_thick(0.05f),
    m_solver_iterations(n),
    m_draw_wire(false)
{
    ;
}

ClothSim::~ClothSim()
{
    m_normals.clear();
    m_colors.clear();
    m_triangle_list.clear();

    m_edge_list.clear();

    unsigned int i, size;
    size = m_constraints_int.size();
    for(i = 0; i < size; ++i)
        delete m_constraints_int[i];
    m_constraints_int.clear();

    m_constraints_ext.clear();
    m_self_collision.clear();
}

void ClothSim::initialize(unsigned int dim_x, unsigned int dim_z, const glm::vec3& cloth_min, const glm::vec3& cloth_max)
{// initialize the cloth here. feel free to create your own initialization.
    m_dimx = dim_x;
    m_dimz = dim_z;
    glm::vec3 delta;
    delta.x = (cloth_max.x - cloth_min.x) / (float)(m_dimx - 1);
    delta.y = (cloth_max.y - cloth_min.y) / (float)(m_dimz - 1);
    delta.z = (cloth_max.z - cloth_min.z) / (float)(m_dimz - 1);
    // if you want, you can substitute this part using a obj file loader.
    // We'll be dealing with the most simple case, so things are done manually here.
    m_vertices.resize(m_dimx * m_dimz);
    m_normals.resize(m_dimx * m_dimz);
    m_colors.resize(m_dimx * m_dimz);
    // Assign initial position, velocity and mass to all the vertices.
    unsigned int i, k, index;
    for(i = 0; i < m_dimx; ++i)
    {
        for(k = 0; k < m_dimz; ++k)
        {
            index = m_dimz * i + k;
            m_vertices.pos(index) = glm::vec3(delta.x * i + cloth_min.x, delta.y * k + cloth_min.y, delta.z * k + cloth_min.z);
            m_vertices.vel(index) = glm::vec3(0.0f);
            m_vertices.set_inv_mass(index, 1.0f);
        }
    }
    // TODO: change color if you want to.
    glm::vec3 cloth_color(0.25f, 0.65f, 0.85f);
    for(i = 0; i < m_dimx; ++i)
    {
        for(k = 0; k < m_dimz; ++k)
        {
            index = m_dimz * i + k;
            m_colors[index] = cloth_color;
        }
    }
    // Generate the triangle list.
    // (m_dimx - 1) * (m_dimz - 1) * 2 triangles for a m_dimx * m_dimz square, and 3 components for each triangle.
    m_triangle_list.resize((m_dimx - 1) * (m_dimz - 1) * 2 * 3);
    printf("Triangle number: %u\n", m_triangle_list.size() / 3);
    // loop over all the small squares.
    bool row_flip = false, column_flip = false;
    for(i = 0; i < m_dimx - 1; ++i)
    {
        for(k = 0; k < m_dimz - 1; ++k)
        {
            index = (m_dimz - 1) * i + k;

            // first triangle
            m_triangle_list[6 * index + 0] = m_dimz * i + k;
            m_triangle_list[6 * index + 1] = m_dimz * i + k + 1;
            m_triangle_list[6 * index + 2] = m_dimz * (i + 1) + ((row_flip ^ column_flip) ? (k + 1) : k);
            // second triangle
            m_triangle_list[6 * index + 3] = m_dimz * (i + 1) + k + 1;
            m_triangle_list[6 * index + 4] = m_dimz * (i + 1) + k;
            m_triangle_list[6 * index + 5] = m_dimz * i + ((row_flip ^ column_flip) ? k : (k + 1));

            row_flip = !row_flip;
        }
        column_flip = !column_flip;
        row_flip = false;
    }
    // generate edge from the geometry
    generate_edge_list();
    // generate internal constraints.
    generate_internal_constraints();
}

void ClothSim::update(const Scene* const scene, float dt)
{// update the system for a certain time step dt.
    glm::vec3 gravity(0.0f, -9.8f, 0.0f);
    apply_external_force(gravity, dt);
    damp_velocity(0.01f);
    compute_predicted_position(dt);
    collision_detection(scene);
    resolve_constriants();
    integration(dt);
    update_velocity(0.98f, 0.4f);
    compute_normal();
    clean_collision_constraints();
    
}

void ClothSim::draw(const VBO& vbos)
{// visualize the cloth on the screen.

    glPolygonMode(GL_FRONT_AND_BACK, (m_draw_wire ? GL_LINE : GL_FILL));

    unsigned int size = m_vertices.size();
    unsigned int element_num = m_triangle_list.size();
    // position
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_vertices.pos(0), GL_DYNAMIC_DRAW);
    // color
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_colors[0], GL_STATIC_DRAW);
    // normal
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_normals[0], GL_DYNAMIC_DRAW);

    // indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, element_num * sizeof(unsigned int), &m_triangle_list[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glDrawElements(GL_TRIANGLES, element_num, GL_UNSIGNED_INT, 0);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void ClothSim::generate_edge_list()
{// generate all the edges from the vertices and triangle list.
    // courtesy of Eric Lengyel, "Building an Edge List for an Arbitrary Mesh". Terathon Software 3D Graphics Library.
    // http://www.terathon.com/code/edges.html
    unsigned int vert_num = m_vertices.size();
    unsigned int tri_num = m_triangle_list.size() / 3;

    unsigned int *first_edge = new unsigned int[vert_num + 3 * tri_num];
    unsigned int *next_edge = first_edge + vert_num;
    
    for(unsigned int i = 0; i < vert_num; ++i)
        first_edge[i] = 0xFFFFFFFF;
    // First pass over all triangles. Finds out all the edges satisfying the condition that
    // the first vertex index is less than the second vertex index when the direction from 
    // the first to the second represents a counterclockwise winding around the triangle to
    // which the edge belongs. For each edge found, the edge index is stored in a linked 
    // list of edges belonging to the lower-numbered vertex index i. This allows us to 
    // quickly find an edge in the second pass whose higher-numbered vertex is i.

    unsigned int edge_count = 0;
    const unsigned int* triangle = &m_triangle_list[0];
    unsigned int i1, i2;
    for(unsigned int t = 0; t < tri_num; ++t)
    {
        i1 = triangle[2];
        for(unsigned int n = 0; n < 3; ++n)
        {
            i2 = triangle[n];
            if(i1 < i2)
            {
                Edge new_edge;
                new_edge.m_v1 = i1;
                new_edge.m_v2 = i2;
                new_edge.m_tri1 = t;
                new_edge.m_tri2 = t;
                m_edge_list.push_back(new_edge);

                unsigned int edge_idx = first_edge[i1];
                if(edge_idx == 0xFFFFFFFF)
                {
                    first_edge[i1] = edge_count;
                }
                else
                {
                    while(true)
                    {
                        unsigned int idx = next_edge[edge_idx];
                        if(idx == 0xFFFFFFFF)
                        {
                            next_edge[edge_idx] = edge_count;
                            break;
                        }
                        edge_idx = idx;
                    }
                }

                next_edge[edge_count] = 0xFFFFFFFF;
                edge_count++;
            }
            i1 = i2;
        }
        triangle += 3;
    }

    // Second pass over all triangles. Finds out all the edges satisfying the condition that
    // the first vertex index is greater than the second vertex index when the direction from 
    // the first to the second represents a counterclockwise winding around the triangle to
    // which the edge belongs. For each of these edges, the same edge should have already been
    // found in the first pass for a different triangle. So we search the list of edges for the
    // higher-numbered index for the matching edge and fill in the second triangle index. The 
    // maximum number of the comparisons in this search for any vertex is the number of edges
    // having that vertex as an endpoint.
    triangle = &m_triangle_list[0];
    for(unsigned int t = 0; t < tri_num; ++t)
    {
        i1 = triangle[2];
        for(unsigned int n = 0; n < 3; ++n)
        {
            i2 = triangle[n];
            if(i1 > i2)
            {
                bool is_new_edge = true;
                for(unsigned int edge_idx = first_edge[i2]; edge_idx != 0xFFFFFFFF; edge_idx = next_edge[edge_idx])
                {
                    Edge *edge = &m_edge_list[edge_idx];
                    if((edge->m_v2 == i1) && (edge->m_tri1 == edge->m_tri2))
                    {
                        edge->m_tri2 = t;
                        is_new_edge = false;
                        break;
                    }
                }
                // for case where a edge belongs to only one triangle. i.e. mesh is not watertight.
                if(is_new_edge)
                {
                    Edge new_edge;
                    new_edge.m_v1 = i1;
                    new_edge.m_v2 = i2;
                    new_edge.m_tri1 = t;
                    new_edge.m_tri2 = t;
                    m_edge_list.push_back(new_edge);

                    unsigned int edge_idx = first_edge[i1];
                    if(edge_idx == 0xFFFFFFFF)
                    {
                        first_edge[i1] = edge_count;
                    }
                    else
                    {
                        while(true)
                        {
                            unsigned int idx = next_edge[edge_idx];
                            if(idx == 0xFFFFFFFF)
                            {
                                next_edge[edge_idx] = edge_count;
                                break;
                            }
                            edge_idx = idx;
                        }
                    }

                    next_edge[edge_count] = 0xFFFFFFFF;
                    edge_count++;
                }
            }
            i1 = i2;
        }
        triangle += 3;
    }

    delete[] first_edge;
    printf("Edge number: %u.\n", m_edge_list.size());
}

void ClothSim::generate_internal_constraints()
{// TODO: generate all internal constraints in this function.
    // generating fixed point constraints.
    unsigned int i, k, index;
    for(i = 0; i < m_dimx; ++i)
    {
        for(k = 0; k < m_dimz; ++k)
        {
            index = m_dimz * i + k;
            // TODO: add FixedPointConstraint to internal constraint.
			if (i == 0) {
				FixedPointConstraint* curr = new FixedPointConstraint(&this->m_vertices, index, this->m_vertices.pos(index));
			}
        }
    }
    // generate stretch constraints. assign a stretch constraint for each edge.
    glm::vec3 p1, p2;
    // TODO: assign an initial value for stretch stiffness.
    float stretch_stiff = 0.02f; // 1.0f;
    float s_stiff = 1.0f - std::pow((1 - stretch_stiff), 1.0f / m_solver_iterations);
    for(std::vector<Edge>::iterator e = m_edge_list.begin(); e != m_edge_list.end(); ++e)
    {// TODO: add stretch constraint here.
		int id1 = e->m_v1; // vertex indices
		int id2 = e->m_v2;
		double len = glm::length(m_vertices.pos(id2) - m_vertices.pos(id1)); // distance from v1 to v2

		StretchConstraint* curr = new StretchConstraint(&this->m_vertices, stretch_stiff, id1, id2, len);
		m_constraints_int.push_back(curr);
    }

    // generate the bending constraints.
    glm::vec3 p3, p4;
    float phi;
    unsigned int id1, id2, id3, id4;
    unsigned int* tri;

    // TODO: assign an initial value for bend stiffness. DONOT USE 1.0f.
    float bend_stiff = 0.02f; // 0.1f;
    float b_stiff = 1.0f - std::pow((1 - bend_stiff), 1.0f / m_solver_iterations);
    for(std::vector<Edge>::iterator e = m_edge_list.begin(); e != m_edge_list.end(); ++e)
    {
        if(e->m_tri1 == e->m_tri2)
            continue;
        id1 = e->m_v1;
        id2 = e->m_v2;
        // here only the shared edge would be useful. 
        // based on how we extract all these "shared" edges, we know that id1 < id2 and id1->id2 is the looping direction for the first triangle.
        tri = &m_triangle_list[3 * e->m_tri1];
        while((*tri == id1)||(*tri == id2))
            tri++;
        id3 = *tri;

        tri = &m_triangle_list[3 * e->m_tri2];
        while((*tri == id1)||(*tri == id2))
            tri++;
        id4 = *tri;

        // TODO: add bend constraint here. hacky bend for basic requirement. real bend constraint for extra credit.

		float len = glm::length(m_vertices.pos(id3) - m_vertices.pos(id4));
		StretchConstraint* curr = new StretchConstraint(&this->m_vertices, bend_stiff, id3, id4, len);
		m_constraints_int.push_back(curr);
    }
}

void ClothSim::compute_normal()
{// compute normal for all the vertex. only necessary for visualization.
    // reset all the normal.
    glm::vec3 zero(0.0f);
    for(std::vector<glm::vec3>::iterator n = m_normals.begin(); n != m_normals.end(); ++n)
    {
        *n = zero;
    }
    // calculate normal for each individual triangle
    unsigned int triangle_num = m_triangle_list.size() / 3;
    unsigned int id0, id1, id2;
    glm::vec3 p0, p1, p2;
    glm::vec3 normal;
    for(unsigned int i = 0; i < triangle_num; ++i)
    {
        id0 = m_triangle_list[3 * i];
        id1 = m_triangle_list[3 * i + 1];
        id2 = m_triangle_list[3 * i + 2];

        p0 = m_vertices.pos(id0);
        p1 = m_vertices.pos(id1);
        p2 = m_vertices.pos(id2);
        
        normal = glm::normalize(glm::cross(p1 - p0, p2 - p1));

        m_normals[id0] += normal;
        m_normals[id1] += normal;
        m_normals[id2] += normal;
    }
    // re-normalize all the normals.
    for(std::vector<glm::vec3>::iterator n = m_normals.begin(); n != m_normals.end(); ++n)
    {
        *n = glm::normalize(*n);
    }
}

void ClothSim::apply_external_force(const glm::vec3& force, float dt)
{// apply external force to cloth.
    unsigned int size = m_vertices.size();
    for(unsigned int i = 0; i < size; ++i)
    {// TODO: apply force to velocity of all vertices.
		glm::vec3 acc = force * this->m_vertices.inv_mass(i); // F = ma
		glm::vec3 vel = acc * dt;
		m_vertices.vel(i) += vel;
    }
}

void ClothSim::damp_velocity(float k_damp)
{
    // TODO: apply damping according to chapter 3.5 in original paper.
    
	glm::vec3 xcm_num(0); // 1, 2
	glm::vec3 vcm_num(0);
	float m_sum = 0.0f;

	unsigned int size = m_vertices.size();
	for (unsigned int i = 0; i < size; i++) {
		float m = 1.0f / m_vertices.inv_mass(i);

		xcm_num += m_vertices.pos(i) * m;
		vcm_num += m_vertices.vel(i) * m;
		
		m_sum += m_vertices.inv_mass(i);
	}

	glm::vec3 xcm = xcm_num / m_sum;
	glm::vec3 vcm = vcm_num / m_sum;

	glm::vec3 L(0); // 3, 4
	glm::mat3 I(0);

	for (unsigned int i = 0; i < size; i++) {
		glm::vec3 pos = m_vertices.pos(i);
		glm::vec3 vel = m_vertices.vel(i);
		float m = 1.0f / m_vertices.inv_mass(i);

		glm::vec3 RI = pos - xcm;
		glm::vec3 b = m * vel;

		L += glm::cross(RI, b);

		glm::mat3 cpiRI(0, RI[2], -RI[1], -RI[2], 0, RI[0], RI[1], -RI[0], 0);
		glm::mat3 cpiRI_t = glm::transpose(cpiRI);
		I += cpiRI_t * cpiRI * m;
	}

	glm::vec3 W = glm::inverse(I) * L; // 5

	for (unsigned int i = 0; i < size; i++) { // 6, 7, 8, 9
		glm::vec3 pos = m_vertices.pos(i);
		glm::vec3 vel = m_vertices.vel(i);

		glm::vec3 RI = pos - xcm;

		glm::vec3 dv = vcm + glm::cross(W, RI) - vel;

		m_vertices.vel(i) += k_damp * dv;
	}
}

void ClothSim::compute_predicted_position(float dt)
{
    unsigned int size = m_vertices.size();
    for(unsigned int i = 0; i < size; ++i)
    {// TODO: compute predicted position for all vertices.
        // this is just an example line and assign a initial value so that the program won't be too slow.
		glm::vec3 pos = this->m_vertices.vel(i) * dt; // compute from velocity
        m_vertices.predicted_pos(i) = m_vertices.pos(i) + pos; // add to current position
    }
}

void ClothSim::collision_detection(const Scene* const scene)
{// detect collision between cloth and the scene, and generate external constraints.
    unsigned int i, size;
    size = m_vertices.size();
    glm::vec3 x, p, q, n;
    for(i = 0; i < size; ++i)
    {
        x = m_vertices.pos(i);
        p = m_vertices.predicted_pos(i);
        if(scene->line_intersection(x, p, m_thick, q, n))
        {
            CollisionConstraint c(&m_vertices, i, q, n);
            m_constraints_ext.push_back(c);
        }
    }
    // TODO: implement self collision if you want to.
    // self_collision_detection();
}

void ClothSim::self_collision_detection()
{// TODO: implement self collision if you want to.
    ;
}

void ClothSim::resolve_constriants()
{// resolve all the constraints, including both internal and external constraints.
    bool all_solved = true;
    bool reverse = false;
    int i, size;
    for(unsigned int n = 0; n < m_solver_iterations; ++n)
    {
        // solve all the internal constraints.
        size = m_constraints_int.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= m_constraints_int[i]->project_constraint();
        }
        // solve all the external constraints.
        size = m_constraints_ext.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= m_constraints_ext[i].project_constraint();
        }
        // solve all the self collisions.
        size = m_self_collision.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= m_self_collision[i].project_constraint();
        }

        if(all_solved)
            break;
        reverse = !reverse;
    }
    m_vertices.unlock_pos_all();
}

void ClothSim::integration(float dt)
{// integration the position based on optimized prediction, and determine velocity based on position.
    unsigned int size = m_vertices.size();
    float inv_dt = 1.0f / dt;
    for(unsigned int i = 0; i < size; ++i)
    {// TODO: compute position and velocity for all vertices based on predicted position.

		glm::vec3 posDiff = m_vertices.predicted_pos(i) - m_vertices.pos(i);
		m_vertices.vel(i) = posDiff * inv_dt;

		m_vertices.pos(i) = m_vertices.predicted_pos(i);
    }
}

void ClothSim::update_velocity(float friction, float restitution)
{// update velocity based on collision restitution or friction.
    glm::vec3 normal, vn, vt;
    float norm_fraction;
    for(std::vector<CollisionConstraint>::iterator s = m_constraints_ext.begin(); s != m_constraints_ext.end(); ++s)
    {// TODO: modify the velocity according to collisions
		int id = s->index();
		glm::vec3 vel = m_vertices.vel(id);

		normal = s->normal();
		float nProj = glm::dot(vel, normal); // projection of velocity onto normal

		vn = nProj * normal;
		vt = vel - vn;

		m_vertices.vel(id) = vn * restitution + vt * friction; // compute new velocity
    }

    for(std::vector<SelfCollisionConstraint>::iterator s = m_self_collision.begin(); s != m_self_collision.end(); ++s)
    {// TODO: add this part if you added self collisions already.
        ;
    }
}

void ClothSim::clean_collision_constraints()
{
    m_constraints_ext.clear();
    m_self_collision.clear();
}