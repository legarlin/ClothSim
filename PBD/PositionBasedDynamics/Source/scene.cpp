// Cloth Simulation using Position Based Dynamics
// Courtesy of Aline Normoyle
// Copyright 2013 Xing Du

#include "scene.h"
#include <cassert>

//----------Scene Class----------//
Scene::Scene(const char* file_name)
{
    load_from_file(file_name);
}

Scene::~Scene()
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        delete (*iter);
    }
    m_primitives.clear();
}

void Scene::load_from_file(const char* file_name)
{
    TiXmlDocument xml_file(file_name);
    if(xml_file.LoadFile())
    {
        XMLSceneVisitor visitor(this);
        xml_file.Accept(&visitor);
    }
}

void Scene::draw(const VBO& vbos)
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        (*iter)->draw(vbos);
    }
}

void Scene::insert_primitive(Primitive* const new_primitive)
{
    m_primitives.push_back(new_primitive);
}

bool Scene::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{// assume no intersection between primitives. i.e. a line intersects at most one primitive.
    for(std::vector<Primitive*>::const_iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        if((*iter)->line_intersection(p1, p2, threshold, intersect, normal))
            return true;
    }
    return false;
}

//----------Plane Class----------//
void Scene::Plane::init_visualization()
{
    m_positions.clear();
    m_colors.clear();
    m_normals.clear();
    m_indices.clear();

    glm::vec3 center(m_value * m_normal);
    glm::vec3 local_x, local_z;
    local_x = glm::cross(m_normal, glm::vec3(0.0f, 0.0f, 1.0f));
    if(glm::length(local_x) < 0.00001f)
        local_x = glm::cross(m_normal, glm::vec3(1.0f, 0.0f, 0.0f));
    local_x = glm::normalize(local_x);
    local_z = glm::normalize(glm::cross(local_x, m_normal));

    glm::vec3 mat_color(0.6f);
    unsigned int slice = 24;

    glm::vec3 vertex(center);
    m_positions.push_back(center);
    m_normals.push_back(m_normal);
    m_colors.push_back(mat_color);

    float delta = 360.0f / slice;
    float radius = 100.0f;
    glm::vec3 local_pos;
    for(float theta = 0.0f; theta <= 360.0f; theta += delta)
    {
        local_pos.x = radius * cos(glm::radians(theta));
        local_pos.z = radius * sin(glm::radians(theta));

        vertex = local_pos.x * local_x - local_pos.z * local_z + center;

        m_positions.push_back(vertex);
        m_normals.push_back(m_normal);
        m_colors.push_back(mat_color);
    }
    for(unsigned int i = 0; i < slice; ++i)
    {
        m_indices.push_back(0);
        m_indices.push_back(i + 1);
        m_indices.push_back(i + 2);
    }
    m_indices.push_back(0);
    m_indices.push_back(slice);
    m_indices.push_back(1);
}

void Scene::Plane::draw(const VBO& vbos) const
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // position
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_positions.size() * sizeof(float), &m_positions[0], GL_STREAM_DRAW);

    // color
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_colors.size() * sizeof(float), &m_colors[0], GL_STREAM_DRAW);

    // normal
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_normals.size() * sizeof(float), &m_normals[0], GL_STREAM_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned short), &m_indices[0], GL_STATIC_DRAW);

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
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_SHORT, 0);//GL_UNSIGNED_INT

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

bool Scene::Plane::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{
    float v1, v2;
    v1 = glm::dot(p1, m_normal) - m_value;
    v2 = glm::dot(p2, m_normal) - m_value;
    if(v2 < threshold)
    {
        normal = m_normal;
        if(v1 >= threshold)
        {// continuous collision handling.
            intersect = ((v1 - threshold) * p2 - (v2 - threshold) * p1) / (v1 - v2);
        }
        else
        {// static collision handling.
            intersect = p2 - (v2 - threshold) * normal;
        }
        return true;
    }
    else
        return false;
}

//----------Sphere Class----------//
void Scene::Sphere::init_visualization()
{
    m_positions.clear();
    m_colors.clear();
    m_normals.clear();
    m_indices.clear();

    glm::vec3 mat_color(0.6f);
    unsigned int slice = 24, stack = 10;

    glm::vec3 tnormal(0.0f, 1.0f, 0.0f), tpos;
	tpos = m_center + m_radius * tnormal;

    m_positions.push_back(tpos);
    m_normals.push_back(tnormal);
    m_colors.push_back(mat_color);

	float theta_z, theta_y, sin_z;
    float delta_y = 360.0f / slice, delta_z = 180.0f / stack;
	//loop over the sphere
	for(theta_z = delta_z; theta_z < 179.99f; theta_z += delta_z)
	{
		for(theta_y = 0.0f; theta_y < 359.99f; theta_y += delta_y)
		{
			sin_z = sin(glm::radians(theta_z));
			
            tnormal.x = sin_z * cos(glm::radians(theta_y));
			tnormal.y = cos(glm::radians(theta_z));
			tnormal.z = -sin_z * sin(glm::radians(theta_y));

			tpos = m_center + m_radius * tnormal;

            m_positions.push_back(tpos);
            m_normals.push_back(tnormal);
            m_colors.push_back(mat_color);
		}
	}
	tnormal = glm::vec3(0.0f, -1.0f, 0.0f);
    tpos = m_center + m_radius * tnormal;

    m_positions.push_back(tpos);
    m_normals.push_back(tnormal);
    m_colors.push_back(mat_color);

	//indices
	unsigned int j = 0, k = 0;
	for(j = 0; j < slice - 1; ++j)
	{
		m_indices.push_back(0);
		m_indices.push_back(j + 1);
		m_indices.push_back(j + 2);
	}
	m_indices.push_back(0);
	m_indices.push_back(slice);
	m_indices.push_back(1);

	for(j = 0; j < stack - 2; ++j)
	{
		for(k = 1 + slice * j; k < slice * (j + 1); ++k)
		{
			m_indices.push_back(k);
			m_indices.push_back(k + slice);
			m_indices.push_back(k + slice + 1);

			m_indices.push_back(k);
			m_indices.push_back(k + slice + 1);
			m_indices.push_back(k + 1);
		}
		m_indices.push_back(k);
		m_indices.push_back(k + slice);
		m_indices.push_back(k + 1);

		m_indices.push_back(k);
		m_indices.push_back(k + 1);
		m_indices.push_back(k + 1 - slice);
	}

    unsigned int bottom_id = (stack - 1) * slice + 1;
    unsigned int offset = bottom_id - slice;
	for(j = 0; j < slice - 1; ++j)
	{
		m_indices.push_back(j + offset);
		m_indices.push_back(bottom_id);
		m_indices.push_back(j + offset + 1);
	}
	m_indices.push_back(bottom_id - 1);
	m_indices.push_back(bottom_id);
	m_indices.push_back(offset);

	if(m_indices.size() != 6 * (stack - 1) * slice)
		printf("indices number not correct!\n");
}

void Scene::Sphere::draw(const VBO& vbos) const
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);                                                                                               
    // position
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_positions.size() * sizeof(float), &m_positions[0], GL_STREAM_DRAW);

    // color
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_colors.size() * sizeof(float), &m_colors[0], GL_STREAM_DRAW);

    // normal
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_normals.size() * sizeof(float), &m_normals[0], GL_STREAM_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned short), &m_indices[0], GL_STATIC_DRAW);

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
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_SHORT, 0);//GL_UNSIGNED_INT

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

bool Scene::Sphere::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{// TODO: implement line-sphere intersection. you can refer to line-plane intersection.
	float r = m_radius + 0.01f; // nudge out
	glm::vec3 p1c = p1 - m_center;
	glm::vec3 p2c = p2 - m_center;

    float v1, v2;// v1 v2 are distance to sphere for p1 and p2.

    v1 = glm::length(p1c) - r;
    v2 = glm::length(p2c) - r;
    if(v2 < threshold)
    {
		glm::vec3 d = glm::normalize(p2 - p1);
		//glm::vec3 p1c = p1 - m_center;
		float A = glm::dot(d, d);
		float B = 2.0f * glm::dot(d, p1c);
		float C = glm::dot(p1c, p1c) - pow(r, 2);

		float discriminant = pow(B, 2) - 4.0f * A * C;
		float t;

        if(v1 >= threshold)
        {// continuous collision handling.
			if (discriminant < 0) {
				return false;
			}
			else if (discriminant == 0) {
				t = -B/(2*A);
			}
			else {
				float t1 = (-B - sqrt(discriminant)) / (2*A);
				float t2 = (-B + sqrt(discriminant)) / (2*A);

				if (t1 < t2 && t1 > 0) {
					t = t1;
				}
				else if (t2 > 0) {
					t = t2;
				}
				else {
					return false;
				}
			}

            intersect = p1 + t * d;
            normal = glm::normalize(intersect - m_center);
        }
        else
        {// static collision handling.
			if (discriminant < 0) {
				return false;
			}
			else if (discriminant == 0) {
				t = -B/(2*A);
			}
			else {
				float t1 = (-B - sqrt(discriminant)) / (2*A);
				float t2 = (-B + sqrt(discriminant)) / (2*A);

				if (abs(t1) < abs(t2)) {
					t = t1;
				}
				else {
					t = t2;
				}
			}

            intersect = p1 + t * d;
            normal = glm::normalize(intersect - m_center);
        }
        return true;
    }
    else
        return false;
}

//----------Scene Visitor Class----------//
XMLSceneVisitor::XMLSceneVisitor(Scene* const scene) : 
    TiXmlVisitor(),
    m_scene(scene),
    m_current(NULL)
{
    ;
}

XMLSceneVisitor::XMLSceneVisitor(const XMLSceneVisitor& other) : 
    TiXmlVisitor(other),
    m_scene(other.m_scene),
    m_current(other.m_current)
{
    ;
}

XMLSceneVisitor::~XMLSceneVisitor()
{
    ;
}

bool XMLSceneVisitor::VisitEnter(const TiXmlElement& element, const TiXmlAttribute* attribute)
{
    if(element.ValueStr() == "scene")
    {
        return (element.Parent() == element.GetDocument());
    }
    else if (element.ValueStr() == "materials")
    {
        return true;
    }
    else if (element.ValueStr() == "primitives")
    {
        return (element.Parent()->ValueStr() == "scene");
    }
    else if (element.ValueStr() == "plane")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        double nx(0.0f), ny(1.0f), nz(0.0f), value(0.0f);

        element.Attribute("nx", &nx);
        element.Attribute("ny", &ny);
        element.Attribute("nz", &nz);
        element.Attribute("value", &value);

        glm::vec3 normal(nx, ny, nz);
        normal = glm::normalize(normal);

        m_current = new Scene::Plane(normal, value);
        return true;
    }
    else if (element.ValueStr() == "sphere")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        double cx(0.0f), cy(1.0f), cz(0.0f), radius(0.0f);

        element.Attribute("cx", &cx);
        element.Attribute("cy", &cy);
        element.Attribute("cz", &cz);
        element.Attribute("radius", &radius);

        glm::vec3 center(cx, cy, cz);
        m_current = new Scene::Sphere(center, radius);
        return true;
    }
    else
        return false;
}

bool XMLSceneVisitor::VisitExit( const TiXmlElement& element)
{
    if(element.ValueStr() == "scene")
    {
        return (element.Parent() == element.GetDocument());
    }
    else if (element.ValueStr() == "materials")
    {
        return true;
    }
    else if (element.ValueStr() == "primitives")
    {
        return true;
    }
    else if (element.ValueStr() == "plane")
    {
        m_scene->insert_primitive(m_current);
        m_current = NULL;
        return true;
    }
    else if (element.ValueStr() == "sphere")
    {
        m_scene->insert_primitive(m_current);
        m_current = NULL;
        return true;
    }
    else
        return false;
}