/*********************************************************************************************************************
 *
 * dynamicmesh.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/



#include "dynamicmesh.h"

#include "massspringsystem.h"
#include "arap.h"
#include "fem.h"
#include "pbd.h"


namespace CompGeom
{
    void DynamicMesh::createGrid(const float _lengthSide, const unsigned int _nbVertPerSide)
    {
        Mesh::createGrid(_lengthSide, _nbVertPerSide);

        //// Temporary definition of hard-coded boundary conditions (4x4)
        //m_fixedPointsIds = { 0, 3, 12, 15 };
        //m_constraintPoints = { std::make_pair<uint32_t, glm::vec3>(5, glm::vec3(0.0, 0.0, 1.0) /*target pos*/) };
        //m_constraintPointsFEM = { std::make_pair<uint32_t, glm::vec3>(5, glm::vec3(0.5, 0.5, 0.0) /*target pos*/) };

        // Temporary definition of hard-coded boundary conditions (4x4)
        m_fixedPointsIds = { 0, 3, 12, 15 };
        m_constraintPoints = { std::make_pair<uint32_t, glm::vec3>(5, glm::vec3(0.0, 0.0, 1.0) /*target pos*/) };
        m_constraintPointsFEM = { std::make_pair<uint32_t, glm::vec3>(5, glm::vec3(0.5, 0.5, 0.0) /*target pos*/) };
    
    }


/*
 * Builds a mass-spring system from the tesselation
 */
bool DynamicMesh::buildMassSpringSystem(MassSpringSystem& _massSpringSystem)
{
    std::vector<glm::vec3> verticesPos;
    for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
    {
        verticesPos.push_back(it->pos);
    }

    _massSpringSystem.initialize(verticesPos, m_indices, m_fixedPointsIds, m_constraintPoints);

    return true;
}


/*
 * Updates the tesselation from mass-spring state
 */
bool DynamicMesh::readMassSpringSystem(MassSpringSystem& _massSpringSystem)
{
    std::vector<glm::vec3> newPos;
    _massSpringSystem.getResult(newPos);

    assert(m_vertices.size() == newPos.size()) ;

    if (m_vertices.size() != newPos.size())
    {
        std::cerr << "m_vertices.size() != _massSpringSystem.getResult().size() " << std::endl;
        return false;
    }

    for (size_t i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = newPos.at(i);
    }   

    return true;
}


/*
 * Builds a PBD from the tesselation
 */
bool DynamicMesh::buildPBD(Pbd& _pbd)
{
    std::vector<glm::vec3> verticesPos;
    for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
    {
        verticesPos.push_back(it->pos);
    }

    _pbd.initialize(verticesPos, m_indices, m_fixedPointsIds, m_constraintPoints);

    return true;
}


/*
 * Updates the tesselation from PBD state
 */
bool DynamicMesh::readPBD(Pbd& _pbd)
{
    std::vector<glm::vec3> newPos;
    _pbd.getResult(newPos);

    assert(m_vertices.size() == newPos.size()) ;

    if (m_vertices.size() != newPos.size())
    {
        std::cerr << "m_vertices.size() != _pbd.getResult().size() " << std::endl;
        return false;
    }

    for (size_t i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = newPos.at(i);
    }   
    return true;
}


/*
 * Builds a arap
 */
bool DynamicMesh::buildARAP(Arap& _arap)
{
    std::vector<glm::vec3> verticesPos;
    for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
    {
        verticesPos.push_back(it->pos);
    }   


    std::vector<std::pair<uint32_t, glm::vec3> > fixedAnchors;
    for (size_t i = 0; i < m_fixedPointsIds.size(); ++i)
    {
        // add fixed points as anchors
        uint32_t id = m_fixedPointsIds.at(i);
        // use initial vertex coords as target position
        glm::vec3 position = verticesPos.at(m_fixedPointsIds.at(i));
        fixedAnchors.push_back(std::make_pair(id, position));
    }
    std::vector<std::pair<uint32_t, glm::vec3> > constraints;
    for (size_t i = 0; i < m_constraintPoints.size(); ++i)
    {
        // add constraint points as anchors
        uint32_t id = m_constraintPoints.at(i).first;
        glm::vec3 position = m_constraintPoints.at(i).second;
        constraints.push_back(std::make_pair(id, position));
    }

    _arap.initialize(verticesPos, m_indices, m_fixedPointsIds, constraints);

    verticesPos.clear();

    return true;
}


/*
 * Updates the tesselation from arap state
 */
bool DynamicMesh::readARAP(Arap& _arap)
{
    std::vector<glm::vec3> newPos;
    _arap.getResult(newPos);

    assert(m_vertices.size() == newPos.size()) ;

    if (m_vertices.size() != newPos.size())
    {
        std::cerr << "m_vertices.size() != _arap.getResult().size() " << std::endl;
        return false;
    }

    for (size_t i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = newPos.at(i);
    }   

    return true;
}


bool DynamicMesh::buildFEM(Fem& _fem)
{
    std::vector<glm::vec3> verticesPos;
    for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
    {
        verticesPos.push_back(it->pos);
    }   

     _fem.initialize(verticesPos, m_indices, m_fixedPointsIds, m_constraintPointsFEM);

    verticesPos.clear();

    return true;
}


bool DynamicMesh::readFEM(Fem& _fem)
{
    std::vector<glm::vec3> newPos;
    _fem.getResult(newPos);

    assert(m_vertices.size() == newPos.size()) ;

    if (m_vertices.size() != newPos.size())
    {
        std::cerr << "m_vertices.size() != _fem.getResult().size() " << std::endl;
        return false;
    }

    for (int i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = newPos.at(i);
    }

    return true;
}

} // namespace CompGeom