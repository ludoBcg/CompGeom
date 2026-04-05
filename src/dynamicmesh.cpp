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

        //// Temporary definition of hard-coded boundary conditions (5x5)
        //m_fixedPointsIds = { 0, 4, 20, 24 };
        //m_constraintPoints = { std::make_pair<uint32_t, glm::vec3>(12, glm::vec3(0.0, 0.0, 1.0) /*target pos*/) };
        //m_constraintPointsFEM = { std::make_pair<uint32_t, glm::vec3>(12, glm::vec3(0.5, 0.5, 0.0) /*target pos*/) };

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
    _massSpringSystem.clear();

    std::vector<std::pair<unsigned int, unsigned int> > vertIds;
    
    for (int i = 0; i < m_vertices.size(); i++)
    {
        _massSpringSystem.addPoint(m_vertices.at(i).pos, 1.0f, 0.1f);
    }

    for (int i = 0; i < m_indices.size(); i += 3)
    {
        unsigned int id0 = m_indices.at(i);
        unsigned int id1 = m_indices.at(i + 1);
        unsigned int id2 = m_indices.at(i + 2);

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id0, id1)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id1, id0)) == vertIds.end() )
        {
            _massSpringSystem.addSpring(id0, id1, 0.25f);
            vertIds.push_back(std::make_pair(id0, id1));
        }

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id1, id2)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id2, id1)) == vertIds.end() )
        {
            _massSpringSystem.addSpring(id1, id2, 0.25f);
            vertIds.push_back(std::make_pair(id1, id2));
        }

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id2, id0)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id0, id2)) == vertIds.end() )
        {
            _massSpringSystem.addSpring(id2, id0, 0.25f);
            vertIds.push_back(std::make_pair(id2, id0));
        }
    }

    _massSpringSystem.addConstraints(m_fixedPointsIds, m_constraintPoints);

    return true;
}


/*
 * Updates the tesselation from mass-spring state
 */
bool DynamicMesh::readMassSpringSystem(MassSpringSystem& _massSpringSystem)
{
    if (m_vertices.size() != _massSpringSystem.getPointsT().size())
    {
        std::cerr << "m_vertices.size() != _massSpringSystem.getPointsT().size() " << std::endl;
        return false;
    }

    assert(m_vertices.size() == _massSpringSystem.getPointsT().size()) ;

    for (int i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = _massSpringSystem.getPointsT().at(i).getPosition();
    }   
    
    return true;
}


/*
 * Builds a PBD from the tesselation
 */
bool DynamicMesh::buildPBD(Pbd& _pbd)
{
    _pbd.clear();

    std::vector<std::pair<unsigned int, unsigned int> > vertIds;
    
    for (int i = 0; i < m_vertices.size(); i++)
    {
        _pbd.addPoint(m_vertices.at(i).pos, 1.0f, 0.1f);
    }

    for (int i = 0; i < m_indices.size(); i += 3)
    {
        unsigned int id0 = m_indices.at(i);
        unsigned int id1 = m_indices.at(i + 1);
        unsigned int id2 = m_indices.at(i + 2);

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id0, id1)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id1, id0)) == vertIds.end() )
        {
            _pbd.addDistanceConstraint(id0, id1, 0.5f);
            vertIds.push_back(std::make_pair(id0, id1));
        }

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id1, id2)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id2, id1)) == vertIds.end() )
        {
            _pbd.addDistanceConstraint(id1, id2, 0.5f);
            vertIds.push_back(std::make_pair(id1, id2));
        }

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id2, id0)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id0, id2)) == vertIds.end() )
        {
            _pbd.addDistanceConstraint(id2, id0, 0.5f);
            vertIds.push_back(std::make_pair(id2, id0));
        }
    }

    _pbd.addConstraints(m_fixedPointsIds, m_constraintPoints);

    for (auto it = m_fixedPointsIds.begin(); it != m_fixedPointsIds.end(); ++it)
    {
        _pbd.addAnchorConstraint(*it, m_vertices.at(*it).pos);
    }

    return true;
}


/*
 * Updates the tesselation from PBD state
 */
bool DynamicMesh::readPBD(Pbd& _pbd)
{
    if (m_vertices.size() != _pbd.getPointsT().size())
    {
        std::cerr << "m_vertices.size() != _pbd.getPointsT().size() " << std::endl;
        return false;
    }

    assert(m_vertices.size() == _pbd.getPointsT().size()) ;

    for (int i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = _pbd.getPointsT().at(i).getPosition();
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

    _fem.initialize(verticesPos, m_indices, 10.5, 0.5);

    verticesPos.clear();

    _fem.addConstraints(m_fixedPointsIds, m_constraintPointsFEM);
    //_fem.solve();

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