/*********************************************************************************************************************
 *
 * dynamicmesh.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/



#include "dynamicmesh.h"

#include "dynamicalmodel.h"


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

bool DynamicMesh::buildDynamicalModel(DynamicalModel& _model)
{
    std::vector<glm::vec3> verticesPos;
    for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
    {
        verticesPos.push_back(it->pos);
    }

    _model.initialize(verticesPos, m_indices, m_fixedPointsIds, m_constraintPoints);

    return true;
}
bool DynamicMesh::readDynamicalModel(DynamicalModel& _model)
{
    std::vector<glm::vec3> newPos;
    _model.getResult(newPos);

    assert(m_vertices.size() == newPos.size()) ;

    if (m_vertices.size() != newPos.size())
    {
        std::cerr << "m_vertices.size() != _model.getResult().size() " << std::endl;
        return false;
    }

    for (size_t i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = newPos.at(i);
    }   

    return true;
}


} // namespace CompGeom