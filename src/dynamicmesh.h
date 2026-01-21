/*********************************************************************************************************************
 *
 * dynamicmesh.h
 *
 * Specific mesh with deformation methods
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef DYNAMICMESH_H
#define DYNAMICMESH_H

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "mesh.h"


namespace CompGeom
{

class MassSpringSystem;
class Arap;
class Fem;

class DynamicMesh : public Mesh
{


public:

    DynamicMesh() = default;

    DynamicMesh(DynamicMesh const& _other) = default;

    DynamicMesh& operator=(DynamicMesh const& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    DynamicMesh(DynamicMesh&& _other)
        : Mesh(std::move(_other)) 
    {}

    DynamicMesh& operator=(DynamicMesh&& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    virtual ~DynamicMesh() {};

    void createGrid(const float _lengthSide, const unsigned int _nbVertPerSide) override;

    bool isAdjacencyEmpty() const;
    unsigned int getVertexDegree(const unsigned int _id) const;
   
    bool buildMassSpringSystem(MassSpringSystem& _massSpringSystem);
    bool readMassSpringSystem(MassSpringSystem& _massSpringSystem);
    bool buildARAP(Arap& _arap);
    bool readARAP(Arap& _arap);
    bool buildFEM(Fem& _fem);
    bool readFEM(Fem& _fem);


protected:

    // Adjacency matrix
    std::vector<std::vector<bool> > m_adjacency;

    // List of fixed points
    std::vector<uint32_t> m_fixedPointsIds;
    // List of constraint points (Id, target pos)
    std::vector<std::pair<uint32_t, glm::vec3> > m_constraintPoints;
    std::vector<std::pair<uint32_t, glm::vec3> > m_constraintPointsFEM;


}; // class DynamicMesh

} // namespace CompGeom



#endif // DYNAMICMESH_H