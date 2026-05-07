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

class DynamicalModel;


/*!
* \class SurfaceMesh
* \brief Triangle mesh with deformation methods
*/
class DynamicMesh : public Mesh
{


public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn DynamicMesh
    * \brief Default constructor
    */
    DynamicMesh() = default;

    /*!
    * \fn DynamicMesh
    * \brief Copy constructor
    */
    DynamicMesh(DynamicMesh const& _other) = default;

    /*!
    * \fn operator=
    * \brief Copy assignment operator
    */
    DynamicMesh& operator=(DynamicMesh const& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn DynamicMesh
    * \brief Move constructor
    */
    DynamicMesh(DynamicMesh&& _other)
        : Mesh(std::move(_other)) 
    {}

    /*!
    * \fn operator=
    * \brief Move assignment operator
    */
    DynamicMesh& operator=(DynamicMesh&& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn ~DynamicMesh
    * \brief Desctructor
    */
    virtual ~DynamicMesh() {};



    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn createGrid
    * \brief Builds a squared regular grid mesh
    *        Includes fixed points and constraints points
    * \param _lengthSide : length of grid side
    * \param _nbVertPerSide : number of vertices per side
    */
    void createGrid(const float _lengthSide, const unsigned int _nbVertPerSide) override;
   
    /*!
    * \fn buildDynamicalModel
    * \brief Initializes a DynamicalModel from current DynamicMesh geometry
    * \param _model : DynamicalModel to initialize
    */
    bool buildDynamicalModel(DynamicalModel& _model);

    /*!
    * \fn readDynamicalModel
    * \brief Reads a DynamicalModel and update DynamicMesh geometry from its current state
    * \param _model : DynamicalModel to read
    */
    bool readDynamicalModel(DynamicalModel& _model);


protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    std::vector<uint32_t> m_fixedPointsIds;     /*!< List of fixed points */

    std::vector<std::pair<uint32_t, glm::vec3> > m_constraintPoints;    /*!< List of constraint points (Id, target pos) */
    std::vector<std::pair<uint32_t, glm::vec3> > m_constraintPointsFEM; /*!< List of constraint points (Id, target pos) for FEM */


}; // class DynamicMesh

} // namespace CompGeom



#endif // DYNAMICMESH_H