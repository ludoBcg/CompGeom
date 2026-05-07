/*********************************************************************************************************************
 *
 * surfacemesh.h
 *
 * Abstract class for interpolated surface meshes
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef SURFACEMESH_H
#define SURFACEMESH_H

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "mesh.h"


namespace CompGeom
{
    

/*!
* \class SurfaceMesh
* \brief Abstract class for interpolated surface meshes
*/
class SurfaceMesh : public Mesh
{

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn SurfaceMesh
    * \brief Default constructor
    */
    SurfaceMesh() = default;

    /*!
    * \fn SurfaceMesh
    * \brief Copy constructor
    */
    SurfaceMesh(SurfaceMesh const& _other) = default;

    /*!
    * \fn operator=
    * \brief Copy assignment operator
    */
    SurfaceMesh& operator=(SurfaceMesh const& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn SurfaceMesh
    * \brief Move constructor
    */
    SurfaceMesh(SurfaceMesh&& _other)
        : Mesh(std::move(_other)) 
    {}

    /*!
    * \fn operator=
    * \brief Move assignment operator
    */
    SurfaceMesh& operator=(SurfaceMesh&& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn ~SurfaceMesh
    * \brief Desctructor
    */
    virtual ~SurfaceMesh() {};



    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn buildParametricSurface
    * \brief Builds a parametric surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    * \param _nbSteps : number of intermediate steps along each dimension of the surface
    */
    virtual void buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps) = 0;

    /*!
    * \fn updateParametricSurface
    * \brief Updates the geometry of a parametric surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    */
    virtual void updateParametricSurface(Mesh& _ctrlPolygon) = 0;


protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    unsigned int m_nbSteps = 0;     /*!< number of intermediate steps */
    


    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn computePtUV
    * \brief Calculates 3D coordinates of surface point at parametric coords (u,v)
    * \param _ctrlPoints : 4x4 array of control points (bicubic parametric surface)
    * \param _u, _v : parametric coordinate (_u, _v in [0.0, 1.0])
    */
    virtual glm::vec3 computePtUV(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints, 
                                  const float _u, const float _v) = 0;


}; // class SurfaceMesh

} // namespace CompGeom



#endif // SURFACEMESH_H