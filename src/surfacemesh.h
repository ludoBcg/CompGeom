/*********************************************************************************************************************
 *
 * surfacemesh.h
 *
 * Specific mesh for interpolated surfaces
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
     * List of parametric surfaces algorithms
     */
    enum class eParametricSurface
    {
        BEZIER,     /* Bezier surface */
        BSPLINE     /* b-spline surface */
    };
    

class SurfaceMesh : public Mesh
{

public:

    SurfaceMesh() = default;

    SurfaceMesh(SurfaceMesh const& _other) = default;

    SurfaceMesh& operator=(SurfaceMesh const& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    SurfaceMesh(SurfaceMesh&& _other)
        : Mesh(std::move(_other)) 
    {}

    SurfaceMesh& operator=(SurfaceMesh&& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    virtual ~SurfaceMesh() {};

    /*!
    * \fn buildParametricSurface
    * \brief Builds a parametric (Bezier or b-spline) surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    * \param _nbSteps : number of intermediate steps along each dimension of the surface
    * \param _paramSurface : type of parametric surface to build
    */
    void buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps, eParametricSurface _paramSurface);

    /*!
    * \fn updateParametricSurface
    * \brief Updates the geometry of a parametric (Bezier or b-spline) surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    * \param _nbSteps : number of intermediate steps along each dimension of the surface
    * \param _paramSurface : type of parametric surface to build
    */
    void updateParametricSurface(Mesh& _ctrlPolygon, int _nbSteps, eParametricSurface _paramSurface);

protected:

    /*!
    * \fn fact
    * \brief Factorial function i!
    */
    int fact(int _i);

    /*!
    * \fn BernsteinCoeff
    * \brief Bernstein basis function for Bezier surface point calculation
    * \param _n : degree of the Bezier curve (i.e., nb ctrl points - 1)
    * \param _i : index of ctrl point used for this basis function (_i in [0, _n])
    * \param _t : parametric coordinate (_t in [0.0, 1.0])
    */
    double BernsteinCoeff(int _n, int _i, double _t);

    /*!
    * \fn RiesenfeldCoeff
    * \brief Riesenfeld polynomial for b-spline surface point calculation
    * \param _n : degree of the b-spline curve (i.e., nb ctrl points - 1)
    * \param _i : index of ctrl point used for this basis function (_i in [0, _n])
    * \param _t : parametric coordinate (_t in [0.0, 1.0])
    */
    double RiesenfeldCoeff(int _n, int _i, double _t);

    /*!
    * \fn computeBezierPt
    * \brief Calculates 3D coordinates of surface point at parametric coords (u,v)
    * \param _ctrlPoints : 4x4 array of control points (bicubic parametric surface)
    * \param _u, _v : parametric coordinate (_u, _v in [0.0, 1.0])
    */
    glm::vec3 computeBezierPt(glm::vec3 _ctrlPoints[4][4], float _u, float _v);

    /*!
    * \fn computeBsplinePt
    * \brief Calculates 3D coordinates of surface point at parametric coords (u,v)
    * \param _ctrlPoints : 4x4 array of control points (bicubic parametric surface)
    * \param _u, _v : parametric coordinate (_u, _v in [0.0, 1.0])
    */
    glm::vec3 computeBsplinePt(glm::vec3 _ctrlPoints[4][4], float _u, float _v);


}; // class SurfaceMesh

} // namespace CompGeom



#endif // SURFACEMESH_H