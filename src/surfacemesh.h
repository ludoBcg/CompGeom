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

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "mesh.h"


namespace CompGeom
{
    /*!
     * List of parametric surfaces algorithms
     */
    enum class eParametricSurface
    {
        BEZIER,     /* Bezier surface */
        BSPLINE,    /* b-spline surface */
        TPS         /* Thin Plate Spline surface */
    };
    

class SurfaceMesh : public Mesh
{
    // Pivoted LU decomposition
    typedef Eigen::FullPivLU<Eigen::MatrixXd> TpsLU;

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
    * \param _paramSurface : type of parametric surface to build
    */
    void updateParametricSurface(Mesh& _ctrlPolygon, eParametricSurface _paramSurface);

    /*!
    * \fn buildTPSsurface
    * \brief Builds a Thin Plate Spline surface from a control polygon
    *        cf. https://elonen.iki.fi/code/tpsdemo/
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    * \param _nbSteps : number of intermediate steps along each dimension of the surface
    */
    void buildTPSsurface(Mesh& _ctrlPolygon, int _nbSteps);

    /*!
    * \fn updateTPSsurface
    * \brief Updates the geometry of a Thin Plate Spline surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    */
    void updateTPSsurface(Mesh& _ctrlPolygon);


protected:

    TpsLU m_LU;

    unsigned int m_nbSteps = 0;

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


    
    /*!
    * \fn tpsBaseFunc
    * \brief function U(r)
    */
    double tpsBaseFunc(double _r);

    /*!
    * \fn buildTPSsubmatrixK
    * \brief builds the _p x _p submatrix K
    * \param _matK : submatrix K to build
    * \param _ctrlPoints : list of control point, _ctrlPoints.size() >= 3
    */
    bool buildTPSsubmatrixK(Eigen::MatrixXd& _matK, std::vector<glm::vec3>& _ctrlPoints);

    /*!
    * \fn buildTPSsubmatrixP
    * \brief builds the _p x 3 submatrix P
    * \param _matP : submatrix P to build
    * \param _ctrlPoints : list of control point, _ctrlPoints.size() >= 3
    */
    bool buildTPSsubmatrixP(Eigen::MatrixXd& _matP, std::vector<glm::vec3>& _ctrlPoints);

    /*!
    * \fn assembleTPSmatrixL
    * \brief assembles the global matrix L from submatrices
    * \param _matL : global matrix L to build
    * \param _ctrlPoints : list of control point, _ctrlPoints.size() >= 3
    */
    bool assembleTPSmatrixL(Eigen::MatrixXd& _matL, std::vector<glm::vec3>& _ctrlPoints);

    /*!
    * \fn buildVectorV
    * \brief builds the right-hand side vector V
    * \param _ctrlPoints : list of control point, _ctrlPoints.size() >= 3
    */
    bool buildTPSvectorV(Eigen::VectorXd& _vecV, std::vector<glm::vec3>& _ctrlPoints);


}; // class SurfaceMesh

} // namespace CompGeom



#endif // SURFACEMESH_H