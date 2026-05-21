/*********************************************************************************************************************
 *
 * tps.h
 *
 * Thin Plate Spline surfaces
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef TPS_H
#define TPS_H

#include "surfacemesh.h"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/Geometry>


namespace CompGeom
{


/*!
* \class TPS
* \brief Thin Plate Spline surface
*/
class TPS : public SurfaceMesh
{
    // Pivoted LU decomposition
    typedef Eigen::FullPivLU<Eigen::MatrixXd> TpsLU;


public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn TPS
    * \brief Default constructor
    */
    TPS() = default;

    /*!
    * \fn TPS
    * \brief Copy constructor
    */
    TPS(TPS const& _other) = default;

    /*!
    * \fn operator=
    * \brief Copy assignment operator
    */
    TPS& operator=(TPS const& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn TPS
    * \brief Move constructor
    */
    TPS(TPS&& _other)
        : SurfaceMesh(std::move(_other)) 
    {}

    /*!
    * \fn operator=
    * \brief Move assignment operator
    */
    TPS& operator=(TPS&& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn ~TPS
    * \brief Destructor
    */
    virtual ~TPS() {};



    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn buildParametricSurface
    * \brief Builds a Thin Plate Spline surface from a control polygon
    *        cf. https://elonen.iki.fi/code/tpsdemo/
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    * \param _nbSteps : number of intermediate steps along each dimension of the surface
    */
    void buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps) override;

    /*!
    * \fn updateParametricSurface
    * \brief Updates the geometry of a Thin Plate Spline surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    */
     void updateParametricSurface(Mesh& _ctrlPolygon) override;


protected:

    /*!
    * \fn baseFunc
    * \brief function U(r)
    */
    double baseFunc(const double _r);

    /*!
    * \fn buildSubmatrixK
    * \brief builds the _p x _p submatrix K
    * \param _matK : submatrix K to build
    * \param _ctrlPoints : list of control point, _ctrlPoints.size() >= 3
    */
    bool buildSubmatrixK(Eigen::MatrixXd& _matK, std::vector<glm::vec3>& _ctrlPoints);

    /*!
    * \fn buildSsubmatrixP
    * \brief builds the _p x 3 submatrix P
    * \param _matP : submatrix P to build
    * \param _ctrlPoints : list of control point, _ctrlPoints.size() >= 3
    */
    bool buildSubmatrixP(Eigen::MatrixXd& _matP, std::vector<glm::vec3>& _ctrlPoints);

    /*!
    * \fn assembleMatrixL
    * \brief assembles the global matrix L from submatrices
    * \param _matL : global matrix L to build
    * \param _ctrlPoints : list of control point, _ctrlPoints.size() >= 3
    */
    bool assembleMatrixL(Eigen::MatrixXd& _matL, std::vector<glm::vec3>& _ctrlPoints);

    /*!
    * \fn buildVectorV
    * \brief builds the right-hand side vector V
    * \param _ctrlPoints : list of control point, _ctrlPoints.size() >= 3
    */
    bool buildVectorV(Eigen::VectorXd& _vecV, std::vector<glm::vec3>& _ctrlPoints);

    /*!
    * \fn computePtUV
    * \brief Calculates 3D coordinates of surface point at parametric coords (u,v)
    * \param _ctrlPoints : 4x4 array of control points (bicubic parametric surface)
    * \param _u, _v : parametric coordinate (_u, _v in [0.0, 1.0])
    */
    glm::vec3 computePtUV(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints, 
                           const float _u, const float _v) override;



    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    TpsLU m_LU; /*!< pivoted LU decomposition for system solver */


}; // class TPS

} // namespace CompGeom



#endif // TPS_H