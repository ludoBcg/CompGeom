/*********************************************************************************************************************
 *
 * bspline.h
 *
 * B-spline surfaces
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef BSPLINE_H
#define BSPLINE_H

#include "surfacemesh.h"


namespace CompGeom
{


/*!
* \class Bspline
* \brief b-spline surface using either de Boor algorithm, or Riesenfeld polynomials
*/
class Bspline : public SurfaceMesh
{

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn Bspline
    * \brief Default constructor
    */
    Bspline() = default;

    /*!
    * \fn Bspline
    * \brief Constructor
    */
    Bspline(bool _useDeBoor)
        : m_useDeBoor(_useDeBoor)
    {}

    /*!
    * \fn Bspline
    * \brief Copy constructor
    */
    Bspline(Bspline const& _other) = default;

    /*!
    * \fn operator=
    * \brief Copy assignment operator
    */
    Bspline& operator=(Bspline const& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn Bspline
    * \brief Move constructor
    */
    Bspline(Bspline&& _other)
        : SurfaceMesh(std::move(_other)) 
    {}

    /*!
    * \fn operator=
    * \brief Move assignment operator
    */
    Bspline& operator=(Bspline&& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn ~Bspline
    * \brief Destructor
    */
    virtual ~Bspline() {};



    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn buildParametricSurface
    * \brief Builds a b-spline surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    * \param _nbSteps : number of intermediate steps along each dimension of the surface
    */
    void buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps) override;

    /*!
    * \fn updateParametricSurface
    * \brief Updates the geometry of a b-spline surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    */
     void updateParametricSurface(Mesh& _ctrlPolygon) override;


protected:


    /*!
    * \fn fact
    * \brief Factorial function
    */
    int fact(int _i);

    int findKnotSpan(int _nbCtrlPts, int _degree, double _t);


    glm::vec3 deBoorT(int _degree, int _knotSpan, double _t,
                      const std::vector<double>& _knots,
                      std::vector<glm::vec3>& _newPts );


    glm::vec3 deBoorUV(int _degree, int _knotSpan, double _u, double _v,
                       const std::vector<double>& _knots,
                       const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints);

    void builKnots(const int _degree);

    /*!
    * \fn RiesenfeldCoeff
    * \brief Riesenfeld polynomial for b-spline surface point calculation
    * \param _n : degree of the b-spline curve (i.e., nb ctrl points - 1)
    * \param _i : index of ctrl point used for this basis function (_i in [0, _n])
    * \param _t : parametric coordinate (_t in [0.0, 1.0])
    */
    double RiesenfeldCoeff(int _n, int _i, double _t);


    /*!
    * \fn computePtUV
    * \brief Calculates 3D coordinates of surface point at parametric coords (u,v)
    * \param _ctrlPoints : 4x4 array of control points (bicubic parametric surface)
    * \param _u, _v : parametric coordinate (_u, _v in [0.0, 1.0])
    */
    glm::vec3 computePtUV(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints, 
                                  const float _u, const float _v) override;

    glm::vec3 computePtUVDeBoor(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints, 
                                const float _u, const float _v);


    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    std::vector<double> m_knots;
    bool m_useDeBoor = true;

}; // class Bspline

} // namespace CompGeom



#endif // BSPLINE_H