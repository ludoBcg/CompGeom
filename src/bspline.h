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


class Bspline : public SurfaceMesh
{

public:

    Bspline() = default;

    Bspline(bool _useDeBoor)
        : m_useDeBoor(_useDeBoor)
    {}

    Bspline(Bspline const& _other) = default;

    Bspline& operator=(Bspline const& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    Bspline(Bspline&& _other)
        : SurfaceMesh(std::move(_other)) 
    {}

    Bspline& operator=(Bspline&& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    virtual ~Bspline() {};

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

    std::vector<double> m_knots;
    bool m_useDeBoor = true;


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


}; // class Bspline

} // namespace CompGeom



#endif // BSPLINE_H