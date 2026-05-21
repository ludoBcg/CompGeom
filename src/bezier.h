/*********************************************************************************************************************
 *
 * bezier.h
 *
 * Bezier surfaces
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef BEZIER_H
#define BEZIER_H

#include "surfacemesh.h"


namespace CompGeom
{


/*!
* \class Bezier
* \brief Bicubic Bezier surface
*/
class Bezier : public SurfaceMesh
{

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn Bezier
    * \brief Default constructor
    */
    Bezier() = default;

    /*!
    * \fn Bezier
    * \brief Copy constructor
    */
    Bezier(Bezier const& _other) = default;

    /*!
    * \fn operator=
    * \brief Copy assignment operator
    */
    Bezier& operator=(Bezier const& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn Bezier
    * \brief Move constructor
    */
    Bezier(Bezier&& _other)
        : SurfaceMesh(std::move(_other)) 
    {}

    /*!
    * \fn operator=
    * \brief Move assignment operator
    */
    Bezier& operator=(Bezier&& _other)
    {
        Mesh::operator=(_other);
        return *this;
    }

    /*!
    * \fn ~Bezier
    * \brief Destructor
    */
    virtual ~Bezier() {};


    
    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn buildParametricSurface
    * \brief Builds a Bezier surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    * \param _nbSteps : number of intermediate steps along each dimension of the surface
    */
    void buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps) override;

    /*!
    * \fn updateParametricSurface
    * \brief Updates the geometry of a Bezier surface from a control polygon
    * \param _ctrlPolygon : control polygon mesh (can be a dynamic mesh)
    */
     void updateParametricSurface(Mesh& _ctrlPolygon) override;


protected:

    /*!
    * \fn fact
    * \brief Factorial function
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
    * \fn computePtUV
    * \brief Calculates 3D coordinates of surface point at parametric coords (u,v)
    * \param _ctrlPoints : 4x4 array of control points (bicubic parametric surface)
    * \param _u, _v : parametric coordinate (_u, _v in [0.0, 1.0])
    */
    glm::vec3 computePtUV(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints, 
                                  const float _u, const float _v) override;


}; // class Bezier

} // namespace CompGeom



#endif // BEZIER_H