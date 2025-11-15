/*********************************************************************************************************************
 *
 * arap.h
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef ARAP_H
#define ARAP_H

#include <iostream>
#include <vector>
#include <assert.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/Geometry>


namespace CompGeom
{

/*!
* \class Arap
* \brief As-Rigid-as-Possible mesh deformation, described is:
* O. Sorkine and M. Alexa. "As-rigid-as-possible surface modeling".
* In Proceedings of Eurographics/ACM SIGGRAPH Symposium on Geometry Processing (SGP),
* pp 109-116, 2007.
* cf. https://igl.ethz.ch/projects/ARAP/index.php
*/
class Arap
{
    typedef Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Upper>  STSimplicialLLT;
        

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn Arap
    * \brief Default constructor
    */
    Arap() = default;


    /*!
    * \fn ~Arap
    * \brief Destructor
    */
    virtual ~Arap() {};


    /*----------------------------------------------------------------------------------------------+
    |                                     GETTERS / SETTERS                                         |
    +-----------------------------------------------------------------------------------------------*/

    

    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    void initialize(std::vector<glm::vec3>& _vertices, std::vector<std::vector<bool> >& _adjacency);
    
protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    STSimplicialLLT m_llt;                /*!< sparse Cholesky decomposition from the Laplacian matrix if the mesh */
    std::vector<Eigen::Matrix3d> m_rot;   /*!< list of local rotation matrices */

}; // class Arap

} // namespace CompGeom

#endif // ARAP_H