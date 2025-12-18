/*********************************************************************************************************************
 *
 * fem.h
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef FEM_H
#define FEM_H

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
* \class Fem
* \brief Finite-Element Method 2D
*
* For each element e, the equilibrium is defined by the equation:
* K_e * u_e = f_e
*
* with:
* - u_e the displacement vector
* - f_e the external forces
* - K_e the stiffness matrix defined by : 
*   K_e = (B_e)^T * E * B_e * V_e
*   with:
*   - B_e the displacement deformation matrix (see definitions later
*   - E the material stiffness matrix
*   - V_e the volume of the element
*
* The global stiffness matrix K is then assembled from all K_e
* Finally, we can build the large, sparse linear system:
* K * u = f
*
*/
class Fem
{
    
    typedef Eigen::ConjugateGradient<Eigen::MatrixXd, Eigen::Lower|Eigen::Upper> FemCG;

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn Fem
    * \brief Default constructor
    */
    Fem() = default;


    /*!
    * \fn ~Fem
    * \brief Destructor
    */
    virtual ~Fem() {};


    /*----------------------------------------------------------------------------------------------+
    |                                     GETTERS / SETTERS                                         |
    +-----------------------------------------------------------------------------------------------*/

    

    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    void initialize(std::vector<glm::vec3>& _vertices, std::vector<uint32_t>& _indices,
	                double _mu, double _lambda, double _dt);

    void addConstraints(std::vector<uint32_t>& _fixedConstraints, std::vector<std::pair<uint32_t, glm::vec3> > _movingConstraint);


    /*!
    * \fn buildPe
    * \brief Builds the barycentric matrix for a triangle with vertices _i1, _i2, and _i3
    */
    void buildPe(Eigen::Matrix3d& _Pe, int _i1, int _i2, int _i3);
	
    /*!
    * \fn buildBe
    * \brief Builds the displacement-deformation matrix
    */
    void buildBe(Eigen::MatrixXd& _Be, const Eigen::Matrix3d& _Pe);

    /*!
    * \fn buildE
    * \brief Builds the elasticity matrix
    */
    void buildE();
	
    /*!
    * \fn buildKe
    * \brief Builds the stiffness matrix for a triangle with vertices _i1, _i2, and _i3
    */
    void buildKe(Eigen::MatrixXd& _Ke, int _i1, int _i2, int _i3);
	
    /*!
    * \fn assembleK
    * \brief Builds the global stiffness matrix
    */
    void assembleK();

    /*!
    * \fn setBoundaryConditionsFixed
    * \brief Eliminates the rows and columns corresponding to fixed Vertices
    */
    void setBoundaryConditionsFixed();

    /*!
    * \fn setBoundaryConditionsForces
    * \brief Add external forces to the global system K * u = f
    */
    void setBoundaryConditionsForces();

    void getResult(std::vector<glm::vec3>& _res);
    void solve();

protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/


    Eigen::MatrixXd m_matK;                      /*!< Global stiffness matrix */
    Eigen::Matrix3d m_matE;                      /*!< Elasticity matrix */
    Eigen::VectorXd m_vecU;
    Eigen::VectorXd m_vecF;
    FemCG m_CG;

    double m_mu = 10.5;						     /*!< Lame parameters */
	double m_lambda = 0.5;
	double m_dt =0.05;						     /*!< Time Step */

    std::vector<glm::vec3> m_initVertices;       /* initial vertices */
    std::vector<uint32_t> m_indices;

    std::vector<uint32_t> m_fixedConstraints;    /* each fixed constraint point is identified by its id */
    std::vector<std::pair<uint32_t, glm::vec3> > m_movingConstraints; /* each moving constraint point is identified by its id and target position */


}; // class Fem

} // namespace CompGeom

#endif // FEM_H