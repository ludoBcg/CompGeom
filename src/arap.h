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

#include "dynamicalmodel.h"

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
class Arap : public DynamicalModel
{
    // Sparse LL^T Cholesky factorization
    typedef Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Upper> ArapSimplicialLLT;
        

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

    /*!
    * \fn initialize
    * \brief Initializes dynamical model
    * \param _vertices : List of vertices
    * \param _indices : List of indices
    * \param _fixedPointsIds : List of fixed points indices
    * \param _constraintPoints : List of constraint points (Id, target pos)
    * \return : success
    */
    bool initialize( std::vector<glm::vec3>& _verticesPos
                   , std::vector<uint32_t>& _indices
                   , std::vector<uint32_t>& _fixedPointsIds
                   , std::vector<std::pair<uint32_t, glm::vec3> >& _constraintPoints) override;

    /*!
    * \fn iterate
    * \brief Update system state for one timestep, using numerical integration
    * \return : success
    */
    bool iterate() override;

    /*!
    * \fn getResult
    * \brief Returns new vertices' position
    * \param _res : List of vertices to return
    * \return : success
    */
    bool getResult(std::vector<glm::vec3>& _res) override;


    /*!
    * \fn updateConstraints
    * \brief Moves anchors' positions for live animation
    */
    void updateAnchors();

    bool isAdjacencyEmpty() const;
    unsigned int getVertexDegree(const unsigned int _id) const;


protected:

    /*!
    * \fn buildMatrixL
    * \brief Build Laplacaian matrix
    */
    bool buildMatrixL();

    /*!
    * \fn initGuessMatrixX
    * \brief First iteration to fill-in matrix X 
    */
    bool initGuessMatrixX();

    /*!
    * \fn extractRot
    * \brief Calculates rigid transformation 
    */
    void extractRot(const Eigen::Matrix3d& _matJ, Eigen::Matrix3d& _matR);

    /*!
    * \fn localStep
    * \brief Computes optimal transformation from x (in matrix B) to x' (in matrix X)
    */
    void localStep();

    /*!
    * \fn globalStep
    * \brief Solve LX=B system to update values of X
    * \return : success
    */
    bool globalStep();

    /*!
    * \fn l2Energy
    */
    double l2Energy();

    /*!
    * \fn solve
    * \brief Complete solving process, i.e., one iteration for live animation
    * \return : success
    */
    bool solve(double _eps);


    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    ArapSimplicialLLT m_llt;                /*!< sparse Cholesky decomposition from the Laplacian matrix if the mesh */
    std::vector<Eigen::Matrix3d> m_rot;     /*!< list of local rotation matrices */
    Eigen::MatrixX3d m_matX;                /*!< X matrix (coordinates of vertices) */

    std::map<uint32_t, glm::vec3> m_anchorsMap; /* each anchor point is identified by its id and target position */
    std::vector<std::pair<uint32_t, glm::vec3> > m_constraints; /* backup ultimate target position for moving anchors */
    double m_anchorsWeight = 1.0;               /* anchors' weight */
    double m_edgesWeight = 1.0;                 /* edges' weight, we use constant weight instead of cotan weights */

    std::vector<glm::vec3> m_initVertices;       /* initial vertices */
    std::vector<std::vector<bool> > m_adjacency; /* adjacency matrix */


}; // class Arap

} // namespace CompGeom

#endif // ARAP_H