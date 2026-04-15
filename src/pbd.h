/*********************************************************************************************************************
 *
 * pbd.h
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef PBD_H
#define PBD_H

#include "dynamicalmodel.h"

#include "numericalintegration.h"


namespace CompGeom
{

// based on : //https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/tree/master/PositionBasedDynamics

class AnchorConstraint
{
public:
    AnchorConstraint(const unsigned int _pointsId, const glm::vec3& _fixedPos)
    : m_pointsId(_pointsId)
    , m_fixedPos(_fixedPos)
    {}

    unsigned int m_pointsId;
    glm::vec3 m_fixedPos;
};

class DistanceConstraint
{
public:

    void precomputeInverseMassCoeff() 
    {
	    auto w_1 = 1.0; //current->getObject()->getMasses()->at(i1).inverse;
	    auto w_2 = 1.0; //current->getObject()->getMasses()->at(i2).inverse;
	    // inverse masses precomputation
	    m_scalar_1 = 1.f / (w_1 / (w_1 + w_2));
	    m_scalar_2 = 1.f / (w_2 / (w_1 + w_2));
    }

    DistanceConstraint( const unsigned int _id1, const unsigned int _id2
                      , glm::vec3& _p1, glm::vec3& _p2
                      , float _stiffness)
        : m_pointsIds(_id1, _id2)
        , m_stiffness(_stiffness)
        , m_restLength( glm::length(_p1 - _p2) )
    {
        precomputeInverseMassCoeff() ;
    }

    std::pair<unsigned int, unsigned int> m_pointsIds; /*!< adjacent points */
    float m_stiffness  = 0.25f;          /*!< stiffness factor */
    float m_restLength = 0.0f;           /*!< resting length */

    float m_scalar_1 = 0.0f;
    float m_scalar_2 = 0.0f;
};


/*!
* \class Pbd
* \brief Position Based Dynamics
*/
class Pbd : public DynamicalModel
{

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn Pbd
    * \brief Default constructor
    */
    Pbd() = default;


    /*!
    * \fn ~Pbd
    * \brief Destructor
    */
    virtual ~Pbd() {};


    /*----------------------------------------------------------------------------------------------+
    |                                     GETTERS / SETTERS                                         |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn getPointsT
    */
    inline std::vector<Point>& getPointsT() { return m_pointsT; }


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
    * \fn addPoint
    * \brief Add a new point in m_pointsT
    */
    void addPoint(glm::vec3 _pos, float _mass, float _damping);


    void addConstraints(std::vector<uint32_t>& _fixedConstraints, std::vector<std::pair<uint32_t, glm::vec3> > _movingConstraint);

    /*!
    * \fn clear
    * \brief cleanup points
    */
    void clear();

    void copyPoints(std::vector<Point>& _src, std::vector<Point>& _dst);

    void clearForces();

    /*!
    * \fn updateExternalForces
    * \brief Add constraint forces on points
    */
    void updateExternalForces();

    /*!
    * \fn updateInternalForces
    * \brief Calculate spring forces based on current positions in m_pointsT
    */
    void updateInternalForces();

    void project_DistanceConstraint(DistanceConstraint& _distanceConstraint, int _nbIterations);
    void project_AnchorConstraint(AnchorConstraint& _anchorConstraint);

    void addDistanceConstraint(const unsigned int _idPt1, const unsigned int _idPt2, const float _stiffness);
    void addAnchorConstraint(const unsigned int _idPt, const glm::vec3& _pos);

protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    std::vector<Point> m_pointsT;       /*!< points at time T */

    std::vector<Point> m_pointsTestimate;

    std::vector<DistanceConstraint> m_distanceConstraints;
    std::vector<AnchorConstraint> m_anchorConstraints;

    std::vector<std::pair<uint32_t, glm::vec3> > m_movingConstraints;

    NumericalIntegrationEuler m_integrationEuler;


}; // class Pbd

} // namespace CompGeom

#endif // PBD_H