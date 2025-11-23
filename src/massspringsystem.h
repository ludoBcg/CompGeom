/*********************************************************************************************************************
 *
 * massspringsystem.h
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef MASSSPRINGSYSTEM_H
#define MASSSPRINGSYSTEM_H

#include <iostream>
#include <vector>
#include <assert.h>

#include "numericalintegration.h"


namespace CompGeom
{

    /*!
     * List of numerical integration methods
     */
    enum class eNumIntegMethods
    {
        FORWARD_EULER,    /* forward, explicit Euler */
        SYMPLECTIC_EULER, /* forward, semi-implicit Euler */
        BACKWARD_EULER,   /* backward, implicit Euler */
        LEAPFROG,         /* leap frog */
        MIDPOINT,         /* mid-point */
        VERLET,           /* Verlet */
        RK4               /* Runge-Kutta, 4th order */
    };

/*!
* \class MassSpringSystem
* \brief ...
*/
class MassSpringSystem
{

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn MassSpringSystem
    * \brief Default constructor
    */
    MassSpringSystem() = default;


    /*!
    * \fn ~MassSpringSystem
    * \brief Destructor
    */
    virtual ~MassSpringSystem() {};


    /*----------------------------------------------------------------------------------------------+
    |                                     GETTERS / SETTERS                                         |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn setNumIntegMethod
    */
    inline void setNumIntegMethod(eNumIntegMethods _numIntegMethod) { m_numIntegMethod = _numIntegMethod; }

    /*!
    * \fn getPointsT
    */
    inline std::vector<Point>& getPointsT() { return m_pointsT; }


    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn addPoint
    * \brief Add a new point in m_pointsT
    */
    void addPoint(glm::vec3 _pos, float _mass, float _damping);

    /*!
    * \fn addSpring
    * \brief Add a new spring in m_pointsT
    */
    void addSpring(const unsigned int _idPt1, const unsigned int _idPt2, const float _stiffness);
    

    void addConstraints(std::vector<uint32_t>& _fixedConstraints, std::vector<std::pair<uint32_t, glm::vec3> > _movingConstraint);

    /*!
    * \fn clear
    * \brief cleanup points
    */
    void clear();

    void copyPoints(std::vector<Point>& _src, std::vector<Point>& _dst);

    
    //void initSprings();
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

    /*!
    * \fn iterate
    * \brief Update system state for one timestep, using numerical integration
    */
    void iterate();

    void print();

    
protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    std::vector<Point> m_pointsT;       /*!< points at time T */
    std::vector<Point> m_pointsTinit;   /*!< buffer to store points at a previous state*/

    /*!< buffers to intermediate states of points in RK4 */
    std::vector<Point> m_pointsK1;
    std::vector<Point> m_pointsK2;
    std::vector<Point> m_pointsK3;
    std::vector<Point> m_pointsK4;

    std::vector<Spring> m_springs;

    std::vector<uint32_t> m_fixedConstraints; /* each fixed constraint point is identified by its id */
    std::vector<std::pair<uint32_t, glm::vec3> > m_movingConstraints; /* each moving constraint point is identified by its id and target position */
    float m_extForceFactor = 1.0f;

    NumericalIntegrationEuler m_integrationEuler;
    NumericalIntegrationVerlet m_integrationVerlet;
    NumericalIntegrationRK4 m_integrationRK4;
    //eNumIntegMethods m_numIntegMethod = eNumIntegMethods::FORWARD_EULER;
    //eNumIntegMethods m_numIntegMethod = eNumIntegMethods::SYMPLECTIC_EULER;
    //eNumIntegMethods m_numIntegMethod = eNumIntegMethods::BACKWARD_EULER;
    //eNumIntegMethods m_numIntegMethod = eNumIntegMethods::LEAPFROG;
    //eNumIntegMethods m_numIntegMethod = eNumIntegMethods::MIDPOINT;
    //eNumIntegMethods m_numIntegMethod = eNumIntegMethods::VERLET;
    eNumIntegMethods m_numIntegMethod = eNumIntegMethods::RK4;

    unsigned int m_counter = 0;

}; // class MassSpringSystem

} // namespace CompGeom

#endif // MASSSPRINGSYSTEM_H