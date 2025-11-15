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
* \class MassSpringSystem
* \brief ...
*/
class MassSpringSystem
{
    /*!
     * List of numerical integration methods
     */
    enum class eNumIntegMethods
    {
        FORWARD_EULER,    /* forward, explicit Euler */
        SYMPLECTIC_EULER, /* forward, semi-implicit Euler */
        BACKWARD_EULER,   /* backward, implicit Euler */
        LEAPFROG,
        MIDPOINT,
        VERLET,
        RK4               /* Runge-Kutta, 4th order */
    };
    

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

    NumericalIntegrationEuler m_integrationEuler;
    NumericalIntegrationVerlet m_integrationVerlet;
    NumericalIntegrationRK4 m_integrationRK4;
    eNumIntegMethods m_numIntegMethod = eNumIntegMethods::RK4;

    unsigned int m_counter = 0;

}; // class MassSpringSystem

} // namespace CompGeom

#endif // MASSSPRINGSYSTEM_H