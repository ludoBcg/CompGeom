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
        VERLET
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

    std::vector<Point> m_pointsT;
    std::vector<Point> m_pointsTinit;
    std::vector<Point> m_pointsTtemp;

    std::vector<Spring> m_springs;

    NumericalIntegrationEuler m_integrationEuler;
    NumericalIntegrationVerlet m_integrationVerlet;
    eNumIntegMethods m_numIntegMethod = eNumIntegMethods::VERLET;

    unsigned int m_counter = 0;

}; // class MassSpringSystem

} // namespace CompGeom

#endif // POINT_H