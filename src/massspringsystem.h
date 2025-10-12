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
    * \fn clear
    * \brief cleanup points
    */
    void clear();


    void updateExternalForces();
    void iterate();
    void print();
    
protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    std::vector<Point> m_pointsT;

    NumericalIntegrationEuler m_numericalIntegration;

}; // class MassSpringSystem

} // namespace CompGeom

#endif // POINT_H