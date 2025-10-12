/*********************************************************************************************************************
 *
 * numericalintegration.h
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef NUMERICALINTEGRATION_H
#define NUMERICALINTEGRATION_H


#include "point.h"

#include <vector>


namespace CompGeom
{

/*!
* \class NumericalIntegrationEuler
* \brief ...
*/
class NumericalIntegrationEuler
{
    

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn NumericalIntegrationEuler
    * \brief Default constructor
    */
    NumericalIntegrationEuler() = default;

    /*!
    * \fn ~NumericalIntegrationEuler
    * \brief Destructor
    */
    virtual ~NumericalIntegrationEuler() = default;


    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn updatePositions
    * \brief Update position of points
    * \param _pointsT : list of points at time T
    * \param _dt : time step
    */
    void updatePositions(std::vector<Point>& _pointsT, float _dt);


    /*!
    * \fn updateVelocities
    * \brief Update velociy of points
    * \param _pointsT : list of points at time T
    * \param _dt : time step
    */
    void updateVelocities(std::vector<Point>& _pointsT, float _dt);

    
protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/



}; // class NumericalIntegrationEuler

} // namespace CompGeom

#endif // NUMERICALINTEGRATION_H