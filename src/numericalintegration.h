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


#include "spring.h"

#include <vector>


namespace CompGeom
{

/*!
* \class NumericalIntegrationEuler
* \brief Euler method
* 
* Forward Euler:
* p = p + v*h
* v = v + (f/m)*h
*
* Symplectic/Semi-implicit Euler:
* v = v + (f/m)*h
* p = p + v*h
*
* Backward/Implicit Euler:
* p = pInit + v*h
* v = vInit + (f/m)*h
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
    * \fn updatePositionsFw
    * \brief Update position of points (forward version)
    * \param _pointsT : list of points at time T
    * \param _dt : time step
    */
    void updatePositionsFw(std::vector<Point>& _pointsT, float _dt);

    /*!
    * \fn updateVelocitiesFw
    * \brief Update velociy of points (forward version)
    * \param _pointsT : list of points at time T
    * \param _dampFact : damping factor
    * \param _dt : time step
    */
    void updateVelocitiesFw(std::vector<Point>& _pointsT, float _dampFact, float _dt);


    /*!
    * \fn updatePositionsBw
    * \brief Update position of points (backward version)
    * \param _pointsT : list of points at time T
    * \param _pointsTnext : list of points at time T+1
    * \param _dt : time step
    */
    void updatePositionsBw(std::vector<Point>& _pointsT, std::vector<Point>& _pointsTnext, float _dt);
    
    /*!
    * \fn updateVelocitiesBw
    * \brief Update velociy of points (backward version)
    * \param _pointsT : list of points at time T
    * \param _pointsTnext : list of points at time T+1
    * \param _dampFact : damping factor
    * \param _dt : time step
    */
    void updateVelocitiesBw(std::vector<Point>& _pointsT, std::vector<Point>& _pointsTnext, float _dampFact, float _dt);

}; // class NumericalIntegrationEuler


/*!
* \class NumericalIntegrationVerlet
* \brief Verlet method
* 
* Stormer–Verlet:
* p(t+h) = 2*p(t) - p(t-h) + (f/m) * (h*h)
*/
class NumericalIntegrationVerlet
{
    
public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn NumericalIntegrationVerlet
    * \brief Default constructor
    */
    NumericalIntegrationVerlet() = default;

    /*!
    * \fn ~NumericalIntegrationVerlet
    * \brief Destructor
    */
    virtual ~NumericalIntegrationVerlet() = default;


    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn updatePosAndVel
    * \brief Update position and velocity of points
    * \param _pointsT : list of points at time T
    * \param _pointsTprev : list of points at time T-1
    * \param _dampFact : damping factor
    * \param _dt : time step
    */
    void updatePosAndVel(std::vector<Point>& _pointsT, std::vector<Point>& _pointsTprev, float _dampFact, float _dt);
 
}; // class NumericalIntegrationVerlet


/*!
* \class NumericalIntegrationRK4
* \brief Runge-Kutta method (fourth order)
* 
* Given y(t) = (p(t), v(t)),
* y(t+1) is calculated with a a weighted average
* y(t+1) = y(t) + (h/6)(k1 + 2k2 + 2k3 + k4)
* 
* with 4 increments:
* k1 = F(t ,y(t) )
* k2 = F(t+(h/2) ,y(t) + (h/2)*k1 )
* k3 = F(t+(h/2) ,y(t) + (h/2)*k2 )
* k4 = F(t+h ,y(t) + h*k3 )
*/
class NumericalIntegrationRK4
{
    
public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn NumericalIntegrationRK4
    * \brief Default constructor
    */
    NumericalIntegrationRK4() = default;

    /*!
    * \fn ~NumericalIntegrationRK4
    * \brief Destructor
    */
    virtual ~NumericalIntegrationRK4() = default;


    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn computeTempPosAndVel
    * \brief Compute temporary positions and velocities for the next increment k_n
    * \param _pointsT : list of points at time T
    * \param _incrementF : list of previous increment k_n-1
    * \param _dampFact : damping factor
    * \param _dt : time step
    */
    void computeTempPosAndVel(std::vector<Point>& _pointsTinit, std::vector<Point>& _pointsT, 
                              std::vector<Point>& _prevK, std::vector<Point>& _nextK,
                              float _dampFact, float _dt);

    void computeFinalPos(std::vector<Point>& _pointsT, std::vector<Point>& _pointsTinit,
                         std::vector<Point>& _K1, std::vector<Point>& _K2,
                         std::vector<Point>& _K3, std::vector<Point>& _K4,
                         float _dampFact, float _dt);
 
}; // class NumericalIntegrationRK4

} // namespace CompGeom

#endif // NUMERICALINTEGRATION_H