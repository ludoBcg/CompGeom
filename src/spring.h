/*********************************************************************************************************************
 *
 * spring.h
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef SPRING_H
#define SPRING_H

#include <cstdlib>
#include <tuple>

#include "point.h"


namespace CompGeom
{

/*!
* \class Spring
* \brief ...
*/
class Spring
{
    

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn Point
    * \brief Default constructor
    */
    Spring() = default;

    /*!
    * \fn Spring
    * \brief Constructor
    * \param _p0, _p1 : positions of adjacent points
    * \param _stiffness : stiffness factor
    */
    Spring( const unsigned int _id1, const unsigned int _id2
          , glm::vec3& _p1, glm::vec3& _p2
          , float _stiffness );

    /*!
    * \fn Spring
    * \brief Copy constructor
    */
    Spring(Spring const& _other);

    /*!
    * \fn operator=
    * \brief Copy assignment operator
    */
    Spring& operator=(Spring const& _other);

    /*!
    * \fn Spring
    * \brief Move constructor
    */
    Spring(Spring&& _other);

    /*!
    * \fn operator=
    * \brief Move assignment operator
    */
    Spring& operator=(Spring&& _other);

    /*!
    * \fn ~Spring
    * \brief Destructor
    */
    virtual ~Spring() {};




    /*----------------------------------------------------------------------------------------------+
    |                                     GETTERS / SETTERS                                         |
    +-----------------------------------------------------------------------------------------------*/

    /*! \fn getPointsIds */
    inline std::pair<unsigned int, unsigned int> getPointsIds() const { return m_pointsIds; }

    /*! \fn getStiffness */
    inline float getStiffness() const { return m_stiffness; }
    /*! \fn setStiffness */
    inline void setStiffness(float _stiffness) { m_stiffness = _stiffness; }

    /*! \fn getRestLength */
    inline float getRestLength() const { return m_restLength; }
    /*! \fn setRestLength */
    inline void setRestLength(float _restLength) { m_restLength = _restLength; }



    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/


    /*!
    * \fn computeForce
    * \brief Calculates the spring force, given new positions of adjacent points
    */
    const glm::vec3 calculateForce(glm::vec3 const& _p1, glm::vec3 const& _p2);
    
protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    std::pair<unsigned int, unsigned int> m_pointsIds; /*!< adjacent points */

    float m_stiffness  = 1.0f;          /*!< stiffness factor */
    float m_restLength = 0.0f;          /*!< resting length */

}; // class Spring

} // namespace CompGeom

#endif // SPRING_H