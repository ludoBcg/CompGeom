/*********************************************************************************************************************
 *
 * point.h
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef POINT_H
#define POINT_H

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>


namespace CompGeom
{

/*!
* \class Point
* \brief ...
*/
class Point
{
    

public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn Point
    * \brief Default constructor
    */
    Point() = default;

    /*!
    * \fn Point
    * \brief Constructor
    * \param _position : position coordinates
    * \param _mass : mass of point
    * \param _damping : damping factor
    */
    Point(glm::vec3 _position, float _mass, float _damping);

    /*!
    * \fn Point
    * \brief Copy constructor
    */
    Point(Point const& _other);

    /*!
    * \fn operator=
    * \brief Copy assignment operator
    */
    Point& operator=(Point const& _other);

    /*!
    * \fn Point
    * \brief Move constructor
    */
    Point(Point&& _other);

    /*!
    * \fn operator=
    * \brief Move assignment operator
    */
    Point& operator=(Point&& _other);

    /*!
    * \fn ~Point
    * \brief Destructor
    */
    virtual ~Point() {};




    /*----------------------------------------------------------------------------------------------+
    |                                     GETTERS / SETTERS                                         |
    +-----------------------------------------------------------------------------------------------*/

    /*! \fn getPosition */
    inline glm::vec3& getPosition() { return m_position; }
    /*! \fn setPosition */
    inline void setPosition(const glm::vec3& _position) { m_position = _position; }

    /*! \fn getVelocity */
    inline glm::vec3& getVelocity() { return m_velocity; }
    /*! \fn setVelocity */
    inline void setVelocity(const glm::vec3& _velocity) { m_velocity = _velocity; }
    /*! \fn addVelocity */
    inline void addVelocity(const glm::vec3& _velocity) { m_velocity += _velocity; }

    /*! \fn getForce */
    inline const glm::vec3& getForce() const { return m_force; }
    /*! \fn setForce */
    inline void setForce(const glm::vec3& _force) { m_force = _force; }
    /*! \fn addForce */
    inline void addForce(const glm::vec3& _force) { m_force += _force; }

    /*! \fn getMass */
    inline float getMass() const { return m_mass; }
    /*! \fn setMass */
    inline void setMass(float _mass) { m_mass = _mass; }

    /*! \fn getDamping */
    inline float getDamping() const { return m_damping; }
    /*! \fn setDamping */
    inline void setDamping(float _damping) { m_damping = _damping; }

    /*! \fn isFixed */
    inline bool isFixed() const { return m_fixed; }
    /*! \fn setFixed */
    inline void setFixed(bool _fixed) { m_fixed = _fixed; }



    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    
protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/

    glm::vec3 m_position = glm::vec3(0.0);    /*!< position coordinates */
    glm::vec3 m_velocity = glm::vec3(0.0);    /*!< velocity vector */
    glm::vec3 m_force    = glm::vec3(0.0);    /*!< sum of all forces */

    float m_mass    = 1.0f;    /*!< mass of point */
    float m_damping = 0.0f;    /*!< damping factor */
    bool  m_fixed   = false;   /*!< true, if point is fixed in space */

}; // class Point

} // namespace CompGeom

#endif // POINT_H