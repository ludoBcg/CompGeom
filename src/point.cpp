/*********************************************************************************************************************
 *
 * point.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/


#include "point.h"

#include <iostream>


namespace CompGeom
{

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    Point::Point(glm::vec3 _position, float _mass, float _damping)
        : m_position(_position)
        , m_velocity(0.0)
        , m_force(0.0)
        , m_mass(_mass)
        , m_damping(_damping)
        , m_fixed(false)
    {}


    Point::Point(Point const& _other)
        : m_position(_other.m_position)
        , m_velocity(_other.m_velocity)
        , m_force(_other.m_force)
        , m_mass(_other.m_mass)
        , m_damping(_other.m_damping)
        , m_fixed(_other.m_fixed)
    {}


    Point& Point::operator=(Point const& _other)
    { 
        m_position = _other.m_position;
        m_velocity = _other.m_velocity;
        m_force = _other.m_force;
        m_mass = _other.m_mass;
        m_damping = _other.m_damping;
        m_fixed = _other.m_fixed;
        return *this;
    }


    Point::Point(Point&& _other)
        : m_position(std::move(_other.m_position))
        , m_velocity(std::move(_other.m_velocity))
        , m_force(std::move(_other.m_force))
        , m_mass(_other.m_mass)
        , m_damping(_other.m_damping)
        , m_fixed(_other.m_fixed)
    {}


    Point& Point::operator=(Point&& _other)
    {
        m_position = std::move(_other.m_position);
        m_velocity = std::move(_other.m_velocity);
        m_force =std::move( _other.m_force);
        m_mass = _other.m_mass;
        m_damping = _other.m_damping;
        m_fixed = _other.m_fixed;
        return *this;
    }

} // namespace CompGeom