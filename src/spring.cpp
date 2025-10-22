/*********************************************************************************************************************
 *
 * spring.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/


#include "spring.h"


namespace CompGeom
{

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    Spring::Spring( const unsigned int _id1, const unsigned int _id2
                  , glm::vec3& _p1, glm::vec3& _p2
                  , float _stiffness )
        : m_pointsIds(_id1, _id2)
        , m_stiffness(_stiffness)
        , m_restLength( glm::length(_p1 - _p2) ) // resting length is the initial distance between the two adjacent point
    {
    }


    Spring::Spring(Spring const& _other)
        : m_pointsIds(_other.m_pointsIds)
        , m_stiffness(_other.m_stiffness)
        , m_restLength(_other.m_restLength)
    {}


    Spring& Spring::operator=(Spring const& _other)
    { 
        m_pointsIds = _other.m_pointsIds;
        m_stiffness = _other.m_stiffness;
        m_restLength = _other.m_restLength;
        return *this;
    }


    Spring::Spring(Spring&& _other)
        : m_pointsIds(_other.m_pointsIds)
        , m_stiffness(_other.m_stiffness)
        , m_restLength(_other.m_restLength)
    {}


    Spring& Spring::operator=(Spring&& _other)
    {
        m_pointsIds = _other.m_pointsIds;
        m_stiffness = _other.m_stiffness;
        m_restLength = _other.m_restLength;
        return *this;
    }


    const glm::vec3 Spring::calculateForce(glm::vec3 const& _p1, glm::vec3 const& _p2)
    {
        glm::vec3 springVec = _p2 - _p1;
        float lengthDiff = glm::length(springVec) - getRestLength();
        glm::vec3 springForce = getStiffness() * lengthDiff * glm::normalize(springVec);

        return springForce;
    }

} // namespace CompGeom