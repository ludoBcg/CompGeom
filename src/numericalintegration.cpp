/*********************************************************************************************************************
 *
 * numericalintegration.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/


#include "numericalintegration.h"


namespace CompGeom
{

	void NumericalIntegrationEuler::updatePositions(std::vector<Point>& _pointsT, float _dt)
	{
		for(int i=0 ; i<_pointsT.size(); i++)
        {
            if(!_pointsT.at(i).isFixed())
            {
                // p(t+h) = p(t) + h*v(t)
                const glm::vec3 newPos = _pointsT.at(i).getPosition() + _dt * _pointsT.at(i).getVelocity();
                _pointsT.at(i).setPosition(newPos);
            }
        }
	}

    void NumericalIntegrationEuler::updateVelocities(std::vector<Point>& _pointsT, float _dt)
	{
        for(int i=0 ; i<_pointsT.size(); i++)
        {
            if(!_pointsT.at(i).isFixed())
            {
                // v(t+h) = v(t) + (h/m)*f(t)
                glm::vec3 damping = 0.05f * _pointsT.at(i).getVelocity();
                glm::vec3 force = _pointsT.at(i).getForce() - damping;
                const glm::vec3 newVel = _pointsT.at(i).getVelocity() + (_dt / _pointsT.at(i).getMass()) * force;
                _pointsT.at(i).setVelocity(newVel);
            }
            else
                _pointsT.at(i).setVelocity(glm::vec3(0.0));
        }
    }


} // namespace CompGeom