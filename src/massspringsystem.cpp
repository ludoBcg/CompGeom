/*********************************************************************************************************************
 *
 * massspringsystem.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/


#include "massspringsystem.h"

#include <iostream>


namespace CompGeom
{

	void MassSpringSystem::addPoint(glm::vec3 _pos, float _mass, float _damping)
	{
		Point pt(_pos, _mass, _damping);
		m_pointsT.push_back(pt);
	}


	void MassSpringSystem::clear()
	{
		m_pointsT.clear();
	}


	void MassSpringSystem::updateExternalForces()
	{
		m_pointsT.at(3).setForce(glm::vec3(0.0f, 0.0f, 0.001f));
	}


	void MassSpringSystem::iterate()
	{
		float dt = 0.01f;
		updateExternalForces();
		m_numericalIntegration.updatePositions(m_pointsT, dt);
        m_numericalIntegration.updateVelocities(m_pointsT, dt);
	}


	void MassSpringSystem::print()
	{
		std::cout << "\n MassSpringSystem: " << std::endl;

		for(int i=0; i<m_pointsT.size(); i++)
		{
			std::cout << "   Point " << i << std::endl;
			std::cout << "       Pos: " << m_pointsT.at(i).getPosition().x 
								 << " " << m_pointsT.at(i).getPosition().y
								 << " " << m_pointsT.at(i).getPosition().z
								 << std::endl;
			std::cout << "       Vel: " << m_pointsT.at(i).getVelocity().x 
								 << " " << m_pointsT.at(i).getVelocity().y
								 << " " << m_pointsT.at(i).getVelocity().z
								 << std::endl;
			std::cout << "       Force: " << m_pointsT.at(i).getForce().x 
								   << " " << m_pointsT.at(i).getForce().y
								   << " " << m_pointsT.at(i).getForce().z
								   << std::endl;
			std::cout << "       Mass: " << m_pointsT.at(i).getMass() << std::endl;
			std::cout << "       Damping: " << m_pointsT.at(i).getDamping() << std::endl;
			std::cout << "       isFixed: " << m_pointsT.at(i).isFixed() << std::endl << std::endl;
		}
	}


} // namespace CompGeom