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


	void MassSpringSystem::addSpring(const unsigned int _idPt1, const unsigned int _idPt2, const float _stiffness)
	{
		assert(_idPt1 != _idPt2);

		Spring spring( _idPt1, _idPt2
			         , m_pointsT.at(_idPt1).getPosition(), m_pointsT.at(_idPt2).getPosition()
                     , _stiffness );
        m_springs.push_back(spring);
	}


	void MassSpringSystem::clear()
	{
		m_pointsT.clear();
		m_pointsTinit.clear();

		m_pointsK1.clear();
		m_pointsK2.clear();
		m_pointsK3.clear();
		m_pointsK4.clear();
	}


	void MassSpringSystem::copyPoints(std::vector<Point>& _src, std::vector<Point>& _dst)
    {
        if (_src.size() != _dst.size())
        {
            _dst.clear();
            _dst.assign(_src.size(), Point());
        }
        for (int i = 0; i < _src.size(); i++)
        {
            _dst.at(i) = _src.at(i);
        }
    }


	void MassSpringSystem::clearForces()
	{
		for (int i = 0; i < m_pointsT.size(); i++)
        {
             glm::vec3 nullForce(0.0, 0.0, 0.0);
             m_pointsT.at(i).setForce(nullForce);
        }

		// boundary conditions (temporarily hardcoded fo 5x5 grid)
		m_pointsT.at(0).setFixed(true);
		m_pointsT.at(1).setFixed(true);
		m_pointsT.at(2).setFixed(true);
		m_pointsT.at(3).setFixed(true);
		m_pointsT.at(4).setFixed(true);

		m_pointsT.at(5).setFixed(true);
		m_pointsT.at(10).setFixed(true);
		m_pointsT.at(15).setFixed(true);

		m_pointsT.at(9).setFixed(true);
		m_pointsT.at(14).setFixed(true);
		m_pointsT.at(19).setFixed(true);

		m_pointsT.at(20).setFixed(true);
		m_pointsT.at(21).setFixed(true);
		m_pointsT.at(22).setFixed(true);
		m_pointsT.at(23).setFixed(true);
		m_pointsT.at(24).setFixed(true);
		
	}


	void MassSpringSystem::updateExternalForces()
	{
		glm::vec3 targetPos(0.0, 0.0, 1.0);
        glm::vec3 forceVec = glm::normalize(targetPos - m_pointsT.at(12).getPosition()) * 0.25f; 

		// apply force on one point (temporarily hardcoded fo 5x5 grid)
		m_pointsT.at(12).addForce(forceVec);
	}

	void MassSpringSystem::updateInternalForces()
	{
		for (int i = 0; i < m_springs.size(); i++)
        {
            unsigned int id1 = m_springs.at(i).getPointsIds().first;
            unsigned int id2 = m_springs.at(i).getPointsIds().second;
            glm::vec3 p1 = m_pointsT.at(id1).getPosition();
            glm::vec3 p2 = m_pointsT.at(id2).getPosition();
           
            const glm::vec3 springForce = m_springs.at(i).calculateForce(p1, p2);

            m_pointsT.at(id1).addForce(springForce);
            m_pointsT.at(id2).addForce(-springForce);
        }
	}


	void MassSpringSystem::iterate()
	{
		float dt = 0.01f;
		float damping = 0.05f;

		switch(m_numIntegMethod)
		{
			case eNumIntegMethods::FORWARD_EULER:
			{
				dt = 0.01f;

				// calculate F_t
				clearForces();
				updateExternalForces();
				updateInternalForces();

				// P_t+1 = P_t + V_t * dt
				m_integrationEuler.updatePositionsFw(m_pointsT, dt);
				// V_t+1 = V_t + F_t * dt
				m_integrationEuler.updateVelocitiesFw(m_pointsT, damping, dt);

				break;
			}
			case eNumIntegMethods::SYMPLECTIC_EULER:
			{
				dt = 0.02f;

				// calculate F_t
				clearForces();
				updateExternalForces();
				updateInternalForces();

				// V_t+1 = V_t + F_t * dt
				m_integrationEuler.updateVelocitiesFw(m_pointsT, damping, dt);
				// P_t+1 = P_t + V_t+1 * dt
				m_integrationEuler.updatePositionsFw(m_pointsT, dt);

				break;
			}
			case eNumIntegMethods::BACKWARD_EULER:
			{
				dt = 0.1f;
            
				// copy P_t and V_t in m_pointsTinit
				copyPoints(m_pointsT, m_pointsTinit);

				// estimate P_t+1 and store it in m_pointsT, using symplectic Euler
				m_integrationEuler.updateVelocitiesFw(m_pointsT, damping, dt);
				m_integrationEuler.updatePositionsFw(m_pointsT, dt);
            
				// calculate F_t+1 based on P_t+1 estimation and store it in m_pointsT
				clearForces();
				updateExternalForces();
				updateInternalForces();

				// V_t+1 = V_t + F_t+1 * dt
				m_integrationEuler.updateVelocitiesBw(m_pointsTinit, m_pointsT, damping, dt);
				// final P_t+1 = P_t + V_t+1 * dt
				m_integrationEuler.updatePositionsBw(m_pointsTinit, m_pointsT, dt);

				break;
			}
			case eNumIntegMethods::LEAPFROG:
			{
				dt = 0.1f;

				if (m_counter % 2 == 0)
				{
					// P_t+1 = P_t + V_t * dt
					m_integrationEuler.updatePositionsFw(m_pointsT, dt);
				}
				else
				{
					// calculate F_t
					clearForces();
					updateExternalForces();
					updateInternalForces();

					// V_t+1 = V_t + F_t * dt
					m_integrationEuler.updateVelocitiesFw(m_pointsT, damping, dt);
				}
				m_counter < std::numeric_limits<unsigned int>::max() ? m_counter++ : m_counter = 0;
				
				break;
			}
			case eNumIntegMethods::MIDPOINT:
			{
				dt = 0.1f;

				// calculate F_t
				clearForces();
				updateExternalForces();
				updateInternalForces();

				// copy P_t and V_t in m_pointsTinit
				copyPoints(m_pointsT, m_pointsTinit);

				// P_t+1 = P_t + V_t * dt
				m_integrationEuler.updatePositionsFw(m_pointsT, dt*0.5f);
				// V_t+1 = V_t + F_t * dt
				m_integrationEuler.updateVelocitiesFw(m_pointsT, damping, dt*0.5f);
				
				// calculate F_t+0.5
				clearForces();
				updateExternalForces();
				updateInternalForces();

				// V_t+1 = V_t + F_t+1 * dt
				m_integrationEuler.updateVelocitiesBw(m_pointsTinit, m_pointsT, damping, dt);
				// final P_t+1 = P_t + V_t+1 * dt
				m_integrationEuler.updatePositionsBw(m_pointsTinit, m_pointsT, dt);


				break;
			}
			case eNumIntegMethods::VERLET:
			{
				dt = 0.1f;

				if (m_counter == 0)
				{
					// Apply forward Euler for the first iteration

					clearForces();
					updateExternalForces();
					updateInternalForces();

					// copy P_0  in m_pointsTinit
					copyPoints(m_pointsT, m_pointsTinit);

					m_integrationEuler.updatePositionsFw(m_pointsT, dt);
					m_integrationEuler.updateVelocitiesFw(m_pointsT, damping, dt);

					// m_pointsT now contains P_1
					
					m_counter++;
				} 
				else 
				{
					// calculate F_t
					clearForces();
					updateExternalForces();
					updateInternalForces();
					m_integrationVerlet.updatePosAndVel(m_pointsT, m_pointsTinit, damping, dt);
					
				}

				break;
			}
			case eNumIntegMethods::RK4:
			{
				dt = 0.1f;

				copyPoints(m_pointsT, m_pointsK1);
				copyPoints(m_pointsT, m_pointsK2);
				copyPoints(m_pointsT, m_pointsK3);
				copyPoints(m_pointsT, m_pointsK4);

				// calculate F_t
				clearForces();
				updateExternalForces();
				updateInternalForces();
				// copy P_0  in m_pointsTinit
			    copyPoints(m_pointsT, m_pointsTinit);
				
				// k1 = F(t ,y(t) )
			    // i.e., slope at initial position
				m_integrationRK4.computeTempPosAndVel(m_pointsTinit, m_pointsT, m_pointsTinit, m_pointsK1, damping, 0 /*dt*/);
				clearForces();
				updateExternalForces();
				updateInternalForces();
				// k2 = F(t+(h/2) ,y(t) + (h/2)*k1 )
				// i.e., slope at midpoint position, based on k1 estimation
				m_integrationRK4.computeTempPosAndVel(m_pointsTinit, m_pointsT, m_pointsK1, m_pointsK2, damping, dt * 0.5);
				clearForces();
				updateExternalForces();
				updateInternalForces();
				// k3 = F(t+(h/2) ,y(t) + (h/2)*k2 )
				// i.e., slope at midpoint position, based on k2 estimation
				m_integrationRK4.computeTempPosAndVel(m_pointsTinit, m_pointsT, m_pointsK2, m_pointsK3, damping, dt * 0.5);
				clearForces();
				updateExternalForces();
				updateInternalForces();
				// k4 = F(t+h ,y(t) + h*k3 )
				// i.e., slope at next position, based on k3 estimation
				m_integrationRK4.computeTempPosAndVel(m_pointsTinit, m_pointsT, m_pointsK3, m_pointsK4, damping, dt);
				clearForces();
				updateExternalForces();
				updateInternalForces();

				copyPoints(m_pointsTinit, m_pointsT);
				m_integrationRK4.computeFinalPos(m_pointsT, m_pointsTinit, m_pointsK1, m_pointsK2, m_pointsK3, m_pointsK4, damping, dt / 6.0);

				break;
			}
			default:
			{
				std::cerr << "Invalid numerical integration method" << std::endl;
				break;
			}
		}
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

		for(int i=0; i<m_springs.size(); i++)
		{
			std::cout << "   Spring " << i << std::endl;
			std::cout << "       P1, P2 ids: " << m_springs.at(i).getPointsIds().first 
				                        << " " << m_springs.at(i).getPointsIds().second << std::endl;
			std::cout << "       RestLength: " <<  m_springs.at(i).getRestLength() << std::endl;
			std::cout << "       Stiffness: " << m_springs.at(i).getStiffness() << std::endl << std::endl;
		}
	}


} // namespace CompGeom