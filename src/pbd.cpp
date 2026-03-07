/*********************************************************************************************************************
 *
 * pbd.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#include "pbd.h"

#include <iostream>


namespace CompGeom
{

	void Pbd::addPoint(glm::vec3 _pos, float _mass, float _damping)
	{
		Point pt(_pos, _mass, _damping);
		m_pointsT.push_back(pt);
	}


	void Pbd::addConstraints(std::vector<uint32_t>& _fixedConstraints, std::vector<std::pair<uint32_t, glm::vec3> > _movingConstraint)
	{
		m_movingConstraints = _movingConstraint;
	}

	void Pbd::addDistanceConstraint(const unsigned int _idPt1, const unsigned int _idPt2, const float _stiffness)
	{
		assert(_idPt1 != _idPt2);

		DistanceConstraint distanceConstraint( _idPt1, _idPt2
											 , m_pointsT.at(_idPt1).getPosition(), m_pointsT.at(_idPt2).getPosition()
											 , _stiffness );
        m_distanceConstraints.push_back(distanceConstraint);
	}

	void Pbd::addAnchorConstraint(const unsigned int _idPt, const glm::vec3& _pos)
	{
		AnchorConstraint anchorConstraint( _idPt, _pos );
        m_anchorConstraints.push_back(anchorConstraint);
	}


	void Pbd::clear()
	{
		m_pointsT.clear();
		m_pointsTestimate.clear();
	}


	void Pbd::copyPoints(std::vector<Point>& _src, std::vector<Point>& _dst)
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


	void Pbd::clearForces()
	{
		for (int i = 0; i < m_pointsT.size(); i++)
        {
             glm::vec3 nullForce(0.0, 0.0, 0.0);
             m_pointsT.at(i).setForce(nullForce);
        }
	}


	void Pbd::updateExternalForces()
	{
		//apply force on moving constraints (temporarily hardcoded for 5x5 grid)
		for (auto it = m_movingConstraints.begin(); it != m_movingConstraints.end(); ++it)
		{
			glm::vec3 targetPos = it->second;
			glm::vec3 constraintPos = m_pointsT.at(it->first).getPosition();
			glm::vec3 forceVec = targetPos - constraintPos;
			if(glm::length(forceVec) > 1.0f /*m_extForceFactor*/)
				forceVec = glm::normalize(forceVec) * 1.0f /*m_extForceFactor*/; 
			m_pointsT.at(it->first).addForce(forceVec);
		}
	}


	void Pbd::updateInternalForces()
	{
		
	}

	// https://github.com/marcelogm/pbd/blob/master/src/simulation/Simulator.cpp
    void Pbd::iterate()
	{
		const auto iterations = 10;
		const auto delta_t = 0.01f;
		const float dampingFactor = 0.05f;

		/*******************************************************************************/
		// 1. Apply external forces

		// m_pointsT.at(it->first).addForce(forceVec);
		updateExternalForces();

		// V_t+1 = V_t + F_t * dt
		// const glm::vec3 newVel = _pointsT.at(i).getVelocity() + (_dt / _pointsT.at(i).getMass()) * force;
        // _pointsT.at(i).setVelocity(newVel);
		m_integrationEuler.updateVelocitiesFw(m_pointsT, dampingFactor, delta_t);

		
		//// apply external forces
		//for (size_t j = 0; j < object->getVelocities()->size(); j++) 
		//{
		//	std::vector<vec3>* velocities = object->getVelocities();
		//	velocities->at(j) += ((delta_t * vec3(0) /*extForce*/)  * 10.f /*factor*/);
		//}
		/*******************************************************************************/



		/*******************************************************************************/
		// 2. Integrate

		// copy P_t and V_t in m_pointsTinit
		copyPoints(m_pointsT, m_pointsTestimate);

		for (size_t step = 0; step < 1 /*substeps*/; step++)
		{
			// final P_t+1 = P_t + V_t+1 * dt
			// p(t+h) = p(t) + h*v(t+h)
            // const glm::vec3 newPos = _pointsT.at(i).getPosition() + _dt * _pointsTnext.at(i).getVelocity();
            // _pointsTnext.at(i).setPosition(newPos);
			//m_integrationEuler.updatePositionsBw(m_pointsTinit, m_pointsT, delta_t);
			m_integrationEuler.updatePositionsBw(m_pointsT, m_pointsTestimate, delta_t);
		}

		//// integrate
		//for (size_t step = 0; step < params->substeps; step++) 
		//{
		//	const auto positions = object->getVertices();
		//	const auto velocities = object->getVelocities();
		//	auto p = object->getEstimate();
		//	for (int i = 0; i < positions->size(); i++)
		//	{
		//		const auto x = positions->at(i);
		//		const auto v = velocities->at(i);
		//		p->at(i) = x + delta_t * v;
		//	}
		//}
		/*******************************************************************************/

	
		/*******************************************************************************/
		// 3. Solve
		for (int i = 0; i < iterations; i++)
		{
			for(int j=0 ; j<m_distanceConstraints.size(); j++)
			{
				project_DistanceConstraint(m_distanceConstraints.at(j));
			}
			for(int j=0 ; j<m_anchorConstraints.size(); j++)
			{
				project_AnchorConstraint(m_anchorConstraints.at(j));
			}
		}
		
		// solve 
		/*
		for (int i = 0; i < iterations; i++) 
		{
			for (auto constraint : scene->getConstraints()) 
			{
				constraint->project();

				//https://github.com/marcelogm/pbd/blob/master/src/simulation/constraints/DistanceConstraint.cpp
				
				//https://github.com/marcelogm/pbd/blob/master/src/simulation/constraints/AnchorConstraint.cpp
			}
		}
		*/
		/*******************************************************************************/


		/*******************************************************************************/
		// 4. Update vertices and velocities

		assert(m_pointsTestimate.size() == m_pointsT.size());

        for(int i=0 ; i<m_pointsT.size(); i++)
        {
            if(!m_pointsT.at(i).isFixed())
            {
				//const auto p = m_pointsT.at(i).getPosition();
				const auto p = m_pointsTestimate.at(i).getPosition(); 
				const auto x = m_pointsT.at(i).getPosition();
                const glm::vec3 newVel = (1.0f / delta_t) * (p - x);
                m_pointsT.at(i).setVelocity(newVel);
				m_pointsT.at(i).setPosition(p);
            }
			else
			{
				m_pointsT.at(i).setVelocity(glm::vec3(0.0));
			}
        }

		// update vertices and velocities (momentum)
		//for (int j = 0; j < object->getVertices()->size(); j++) 
		//{
		//	const auto p = object->getEstimate()->at(j); /* m_pointsT */
		//	const auto x = object->getVertices()->at(j); /* m_pointsTinit */
		//	object->getVelocities()->at(j) = (p - x) / delta_t;
		//	object->getVertices()->at(j) = p;
		//}
		/*******************************************************************************/



		/*******************************************************************************/
		// 5. Velocity update: dump velocities

		for(int i=0 ; i<m_pointsT.size(); i++)
        {
            if(!m_pointsT.at(i).isFixed())
            {
                const glm::vec3 newVel = m_pointsT.at(i).getVelocity() *= 0.00998f; // @@@ 0.998f
                m_pointsT.at(i).setVelocity(newVel);
            }
			else
			{
				m_pointsT.at(i).setVelocity(glm::vec3(0.0));
			}
        }

		// velocity update: dump velocities 
		//for (int j = 0; j < object->getVertices()->size(); j++)
		//{
		//	object->getVelocities()->at(j) *= 0.998f;
		//}
		/*******************************************************************************/

		//copyPoints(m_pointsTinit, m_pointsT);
	}

	void Pbd::project_DistanceConstraint(DistanceConstraint& _distanceConstraint)
	{
		//auto params = Configuration::getInstance()->getSimulationParams();
		auto iterations = 1 /*params->iterations*/;
		auto stiffness = _distanceConstraint.m_stiffness;

		//auto object = current->getObject();
		auto i1 = _distanceConstraint.m_pointsIds.first;
		auto i2 = _distanceConstraint.m_pointsIds.second;
		auto p1 = m_pointsTestimate.at(i1).getPosition();
		auto p2 = m_pointsTestimate.at(i2).getPosition();

		// distance_constraint = | x_{1,2} - d |
		//auto error = glm::l2Norm(p1 - p2) - distance;
		glm::vec3 springVec = p2 - p1;
        float error = glm::length(springVec) - _distanceConstraint.m_restLength;
        

		// gradient_x_1 C(x_1, x_2) = n
		// gradient_x_2 C(x_1, x_2) = -n
		// n = x_{1,2} / | x_{1,2} |
		//glm::vec3 gradient = (p1 - p2) / (float)(glm::l2Norm(p1 - p2) + 0.1);
		glm::vec3 gradient = (p1 - p2) / (float)(glm::length(p1 - p2) + 0.1);

		glm::vec3 delta_p_1 = _distanceConstraint.m_scalar_1 * -error * gradient;
		glm::vec3 delta_p_2 = _distanceConstraint.m_scalar_2 * error * gradient;

		// k = 1 ? (1 ? k)^(1/ns)
		auto k = 1.f - pow(1.f - stiffness, 1.f / iterations);

		// delta_x_1 = - (w_1 / (w_1 + w_2)) * (| x_{1,2} - d |) * n
		m_pointsTestimate.at(i1).setPosition(p1 + k * delta_p_1);
		m_pointsTestimate.at(i2).setPosition(p2 + k * delta_p_2);
	}

	void Pbd::project_AnchorConstraint(AnchorConstraint& _anchorConstraint)
	{
		auto id = _anchorConstraint.m_pointsId;
		auto pos = _anchorConstraint.m_fixedPos;

		m_pointsTestimate.at(id).setPosition(pos);
	}


} // namespace CompGeom