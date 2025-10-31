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

	void NumericalIntegrationEuler::updatePositionsFw(std::vector<Point>& _pointsT, float _dt)
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

    void NumericalIntegrationEuler::updateVelocitiesFw(std::vector<Point>& _pointsT, float _dampFact, float _dt)
	{
        for(int i=0 ; i<_pointsT.size(); i++)
        {
            if(!_pointsT.at(i).isFixed())
            {
                // v(t+h) = v(t) + (h/m)*f(t)
                glm::vec3 damping = _dampFact * _pointsT.at(i).getVelocity();
                glm::vec3 force = _pointsT.at(i).getForce() - damping;
                const glm::vec3 newVel = _pointsT.at(i).getVelocity() + (_dt / _pointsT.at(i).getMass()) * force;
                _pointsT.at(i).setVelocity(newVel);
            }
            else
                _pointsT.at(i).setVelocity(glm::vec3(0.0));
        }
    }


    void NumericalIntegrationEuler::updatePositionsBw(std::vector<Point>& _pointsT, std::vector<Point>& _pointsTnext, float _dt)
	{
        assert(_pointsT.size() == _pointsTnext.size());

		for(int i=0 ; i<_pointsT.size(); i++)
        {
            if(!_pointsT.at(i).isFixed())
            {
                // p(t+h) = p(t) + h*v(t+h)
                const glm::vec3 newPos = _pointsT.at(i).getPosition() + _dt * _pointsTnext.at(i).getVelocity();
                _pointsTnext.at(i).setPosition(newPos);
            }
            else
                _pointsTnext.at(i).setPosition(_pointsT.at(i).getPosition());
        }
	}

    void NumericalIntegrationEuler::updateVelocitiesBw(std::vector<Point>& _pointsT, std::vector<Point>& _pointsTnext, float _dampFact, float _dt)
	{
        assert(_pointsT.size() == _pointsTnext.size());

        for(int i=0 ; i<_pointsT.size(); i++)
        {
            if(!_pointsT.at(i).isFixed())
            {
                // v(t+h) = v(t) + (h/m)*f(t+h)
                glm::vec3 damping = _dampFact * _pointsT.at(i).getVelocity();
                glm::vec3 force = _pointsTnext.at(i).getForce() - damping;
                const glm::vec3 newVel = _pointsT.at(i).getVelocity() + (_dt / _pointsT.at(i).getMass()) * force;
                _pointsTnext.at(i).setVelocity(newVel);
            }
            else
                _pointsTnext.at(i).setVelocity(glm::vec3(0.0));
        }
    }


    
    void NumericalIntegrationVerlet::updatePosAndVel(std::vector<Point>& _pointsT, std::vector<Point>& _pointsTprev, float _dampFact, float _dt)
    {
        // Stormer–Verlet
        // p(t+h) = 2*p(t) - p(t-1) + h*h*a(t)
        // with a(t) = (1 / m) * force

        for(int i=0 ; i<_pointsT.size(); i++)
        {
            if(!_pointsT.at(i).isFixed())
            {
                const glm::vec3 oldPos = _pointsT.at(i).getPosition();

                glm::vec3 damping = _dampFact * _pointsT.at(i).getVelocity();
                glm::vec3 force = _pointsT.at(i).getForce() - damping;
                glm::vec3 acceleration = (1.0f / _pointsT.at(i).getMass()) * force;
                const glm::vec3 newPos = 2.0f * _pointsT.at(i).getPosition() - _pointsTprev.at(i).getPosition() + _dt * _dt * acceleration;
                _pointsTprev.at(i).setPosition( _pointsT.at(i).getPosition() );
                _pointsT.at(i).setPosition(newPos);

                glm::vec3 newVel = (newPos - oldPos) / _dt;
                _pointsT.at(i).setVelocity(newVel);
            }
            else
                _pointsT.at(i).setVelocity(glm::vec3(0.0));
        }
    }


    void NumericalIntegrationRK4::computeTempPosAndVel(std::vector<Point>& _pointsTinit, std::vector<Point>& _pointsT, 
                                                       std::vector<Point>& _prevK, std::vector<Point>& _nextK,
                                                       float _dampFact, float _dt)
    {
        // Calculates an intermediate increment kn
        // k_n = F(t+dt ,y(t) + dt*k_n-1 )
        // k_n represents a slope vector
        // p(t') = p(t) + dt*p(k_n-1)
        // v(t') = v(t) + dt*v(k_n-1)
        // p(k_n) = v(t')
        // v(k_n) = f(p(t'))
        assert(_pointsT.size() == _prevK.size());
        assert(_pointsT.size() == _nextK.size());

        for(int i=0 ; i<_pointsT.size(); i++)
        {
            const glm::vec3 initPos = _pointsTinit.at(i).getPosition();
            const glm::vec3 initVel = _pointsTinit.at(i).getVelocity();

            if(!_pointsT.at(i).isFixed())
            {
                const glm::vec3 tempPos = initPos + _dt * _prevK.at(i).getPosition();
                const glm::vec3 tempVel = initVel + _dt * _prevK.at(i).getVelocity();
                
                _pointsT.at(i).setPosition(tempPos);
                _nextK.at(i).setPosition(tempVel);
                _nextK.at(i).setVelocity(_pointsT.at(i).getForce());

            }
            else
            {
                _nextK.at(i).setPosition(initPos);
                _nextK.at(i).setVelocity(glm::vec3(0.0));
            }
        }
    }

    void NumericalIntegrationRK4::computeFinalPos(std::vector<Point>& _pointsT, std::vector<Point>& _pointsTinit,
        std::vector<Point>& _K1, std::vector<Point>& _K2,
        std::vector<Point>& _K3, std::vector<Point>& _K4,
        float _dampFact, float _dt)
    {
        // Final position p(t+1) is calculated as a weighted average
        // p(t+1) = p(t) + (h/6)(k1 + 2k2 + 2k3 + k4)
        for(int i=0 ; i<_pointsT.size(); i++)
        {
            if(!_pointsT.at(i).isFixed())
            {
                const glm::vec3 tempPos = _pointsTinit.at(i).getPosition() + _dt * (_K1.at(i).getPosition() 
                                                                                  + _K2.at(i).getPosition() * 2.0f 
                                                                                  + _K3.at(i).getPosition() * 2.0f
                                                                                  + _K4.at(i).getPosition() );
                
                _pointsT.at(i).setPosition(tempPos);
            }

        }
    }


    //void NumericalIntegrationRK4::getForces(std::vector<Point>& _pointsT, std::vector<glm::vec3>& _resK)
    //{
    //    assert(_pointsT.size() == _resK.size());

    //    for(int i=0 ; i<_pointsT.size(); i++)
    //    {
    //        if(!_pointsT.at(i).isFixed())
    //        {
    //            _resK.at(i) = _pointsT.at(i).getForce();
    //        }
    //        else
    //        {
    //            _resK.at(i) = glm::vec3(0.0);
    //        }
    //    }
    //}



} // namespace CompGeom