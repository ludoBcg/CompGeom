/*********************************************************************************************************************
 *
 * fem.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/


#include "fem.h"

#include <iostream>


namespace CompGeom
{

void Fem::initialize(std::vector<glm::vec3>& _vertices, std::vector<uint32_t>& _indices,
	                 double _mu, double _lambda, double _dt)
{
	m_initVertices = _vertices;
    m_indices = _indices;

	m_mu = _mu;
	m_lambda = _lambda;
	m_dt = _dt;

	buildE();

	assembleK();
}


void Fem::addConstraints(std::vector<uint32_t>& _fixedConstraints, std::vector<std::pair<uint32_t, glm::vec3> > _movingConstraint)
{
	m_fixedConstraints = _fixedConstraints;
	m_movingConstraints = _movingConstraint;

	setBoundaryConditionsFixed();
	//setBoundaryConditionsForces();
}


void Fem::buildPe(Eigen::Matrix3d& _Pe, int _i1, int _i2, int _i3)
{
	/*
    * Piecewise Approximation 2D:
	*
	* - A 2D domain is approximated by a discrete set of nodes,
	*   connected by triangular elements.
	*
	* - A function f(x,y) defined on this 2D domain is approximated by 
	*   a linear combination of basis functions N_i(x,y) defined for each node i.
	*
	* - A linear triangular element is composed of 3 nodes and 3 linear basis functions:
	*   N_i(x,y) = alpha_i + beta_i * x + gamma_i * y , i = 1, 2, 3
	*   with alpha, beta, gamma unknown factors
	*
	* - N_1(x,y) is maximal for node i=1, with coords (x_1, y_1) ( N_1(x_1, y_1) = 1), 
	*   whereas it is minimal for nodes i=2 and i=3 ( N_1(x_2, y_2) = 0, N_1(x_3, y_3) = 0), 
	*   therefore we can build the system:
	*	N_1(x_1,y_1) = alpha_1 + beta_1 * x_1 + gamma_1 * y_1 = 1
	*	N_1(x_2,y_2) = alpha_1 + beta_1 * x_2 + gamma_1 * y_2 = 0
	*	N_1(x_3,y_3) = alpha_1 + beta_1 * x_3 + gamma_1 * y_3 = 0
	*   in matrix form:
	*   | 1 x_1 y_1|   |alpha_1|   |1|
	*   | 1 x_2 y_2| * |beta_1 | = |0|
	*   | 1 x_3 y_3|   |gamma_1|   |0|
	*
	* - Repeating for N_2 and N_3 ( N_2(x_1, y_1) = 0, N_2(x_2, y_2) = 1, N_2(x_3, y_3) = 0,
	*                               N_3(x_1, y_1) = 0, N_3(x_2, y_2) = 0, N_3(x_3, y_3) = 1)
	*   gives the full system for the element e:
	*   *   in matrix form:
	*   |1 x_1 y_1|   |alpha_1 alpha_2 alpha_3|   |1 0 0|
	*   |1 x_2 y_2| * |beta_1  beta_2  beta_3 | = |0 1 0|
	*   |1 x_3 y_3|   |gamma_1 gamma_2 gamma_3|   |0 0 1|
	*
	* - The unknown factors (alpha_i, beta_i, gamma_i) contained in matrix Pe 
	*   can then be deduced by solving the system:
	*         |alpha_1 alpha_2 alpha_3|          |1 x_1 y_1|
	*   Pe =  |beta_1  beta_2  beta_3 | = inverse|1 x_2 y_2|
	*         |gamma_1 gamma_2 gamma_3|          |1 x_3 y_3|
    */

	_Pe.col(0) = Eigen::Vector3d(1.0, 1.0, 1.0);

	// x_1 y_1
	_Pe.row(0)[1] = m_initVertices[_i1].x;
	_Pe.row(0)[2] = m_initVertices[_i1].y;

	// x_2 y_2
	_Pe.row(1)[1] = m_initVertices[_i2].x;
	_Pe.row(1)[2] = m_initVertices[_i2].y;

	// x_3 y_3
	_Pe.row(2)[1] = m_initVertices[_i3].x;
	_Pe.row(2)[2] = m_initVertices[_i3].y;

	_Pe = _Pe.inverse().eval();
}


void Fem::buildBe(Eigen::MatrixXd& _Be, const Eigen::Matrix3d& _Pe)
{
	/*
    * Displacement deformation matrix Be for the element e, defined by:
	*         |alpha_2    0    beta_2   0    gamma_2   0    |
	*   Be =  |   0    alpha_3   0    beta_3    0    gamma_3|
	*         |alpha_3 alpha_2 beta_3 beta_2 gamma_3 gamma_2|
	*
	*   with factors (alpha_i, beta_i, gamma_i) taken from matrix Pe:
	*         |alpha_1 alpha_2 alpha_3|
	*   Pe =  |beta_1  beta_2  beta_3 |
	*         |gamma_1 gamma_2 gamma_3|
    */

	_Be.resize(3, 6);

	double alpha_2 = _Pe.row(0)[1];
	double beta_2  = _Pe.row(1)[1];
	double gamma_2 = _Pe.row(2)[1];
	double alpha_3 = _Pe.row(0)[2];
	double beta_3  = _Pe.row(1)[2];
	double gamma_3 = _Pe.row(2)[2];

	_Be.row(0)[0] = alpha_2;
	_Be.row(0)[2] = beta_2;
	_Be.row(0)[4] = gamma_2;

	_Be.row(1)[1] = alpha_3; 
	_Be.row(1)[3] = beta_3; 
	_Be.row(1)[5] = gamma_3;
	
	_Be.row(2)[0] = 0.5 * alpha_3;
	_Be.row(2)[1] = 0.5 * alpha_2;
	_Be.row(2)[2] = 0.5 * beta_3;
	_Be.row(2)[3] = 0.5 * beta_2;
	_Be.row(2)[4] = 0.5 * gamma_3;
	_Be.row(2)[5] = 0.5 * gamma_2;

}


void Fem::buildE()
{
	/*
    * Elasticity matrix E (or material stiffness matrix) for the element e, defined by:
	*       |(lambda + 2 * mu)      lambda        0 |
	*   E = |     lambda       (lambda + 2 * mu)  0 |
	*       |        0                 0         mu |
	*
	*   with lambda and mu the Lamé constants calculated from 
	*   the Poisson's ratio and the Young modulus, which define
	*   the mesh elasticity
    */

	m_matE.row(0)[0] = 2.0 * m_mu + m_lambda; 
	m_matE.row(0)[1] = m_lambda;

	m_matE.row(1)[0] = m_lambda;
	m_matE.row(1)[1] = 2.0 * m_mu + m_lambda;

	m_matE.row(2)[2] = m_mu;

}


void Fem::buildKe(Eigen::MatrixXd& _Ke, int _i1, int _i2, int _i3)
{
	_Ke.resize(6, 6);

	Eigen::MatrixXd Be(3, 6);
	Eigen::Matrix3d Pe(3, 3);
	double vol;

	buildPe(Pe, _i1, _i2, _i3);

	double detPe = Pe.determinant();

	vol = std::abs(1 / (detPe * 2)); //Calculation of the triangle volume (2D->area)

	buildBe(Be, Pe);

	_Ke = (Be.transpose() * (m_matE * Be)) * vol; 
	//Ke = (Be.transpose().Multiply(E.Multiply(Be))).ProductWithScalar(vol);
}


//Global stiffness matrix. Dimensions (2 x numberOfVertexes, 2 x numberOfVertices)
void Fem::assembleK()
{
	size_t nbVertices = m_initVertices.size();
	size_t nbTriangles = m_indices.size() / 3;
	
	m_matK.resize(2 * nbVertices, 2 * nbVertices);
	m_matK.setZero();

	for (size_t tId = 0; tId < nbTriangles; tId++)
	{
		// build matrix Ke for triangle element e
		Eigen::MatrixXd Ke(6, 6);
		buildKe(Ke, m_indices[tId * 3], m_indices[tId * 3 + 1], m_indices[tId * 3 + 2]);
		
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				// for each node in e
				// calculate node index in global matrix K
				int destI, destJ;
				destI = 2 * m_indices[tId * 3 + i];
				destJ = 2 * m_indices[tId * 3 + j];

				// copy content of Ke into K
				for (int x = 0; x < 2; x++)
				{
					for (int y = 0; y < 2; y++)
					{
						m_matK.row(destI + x)[destJ + y] += Ke.row((i * 2) + x)[ (j * 2) + y];
					}
				}
			}
		}
	}
}


void Fem::setBoundaryConditionsFixed()
{
	size_t nbVertices = m_initVertices.size();
	size_t nbTriangles = m_indices.size() / 3;

	Eigen::MatrixXd tempK(2 * nbVertices - m_fixedConstraints.size(), 2 * nbVertices - m_fixedConstraints.size());

	int i2 = 0;
	for (int i = 0; i < 2 * nbVertices; i++)
	{
		if (std::find(m_fixedConstraints.begin(), m_fixedConstraints.end(), i) == m_fixedConstraints.end()) 
		{
			int j2 = 0;
			for (int j = 0; j < 2 * nbVertices; ++j)
			{
				if (std::find(m_fixedConstraints.begin(), m_fixedConstraints.end(), j) == m_fixedConstraints.end()) 
				{
					tempK.row(i2)[j2] = m_matK.row(i)[j];
					j2++;
				}
			}
			i2++;
		}	
	}
	m_matK = tempK;
}


void Fem::setBoundaryConditionsForces()
{
	// ...
}


void Fem::solve()
{
	//...
}


void Fem::getResult(std::vector<glm::vec3>& _res)
{
    _res.clear();

	int j=0;
    for(int i = 0; i < m_initVertices.size(); i++)
    {
		glm::vec3 initPos = m_initVertices.at(i);
		Eigen::Vector3d displacement;
		displacement.setZero();
		if (std::find(m_fixedConstraints.begin(), m_fixedConstraints.end(), i) == m_fixedConstraints.end())
		{
			displacement[0] = m_vecU.row(j*2)[0];
			displacement[1] = m_vecU.row(j*2+1)[0];
			j++;
		}
		
        _res.push_back(initPos + glm::vec3(displacement[0], displacement[1], displacement[2]));
    }
}
	

} // namespace CompGeom