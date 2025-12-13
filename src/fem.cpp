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

void Fem::initialize(std::vector<glm::vec3>& _vertices, std::vector<uint32_t>& _indices)
{
	m_initVertices = _vertices;
    m_indices = _indices;

	assembleK(m_matK);

	// Convert sparse matrix to dense matrix for console printout
    // std::cout <<  Eigen::MatrixXd(m_matK) << std::endl;
}


void Fem::addConstraints(std::vector<uint32_t>& _fixedConstraints, std::vector<std::pair<uint32_t, glm::vec3> > _movingConstraint)
{
	m_fixedConstraints = _fixedConstraints;
	m_movingConstraints = _movingConstraint;

	setBoundaryConditions(m_matK);
}


void Fem::buildPe(Eigen::Matrix3d& _Pe, int _i1, int _i2, int _i3)
{

	_Pe.col(0) = Eigen::Vector3d(1.0, 1.0, 1.0);


	_Pe.row(0)[1] = m_initVertices[_i1].x;
	_Pe.row(0)[2] = m_initVertices[_i1].y;

	_Pe.row(1)[1] = m_initVertices[_i2].x;
	_Pe.row(1)[2] = m_initVertices[_i2].y;

	_Pe.row(2)[1] = m_initVertices[_i3].x;
	_Pe.row(2)[2] = m_initVertices[_i3].y;

	_Pe = _Pe.inverse().eval();
}


void Fem::buildBe(Eigen::MatrixXd& _Be, const Eigen::Matrix3d& _Pe)
{
	_Be.resize(3, 6);

	_Be.row(0)[0] = _Pe.row(0)[1];
	_Be.row(0)[2] = _Pe.row(1)[1];
	_Be.row(0)[4] = _Pe.row(2)[1];

	_Be.row(1)[1] = _Pe.row(0)[2]; 
	_Be.row(1)[3] = _Pe.row(1)[2]; 
	_Be.row(1)[5] = _Pe.row(2)[2];
	
	_Be.row(2)[0] = 0.5 * _Pe.row(0)[2];
	_Be.row(2)[1] = 0.5 * _Pe.row(0)[1];
	_Be.row(2)[2] = 0.5 * _Pe.row(1)[2];

	_Be.row(2)[3] = 0.5 * _Pe.row(1)[1];
	_Be.row(2)[4] = 0.5 * _Pe.row(2)[2];
	_Be.row(2)[5] = 0.5 * _Pe.row(2)[1];

}


void Fem::buildKe(Eigen::MatrixXd& _Ke, int _i1, int _i2, int _i3)
{
	_Ke.resize(6, 6);

	Eigen::MatrixXd Be(3, 6);
	Eigen::Matrix3d E(3, 3);
	Eigen::Matrix3d Pe(3, 3);
	double vol;

	buildPe(Pe, _i1, _i2, _i3);

	double detPe = Pe.determinant();

	vol = std::abs(1 / (detPe * 2)); //Calculation of the triangle volume (2D->area)

	buildBe(Be, Pe);

	E.row(0)[0] = 2.0 * m_mu + m_lambda; 
	E.row(0)[1] = m_lambda;

	E.row(1)[0] = m_lambda;
	E.row(1)[1] = 2.0 * m_mu + m_lambda;

	E.row(2)[2] = m_mu;

	_Ke = (Be.transpose() * (E * Be)) * vol; 
	//Ke = (Be.transpose().Multiply(E.Multiply(Be))).ProductWithScalar(vol);
}


//Global stiffness matrix. Dimensions (2 x numberOfVertexes, 2 x numberOfVertices)
void Fem::assembleK(Eigen::MatrixXd& _K )
{
	int numberOfVertices = m_initVertices.size();
	int numberOfTriangles = m_indices.size() / 3;
	
	_K.resize(2 * numberOfVertices, 2 * numberOfVertices);

	for (int tIdx = 0; tIdx < numberOfTriangles; tIdx++)
	{
		Eigen::MatrixXd Ke(6, 6);
		buildKe(Ke, m_indices[tIdx * 3], m_indices[tIdx * 3 + 1], m_indices[tIdx * 3 + 2]);
		
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				int destI, destJ;
				destI = 2 * m_indices[tIdx * 3 + i];
				destJ = 2 * m_indices[tIdx * 3 + j];

				for (int t = 0; t < 2; ++t)
				{
					for (int m = 0; m < 2; ++m)
					{
						_K.row(destI + t)[destJ + m] += Ke.row((i * 2) + t)[ (j * 2) + m];
					}
				}
			}
		}
	}
}


void Fem::setBoundaryConditions(Eigen::MatrixXd& _S)
{
	int numberOfVertices = m_initVertices.size();
	int numberOfTriangles = m_indices.size() / 3;

	Eigen::MatrixXd tempS(2 * numberOfVertices - m_fixedConstraints.size(), 2 * numberOfVertices - m_fixedConstraints.size());

	int i2 = 0;
	
	for (int i = 0; i < 2 * numberOfVertices; i++)
	{
		if (std::find(m_fixedConstraints.begin(), m_fixedConstraints.end(), i) == m_fixedConstraints.end()) 
		{
			int j2 = 0;
			for (int j = 0; j < 2 * numberOfVertices; ++j)
			{
				if (std::find(m_fixedConstraints.begin(), m_fixedConstraints.end(), j) == m_fixedConstraints.end()) 
				{
					tempS.row(i2)[j2] = _S.row(i)[j];
					j2++;
				}
			}
			i2++;
		}	
	}
	_S = tempS;
}
	

} // namespace CompGeom