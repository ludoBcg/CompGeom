/*********************************************************************************************************************
 *
 * arap.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/


#include "arap.h"

#include <iostream>


namespace CompGeom
{

    bool Arap::initialize( std::vector<glm::vec3>& _verticesPos
                         , std::vector<uint32_t>& _indices
                         , std::vector<uint32_t>& _fixedPointsIds
                         , std::vector<std::pair<uint32_t, glm::vec3> >& _constraintPoints)
    {
        m_initVertices = _verticesPos;
        m_anchorsWeight = 100.0;

        // 1. build adjacency matrix

        // initializes empty adjacency matrix
        m_adjacency.clear();
        m_adjacency = std::vector<std::vector<bool> >(_verticesPos.size(), std::vector<bool> (_verticesPos.size(), false));

        // for each triangle
        for(auto it = _indices.begin(); it != _indices.end(); it += 3)
        {
            // get the 3 vertices indices
            unsigned int id0 = *it;
            unsigned int id1 = *(it+1);
            unsigned int id2 = *(it+2);

            // add corresponding edges in adjacency matrix
            // Edge (0,1)
            m_adjacency.at(id0).at(id1) = m_adjacency.at(id1).at(id0) = true;
            // Edge (1,2) 
            m_adjacency.at(id1).at(id2) = m_adjacency.at(id2).at(id1) = true;
            // Edge (2,0)
            m_adjacency.at(id2).at(id0) = m_adjacency.at(id0).at(id2) = true;
        }


        // 2. add _fixedPointsIds to m_anchorsMap
        std::vector<std::pair<uint32_t, glm::vec3> > fixedAnchors;
        for (size_t i = 0; i < _fixedPointsIds.size(); ++i)
        {
            // add fixed points as anchors
            uint32_t id = _fixedPointsIds.at(i);
            // use initial vertex coords as target position
            glm::vec3 position = _verticesPos.at(_fixedPointsIds.at(i));
            fixedAnchors.push_back(std::make_pair(id, position));
        }
        for (auto it = fixedAnchors.begin(); it != fixedAnchors.end(); ++it)
        {
            m_anchorsMap.insert(*it);
        }

        // 3. add _constraints to m_anchorsMap with _vertices as position
        for (auto it = _constraintPoints.begin(); it != _constraintPoints.end(); ++it)
        {
            m_anchorsMap.insert(std::make_pair(it->first, _verticesPos.at(it->first) ) );
        }

        m_constraints = _constraintPoints;

        updateAnchors();

        const size_t nbVert = _verticesPos.size();

		m_rot.resize(nbVert);

        if(!buildMatrixL())
        {
            std::cerr<<"build L matrix error!"<<std::endl;
            return false;
        }

        bool success = initGuessMatrixX();

        return success;
    }


    bool Arap::iterate()
    {
        return this->solve(1e-6);
    }


    bool Arap::getResult(std::vector<glm::vec3>& _res)
    {
        _res.clear();

        for(int i = 0; i < m_matX.rows(); i++)
        {
            _res.push_back(glm::vec3(m_matX.row(i)[0], m_matX.row(i)[1], m_matX.row(i)[2]));
        }

        return true;
    }


    void Arap::updateAnchors()
    {
        for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it)
        {
            glm::vec3 anchorTargetPos = it->second;
            glm::vec3 anchorCurrentPos = m_anchorsMap.at(it->first);

            glm::vec3 displacementVec = anchorTargetPos - anchorCurrentPos;
            if(glm::length(displacementVec) > 0.001f)
                displacementVec = glm::normalize(displacementVec) * 0.01f;

            glm::vec3 anchorNextPos = anchorCurrentPos + displacementVec;
            m_anchorsMap.at(it->first) = anchorNextPos;
        }
    }


    bool Arap::buildMatrixL()
    {
        // Build the Lapacian matrix, 
        // This is an adjacency matrix with factors defining the Laplacian operator, 
        // i.e., the differential coordinates (or "umbrella" vector) delta_i 
        // defined for each vertex v_i by the difference its position and the center of mass 
        // (or barycenter) of its immediate (first-ring) neighborhood N(i):
        // delta_i = (1/d_i) * sum(v_i - v_j) =  (d_i * v_i) - sum(v_j)
        // with j in N(i) and d_i the degree (size of N(i)) of v_i

        // Number of Non-Zero (nnz) element in the matrix
        size_t nnz = 0;
        for (int i = 0; i < m_adjacency.size(); i++)
        {
            // diagonal elements
            nnz++;
            for (int j = 0; j < m_adjacency.at(i).size(); j++)
            {
                if (m_adjacency.at(i).at(j) == true)
                {
                    // count edges
                    nnz++;
                }
            }
        }

        // Each non-zero element is stored as a triplet (idRow, idColumn, value)
        std::vector<Eigen::Triplet<double> > triples;
        triples.reserve(nnz);

        for (int i=0; i<m_adjacency.size(); i++)
		{
            double d_i = 0.0;
			for (int j=0; j<m_adjacency.at(i).size(); j++)
		    {
                if (m_adjacency.at(i).at(j) == true)
                {
                    // each neighbor v_j is assigned a -1 factor
                    triples.push_back(Eigen::Triplet<double>(i, j, -m_edgesWeight));
					d_i += m_edgesWeight;
                }
		    }
            
            // add anchor weights in diagonal
            // NOTE: anchors are necessary to prevent the matrix from being negative definite,
            // thus avoiding the Eigen::NumericalIssue in the sparse Cholesky decomposition later (see SimplicialLLT::compute() below)
            if (m_anchorsMap.find(i) != m_anchorsMap.end())
            {
                d_i += m_anchorsWeight;
            }

            // add d_i factor in diagonal
            triples.push_back(Eigen::Triplet<double>(i, i, d_i));
		}

        // Build sparse Laplacian matrix from triplets
        const size_t nbVert = m_initVertices.size();
        Eigen::SparseMatrix<double> Laplace((int)nbVert, (int)nbVert);
        Laplace.setFromTriplets(triples.begin(),triples.end());

        m_llt.compute(Laplace);

        auto info = m_llt.info();
        std::string success = info == Eigen::Success ? "Success" : Eigen::NumericalIssue ? "NumericalIssue" : "Unknown";
        std::cout << "SimplicialLLT computation: " << success << std::endl;

        // Convert sparse matrix to dense matrix for console printout
        //std::cout <<  Eigen::MatrixXd(Laplace) << std::endl;

        return info == Eigen::Success;
    }


    void Arap::extractRot(const Eigen::Matrix3d& _matJ, Eigen::Matrix3d& _matR)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(_matJ, Eigen::ComputeThinU | Eigen::ComputeThinV);

        const Eigen::Matrix3d& _matU = svd.matrixU();
        const Eigen::Matrix3d& _matV = svd.matrixV();

        _matR = _matV * _matU.transpose();

        if(_matR.determinant() < 0){

            Eigen::Matrix3d _matUt = _matU;
            _matUt.col(2) = -_matU.col(2);
            _matR = _matV * _matU.transpose();
        }
    }


    void Arap::localStep()
    {
        // For each vertex i
        for (int i = 0; i < m_initVertices.size(); ++i)
        {
            Eigen::Matrix3d J  = Eigen::Matrix3d::Zero();

            Eigen::Matrix3d &R = m_rot.at(i);

            const Eigen::Vector3d v_i(m_initVertices.at(i).x, m_initVertices.at(i).y, m_initVertices.at(i).z);

            for (int j=0; j < m_adjacency.at(i).size(); ++j)
            {
                // For each neigbhor j
                if (i != j && m_adjacency.at(i).at(j) != 0)
                {
                    // compute vector (v_j, v_i)
                    
                    const Eigen::Vector3d v_j(m_initVertices.at(j).x, m_initVertices.at(j).y, m_initVertices.at(j).z);

                    const Eigen::Vector3d rv_ji = v_i - v_j;

                    // call m_matX = llt.solve(b) before to init m_matX
                    const Eigen::Vector3d dv_ji = m_matX.row(i) - m_matX.row(j);

                    J += m_edgesWeight * rv_ji * dv_ji.transpose();
                }
            }
            extractRot(J, R);
        }
    }


    bool Arap::initGuessMatrixX()
    {
        Eigen::MatrixX3d matB = Eigen::MatrixX3d::Zero(m_initVertices.size(), 3);

        // For each vertex i
        for (int i = 0; i < m_initVertices.size(); ++i)
        {
            const Eigen::Vector3d v_i(m_initVertices.at(i).x, m_initVertices.at(i).y, m_initVertices.at(i).z);

            for (int j = 0; j < m_adjacency.at(i).size(); ++j)
            {
                // For each neigbhor j
                if (i != j && m_adjacency.at(i).at(j) != 0)
                {
                    // compute vector (v_j, v_i)

                    const Eigen::Vector3d v_j(m_initVertices.at(j).x, m_initVertices.at(j).y, m_initVertices.at(j).z);

                    const Eigen::Vector3d v_ji = v_i - v_j;

                    matB.row(i) += m_edgesWeight * v_ji;
                }
            }
        }
        
        for(auto it = m_anchorsMap.begin(); it != m_anchorsMap.end(); ++it)
        {
            glm::vec3 anchorPos = it->second;
            matB.row(it->first) += m_anchorsWeight * Eigen::Vector3d(anchorPos.x, anchorPos.y, anchorPos.z);
        }

        if(m_llt.info() == Eigen::Success)
        {
            m_matX = m_llt.solve(matB);
            return true;
        }

        return false;
    }


    bool Arap::globalStep()
    {
        Eigen::MatrixX3d matB = Eigen::MatrixX3d::Zero(m_initVertices.size(), 3);

        // For each vertex i
        for (int i = 0; i < m_initVertices.size(); ++i)
        {
            Eigen::Matrix3d& R_i = m_rot.at(i);

            const Eigen::Vector3d v_i(m_initVertices.at(i).x, m_initVertices.at(i).y, m_initVertices.at(i).z);

            for (int j = 0; j < m_adjacency.at(i).size(); ++j)
            {
                // For each neigbhor j
                if (i != j && m_adjacency.at(i).at(j) != 0)
                {
                    // compute vector (v_j, v_i)

                    const Eigen::Vector3d v_j(m_initVertices.at(j).x, m_initVertices.at(j).y, m_initVertices.at(j).z);

                    const Eigen::Vector3d v_ji = v_i - v_j;

                    const Eigen::Matrix3d& R_j = m_rot.at(j);

                    matB.row(i) += 0.5 * m_edgesWeight * (R_i + R_j) * v_ji;
                }
            }
        }

        for(auto it = m_anchorsMap.begin(); it != m_anchorsMap.end(); ++it)
        {
            glm::vec3 anchorPos = it->second;
            matB.row(it->first) += m_anchorsWeight * Eigen::Vector3d(anchorPos.x, anchorPos.y, anchorPos.z);
        }

        if(m_llt.info() == Eigen::Success)
        {
            m_matX = m_llt.solve(matB);
            return true;
        }
        return false;
    }


    double Arap::l2Energy()
    {
        double e = 0;

        // For each vertex i
        for (int i = 0; i < m_initVertices.size(); ++i)
        {
            Eigen::Matrix3d& R = m_rot.at(i);

            const Eigen::Vector3d v_i(m_initVertices.at(i).x, m_initVertices.at(i).y, m_initVertices.at(i).z);

            for (int j = 0; j < m_adjacency.at(i).size(); ++j)
            {
                // For each neigbhor j
                if (i != j && m_adjacency.at(i).at(j) != 0)
                {
                    // compute vector (v_j, v_i)

                    const Eigen::Vector3d v_j(m_initVertices.at(j).x, m_initVertices.at(j).y, m_initVertices.at(j).z);

                    const Eigen::Vector3d rv_ji = v_i - v_j;

                    const Eigen::Vector3d dv_ji = m_matX.row(i) - m_matX.row(j);

                    e += m_edgesWeight * (dv_ji - R * rv_ji).squaredNorm();
                }
            }
        }
        return e;
    }


    bool Arap::solve(double _eps)
    {
        bool success = false;

        updateAnchors();

        size_t iter = 0;
        double err1 = 1,err2 = 0;
        //local-to-global interations
        while(fabs(err2-err1) > _eps)
        {
            localStep();
            success = globalStep();
            err1 = err2;
            err2 = l2Energy();
            iter++;
        }

        //std::cout<<"The number of iteration is: "<<iter<<std::endl;
        //std::cout<<"The residual is "<<err2<<std::endl;

        //update moving anchors positions
        for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it)
        {
            glm::vec3 anchorNewPos = glm::vec3(m_matX.row(it->first)[0], m_matX.row(it->first)[1], m_matX.row(it->first)[2]);
            m_anchorsMap.at(it->first) = anchorNewPos;
        }
        return success;
    }


    /*
     * Check if adjacency matrix is empty (i.e., contains only false) 
     */
    bool Arap::isAdjacencyEmpty() const
    {
    
        bool isEmpty = std::all_of(m_adjacency.begin(), m_adjacency.end(),
                                   [](const std::vector<bool>& _vec)
                                    {
                                           return std::all_of(_vec.begin(), _vec.end(),
                                                              [](bool _value){ return _value == false; });
                                    });

        return isEmpty;
    }


    /*
     * Calculates the degree of a given vertex
     * (i.e., number of connected vertices in the first-ring neighborhood) 
     */
    unsigned int Arap::getVertexDegree(const unsigned int _id) const
    {

        unsigned int cpt = 0;
        for (auto it = m_adjacency.at(_id).begin(); it != m_adjacency.at(_id).end(); ++it)
        {
            if(*it == true)
                cpt++;
        }
        return cpt;
    }
	

} // namespace CompGeom