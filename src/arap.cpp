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
    void Arap::updateAnchors()
    {
        glm::vec3 anchorTarget = m_backupMovingAnchor;
        glm::vec3 anchorPt = m_anchors.back().second;
        glm::vec3 displacementVec = anchorTarget - anchorPt;
        if(glm::length(displacementVec) > 0.001f)
            displacementVec = glm::normalize(displacementVec) * 0.01f;
        anchorPt += displacementVec;
        m_anchors.back().second = anchorPt;
    }


	bool Arap::initialize(std::vector<glm::vec3>& _vertices, std::vector<std::vector<bool> >& _adjacency,
                          std::vector<std::pair<uint32_t, glm::vec3> >& _anchors, double _anchorsWeight)
	{
        m_initVertices = _vertices;
        m_adjacency = _adjacency;
        m_anchors = _anchors;
        m_anchorsWeight = _anchorsWeight;
        m_backupMovingAnchor = m_anchors.back().second;
        m_anchors.back().second = _vertices.at(m_anchors.back().first);
        updateAnchors();

        const size_t nbVert = _vertices.size();

		m_rot.resize(nbVert);

        if(!buildMatrixL())
        {
            std::cerr<<"build L matrix error!"<<std::endl;
            return false;
        }

        bool success = initGuessMatrixX();

        return success;
        
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
            if (std::find_if(m_anchors.begin(), m_anchors.end(), 
                          [&](const std::pair<uint32_t, glm::vec3>& pair) { return pair.first == (uint32_t)i; }) != m_anchors.end())
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


    int Arap::extractRot(const Eigen::Matrix3d& _matJ, Eigen::Matrix3d& _matR)
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

        return 1;
    }


    int Arap::localStep()
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

        return 1;
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


        const size_t num_anchors = m_anchors.size();
        for(size_t i = 0; i < num_anchors; ++i)
        {
            glm::vec3 anchorPos = m_anchors.at(i).second;
            matB.row(m_anchors.at(i).first) += m_anchorsWeight * Eigen::Vector3d(anchorPos.x, anchorPos.y, anchorPos.z);
        }

        if(m_llt.info() == Eigen::Success)
        {
            m_matX = m_llt.solve(matB);
            return true;
        }

        return false;
    }


    int Arap::globalStep()
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


        const size_t num_anchors = m_anchors.size();
        for(size_t i = 0; i < num_anchors; ++i)
        {
            glm::vec3 anchorPos = m_anchors.at(i).second;
            matB.row(m_anchors.at(i).first) += m_anchorsWeight * Eigen::Vector3d(anchorPos.x, anchorPos.y, anchorPos.z);
        }

        if(m_llt.info() == Eigen::Success)
        {
            m_matX = m_llt.solve(matB);

            return 1;
        }

        return 0;

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


    int Arap::solve(double _eps)
    {
        size_t iter = 0;
        double err1 = 1,err2 = 0;
        //local-to-global interations
        while(fabs(err2-err1) > _eps)
        {
            localStep();
            globalStep();
            err1 = err2;
            err2 = l2Energy();
            iter++;

        }

        //std::cout<<"The number of iteration is: "<<iter<<std::endl;
        //std::cout<<"The residual is "<<err2<<std::endl;

        return 1;
    }

    void Arap::getResult(std::vector<glm::vec3>& _res)
    {
        _res.clear();

        for(int i = 0; i < m_matX.rows(); i++)
        {
            _res.push_back(glm::vec3(m_matX.row(i)[0], m_matX.row(i)[1], m_matX.row(i)[2]));
        }
    }
	

} // namespace CompGeom