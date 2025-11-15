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
	void Arap::initialize(std::vector<glm::vec3>& _vertices, std::vector<std::vector<bool> >& _adjacency)
	{
        const size_t nbVert = _vertices.size();

		m_rot.resize(nbVert);

        // Build the Lapacian matrix, 
        // This is an adjacency matrix with factors defining the Laplacian operator, 
        // i.e., the differential coordinates (or "umbrella" vector) delta_i 
        // defined for each vertex v_i by the difference its position and the center of mass 
        // (or barycenter) of its immediate (first-ring) neighborhood N(i):
        // delta_i = (1/d_i) * sum(v_i - v_j) =  (d_i * v_i) - sum(v_j)
        // with j in N(i) and d_i the degree (size of N(i)) of v_i

        // Sparse matrix: each row is stored as a map of (idColumn, value)
        std::vector<std::map<int, double> > lapMatrix(nbVert); 

        for (int i=0; i<_adjacency.size(); i++)
		{
            double d_i = 0.0;
			for (int j=0; j<_adjacency.at(i).size(); j++)
		    {
                if (_adjacency.at(i).at(j) == true)
                {
                    // each neighbor v_j is assigned a -1 factor
                    lapMatrix.at(i).insert_or_assign(j, -1.0);
                    lapMatrix.at(j).insert_or_assign(i, -1.0);
					d_i += 1.0;
                }
		    }
            // add d_i factor in diagonal
            lapMatrix.at(i).insert_or_assign(i, d_i);
		}

        // add constraint weights in diagonal
        // (temporarily hardcoded constraints for 5x5 grid)
        // NOTE: constraints are not used yet, but adding them in the system prevents
        // the matrix from being negative definite, thus avoiding the Eigen::NumericalIssue
        // in the sparse Cholesky decomposition later (see SimplicialLLT::compute() below)
        std::vector<int> constraints = {0, 1, 2, 3 ,4,      // top border
                                        5, 10, 15,          // left border
                                        9, 14, 19,          // right border
                                        20, 21, 22, 23, 24, // bottom border
                                        12                  // center anchor
                                        };
        for (auto it = constraints.begin(); it != constraints.end(); ++it)
        {
            lapMatrix.at(*it).at(*it) += 1.0;
        }

        // Number of Non-Zero (nnz) element in the matrix
        size_t nnz = 0;
        for(size_t i = 0;i < nbVert; ++i)
        {
            nnz += lapMatrix.at(i).size();
        }

        // Each non-zero element is stored as a triplet (idRow, idColumn, value)
        std::vector<Eigen::Triplet<double> > triples;
        triples.reserve(nnz);
        for(int i = 0; i < (int)nbVert; ++i)
        {
            for(std::map<int, double>::const_iterator it = lapMatrix.at(i).begin(); it != lapMatrix.at(i).end(); ++it)
            {
                triples.push_back(Eigen::Triplet<double>(i, it->first, it->second));
            }
        }

        // Build sparse Laplacian matrix from triplets
        Eigen::SparseMatrix<double> Laplace((int)nbVert, (int)nbVert);
        Laplace.setFromTriplets(triples.begin(),triples.end());

        m_llt.compute(Laplace);

        auto info = m_llt.info();
        std::string success = info == Eigen::Success ? "Success" : Eigen::NumericalIssue ? "NumericalIssue" : "Unknown";
        std::cout << "Success: " << success << std::endl;

        // Convert sparse matrix to dense matrix for console printout
        //std::cout <<  Eigen::MatrixXd(Laplace) << std::endl;
        
    }
	

} // namespace CompGeom