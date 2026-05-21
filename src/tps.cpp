/*********************************************************************************************************************
 *
 * tps.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/



#include "tps.h"


namespace CompGeom
{


void TPS::buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps)
{
    m_nbSteps = _nbSteps;

    // cf. https://elonen.iki.fi/code/tpsdemo/

    // We use bicubic surface (i.e., 4 x 4 control points)
    const int nbCtrlPtsPerSide = 4;

    float stepSize = 1.0f / static_cast<float>(m_nbSteps);

    // nb vertices on each side = nb intermediate steps + 1
    int nbVerticesPerSide = m_nbSteps + 1;
    int nbVertices = static_cast<int>(pow(nbVerticesPerSide, 2.0));
    m_vertices.clear();
    m_vertices.assign(nbVertices, Vertex{});

    // 1. Control points grid
    std::vector<glm::vec3> ctrlPoints;
    ctrlPoints.assign(nbCtrlPtsPerSide*nbCtrlPtsPerSide, glm::vec3(0.0f));

    int cpt = 0;
    for (auto it = _ctrlPolygon.getVertices().begin(); it != _ctrlPolygon.getVertices().end(); ++it)
    {
        ctrlPoints.at(cpt) = it->pos;

        cpt++;
    }

    // 2. Build Lx=v system
    size_t p = ctrlPoints.size();
    Eigen::MatrixXd matL;
    Eigen::VectorXd vecV;
    assembleMatrixL(matL, ctrlPoints);
    buildVectorV(vecV, ctrlPoints);
    Eigen::VectorXd vecX;
    vecX.resize(p + 3);

    // 3. Solve Lx=v
    m_LU.compute(matL);

    vecX = m_LU.solve(vecV);


    // 4. Interpolate the surface vertices

    // for each intermediate coords (u,v)
    float u, v = 0.0f;
    int id = 0;
    glm::vec3 offsetCoords = ctrlPoints.front();
    glm::vec3 gridDims = ctrlPoints.back() - ctrlPoints.front();
    for (int i = 0; i < nbVerticesPerSide; i++)
    {
        u = std::min(1.0f, stepSize * i);
        u = offsetCoords.x + u * gridDims.x;

        for (int j = 0; j < nbVerticesPerSide; j++)
        {
            v = std::min(1.0f, stepSize * j);
            v = offsetCoords.y + v * gridDims.y;
        
            double h = vecX(p) + vecX(p+1)*u + vecX(p+2)*v;
            glm::vec3 pt_i, pt_cur(u, v, 0.0f);
            for ( unsigned i=0; i<p; ++i )
            {
                pt_i = ctrlPoints.at(i);
                pt_i.z = 0;
                h += vecX(i) * baseFunc( glm::length( pt_i - pt_cur ) );
            }

            glm::vec3 pos(offsetCoords.x + u * gridDims.x, offsetCoords.y + v * gridDims.y, h);
            // add vertex to mesh
            Vertex vert{ pos /* pos */, {0.4f, 0.6f, 0.2f} /* col */, 
                        {1.0f, 1.0f} /* uv */, {0.0f, 0.0f, 1.0f} /* norm */ };

            if(id<nbVertices)
                m_vertices.at(id) = vert;

            id++;
        }
    }
        

    // 5. Triangulate the parametric surface vertices
    m_indices.clear();
    cpt = 0;
    for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
    {
        unsigned int id0 = cpt % nbVerticesPerSide;
        unsigned int id1 = id0 + 1;
        unsigned int id2 = cpt / nbVerticesPerSide;

        if (id0 < m_nbSteps && id2 < m_nbSteps)
        {
            m_indices.push_back(cpt);
            m_indices.push_back(cpt + nbVerticesPerSide);
            m_indices.push_back(cpt + 1);

            m_indices.push_back(cpt + 1);
            m_indices.push_back(cpt + nbVerticesPerSide);
            m_indices.push_back(cpt + 1 + nbVerticesPerSide);
        }

        cpt++;
    }

}



void TPS::updateParametricSurface(Mesh& _ctrlPolygon)
{

    // We use bicubic surface (i.e., 4 x 4 control points)
    const int nbCtrlPtsPerSide = 4;

    float stepSize = 1.0f / static_cast<float>(m_nbSteps);

    // nb vertices on each side = nb intermediate steps + 1
    int nbVerticesPerSide = m_nbSteps + 1;
    int nbVertices = static_cast<int>(pow(nbVerticesPerSide, 2.0));
    
    assert(m_vertices.size() == nbVertices);

    // 1. Control points grid
    std::vector<glm::vec3> ctrlPoints;
    ctrlPoints.assign(nbCtrlPtsPerSide*nbCtrlPtsPerSide, glm::vec3(0.0f));

    int cpt = 0;
    for (auto it = _ctrlPolygon.getVertices().begin(); it != _ctrlPolygon.getVertices().end(); ++it)
    {
        ctrlPoints.at(cpt) = it->pos;

        cpt++;
    }

    // 2. Build Lx=v system
    size_t p = ctrlPoints.size();
    Eigen::MatrixXd matL;
    Eigen::VectorXd vecV;
    assembleMatrixL(matL, ctrlPoints);
    buildVectorV(vecV, ctrlPoints);
    Eigen::VectorXd vecX;
    vecX.resize(p + 3);

    // 3. Solve Lx=v
    m_LU.compute(matL);

    vecX = m_LU.solve(vecV);


    // 4. Interpolate the surface vertices

    // for each intermediate coords (u,v)
    float u, v = 0.0f;
    int id = 0;
    glm::vec3 offsetCoords = ctrlPoints.front();
    glm::vec3 gridDims = ctrlPoints.back() - ctrlPoints.front();
    for (int i = 0; i < nbVerticesPerSide; i++)
    {
        u = std::min(1.0f, stepSize * i);
        u = offsetCoords.x + u * gridDims.x;

        for (int j = 0; j < nbVerticesPerSide; j++)
        {
            v = std::min(1.0f, stepSize * j);
            v = offsetCoords.y + v * gridDims.y;
        
            double h = vecX(p) + vecX(p+1)*u + vecX(p+2)*v;
            glm::vec3 pt_i, pt_cur(u, v, 0.0f);
            for ( unsigned i=0; i<p; ++i )
            {
                pt_i = ctrlPoints.at(i);
                pt_i.z = 0;
                h += vecX(i) * baseFunc( glm::length( pt_i - pt_cur ) );
            }

            glm::vec3 pos(u, v, h);
            // add vertex to mesh
            Vertex vert{ pos /* pos */, {0.4f, 0.6f, 0.2f} /* col */, 
                        {1.0f, 1.0f} /* uv */, {0.0f, 0.0f, 1.0f} /* norm */ };

            if(id<nbVertices)
                m_vertices.at(id) = vert;

            id++;
        }
    }
        
}

    
double TPS::baseFunc(const double _r)
{
    return _r == 0.0 ? 0.0 : _r * _r * log(_r);
}


bool TPS::buildSubmatrixK(Eigen::MatrixXd& _matK, std::vector<glm::vec3>& _ctrlPoints)
{
    size_t p = _ctrlPoints.size();

    if (p < 3)
    {
        std::cerr << "Number of control = " << p << ", must be  >= 3 " << std::endl;
        return false;
    }

    // the higher the lambda, the more rigid the surface
    double lambda = 0.3;

    if(_matK.rows() != p || _matK.cols() != p)
        _matK.resize(p, p);

    assert(_matK.size() == p*p);

	_matK.setZero();

    double a = 0.0;
    for (int i = 0; i < p; i++)
    {
        for (int j = 0; j < p; j++)
        {
                
            if(j==i)
                _matK.col(i)[j] = 0;
            else
            {
                // K_i,j = K_j,i = ||P_i - P_j||^2 * log(||P_i - P_j||)

                glm::vec3 Pi = _ctrlPoints.at(i);
                glm::vec3 Pj = _ctrlPoints.at(j);
                float normPiPj = glm::length(Pi - Pj);

                //_matK.col(i)[j] = (normPiPj * normPiPj) * log(normPiPj);
                _matK.col(i)[j] = baseFunc(normPiPj);

                a += normPiPj * 2;
            }
        }
    }
    a /= static_cast<float>(p*p);

    for (int i = 0; i < p; i++)
    {
        // diagonal: reqularization parameters (lambda * a^2)
        _matK.col(i)[i] = lambda * (a * a);
    }

    return true;
}


bool TPS::buildSubmatrixP(Eigen::MatrixXd& _matP, std::vector<glm::vec3>& _ctrlPoints)
{
    size_t p = _ctrlPoints.size();

    if (p < 3)
    {
        std::cerr << "Number of control = " << p << ", must be  >= 3 " << std::endl;
        return false;
    }

    if(_matP.rows() != p || _matP.cols() != 3)
        _matP.resize(p, 3);

    assert(_matP.size() == p*3);

	_matP.setZero();

    for (int i = 0; i < p; i++)
    {
        glm::vec3 Pi = _ctrlPoints.at(i);
        _matP.col(0)[i] = 1;
        _matP.col(1)[i] = Pi.x;
        _matP.col(2)[i] = Pi.y;
    }

    return true;
}


bool TPS::assembleMatrixL(Eigen::MatrixXd& _matL, std::vector<glm::vec3>& _ctrlPoints)
{
    size_t p = _ctrlPoints.size();

    if (p < 3)
    {
        std::cerr << "Number of control = " << p << ", must be  >= 3 " << std::endl;
        return false;
    }

    Eigen::MatrixXd matK, matP;

    if (buildSubmatrixK(matK, _ctrlPoints) && buildSubmatrixP(matP, _ctrlPoints))
    {

        assert(matK.rows() == p && matK.cols() == p);
        assert(matP.rows() == p && matP.cols() == 3);

        if (_matL.rows() != p + 3 || _matL.cols() != p + 3)
            _matL.resize(p + 3, p + 3);

        assert(_matL.size() == (p + 3) * (p + 3));

        _matL.setZero();

        Eigen::MatrixXd matPt = matP.transpose();

        for (int i = 0; i < p; i++)
        {
            for (int j = 0; j < p; j++)
            {
                _matL.col(i)[j] = matK.col(i)[j];
            }
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < p; j++)
            {
                _matL.col(p + i)[j] = matP.col(i)[j];
                _matL.row(p + i)[j] = matPt.row(i)[j];
            }
        }

        return true;
    }
    return false;
}


bool TPS::buildVectorV(Eigen::VectorXd& _vecV, std::vector<glm::vec3>& _ctrlPoints)
{
    size_t p = _ctrlPoints.size();

    if (p < 3)
    {
        std::cerr << "Number of control = " << p << ", must be  >= 3 " << std::endl;
        return false;
    }

    if(_vecV.rows() != p + 3)
        _vecV.resize(p + 3);

    assert(_vecV.size() == p + 3);

    _vecV.setZero();

    for (int i = 0; i < p; i++)
    {
        glm::vec3 Pi = _ctrlPoints.at(i);
        _vecV[i] = Pi.z;
    }

    return true;
}



glm::vec3 TPS::computePtUV(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints,
                              const float _u, const float _v)
{
    return glm::vec3(0.0f, 0.0f, 0.0f);
}


} // namespace CompGeom