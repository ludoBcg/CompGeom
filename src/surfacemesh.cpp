/*********************************************************************************************************************
 *
 * surfacemesh.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/



#include "surfacemesh.h"
#include "vkcontext.h"

#include <iterator>
#include <algorithm>
#include <memory>


namespace CompGeom
{


int SurfaceMesh::fact(int _i)
{
    // i! = i * (i-1) * (i-2) * ... * 2 * 1

    // recursive version: i! = i * (i-1)!
    return _i == 0 ? 1 : _i * fact(_i-1); 
}


double SurfaceMesh::BernsteinCoeff(int _n, int _i, double _t)
{
    // Binomial coefficient C(n, i) = (n!) / (i! (n! - i!))
	double C = fact(_n) / (double)(fact(_i) * fact(_n-_i));

    // Compute Bernstein basis polynomials
    // B_i^n(t) = C(n, i) * t^i * (1-t)^(n-i)
	double B = C * pow(_t, _i) * pow((1.0 - _t), (_n - _i));

	return B;
}


glm::vec3 SurfaceMesh::computeBezierPt(glm::vec3 _ctrlPoints[4][4], float _u, float _v) 
{
    // degree = nb ctrl pts - 1
    // Bicubic surface: degree n = m = 3 (i.e., 4 ctrl pts)
    const int nbCtrlPts = 4;
    const int degree = nbCtrlPts - 1;

    glm::vec3 surfacePoint(0.0f, 0.0f, 0.0f);
    
    for (int i = 0; i < nbCtrlPts; i++)
    {
        for (int j = 0; j < nbCtrlPts; j++)
        {
            double Bu = BernsteinCoeff(degree, i, _u);
            double Bv = BernsteinCoeff(degree, j, _v);
            float coeff = static_cast<float>(Bu * Bv);
            
            surfacePoint.x += _ctrlPoints[i][j].x * coeff;
            surfacePoint.y += _ctrlPoints[i][j].y * coeff;
            surfacePoint.z += _ctrlPoints[i][j].z * coeff;
        }
    }
    return surfacePoint;
}


double SurfaceMesh::RiesenfeldCoeff(int _n, int _i, double _t)
{
    // Compute Riesenfeld polynomial
    // R_i^n(t) = (n+1) * sum_(k=0)^(n-1)( (-1)^k * (t+m-i-k)^n / (k!(m-k+1)!) )

    double sum = 0.0; 
	for(int k = 0; k <=_n-_i; k++)
		sum += pow(-1, k) * (pow(_t+_n-_i-k, _n)) / (double)(fact(k) * fact(_n-k+1));

	return (_n + 1) * sum;
}


// B-spline basis function
glm::vec3 SurfaceMesh::computeBsplinePt(glm::vec3 _ctrlPoints[4][4], float _u, float _v) 
{
    const int nbCtrlPts = 4;
    const int degree = nbCtrlPts - 1;

    glm::vec3 surfacePoint(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < nbCtrlPts; ++i)
    {
        double R1 = RiesenfeldCoeff(degree, i, _u);
        for (int j = 0; j < nbCtrlPts; ++j)
        {
            double R2 = RiesenfeldCoeff(degree, j, _v);
			float Ruv = static_cast<float>(R1 * R2);
                       
            surfacePoint.x += _ctrlPoints[i][j].x * Ruv;
            surfacePoint.y += _ctrlPoints[i][j].y * Ruv;
            surfacePoint.z += _ctrlPoints[i][j].z * Ruv;
        }
    }
    
    return surfacePoint;
}


void SurfaceMesh::buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps, eParametricSurface _paramSurface)
{
    m_nbSteps = _nbSteps;

    // We use bicubic surface (i.e., 4 x 4 control points)
    const int nbCtrlPtsPerSide = 4;

    float stepSize = 1.0f / static_cast<float>(m_nbSteps);

    // nb vertices on each side = nb intermediate steps + 1
    int nbVerticesPerSide = m_nbSteps + 1;
    int nbVertices = static_cast<int>(pow(nbVerticesPerSide, 2.0));
    m_vertices.clear();
    m_vertices.assign(nbVertices, Vertex{});

    // 1. Control points grid
    glm::vec3 ctrlPoints[nbCtrlPtsPerSide][nbCtrlPtsPerSide];

    int idX = 0;
    int idY = 0;
    int cpt = 0;
    for (auto it = _ctrlPolygon.getVertices().begin(); it != _ctrlPolygon.getVertices().end(); ++it)
    {
        idY = (cpt / nbCtrlPtsPerSide);
        idX = cpt - (idY * nbCtrlPtsPerSide);

        ctrlPoints[idX][idY] = it->pos;

        cpt++;
    }

    // 2. Build the parametric surface vertices
    int id = 0;
    float u, v = 0.0f;
    // for each intermediate coords (u,v)
    for (int i = 0; i < nbVerticesPerSide; i++)
    {
        u = std::min(1.0f, stepSize * i);

        for (int j = 0; j < nbVerticesPerSide; j++)
        {
            v = std::min(1.0f, stepSize * j);

            // calculate surface point coordinates, 
            // depending on the chosen parametric surface algorithm
            glm::vec3 pos(0.0f);
            if(_paramSurface == eParametricSurface::BEZIER)
                pos = computeBezierPt(ctrlPoints, u, v);
            else if(_paramSurface == eParametricSurface::BSPLINE)
                pos = computeBsplinePt(ctrlPoints, u, v);

            // add vertex to mesh
            Vertex vert{ pos /* pos */, {0.4f, 0.6f, 0.2f} /* col */, 
                        {1.0f, 1.0f} /* uv */, {0.0f, 0.0f, 1.0f} /* norm */ };

            if(id<nbVertices)
                m_vertices.at(id) = vert;

            id++;
        }
    }

    // 3. Triangulate the parametric surface vertices
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
            m_indices.push_back(cpt + 1);
            m_indices.push_back(cpt + nbVerticesPerSide);

            m_indices.push_back(cpt + 1);
            m_indices.push_back(cpt + 1 + nbVerticesPerSide);
            m_indices.push_back(cpt + nbVerticesPerSide);
        }

        cpt++;
    }

}


void SurfaceMesh::updateParametricSurface(Mesh& _ctrlPolygon, eParametricSurface _paramSurface)
{
    // We use bicubic surface (i.e., 4 x 4 control points)
    const int nbCtrlPtsPerSide = 4;

    float stepSize = 1.0f / static_cast<float>(m_nbSteps);

    // nb vertices on each side = nb intermediate steps + 1
    int nbVerticesPerSide = m_nbSteps + 1;
    int nbVertices = static_cast<int>(pow(nbVerticesPerSide, 2.0));

    assert(m_vertices.size() == nbVertices);

    // 1. Control points grid
    glm::vec3 ctrlPoints[nbCtrlPtsPerSide][nbCtrlPtsPerSide];

    int idX = 0;
    int idY = 0;
    int cpt = 0;
    for (auto it = _ctrlPolygon.getVertices().begin(); it != _ctrlPolygon.getVertices().end(); ++it)
    {
        idY = (cpt / nbCtrlPtsPerSide);
        idX = cpt - (idY * nbCtrlPtsPerSide);

        ctrlPoints[idX][idY] = it->pos;

        cpt++;
    }

    // 2. Build the parametric surface vertices
    int id = 0;
    float u, v = 0.0f;
    // for each intermediate coords (u,v)
    for (int i = 0; i < nbVerticesPerSide; i++)
    {
        u = std::min(1.0f, stepSize * i);

        for (int j = 0; j < nbVerticesPerSide; j++)
        {
            v = std::min(1.0f, stepSize * j);

            // calculate surface point coordinates, 
            // depending on the chosen parametric surface algorithm
            glm::vec3 pos(0.0f);
            if(_paramSurface == eParametricSurface::BEZIER)
                pos = computeBezierPt(ctrlPoints, u, v);
            else if(_paramSurface == eParametricSurface::BSPLINE)
                pos = computeBsplinePt(ctrlPoints, u, v);

            // update vertex position
            if(id < nbVertices)
                m_vertices.at(id).pos = pos;

            id++;
        }
    }

}



double SurfaceMesh::tpsBaseFunc(double _r)
{
    return _r == 0.0 ? 0.0 : _r * _r * log(_r);
}


bool SurfaceMesh::buildTPSsubmatrixK(Eigen::MatrixXd& _matK, std::vector<glm::vec3>& _ctrlPoints)
{
    size_t p = _ctrlPoints.size();

    if (p < 3)
    {
        std::cerr << "Number of control = " << p << ", must be  >= 3 " << std::endl;
        return false;
    }

    // the higher the lambda, the more rigid the surface
    double lambda = 0.0;

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
                _matK.col(i)[j] = tpsBaseFunc(normPiPj);

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


bool SurfaceMesh::buildTPSsubmatrixP(Eigen::MatrixXd& _matP, std::vector<glm::vec3>& _ctrlPoints)
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


bool SurfaceMesh::assembleTPSmatrixL(Eigen::MatrixXd& _matL, std::vector<glm::vec3>& _ctrlPoints)
{
    size_t p = _ctrlPoints.size();

    if (p < 3)
    {
        std::cerr << "Number of control = " << p << ", must be  >= 3 " << std::endl;
        return false;
    }

    Eigen::MatrixXd matK, matP;

    if (buildTPSsubmatrixK(matK, _ctrlPoints) && buildTPSsubmatrixP(matP, _ctrlPoints))
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


bool SurfaceMesh::buildTPSvectorV(Eigen::VectorXd& _vecV, std::vector<glm::vec3>& _ctrlPoints)
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

void SurfaceMesh::buildTPSsurface(Mesh& _ctrlPolygon, int _nbSteps)
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
    assembleTPSmatrixL(matL, ctrlPoints);
    buildTPSvectorV(vecV, ctrlPoints);
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
                h += vecX(i) * tpsBaseFunc( glm::length( pt_i - pt_cur ) );
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
            m_indices.push_back(cpt + 1);
            m_indices.push_back(cpt + nbVerticesPerSide);

            m_indices.push_back(cpt + 1);
            m_indices.push_back(cpt + 1 + nbVerticesPerSide);
            m_indices.push_back(cpt + nbVerticesPerSide);
        }

        cpt++;
    }

}

void SurfaceMesh::updateTPSsurface(Mesh& _ctrlPolygon)
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
    assembleTPSmatrixL(matL, ctrlPoints);
    buildTPSvectorV(vecV, ctrlPoints);
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
                h += vecX(i) * tpsBaseFunc( glm::length( pt_i - pt_cur ) );
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


} // namespace CompGeom