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
            float Bu = BernsteinCoeff(degree, i, _u);
            float Bv = BernsteinCoeff(degree, j, _v);
            float coeff = Bu * Bv;
            
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
			float Ruv = R1 * R2;
                       
            surfacePoint.x += _ctrlPoints[i][j].x * Ruv;
            surfacePoint.y += _ctrlPoints[i][j].y * Ruv;
            surfacePoint.z += _ctrlPoints[i][j].z * Ruv;
        }
    }
    
    return surfacePoint;
}


void SurfaceMesh::buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps, eParametricSurface _paramSurface)
{
    // We use bicubic surface (i.e., 4 x 4 control points)
    const int nbCtrlPtsPerSide = 4;

    float stepSize = 1.0 / static_cast<float>(_nbSteps);

    // nb vertices on each side = nb intermediate steps + 1
    int nbVerticesPerSide = _nbSteps + 1;
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
        int id0 = cpt % nbVerticesPerSide;
        int id1 = id0 + 1;
        int id2 = cpt / nbVerticesPerSide;

        if (id0 < _nbSteps && id2 < _nbSteps)
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


void SurfaceMesh::updateParametricSurface(Mesh& _ctrlPolygon, int _nbSteps, eParametricSurface _paramSurface)
{
    // We use bicubic surface (i.e., 4 x 4 control points)
    const int nbCtrlPtsPerSide = 4;

    float stepSize = 1.0 / static_cast<float>(_nbSteps);

    // nb vertices on each side = nb intermediate steps + 1
    int nbVerticesPerSide = _nbSteps + 1;
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


} // namespace CompGeom