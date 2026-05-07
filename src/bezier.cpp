/*********************************************************************************************************************
 *
 * bezier.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/



#include "bezier.h"


namespace CompGeom
{

int Bezier::fact(int _i)
{
    // i! = i * (i-1) * (i-2) * ... * 2 * 1

    // recursive version: i! = i * (i-1)!
    return _i == 0 ? 1 : _i * fact(_i-1); 
}


double Bezier::BernsteinCoeff(int _n, int _i, double _t)
{
    // Binomial coefficient C(n, i) = (n!) / (i! (n! - i!))
	double C = fact(_n) / (double)(fact(_i) * fact(_n-_i));

    // Compute Bernstein basis polynomials
    // B_i^n(t) = C(n, i) * t^i * (1-t)^(n-i)
	double B = C * pow(_t, _i) * pow((1.0 - _t), (_n - _i));

	return B;
}


glm::vec3 Bezier::computePtUV(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints,
                              const float _u, const float _v)
{
    // degree = nb ctrl pts - 1
    // Bicubic surface: degree n = m = 3 (i.e., 4 ctrl pts)
    const int nbCtrlPts = 4;
    const int degree = nbCtrlPts - 1;

    if (_ctrlPoints.size() != 4 || _ctrlPoints.front().size() != 4)
    {
        std::cerr << "Number of control point array must be 4x4 for  bicubic Bezier surface" << std::endl;
    }

    glm::vec3 surfacePoint(0.0f, 0.0f, 0.0f);
    
    for (int i = 0; i < nbCtrlPts; i++)
    {
        for (int j = 0; j < nbCtrlPts; j++)
        {
            double Bu = BernsteinCoeff(degree, i, _u);
            double Bv = BernsteinCoeff(degree, j, _v);
            float coeff = static_cast<float>(Bu * Bv);
            
            surfacePoint += _ctrlPoints.at(i).at(j) * coeff;
        }
    }
    return surfacePoint;
}


void Bezier::buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps)
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
    std::array<std::array<glm::vec3, nbCtrlPtsPerSide>, nbCtrlPtsPerSide> ctrlPoints;

    int idX = 0;
    int idY = 0;
    int cpt = 0;
    for (auto it = _ctrlPolygon.getVertices().begin(); it != _ctrlPolygon.getVertices().end(); ++it)
    {
        idY = (cpt / nbCtrlPtsPerSide);
        idX = cpt - (idY * nbCtrlPtsPerSide);

        ctrlPoints.at(idX).at(idY) = it->pos;

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
            glm::vec3 pos = computePtUV(ctrlPoints, u, v);

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


void Bezier::updateParametricSurface(Mesh& _ctrlPolygon)
{
    // We use bicubic surface (i.e., 4 x 4 control points)
    const int nbCtrlPtsPerSide = 4;

    float stepSize = 1.0f / static_cast<float>(m_nbSteps);

    // nb vertices on each side = nb intermediate steps + 1
    int nbVerticesPerSide = m_nbSteps + 1;
    int nbVertices = static_cast<int>(pow(nbVerticesPerSide, 2.0));

    assert(m_vertices.size() == nbVertices);

    // 1. Control points grid
    std::array<std::array<glm::vec3, nbCtrlPtsPerSide>, nbCtrlPtsPerSide> ctrlPoints;

    int idX = 0;
    int idY = 0;
    int cpt = 0;
    for (auto it = _ctrlPolygon.getVertices().begin(); it != _ctrlPolygon.getVertices().end(); ++it)
    {
        idY = (cpt / nbCtrlPtsPerSide);
        idX = cpt - (idY * nbCtrlPtsPerSide);

        ctrlPoints.at(idX).at(idY) = it->pos;

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
            glm::vec3 pos = computePtUV(ctrlPoints, u, v);

            // update vertex position
            if(id < nbVertices)
                m_vertices.at(id).pos = pos;

            id++;
        }
    }

}



} // namespace CompGeom