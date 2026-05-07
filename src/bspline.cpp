/*********************************************************************************************************************
 *
 * bspline.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/



#include "bspline.h"


namespace CompGeom
{

int Bspline::fact(int _i)
{
    // i! = i * (i-1) * (i-2) * ... * 2 * 1

    // recursive version: i! = i * (i-1)!
    return _i == 0 ? 1 : _i * fact(_i-1); 
}


int Bspline::findKnotSpan(int _nbCtrlPts, int _degree, double _t)
{
    // ...

    // fallback
    return _degree; 
}


glm::vec3 Bspline::deBoorT(int _degree, int _knotSpan, double _t,
                           const std::vector<double>& _knots,
                           std::vector<glm::vec3>& _newPts )
{
    //https://en.wikipedia.org/wiki/De_Boor%27s_algorithm

    for (int r = 1; r <= _degree; r++)
    {
        for (int j = r; j <= _degree; j++)
        {
            double denom = _knots.at(j + 1 + _knotSpan - r) - _knots.at(j + _knotSpan - _degree);
            double alpha = (denom == 0.0) ? 0.0 : (_t - _knots.at(_knotSpan - _degree + j)) / denom;
            // new value is linear interpolation between 2 neighbours old values
            _newPts.at(j) = (1.0f - static_cast<float>(alpha)) * _newPts.at(j - 1) + static_cast<float>(alpha) * _newPts.at(j);
        }
    }

    return _newPts.at(_degree);
}


glm::vec3 Bspline::deBoorUV(int _degree, int _knotSpan, double _u, double _v,
                            const std::vector<double>& _knots,
                            const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints)
{
    // Step 1: De Boor in u direction

    // store result
    std::vector<glm::vec3> resU(_degree + 1);

    // for each row in ctrl points grid
    for (int ctrlX = 0; ctrlX <= _degree; ctrlX++)
    {
        // Init list with control points of current row
        std::vector<glm::vec3> newPts_list(_degree + 1);
        for (int ctrlY = 0; ctrlY <= _degree; ctrlY++)
        {
            newPts_list.at(ctrlY) = _ctrlPoints.at(_knotSpan - _degree + ctrlY).at(_knotSpan - _degree + ctrlX);
        }

        // De Boor for t = u
        resU.at(ctrlX) = deBoorT(_degree, _knotSpan, _u, _knots, newPts_list );
    }


    // Step 2: De Boor in v direction

    // store final result
    // Init with result from first step
    std::vector<glm::vec3> resUV = resU;

    // De Boor for t = v
    return deBoorT(_degree, _knotSpan, _v, _knots, resUV );
}


void Bspline::builKnots(const int _degree)
{
    // Build clamped uniform knot vector
    m_knots.assign(8, 0.0);
    double cpt = 1.0;
    for (int i = 0; i < 8; ++i)
    {
        if (i <= _degree)
        {
            // duplicate first ctrl point for clamping
            m_knots.at(i) = 0.0;
        }
        else
        {
            if (i >= (8 - 1) - _degree)
            {
                // duplicate last ctrl point for clamping
                m_knots.at(i) = cpt;
            }
            else
            {
                // intermediate point (if any)
                m_knots.at(i) = cpt;
                cpt += 1.0;
            }
        }
    }
}


glm::vec3 Bspline::computePtUVDeBoor(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints, 
                                     const float _u, const float _v) 
{
    const int nbCtrlPts = 4; // = order
    const int degree = nbCtrlPts - 1;

    if (_ctrlPoints.size() != 4 || _ctrlPoints.front().size() != 4)
    {
        std::cerr << "Number of control point array must be 4x4 for  bicubic b-spline surface" << std::endl;
    }

    int k = findKnotSpan(nbCtrlPts, degree, _u);
    glm::vec3 surfacePoint = deBoorUV(degree, k, _u, _v, m_knots, _ctrlPoints);

    return surfacePoint;
}


double Bspline::RiesenfeldCoeff(int _n, int _i, double _t)
{
    // Compute Riesenfeld polynomial
    // R_i^n(t) = (n+1) * sum_(k=0)^(n-i)( (-1)^k * (t+n-i-k)^n / (k!(n-k+1)!) )

    double sum = 0.0; 
	for(int k = 0; k <=_n-_i; k++)
		sum += pow(-1, k) * (pow(_t+_n-_i-k, _n)) / (double)(fact(k) * fact(_n-k+1));

	return (_n + 1) * sum;
}


glm::vec3 Bspline::computePtUV(const std::array<std::array<glm::vec3, 4>, 4>& _ctrlPoints, 
                               const float _u, const float _v) 
{
    const int nbCtrlPts = 4;
    const int degree = nbCtrlPts - 1;

    if (_ctrlPoints.size() != 4 || _ctrlPoints.front().size() != 4)
    {
        std::cerr << "Number of control point array must be 4x4 for  bicubic b-spline surface" << std::endl;
    }

    glm::vec3 surfacePoint(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < nbCtrlPts; ++i)
    {
        double R1 = RiesenfeldCoeff(degree, i, _u);
        for (int j = 0; j < nbCtrlPts; ++j)
        {
            double R2 = RiesenfeldCoeff(degree, j, _v);
			float Ruv = static_cast<float>(R1 * R2);
                       
            surfacePoint += _ctrlPoints.at(i).at(j) * Ruv;
        }
    }
    
    return surfacePoint;
}


void Bspline::buildParametricSurface(Mesh& _ctrlPolygon, int _nbSteps)
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

    // 1.5 Knot vector for B-spline
    builKnots(nbCtrlPtsPerSide-1);

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
            glm::vec3 pos = computePtUVDeBoor(ctrlPoints, u, v);

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


void Bspline::updateParametricSurface(Mesh& _ctrlPolygon)
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
            // depending on the chosen algorithm
            glm::vec3 pos(0.0f);
            if(m_useDeBoor)
                pos = computePtUVDeBoor(ctrlPoints, u, v);
            else
                pos = computePtUV(ctrlPoints, u, v);

            // update vertex position
            if(id < nbVertices)
                m_vertices.at(id).pos = pos;

            id++;
        }
    }

}



} // namespace CompGeom