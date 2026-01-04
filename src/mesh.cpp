/*********************************************************************************************************************
 *
 * mesh.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/



#include "mesh.h"
#include "vkcontext.h"
#include "massspringsystem.h"
#include "arap.h"
#include "fem.h"

#include <iterator>
#include <algorithm>
#include <memory>


namespace CompGeom
{


/*
 * Destroyes buffers and frees memory 
 */
void Mesh::cleanup(VkContext& _context)
{
    vkDestroyBuffer(_context.getDevice(), m_indexBuffer, nullptr);
    vkFreeMemory(_context.getDevice(),m_indexBufferMemory, nullptr);

    vkDestroyBuffer(_context.getDevice(),m_vertexBuffer, nullptr);
    vkFreeMemory(_context.getDevice(), m_vertexBufferMemory, nullptr);
}


/*
 * Convert 2d Grid Cartesian coords into row-major 1D index
 */
unsigned int Mesh::id2Dto1D(const unsigned int _i, const unsigned int _j,
                            const unsigned int _nbVertI, const unsigned int _nbVertJ) const
{
    assert(_i < _nbVertI && _i < _nbVertJ);
    return (_i % _nbVertI + _j * _nbVertI);
}


/*
 * Builds a square-shaped, regular grid of triangulated vertices
 * Each side has a size of _lengthSide, discretized with _nbVertPerSide vertices
 * The grid is centered on model space origine point (0,0,0)
 */
void Mesh::createGrid(const float _lengthSide, const unsigned int _nbVertPerSide)
{
    m_vertices.clear();
    m_indices.clear();
    m_adjacency.clear();
    // initializes empty adjacency matrix
    m_adjacency = std::vector<std::vector<bool> >(_nbVertPerSide*_nbVertPerSide, std::vector<bool> (_nbVertPerSide*_nbVertPerSide, false));

    // Example of tesselation:
    // _lengthSide = 1.0, _nbVertPerSide=3
    //  + - + - +
    //  | \ | \ |
    //  + - + - +
    //  | \ | \ |
    //  + - + - +
    //  \__1.0__/  

    float spacing = _lengthSide / static_cast<float>(_nbVertPerSide - 1);

    // creates list of vertices, with row-major indexing
    //  0 - 1 - 2
    //  |   |   |
    //  3 - 4 - 5
    //  |   |   | 
    //  6 - 7 - 8
    for(unsigned int i=0; i<_nbVertPerSide; i++)
    {
        for(unsigned int j=0; j<_nbVertPerSide; j++)
        {
            Vertex vert{ { spacing * i - (_lengthSide * 0.5f),  spacing * j  - (_lengthSide * 0.5f), 0.0f} /* pos */, 
                         {1.0f, 1.0f, 1.0f} /* col */, {1.0f, 1.0f} /* uv */, {0.0f, 0.0f, 1.0f} /* norm */ };
            m_vertices.push_back(vert);
        }
    }

    // Vertex list --> Tesselation
    //       0 - 1     0 - 1
    //       |   | --> | \ |
    //       3 - 4     2 - 3 
    for(unsigned int i=1; i<_nbVertPerSide; i++)
    {
        for(unsigned int j=1; j<_nbVertPerSide; j++)
        {
            // For each square cell in the grid, get indices of the 4 vertices
            unsigned int id0 = id2Dto1D(i-1, j-1, _nbVertPerSide, _nbVertPerSide);
            unsigned int id1 = id2Dto1D(i, j-1, _nbVertPerSide, _nbVertPerSide);
            unsigned int id2 = id2Dto1D(i-1, j, _nbVertPerSide, _nbVertPerSide);
            unsigned int id3 = id2Dto1D(i, j, _nbVertPerSide, _nbVertPerSide);

            // Add indices to the list to define 2 triangles in clockwise order
            // First triangle (0,1,3)
            m_indices.push_back(id0);
            m_indices.push_back(id1); 
            m_indices.push_back(id3);
            // Second triangle (3,2,0)
            m_indices.push_back(id3);
            m_indices.push_back(id2);
            m_indices.push_back(id0);

            // Add corresponding edges in the adjacency matrix
            // Edge (0,1)
            m_adjacency.at(id0).at(id1) = m_adjacency.at(id1).at(id0) = true;
            // Edge (0,2) 
            m_adjacency.at(id0).at(id2) = m_adjacency.at(id2).at(id0) = true;
            // Edge (0,3)
            m_adjacency.at(id0).at(id3) = m_adjacency.at(id3).at(id0) = true;
            // Edge (1,3)
            m_adjacency.at(id1).at(id3) = m_adjacency.at(id3).at(id1) = true;
            // Edge (2,3)
            m_adjacency.at(id2).at(id3) = m_adjacency.at(id3).at(id2) = true;
        }
    }

    // Temporary definition of hard-coded boundary conditions
    m_fixedPointsIds = { 0, 4, 20, 24 };
    m_constraintPoints = { std::make_pair<uint32_t, glm::vec3>(12, glm::vec3(0.0, 0.0, 1.0) /*target pos*/) };
    m_constraintPointsFEM = { std::make_pair<uint32_t, glm::vec3>(12, glm::vec3(0.5, 0.5, 0.0) /*target pos*/) };
}


/*
 * Check if adjacency matrix is empty (i.e., contains only false) 
 */
bool Mesh::isAdjacencyEmpty() const
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
unsigned int Mesh::getVertexDegree(const unsigned int _id) const
{

    unsigned int cpt = 0;
    for (auto it = m_adjacency.at(_id).begin(); it != m_adjacency.at(_id).end(); ++it)
    {
        if(*it == true)
            cpt++;
    }
    return cpt;
}


/*
 * Builds a mass-spring system from the tesselation
 */
bool Mesh::buildMassSpringSystem(MassSpringSystem& _massSpringSystem)
{
    _massSpringSystem.clear();

    std::vector<std::pair<unsigned int, unsigned int> > vertIds;
    
    for (int i = 0; i < m_vertices.size(); i++)
    {
        _massSpringSystem.addPoint(m_vertices.at(i).pos, 1.0f, 0.1f);
    }

    for (int i = 0; i < m_indices.size(); i += 3)
    {
        unsigned int id0 = m_indices.at(i);
        unsigned int id1 = m_indices.at(i + 1);
        unsigned int id2 = m_indices.at(i + 2);

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id0, id1)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id1, id0)) == vertIds.end() )
        {
            _massSpringSystem.addSpring(id0, id1, 0.25f);
            vertIds.push_back(std::make_pair(id0, id1));
        }

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id1, id2)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id2, id1)) == vertIds.end() )
        {
            _massSpringSystem.addSpring(id1, id2, 0.25f);
            vertIds.push_back(std::make_pair(id1, id2));
        }

        if( std::find(vertIds.begin(), vertIds.end(), std::make_pair(id2, id0)) == vertIds.end()
         && std::find(vertIds.begin(), vertIds.end(), std::make_pair(id0, id2)) == vertIds.end() )
        {
            _massSpringSystem.addSpring(id2, id0, 0.25f);
            vertIds.push_back(std::make_pair(id2, id0));
        }
    }

    _massSpringSystem.addConstraints(m_fixedPointsIds, m_constraintPoints);

    return true;
}


/*
 * Updates the tesselation from mass-spring state
 */
bool Mesh::readMassSpringSystem(MassSpringSystem& _massSpringSystem)
{
    if (m_vertices.size() != _massSpringSystem.getPointsT().size())
    {
        std::cerr << "m_vertices.size() != _massSpringSystem.getPointsT().size() " << std::endl;
        return false;
    }

    assert(m_vertices.size() == _massSpringSystem.getPointsT().size()) ;

    for (int i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = _massSpringSystem.getPointsT().at(i).getPosition();
    }   
    
    return true;
}


/*
 * Builds a arap
 */
bool Mesh::buildARAP(Arap& _arap)
{
    std::vector<glm::vec3> verticesPos;
    for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
    {
        verticesPos.push_back(it->pos);
    }   


    std::vector<std::pair<uint32_t, glm::vec3> > anchors;
    for (size_t i = 0; i < m_fixedPointsIds.size(); ++i)
    {
        // add fixed points as anchors
        uint32_t id = m_fixedPointsIds.at(i);
        // use initial vertex coords as target position
        glm::vec3 position = verticesPos.at(m_fixedPointsIds.at(i));
        anchors.push_back(std::make_pair(id, position));
    }
    for (size_t i = 0; i < m_constraintPoints.size(); ++i)
    {
        // add constraint points as anchors
        uint32_t id = m_constraintPoints.at(i).first;
        glm::vec3 position = m_constraintPoints.at(i).second;
        anchors.push_back(std::make_pair(id, position));
    }

    _arap.initialize(verticesPos, m_adjacency, anchors, 50.0);

    verticesPos.clear();

    return true;
}


/*
 * Updates the tesselation from arap state
 */
bool Mesh::readARAP(Arap& _arap)
{
    std::vector<glm::vec3> newPos;
    _arap.getResult(newPos);

    assert(m_vertices.size() == newPos.size()) ;

    if (m_vertices.size() != newPos.size())
    {
        std::cerr << "m_vertices.size() != _arap.getResult().size() " << std::endl;
        return false;
    }

    for (size_t i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = newPos.at(i);
    }   

    return true;
}


bool Mesh::buildFEM(Fem& _fem)
{
    std::vector<glm::vec3> verticesPos;
    for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
    {
        verticesPos.push_back(it->pos);
    }   

    _fem.initialize(verticesPos, m_indices, 10.5, 0.5);

    verticesPos.clear();

    _fem.addConstraints(m_fixedPointsIds, m_constraintPointsFEM);
    //_fem.solve();

    return true;
}


bool Mesh::readFEM(Fem& _fem)
{
    std::vector<glm::vec3> newPos;
    _fem.getResult(newPos);

    assert(m_vertices.size() == newPos.size()) ;

    if (m_vertices.size() != newPos.size())
    {
        std::cerr << "m_vertices.size() != _fem.getResult().size() " << std::endl;
        return false;
    }

    for (int i = 0; i < m_vertices.size(); i++)
    {
        m_vertices.at(i).pos = newPos.at(i);
    }

    return true;
}


/*
 * Creation of vertex buffer
 */
void Mesh::createVertexBuffer(VkContext& _context)
{
    VkDeviceSize bufferSize = sizeof(m_vertices[0]) * m_vertices.size();

    // Init temporary CPU buffer (stagingBuffer) with associated memory storage (stagingBufferMemory)
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBuffer( _context.getPhysicalDevice(), _context.getDevice(), bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 stagingBuffer, stagingBufferMemory);

    // map memory buffer (data) with stagingBufferMemory
    void* data;
    vkMapMemory(_context.getDevice(), stagingBufferMemory, 0, bufferSize, 0, &data);
    // fill-in data  with m_vertices content
    memcpy(data, m_vertices.data(), (size_t)bufferSize);
    // unmap, now that stagingBufferMemory contains m_vertices data
    vkUnmapMemory(_context.getDevice(), stagingBufferMemory);

    // Init actual vertex buffer (m_vertexBuffer) with associated memory storage (m_vertexBufferMemory)
    createBuffer( _context.getPhysicalDevice(), _context.getDevice(), bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                 m_vertexBuffer, m_vertexBufferMemory);
    // data is copied from stagingBuffer to m_vertexBuffer
    copyBuffer(_context.getDevice(), _context.getCommandPool(), _context.getGraphicsQueue(), stagingBuffer, m_vertexBuffer, bufferSize);

    // cleanup temporary data after copy
    vkDestroyBuffer(_context.getDevice(), stagingBuffer, nullptr);
    vkFreeMemory(_context.getDevice(), stagingBufferMemory, nullptr);
}


void Mesh::updateVertexBuffer(VkContext& _context)
{
    VkDeviceSize bufferSize = sizeof(m_vertices[0]) * m_vertices.size();

    // Init temporary CPU buffer (stagingBuffer) with associated memory storage (stagingBufferMemory)
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBuffer( _context.getPhysicalDevice(), _context.getDevice(), bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 stagingBuffer, stagingBufferMemory);

    // map memory buffer (data) with stagingBufferMemory
    void* data;
    vkMapMemory(_context.getDevice(), stagingBufferMemory, 0, bufferSize, 0, &data);
    // fill-in data  with m_vertices content
    memcpy(data, m_vertices.data(), (size_t)bufferSize);
    // unmap, now that stagingBufferMemory contains m_vertices data
    vkUnmapMemory(_context.getDevice(), stagingBufferMemory);

    // Init actual vertex buffer (m_vertexBuffer) with associated memory storage (m_vertexBufferMemory)
    //createBuffer( _context.getPhysicalDevice(), _context.getDevice(), bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
    //             m_vertexBuffer, m_vertexBufferMemory);
    // data is copied from stagingBuffer to m_vertexBuffer
    copyBuffer(_context.getDevice(), _context.getCommandPool(), _context.getGraphicsQueue(), stagingBuffer, m_vertexBuffer, bufferSize);

    // cleanup temporary data after copy
    vkDestroyBuffer(_context.getDevice(), stagingBuffer, nullptr);
    vkFreeMemory(_context.getDevice(), stagingBufferMemory, nullptr);
}


/*
 * Creation of index buffer
 */
void Mesh::createIndexBuffer(VkContext& _context)
{
    VkDeviceSize bufferSize = sizeof(m_indices[0]) * m_indices.size();

    // temporary CPU buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBuffer( _context.getPhysicalDevice(), _context.getDevice(), bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 stagingBuffer, stagingBufferMemory);

    void* data;
    vkMapMemory(_context.getDevice(), stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, m_indices.data(), (size_t)bufferSize);
    vkUnmapMemory(_context.getDevice(), stagingBufferMemory);

    // actual index buffer
    createBuffer( _context.getPhysicalDevice(), _context.getDevice(), bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                 m_indexBuffer, m_indexBufferMemory);
    // data is copied from staging buffer
    copyBuffer(_context.getDevice(), _context.getCommandPool(), _context.getGraphicsQueue(), stagingBuffer, m_indexBuffer, bufferSize);

    // cleanup data after copy
    vkDestroyBuffer(_context.getDevice(), stagingBuffer, nullptr);
    vkFreeMemory(_context.getDevice(), stagingBufferMemory, nullptr);
}

} // namespace CompGeom