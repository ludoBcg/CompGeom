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
                         {0.8f, 0.4f, 0.0f} /* col */, {1.0f, 1.0f} /* uv */, {0.0f, 0.0f, 1.0f} /* norm */ };
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
        }
    }

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