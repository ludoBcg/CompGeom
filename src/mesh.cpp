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
#include "spring.h"
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
 * Creates a quad
 */
void Mesh::createQuad()
{
    m_vertices.clear();
    m_indices.clear();

    // list of vertices for 2 quads, made of 2 triangles each
    // 2 -- 3
    // | \  |
    // |  \ |
    // 1 -- 0 
    m_vertices = {
        { { 0.5f,  0.5f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f} },
        { {-0.5f,  0.5f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f} },
        { {-0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f} },
        { { 0.5f, -0.5f, 0.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 0.0f} },
    };

    // list of indices
    m_indices = {
        0, 1, 2, 2, 3, 0,
    };
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