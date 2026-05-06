/*********************************************************************************************************************
 *
 * mesh.h
 *
 * Mesh class to store geometry and handle vertex and index buffers
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef MESH_H
#define MESH_H

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>


#include "vkutils.h"


namespace CompGeom
{

class VkContext;


/*
* Structure for vertex attributes
*/
struct Vertex 
{
    glm::vec3 pos;
    glm::vec3 color;
    glm::vec2 texCoord;
    glm::vec3 normal;


    bool operator==(const Vertex& _other) const 
    {
        return pos == _other.pos && color == _other.color && texCoord == _other.texCoord && normal == _other.normal;
    }

    static VkVertexInputBindingDescription getBindingDescription() 
    {
        VkVertexInputBindingDescription bindingDescription{};
        bindingDescription.binding = 0;
        bindingDescription.stride = sizeof(Vertex);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        // VK_VERTEX_INPUT_RATE_VERTEX = Move to the next data entry after each vertex
        // VK_VERTEX_INPUT_RATE_INSTANCE = Move to the next data entry after each instance

        return bindingDescription;
    }

    static std::array<VkVertexInputAttributeDescription, 4> getAttributeDescriptions() 
    {
        std::array<VkVertexInputAttributeDescription, 4> attributeDescriptions{};

        // Attribute description for position
        attributeDescriptions[0].binding = 0;
        attributeDescriptions[0].location = 0;
        attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[0].offset = offsetof(Vertex, pos);

        // Attribute description for color
        attributeDescriptions[1].binding = 0;
        attributeDescriptions[1].location = 1;
        attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[1].offset = offsetof(Vertex, color);

        // Attribute description for UVs
        attributeDescriptions[2].binding = 0;
        attributeDescriptions[2].location = 2;
        attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
        attributeDescriptions[2].offset = offsetof(Vertex, texCoord);

        // Attribute description for normals
        attributeDescriptions[3].binding = 0;
        attributeDescriptions[3].location = 3;
        attributeDescriptions[3].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[3].offset = offsetof(Vertex, normal);

        return attributeDescriptions;
    }

};


/*!
* \class Mesh
* \brief Mesh class to store geometry and handle vertex and index buffers
*/
class Mesh
{
    
public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn Mesh
    * \brief Default constructor
    */
    Mesh() = default;

    /*!
    * \fn Mesh
    * \brief Copy constructor
    */
    Mesh(Mesh const& _other) = default;

    /*!
    * \fn operator=
    * \brief Copy assignment operator
    */
    Mesh& operator=(Mesh const& _other)
    {
        m_vertices = _other.m_vertices;
        m_indices = _other.m_indices;
        m_vertexBuffer = _other.m_vertexBuffer;
        m_vertexBufferMemory = _other.m_vertexBufferMemory;
        m_indexBuffer = _other.m_indexBuffer;
        m_indexBufferMemory = _other.m_indexBufferMemory;
        return *this;
    }

    /*!
    * \fn Mesh
    * \brief Move constructor
    */
    Mesh(Mesh&& _other)
        : m_vertices(std::move(_other.m_vertices))
        , m_indices(std::move(_other.m_indices))
        , m_vertexBuffer(_other.m_vertexBuffer)
        , m_vertexBufferMemory(_other.m_vertexBufferMemory)
        , m_indexBuffer(_other.m_indexBuffer)
        , m_indexBufferMemory(_other.m_indexBufferMemory)
    {}

    /*!
    * \fn operator=
    * \brief Move assignment operator
    */
    Mesh& operator=(Mesh&& _other)
    {
        m_vertices = std::move(_other.m_vertices);
        m_indices = std::move(_other.m_indices);
        m_vertexBuffer = _other.m_vertexBuffer;
        m_vertexBufferMemory = _other.m_vertexBufferMemory;
        m_indexBuffer = _other.m_indexBuffer;
        m_indexBufferMemory = _other.m_indexBufferMemory;
        return *this;
    }

    /*!
    * \fn ~Mesh
    * \brief Desctructor
    */
    virtual ~Mesh() {};



    /*----------------------------------------------------------------------------------------------+
    |                                     GETTERS / SETTERS                                         |
    +-----------------------------------------------------------------------------------------------*/

    /*! \fn getVertices */
    std::vector<Vertex> const& getVertices() const { return m_vertices; }
    /*! \fn getIndices */
    std::vector<uint32_t> const& getIndices() const { return m_indices; }
    /*! \fn getVertexBuffer */
    VkBuffer const getVertexBuffer() const { return m_vertexBuffer; }
    /*! \fn getVertexBufferMemory */
    VkDeviceMemory const& getVertexBufferMemory() const { return m_vertexBufferMemory; }
    /*! \fn getIndexBuffer */
    VkBuffer const getIndexBuffer() const { return m_indexBuffer; }
    /*! \fn getIndexBufferMemory */
    VkDeviceMemory const getIndexBufferMemory() const { return m_indexBufferMemory; }



    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn createVertexBuffer
    * \brief Initializes vertex buffer
    * \param _context : Vulkan context
    */
    void createVertexBuffer(VkContext& _context);

    /*!
    * \fn updateVertexBuffer
    * \brief Updates vertex buffer with current data
    * \param _context : Vulkan context
    */
    void updateVertexBuffer(VkContext& _context);

    /*!
    * \fn createIndexBuffer
    * \brief Initializes index buffer
    * \param _context : Vulkan context
    */
    void createIndexBuffer(VkContext& _context);

    /*!
    * \fn cleanup
    * \brief Clears memory buffers
    * \param _context : Vulkan context to clean
    */
    void cleanup(VkContext& _context);



protected:

    std::vector<Vertex> m_vertices;         /*!< List of vertices */
    std::vector<uint32_t> m_indices;        /*!< List of indices */

    VkBuffer m_vertexBuffer;                /*!< Vertex buffer */
    VkDeviceMemory m_vertexBufferMemory;    /*!< Handle to the vertex buffer memory */

    VkBuffer m_indexBuffer;                 /*!< Index buffer */
    VkDeviceMemory m_indexBufferMemory;     /*!< Handle to the index buffer memory */
    


    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn id2Dto1D
    * \brief Converts 2D array indices to 1D array index
    * \param _i, _j : 2D indices
    * \param _nbVertI, _nbVertJ : dimensions of 2D array
    * \return : 1D index
    */
    unsigned int id2Dto1D(const unsigned int _i, const unsigned int _j,
                          const unsigned int _nbVertI, const unsigned int _nbVertJ) const;

    /*!
    * \fn createGrid
    * \brief Builds a squared regular grid mesh
    * \param _lengthSide : length of grid side
    * \param _nbVertPerSide : number of vertices per side
    */
    virtual void createGrid(const float _lengthSide, const unsigned int _nbVertPerSide);

    /*!
    * \fn updateNormals
    * \brief Update coordinates of normal vectors using current positions of vertices
    */
    void updateNormals();


}; // class Mesh

} // namespace CompGeom


//namespace std {
//    template<> struct hash<VulkanDemo::Vertex> {
//        size_t operator()(VulkanDemo::Vertex const& vertex) const {
//            return ((hash<glm::vec3>()(vertex.pos) ^
//                    (hash<glm::vec3>()(vertex.color) << 1)) >> 1) ^
//                    (hash<glm::vec2>()(vertex.texCoord) << 1);
//        }
//    };
//}
namespace std {
    template<> struct hash<CompGeom::Vertex> {
        size_t operator()(CompGeom::Vertex const& vertex) const 
        {
            std::size_t h1 = hash<glm::vec3>()(vertex.pos);
            std::size_t h2 = hash<glm::vec3>()(vertex.color);
            std::size_t h3 = hash<glm::vec2>()(vertex.texCoord);
            std::size_t h4 = hash<glm::vec3>()(vertex.normal);
            std::string stg = std::to_string(h1) + std::to_string(h2) + std::to_string(h3) + std::to_string(h4);
            return std::hash<std::string>()(stg);
        }
    };
}

#endif // MESH_H