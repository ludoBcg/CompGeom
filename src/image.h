/*********************************************************************************************************************
 *
 * image.h
 *
 * Image class to store 2D images
 * Used to manage textures, depth buffer, color buffer for multisampling ...
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef IMAGE_H
#define IMAGE_H


#include "vkutils.h"

namespace CompGeom
{

class VkContext;

class Image
{
    

public:

    Image() = default;

    Image(Image const& _other) = default;

    Image& operator=(Image const& _other)
    {
        m_image = _other.m_image;
        m_imageMemory = _other.m_imageMemory;
        m_imageView = _other.m_imageView;
        m_mipLevels = _other.m_mipLevels;
        m_sampler = _other.m_sampler;
        return *this;
    }

    Image(Image&& _other)
        : m_image(_other.m_image)
        , m_imageMemory(_other.m_imageMemory)
        , m_imageView(_other.m_imageView)
        , m_mipLevels(_other.m_mipLevels)
        , m_sampler(_other.m_sampler)
    {}

    Image& operator=(Image&& _other)
    {
        m_image = _other.m_image;
        m_imageMemory = _other.m_imageMemory;
        m_imageView = _other.m_imageView;
        m_mipLevels = _other.m_mipLevels;
        m_sampler = _other.m_sampler;
        return *this;
    }

    virtual ~Image() {};


    VkImage const getImage() const { return m_image; }
    VkDeviceMemory const getImageMemory() const { return m_imageMemory; }
    VkImageView getImageView() { return m_imageView; }
    uint32_t const getMiplevels() const { return m_mipLevels; }
    VkSampler const getSampler() const { return m_sampler; }

    void cleanup(VkContext& _context);

    void createImageView(VkContext& _context, VkFormat _format, VkImageAspectFlags _aspectFlags);


    void createImage(VkContext& _context,
                     uint32_t _width, uint32_t _height,
                     VkSampleCountFlagBits _numSamples, VkFormat _format,
                     VkImageTiling _tiling, VkImageUsageFlags _usage, VkMemoryPropertyFlags _properties);
    void transitionImageLayout(VkContext& _context,
                               VkFormat _format, VkImageLayout _oldLayout, VkImageLayout _newLayout);


protected:

    VkImage m_image;
    VkDeviceMemory m_imageMemory;
    VkImageView m_imageView;
    uint32_t m_mipLevels = 1;
    VkSampler m_sampler = nullptr;

}; // class Image

} // namespace CompGeom

#endif // IMAGE_H