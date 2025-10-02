/*********************************************************************************************************************
 *
 * image.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#define NOMINMAX 
#include <cstdint> // Necessary for uint32_t
#include <limits> // Necessary for std::numeric_limits
#include <algorithm> // Necessary for std::clamp

#include "image.h"
#include "vkcontext.h"



namespace CompGeom
{


/*
 * Destroyes buffers and frees memory
 */
void Image::cleanup(VkContext& _context)
{
    if(m_sampler != nullptr)
        vkDestroySampler(_context.getDevice(), m_sampler, nullptr);
    vkDestroyImageView(_context.getDevice(), m_imageView, nullptr);
    vkDestroyImage(_context.getDevice(), m_image, nullptr);
    vkFreeMemory(_context.getDevice(), m_imageMemory, nullptr);
}


/*
 * Image object creation and memory allocation
 */
void Image::createImage(VkContext& _context,
                        uint32_t _width, uint32_t _height,
                        VkSampleCountFlagBits _numSamples, VkFormat _format,
                        VkImageTiling _tiling, VkImageUsageFlags _usage, VkMemoryPropertyFlags _properties)
{
    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = _width;
    imageInfo.extent.height = _height;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = m_mipLevels;
    imageInfo.arrayLayers = 1;
    imageInfo.format = _format;
    imageInfo.tiling = _tiling;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = _usage;
    imageInfo.samples = _numSamples; //VK_SAMPLE_COUNT_1_BIT
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageInfo.flags = 0; // Optional

    if (vkCreateImage(_context.getDevice(), &imageInfo, nullptr, &m_image) != VK_SUCCESS) {
        throw std::runtime_error("failed to create image!");
    }

    VkMemoryRequirements memRequirements;
    vkGetImageMemoryRequirements(_context.getDevice(), m_image, &memRequirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = findMemoryType(_context.getPhysicalDevice(), memRequirements.memoryTypeBits, _properties);

    if (vkAllocateMemory(_context.getDevice(), &allocInfo, nullptr, &m_imageMemory) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate image memory!");
    }

    vkBindImageMemory(_context.getDevice(), m_image, m_imageMemory, 0);
}


/*
 * Creation of one image view
 */
void Image::createImageView(VkContext& _context, VkFormat _format, VkImageAspectFlags _aspectFlags)
{
    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = m_image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = _format;
    viewInfo.subresourceRange.aspectMask = _aspectFlags;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = m_mipLevels;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(_context.getDevice(), &viewInfo, nullptr, &m_imageView) != VK_SUCCESS) {
        throw std::runtime_error("failed to create texture image view!");
    }

}


/*
 * Handles layout transitions
 */
void Image::transitionImageLayout(VkContext& _context, 
                                  VkFormat _format, VkImageLayout _oldLayout, VkImageLayout _newLayout)
{
    VkCommandBuffer commandBuffer = beginSingleTimeCommands(_context.getDevice(), _context.getCommandPool());

    VkPipelineStageFlags sourceStage;
    VkPipelineStageFlags destinationStage;

    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = _oldLayout;
    barrier.newLayout = _newLayout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = m_image;

    if (_newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

        if (hasStencilComponent(_format)) {
            barrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
        }
    }
    else {
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    }

    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = m_mipLevels;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;

    if (_oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && _newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
    {
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

        sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    }
    else if (_oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && _newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
    {
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    }
    else if (_oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && _newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
    {
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

        sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        destinationStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    }
    else {
        throw std::invalid_argument("unsupported layout transition!");
    }


    vkCmdPipelineBarrier(
        commandBuffer,
        sourceStage, destinationStage,
        0,
        0, nullptr,
        0, nullptr,
        1, &barrier
    );

    endSingleTimeCommands(_context.getDevice(), commandBuffer, _context.getCommandPool(), _context.getGraphicsQueue());
}


} // namespace CompGeom