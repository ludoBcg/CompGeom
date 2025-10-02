/*********************************************************************************************************************
 *
 * vkcontext.h
 *
 * Class representation of a Vulkan context
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef VKCONTEXT_H
#define VKCONTEXT_H


#include "vkutils.h"

namespace CompGeom
{


class VkContext
{
    

public:

    VkContext() = default;

    VkContext(VkContext const& _other) = default;

    VkContext(VkInstance _instance,  VkDebugUtilsMessengerEXT _debugMessenger,
           VkPhysicalDevice _physicalDevice, VkDevice _device, VkCommandPool _commandPool,
           VkQueue _graphicsQueue, VkQueue _presentQueue, VkSurfaceKHR _surface)
        : m_instance(_instance)
        , m_debugMessenger(_debugMessenger)
        , m_physicalDevice(_physicalDevice)
        , m_device(_device)                  
        , m_graphicsQueue(_graphicsQueue)
        , m_presentQueue(_presentQueue)
        , m_commandPool(_commandPool)
        , m_surface(_surface)
    {}

    VkContext& operator=(VkContext const& _other)
    {
        m_instance = _other.m_instance;
        m_debugMessenger = _other.m_debugMessenger;
        m_physicalDevice = _other.m_physicalDevice;
        m_device = _other.m_device;                                 
        m_graphicsQueue = _other.m_graphicsQueue;
        m_presentQueue = _other.m_presentQueue;
        m_commandPool = _other.m_commandPool;
        m_surface = _other.m_surface;
        return *this;
    }

    VkContext(VkContext&& _other)
        : m_instance(_other.m_instance)
        , m_debugMessenger(_other.m_debugMessenger)
        , m_physicalDevice(_other.m_physicalDevice)
        , m_device(_other.m_device)                  
        , m_graphicsQueue(_other.m_graphicsQueue)
        , m_presentQueue(_other.m_presentQueue)
        , m_commandPool(_other.m_commandPool)
        , m_surface(_other.m_surface)
    {}

    VkContext& operator=(VkContext&& _other)
    {
        m_instance = _other.m_instance;
        m_debugMessenger = _other.m_debugMessenger;
        m_physicalDevice = _other.m_physicalDevice;
        m_device = _other.m_device;                                 
        m_graphicsQueue = _other.m_graphicsQueue;
        m_presentQueue = _other.m_presentQueue;
        m_commandPool = _other.m_commandPool;
        m_surface = _other.m_surface;
        return *this;
    }

    virtual ~VkContext() {};


    VkInstance const& getInstance() const { return m_instance; }
    VkDebugUtilsMessengerEXT const& getDebugMessenger() const { return m_debugMessenger; }
    VkPhysicalDevice const& getPhysicalDevice() const { return m_physicalDevice; }
    VkDevice const& getDevice() const { return m_device; }
    VkQueue const& getGraphicsQueue() const { return m_graphicsQueue; }
    VkQueue const& getPresentQueue() const { return m_presentQueue; }
    VkCommandPool const& getCommandPool() const { return m_commandPool; }
    VkSurfaceKHR const& getSurface() const { return m_surface; }


    void createInstance();
    void setupDebugMessenger();
    void setPhysicalDevice(VkPhysicalDevice _physicalDevice) { m_physicalDevice = _physicalDevice; }
    void createLogicalDevice();
    void createCommandPool();
    void createSurface(GLFWwindow* _window);

    void pickPhysicalDevice(VkSampleCountFlagBits& _msaaSamples);
    VkSampleCountFlagBits getMaxUsableSampleCount();

protected:

    VkInstance m_instance;
    VkDebugUtilsMessengerEXT m_debugMessenger;          // debug callback
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE; // the graphic device
    VkDevice m_device;                                  // logical device handle (i.e., similar to OpenGL context)
    VkQueue m_graphicsQueue;                            // graphics queue handle
    VkQueue m_presentQueue;                             // presentation queue handle
    VkCommandPool m_commandPool;                        // command pool handle
    VkSurfaceKHR m_surface;                             // abstract type of surface to present rendered images to

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT _messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT _messageType,
        const VkDebugUtilsMessengerCallbackDataEXT* _pCallbackData,
        void* _pUserData);
    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& _createInfo);
    

}; // class VkContext

} // namespace CompGeom

#endif // VKCONTEXT_H