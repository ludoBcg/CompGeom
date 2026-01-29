/*********************************************************************************************************************
 *
 * vkapp.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/


#include <algorithm> // Necessary for std::clamp
#include <chrono>
#include <unordered_map>

#include "vkapp.h"


namespace CompGeom
{


/*
 * Main app execution
 */
void VkApp::run()
{
    initWindow();
    initVulkan();
    initUBO();
    mainLoop();
    cleanup();
}


/*
 * Creates a GLFW window
 */
void VkApp::initWindow()
{
    glfwInit();

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    //glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    // setup glfw window
    m_window = glfwCreateWindow(WIDTH, HEIGHT, "CompGeom", nullptr, nullptr);
    glfwSetWindowUserPointer(m_window, this);
    glfwSetFramebufferSizeCallback(m_window, framebufferResizeCallback);
    glfwSetKeyCallback(m_window, keyCallback);
    glfwSetMouseButtonCallback(m_window, mouseButtonCallback);
    glfwSetScrollCallback(m_window, scrollCallback);
    glfwSetCursorPosCallback(m_window, cursorPosCallback);

    infoLog() << "initWindow(): OK ";
}

/*
 * Initializes mesh geometry and animation model 
 */
void VkApp::initGeomModel()
{
    // build grid geometry
    m_dynMesh.createGrid(1.5f, 4);
    m_surfMesh.buildParametricSurface(m_dynMesh, 18, eParametricSurface::BEZIER);
    m_surfMesh.createVertexBuffer(*m_contextPtr);
    m_surfMesh.createIndexBuffer(*m_contextPtr);

    m_dynMesh.createVertexBuffer(*m_contextPtr);
    m_dynMesh.createIndexBuffer(*m_contextPtr);

    if (ANIMATION_MODEL != eAnimationModels::ARAP && ANIMATION_MODEL != eAnimationModels::FEM)
    {
        m_dynMesh.buildMassSpringSystem(m_massSpringSystem);
    }

    switch (ANIMATION_MODEL)
    {
        case eAnimationModels::MS_FWE:
        {
            m_massSpringSystem.setNumIntegMethod(eNumIntegMethods::FORWARD_EULER);
            break;
        }
        case eAnimationModels::MS_SE:
        {
            m_massSpringSystem.setNumIntegMethod(eNumIntegMethods::SYMPLECTIC_EULER);
            break;
        }
        case eAnimationModels::MS_BWE:
        {
            m_massSpringSystem.setNumIntegMethod(eNumIntegMethods::BACKWARD_EULER);
            break;
        }
        case eAnimationModels::MS_LF:
        {
            m_massSpringSystem.setNumIntegMethod(eNumIntegMethods::LEAPFROG);
            break;
        }
        case eAnimationModels::MS_MID:
        {
            m_massSpringSystem.setNumIntegMethod(eNumIntegMethods::MIDPOINT);
            break;
        }
        case eAnimationModels::MS_VER:
        {
            m_massSpringSystem.setNumIntegMethod(eNumIntegMethods::VERLET);
            break;
        }
        case eAnimationModels::MS_RK4:
        {
            m_massSpringSystem.setNumIntegMethod(eNumIntegMethods::RK4);
            break;
        }
        case eAnimationModels::ARAP:
        {
            m_dynMesh.buildARAP(m_arap);
            m_dynMesh.readARAP(m_arap);
            m_dynMesh.updateVertexBuffer(*m_contextPtr);

            break;
        }
        case eAnimationModels::FEM:
        {
            m_dynMesh.buildFEM(m_fem);
            m_dynMesh.readFEM(m_fem);
            m_dynMesh.updateVertexBuffer(*m_contextPtr);

            break;
        }
    }

}

/*
 * Initializes Vulkan 
 */
void VkApp::initVulkan()
{
    m_contextPtr = std::make_shared<VkContext>();

    m_contextPtr->createInstance();
    m_contextPtr->setupDebugMessenger();
    m_contextPtr->createSurface(m_window);
    m_contextPtr->pickPhysicalDevice(m_msaaSamples);
    m_contextPtr->createLogicalDevice();
    createSwapChain();
    createImageViews();
    createRenderPass();
    createDescriptorSetLayout();
    createGraphicsPipeline(); 
    m_contextPtr->createCommandPool();
    createColorResources();
    createDepthResources();
    createFramebuffers();

    initGeomModel();

    createUniformBuffers();
    createDescriptorPool();
    createDescriptorSets();
    createCommandBuffers();
    createSyncObjects();

    infoLog() << "initVulkan(): OK ";
}

/*
 * Initializes transformation Matrices 
 */
void VkApp::initUBO()
{
    m_camera.init(0.01f, 8.0f, 45.0f, 1.0f, m_swapChainExtent.width, m_swapChainExtent.height, glm::vec3(0.0f, 2.0f, 3.0f), glm::vec3(0.0f, 0.0f, 0.0f), 0); 
    m_trackball.init(m_swapChainExtent.width, m_swapChainExtent.height);

    // initial transformation to re-orient mesh
    m_initModel = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f))
                * glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    // build MVP matrices
    m_ubo.model = m_initModel;
    m_ubo.view = m_camera.getViewMatrix();
    m_ubo.proj = m_camera.getProjectionMatrix();
    m_ubo.proj[1][1] *= -1;
    m_ubo.lightPos = glm::vec3(2.0f, 2.0f, 0.0f); // light source position in view space
}


/*
 * Executes main loop until app closed
 */
void VkApp::mainLoop()
{
    infoLog() << "enter main loop ";
    while (!glfwWindowShouldClose(m_window))
    {
        glfwPollEvents();

        updateGeom();

        drawFrame();
    }

    vkDeviceWaitIdle(m_contextPtr->getDevice());

    infoLog() << "exit main loop ";
}

/*
 * Cleanup before closing
 */
void VkApp::cleanup()
{
    cleanupSwapChain();

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) 
    {
        vkDestroyBuffer(m_contextPtr->getDevice(), m_uniformBuffers[i], nullptr);
        vkFreeMemory(m_contextPtr->getDevice(), m_uniformBuffersMemory[i], nullptr);
    }

    vkDestroyDescriptorPool(m_contextPtr->getDevice(), m_descriptorPool, nullptr);
    vkDestroyDescriptorSetLayout(m_contextPtr->getDevice(), m_descriptorSetLayout, nullptr);

    m_dynMesh.cleanup(*m_contextPtr);
    m_surfMesh.cleanup(*m_contextPtr);

    vkDestroyPipeline(m_contextPtr->getDevice(), m_graphicsPipelineOffscreen, nullptr);
    vkDestroyPipeline(m_contextPtr->getDevice(), m_graphicsPipeline, nullptr);
    vkDestroyPipelineLayout(m_contextPtr->getDevice(), m_pipelineLayoutOffscreen, nullptr);
    vkDestroyPipelineLayout(m_contextPtr->getDevice(), m_pipelineLayout, nullptr);
    

    vkDestroyRenderPass(m_contextPtr->getDevice(), m_renderPass, nullptr);

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) 
    {
        vkDestroySemaphore(m_contextPtr->getDevice(), m_imageAvailableSemaphores[i], nullptr);
        vkDestroySemaphore(m_contextPtr->getDevice(), m_renderFinishedSemaphores[i], nullptr);
        vkDestroyFence(m_contextPtr->getDevice(), m_inFlightFences[i], nullptr);
    }

    // Command buffers are automatically freed when their command pool is destroyed
    vkDestroyCommandPool(m_contextPtr->getDevice(), m_contextPtr->getCommandPool(), nullptr);

    vkDestroyDevice(m_contextPtr->getDevice(), nullptr);

    if (enableValidationLayers) 
    {
        DestroyDebugUtilsMessengerEXT(m_contextPtr->getInstance(), m_contextPtr->getDebugMessenger(), nullptr);
    }

    vkDestroySurfaceKHR(m_contextPtr->getInstance(), m_contextPtr->getSurface(), nullptr);

    vkDestroyInstance(m_contextPtr->getInstance(), nullptr);

    glfwDestroyWindow(m_window);

    glfwTerminate();

    infoLog() << "cleanup(): OK ";
}


/*
 * Creation of swap chain
 */
void VkApp::createSwapChain()
{
    SwapChainSupportDetails swapChainSupport = querySwapChainSupport(m_contextPtr->getPhysicalDevice(), m_contextPtr->getSurface());

    VkSurfaceFormatKHR surfaceFormat = chooseSwapSurfaceFormat(swapChainSupport.formats);
    VkPresentModeKHR presentMode = chooseSwapPresentMode(swapChainSupport.presentModes);
    VkExtent2D extent = chooseSwapExtent(swapChainSupport.capabilities, m_window);

    uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
    if (swapChainSupport.capabilities.maxImageCount > 0 && imageCount > swapChainSupport.capabilities.maxImageCount) 
    {
        imageCount = swapChainSupport.capabilities.maxImageCount;
    }

    VkSwapchainCreateInfoKHR createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = m_contextPtr->getSurface();
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = extent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    QueueFamilyIndices indices = findQueueFamilies(m_contextPtr->getPhysicalDevice(), m_contextPtr->getSurface());
    uint32_t queueFamilyIndices[] = { indices.graphicsFamily.value(), indices.presentFamily.value() };

    if (indices.graphicsFamily != indices.presentFamily) 
    {
        createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        createInfo.queueFamilyIndexCount = 2;
        createInfo.pQueueFamilyIndices = queueFamilyIndices;
    }
    else 
    {
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        createInfo.queueFamilyIndexCount = 0; // Optional
        createInfo.pQueueFamilyIndices = nullptr; // Optional
    }

    createInfo.preTransform = swapChainSupport.capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = presentMode;
    createInfo.clipped = VK_TRUE;

    createInfo.oldSwapchain = VK_NULL_HANDLE;

    if (vkCreateSwapchainKHR(m_contextPtr->getDevice(), &createInfo, nullptr, &m_swapChain) != VK_SUCCESS) 
    {
        throw std::runtime_error("failed to create swap chain!");
    }

    vkGetSwapchainImagesKHR(m_contextPtr->getDevice(), m_swapChain, &imageCount, nullptr);
    m_swapChainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(m_contextPtr->getDevice(), m_swapChain, &imageCount, m_swapChainImages.data());

    m_swapChainImageFormat = surfaceFormat.format;
    m_swapChainExtent = extent;

    initUBO(); // re-init camera and trackball when resize occurs

    infoLog() << "createSwapChain(): OK ";
}


/*
 * Creation of one image view
 */
VkImageView VkApp::createImageView(VkImage _image, VkFormat _format, VkImageAspectFlags _aspectFlags, uint32_t _mipLevels)
{
    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = _image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = _format;
    viewInfo.subresourceRange.aspectMask = _aspectFlags;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = _mipLevels;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    VkImageView imageView;
    if (vkCreateImageView(m_contextPtr->getDevice(), &viewInfo, nullptr, &imageView) != VK_SUCCESS) {
        throw std::runtime_error("failed to create texture image view!");
    }

    return imageView;
}


/*
 * Creation of image views
 */
void VkApp::createImageViews() 
{
    // creates as many image views as we have images
    m_swapChainImageViews.resize(m_swapChainImages.size());

    for (uint32_t i = 0; i < m_swapChainImages.size(); i++) 
    {
        m_swapChainImageViews[i] = createImageView(m_swapChainImages[i], m_swapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT, 1);
    }

    infoLog() << "createImageViews(): OK ";
}


/*
 * Creation of render pass 
 */
void VkApp::createRenderPass()
{
    // 1. Define attachments ------------------------------------------------------
    std::array<VkAttachmentDescription, 4> attachments{};
    // Color attachment for offscreen rendering (ID=0)
    attachments.at(0).format = m_swapChainImageFormat;
    attachments.at(0).samples = m_msaaSamples;
    attachments.at(0).loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments.at(0).storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments.at(0).stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments.at(0).stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments.at(0).initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments.at(0).finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    // Color attachment for onscreen rendering(ID=1)
    attachments.at(1).format = m_swapChainImageFormat;
    attachments.at(1).samples = m_msaaSamples;
    attachments.at(1).loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments.at(1).storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments.at(1).stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments.at(1).stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments.at(1).initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments.at(1).finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    // Depth attachment (ID=2)
	attachments.at(2).format = findDepthFormat();
	attachments.at(2).samples = m_msaaSamples;
	attachments.at(2).loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	attachments.at(2).storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	attachments.at(2).stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	attachments.at(2).stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	attachments.at(2).initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	attachments.at(2).finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    // Resolve attachment for multisampling (ID=3)
    attachments.at(3).format = m_swapChainImageFormat;
    attachments.at(3).samples = VK_SAMPLE_COUNT_1_BIT;
    attachments.at(3).loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments.at(3).storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments.at(3).stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments.at(3).stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments.at(3).initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments.at(3).finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    // 2. Create attachment references ------------------------------------------
    std::array<VkAttachmentReference, 2> colorAttachmentsRef{};
	colorAttachmentsRef.at(0).attachment = 0; // Color attachment (ID=0)
    colorAttachmentsRef.at(0).layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachmentsRef.at(1).attachment = 1; // Color attachment (ID=1)
    colorAttachmentsRef.at(1).layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    VkAttachmentReference depthAttachmentRef{};
    depthAttachmentRef.attachment = 2; // Depth attachment (ID=2)
    depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    std::array<VkAttachmentReference, 2> colorAttachmentResolvesRef{};
    colorAttachmentResolvesRef.at(0).attachment = 3; // Resolve attachment (ID=3)
    colorAttachmentResolvesRef.at(0).layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachmentResolvesRef.at(1).attachment = 3; // Resolve attachment (ID=3)
    colorAttachmentResolvesRef.at(1).layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    // 3. Define subpasses ------------------------------------------------------
	std::array<VkSubpassDescription, 2> subpassDescriptions{};

    
	// First subpass for offscreen rendering
	subpassDescriptions.at(0).pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpassDescriptions.at(0).colorAttachmentCount = 1; //number of color attachments this subpass will WRITE to
	subpassDescriptions.at(0).pColorAttachments = colorAttachmentsRef.data(); // color OUTPUTS
	subpassDescriptions.at(0).pDepthStencilAttachment = &depthAttachmentRef;
    // pResolveAttachments is NULL or a pointer to an array of 
    // colorAttachmentCount VkAttachmentReference structures defining the 
    // resolve attachments for this subpass and their layouts.
    subpassDescriptions.at(0).pResolveAttachments = colorAttachmentResolvesRef.data();

    // Second subpass for onscreen rendering
	subpassDescriptions.at(1).pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpassDescriptions.at(1).colorAttachmentCount = 1;
	subpassDescriptions.at(1).pColorAttachments = &colorAttachmentsRef.at(1);
	subpassDescriptions.at(1).pDepthStencilAttachment = &depthAttachmentRef;
    subpassDescriptions.at(1).pResolveAttachments = colorAttachmentResolvesRef.data();


    // Subpass dependencies for layout transitions
    std::array<VkSubpassDependency, 2> dependencies{};
    // First subpass
    dependencies.at(0).srcSubpass = VK_SUBPASS_EXTERNAL;
    dependencies.at(0).dstSubpass = 0;
    dependencies.at(0).srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies.at(0).srcAccessMask = 0;
    dependencies.at(0).dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies.at(0).dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    // Second subpass
    dependencies.at(1).srcSubpass = 0;
    dependencies.at(1).dstSubpass = 1;
    dependencies.at(1).srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies.at(1).srcAccessMask = 0;
    dependencies.at(1).dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies.at(1).dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    // assemble info to build render pass
    VkRenderPassCreateInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = static_cast<uint32_t>(subpassDescriptions.size());
    renderPassInfo.pSubpasses = subpassDescriptions.data();
    renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();


    if (vkCreateRenderPass(m_contextPtr->getDevice(), &renderPassInfo, nullptr, &m_renderPass) != VK_SUCCESS) {
        throw std::runtime_error("failed to create render pass!");
    }

    infoLog() << "createRenderPass(): OK ";
}


/*
 * Bindings layouts
 */
void VkApp::createDescriptorSetLayout()
{
    // uniform buffer binding
    VkDescriptorSetLayoutBinding uboLayoutBinding{};
    uboLayoutBinding.binding = 0;
    uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboLayoutBinding.descriptorCount = 1;
    uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    uboLayoutBinding.pImmutableSamplers = nullptr; // Optional

    // sampler (i.e., texture) binding
    VkDescriptorSetLayoutBinding samplerLayoutBinding{};
    samplerLayoutBinding.binding = 1;
    samplerLayoutBinding.descriptorCount = 1;
    samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    samplerLayoutBinding.pImmutableSamplers = nullptr;
    samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    std::array<VkDescriptorSetLayoutBinding, 2> bindings = { uboLayoutBinding, samplerLayoutBinding };
    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    if (vkCreateDescriptorSetLayout(m_contextPtr->getDevice(), &layoutInfo, nullptr, &m_descriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor set layout!");
    }

    infoLog() << "createDescriptorSetLayout(): OK ";
}


/*
 * Creation of graphics pipeline
 */
void VkApp::createGraphicsPipeline()
{
    // Offscreen rendering (first subpass) shaders
    auto vertShaderCodeOffscreen = GLtools::readFile("../../src/shaders/vertOffscreen.spv");
    auto fragShaderCodeOffscreen = GLtools::readFile("../../src/shaders/fragOffscreen.spv");
    VkShaderModule vertShaderModuleOffscreen = createShaderModule(vertShaderCodeOffscreen);
    VkShaderModule fragShaderModuleOffscreen = createShaderModule(fragShaderCodeOffscreen);

    VkPipelineShaderStageCreateInfo vertShaderStageInfoOffscreen{};
    vertShaderStageInfoOffscreen.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfoOffscreen.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfoOffscreen.module = vertShaderModuleOffscreen;
    vertShaderStageInfoOffscreen.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfoOffscreen{};
    fragShaderStageInfoOffscreen.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfoOffscreen.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfoOffscreen.module = fragShaderModuleOffscreen;
    fragShaderStageInfoOffscreen.pName = "main";

    VkPipelineShaderStageCreateInfo shaderStagesOffscreen[] = { vertShaderStageInfoOffscreen, fragShaderStageInfoOffscreen };
    
    // Onscreen rendering (second subpass) shaders
    auto vertShaderCode = GLtools::readFile("../../src/shaders/vert.spv");
    auto fragShaderCode = GLtools::readFile("../../src/shaders/frag.spv");
    VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
    VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

    VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
    vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = vertShaderModule;
    vertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
    fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = fragShaderModule;
    fragShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };



    // describes the format of the vertex data that will be passed to the vertex shader
    VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
    auto bindingDescription = Vertex::getBindingDescription();
    auto attributeDescriptions = Vertex::getAttributeDescriptions();
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.vertexBindingDescriptionCount = 1;
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
    vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();


    // Describes what kind of geometry will be drawn from the vertices and if primitive restart should be enabled.
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;


    //// defines viewport
    //VkViewport viewport{};
    //viewport.x = 0.0f;
    //viewport.y = 0.0f;
    //viewport.width = (float)m_swapChainExtent.width; // in [0.0f, 1.0f]
    //viewport.height = (float)m_swapChainExtent.height; // in [0.0f, 1.0f]
    //viewport.minDepth = 0.0f;
    //viewport.maxDepth = 1.0f;

    //// defines a scissor rectangle (i.e., viewport clipping) that covers the entire viewport
    //VkRect2D scissor{};
    //scissor.offset = { 0, 0 };
    //scissor.extent = m_swapChainExtent;

    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    //viewportState.pViewports = &viewport; // static version
    viewportState.scissorCount = 1;
    // viewportState.pScissors = &scissor; // static version


    // defines rasterizer
    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    rasterizer.polygonMode = VK_POLYGON_MODE_LINE /*VK_POLYGON_MODE_FILL*/;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_NONE; // VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE; // VK_FRONT_FACE_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f; // Optional
    rasterizer.depthBiasClamp = 0.0f; // Optional
    rasterizer.depthBiasSlopeFactor = 0.0f; // Optional

    // configures multisampling for anti-aliasing
    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE; // use VK_TRUE to enable sample shading in the pipeline
    multisampling.rasterizationSamples = m_msaaSamples;
    multisampling.minSampleShading = 1.0f; // use .2f as min fraction for sample shading, closer to one is smoother
    multisampling.pSampleMask = nullptr; // Optional
    multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
    multisampling.alphaToOneEnable = VK_FALSE; // Optional

    // depth buffer
    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.minDepthBounds = 0.0f; // Optional
    depthStencil.maxDepthBounds = 1.0f; // Optional
    depthStencil.stencilTestEnable = VK_FALSE;
    depthStencil.front = {}; // Optional
    depthStencil.back = {}; // Optional

    // Defines blending between fragment shader output color with the color that is already in the framebuffer
    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_TRUE;
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

    VkPipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;
    colorBlending.blendConstants[0] = 0.0f; // Optional
    colorBlending.blendConstants[1] = 0.0f; // Optional
    colorBlending.blendConstants[2] = 0.0f; // Optional
    colorBlending.blendConstants[3] = 0.0f; // Optional

    // limited amount of the state that can actually be changed without recreating the pipeline at draw time
    // we enables dynamic viewport and scissor
    std::vector<VkDynamicState> dynamicStates = {
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR
            //VK_DYNAMIC_STATE_COLOR_WRITE_MASK_EXT
    };
    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();


    // Pipeline layouts
    // Offscreen scene rendering
    VkPipelineLayoutCreateInfo pipelineLayoutInfoOffscreen{};
    pipelineLayoutInfoOffscreen.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfoOffscreen.setLayoutCount = 1;
    pipelineLayoutInfoOffscreen.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutInfoOffscreen.pushConstantRangeCount = 0; // Optional
    pipelineLayoutInfoOffscreen.pPushConstantRanges = nullptr; // Optional

    if (vkCreatePipelineLayout(m_contextPtr->getDevice(), &pipelineLayoutInfoOffscreen, nullptr, &m_pipelineLayoutOffscreen) != VK_SUCCESS) {
        throw std::runtime_error("failed to create pipeline layout Offscreen!");
    }
    // Scene rendering
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 0; // Optional
    pipelineLayoutInfo.pPushConstantRanges = nullptr; // Optional

    if (vkCreatePipelineLayout(m_contextPtr->getDevice(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create pipeline layout!");
    }


    // Assemble info for creation of graphics pipeline
    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStagesOffscreen;
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = m_pipelineLayoutOffscreen; // references the structures describing the fixed-function stage
    pipelineInfo.renderPass = m_renderPass;
    pipelineInfo.subpass = 0; // first subpass
    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
    pipelineInfo.basePipelineIndex = -1; // Optional

    // Finally creates the pipeline
    if (vkCreateGraphicsPipelines(m_contextPtr->getDevice(), VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_graphicsPipelineOffscreen) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipelineOffscreen!");
    }
    

    // Re-define pipeline info for second subpass (onscreen rendering)
    pipelineInfo.pStages = shaderStages;
    pipelineInfo.layout = m_pipelineLayout;
    pipelineInfo.subpass = 1;
    
    if (vkCreateGraphicsPipelines(m_contextPtr->getDevice(), VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_graphicsPipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipeline!");
    }
    

    vkDestroyShaderModule(m_contextPtr->getDevice(), fragShaderModuleOffscreen, nullptr);
    vkDestroyShaderModule(m_contextPtr->getDevice(), vertShaderModuleOffscreen, nullptr);
    vkDestroyShaderModule(m_contextPtr->getDevice(), fragShaderModule, nullptr);
    vkDestroyShaderModule(m_contextPtr->getDevice(), vertShaderModule, nullptr);
    

    infoLog() << "createGraphicsPipeline(): OK ";
}


/*
 * Creation of shader modules
 */
VkShaderModule VkApp::createShaderModule(const std::vector<char>& _code)
{
    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = _code.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(_code.data());

    VkShaderModule shaderModule;
    if (vkCreateShaderModule(m_contextPtr->getDevice(), &createInfo, nullptr, &shaderModule) != VK_SUCCESS) 
    {
        throw std::runtime_error("failed to create shader module!");
    }

    return shaderModule;
}


/*
 * Creation of framebuffers
 */
void VkApp::createFramebuffers()
{
    m_swapChainFramebuffers.resize(m_swapChainImageViews.size());

    // iterate through the image views and create a framebuffer for each of them
    for (size_t i = 0; i < m_swapChainImageViews.size(); i++)
    {
        // number of attachments must match the attachments defined in createRenderPass()
        std::array<VkImageView, 4> attachments = { m_offscreenImage.getImageView(),
                                                   m_colorImage.getImageView(),
                                                   m_depthImage.getImageView(),
                                                   m_swapChainImageViews[i] };

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = m_renderPass;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = m_swapChainExtent.width;
        framebufferInfo.height = m_swapChainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(m_contextPtr->getDevice(), &framebufferInfo, nullptr, &m_swapChainFramebuffers[i]) != VK_SUCCESS) {
            throw std::runtime_error("failed to create framebuffer!");
        }
    }

    infoLog() << "createFramebuffers(): OK ";
}


/*
 * Setup depth-buffer
 */
void VkApp::createDepthResources()
{
    VkFormat depthFormat = findDepthFormat();

    m_depthImage.createImage(*m_contextPtr,
                             m_swapChainExtent.width, m_swapChainExtent.height, m_msaaSamples,
                             depthFormat,
                             VK_IMAGE_TILING_OPTIMAL,
                             VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                             VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );

    m_depthImage.createImageView(*m_contextPtr, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);

    m_depthImage.transitionImageLayout(*m_contextPtr, depthFormat,
                          VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
}


/*
 * Sorts a given list of candidate formats from most desirable to least desirable, 
 * and checks which is the first one that is supported
 */
VkFormat VkApp::findSupportedFormat(const std::vector<VkFormat>& _candidates, VkImageTiling _tiling, VkFormatFeatureFlags _features)
{
    for (VkFormat format : _candidates) 
    {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(m_contextPtr->getPhysicalDevice(), format, &props);

        if (_tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & _features) == _features) {
            return format;
        }
        else if (_tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & _features) == _features) {
            return format;
        }
    }

    throw std::runtime_error("failed to find supported format!");
}


/*
 * Helper function to select a format with a depth component that supports usage as depth attachment
 */
VkFormat VkApp::findDepthFormat()
{
    return findSupportedFormat(
        { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT },
        VK_IMAGE_TILING_OPTIMAL,
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
    );
}


/*
 * Creates a multisampled color buffer
 */
void VkApp::createColorResources()
{
    VkFormat colorFormat = m_swapChainImageFormat;

    // Color-coded position buffer for offscreen rendering
    m_offscreenImage.createImage(*m_contextPtr,
                                 m_swapChainExtent.width, m_swapChainExtent.height, m_msaaSamples,
                                 colorFormat,
                                 VK_IMAGE_TILING_OPTIMAL,
                                 VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );

    m_offscreenImage.createImageView(*m_contextPtr, colorFormat, VK_IMAGE_ASPECT_COLOR_BIT);

    
    // Rendered color buffer for onscreen rendering
    m_colorImage.createImage(*m_contextPtr,
                             m_swapChainExtent.width, m_swapChainExtent.height, m_msaaSamples,
                             colorFormat,
                             VK_IMAGE_TILING_OPTIMAL,
                             VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
                             VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );

    m_colorImage.createImageView(*m_contextPtr, colorFormat, VK_IMAGE_ASPECT_COLOR_BIT);

}


/*
 * Creation of Uniforms buffer
 */
void VkApp::createUniformBuffers() 
{
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);

    m_uniformBuffers.resize(MAX_FRAMES_IN_FLIGHT);
    m_uniformBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);
    m_uniformBuffersMapped.resize(MAX_FRAMES_IN_FLIGHT);

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) 
    {
        createBuffer(m_contextPtr->getPhysicalDevice(), m_contextPtr->getDevice(), bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     m_uniformBuffers[i], m_uniformBuffersMemory[i]);

        vkMapMemory(m_contextPtr->getDevice(), m_uniformBuffersMemory[i], 0, bufferSize, 0, &m_uniformBuffersMapped[i]);
    }
}


/*
 * Descriptors allocation from a pool
 */
void VkApp::createDescriptorPool() 
{
    // Two descriptors: uniforms and sampler
    std::array<VkDescriptorPoolSize, 2> poolSizes{};
    poolSizes.at(0).type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes.at(0).descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
    poolSizes.at(1).type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes.at(1).descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

    if (vkCreateDescriptorPool(m_contextPtr->getDevice(), &poolInfo, nullptr, &m_descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor pool!");
    }
}


/*
 * Allocates the descriptor sets
 */
void VkApp::createDescriptorSets()
{
    std::vector<VkDescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, m_descriptorSetLayout);
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = m_descriptorPool;
    allocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
    allocInfo.pSetLayouts = layouts.data();

    m_descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);
    if (vkAllocateDescriptorSets(m_contextPtr->getDevice(), &allocInfo, m_descriptorSets.data()) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) 
    {
        VkDescriptorBufferInfo bufferInfo{};
        bufferInfo.buffer = m_uniformBuffers.at(i);
        bufferInfo.offset = 0;
        bufferInfo.range = sizeof(UniformBufferObject);

        std::array<VkWriteDescriptorSet, 1> descriptorWrites{};
        descriptorWrites.at(0).sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrites.at(0).dstSet = m_descriptorSets.at(i);
        descriptorWrites.at(0).dstBinding = 0;
        descriptorWrites.at(0).dstArrayElement = 0;
        descriptorWrites.at(0).descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        descriptorWrites.at(0).descriptorCount = 1;
        descriptorWrites.at(0).pBufferInfo = &bufferInfo;

        vkUpdateDescriptorSets(m_contextPtr->getDevice(), static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
    }
}


/*
 * Creation of command buffer
 */
void VkApp::createCommandBuffers()
{
    m_commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = m_contextPtr->getCommandPool();
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;
    allocInfo.commandBufferCount = (uint32_t)m_commandBuffers.size();

    if (vkAllocateCommandBuffers(m_contextPtr->getDevice(), &allocInfo, m_commandBuffers.data()) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate command buffers!");
    }

    infoLog() << "createCommandBuffer(): OK ";
}


/*
 * Writes commands into a command buffer
 */
void VkApp::recordCommandBuffer(VkCommandBuffer _commandBuffer, uint32_t _imageIndex) 
{
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = 0; // Optional
    beginInfo.pInheritanceInfo = nullptr; // Optional

    if (vkBeginCommandBuffer(_commandBuffer, &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("failed to begin recording command buffer!");
    }


    // Prepares render pass
    VkRenderPassBeginInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    renderPassInfo.renderPass = m_renderPass;
    renderPassInfo.framebuffer = m_swapChainFramebuffers.at(_imageIndex);
    renderPassInfo.renderArea.offset = { 0, 0 };
    renderPassInfo.renderArea.extent = m_swapChainExtent;

    // m_renderPass was created with 3 attachments (cf.createRenderPass())
    // -> we must define 3 clear values
    std::array<VkClearValue, 3> clearValues{}; 
    clearValues.at(0).color = { {0.05f, 0.05f, 0.05f, 1.0f} }; // color clear value for first color attachment (offscreen rendering)
    clearValues.at(1).color = { {0.0f, 0.0f, 0.05f, 1.0f} };   // color clear value for second color attachment (onscreen gbuffer)
    clearValues.at(2).depthStencil = { 1.0f, 0 };              // depth clear value for depth attachment

    renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
    renderPassInfo.pClearValues = clearValues.data();

    // Begins render pass (first performs offscreen rendering)
    vkCmdBeginRenderPass(_commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
    {
		// Bind Offscreen rendering pipeline
        vkCmdBindPipeline(_commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_graphicsPipelineOffscreen);
        
        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(m_swapChainExtent.width);
        viewport.height = static_cast<float>(m_swapChainExtent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(_commandBuffer, 0, 1, &viewport);

        VkRect2D scissor{};
        scissor.offset = { 0, 0 };
        scissor.extent = m_swapChainExtent;
        vkCmdSetScissor(_commandBuffer, 0, 1, &scissor);


        // Bind vertex buffer
        VkBuffer vertexBuffers[] = { m_surfMesh.getVertexBuffer() };
        VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(_commandBuffer, 0, 1, vertexBuffers, offsets);

        // Bind index buffer
        vkCmdBindIndexBuffer(_commandBuffer, m_surfMesh.getIndexBuffer(), 0, VK_INDEX_TYPE_UINT32 /*VK_INDEX_TYPE_UINT16*/);

        // Bind descriptors (i.e., uniforms) for offscreen rendering pipeline layout
        vkCmdBindDescriptorSets(_commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayoutOffscreen, 0, 1, &m_descriptorSets[m_currentFrame], 0, nullptr);
        
        // Issue draw command
        //vkCmdDrawIndexed(_commandBuffer, static_cast<uint32_t>(m_surfMesh.getIndices().size() ), 1, 0, 0, 0);
	}


    // Second render pass (onscreen rendering)
    vkCmdNextSubpass(_commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    {
        // Basic drawing commands
        vkCmdBindPipeline(_commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_graphicsPipeline);

        VkBuffer vertexBuffers[] = { m_dynMesh.getVertexBuffer() };
        VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(_commandBuffer, 0, 1, vertexBuffers, offsets);
        vkCmdBindIndexBuffer(_commandBuffer, m_dynMesh.getIndexBuffer(), 0, VK_INDEX_TYPE_UINT32);

        // Bind descriptors (i.e., uniforms) for onscreen rendering pipeline layout
        vkCmdBindDescriptorSets(_commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayout, 0, 1, &m_descriptorSets[m_currentFrame], 0, nullptr);

        // Issue draw command
        //vkCmdDraw(_commandBuffer, static_cast<uint32_t>(m_vertices.size()), 1, 0, 0); // unindexed vertex buffer version
        vkCmdDrawIndexed(_commandBuffer, static_cast<uint32_t>(m_dynMesh.getIndices().size() ), 1, 0, 0, 0); // indexed vertex buffer version


        VkBuffer vertexBuffers2[] = { m_surfMesh.getVertexBuffer() };
        vkCmdBindVertexBuffers(_commandBuffer, 0, 1, vertexBuffers2, offsets);
        vkCmdBindIndexBuffer(_commandBuffer, m_surfMesh.getIndexBuffer(), 0, VK_INDEX_TYPE_UINT32);
        vkCmdBindDescriptorSets(_commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayout, 0, 1, &m_descriptorSets[m_currentFrame], 0, nullptr);
        vkCmdDrawIndexed(_commandBuffer, static_cast<uint32_t>(m_surfMesh.getIndices().size() ), 1, 0, 0, 0);
    }

    // Ends render pass
    vkCmdEndRenderPass(_commandBuffer);

    if (vkEndCommandBuffer(_commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("failed to record command buffer!");
    }

 //       void* data;
	//vkMapMemory( m_contextPtr->getDevice(), m_colorImage.getImageMemory(), 0, VK_WHOLE_SIZE, 0, &data );
	//std::ofstream ofs( "out.raw", std::ostream::binary );
	//ofs.write( (char*)data, m_swapChainExtent.width * m_swapChainExtent.height * m_swapChainImageFormat * 4);
	//vkUnmapMemory( m_contextPtr->getDevice(), m_colorImage.getImageMemory() );
}


/*
 * Creation of semaphores and fences
 */
void VkApp::createSyncObjects()
{
    m_imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    m_renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    m_inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
    {
        if (vkCreateSemaphore(m_contextPtr->getDevice(), &semaphoreInfo, nullptr, &m_imageAvailableSemaphores[i]) != VK_SUCCESS ||
            vkCreateSemaphore(m_contextPtr->getDevice(), &semaphoreInfo, nullptr, &m_renderFinishedSemaphores[i]) != VK_SUCCESS ||
            vkCreateFence(m_contextPtr->getDevice(), &fenceInfo, nullptr, &m_inFlightFences[i]) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to create semaphores!");
        }
    }

    infoLog() << "createSyncObjects(): OK ";
}


/*
 * Geometry update function
 */
void VkApp::updateGeom()
{
    if (ANIMATION_MODEL == eAnimationModels::ARAP )
    {
        m_arap.solve(1e-6);
        m_dynMesh.readARAP(m_arap);
    }
    else if (ANIMATION_MODEL == eAnimationModels::FEM )
    {
        m_fem.updateBoundaryConditions();
        m_fem.solve();
        m_dynMesh.readFEM(m_fem);
    }
    else
    {
        m_massSpringSystem.iterate();
        m_dynMesh.readMassSpringSystem(m_massSpringSystem);
    }
    m_surfMesh.updateParametricSurface(m_dynMesh, 18, eParametricSurface::BEZIER);
    m_surfMesh.updateVertexBuffer(*m_contextPtr);
    m_dynMesh.updateVertexBuffer(*m_contextPtr);
}


/*
 * Drawing function
 */
void VkApp::drawFrame()
{

    vkWaitForFences(m_contextPtr->getDevice(), 1, &m_inFlightFences[m_currentFrame], VK_TRUE, UINT64_MAX);

    uint32_t imageIndex;
    VkResult result = vkAcquireNextImageKHR(m_contextPtr->getDevice(), m_swapChain, UINT64_MAX, 
                                            m_imageAvailableSemaphores[m_currentFrame], VK_NULL_HANDLE, &imageIndex);

    if (result == VK_ERROR_OUT_OF_DATE_KHR) 
    {
        recreateSwapChain();
        return;
    }
    else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("failed to acquire swap chain image!");
    }

    updateUniformBuffer(m_currentFrame);

    // Only reset the fence if we are submitting work
    vkResetFences(m_contextPtr->getDevice(), 1, &m_inFlightFences[m_currentFrame]);

    vkResetCommandBuffer(m_commandBuffers[m_currentFrame], 0);
    recordCommandBuffer(m_commandBuffers[m_currentFrame], imageIndex);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore waitSemaphores[] = { m_imageAvailableSemaphores[m_currentFrame] };
    VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffers[m_currentFrame];

    VkSemaphore signalSemaphores[] = { m_renderFinishedSemaphores[m_currentFrame] };
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    if (vkQueueSubmit(m_contextPtr->getGraphicsQueue(), 1, &submitInfo, m_inFlightFences[m_currentFrame]) != VK_SUCCESS) {
        throw std::runtime_error("failed to submit draw command buffer!");
    }

    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;
    VkSwapchainKHR swapChains[] = { m_swapChain };
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = swapChains;
    presentInfo.pImageIndices = &imageIndex;
    presentInfo.pResults = nullptr; // Optional

    result = vkQueuePresentKHR(m_contextPtr->getPresentQueue(), &presentInfo);

    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || m_framebufferResized)
    {
        m_framebufferResized = false;
        recreateSwapChain();
    }
    else if (result != VK_SUCCESS) {
        throw std::runtime_error("failed to present swap chain image!");
    }

    // update current frame id
    m_currentFrame = (m_currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}


/*
 * Cleanup the swap chain before recreating it
 */
void VkApp::cleanupSwapChain() 
{
    m_offscreenImage.cleanup(*m_contextPtr);
    m_colorImage.cleanup(*m_contextPtr);
    m_depthImage.cleanup(*m_contextPtr);

    for (size_t i = 0; i < m_swapChainFramebuffers.size(); i++)
    {
        vkDestroyFramebuffer(m_contextPtr->getDevice(), m_swapChainFramebuffers[i], nullptr);
    }

    for (size_t i = 0; i < m_swapChainImageViews.size(); i++)
    {
        vkDestroyImageView(m_contextPtr->getDevice(), m_swapChainImageViews[i], nullptr);
    }

    vkDestroySwapchainKHR(m_contextPtr->getDevice(), m_swapChain, nullptr);
}


/*
 * Recreate the swap chain whenever event happens
 */
void VkApp::recreateSwapChain() 
{
    int width = 0, height = 0;
    glfwGetFramebufferSize(m_window, &width, &height);
    while (width == 0 || height == 0)
    {
        glfwGetFramebufferSize(m_window, &width, &height);
        glfwWaitEvents();
    }

    vkDeviceWaitIdle(m_contextPtr->getDevice());

    cleanupSwapChain();

    createSwapChain();
    createImageViews();
    createColorResources();
    createDepthResources();
    createFramebuffers();
}


/*
 * Generates a new transformation every frame to make the geometry spin around
 */
void VkApp::updateUniformBuffer(uint32_t _currentImage) 
{
    m_ubo.model = m_trackball.getRotationMatrix() 
                * m_initModel;

    memcpy(m_uniformBuffersMapped[_currentImage], &m_ubo, sizeof(m_ubo));
}


/*
 * Window resize callback
 */
void VkApp::framebufferResizeCallback(GLFWwindow* _window, int _width, int _height)
{
    auto app = reinterpret_cast<VkApp*>(glfwGetWindowUserPointer(_window));
    app->m_framebufferResized = true;
}

/*
 * Keyboard event callback
 */
void VkApp::keyCallback(GLFWwindow* _window, int _key, int _scancode, int _action, int _mods)
{
    // return to init positon when "R" pressed
    if (_key == GLFW_KEY_R && _action == GLFW_PRESS)
    {
        auto app = reinterpret_cast<VkApp*>(glfwGetWindowUserPointer(_window));
        app->m_trackball.reStart();
    }
}

/*
 * Mouse button event callback
 */
void VkApp::mouseButtonCallback(GLFWwindow* _window, int _button, int _action, int _mods)
{
    auto app = reinterpret_cast<VkApp*>(glfwGetWindowUserPointer(_window));

    // get mouse cursor position
    double x, y;
    glfwGetCursorPos(_window, &x, &y);

    // activate/de-activate trackball with mouse button
    if (_action == GLFW_PRESS) 
    {
        if (_button == GLFW_MOUSE_BUTTON_LEFT)
            app->m_trackball.startTracking( glm::vec2(x, y) );
    }
    else 
    {
        if (_button == GLFW_MOUSE_BUTTON_LEFT)
            app->m_trackball.stopTracking();
    }
    
}

/*
 * Mouse scroll event callback
 */
void VkApp::scrollCallback(GLFWwindow* _window, double _xoffset, double _yoffset)
{
}


/*
 * Mouse cursor event callback
 */
void VkApp::cursorPosCallback(GLFWwindow* _window, double _x, double _y)
{
    auto app = reinterpret_cast<VkApp*>(glfwGetWindowUserPointer(_window));

    // rotate trackball according to mouse cursor movement
    if ( app->m_trackball.isTracking()) 
        app->m_trackball.move( glm::vec2(_x, _y) );
}


} // namespace CompGeom