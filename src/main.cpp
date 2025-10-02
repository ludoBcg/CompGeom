/*********************************************************************************************************************
 *
 * main.cpp
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#include "vkapp.h"

#include <stdexcept>
#include <iostream>


int main() 
{

    CompGeom::VkApp app;

    try 
    {
        app.run();
    }
    catch (const std::exception& e) 
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;

}