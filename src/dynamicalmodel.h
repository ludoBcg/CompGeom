/*********************************************************************************************************************
 *
 * dynamicalmodel.h
 *
 * CompGeom
 * Ludovic Blache
 *
 *********************************************************************************************************************/

#ifndef DYNAMICALMODEL_H
#define DYNAMICALMODEL_H

#include <iostream>
#include <vector>
#include <assert.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>


namespace CompGeom
{


/*!
* \class DynamicalModel
* \brief Abstract class for dynamical models
*/
class DynamicalModel
{
   
public:

    /*----------------------------------------------------------------------------------------------+
    |                                        CONSTRUCTORS                                           |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn DynamicalModel
    * \brief Default constructor
    */
    DynamicalModel() = default;


    /*!
    * \fn ~DynamicalModel
    * \brief Destructor
    */
    virtual ~DynamicalModel() = default;


    /*----------------------------------------------------------------------------------------------+
    |                                     GETTERS / SETTERS                                         |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * ...
    */
    

    /*----------------------------------------------------------------------------------------------+
    |                                        MISCELLANEOUS                                          |
    +-----------------------------------------------------------------------------------------------*/

    /*!
    * \fn initialize
    * \brief Initializes dynamical model
    * \param _vertices : List of vertices' positions
    * \param _indices : List of indices
    * \param _fixedPointsIds : List of fixed points indices
    * \param _constraintPoints : List of constraint points (Id, target pos)
    * \return : success
    */
    virtual bool initialize( std::vector<glm::vec3>& _verticesPos
                           , std::vector<uint32_t>& _indices
                           , std::vector<uint32_t>& _fixedPointsIds
                           , std::vector<std::pair<uint32_t, glm::vec3> >& _constraintPoints) = 0;

    /*!
    * \fn iterate
    * \brief Update system state for one timestep, using numerical integration
    * \return : success
    */
    virtual bool iterate() = 0;

    /*!
    * \fn getResult
    * \brief Returns new vertices' position
    * \param _res : List of vertices to return
    * \return : success
    */
    virtual bool getResult(std::vector<glm::vec3>& _res) = 0;


protected:

    /*----------------------------------------------------------------------------------------------+
    |                                         ATTRIBUTES                                            |
    +-----------------------------------------------------------------------------------------------*/



}; // class DynamicalModel

} // namespace CompGeom

#endif // DYNAMICALMODEL_H