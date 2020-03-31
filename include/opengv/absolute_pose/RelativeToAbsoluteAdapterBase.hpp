/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

/**
 * \file CentralToAbsoluteAdapter.hpp
 * \brief Adapter-class for passing bearing-vector correspondences and camera origins to the
 *        central relative-to-absolute-pose algorithm. It maps opengv types
 *        back to opengv types.
 */

#ifndef OPENGV_ABSOLUTE_POSE_RELATIVETOABSOLUTEADAPTER_HPP_
#define OPENGV_ABSOLUTE_POSE_RELATIVETOABSOLUTEADAPTER_HPP_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/absolute_pose/AbsoluteAdapterBase.hpp>

/**
 * \brief The namespace of this library.
 */
namespace opengv
{
/**
 * \brief The namespace for the absolute pose methods.
 */
namespace absolute_pose
{


class RelativeToAbsoluteAdapterBase : public AbsoluteAdapterBase
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	/**
	* Constructor
	*/
	RelativeToAbsoluteAdapterBase() {}
			
	RelativeToAbsoluteAdapterBase(
		const opengv::translation_t & t,
		const opengv::rotation_t & R) : AbsoluteAdapterBase(t, R) {}

	/**
	* Destructor
	*/
	virtual ~RelativeToAbsoluteAdapterBase() {};

	//Access of correspondences

	/**
	* \brief Retrieve the bearing vector of a correspondence in viewpoint 1.
	* \param[in] index The serialized index of the correspondence.
	* \return The corresponding bearing vector.
	*/
	//virtual opengv::bearingVector_t getBearingVector1(size_t index) const = 0;
	/**
	* \brief Retrieve the bearing vector of a correspondence in viewpoint 2.
	* \param[in] index The serialized index of the correspondence.
	* \return The corresponding bearing vector.
	*/
	virtual opengv::bearingVector_t getBearingVector2(size_t index) const = 0;
	/**
	* \brief Retrieve the origin of camera 1.
	* \return The origin of camera 1.
	*/
	virtual opengv::translation_t geto1() const = 0;
	/**
	* \brief Retrieve the origin of camera 2.
	* \return The origin of camera 2.
	*/
	virtual opengv::translation_t geto2() const = 0;

	/* Special overrides from parent class for this adapter */

	///** See parent-class. Not applicable to this adapter. Returns zero vector. */
	//virtual opengv::bearingVector_t getBearingVector(size_t index) const;
	/** See parent-class */
	virtual double getWeight(size_t index) const;
	/** See parent-class. Returns zero for this adapter. */
	virtual opengv::translation_t getCamOffset(size_t index) const;
	/** See parent-class Returns identity for this adapter. */
	virtual opengv::rotation_t getCamRotation(size_t index) const;
	/** See parent-class Returns zero point for this adapter. */
	//virtual opengv::point_t getPoint(size_t index) const;
};

}
}

#endif /* OPENGV_ABSOLUTE_POSE_CENTRALABSOLUTEADAPTER_HPP_ */
