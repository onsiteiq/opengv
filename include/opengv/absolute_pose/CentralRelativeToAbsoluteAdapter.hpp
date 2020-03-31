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

#ifndef OPENGV_ABSOLUTE_POSE_CENTRALRELATIVETOABSOLUTEADAPTER_HPP_
#define OPENGV_ABSOLUTE_POSE_CENTRALRELATIVETOABSOLUTEADAPTER_HPP_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/absolute_pose/RelativeToAbsoluteAdapterBase.hpp>

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

/**
 * Check the documentation of the parent-class to understand the meaning of
 * an AbsoluteAdapter. This child-class is for the central case and holds data
 * in form of references to opengv-types.
 */
class CentralRelativeToAbsoluteAdapter : public RelativeToAbsoluteAdapterBase
{
protected:
	using AbsoluteAdapterBase::_t;
	using AbsoluteAdapterBase::_R;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * \brief Constructor. See protected class-members to understand parameters
   */
  CentralRelativeToAbsoluteAdapter(
      const bearingVectors_t & bearingVectors1,
	  const bearingVectors_t & bearingVectors2,
      const translation_t & o1,
	  const translation_t & o2,
	  const translation_t & t );
  /**
   * Destructor
   */
  virtual ~CentralRelativeToAbsoluteAdapter();

  // From CentralRelativeToAbsoluteAdapterBase
  
  /** See parent class */
  //virtual opengv::bearingVector_t getBearingVector1(size_t index) const;
  virtual opengv::bearingVector_t getBearingVector(size_t index) const;
  /** See parent class */
  virtual opengv::bearingVector_t getBearingVector2(size_t index) const;
  /** See parent class */
  opengv::translation_t geto1() const { return _o1; };
  /** See parent class */
  opengv::translation_t geto2() const { return _o2; };

  // From AbsoluteAdapterBase

  /** See parent-class */
  virtual size_t getNumberCorrespondences() const;

  virtual opengv::point_t getPoint(size_t index) const;
  
protected:
  /** Reference to origin of viewpoint 1. */
  const translation_t & _o1;
  /** Reference to origin of viewpoint 2. */
  const translation_t & _o2;
  /** Reference to bearing-vectors expressed in viewpoint 1. */
  const bearingVectors_t & _bearingVectors1;
  /** Reference to bearing-vectors expressed in viewpoint 2. */
  const bearingVectors_t & _bearingVectors2;
};

}
}

#endif /* OPENGV_ABSOLUTE_POSE_CENTRALABSOLUTEADAPTER_HPP_ */
