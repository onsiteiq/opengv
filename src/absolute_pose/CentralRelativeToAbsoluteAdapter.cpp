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


#include <opengv/absolute_pose/CentralRelativeToAbsoluteAdapter.hpp>


opengv::absolute_pose::CentralRelativeToAbsoluteAdapter::CentralRelativeToAbsoluteAdapter(
	const bearingVectors_t & bearingVectors1,
	const bearingVectors_t & bearingVectors2,
	const translation_t & o1,
	const translation_t & o2,
	const translation_t & t) :
    RelativeToAbsoluteAdapterBase(t, Eigen::Matrix3d::Identity()),
    _bearingVectors1(bearingVectors1),
	_bearingVectors2(bearingVectors2),
	_o1(o1), _o2(o2)
{}

opengv::absolute_pose::CentralRelativeToAbsoluteAdapter::~CentralRelativeToAbsoluteAdapter()
{}

opengv::point_t
opengv::absolute_pose::CentralRelativeToAbsoluteAdapter::getPoint(
	size_t index) const
{
	// For this adapter points are procedurally generated. We might consider
	// creating them ahead of time outside of the adapter as well.

	rotation_t R = getR();

	bearingVector_t b1 = getBearingVector(index);

	// Bearings are in camera coordinate system so transform to
	// world coordinates.
	b1 = R.transpose() * b1;

	bearingVector_t b2 = getBearingVector2(index);

	translation_t t = gett();
	translation_t o1 = geto1();
	translation_t o2 = geto2();

	t = t / t.norm();

	Eigen::Vector3d plane_n = b1.cross(t);

	// We cannot determine the translation if the bearing from
	// camera two (b2) is in the plane formed by the camera one bearing
	// (b1) and unscaled translation (t).

	if (fabs(plane_n.dot(b2)) < 0.001)
		return Eigen::Vector3d::Zero();

	// Intersect the ray from camera two with the plane described in
	// the above comment.

	double d = (o1 - o2).dot(plane_n) / b2.dot(plane_n);

	Eigen::Vector3d p = d * b2 + o2;

	return p;
}

opengv::bearingVector_t
opengv::absolute_pose::CentralRelativeToAbsoluteAdapter::getBearingVector(
	size_t index) const
{
	assert(index < _bearingVectors1.size());
	return _bearingVectors1[index];
}

opengv::bearingVector_t
opengv::absolute_pose::CentralRelativeToAbsoluteAdapter::getBearingVector2(
	size_t index) const
{
	assert(index < _bearingVectors2.size());
	return _bearingVectors2[index];
}

size_t
opengv::absolute_pose::CentralRelativeToAbsoluteAdapter::getNumberCorrespondences() const
{
	return _bearingVectors1.size();
}
