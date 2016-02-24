#pragma once

/**\file transform_stamped.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <string>
#include <stdint.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace socket_to_tf {
// ############################################################################   TransformStamped   ###########################################################################
/**
 * \brief Description...
 */
struct TransformStamped {
	double x;
	double y;
	double z;
	double qx;
	double qy;
	double qz;
	double qw;
	uint32_t timestamp_seconds;
	uint32_t timestamp_nanoseconds;
	std::string source_frame;
	std::string target_frame;

	template<typename Archive>
	void serialize(Archive& ar, const unsigned int version) {
		ar & x;
		ar & y;
		ar & z;
		ar & qx;
		ar & qy;
		ar & qz;
		ar & qw;
		ar & timestamp_seconds;
		ar & timestamp_nanoseconds;
		ar & source_frame;
		ar & target_frame;
	}
};

} /* namespace socket_to_tf */
