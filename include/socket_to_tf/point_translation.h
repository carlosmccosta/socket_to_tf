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
struct PointTranslation {
	int16_t x_start;
	int16_t y_start;
	int16_t z_start;
	int16_t x_end;
	int16_t y_end;
	int16_t z_end;

	template<typename Archive>
	void serialize(Archive& ar, const unsigned int version) {
		ar & x_start;
		ar & y_start;
		ar & z_start;
		ar & x_end;
		ar & y_end;
		ar & z_end;
	}
};

} /* namespace socket_to_tf */
