
//
// src: rawdata.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#include <ros/ros.h>
#include "perfect_velodyne/rawdata.h"

using namespace std;
using namespace velodyne_rawdata;

namespace perfect_velodyne
{

	RawDataWithNormal::RawDataWithNormal() {}

	void RawDataWithNormal::unpack(const velodyne_msgs::VelodynePacket &pkt,
			VPointCloudNormal &pc)
	{
		ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

		/** special parsing for the VLP16 **/
		// if (calibration_.num_lasers == 16)
		// {
		// 	unpack_vlp16(pkt, pc);
		// 	return;
		// }

		const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

		for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

			// upper bank lasers are numbered [0..31]
			// NOTE: this is a change from the old velodyne_common implementation
			int bank_origin = 0;
			if (raw->blocks[i].header == LOWER_BANK) {
				// lower bank lasers are [32..63]
				bank_origin = 32;
			}

			for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {

				float x, y, z;
				float intensity;
				uint8_t laser_number;       ///< hardware laser number

				laser_number = j + bank_origin;
				velodyne_pointcloud::LaserCorrection &corrections = 
					calibration_.laser_corrections[laser_number];

				/** Position Calculation */

				union two_bytes tmp;
				tmp.bytes[0] = raw->blocks[i].data[k];
				tmp.bytes[1] = raw->blocks[i].data[k+1];
				/*condition added to avoid calculating points which are not
				  in the interesting defined area (min_angle < area < max_angle)*/
				if ((raw->blocks[i].rotation >= config_.min_angle 
							&& raw->blocks[i].rotation <= config_.max_angle 
							&& config_.min_angle < config_.max_angle)
						||(config_.min_angle > config_.max_angle 
							&& (raw->blocks[i].rotation <= config_.max_angle 
								|| raw->blocks[i].rotation >= config_.min_angle))){
					float distance = tmp.uint * DISTANCE_RESOLUTION;
					distance += corrections.dist_correction;

					float cos_vert_angle = corrections.cos_vert_correction;
					float sin_vert_angle = corrections.sin_vert_correction;
					float cos_rot_correction = corrections.cos_rot_correction;
					float sin_rot_correction = corrections.sin_rot_correction;

					// cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
					// sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
					float cos_rot_angle = 
						cos_rot_table_[raw->blocks[i].rotation] * cos_rot_correction + 
						sin_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;
					float sin_rot_angle = 
						sin_rot_table_[raw->blocks[i].rotation] * cos_rot_correction - 
						cos_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;

					float horiz_offset = corrections.horiz_offset_correction;
					float vert_offset = corrections.vert_offset_correction;

					// Compute the distance in the xy plane (w/o accounting for rotation)
					/**the new term of 'vert_offset * sin_vert_angle'
					 * was added to the expression due to the mathemathical
					 * model we used.
					 */
					float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

					// Calculate temporal X, use absolute value.
					float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
					// Calculate temporal Y, use absolute value
					float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
					if (xx < 0) xx=-xx;
					if (yy < 0) yy=-yy;

					// Get 2points calibration values,Linear interpolation to get distance
					// correction for X and Y, that means distance correction use
					// different value at different distance
					float distance_corr_x = 0;
					float distance_corr_y = 0;
					if (corrections.two_pt_correction_available) {
						distance_corr_x = 
							(corrections.dist_correction - corrections.dist_correction_x)
							* (xx - 2.4) / (25.04 - 2.4) 
							+ corrections.dist_correction_x;
						distance_corr_x -= corrections.dist_correction;
						distance_corr_y = 
							(corrections.dist_correction - corrections.dist_correction_y)
							* (yy - 1.93) / (25.04 - 1.93)
							+ corrections.dist_correction_y;
						distance_corr_y -= corrections.dist_correction;
					}

					float distance_x = distance + distance_corr_x;
					/**the new term of 'vert_offset * sin_vert_angle'
					 * was added to the expression due to the mathemathical
					 * model we used.
					 */
					xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
					///the expression wiht '-' is proved to be better than the one with '+'
					x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

					float distance_y = distance + distance_corr_y;
					xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
					/**the new term of 'vert_offset * sin_vert_angle'
					 * was added to the expression due to the mathemathical
					 * model we used.
					 */
					y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

					// Using distance_y is not symmetric, but the velodyne manual
					// does this.
					/**the new term of 'vert_offset * cos_vert_angle'
					 * was added to the expression due to the mathemathical
					 * model we used.
					 */
					z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

					/** Use standard ROS coordinate system (right-hand rule) */
					float x_coord = y;
					float y_coord = -x;
					float z_coord = z;

					/** Intensity Calculation */

					float min_intensity = corrections.min_intensity;
					float max_intensity = corrections.max_intensity;

					intensity = raw->blocks[i].data[k+2];

					float focal_offset = 256 
						* (1 - corrections.focal_distance / 13100) 
						* (1 - corrections.focal_distance / 13100);
					float focal_slope = corrections.focal_slope;
					intensity += focal_slope * (std::abs(focal_offset - 256 * 
								(1 - static_cast<float>(tmp.uint)/65535)*(1 - static_cast<float>(tmp.uint)/65535)));
					intensity = (intensity < min_intensity) ? min_intensity : intensity;
					intensity = (intensity > max_intensity) ? max_intensity : intensity;

					// if (pointInRange(distance)) {

					// convert polar coordinates to Euclidean XYZ
					VPointNormal point;
					point.ring = corrections.laser_ring;
					point.x = x_coord;
					point.y = y_coord;
					point.z = z_coord;
					point.intensity = intensity;

					// append this point to the cloud
					pc.points.push_back(point);
					++pc.width;
					// }
				}
			}
		}
	}

	// private

} // namespace perfect_velodyne

