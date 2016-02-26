/**\file socket_to_tf.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <socket_to_tf/socket_to_tf.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace socket_to_tf {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void SocketToTF::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	node_handle_ = node_handle;
	private_node_handle_ = private_node_handle;
	private_node_handle_->param("server_protocol", transform_socket_server_protocol_, std::string("tcp"));
	private_node_handle_->param("server_host", transform_socket_server_host_, std::string("localhost"));
	private_node_handle_->param("server_port_sync", transform_socket_server_port_sync_, -1);
	private_node_handle_->param("server_port_data", transform_socket_server_port_data_, 1337);

	private_node_handle_->param("use_static_transform_broadcaster", use_static_transform_broadcaster_, false);
	if (use_static_transform_broadcaster_) {
		static_transform_broadcaster_ptr_ = boost::shared_ptr< tf2_ros::StaticTransformBroadcaster >(new tf2_ros::StaticTransformBroadcaster());
	} else {
		transform_broadcaster_ptr_ = boost::shared_ptr< tf2_ros::TransformBroadcaster >(new tf2_ros::TransformBroadcaster());
	}

	private_node_handle_->param("default_source_frame", default_source_frame_, std::string("default_source_frame"));
	private_node_handle_->param("default_target_frame", default_target_frame_, std::string("default_target_frame"));
	transform_stamped_.child_frame_id = default_source_frame_;
	transform_stamped_.header.frame_id = default_target_frame_;
	transform_stamped_.header.seq = 0;
	transform_stamped_.transform.translation.x = 0;
	transform_stamped_.transform.translation.y = 0;
	transform_stamped_.transform.translation.z = 0;
	transform_stamped_.transform.rotation.x = 0;
	transform_stamped_.transform.rotation.y = 0;
	transform_stamped_.transform.rotation.z = 0;
	transform_stamped_.transform.rotation.w = 1;

	std::string socket_message_type;
	private_node_handle_->param("socket_message_type", socket_message_type, std::string("TransformStamped"));
	if (socket_message_type == "PointTranslation") {
		socket_message_type_ = PointTranslation;
	} else {
		socket_message_type_ = TransformStamped;
	}

	private_node_handle_->param("use_incremental_transform_for_point_translation", use_incremental_transform_for_point_translation_, false);
	private_node_handle_->param("approximate_point_translation_to_axis", approximate_point_translation_to_axis_, false);
	private_node_handle_->param("use_boost_to_parse_point_translation_message", use_boost_to_parse_point_translation_message_, false);
	private_node_handle_->param("use_raw_sockets", use_raw_sockets_, false);
	private_node_handle_->param("use_raw_sockets_as_server", use_raw_sockets_as_server_, false);
	if (use_raw_sockets_as_server_) {
		transform_socket_server_host_ = "*";
	}
}


void SocketToTF::startPublishingTFFromSocket() {
	ros::Time::waitForValid();

	if (transform_socket_server_port_data_ >= 0) {
		zmq::context_t context(1);
		zmq::socket_t subscriber(context, use_raw_sockets_ ? ZMQ_STREAM : ZMQ_SUB);
		std::stringstream ss_connection_;
		ss_connection_ << transform_socket_server_protocol_ << "://" << transform_socket_server_host_ << ":" << transform_socket_server_port_data_;
		if (use_raw_sockets_as_server_) {
			subscriber.bind(ss_connection_.str().c_str());
		} else {
			subscriber.connect(ss_connection_.str().c_str());
		}

		if (!use_raw_sockets_) {
			subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
		}

		if (!use_raw_sockets_ && transform_socket_server_port_sync_>= 0 && !syncWithPublisher(context)) {
			ROS_ERROR("Could not sync with the server!");
			return;
		}

		while (ros::ok()) {
			zmq::message_t peer_id;
			zmq::message_t message;

			ROS_INFO_STREAM("Listening for data using [" << ss_connection_.str() <<"]");
			if (use_raw_sockets_) { if (!subscriber.recv(&peer_id)) { ROS_WARN("Failed to receive peer_id"); continue; } }

			if (!subscriber.recv(&message)) {
				if (use_raw_sockets_) {
					ROS_WARN_STREAM("Failed to receive message from peer with id " << std::string(static_cast<char*>(peer_id.data()), peer_id.size()));
				}
				continue;
			}

			if (message.size() == 0) {
				if (use_raw_sockets_) {
//					if (!subscriber.send(peer_id, ZMQ_SNDMORE)) { ROS_WARN_STREAM("Failed to send receive ack to peer with id " << std::string(static_cast<char*>(peer_id.data()), peer_id.size())); }
					ROS_INFO_STREAM("Received empty message from peer with id " << std::string(static_cast<char*>(peer_id.data()), peer_id.size()));
				}
				continue;
			}

			bool transform_updated = false;
			if (socket_message_type_ == TransformStamped) {
				transform_updated = updateTransformFromSocketTransform(message);
			} else {
				transform_updated = updateTransformFromSocketPointTranslation(message);
			}

			if (transform_updated) {
				transform_stamped_.header.seq = number_published_msgs_++;
				if (transform_stamped_.header.stamp.toSec() == 0.0) {
					transform_stamped_.header.stamp = ros::Time::now();
				}

				if (static_transform_broadcaster_ptr_) {
					static_transform_broadcaster_ptr_->sendTransform(transform_stamped_);
				} else if (transform_broadcaster_ptr_) {
					transform_broadcaster_ptr_->sendTransform(transform_stamped_);
				}
				/*if (use_raw_sockets_) {
					if (!subscriber.send(peer_id, ZMQ_SNDMORE)) {
						ROS_WARN_STREAM("Failed to send receive ack to peer with id " << std::string(static_cast<char*>(peer_id.data()), peer_id.size()));
					}
				}*/
			}
		}

		subscriber.close();
		context.close();
	} else {
		ROS_ERROR_STREAM("TCP data port must be > 0 [ sync port: " << transform_socket_server_port_sync_ << " | data port: " << transform_socket_server_port_data_ << " ]");
	}
}

bool SocketToTF::syncWithPublisher(zmq::context_t& context) {
	if (transform_socket_server_port_sync_ >= 0) {
		ROS_INFO("Syncing with the publisher");
		zmq::socket_t syncclient(context, ZMQ_REQ);
		std::stringstream ss_connection_sync_;
		ss_connection_sync_ << transform_socket_server_protocol_ << "://" + transform_socket_server_host_ << ":" << transform_socket_server_port_sync_;
		syncclient.connect(ss_connection_sync_.str().c_str());

		zmq::message_t send_message(0);
		if (!syncclient.send(send_message)) { syncclient.close(); return false; }

		zmq::message_t recv_message;
		bool recv_status = syncclient.recv(&recv_message);
		syncclient.close();
		ROS_INFO("Syncing finished");
		return recv_status;
	} else {
		return false;
	}
}


bool SocketToTF::updateTransformFromSocketTransform(zmq::message_t& message) {
	socket_to_tf::TransformStamped transform;
	boost::iostreams::basic_array_source<char> source((char*)message.data(), message.size());
	boost::iostreams::stream<boost::iostreams::basic_array_source <char> > input_stream(source);
	boost::archive::binary_iarchive ia(input_stream);
	ia >> transform;

	if (boost::math::isfinite(transform.x) && boost::math::isfinite(transform.y) && boost::math::isfinite(transform.z) &&
			boost::math::isfinite(transform.qx) && boost::math::isfinite(transform.qy) && boost::math::isfinite(transform.qz) && boost::math::isfinite(transform.qw)) {
		transform_stamped_.transform.translation.x = transform.x;
		transform_stamped_.transform.translation.y = transform.y;
		transform_stamped_.transform.translation.z = transform.z;
		transform_stamped_.transform.rotation.x = transform.qx;
		transform_stamped_.transform.rotation.y = transform.qy;
		transform_stamped_.transform.rotation.z = transform.qz;
		transform_stamped_.transform.rotation.w = transform.qw;
		transform_stamped_.child_frame_id = transform.source_frame;
		transform_stamped_.header.frame_id = transform.target_frame;
		transform_stamped_.header.stamp.sec = transform.timestamp_seconds;
		transform_stamped_.header.stamp.nsec = transform.timestamp_nanoseconds;

		std::stringstream ss_data;
		ss_data << "{ " << transform.x << " " << transform.y << " " << transform.z << " " << transform.qx << " " << transform.qy << " " << transform.qz << " " << transform.qw << " " << transform.source_frame << " " << transform.target_frame << " }";
		std::string transform_data = ss_data.str();
		ROS_INFO_STREAM("Received message with size " << message.size() << ": " << transform_data);

		return true;
	}

	return false;
}


bool SocketToTF::updateTransformFromSocketPointTranslation(zmq::message_t& message) {
	socket_to_tf::PointTranslation point_translation;

	if (use_boost_to_parse_point_translation_message_) {
		boost::iostreams::basic_array_source<char> source((char*)message.data(), message.size());
		boost::iostreams::stream<boost::iostreams::basic_array_source <char> > input_stream(source);
		boost::archive::binary_iarchive ia(input_stream);
		ia >> point_translation;
	} else {
		size_t expected_msg_size = sizeof (socket_to_tf::PointTranslation);
		if (message.size() != expected_msg_size) { return false; }
		std::memcpy((void*)&point_translation.x_start, (void*)((char*)message.data() + 0 * sizeof (int16_t)), sizeof (int16_t));
		std::memcpy((void*)&point_translation.y_start, (void*)((char*)message.data() + 1 * sizeof (int16_t)), sizeof (int16_t));
		std::memcpy((void*)&point_translation.z_start, (void*)((char*)message.data() + 2 * sizeof (int16_t)), sizeof (int16_t));
		std::memcpy((void*)&point_translation.x_end,   (void*)((char*)message.data() + 3 * sizeof (int16_t)), sizeof (int16_t));
		std::memcpy((void*)&point_translation.y_end,   (void*)((char*)message.data() + 4 * sizeof (int16_t)), sizeof (int16_t));
		std::memcpy((void*)&point_translation.z_end,   (void*)((char*)message.data() + 5 * sizeof (int16_t)), sizeof (int16_t));
	}

	if (approximate_point_translation_to_axis_) {
		approximatePointTranslationToAxis(point_translation);
	}

	if (boost::math::isfinite(point_translation.x_start) && boost::math::isfinite(point_translation.y_start) && boost::math::isfinite(point_translation.z_start) &&
			boost::math::isfinite(point_translation.x_end) && boost::math::isfinite(point_translation.y_end) && boost::math::isfinite(point_translation.z_end)) {
		if (use_incremental_transform_for_point_translation_) {
			transform_stamped_.transform.translation.x = 0;
			transform_stamped_.transform.translation.y = 0;
			transform_stamped_.transform.translation.z = 0;
		}
		transform_stamped_.transform.translation.x += point_translation.x_end * 0.0001 - point_translation.x_start * 0.0001;
		transform_stamped_.transform.translation.y += point_translation.y_end * 0.0001 - point_translation.y_start * 0.0001;
		transform_stamped_.transform.translation.z += point_translation.z_end * 0.0001 - point_translation.z_start * 0.0001;

		transform_stamped_.header.seq = number_published_msgs_++;
		transform_stamped_.header.stamp.sec = 0;
		transform_stamped_.header.stamp.nsec = 0;

		std::stringstream ss_data;
		ss_data << "{ " << point_translation.x_start << " " << point_translation.y_start << " " << point_translation.z_start << " " << point_translation.x_end << " " << point_translation.y_end << " " << point_translation.z_end << " }";
		std::string point_translation_data = ss_data.str();
		ROS_INFO_STREAM("Received message with size " << message.size() << ": " << point_translation_data);

		return true;
	}

	return false;
}


void SocketToTF::approximatePointTranslationToAxis(socket_to_tf::PointTranslation& point_translation) {
	double dx = point_translation.x_end - point_translation.x_start;
	double dy = point_translation.y_end - point_translation.y_start;
	double dz = point_translation.z_end - point_translation.z_start;
	point_translation.x_start = 0;
	point_translation.y_start = 0;
	point_translation.z_start = 0;
	point_translation.x_end = 0;
	point_translation.y_end = 0;
	point_translation.z_end = 0;
	if (dx > dy && dx > dz) { point_translation.x_end = dx; }
	if (dy > dx && dy > dz) { point_translation.y_end = dy; }
	if (dz > dx && dz > dy) { point_translation.z_end = dz; }
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

} /* namespace socket_to_tf */
