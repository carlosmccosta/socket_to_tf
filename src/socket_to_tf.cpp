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
	private_node_handle_->param("server_port_sync", transform_socket_server_port_sync_, 1331);
	private_node_handle_->param("server_port_data", transform_socket_server_port_data_, 1337);
}


void SocketToTF::startPublishingTFFromSocket() {
	ros::Time::waitForValid();

	if (transform_socket_server_port_sync_ >= 0 && transform_socket_server_port_data_ >= 0) {
		zmq::context_t context(1);
		zmq::socket_t subscriber(context, ZMQ_SUB);
		std::stringstream ss_connection_data_;
		ss_connection_data_ << transform_socket_server_protocol_ << "://" << transform_socket_server_host_ << ":" << transform_socket_server_port_data_;
		subscriber.connect(ss_connection_data_.str().c_str());
		subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

		if (!syncWithPublisher(context)) {
			ROS_ERROR("Could not sync with the server!");
			return;
		}

		while (ros::ok()) {
			zmq::message_t message;
			std::cout << "Listening for data using [" << ss_connection_data_.str() <<"]" << std::endl;
			subscriber.recv(&message);
			std::cout << "Received message with size " << message.size() << ": " << (std::string(static_cast<char*>(message.data()), message.size())) << std::endl;

//			boost::math::isfinite()
//			transform_broadcaster_.sendTransform(transform_stamped_);
		}
		subscriber.close();
		context.close();
	} else {
		ROS_ERROR_STREAM("TCP ports must be > 0 [ sync port: " << transform_socket_server_port_sync_ << " | data port: " << transform_socket_server_port_data_ << " ]");
	}
}

bool SocketToTF::syncWithPublisher(zmq::context_t& context) {
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
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

} /* namespace socket_to_tf */
