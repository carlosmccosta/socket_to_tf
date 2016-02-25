/**\file socket_to_tf_sender.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <zmq.hpp>
#include <socket_to_tf/transform_stamped.h>
#include <unistd.h>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
void wait_for_subscribers(zmq::context_t& context, int sync_port, int min_number_of_subscribers = 1, const std::string& server_protocol = std::string("tcp")) {
	if (sync_port >= 0) {
		zmq::socket_t syncclient(context, ZMQ_REP);
		std::stringstream ss_connection_sync_;
		ss_connection_sync_ << server_protocol << "://*:" << sync_port;
		syncclient.bind(ss_connection_sync_.str().c_str());

		int number_of_connected_subscribers = 0;
		while (number_of_connected_subscribers < min_number_of_subscribers) {
			zmq::message_t recv_message;
			if (syncclient.recv(&recv_message)) {
				zmq::message_t empty_send_message(0);
				if (syncclient.send(empty_send_message)) {
					++number_of_connected_subscribers;
				}
			}
		}
	} else {
		usleep(200000);
	}
}


int main(int argc, char** argv) {
	// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		// parameters
	if (argc < 12 || argc > 15) {
		std::cerr << "Usage: socket_transform_publisher server_port_sync server_port_data x y z qx qy qz qw source_frame target_frame [number_of_retransmissions] [min_number_of_subscribers] [server_protocol]" << std::endl;
		return 1;
	}

	int number_of_retransmissions = 1;
	if (argc > 12) { number_of_retransmissions = atoi(argv[12]); }

	int min_number_of_subscribers = 1;
	if (argc > 13) { min_number_of_subscribers = atoi(argv[13]); }

	std::string server_protocol("tcp");
	if (argc > 14) { server_protocol = std::string(argv[14]); }



	// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// setup connection
	zmq::context_t context;
	zmq::socket_t publisher(context, ZMQ_PUB);
	std::stringstream ss_bind;
	ss_bind << server_protocol << "://*:" << argv[2];
	publisher.bind(ss_bind.str().c_str());
	wait_for_subscribers(context, atoi(argv[1]), min_number_of_subscribers, server_protocol);



	// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// send data
	std::stringstream ss_data;
	ss_data << "{ " << argv[3] << " " << argv[4] << " " << argv[5] << " " << argv[6] << " " << argv[7] << " " << argv[8] << " " << argv[9] << " " << argv[10] << " " << argv[11] << " }";
	std::string transform_data = ss_data.str();

	socket_to_tf::TransformStamped transform;
	transform.x = atof(argv[3]);
	transform.y = atof(argv[4]);
	transform.z = atof(argv[5]);
	transform.qx = atof(argv[6]);
	transform.qy = atof(argv[7]);
	transform.qz = atof(argv[8]);
	transform.qw = atof(argv[9]);
	transform.source_frame = argv[10];
	transform.target_frame = argv[11];
	transform.timestamp_seconds = 0;
	transform.timestamp_nanoseconds = 0;

	std::vector<char> buffer;
	boost::iostreams::stream<boost::iostreams::back_insert_device< std::vector<char> > > output_stream(buffer);
	boost::archive::binary_oarchive oa(output_stream);
	oa << transform;
	output_stream.flush();

	for (int i = 0; i < number_of_retransmissions; ++i) {
		zmq::message_t message(buffer.size());
		std::memcpy(message.data(), buffer.data(), buffer.size());
		std::cout << "Sending message with size " << buffer.size() << " using [" << ss_bind.str() << "]: " << transform_data << std::endl;
		if (publisher.send(message)) { std::cout << "Successfully sent data!" << std::endl; }
	}

	return 0;
}
// ###################################################################################   </main>   #############################################################################
