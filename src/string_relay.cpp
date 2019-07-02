/**\file string_relay.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <string_relay/string_relay.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace string_relay {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void StringRelay::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	node_handle_ = node_handle;
	private_node_handle_ = private_node_handle;
	private_node_handle_->param("string_topic", string_topic_, std::string("message"));
	private_node_handle_->param("server_protocol", socket_server_protocol_, std::string("tcp"));
	private_node_handle_->param("server_host", socket_server_host_, std::string("localhost"));
	private_node_handle_->param("server_port_sync", socket_server_port_sync_, -1);
	private_node_handle_->param("server_port_data", socket_server_port_data_, 1337);

	private_node_handle_->param("use_raw_sockets", use_raw_sockets_, true);
	private_node_handle_->param("use_raw_sockets_as_server", use_raw_sockets_as_server_, true);
	if (use_raw_sockets_as_server_) {
		socket_server_host_ = "*";
	}

	string_publisher_ = node_handle->advertise<std_msgs::String>(string_topic_, 5, true);
}


void StringRelay::startPublishingStringFromSocket() {
	ros::Time::waitForValid();

	if (socket_server_port_data_ >= 0) {
		zmq::context_t context(1);
		zmq::socket_t subscriber(context, use_raw_sockets_ ? ZMQ_STREAM : ZMQ_SUB);
		std::stringstream ss_connection;
		ss_connection << socket_server_protocol_ << "://" << socket_server_host_ << ":" << socket_server_port_data_;
		if (use_raw_sockets_as_server_) {
			subscriber.bind(ss_connection.str().c_str());
		} else {
			subscriber.connect(ss_connection.str().c_str());
		}

		if (!use_raw_sockets_) {
			subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
		}

		if (!use_raw_sockets_ && socket_server_port_sync_>= 0 && !syncWithPublisher(context)) {
			ROS_ERROR("Could not sync with the server!");
			return;
		}

		while (ros::ok()) {
			zmq::message_t peer_id;
			zmq::message_t message;

			ROS_DEBUG_STREAM("Listening for data using [" << ss_connection.str() <<"]");
			if (use_raw_sockets_) {
				if (!subscriber.recv(&peer_id)) {
					ROS_WARN("Failed to receive peer_id");
					continue;
				}
			}

			if (!subscriber.recv(&message)) {
				if (use_raw_sockets_) {
					ROS_WARN_STREAM("Failed to receive message from peer with id " << std::string(static_cast<char*>(peer_id.data()), peer_id.size()));
				}
				continue;
			}

			if (message.size() != 0) {
        publishStringFromMessage(message);
			}
		}

		subscriber.close();
		context.close();
	} else {
		ROS_ERROR_STREAM("TCP data port must be > 0 [ sync port: " << socket_server_port_sync_ << " | data port: " << socket_server_port_data_ << " ]");
	}
}


void StringRelay::publishStringFromMessage(zmq::message_t &message) {
	std_msgs::String msg;
	msg.data = std::string((char*)message.data(), message.size());
  string_publisher_.publish(msg);
  ROS_DEBUG_STREAM("Received message: " << msg.data);
}


bool StringRelay::syncWithPublisher(zmq::context_t& context) {
	if (socket_server_port_sync_ >= 0) {
		ROS_INFO("Syncing with the publisher");
		zmq::socket_t syncclient(context, ZMQ_REQ);
		std::stringstream ss_connection_sync_;
		ss_connection_sync_ << socket_server_protocol_ << "://" + socket_server_host_ << ":" << socket_server_port_sync_;
		syncclient.connect(ss_connection_sync_.str().c_str());

		zmq::message_t send_message(0);
		if (!syncclient.send(send_message)) { syncclient.close(); return false; }

		zmq::message_t recv_message;
		bool recv_status = syncclient.recv(&recv_message);
		syncclient.close();
		ROS_INFO("Syncing with ZMQ publisher finished");
		return recv_status;
	} else {
		return false;
	}
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

} /* namespace string_relay */
