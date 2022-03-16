/*
 * Subscriber.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */

#include <Subscriber.hpp>
#include <iostream>
#include <system_error>
#include <memory>
#include <vector>

using namespace std;
namespace ZMQCommon {

Subscriber::Subscriber():
    topic_(""), address_(""), topiclen_(0) {
	recv_callback_ = nullptr;
	data_ = nullptr;

	context_ = unique_ptr<zmq::context_t>(new zmq::context_t());
}

bool Subscriber::start(string address, string topic) {
	if (thread_) {
		return true;
	}
	terminating_.store(false);
	topic_ = topic;
	address_ = address;
	topiclen_ = topic_.length();
	thread_ = std::make_unique<std::thread>([&]() {
		this->worker(terminating_);
	});
	return true;
}

bool Subscriber::stop() {
	if (!thread_ || !thread_->joinable()) {
		return true;
	}
	
	terminating_.store(true);
	thread_->join();
	thread_.reset();
	//zmq_term(context_.get());
	context_.reset();
	return true;
}

bool Subscriber::isAlive() {
	return thread_ && thread_->joinable() && !terminating_;
}

void Subscriber::worker(std::atomic_bool & terminating) {
	// create socket
	zmq::socket_t socket_(*context_, ZMQ_SUB);
	zmq_pollitem_t items [1];
	int rc = 0;
	int64_t more = 0;
	size_t more_size = sizeof(more);
	zmq::message_t msg;
	//char * buf;
	size_t buf_ptr = 0;

	// setup socket
	try {
		socket_.connect(address_.c_str());
	} catch (exception& e) {
		cerr << "Exception connecting to " << address_ << ", \"" << e.what() << "\"" << endl;
		terminating.store(true);
		return;
	}
  // set linger value to zero
  int linger = 0;
  socket_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
	socket_.setsockopt(ZMQ_SUBSCRIBE,topic_.c_str(), topic_.length());
	// poll items
	items[0].socket = (void*)socket_;
	items[0].events = ZMQ_POLLIN;

	// TODO fix this, findout how to pass the message size from main
	size_t message_size = Subscriber::MaxMessageSize;

	std::vector<char> buf(message_size);

	while (!terminating.load()) {
		try {
			rc = zmq_poll (items, 1, 500); // usec timeout, 500 ms

			if (rc == 0) {
				// no messages
				continue;
			} else if (rc < 0) {
				// error, check
				switch (rc) {
				case ETERM:
					// terminated, exit process
				  //single_thread::notifyTermination();
					continue;
				default:
					//cerr << "Subscriber::process(): zmq_poll exit code " << rc << ". Ignoring." << endl;
					continue;
				}
			}

			socket_.recv(&msg);
			message_size = msg.size();
			memcpy(static_cast<char *>(buf.data() + buf_ptr), static_cast<char *>(msg.data()), message_size);
			buf_ptr += message_size;
			// message with envelope as topic
			socket_.getsockopt(ZMQ_RCVMORE, &more, &more_size);

			if (!more) {
				if (recv_callback_) {
					// notify only if we completely received the message
					recv_callback_(topic_.c_str(), static_cast<char *> (buf.data() + topiclen_), buf_ptr - topiclen_, data_);
				}
				buf_ptr = 0;
			}

		}
		catch (const zmq::error_t& ex) {
            if(ex.num() != ETERM) {
            	cerr << "Subscriber::process(): zmq exception \"" << ex.what() << "\"" << endl;
            }
            continue;
		}
		catch (exception& e) {
			cerr << "Subscriber::process(): exception \"" << e.what() << "\"" << endl;
			continue;
		}

	}
}


Subscriber::~Subscriber() {
	stop();
}

void Subscriber::register_callback(callback_t callback, void* user_data) {
  recv_callback_ = callback;
	data_ = user_data;
}

}
