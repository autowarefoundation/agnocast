// agnocast_callback_info.hpp | register_callback()
inline register_callback(callback_group_) {
	byte callback_info_id;
	d_step {
		callback_info_id = next_callback_info_id;
		next_callback_info_id++;
	}
	
	lock(id2_callback_info_mtx);
	id2_callback_info[callback_info_id].initialized = true;
	id2_callback_info[callback_info_id].callback_group = callback_group_;
	unlock(id2_callback_info_mtx);
	
	need_epoll_updates = true
}

// agnocast_subscription.hpp | class Subscription
proctype AgnocastSubscription(byte callback_group) {
	register_callback(callback_group);
}

inline agnocast_timer_callback() {
	// agnocast_publisher.cpp | publish_core()
	byte cb_info_i;short cb_info_indices[NUM_SUBSCRIPTIONS];byte len_indices = 0;
	d_step {
		for (cb_info_i : 0 .. NUM_SUBSCRIPTIONS - 1) {
			if
			:: id2_callback_info[cb_info_i].initialized -> cb_info_indices[len_indices] = cb_info_i; len_indices++
			:: else
			fi
		}

		num_agnocast_published++;
		// The subscription callbacks registered after this line will not be triggered.
	}
	
	byte loop_i;
	for (loop_i : 0 .. len_indices - 1) {
		cb_info_i = cb_info_indices[loop_i];
		if
		:: epoll_added[cb_info_i] ->
			entry_num[cb_info_i]++;// Simplification: This corresponds to `ioctl(agnocast_fd,AGNOCAST_PUBLISH_MSG_CMD,&publish_msg_args)`.
			epoll!cb_info_i// eventfd signal for publish
		:: else
		fi
	}
	
	atomic {
		num_completed_cbs++
		printf("Agnocast | finish agnocast_timer_callback()\n");
	}
}

// agnocast_publisher.cpp | class Publisher
proctype AgnocastPublisher() {
	Callback timer_cb;
	d_step {
		timer_cb.id = BYTE_MAX;
		timer_cb.type = AGNOCAST_TIMER;
	}
	
	byte pub_i;
	for (pub_i : 0 .. NUM_PUBLISH - 1) {
		ready_executables!timer_cb;
		skip// Simulate period
	}
}

// agnocast_executor.cpp | AgnocastExecutor::receive_message()
inline receive_message(callback_info_id) {
	byte ret_entry_num;
	d_step{
		ret_entry_num = entry_num[callback_info_id];
		entry_num[callback_info_id] = 0;
	}
	
	byte entry_i;
	for (entry_i : 0 .. ret_entry_num - 1) {
		Callback sub_cb;
		d_step {
			sub_cb.id = callback_info_id;
			sub_cb.type = SUBSCRIPTION;
		}
		
		lock(ready_agnocast_executables_mutex_);
		ready_agnocast_executables!sub_cb;
		unlock(ready_agnocast_executables_mutex_);
	}
}

// agnocast_executor.cpp | AgnocastExecutor::wait_and_handle_epoll_event()
inline wait_and_handle_epoll_event() {
	byte callback_info_id;
	
	skip;// Simulate blocking with timeout
	if
	:: d_step {epoll?[callback_info_id] -> epoll?callback_info_id;}
	:: else -> goto finish_wait_and_handle_epoll_event// timeout
	fi
	
	// Only the `callback_info_id` is needed in this model.
	receive_message(callback_info_id);
	
	finish_wait_and_handle_epoll_event:
}

inline get_next_ready_agnocast_executable(ret_cb,ret_result) {
	lock(ready_agnocast_executables_mutex_);
	if
	:: d_step {ready_agnocast_executables?[ret_cb] -> ready_agnocast_executables?ret_cb;} ret_result = true
	:: else -> ret_result = false
	fi
	unlock(ready_agnocast_executables_mutex_);
}

inline get_next_agnocast_executable(ret_cb,ret_result) {
	get_next_ready_agnocast_executable(ret_cb,ret_result);
	if
	:: ret_result -> goto finish_get_next_agnocast_executable
	:: else
	fi
	
	wait_and_handle_epoll_event();
	
	// Try again
	get_next_ready_agnocast_executable(ret_cb,ret_result);
	
	finish_get_next_agnocast_executable:
}

inline execute_agnocast_executable(cb) {
	if
	:: d_step{cb.type == SUBSCRIPTION -> printf("Agnocast | subscription_callback(): callback_info_id = %d\n",cb.id);} subscription_callback(cb.id)
	:: else -> assert(false)
	fi
}

inline validate_callback_group(callback_group_) {
	assert(callback_group_ != -1);

	// TODO(CIE): check dedicated_callback_group_
}

// agnocast_executor.cpp | AgnocastExecutor::prepare_epoll()
inline prepare_epoll() {
	byte cb_info_i;

	lock(id2_callback_info_mtx);
	for (cb_info_i : 0 .. NUM_SUBSCRIPTIONS - 1) {
		if
		:: !id2_callback_info[cb_info_i].initialized || id2_callback_info[cb_info_i].need_epoll_update == false -> goto continue_loop
		:: else
		fi
		
		validate_callback_group(id2_callback_info[cb_info_i].callback_group);
		
		d_step{
			epoll_added[cb_info_i] = true;// epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ...);
			expected_num_completed_cbs = expected_num_completed_cbs + NUM_PUBLISH - num_agnocast_published;
			printf("Agnocast | agnocast subscription is registered: callback_info_id = %d,num_agnocast_published = %d\n",cb_info_i,num_agnocast_published);
		}
		
		// Simplification: The transient local is not modeled.
		
		id2_callback_info[cb_info_i].need_epoll_update = false;
		
		continue_loop:
	}
	
	bool all_updated = true;
	for (cb_info_i : 0 .. NUM_SUBSCRIPTIONS - 1) {
		if
		:: id2_callback_info[cb_info_i].initialized && id2_callback_info[cb_info_i].need_epoll_update -> all_updated = false;break
		:: else
		fi
	}
	
	if
	:: all_updated -> need_epoll_updates = false// `all_updated` is always true for now.
	:: else
	fi
	
	unlock(id2_callback_info_mtx);
}
