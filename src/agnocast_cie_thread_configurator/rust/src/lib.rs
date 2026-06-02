//! Pure-Rust client for the cie_thread_configurator non-ROS thread protocol.
//!
//! Announces a thread's TID and logical name to the configurator daemon over an
//! abstract Unix datagram socket so the daemon can manage its scheduling. Rust
//! counterpart of C++ `spawn_non_ros2_thread` / `send_non_ros_thread_info`.

mod wire;
