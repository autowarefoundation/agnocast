#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"

class NoRclcppServer : public agnocast::Node
{
  using ServiceT = agnocast_sample_interfaces::srv::SumIntArray;
  using RequestT = agnocast::Service<ServiceT>::RequestT;
  using ResponseT = agnocast::Service<ServiceT>::ResponseT;

  typename agnocast::Service<ServiceT>::SharedPtr service_;

public:
  explicit NoRclcppServer() : Node("no_rclcpp_server")
  {
    service_ = this->create_service<ServiceT>(
      "sum_int_array", [this](
                         const agnocast::ipc_shared_ptr<RequestT> & request,
                         agnocast::ipc_shared_ptr<ResponseT> & response) {
        response->sum = 0;
        for (int64_t value : request->data) {
          response->sum += value;
        }
        RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", response->sum);
      });
  }
};

int main(int argc, char * argv[])
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<NoRclcppServer>();
  executor.add_node(node);
  executor.spin();
  // TODO(Koichi98): Add agnocast::shutdown() here
  return 0;
}
