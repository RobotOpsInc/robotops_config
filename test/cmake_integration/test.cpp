#include <robotops/config/v1/config.pb.h>
#include <iostream>

int main() {
    // Test that we can create a protobuf message
    robotops::config::v1::Config config;
    config.set_version("1.0.0");

    // Test that protobuf headers are available
    std::cout << "Protobuf version: " << GOOGLE_PROTOBUF_VERSION << std::endl;
    std::cout << "Config version: " << config.version() << std::endl;

    return 0;
}
