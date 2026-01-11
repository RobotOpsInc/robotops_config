#include <robotops/config/v1/config.pb.h>
#include <iostream>

int main() {
    // Test that we can create a protobuf message
    robotops::config::v1::Config config;
    config.set_schema_version("0.4.13");

    // Test that protobuf headers are available
    std::cout << "Protobuf version: " << GOOGLE_PROTOBUF_VERSION << std::endl;
    std::cout << "Schema version: " << config.schema_version() << std::endl;

    return 0;
}
