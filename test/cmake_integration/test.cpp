#include <robotops/config/v1/config.pb.h>
#include <robotops/config/v1/defaults.hpp>
#include <iostream>

int main() {
    // Test that we can create a protobuf message
    robotops::config::v1::Config config;
    config.set_schema_version("0.7.1");

    // Test that defaults.hpp is accessible and functions work
    robotops::config::v1::Config default_config = robotops::config::v1::CreateDefaultConfig();

    // Test that protobuf headers are available
    std::cout << "Protobuf version: " << GOOGLE_PROTOBUF_VERSION << std::endl;
    std::cout << "Schema version: " << config.schema_version() << std::endl;
    std::cout << "Default config schema version: " << default_config.schema_version() << std::endl;
    std::cout << "✅ All headers accessible" << std::endl;

    return 0;
}
