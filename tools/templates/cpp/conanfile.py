from conan import ConanFile
from conan.tools.files import copy

class RobotOpsConfigConan(ConanFile):
    name = "robotops-config"
    version = "{{VERSION}}"
    license = "Apache-2.0"
    url = "https://github.com/RobotOpsInc/robotops_config"
    description = "Generated Protobuf configuration types for RobotOps distributed tracing"
    topics = ("robotics", "ros2", "tracing", "protobuf")
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "*.h", "*.hpp", "*.cc"

    def requirements(self):
        self.requires("protobuf/6.32.1")

    def package(self):
        copy(self, "*.h", src=self.source_folder, dst=f"{self.package_folder}/include")
        copy(self, "*.hpp", src=self.source_folder, dst=f"{self.package_folder}/include")
        copy(self, "*.cc", src=self.source_folder, dst=f"{self.package_folder}/src")

    def package_info(self):
        self.cpp_info.libs = []
        self.cpp_info.includedirs = ["include"]
