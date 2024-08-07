{ pkgs, lib, ... }:
with pkgs;
with rosPackages.iron;

buildRosPackage {
  pname = "sky360-ros-iron-heartbeat";
  version = "0.1.0";

  src = ./ros;

  buildType = "ament_cmake";
  buildInputs = [
    ament-cmake
    rmw-implementation-cmake
  ];
  checkInputs = [
    ament-cmake-pytest
    ament-lint-auto
    ament-lint-common
    launch
    launch-testing
    launch-testing-ament-cmake
    launch-testing-ros
  ];
  propagatedBuildInputs = [
    example-interfaces
    launch-ros
    launch-xml
    rcl-interfaces
    rclcpp
    rclcpp-components
    rcutils
    rmw
    std-msgs
  ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = ''Sky360 heartbeat ros2 node'';
    license = with lib.licenses; [ mit ];
  };
}
