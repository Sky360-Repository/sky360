{ pkgs, ... }:
{
  services.ros2 = {
    enable = true;
    distro = "iron";
  };

  # systemPackages = with pkgs; [ ros2cli ros2run gazebo ros2launch ros-environment ];
}
