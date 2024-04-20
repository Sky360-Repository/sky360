{ pkgs, ... }:
{
  environment.systemPackages = with pkgs; [
    sky360.sky360-ros-iron-heartbeat
    python3Packages.pyaml # needed for ros2
  ];

  services.ros2 = {
    enable = true;
    distro = "iron";
    domainId = 0;
    systemPackages = p: with p; [ ros2cli ros2run ];
    # # pkgs = 
    nodes = {
      heartbeat = {
        package = pkgs.sky360.sky360-ros-iron-heartbeat.pname;
        node = "heartbeat";
        args = [ ];
        rosArgs = [ ];
        params = { };
      };
      # listener = {
      #   package = pkgs.sky360.sky360-ros-iron-heartbeat.pname;
      #   node = "listener";
      # };

      # talker = {
      #   package = "demo_nodes_cpp";
      #   node = "talker";
      # };
      # listener2 = {
      #   package = "demo_nodes_cpp";
      #   node = "listener";
      # };
    };
  };
}
