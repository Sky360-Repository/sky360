{ pkgs, ... }:
{
  environment.systemPackages = with pkgs; [
    rosPackages.iron.sky360-ros-iron-heartbeat
  ];

  services.ros2 = {
    enable = true;
    distro = "iron";
    domainId = 0;
  
    nodes = {
      heartbeat = {
        package = pkgs.rosPackages.iron.sky360-ros-iron-heartbeat.pname; 
        node = "heartbeat";
        # args = [];
        # rosArgs = [];
        # params = { };  
      };
      listener = {
        package = pkgs.rosPackages.iron.sky360-ros-iron-heartbeat.pname; 
        node = "listener";
      };

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
