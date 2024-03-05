{ config
, pkgs
, ...
}: {
  imports = [
    ./nix.nix
    ./enviornment.nix
    (./debugging)
    ./ros2.nix
    # ./docker.nix
    ./Users
  ];

  programs.fish.enable = true;

  documentation.man.enable = false;

  # networking.networkmanager.enable = true;
  # boot.binfmt.emulatedSystems = [ "aarch64-linux" ];
  services.openssh = {
    enable = true;
  };

  # system.stateVersion = "23.11";
}
