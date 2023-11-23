{ config
, pkgs
, ...
}: {
  imports = [
    ./nix.nix
    ./enviornment.nix
    ./debugging.nix
    ./ros2.nix
    ./docker.nix
    ./Users
  ];

  programs.fish.enable = true;

  # networking.networkmanager.enable = true;
  # boot.binfmt.emulatedSystems = [ "aarch64-linux" ];
  services.openssh = {
    enable = true;
    settings.PasswordAuthentication = false; # disable ssh with password
  };

  # system.stateVersion = "23.11";
}
