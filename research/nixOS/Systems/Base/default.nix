{
  config,
  pkgs,
  ...
}: {
  imports = [
    ./enviornment.nix
    ./debugging.nix
    ./docker.nix
    ./Users
  ];

  programs.fish.enable = true;

  # networking.networkmanager.enable = true;

  services.openssh.enable = true;

  system.stateVersion = "23.11";
}
