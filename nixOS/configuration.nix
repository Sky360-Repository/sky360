{ config, pkgs, ... }:

{
  imports =
    [
      ./Systems/rpi-4
      ./Systems/allskycamera.nix
      ./enviornment.nix
      ./Users
    ];

  programs.fish.enable = true;

  networking.networkmanager.enable = true;

  services.openssh.enable = true;

  system.stateVersion = "22.11";
}
