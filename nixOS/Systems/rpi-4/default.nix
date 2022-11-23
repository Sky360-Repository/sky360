{ config, pkgs, ... }:

{
  imports =
    [
      ./hardware-configuration.nix
      ./boot.nix
    ];
}
