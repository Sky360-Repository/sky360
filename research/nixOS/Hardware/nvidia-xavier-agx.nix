{ config, lib, pkgs, ... }:
{
  # via https://github.com/anduril/jetpack-nixos
  hardware.nvidia-jetpack.enable = true;
  hardware.nvidia-jetpack.som = "xavier-agx"; # Other options include orin-agx, xavier-nx, and xavier-nx-emmc
  hardware.nvidia-jetpack.carrierBoard = "devkit";
}
