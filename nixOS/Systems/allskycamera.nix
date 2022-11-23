{ config, pkgs, ... }:

{
  networking.hostName = "allskycamera";
  environment.systemPackages = with pkgs; [
    indi-full
  ];
}
