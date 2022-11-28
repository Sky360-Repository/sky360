{ config, pkgs, ... }:

{
  networking.hostName = "allskycamera";
  environment.systemPackages = with pkgs; [
    dump1090
  ];
}
