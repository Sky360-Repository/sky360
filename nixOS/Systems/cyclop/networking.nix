{ config, pkgs, ... }:
{
  networking.firewall.allowedTCPPorts = [ 8080 ]; # 45563 - obs-teleport
  
}