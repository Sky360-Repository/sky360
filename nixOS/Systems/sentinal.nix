{ config, pkgs, ... }:

{
  networking.hostName = "sentinal";
  environment.systemPackages = with pkgs; [

  ];
}
