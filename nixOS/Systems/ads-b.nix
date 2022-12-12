{ config, pkgs, ... }:

{
  environment.systemPackages = with pkgs; [
    dump1090
  ];
}
