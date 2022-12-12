{ config, pkgs, ... }:

{
  imports =
    [
      ./snick.nix
    ];

  users.users.sky360 = {
    initialPassword = "sky360";
    isNormalUser = true;
    extraGroups = [ "wheel" ];
    #packages = with pkgs; [
    #];
  };
}
