{ config, pkgs, ... }:

{
  imports =
    [
      ./snick.nix
    ];

  users.users.sky360 = {
    isNormalUser = true;
    home = "/home/sky360";
    initialPassword = "sky360";
    extraGroups = [ "networkmanager" "wheel" "video" "audio" "plugdev" "dialout" ];
    #packages = with pkgs; [
    #];
  };
}
