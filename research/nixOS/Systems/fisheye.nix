{ config, pkgs, ... }:

{
  sdImage = {
    imageName = "sky360-fisheye-pi4.img";
    #compressImage = false;
  };
  networking.hostName = "fisheye";
  environment.systemPackages = with pkgs; [
    indi-full
  ];
}
