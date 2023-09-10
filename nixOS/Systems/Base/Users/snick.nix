{ config, pkgs, ... }:

{
  users.users.snick = {
    isNormalUser = true;
    home = "/home/snick";
    extraGroups = [ "networkmanager" "wheel" "video" "audio" "plugdev" "dialout" ];
    shell = pkgs.fish;
    openssh.authorizedKeys.keys = [
      "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIJ7ayPIBqQlTt2uTa0lrkUXC4Ou+rk+MG6fLn2OfATSb snick@sky360.org"
    ];
    #packages = with pkgs; [
    #];
  };
}
