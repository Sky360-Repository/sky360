{ config, pkgs, ... }:

{
  users.users.snick = {
    isNormalUser = true;
    extraGroups = [ "wheel" ];
    #packages = with pkgs; [
    #];
    shell = pkgs.fish;
    openssh.authorizedKeys.keyFiles = [ ./snick_authorized_keys ];
  };
}
