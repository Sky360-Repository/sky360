{
  config,
  pkgs,
  ...
}: {
  imports = [
    ./snick.nix
  ];

  users.users.sky360 = {
    isNormalUser = true;
    home = "/home/sky360";
    initialPassword = "sky360";
    extraGroups = ["networkmanager" "video" "audio" "plugdev"];
    #packages = with pkgs; [
    #];
  };
}
