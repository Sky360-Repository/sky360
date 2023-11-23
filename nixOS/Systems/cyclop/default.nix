{
  config,
  pkgs,
  lib,
  ...
}: {

  imports = [
    ./networking.nix
  ];
  environment.systemPackages = with pkgs; [
    indi-full
  ];

  nixpkgs.config.allowUnfreePredicate = pkg:
    builtins.elem (lib.getName pkg) [
      "qhyccd_sdk"
    ];
  services.udev.packages = [pkgs.qhyccd_sdk];
}
