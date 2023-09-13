{
  config,
  pkgs,
  lib,
  ...
}: {
  environment.systemPackages = with pkgs; [
    indi-full
  ];

  nixpkgs.config.allowUnfreePredicate = pkg:
    builtins.elem (lib.getName pkg) [
      "qhyccd_sdk"
    ];
  services.udev.packages = [pkgs.qhyccd_sdk];
}
