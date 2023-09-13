{
  config,
  pkgs,
  lib,
  ...
}: {
  # networking.hostName = "cyclop";
  environment.systemPackages = with pkgs; [
    indi-full
  ];

  nixpkgs.config.allowUnfreePredicate = pkg:
    builtins.elem (lib.getName pkg) [
      # Add additional package names here
      "qhyccd_sdk"
    ];
  services.udev.packages = [ pkgs.qhyccd_sdk ];
}
