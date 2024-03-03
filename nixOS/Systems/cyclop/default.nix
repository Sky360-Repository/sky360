{ pkgs
, lib
, ...
}: {

  imports = [
    ./networking.nix
  ];
  environment.systemPackages = with pkgs; [
    # indi-full
    sky360.qhyccd_sdk
  ];

  # nixpkgs.config.allowUnfreePredicate = pkg:
  #   builtins.elem (lib.getName pkg) [
  #     "sky360.qhyccd_sdk"
  #   ];
  services.udev.packages = [ pkgs.sky360.qhyccd_sdk ];
}
