{ config
, pkgs
, lib
, user
, ...
}: {
  boot.loader.grub.configurationLimit = 2;
  #boot.tmp.cleanOnBoot = true;

  nix = {
    settings = {
      substituters = [
        #   "https://mirrors.tuna.tsinghua.edu.cn/nix-channels/store"
        "https://cache.nixos.org/"
        "https://nix-community.cachix.org"
        "https://sky360.cachix.org"
        "https://ros.cachix.org"
      ];
      trusted-public-keys = [
        "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
        "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
        "sky360.cachix.org-1:/CVvRgdh1sEvexxzJ91HzFr/Sw3OTXZ7dxY/0VeqZ3c="
      ];
      auto-optimise-store = true; # Optimise syslinks

      # sandbox - an isolated environment for each build process. It is used to remove further hidden dependencies set by the build environment to improve reproducibility.
      sandbox = true;
    };
    gc = {
      automatic = true;
      dates = "weekly";
      options = "--delete-older-than 7d";
    };
    package = pkgs.nixVersions.unstable;
    #registry.nixpkgs.flake = inputs.nixpkgs;
    extraOptions = ''
      experimental-features = nix-command flakes repl-flake configurable-impure-env
      keep-outputs          = true
      keep-derivations      = true
    '';
  };
}
