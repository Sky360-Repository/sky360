{ lib
, config
, dream2nix
, ...
}: {
  imports = [
    dream2nix.modules.dream2nix.nodejs-package-json-v3
    dream2nix.modules.dream2nix.nodejs-package-lock-v3
    dream2nix.modules.dream2nix.nodejs-granular-v3
  ];

  name = "app";
  version = "0.0.1";

  mkDerivation = {
    src = ./.;
  };

  deps = { nixpkgs, ... }: {
    inherit
      (nixpkgs)
      gnugrep
      stdenv
      ;
  };
}
