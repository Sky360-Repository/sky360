{ dream2nix, ... }: {
  imports = [
    dream2nix.modules.dream2nix.nodejs-package-json-v3
    dream2nix.modules.dream2nix.nodejs-package-lock-v3
    dream2nix.modules.dream2nix.nodejs-granular-v3
  ];

  name = "openmct";
  version = "0.1.0";

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
