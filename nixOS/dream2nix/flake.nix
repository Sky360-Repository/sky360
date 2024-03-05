{
  description = "My flake with dream2nix packages";

  inputs = {
    nixpkgs.url = "nixpkgs/nixos-unstable";
    dream2nix.url = "github:nix-community/dream2nix";
    # nixpkgs.follows = "dream2nix/nixpkgs";
    dream2nix.inputs.nixpkgs.follows = "nixpkgs";
    # flake-parts.url = "github:hercules-ci/flake-parts";
  };

  outputs =
    inputs @ { self
    , dream2nix
    , nixpkgs
      # , flake-parts
    , ...
    }:
    let
      system = "aarch64-linux"; # TODO: support multi-systems using flake-utils or flake-parts
    in
    {
      packages.${system}.packages = dream2nix.lib.importPackages {
        packageSets.nixpkgs = nixpkgs.legacyPackages.${system};
        projectRoot = ./.;
        projectRootFile = "flake.nix";
        packagesDir = ./packages;
      };
    };
}
