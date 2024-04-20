{ dream2nix, ...}:
{
  imports = [ 
    dream2nix.modules.dream2nix.rust-cargo-lock
    dream2nix.modules.dream2nix.buildRustPackage
  ];  

  mkDerivation = { src = ./.; };

  deps = {nixpkgs, ...}: {
    inherit
      (nixpkgs)
      stdenv;
  };

  name = "ros_serialport_driver";
  version = "0.1.0";
}