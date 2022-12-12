{
  inputs = {
    nixpkgs.url = "nixpkgs/nixos-unstable";
    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs = { self, nixpkgs, nixos-generators, ... }: {
    # the installation media is also the installation target,
    # so we don't want to provide the installation configuration.nix.
    packages = {
      aarch64 = {
        # tracker
        sentinal = nixos-generators.nixosGenerate {
          system = "aarch64-linux";
          modules = [
            Hardware/nvidia-xavier-agx.nix
            (Systems/Base)
            Systems/sentinal.nix
          ];
          format = "sd-aarch64-installer";
        };
        # fish eye camera
        fisheye = nixos-generators.nixosGenerate {
          system = "aarch64-linux";
          modules = [
            Hardware/rpi-4.nix
            (Systems/Base)
            Systems/fisheye.nix
          ];
          format = "sd-aarch64-installer";
        };
      };
      x86_64-linux = {
        sentinal = nixos-generators.nixosGenerate {
          system = "x86_64-linux";
          modules = [
            Hardware/x86-64-linux.nix
            (System/Base)
            Systems/sentinal.nix
          ];
          format = "raw-efi";
        };
        # fish eye camera
        fisheye = nixos-generators.nixosGenerate {
          system = "aarch64-linux";
          modules = [
            Hardware/rpi-4.nix
            (Systems/Base)
            Systems/sentinal.nix
          ];
          format = "sd-aarch64";
        };
      };
    };
  };
}
