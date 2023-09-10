{
  inputs = {
    nixpkgs.url = "nixpkgs/nixos-unstable";
    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    # nixos-rk3588.url = "github:ryan4yin/nixos-rk3588";
    nixos-rk3588.url = "github:realsnick/nixos-rk3588";
    #nixos-rk3588.url = "/home/snick/Code/github/nixos-rk3588";

    # mesa-panfork = {
    # url = "gitlab:panfork/mesa/csf";
    # flake = false;
    # };
  };
  outputs = inputs @ {
    self,
    nixpkgs,
    nixos-generators,
    nixos-rk3588,
    # mesa-panfork,
    ...
  }: let
    # system = "aarch64-linux";
  in {
    nixosConfigurations = {
      cyclop-orange_pi_5_plus = import "${nixpkgs}/nixos/lib/eval-config.nix" rec {
        system = "aarch64-linux";
        specialArgs = inputs;
        modules = [
          # import the rk3588 module, which contains the configuration for bootloader/ernel/firmware
          (nixos-rk3588 + "/modules/boards/orangepi5plus.nix")
          Systems/Base
          Systems/cyclop.nix
          # {
          # sdImage = {
          # imageName = "sky360-cyclop-orange_pi_5_plus.img";
          # compressImage = false;
          # };
          # }
        ];
      };
    };

    packages = {
      aarch64 = {
        cyclop-orange_pi_5_plus-sd_image = self.nixosConfigurations.cyclop-orange_pi_5_plus.config.system.build.sdImage;
        ######
        # };
        # Raspberry pi 4 - All Sky Camera
        cyclop-rpi4 = nixos-generators.nixosGenerate {
          system = "aarch64-linux";
          format = "sd-aarch64-installer"; # rpi-4
          modules = [
            Hardware/rpi-4.nix
            Systems/Base
            Systems/cyclop.nix
            {
              sdImage = {
                imageName = "sky360-cyclop-orange_pi_5_plus.img";
                #compressImage = false;
              };
            }
          ];
        };
      };
    };
  };
}
