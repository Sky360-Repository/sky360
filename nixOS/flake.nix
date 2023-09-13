{
  inputs = {
    # nixpkgs.url = "nixpkgs/nixos-unstable";
    nixpkgs.url = "github:realsnick/nixpkgs";
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
    version = "0.2.0";
    # nixpkgs.config.allowUnfree = true;
  in {
    nixosConfigurations = {
      cyclop-orange_pi_5_plus = import "${nixpkgs}/nixos/lib/eval-config.nix" rec {
        system = "aarch64-linux";
        specialArgs = inputs;
        modules = [
          (nixos-rk3588 + "/modules/boards/orangepi5plus.nix")
          Systems/Base
          Systems/cyclop.nix
          {
            networking.hostName = "cyclop-orange_pi_5_plus";
          }
          {
            sdImage = {
              imageName = "sky360-cyclop-orangepi5plus-${version}";
            };
          }
        ];
      };

      cyclop-orange_pi_5 = import "${nixpkgs}/nixos/lib/eval-config.nix" rec {
        system = "aarch64-linux";
        specialArgs = inputs;
        modules = [
          (nixos-rk3588 + "/modules/boards/orangepi5.nix")
          Systems/Base
          Systems/cyclop.nix
          {
            networking.hostName = "cyclop-orange_pi_5";
          }
          {
            sdImage = {
              imageName = "sky360-cyclop-orangepi5-${version}";
            };
          }
        ];
      };

      cyclop-rock5a = import "${nixpkgs}/nixos/lib/eval-config.nix" rec {
        system = "aarch64-linux";
        specialArgs = inputs;
        modules = [
          (nixos-rk3588 + "/modules/boards/rock5a.nix")
          Systems/Base
          Systems/cyclop.nix
          {
            networking.hostName = "cyclop-rock5a";
          }
          {
            sdImage = {
              imageName = "sky360-cyclop-rock5-${version}";
            };
          }
        ];
      };
    };

    packages = {
      aarch64 = {
        cyclop-orange_pi_5_plus-sd_image = self.nixosConfigurations.cyclop-orange_pi_5_plus.config.system.build.sdImage;

        cyclop-orange_pi_5-sd_image = self.nixosConfigurations.cyclop-orange_pi_5.config.system.build.sdImage;

        cyclop-rock5a-sd_image = self.nixosConfigurations.cyclop-rock5a.config.system.build.sdImage;
        ######
        # };
        # Raspberry pi 4 - All Sky Camera
        cyclop-rpi4-sd_image = nixos-generators.nixosGenerate {
          system = "aarch64-linux";
          format = "sd-aarch64-installer"; # rpi-4
          modules = [
            Hardware/rpi-4.nix
            Systems/Base
            Systems/cyclop.nix
            {
              sdImage = {
                imageName = "sky360-cyclop-rpi4-${version}.img";
                #compressImage = false;
              };
            }
          ];
        };

        # system = "x86_64-linux";
        #  modules = [
        #    #Hardware/x86_64.nix
        #    (Systems/Base)
        #    Systems/fisheye.nix
        #  ];
        #  format = "iso";
      };
    };
  };
}
