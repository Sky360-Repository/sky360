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
    # nixos-rk3588.url = "/home/snick/Code/github/nixos-rk3588";

    # mesa-panfork = {
    # url = "gitlab:panfork/mesa/csf";
    # flake = false;
    # };

    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    # nixpkgs.overlays = [ nix-ros-overlay.overlay ];
  };
  outputs =
    inputs @ { self
    , nixpkgs
    , nixos-generators
    , nixos-rk3588
      #, mesa-panfork, #GPU support for the rk3588
    , nix-ros-overlay
    , ...
    }:
    let
      version = "0.2.0";
    in
    {
      nixosConfigurations = {
        cyclop-orange_pi_5_plus = import "${nixpkgs}/nixos/lib/eval-config.nix" rec {
          system = "aarch64-linux";
          specialArgs = inputs;
          modules = [
            (nixos-rk3588 + "/modules/boards/orangepi5plus.nix")
            nix-ros-overlay.nixosModules.default
            Systems/Base
            Systems/cyclop.nix
            {
              networking.hostName = "cyclop-orange_pi_5_plus";
            }
            {
              sdImage = {
                imageName = "sky360-cyclop-${version}-orangepi5plus";
              };
            }
          ];
        };

        cyclop-orange_pi_5 = import "${nixpkgs}/nixos/lib/eval-config.nix" rec {
          system = "aarch64-linux";
          specialArgs = inputs;
          modules = [
            (nixos-rk3588 + "/modules/boards/orangepi5.nix")
            nix-ros-overlay.nixosModules.default
            Systems/Base
            Systems/cyclop.nix
            {
              networking.hostName = "cyclop-orange_pi_5";
            }
            {
              sdImage = {
                imageName = "sky360-cyclop-${version}-orangepi5";
              };
            }
          ];
        };

        cyclop-rock5a = import "${nixpkgs}/nixos/lib/eval-config.nix" rec {
          system = "aarch64-linux";
          specialArgs = inputs;
          modules = [
            (nixos-rk3588 + "/modules/boards/rock5a.nix")
            nix-ros-overlay.nixosModules.default
            Systems/Base
            Systems/cyclop.nix
            {
              networking.hostName = "cyclop-rock5a";
            }
            {
              sdImage = {
                imageName = "sky360-cyclop-${version}-rock5a";
              };
            }
          ];
        };

        cyclop-x86_64 = import "${nixpkgs}/nixos/lib/eval-config.nix" rec {
          system = "x86_64-linux";
          specialArgs = inputs;
          modules = [
            nix-ros-overlay.nixosModules.default
            Systems/Base
            Systems/cyclop.nix
            {
              networking.hostName = "cyclop-x86_64_linux";
            }
            {
              sdImage = {
                imageName = "sky360-cyclop-${version}-x86_64-linux";
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
              nix-ros-overlay.nixosModules.default
              Systems/Base
              Systems/cyclop.nix
              {
                sdImage = {
                  imageName = "sky360-cyclop-${version}-rpi4.img";
                  #compressImage = false;
                };
              }
            ];
          };

          cyclop-x86_64-linux-sd_image = nixos-generators.nixosGenerate {
            system = "aarch64-linux";
            format = "iso";
            modules = [
              nix-ros-overlay.nixosModules.default
              Systems/Base
              Systems/cyclop.nix
              {
                sdImage = {
                  imageName = "sky360-cyclop-${version}-x86_64-linux.img";
                  #compressImage = false;
                };
              }
            ];
          };
        };
      };
    };
}
