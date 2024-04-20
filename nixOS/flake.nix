{
  inputs = {
    nixpkgs.url = "nixpkgs/nixos-unstable";
    dream2nix = {
      url = "github:nix-community/dream2nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixos-rk3588 = {
      # url = "github:ryan4yin/nixos-rk3588";
      url = "github:realsnick/nixos-rk3588";
      # url = "/home/snick/Code/github/nixos-rk3588";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nix-ros-overlay = {
      url = "github:lopsided98/nix-ros-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    sky360-dream2nix = {
      url = "./dream2nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs =
    inputs @ { self
    , nixpkgs
    , dream2nix
    , nixos-generators
    , nixos-rk3588
    , nix-ros-overlay
    , sky360-dream2nix
    , ...
    }:
    let
      system = "aarch64-linux";
      version = "0.3.2";
      lib = nixpkgs.lib;
      config.allowUnfree = true;

      sky360-packages-overlay = (final: prev: {
        sky360 = {
          sky360-ros-iron-heartbeat = prev.callPackage ./ros/iron/heartbeat { };
          qhyccd_sdk = prev.callPackage ./pkgs/qhyccd_sdk.nix { };
        };
      });

      overlays = [
        # nix-ros-overlay.overlays.default
        sky360-packages-overlay
      ];
      pkgs = import nixpkgs {
        inherit system;
        inherit overlays;
        inherit config;
      };

      # generate a cyclop system for any system (x86, arm, etc) and board (nuc, opi5, etc)
      # 
      cyclopSystem = system: board: {
        "cyclop-${board}" = lib.nixosSystem {
          inherit system;
          inherit pkgs;
          inherit config;

          specialArgs = inputs;
          networking.hostName = "cyclop-${board}";
          sdImage = {
            imageName = "sky360-cyclop-${version}-${board}";
          };

          modules = [
            #mkIf for opi 5 
            (nixos-rk3588 + "/modules/boards/${board}")
            nix-ros-overlay.nixosModules.default
            System/Base
            System/cyclop
            System/Base/openmct.nix
          ];
        };
      };
    in
    {

      nixosConfigurations = {
        cyclop-orangepi5plus = cyclopSystem system "orangepi5plus";

        cyclop-orange_pi_5_plus = lib.nixosSystem {
          inherit system;
          pkgs = import nixpkgs {
            inherit system;
            config.allowUnfree = true;
            overlays = [
              # nix-ros-overlay.overlays.default
              sky360-packages-overlay
            ];
          };
          specialArgs = inputs;
          modules = [
            (nixos-rk3588 + "/modules/boards/orangepi5plus.nix")
            nix-ros-overlay.nixosModules.default
            Systems/Base
            Systems/cyclop
            Systems/Base/sky360mct.nix
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
          pkgs = import nixpkgs {
            inherit system;
            config.allowUnfree = true;
            overlays = [
              # nix-ros-overlay.overlays.default
              sky360-packages-overlay
            ];
          };
          specialArgs = inputs;
          modules = [
            (nixos-rk3588 + "/modules/boards/orangepi5.nix")
            nix-ros-overlay.nixosModules.default
            Systems/Base
            Systems/cyclop
            Systems/Base/sky360mct.nix
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
          pkgs = import nixpkgs {
            inherit system;
            config.allowUnfree = true;
            overlays = [
              nix-ros-overlay.overlays.default
              sky360-packages-overlay
            ];
          };
          specialArgs = inputs;
          modules = [
            (nixos-rk3588 + "/modules/boards/rock5a.nix")
            nix-ros-overlay.nixosModules.default
            Systems/Base
            Systems/cyclop
            Systems/Base/sky360mct.nix
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
          pkgs = import nixpkgs {
            inherit system;
            config.allowUnfree = true;
            overlays = [
              nix-ros-overlay.overlays.default
              sky360-packages-overlay
            ];
          };

          specialArgs = inputs;
          modules = [
            nix-ros-overlay.nixosModules.default
            Systems/Base
            Systems/cyclop
            Systems/Base/sky360mct.nix
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
            pkgs = import nixpkgs {
              inherit system;
              config.allowUnfree = true;
              overlays = [
                nix-ros-overlay.overlays.default
                sky360-packages-overlay
              ];
            };
            format = "sd-aarch64-installer"; # rpi-4
            modules = [
              Hardware/rpi-4.nix
              nix-ros-overlay.nixosModules.default
              Systems/Base
              Systems/cyclop
              Systems/Base/sky360mct.nix
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
            pkgs = import nixpkgs {
              inherit system;
              config.allowUnfree = true;
              overlays = [
                nix-ros-overlay.overlays.default
                sky360-packages-overlay
              ];
            };
            format = "iso";
            modules = [
              # nix-ros-overlay.nixosModules.default
              Systems/Base
              Systems/cyclop
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