{ config, lib, pkgs, ... }:

{
  boot.kernelPackages = pkgs.linuxKernel.packages.linux_rpi4;
  #boot.loader.grub.enable = false;
  #boot.loader.generic-extlinux-compatible.enable = true;
  #boot.initrd.includeDefaultModules = false;

  #  boot.initrd.availableKernelModules = [
  #    # Allows early (earlier) modesetting for the Raspberry Pi                                                
  #    "vc4"
  #    "bcm2835_dma"
  #    "i2c_bcm2835"
  #    # Allows early (earlier) modesetting for Allwinner SoCs                                                  
  #    "sun4i_drm"
  #    "sun8i_drm_hdmi"
  #    "sun8i_mixer"
  #  ];

  #hardware.enableRedistributableFirmware = lib.mkForce false;

  nixpkgs.overlays = [
    (final: super: {
      makeModulesClosure = x:
        super.makeModulesClosure (x // {
          allowMissing = true;
        });
    })
  ];
}
