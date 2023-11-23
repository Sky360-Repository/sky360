{ pkgs, sky360, ... }: {
    imports = [
        # ../../dream2nix/packages/openmct        
        ../../dream2nix/packages/openmct/sky360mct_service.nix
    ];

    services.sky360.openmct = {
    enable = true;
    openFirewall = true;
    # TODO: need to test
  };
}
