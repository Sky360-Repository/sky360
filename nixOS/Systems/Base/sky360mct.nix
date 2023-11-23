{ ... }: {

  # imports = [
  #   ./services/openmct.nix # TODO: move to the flake.nix of sky360
  # ];
  services.sky360.openmct = {
    enable = true;
    openFirewall = true;
    # TODO: need to test
  };
}
