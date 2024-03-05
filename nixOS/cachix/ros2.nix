{
  nix = {
    settings = {
      # substituters = [
      #   "https://sky360.cachix.org"
      # ];
      # trusted-public-keys = [
      #   "sky360.cachix.org-1:/CVvRgdh1sEvexxzJ91HzFr/Sw3OTXZ7dxY/0VeqZ3c="
      # ];


      substituters = [
        "https://cache.nixos.org https://ros.cachix.org"
      ];

      trusted-public-keys = [
        "cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY= ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      ];
    };
  };
}
