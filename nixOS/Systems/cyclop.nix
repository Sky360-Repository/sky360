{
  config,
  pkgs,
  ...
}: {
  networking.hostName = "cyclop";
  environment.systemPackages = with pkgs; [
    indi-full
  ];
}
