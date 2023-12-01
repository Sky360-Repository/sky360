{
  config,
  pkgs,
  ...
}: {

  imports = [
    ./helix.nix
  ];
  environment.systemPackages = with pkgs; [
    helix
    nil
    tree-sitter-grammars.tree-sitter-nix
    nixpkgs-fmt
    btop
    tmux
    lsd
    tree
    bat
    wget
    neofetch
    mcfly
    direnv
    ranger
    mc
    git
    gitui
    tig
    gh
    nixpkgs-review
    usbutils
    w3m
  ];
}
