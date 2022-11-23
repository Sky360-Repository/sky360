{ config, pkgs, ... }:

{
  time.timeZone = "America/Chicago";
  i18n.defaultLocale = "en_US.UTF-8";
  console = {
    font = "ter-powerline-v16n";
    packages = [ pkgs.terminus_font pkgs.powerline-fonts ];
  };

  environment.sessionVariables = {
    TERM = "xterm-256color";
    COLORTERM = "truecolor";
  };

  environment.systemPackages = with pkgs; [
    helix
    nil
    tree-sitter-grammars.tree-sitter-nix
    nixpkgs-fmt
    btop
    tmux
    lsd
    exa
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
