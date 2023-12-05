{
  config,
  pkgs,
  ...
}: {

  imports = [
    # ./helix.nix
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
    qutebrowser

    pkgs.alejandra

    pkgs.nodePackages.bash-language-server
    pkgs.cmake-language-server

    # helix needs
    pkgs.zellij
    pkgs.lazygit
    pkgs.nil
    pkgs.rnix-lsp
    pkgs.rust-analyzer
    pkgs.clang-tools
    pkgs.ocamlPackages.ocaml-lsp
    pkgs.vscode-langservers-extracted
    pkgs.dockerfile-language-server-nodejs
    pkgs.haskellPackages.haskell-language-server
    pkgs.nodePackages.typescript-language-server
    pkgs.texlab
    pkgs.lua-language-server
    pkgs.marksman
    pkgs.python310Packages.python-lsp-server
    pkgs.nodePackages.vue-language-server
    pkgs.yaml-language-server
    pkgs.taplo

    pkgs.tree-sitter
    (pkgs.tree-sitter.withPlugins (_: pkgs.tree-sitter.allGrammars))
    pkgs.nixpkgs-fmt
  ];
}
