{ pkgs, ... }: {
  home.packages = [
    pkgs.helix
    pkgs.nil
    pkgs.alejandra
    pkgs.rnix-lsp
    pkgs.rust-analyzer
    pkgs.tree-sitter
    (pkgs.tree-sitter.withPlugins (_: pkgs.tree-sitter.allGrammars))
    pkgs.nixpkgs-fmt
  ];

  programs.helix = {
    enable = true;
    defaultEditor = true;
    settings = {
      editor = {
        line-number = "relative";
        bufferline = "multiple";
        mouse = true;
        true-color = true;
        color-modes = true;
        cursorline = true;
        auto-completion = true;
        completion-trigger-len = 1;

        cursor-shape = {
          insert = "bar";
          normal = "block";
          select = "underline";
        };
        file-picker = {
          hidden = false;
          git-ignore = true;
        };
        soft-wrap = {
          enable = true;
        };
        statusline = {
          left = [ "mode" "spinner" ];
          center = [ "file-name" "position-percentage" ];
          right = [ "version-control" "diagnostics" "selections" "position" "file-encoding" "file-line-ending" "file-type" ];
          separator = "│";
        };
        lsp = {
          enable = true;
          display-messages = true;
          auto-signature-help = true;
          display-inlay-hints = true;
          display-signature-help-docs = true;
          snippets = true;
          goto-reference-include-declaration = true;
        };
        whitespace = {
          render = "all";
          characters = {
            space = " ";
            nbsp = "⍽";
            tab = "→";
            newline = "⏎";

            tabpad = "·"; # Tabs will look like "→···" (depending on tab width)
          };
        };
        indent-guides = {
          render = true;
          character = "╎";
        };
      };
    };
    languages = {
      #language-server = [
      #  {
      #    rnix = {
      #      command = "rnix-lsp";
      #    };
      #  }
      #];
      language = [
        {
          name = "nix";
          auto-format = true;
          formatter = { command = "nixpkgs-fmt"; };
          # language-servers = {command = "nil";};
        }
        {
          name = "typescript";
          auto-format = true;
        }
        {
          name = "javascript";
          auto-format = true;
        }
      ];
    };
  };
}

