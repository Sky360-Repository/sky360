{ config, lib, pkgs, sky360-dream2nix, ... }:

with lib;

let
  cfg = config.services.sky360.openmct;
  defaultUser = "openmct";
  defaultGroup = defaultUser;
  # FIXME: remove hardcoded system `aarch64-linux`
  defaultPackage = sky360-dream2nix.packages.aarch64-linux.packages.openmct;
  # defaultPackage = sky360.packages.openmct;
in
{
  options.services.sky360.openmct = {
    enable = mkEnableOption (mdDoc "Sky360's openMCT service");

    package = mkOption {
      default = defaultPackage;
      defaultText = literalExpression "sky360.packages.openmct";
      type = types.package;
      description = lib.mdDoc "openMCT package to use";
    };

    openFirewall = mkOption {
      type = types.bool; # NOTE: this is why nix language can not be dynamic and require a proper type system. I just don't get it sometimes...the compliation of the code won't slow down compare to a build....
      default = false;
      description = mdDoc "Open ports in the firewall for the service.";
    };

    port = mkOption {
      type = types.port; # NOTE: LOL!!! Exactly!
      default = 8080; # TODO: need to test 
      description = mdDoc "Listening port.";
    };

    # NOTE: this is why nixOS is the way to go...it forces you to not make shortcuts and automate the build and deployment in a very easy way
    security = {
      user = mkOption {
        type = types.str; # NOTE: really? are  we short on chars? why not types.string?
        default = defaultUser;
        description = mdDoc "User under which openMCT runs.";
      };
      group = mkOption {
        type = types.str;
        default = defaultGroup;
        description = mdDoc "Group under which openMCT runs.";
      };
    };

    userDir = mkOption {
      type = types.path; # NOTE: hmmm.... type system!!!
      default = "/var/lib/sky360/openmct";
      description = mdDoc ''
        The directory to store all user data, 
        such as credential files and all library data. 
      '';
    };
    # TODO: 
    # mode: safe/prod 
    # log level: trace/debug/info/warning/error
  };

  config = mkIf cfg.enable {
    users.users.${cfg.security.user} = {
      isSystemUser = true;
      group = cfg.security.group;
    };

    users.groups.${cfg.security.group} = { };

    networking.firewall = mkIf cfg.openFirewall {
      allowedTCPPorts = [ cfg.port ];
    };

    environment.systemPackages = with pkgs; [
      # cfg.package
    ];

    systemd.services.openmct = {
      description = "Sky360 openMCT Service";
      wantedBy = [ "multi-user.target" ];
      after = [ "networking.target" ];
      environment = { HOME = cfg.userDir; };
      path = [ cfg.package ];
      serviceConfig = mkMerge [{
        User = cfg.security.user;
        Group = cfg.security.group;
        # TODO: support --port and more params
        # ExecStart = "${sky360-dream2nix.openmct}/bin/sky360_openmct --port $toString cfg.port}";
        ExecStart = "${cfg.package}/bin/openmct";
        PrivateTmp = true;
        Restart = "always"; # TODO: write options
        WorkingDdddirectory = cfg.userDir;
        StateDirectory = "openmct"; #TODO: test
      }];
    };
  };
}
