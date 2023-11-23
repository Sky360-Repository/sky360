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
    EDITOR= "hx";
  };

  #environment.systemPackages = with pkgs; [
  #];
}
