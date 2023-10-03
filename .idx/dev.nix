{pkgs}: let
  rpkgs = pkgs.extend (import (builtins.fetchTarball "https://github.com/oxalica/rust-overlay/archive/master.tar.gz"));
in {
  # Use https://search.nixos.org/packages?channel=unstable to  find packages
  packages = with rpkgs; [
    (rust-bin.fromRustupToolchainFile ../discobiker/frontend/rust-toolchain.toml)
    trunk
    stdenv.cc
    google-cloud-sdk
  ];

  # sets environment variables in the workspace
  env = {
    # SOME_ENV_VAR = "hello";
    NIX_USER_CONF_FILES = pkgs.writeText "nix.conf" ''
      extra-experimental-features = nix-command flakes repl-flake
    '';
  };

  ide = {
    # search for the extension on https://open-vsx.org/ and use "publisher.id"

    extensions = [
      # "angular.ng-template"
    ];

    # preview configuration, identical to monospace.json
    previews = [
      {
        command = [
          "sh"
          "-c"
          "export RUSTFLAGS=--cfg=web_sys_unstable_apis; cd discobiker/frontend && exec trunk serve --port $PORT --address 0.0.0.0"
        ];

        manager = "web";
        id = "web";
      }
    ];
  };
}