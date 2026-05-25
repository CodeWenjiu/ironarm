{
  description = "Flake configuration for nzea";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
    rust-overlay.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs =
    {
      nixpkgs,
      utils,
      rust-overlay,
      ...
    }:
    utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
        };

        rust-toolchain = pkgs.rust-bin.nightly.latest.default.override {
          extensions = [
            "rust-src"
            "clippy"
            "rust-analyzer"
            "llvm-tools-preview"
          ];
        };

        # Packages whose binaries should be auto-linked into .direnv/bin
        # (so Zed / other tools that don't inherit the full Nix PATH can find them).
        devPackages = [
          rust-toolchain
        ];

        # Build-time dependencies needed by Rust crate build scripts
        # (e.g. C compiler for crates like zstd, ring, etc.).
        guiPackage = with pkgs; [
          pkg-config
          openssl
          xorg.libX11
          xorg.libXcursor
          xorg.libXi
          xorg.libXrandr
          libxkbcommon
          vulkan-loader
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = guiPackage ++ devPackages;

          shellHook = ''
            mkdir -p .direnv/bin
          ''
          + pkgs.lib.concatMapStringsSep "\n" (pkg: ''
            if [ -d "${pkg}/bin" ]; then
              for f in ${pkg}/bin/*; do
                ln -sf "$f" .direnv/bin/
              done
            fi
          '') devPackages;
        };
      }
    );
}
