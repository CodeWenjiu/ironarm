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
          pkgs.stdenv.cc # provides cc/gcc linker for rust-analyzer
        ];

        buildDeps = with pkgs; [
          pkg-config
          clang
          mold
        ];

        guiRuntime = with pkgs; [
          wayland
          libxkbcommon
          libGL
          vulkan-loader
          libX11
          libXcursor
          libXrandr
          libXi
          libxcb
        ];

        allPackages = buildDeps ++ guiRuntime ++ devPackages;
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = allPackages;

          shellHook = ''
            mkdir -p .direnv/bin
            export LD_LIBRARY_PATH=${pkgs.lib.makeLibraryPath guiRuntime}:$LD_LIBRARY_PATH
            export PKG_CONFIG_PATH=${pkgs.openssl.dev}/lib/pkgconfig
          ''
          + pkgs.lib.concatMapStringsSep "\n" (pkg: ''
            if [ -d "${pkg}/bin" ]; then
              for f in ${pkg}/bin/*; do
                ln -sf "$f" .direnv/bin/
              done
            fi
          '') allPackages;
        };
      }
    );
}
