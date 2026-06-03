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
            "rustc-codegen-cranelift-preview"
          ];
        };

        devPackages = [
          rust-toolchain
          pkgs.stdenv.cc
          pkgs.uv
          pkgs.python313
        ];

        buildDeps = with pkgs; [
          pkg-config
          clang
          mold
        ];

        guiRuntime = with pkgs; [
          dejavu_fonts
          noto-fonts
          wayland
          libGL
          vulkan-loader
          libX11
          libXcursor
          libXrandr
          libXi
          libxcb
          libxkbcommon
          glib
          fontconfig
          freetype
          dbus
          libglvnd
          libpng
          zlib
        ];

        allPackages = buildDeps ++ guiRuntime ++ devPackages;
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = allPackages;

          shellHook = ''
            mkdir -p .direnv/bin
            # WSL: use Direct3D12 GPU backend instead of llvmpipe software rendering
            if [ -d /usr/lib/wsl/lib ]; then
              export GALLIUM_DRIVER=d3d12
              export D3D12_FORCE_WARP=0
            fi
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
