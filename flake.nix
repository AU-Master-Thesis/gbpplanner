{
  description = "GBPPlanner: Distributing Multi-Robot Collaboration with Gaussian Belief Propagation";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    nixpkgs.inputs.flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (
      system: let
        pkgs = import nixpkgs {inherit system;};
      in
        with pkgs; {
          devShells.default = pkgs.mkShell rec {
            nativeBuildInputs = [
              pkgs.pkg-config
            ];
            buildInputs = with pkgs; [
              gcc
              valgrind
              heaptrack
              flamegraph
              linuxKernel.packages.linux_6_6.perf
              nodejs
              clang
              cmake
              cmake-language-server
              ninja
              llvmPackages.openmp
              xorg.libX11
              xorg.libXcursor
              xorg.libXi
              xorg.libXrandr # To use the x11 feature
              xorg.libXinerama
              libxkbcommon
              wayland # To use the wayland feature
              vulkan-headers
              vulkan-helper
              vulkan-tools
              # OpenGL
              libGL
            ];
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath buildInputs;
          };
        }
    );
}
