{
  description = "Embassy networking test for the STM32H753ZI (Nucleo-H753ZI)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, rust-overlay }:
    let
      systems = [ "aarch64-darwin" "x86_64-darwin" "aarch64-linux" "x86_64-linux" ];
      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f system);
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ (import rust-overlay) ];
          };

          # Toolchain (channel/components/targets) comes from rust-toolchain.toml,
          # so `cargo` outside the nix shell picks the same one via rustup.
          rust = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
        in
        {
          default = pkgs.mkShell {
            name = "h753zi-net-test";

            packages = [
              rust
              pkgs.probe-rs-tools # probe-rs run / attach / chip list
              pkgs.flip-link      # stack-overflow-protecting linker wrapper
              pkgs.cargo-binutils # cargo size / objdump / nm
            ] ++ pkgs.lib.optionals pkgs.stdenv.isLinux [
              pkgs.libusb1
              pkgs.pkg-config
            ];

            shellHook = ''
              echo "h753zi-net-test dev shell"
              echo "  rustc:    $(rustc --version)"
              echo "  probe-rs: $(probe-rs --version 2>/dev/null | head -1)"
              echo
              echo "  cargo run --release      flash + attach RTT to the board"
              echo "  probe-rs list            show attached probes"
            '';
          };
        });
    };
}
