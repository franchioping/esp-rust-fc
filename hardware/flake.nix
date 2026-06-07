{
  description = "A Nix-flake-based C/C++ development environment";
  inputs = {
    nixpkgs.url = "https://flakehub.com/f/NixOS/nixpkgs/0"; # stable Nixpkgs
    esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
  };

  outputs = { self, ... }@inputs:

    let
      supportedSystems =
        [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forEachSupportedSystem = f:
        inputs.nixpkgs.lib.genAttrs supportedSystems (system:
          f {
            pkgs = import inputs.nixpkgs {
              inherit system;
              overlays = [ inputs.esp-dev.overlays.default ];
              config = {
                permittedInsecurePackages = [ "python3.13-ecdsa-0.19.1" ];
              };
            };
          });
    in {
      devShells = forEachSupportedSystem ({ pkgs }:
        let
          buildInputs = with pkgs;
            [
              esp-idf-full
              rustup
              openssl
              stdenv.cc.cc.lib
              libclang
              (python312.withPackages
                (python-pkgs: with python-pkgs; [ pandas matplotlib ]))
            ] ++ (if pkgs.stdenv.hostPlatform.isDarwin then [ ] else [ gdb ]);
        in {
          default = pkgs.mkShell.override { } {
            packages = buildInputs;

            shellHook = ''
              export LIBCLANG_PATH="${pkgs.libclang.lib}/lib"

              # 2. Automatically map all package lib folders into LD_LIBRARY_PATH
              export LD_LIBRARY_PATH="${
                pkgs.lib.makeLibraryPath buildInputs
              }:$LD_LIBRARY_PATH"
            '';
          };
        });
    };
}
