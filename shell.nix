{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  packages = [
    (pkgs.python3.withPackages (ps: [ ps.numpy ps.pyyaml ps.pytest ps.matplotlib ]))
  ];
  shellHook = "export PYTHONPATH=$PWD:$PYTHONPATH";
}
