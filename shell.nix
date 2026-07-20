{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  packages = [
    (pkgs.python3.withPackages (ps: [ ps.numpy ps.pyyaml ps.pytest ps.matplotlib ps.pymavlink ps.rosbags ]))
  ];
  shellHook = "export PYTHONPATH=$PWD:$PYTHONPATH";
}
