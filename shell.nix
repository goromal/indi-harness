{ anixpkgs ? import ../anixpkgs { } }:
anixpkgs.mkShell {
  packages = [
    (anixpkgs.python313.withPackages (ps: [
      ps.numpy ps.pyyaml ps.pymavlink ps.rosbags ps.pysignals ps.geometry ps.pytest ps.matplotlib
    ]))
  ];
  shellHook = "export PYTHONPATH=$PWD:$PYTHONPATH";
}
