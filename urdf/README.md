# Imported robot assets

Keep source geometry here and editable specifications in `src/draft/robots/`.

`mevius2/` contains the STL URDF and its 13 referenced meshes from
[haraduka/mevius2](https://github.com/haraduka/mevius2), revision
`4f09680bb575574377b903bbc971bdb2695d507b`. Its original MIT license is included.
`SOURCE.json` records the source file, modifications and SHA-256 hashes.
Only mesh paths and XML formatting were changed in the bundled URDF.

The source URDF keeps its fixed root, original names and physical properties.
The native `src/draft/robots/mevius2/` YAML specification reproduces its joint
kinematics using Draft's parametric primitives and common mass/motor sizing.
Changing dimensions changes the generated model, not these reference files.

See [`docs/mevius2.md`](../docs/mevius2.md).
