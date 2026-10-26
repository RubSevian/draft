# Imported robot assets

Keep source geometry here and simulation choices in `configs/`.

`mevius2/` contains the STL URDF and its 13 referenced meshes from
[haraduka/mevius2](https://github.com/haraduka/mevius2), revision
`4f09680bb575574377b903bbc971bdb2695d507b`. Its original MIT license is included.
`SOURCE.json` records the source file, modifications and SHA-256 hashes.
Only mesh paths and XML formatting were changed in the bundled URDF.

The source URDF keeps its fixed root and original names. The importer adds the
floating base, explicit joint mappings, actuators, sensors, floor and optional
payload to generated output. Do not edit source assets to tune a simulation.
The generated folder includes its meshes and can be moved independently.

See [`docs/imported-robots.md`](../docs/imported-robots.md).
