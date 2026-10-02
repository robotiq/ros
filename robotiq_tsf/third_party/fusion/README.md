# Fusion (vendored)

The AHRS part of x-io Technologies' [Fusion](https://github.com/xioTechnologies/Fusion) library, MIT-licensed (see `LICENSE.md`). `robotiq_tsf`'s `OrientationFilter` runs on it.

- Upstream: https://github.com/xioTechnologies/Fusion
- Version: v1.3.3, commit `9325424011892abacc0ce42b8bb1a8ae20264b9b`
- Files: `FusionAhrs.c`, `FusionAhrs.h` and the headers they include, copied unmodified from upstream's `Fusion/` directory.

Vendored rather than added as a submodule so the sources ship in the package's release tarball. `AMENT_IGNORE` keeps the ament linters off upstream's code.

To update, copy the same files from the new upstream release, update the version and commit above, and run the `robotiq_tsf` tests.
