# three.js (vendored for MaxScope 3D)

three.js r186.1 (MIT, see `LICENSE`), from the npm registry tarball
`https://registry.npmjs.org/three/-/three-0.186.1.tgz` (sha1
`6d50f70c2c437f844179bbb56d6f5b774e1ca38a`). Only the experimental 3D page
(`/3d`) loads these files; the main viewer never does.

- `three.module.js`, `three.core.js`: `build/` unchanged.
- `OrbitControls.js`: `examples/jsm/controls/OrbitControls.js` with its one
  import changed from `'three'` to `'./three.module.js'`, because the viewer's
  Content-Security-Policy does not allow an inline import map.

To update, take the same files from a newer tarball and repeat that one edit.
