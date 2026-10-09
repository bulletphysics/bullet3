# OneRobotics A1 robot assets

This directory contains three URDF variants of the OneRobotics A1 robot for
use with PyBullet.

| Variant | URDF | Movable joints |
| --- | --- | ---: |
| Right arm | `right_arm/model.urdf` | 7 |
| Left arm | `left_arm/model.urdf` | 7 |
| Bimanual stand | `bimanual_stand/model.urdf` | 14 |

## Source and modifications

The assets were adapted from
[`katazen/onerobot_h1`](https://github.com/katazen/onerobot_h1) at commit
`ecf530911284ba0e559f7a24dc222fd8e60d31ed`.

The unused top-level MuJoCo `compiler` element was removed from the published
URDF files, and the meshes were packaged with relative paths. The remaining
URDF model fields and all STL mesh bytes are unchanged from that source
revision.

## License and attribution

The OneRobotics A1 robot assets are copyright 2026 OneRobotics and licensed
under the Creative Commons Attribution 4.0 International license (CC BY 4.0).
See [`LICENSE.txt`](LICENSE.txt) for the full license text.

## Known limitations

- The arm models do not include grippers.
- The right- and left-arm Link7 mass and inertia values are marked as temporary
  in the source models.
- The bimanual URDF contains zero effort and velocity values inherited from the
  source. The accompanying example therefore supplies conservative controller
  force values at runtime; those values are not presented as source CAD data.

Run the example from the repository root with one of `right`, `left`, or
`bimanual`:

```bash
PYTHONPATH=examples/pybullet/gym python examples/pybullet/examples/onerobotics_a1.py --model right
```
