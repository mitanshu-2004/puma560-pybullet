# PUMA 560 motion simulation in PyBullet

Type a target point and the arm moves to it, or explains why it cannot.

The simulation uses Python, PyBullet and NumPy.

## Running it

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python simulate.py
```

PyBullet may build from source on Python 3.12 or later, which can take a few minutes and requires a C++ compiler.

A PyBullet window opens with the arm at its starting pose. Enter target coordinates through the terminal:

```text
Target X Y Z in metres, h for home, q to quit: 0.45 -0.15 0.30
```

Coordinates are in metres in the world frame. The robot base is at the origin and its shoulder is 0.672 m above it. The point moved to the target is the origin of the last link, `link7`.

Some example targets are:

| Target | Result |
| --- | --- |
| `0.45 -0.15 0.30` | Accepted |
| `0.9 0.27 0.2` | Outside the arm's reach |
| `0.01 0.01 0.2` | No legal pose found |
| `0.36 -0.09 1.16` | No legal pose found, mostly joint limits |
| `0.1 0.03 0.21` | No legal pose found, mostly self-collision |

To reproduce a path-collision refusal, enter these targets in order:

```text
0.75 -0.54 0.62
0.10 -0.21 0.00
-0.24 0.43 0.34
```

The first two movements are accepted. The third is refused because the arm would collide with itself during the movement.

An accepted target is marked in cyan and its planned tool path is drawn as orange dots. A refused target is marked in red, or in orange when the target pose is valid but the path to it is unsafe. The reason is also printed in the terminal.

The dotted grid at z = 0 is only a visual reference. Collision checks cover the arm's own links, not the ground.

Enter `h` to return the arm to its starting pose. The arm normally follows a planned path home. If it is already in contact or cannot return safely, it is placed directly at the home configuration.

## URDF collision geometry

The URDF and meshes come from [nimasarli/puma560_description](https://github.com/nimasarli/puma560_description) (MIT, license in `puma560_description/LICENSE`), with the mesh paths made relative. The original URDF has visual geometry but no collision geometry. Without `<collision>` elements, PyBullet cannot detect collisions between the robot's links.

A collision element matching the visual mesh and origin was added to each of the seven links. The existing material opacity was also changed from `0.8` to `1` so the links render as solid objects.

## How it works

`simulate.py` handles the GUI and user input. `puma.py` contains the robot model, the inverse-kinematics search built on PyBullet's solver, trajectory generation and safety checks. `measure.py` prints the furthest any point of the arm sits from the axis of a joint that moves it, over 400 random poses, next to the 1.0 m arm radius that the path check assumes.

For each target, the program:

1. Rejects points outside the arm's maximum reach.
2. Runs PyBullet's inverse kinematics from up to 101 starting poses: the middle of each joint's range, then 100 random poses from a fixed seed.
3. Uses forward kinematics to verify that the tool reaches the target within 1 mm.
4. Rejects solutions that exceed joint limits or cause self-collision.
5. Takes the first solution that passes and checks the straight joint-space path to it for self-collision, in steps small enough that no point on the arm moves more than 2 mm between checks. If that path collides, the target is refused.
6. Executes an accepted path using a smooth quintic trajectory.

The movement duration is set so that no joint is commanded faster than 90 degrees per second. During movement, the robot's actual configuration is checked after every simulation step, and the motion stops if a self-collision is detected.
