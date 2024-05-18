# panda-polymetis
Tools for working with the Pandas using [Polymetis](https://facebookresearch.github.io/fairo/polymetis/) from FAIR. Meant to be a (mostly) drop-in replacement for panda-bm_ros_packages.
This is meant to provide a set of convenience functions that operate between polymetis and the robot, while a separate library provides an OpenAI gym-style interface for imitation/reinforcement learning code on top of this repository's code.
Originally developed at Samsung AI Centre -- Montreal, and part of this [tactile imitation learning monorepo](https://github.com/SAIC-MONTREAL/tactile-il).
Modified from the original Samsung version to work with the FR3 (original Panda no longer in production).

## Requirements/Installation
- [transform-utils](https://github.com/utiasSTARS/transform-utils)
  - `git clone https://github.com/utiasSTARS/transform-utils && cd transform-utils && bash install.sh`
- To use this package, you must [install polymetis](https://facebookresearch.github.io/fairo/polymetis/installation.html) on both the RT computer (usually a NUC) that controls the Panda and also on your local computer that will handle vision/policy control
- To use it with an FR3, you *must* [install from source](https://facebookresearch.github.io/fairo/polymetis/installation.html#from-source), and set the `<version_tag_or_commit_hash>` for the `libfranka` build to `0.10.0` (or later, but only `0.10.0` has been tested).
  - **to be confirmed**: the source install might only be required on the NUC, and not on your local version.
- This still has a pytorch issue, which is fixed by reinstall pytorch as:
  - `pip install --force-reinstall torch==1.13.1 --index-url https://download.pytorch.org/whl/cpu`
- If you're installing on your local computer, you can instead use the gpu-enabled pytorch:
  - `pip install --force-reinstall torch==1.13.1`

### Updating pytorch and finishing polymetis build from source
**WARNING**: This requires roughly ~10GB of space in `/`.
**TODO**: this might be completely unnecessary, which is to be verified.
The plan is to simply use the built polymetis (with libfranka 0.10.0) with *non*-gpu enabled pytorch on the NUC, and use non-built polymetis (with libfranka 0.9.0) with gpu enabled pytorch on my local machine, and assume that libfranka is not necessary for the comm between machines....

1. Install gpu-enabled pytorch (the one installed above is cpu only): `pip install --force-reinstall torch==1.13.1`
2. Install cuda toolkit from nvidia website: https://developer.nvidia.com/cuda-downloads.
3. If `nvcc` isn't an available command, install it with `sudo apt install nvidia-cuda-toolkit`
4. Install cudnn library: `sudo apt install nvidia-cudnn`
5. Set a the CUDA_HOME variable: `export CUDA_HOME=/usr/local/cuda`
6. Replace the cmake command from the polymetis instructions with `cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_FRANKA=[OFF/ON] -DBUILD_TESTS=[OFF/ON] -DBUILD_DOCS=[OFF/ON] -DCUDA_TOOLKIT_ROOT_DIR=$CUDA_HOME`
7. Add `#include <stddef.h>` to the top of `fairo/polymetis/polymetis/torch_isolation/include/torch_server_ops.hpp`
8. Run `make -j`

## Usage

### Bring up a robot
1. `cd launch`
2. sim: `bash sim_robot.bash`, real: `bash real_robot.bash`

### test with keyboard control
Instructions for use come up on screen.
`python -m panda_polymetis.tools.keyboard_interface`

## General Polymetis Instructions

## Bringing up the robot with polymetis or polysim
The `launch_robot.py` and `launch_gripper.py` scripts are available from anywhere once you've run `conda activate polymetis`/`source activate polymetis`.

### Sim
*Note:* For now, simulating the gripper is not supported.
Polymetis sort of supports it, but it's experimental, and doesn't work all that well yet.

In one terminal, run
```
launch_robot.py robot_client=franka_sim use_real_time=false gui=true
```

### Real Robot
The robot client and gripper client are launched separately.

#### Arm
On the Nuc, for controlling the robot (assuming ROBOT_IP is set):
```
launch_robot.py robot_client=franka_hardware timeout=20 robot_client.executable_cfg.robot_ip=$ROBOT_IP
```

There are other command line arguments you can set using hydra.
See the [config file](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/conf/robot_client/franka_hardware.yaml).

#### Gripper
On the host machine (_not_ the NUC, since the NUC should have as little running on it as possible),
```
launch_gripper.py gripper=franka_hand robot_client.executable_cfg.robot_ip=$ROBOT_IP
```

## Running our clients
Use the tests/utils to get an idea.

# States
Fields of states that are available are in [this proto file](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/protos/polymetis.proto).
This is probably what we'll have to modify to expose things that aren't yet exposed (e.g. F_ext).
