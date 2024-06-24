# panda-polymetis

Tools for working with the Pandas using [Polymetis](https://facebookresearch.github.io/fairo/polymetis/) from FAIR.
This is meant to provide a set of convenience functions that operate between polymetis and the robot, while a separate library provides an OpenAI gym-style interface for imitation/reinforcement learning code on top of this repository's code.
Originally developed at Samsung AI Centre -- Montreal, and part of this [tactile imitation learning monorepo](https://github.com/SAIC-MONTREAL/tactile-il).
For an example of how these tools are used, see [panda-rl-envs](https://github.com/utiasSTARS/panda-rl-envs).

- [Requirements/Installation](#requirementsinstallation)
  - [Real-Time Computer](#real-time-computer)
    - [Installation on FR3](#installation-on-fr3)
  - [Local Computer](#local-computer)
- [Usage](#usage)
  - [Bring up a robot](#bring-up-a-robot)
  - [Bring up a gripper](#bring-up-a-gripper)
  - [Testing with keyboard control](#testing-with-keyboard-control)
- [States](#states)

## Requirements/Installation
To use this package, you must [install polymetis](https://facebookresearch.github.io/fairo/polymetis/installation.html) on both a RT computer (often a NUC) that controls the Panda and also on your local computer that will handle vision/policy control.
The following sections provide more detail.

### Real-Time Computer
1. Follow the [standard conda instructions](https://facebookresearch.github.io/fairo/polymetis/installation.html) for polymetis, unless you're using an FR3 arm, in which case you must replace this step with [Installation on FR3](#installation-on-fr3) below.
2. This currently causes a pytorch issue because of conda, which can be resolved with:
```bash
pip install --force-reinstall torch==1.13.1 --index-url https://download.pytorch.org/whl/cpu
```
**Note that Ada in the STARS lab already has a functioning install in the `polymetis-local` conda env**.

#### Installation on FR3
Replace step 1 in [Real-Time Computer](#real-time-computer) with an [installation from source](https://facebookresearch.github.io/fairo/polymetis/installation.html#from-source), and set the `<version_tag_or_commit_hash>` for the `libfranka` build to `0.10.0` (or later, but only `0.10.0` has been tested).

### Local Computer
1. Follow the [standard conda instructions](https://facebookresearch.github.io/fairo/polymetis/installation.html) for polymetis.
2. To switch to gpu-enabled pytorch:
```bash
pip install --force-reinstall torch==1.13.1
```
3. Install [transform-utils](https://github.com/utiasSTARS/transform-utils)
```bash
git clone https://github.com/utiasSTARS/transform-utils && cd transform-utils && bash install.sh
```
4. Install this repository:
```bash
git clone https://github.com/utiasSTARS/panda-polymetis.git && cd panda-polymetis && pip install -e .
```

## Usage

### Bring up a robot
1. `cd launch`
2. sim: `bash sim_robot.bash`, real: `bash real_robot.bash`

### Bring up a gripper
Simulating grippers in polymetis is not supported.
In sim, `panda-polymetis` uses a fake gripper client so that you can replicate your real world setup.
On the real robot, you must run a gripper server on the *local computer* with:
- Robotiq 2F85: `launch_gripper.py gripper=robotiq_2f gripper.comport=/dev/ttyUSB0`
- Panda Gripper: `launch_gripper.py`

### Testing with keyboard control
Instructions for use come up on screen.
```bash
python -m panda_polymetis.tools.keyboard_interface
```

## States
Fields of states that are available are in [this proto file](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/protos/polymetis.proto).
This is probably what we'll have to modify to expose things that aren't yet exposed (e.g. F_ext).
