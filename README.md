# accelsim
Simulate accelerometer's readings

Accelerometers can, as their name suggests, measure accelerations.
Most likely your telephone has one.
If you access it with an app (e.g.: Arduino Science Journal) you'll notice that it can read accelerations among its three principal axes, x, y and z.

If you leave your telephone on a table, it will register only one acceleration: that of gravity.
Why is gravity an acceleration?
Well, that's one of the deepest questions of physics, so let's just believe it for now (if you cannot wait, just google Equivalence Principle).

Now, imagine you throw your telephone into the air in a controlled manner.
This will be an interesting experience for the accelerometer.
First, it will record the acceleration induced by your arm.
Then, the telephone will displace and rotate in the air.
This means, among other things, that the accelerometer will "experience" gravity turning around, up and down, left and right.
A complicated set of readings will be recorded.

The purpose of this tiny project is to simulate such a set of accelerometers readings.

More specifically, the inputs and outputs we want are:

## Input

- Movement: in the form of a trajectory, that is, a time-dependent curve.
- Rotation: in the form of a rotation matrix, that is, a time-dependent orthogonal matrix.

## Output

- The x, y and z accelerations as read by the accelerometer.
