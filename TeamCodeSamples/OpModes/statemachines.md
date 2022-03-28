# State Machine

State machine, finite state machine, finite automation all end up at
*state machine* in WikiPedia. Take a look for a formal description of
state machine.

And of couurse
[gmZero](https://gm0.org/en/latest/docs/software/finite-state-machines.html)
has a section on state machines.

In FTC state machines can help organize and enhance your op modes.

## Code

***Note: The code here is due for an update. Check back soon. (March 28,
2022)***

- RHSAutoStateMachineGyro.java - This is an autonomous op mode that uses a
state machine as well as some other useful coding strategies.

Here is the state diagram that goes with the op mode.

```mermaid
stateDiagram-v2
 
    [*] --> waitForStart
    waitForStart --> drivingToStorage: Start Pressed
    waitForStart --> waitForStart: Waiting
    drivingToStorage --> drivingToStorage: Not in Storage
    drivingToStorage --> inStorage: In Storage
    inStorage --> Stop
    Stop --> [*]
