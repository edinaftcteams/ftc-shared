# State Machine

State machine, finite state machine, finite automation all end up at
*state machine* in
[WikiPedia](https://en.wikipedia.org/wiki/Finite-state_machine). Take a
look for a formal description of state machine. And of couurse
[gmZero](https://gm0.org/en/latest/docs/software/finite-state-machines.html)
has a section on state machines.

In FTC robot code, a state machines can help to organize and make it
easier to enhance and update your op modes.

## Code

***Note: The code here is due for an update. Check back soon. (March 28,
2022)***

- RHSAutoStateMachineGyro.java - This is an autonomous op mode that uses a
state machine as well as some other useful coding strategies.

Here is the state diagram that goes with the op mode.

### Autonomous
```mermaid
stateDiagram-v2
    [*] --> Initial
    state if_state <<choice>>
    Initial --> if_state
    if_state --> Park_In_Warehouse
    if_state --> Park_In_Storage
    Park_In_Warehouse --> Deliver_Duck
    Park_In_Storage --> Deliver_Duck
    Deliver_Duck --> Deliver_Freight
    state if_state2 <<choice>>
    Deliver_Freight --> if_state2
    if_state2 --> More_Freight
    if_state2 --> Stop
    More_Freight --> Stop
    Stop --> [*]
