# State Machines

## Testing Mermaid
```mermaid
stateDiagram-v2
 
    [*] --> waitForStart
    waitForStart --> drivingToStorage: Start Pressed
    waitForStart --> waitForStart: Waiting
    drivingToStorage --> drivingToStorage: Not in Storage
    drivingToStorage --> inStorage: In Storage
    inStorage --> Stop
    Stop --> [*]
