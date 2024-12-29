## Rationale

The driver station can take two controllers as inputs.
This means we can have a seperate operator (only the arm controls) and driver (only the driving controls)
The benefit of this is to extract the most out of those who will operate and drive the robot by focusing on their strenghts


## Technical implementation

Basically assume driver 1 = gamepad1.xxxx, and operator = gamepad2.xxxx in the code. So whenever programming for the operator,
set gamepad2, when driver, set gamepad1.