# Robot Control and Motor Assignments

## **Drive Controller (Pilot)**

| PS5 Button             | Assigned Function                                            | Motor ID |
| ---------------------- | ------------------------------------------------------------ | -------- |
| X (Cross)              | Reset Heading                                                |          |
| O (Circle)             |                                                              |          |
| △ (Triangle)           | Ramp up wheels Click Again to cancel                         |          |
| □ (Square)             |                                                              |          |
| L1                     | Speed Mode (Slow Mech and Slow Steer)                        |          |
| L2                     | Slow Mode (set value)                                        |          |
| R1                     | Shooter Sequence Click Again to cancel (drive away to cancle too) |          |
| R2                     | Forced Intake In (HOLD)                                      |          |
| L3 (Left Stick Click)  |                                                              |          |
| R3 (Right Stick Click) | Change Driver Cam                                            |          |
| D-Pad Up               | Climb Up                                                     |          |
| D-Pad Down             | Climb Down                                                   |          |
| D-Pad Left             |                                                              |          |
| D-Pad Right            |                                                              |          |
| Options Button         |                                                              |          |
| Create Button          |                                                              |          |
| PS Button              |                                                              |          |
| Touchpad               |                                                              |          |
| Left Joystick          | Swerve Drive                                                 | 0-11     |
| Right Joystick         | Swerve Turning                                               | 0-11     |

## **Mech Controller (Operator)**

| PS5 Button             | Assigned Function                        | Motor ID |
| ---------------------- | ---------------------------------------- | -------- |
| X (Cross)              | Forced Pivot In (HOLD)                   |          |
| O (Circle)             | Pivot In / Out (Click for IN/OUT)        |          |
| △ (Triangle)           | Shooter Hood Positions (Multiple Clicks) |          |
| □ (Square)             | Shooter Wheel (Click for ON/OFF)         |          |
| L1                     | Hopper Shoot All Balls (HOLD)            |          |
| L2                     | Intake Rollers In Manual                 |          |
| R1                     | Hopper Reverse (Agitator) (HOLD)         |          |
| R2                     | Intake Rollers Out Manual                |          |
| L3 (Left Stick Click)  |                                          |          |
| R3 (Right Stick Click) |                                          |          |
| D-Pad Up               | Winch up Manual                          |          |
| D-Pad Down             | Winch down Manual                        |          |
| D-Pad Left             | Doornob down Manual                      |          |
| D-Pad Right            | Doornob up Manual                        |          |
| Options Button         |                                          |          |
| Create Button          |                                          |          |
| PS Button              |                                          |          |
| Touchpad               |                                          |          |
| Left Joystick          | Hood Up / Down Manual                    |          |
| Right Joystick         | Intake Pivot Up / Down Manual            |          |

## **CAN Assignments**

| Subsystem       | Component        | Type        | Position | CAN ID       |
| --------------- | ---------------- | ----------- | -------- | ------------ |
| **Swerve**      | Drive            | Kraken x60  | FL       | 1            |
|                 | Drive            | Kraken x60  | FR       | 3            |
|                 | Drive            | Kraken x60  | BR       | 5            |
|                 | Drive            | Kraken x60  | BL       | 7            |
|                 | Steer            | Kraken x44  | FL       | 0            |
|                 | Steer            | Kraken x44  | FR       | 2            |
|                 | Steer            | Kraken x44  | BL       | 4            |
|                 | Steer            | Kraken x44  | BR       | 6            |
|                 | CANcoder         | CANcoder    | FL       | 11           |
|                 | CANcoder         | CANcoder    | FR       | 10           |
|                 | CANcoder         | CANcoder    | BL       | 8            |
|                 | CANcoder         | CANcoder    | BR       | 9            |
| **Intake**      | Pivot            | Kraken x60  | -        | 12           |
|                 | Pivot Encoder    | Throughbore | -        | 13           |
|                 | Roller           | Kraken x44  | -        | 14           |
| **Hopper**      | Hopper           | Kraken x44  | -        | 15           |
| **Shooter**     | Hood             | Kraken x44  | -        | 16           |
|                 | Flywheel         | Kraken x44  | -        | 17           |
|                 | Flywheel Encoder | Throughbore | -        | 18           |
| **Climb**       | Doornob          | Kraken x44  | -        | 19           |
|                 | Doornob Encoder  | Throughbore | -        | 20           |
|                 | Winch            | Kraken x60  | -        | 21           |
| **CAN Devices** | CANdi            | CANdi       | -        | 22           |
|                 | CANdi            | CANdi       | -        | 23           |
|                 | CANivore         | CANivore    | -        | "swerve-can" |
|                 | CANivore         | CANivore    | -        | "mech-can"   |
|                 | Pigeon           | Pigeon 2.0  | -        | 24           |

## **Motor Configuration**

| Subsystem   | Motor Name     | ID | Current Limit | Max Velocity | Max Current | Max Acceleration |
| ----------- | -------------- | -- | ------------- | ------------ | ----------- | ---------------- |
| **Swerve**  | Drive FL       | 1  |               |              |             |                  |
|             | Drive FR       | 3  |               |              |             |                  |
|             | Drive BR       | 5  |               |              |             |                  |
|             | Drive BL       | 7  |               |              |             |                  |
|             | Steer FL       | 0  |               |              |             |                  |
|             | Steer FR       | 2  |               |              |             |                  |
|             | Steer BL       | 4  |               |              |             |                  |
|             | Steer BR       | 6  |               |              |             |                  |
| **Intake**  | Pivot          | 12 |               |              |             |                  |
|             | Roller         | 14 |               |              |             |                  |
| **Hopper**  | Hopper         | 15 |               |              |             |                  |
| **Shooter** | Hood           | 16 |               |              |             |                  |
|             | Flywheel       | 17 |               |              |             |                  |
| **Climb**   | Doornob        | 19 |               |              |             |                  |
|             | Winch          | 21 |               |              |             |                  |
