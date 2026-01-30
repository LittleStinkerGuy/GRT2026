# Robot Control and Motor Assignments

## **Drive Controller (Pilot)**

| PS5 Button                     | Assigned Function  | Motor Type | Motor ID   |
| ------------------------------ | ------------------ | ---------- | ---------- |
| X (Cross)                      |                    |            |            |
| O (Circle)                     |                    |            |            |
| △ (Triangle)                   |                    |            |            |
| □ (Square)                     |                    |            |            |
| L1                             |                    |            |            |
| L2                             |                    |            |            |
| R1                             |                    |            |            |
| R2                             |                    |            |            |
| L3 (Left Stick Click)          |                    |            |            |
| R3 (Right Stick Click)         |                    |            |            |
| D-Pad Up                       |                    |            |            |
| D-Pad Down                     |                    |            |            |
| D-Pad Left                     |                    |            |            |
| D-Pad Right                    |                    |            |            |
| Options Button + Create Button |                    |            |            |

## **Mech Controller (Operator)**

| PS5 Button                     | Assigned Mech Function | Motor Type | Motor ID |
| ------------------------------ | ---------------------- | ---------- | -------- |
| X (Cross)                      |                        |            |          |
| O (Circle)                     |    Spinup shooter      | Kraken     | 16       |
| △ (Triangle)                   |                        |            |          |
| □ (Square)                     |    Hopper Spin         | Kraken     | 15       |
| L1                             |                        |            |          |
| R1                             |    Roller intake       | Kraken     | 14       |
| L2                             |    Intake Pivot Up     | Kraken     | 12       |
| R2                             |    Intake Pivot Down   | Kraken     | 12       |
| L3 (Left Stick Click)          |                        |            |          |
| R3 (Right Stick Click)         |                        |            |          |
| D-Pad Up                       |    Doornob down        | Kraken     | 20       |
| D-Pad Down                     |    Doornob up          | Kraken     | 20       |
| D-Pad Left                     |                        |            |          |
| D-Pad Right                    |                        |            |          |
| Options Button                 |                        |            |          |
| Create Button                  |                        |            |          |
| Track pad                      | Intake in + stop roller| Kraken     | 14       |
| Left Joystick                  | Winch up + down        | Kraken     | 19       |



## **Motor & CAN Assignments**

| Component           | Motor Type     | Position | Port |
| ------------------- | -------------- | -------- | ---- |
| **Drive**           | Kraken         | FL       |   1  |
|                     | Kraken         | FR       |   3  |
|                     | Kraken         | BR       |   5  |
|                     | Kraken         | BL       |   7  |
| **Steer**           | Kraken         | FL       |   0  |
|                     | Kraken         | FR       |   2  |
|                     | Kraken         | BL       |   4  |
|                     | Kraken         | BR       |   6  |
| **CANcoder**        | CANcoder       | FL       |  11  |
|                     | CANcoder       | FR       |  10  |
|                     | CANcoder       | BL       |   8  |
|                     | CANcoder       | BR       |   9  |
| **Intake Pivot**    | Kraken         | -        |  12  |
| **Intake Roller**   | Kraken         | -        |  14  |
| **Hopper**          | Kraken         | -        |  15  |
| **Shooter**         | Kraken         | -        |  16  |
|                     | Kraken         | -        |  17  |
| **Climb**           | Kraken         | Left     |  18  |
|                     | Kraken         | Right    |  19  |
| **Winch**           | Kraken         | -        |  20  |
| **Doorknob**        | Kraken         | -        |  21  |

