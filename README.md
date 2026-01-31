# Robot Control and Motor Assignments

## **Drive Controller (Pilot)**

| PS5 Button                     | Assigned Function  | Motor ID   |
| ------------------------------ | ------------------ | ---------- |
| X (Cross)                      |                    |           |
| O (Circle)                     |                    |          |
| △ (Triangle)                   |                    |          |
| □ (Square)                     |                    |           |
| L1                             |                    |         |
| L2                             |                    |          |
| R1                             |                    |           |
| R2                             |                    |           |
| L3 (Left Stick Click)          |                    |         |
| R3 (Right Stick Click)         |                    |          |
| D-Pad Up                       |                    |     |
| D-Pad Down                     |                    |           |
| D-Pad Left                     |                    |            |
| D-Pad Right                    |                    |            |
| Options Button + Create Button |                    |            |

## **Mech Controller (Operator)**

| PS5 Button                     | Assigned Mech Function | Motor ID |
| ------------------------------ | ---------------------- | ---------- |
| X (Cross)                      |                        |             |
| O (Circle)                     |    Spinup shooter      | 16       |
| △ (Triangle)                   |                        |           |
| □ (Square)                     |    Hopper Spin         | 15       |
| L1                             |                        |          |
| R1                             |    Roller intake       |  14       |
| L2                             |    Intake Pivot Up     |  12       |
| R2                             |    Intake Pivot Down   |12       |
| L3 (Left Stick Click)          |                        |          |
| R3 (Right Stick Click)         |                        |      |
| D-Pad Up                       |    Doornob down        |      |
| D-Pad Down                     |    Doornob up          |       |
| D-Pad Left                     |                        |        |
| D-Pad Right                    |                        |      |
| Options Button                 |                        |   |
| Create Button                  |                        |         |
| Track pad                      | Intake in + stop roller|     |
| Left Joystick                  | Winch up + down        |    |



## **Motor & CAN Assignments**

| Component           | Motor Type     | Position | Port |
| ------------------- | -------------- | -------- | ---- |
| **Drive**           | Kraken x60        | FL       |   1  |
|                     | -        | FR       |   3  |
|                     | -         | BR       |   5  |
|                     | -         | BL       |   7  |
| **Steer**           | Kraken x44         | FL       |   0  |
|                     | -         | FR       |   2  |
|                     | -         | BL       |   4  |
|                     | -         | BR       |   6  |
| **CANcoder**        | CANcoder       | FL       |  11  |
|                     |     -   | FR       |  10  |
|                     |      -  | BL       |   8  |
|                     |       - | BR       |   9  |
| **Intake Pivot**    | Kraken x60      | -        |  12  |
| **Intake Roller**   | Kraken x44        | -        |  14  |
| **Hopper**          | Kraken x44        | -        |  15  |
| **Shooter**         | Kraken x44         | -        |  16  |
|                     | Kraken x44        | -        |  17  |
| **Climb**           | Kraken x44        | Arm     |  18  |
|                     | Kraken x60     | Winch    |  19  |
