# 2022 Dec 13
- Developed code for testing and tuning wheel speed control loop. Found in `\HIVE\motion\arduino\speed_control_loop_testing`.
    - This code accepts desired speeds for left motor [rad/s] (e.g. `<L,10>`) and will track them.
    - Can change gains and controller time step easily at top of `.ino` file.
