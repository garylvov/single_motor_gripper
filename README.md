# Single Motor Gripper
### www.garylvov.com
      This package is designed to implement a ROS wrapper for a gripper actuated by a single dynamixel.

      It is currently configured for a copycat of the gripper included with Hello Robot's Stretch 
          See: www.garylvov.com/universal-gripper for more information about the universal gripper 
          copycat, including CAD files and a parts list.

      Package Overview:
          How to install:
              - Clone this repo into the /src folder of your catkin workspace
              - Clone Dynamixel Workbench into the /src folder of your catkin workspace
                  - https://github.com/ROBOTIS-GIT/dynamixel-workbench
              - Use Rosdep within the parent folder of your catkin workspace to resolve deps
                  - rosdep install -i --from-path src --rosdistro noetic -y
              - Run catkin_make from the parent folder of your catkin workspace

          How to run : roslaunch stretch_copycat_gripper gripper.launch
              - Troubleshooting: check what port the dynamixel is connected to
                  - Open Dynamixel Wizard, click Options to see what available ports
                  - Adjust the value in launch/gripper.launch arg: usb_port to match

          Functionality:
              - close.srv - no input arguments. Calling this service closes the gripper.
              - open.srv - no input arguments. Calling this service opens the gripper.

              - toggle.srv - no input arguments. Calling this service changes the gripper state
                      - if called in closed position opens the gripper
                      - if called in open position closes the gripper
                      - if called while in a partially closed position opens the gripper

              - partial.srv - input: uint8 (0 - 255). 255 corresponds to a complete close of the 
                              gripper, and 0 corresponds to the open position of the gripper. All 
                              intermediate values correspond to a partial close of the gripper.
                              This is designed for grasping fragile objects that could be damaged
                              from a full close. If toggle.srv is called after partial.srv, 
                              the gripper opens fully.

      How to modify this package for use with your gripper:
          - Your gripper needs to be actuated by a single dynamixel
          - Use Dynamixel Wizard to determine pertinent motor information
              - Modify the values withing config/standard.yaml to match motor EEPROM Area values
                  - change the default motor ID args within src/gripper_control.cpp in 
                    bool send_motor_request to match your motor ID. Not nessicary if your motor ID is 1.

              - Modify the baud rate within launch/gripper.launch arg baud_rate to match that of gripper

          - Use the Dynamixel Wizard to determine the current_position value of the motor 
            when the gripper is in the closed and open position
              - Modify the uint32_t close_pos and uint32_t open_pos to reflect the 
                open and closed position values 
                  - Ensure that those two variables equal motor values, not degree measurements
