#include "ros/ros.h"
#include "single_motor_gripper/close.h"
#include "single_motor_gripper/open.h"
#include "single_motor_gripper/toggle.h"
#include "single_motor_gripper/partial.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

class Gripper
{
    public:
        Gripper(ros::NodeHandle *nh)
            {
                motor_client = nh->serviceClient<dynamixel_workbench_msgs::DynamixelCommand>
                ("/dynamixel_workbench/dynamixel_command");
                close_service = nh->advertiseService("gripper/close", &Gripper::close_gripper, this);
                open_service = nh->advertiseService("gripper/open", &Gripper::open_gripper, this);
                toggle_service = nh->advertiseService("gripper/toggle", &Gripper::toggle_gripper, this);
                partial_service = nh->advertiseService("/gripper/partial", &Gripper::partial_gripper, this);
            }
    
    private:
        bool state; // State is true if closed, false if open

        // motor position values for approximately open/closed orientation
        // determined experimentally from viewing motor position at close/open in Dynamixel Wizard
        uint32_t close_pos = 340; // approx. 201 degrees
        uint32_t open_pos = 687; // approx. 104 degrees

        // determines the motor orientation in the gripper to be able to perform a partial close 
        // see func partial_gripper : motor_orientation is either 1 or -1
        int motor_orientation = ((int)close_pos - (int)open_pos) / abs(((int)close_pos - (int)open_pos));

        ros::ServiceClient motor_client;

        ros::ServiceServer close_service;
        ros::ServiceServer open_service; 
        ros::ServiceServer toggle_service; 
        ros::ServiceServer partial_service; 
        
        // service callback functions 
        bool close_gripper(single_motor_gripper::close::Request &req,
                        single_motor_gripper::close::Response &res){
            state = true;
            return (this->send_motor_request(close_pos));
        }

        bool open_gripper(single_motor_gripper::open::Request &req,
                        single_motor_gripper::open::Response &res){
            state = false;
            return (this->send_motor_request(open_pos));
        }

        bool toggle_gripper(single_motor_gripper::toggle::Request &req,
                            single_motor_gripper::toggle::Response &res){
            if (state){ // state is true, so gripper is closed, to toggle open gripper
                state = false;
                return (this->send_motor_request(open_pos));
            }
            else{ // state is false, so gripper is closed, to toggle close gripper
                state = true;
                return (this->send_motor_request(close_pos));
            }
        }

        bool partial_gripper(single_motor_gripper::partial::Request &req,
                        single_motor_gripper::partial::Response &res){
            /* This is meant for partial closing upon fragile objects that would be damaged 
               by a full close. The state is changed to be closed so that the toggle function still works
               within this context - toggling from a partial close opens the gripper. */
            double percentage_close = req.value / 255;
            uint32_t close_value = (percentage_close * (open_pos - close_pos) * motor_orientation) + open_pos;
            state = true; 
            return (this->send_motor_request(close_value));
        }

        // client function : communicates with the dynamixel workbench to control the motor
        bool send_motor_request(uint32_t value, std::string command="", uint8_t id=1, 
                                        std::string addr_name="Goal_Position"){
            
            ROS_INFO("Calling Motor Server...");
            this->motor_client.waitForExistence();
            dynamixel_workbench_msgs::DynamixelCommand motor_srv_req;
            motor_srv_req.request.value = value;
            motor_srv_req.request.command = command;
            motor_srv_req.request.id = id;
            motor_srv_req.request.addr_name = addr_name;

            if(this->motor_client.call(motor_srv_req))
            {
                ROS_INFO("Motor Service Call Succesful");
                return true;
            }
            else{
                ROS_ERROR("Failed to Call Dynamixel Workbench Motor Service");
                return false;
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_control");
    ros::NodeHandle nh;
    Gripper g = Gripper(&nh);
    ros::spin();
    return 0;
}