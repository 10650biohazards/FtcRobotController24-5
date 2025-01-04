// Program created by: Danny and William
// Purpose: FTC Robot Software

// The file path of the class
package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.Auto;

// Import all of the necessary FTC libraries and code

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize_RunToPos;

// Create an Autonomous program (Auto) that preselects a TeleOp (controller operated)
@Autonomous(name = "TestingAutoMeet3")
public class TestingAutonomousMeet3 extends LinearOpMode {

    // Execute the function from the RobotInitialize class
    RobotInitialize_RunToPos robot;


    // The code that runs in Auto
    @Override
    public void runOpMode() throws InterruptedException {
        // Way for the RobotInitialize class to be used inside of this class
        robot = new RobotInitialize_RunToPos(this, true);
        // Waits for a person to press start on the control hub
        // then it runs the rest of the program
        waitForStart();

        robot.executeMoveAlt(30, 300, 0, 2500, false); //ydist was 345
        //First position

        sleep(1000);
        //Process of scoring preloaded sample


        robot.executeMoveAlt(1100, -345, 0, 3000, false);
        robot.executeMoveAlt(-100, -0, 0, 3000, false);



        robot.executeMoveAlt(0, 189, 0, 3500, false);

        robot.executeMoveAlt(-1000, 145, 0, 2500, false); //First position again
        sleep(1000);


        robot.executeMoveAlt(994, -102, 0, 3000, false);


        robot.executeMoveAlt(0, 230, 0, 3500, false);

        robot.executeMoveAlt(-981, -115, 0, 3000, false); //First position again

        sleep(1000);

        robot.executeMoveAlt(1400, -700, 0, 2000, false);





    }
}