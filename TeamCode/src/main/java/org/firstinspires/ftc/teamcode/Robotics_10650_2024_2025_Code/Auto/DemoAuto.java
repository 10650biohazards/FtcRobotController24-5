// Program created by: Danny and William
// Purpose: FTC Robot Software

// The file path of the class
package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.Auto;

// Import all of the necessary FTC libraries and code

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize_RunToPos;

// Create an Autonomous program (Auto) that preselects a TeleOp (controller operated)
@Disabled
@Autonomous(name = "Demo Auto", preselectTeleOp = "OLD_TeleOpCode_RobotCentric")
public class DemoAuto extends LinearOpMode {

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
//

        robot.executeMove(0, 400,0, 1000);
        robot.executeMove(400, 0,0, 1000);
        robot.executeMove(0, -400,0, 1000);
        robot.executeMove(-400, 0,0, 1000);
        robot.executeMove(400, 400,0, 1000);
        robot.executeMove(-400, 0,0, 1000);
        robot.executeMove(400, -400,0, 1000);




        while (opModeIsActive()) {
            telemetry.addData("xpos", robot.odom.getPosX());
            telemetry.addData("ypos", robot.odom.getPosY());
            telemetry.update();
        }




    }
}