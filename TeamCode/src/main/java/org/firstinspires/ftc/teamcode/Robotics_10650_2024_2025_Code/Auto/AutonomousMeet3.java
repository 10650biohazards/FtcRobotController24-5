// Program created by: Danny and William
// Purpose: FTC Robot Software

// The file path of the class
package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.Auto;

// Import all of the necessary FTC libraries and code

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;
import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize_RunToPos;

// Create an Autonomous program (Auto) that preselects a TeleOp (controller operated)
@Autonomous(name = "AutoMeet3", preselectTeleOp = "TeleOpCode_RobotCentric")
public class AutonomousMeet3 extends LinearOpMode {

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

//        robot.moveLiftPitch(800, 0.2);
//        robot.pitch.setPosition(0.0339);
//        robot.extenderToPos(849, 0.2);
//        robot.liftPitch(1000, 0.5);
//        sleep(500);
//        robot.extake(1000);
//        robot.moveLiftPitch(800, 0.2);
//        robot.extenderToPos(0, 0.2);



//

        robot.executeMoveAlt(60, 210, 0, 1000, false); //First position
        robot.executeMoveAlt(0, 0, -41, 1000, true); // Turn before second position
        robot.executeMoveAlt(711, 25, -41, 2000, false);
        robot.moveLiftPitch(800, 0.2);
        robot.pitch.setPosition(0.0339);
        robot.moveLiftPitch(3090, 0.6);
        robot.intake();
        robot.executeMoveAlt(0, 300, -41, 2000, false);
        robot.intake.setPower(0);

//











    }
}