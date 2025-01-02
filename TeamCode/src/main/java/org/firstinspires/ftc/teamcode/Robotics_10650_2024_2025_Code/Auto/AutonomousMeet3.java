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

    public void score() {
        robot.moveLiftPitch(800, 0.8,false);
        robot.pitch.setPosition(0.0339);
        robot.extenderToPos(880, 0.8,false);
        sleep(350);
        robot.moveLiftPitch(965, 0.5,true);
        sleep(800);
        robot.extake(500);
        robot.moveLiftPitch(800, 0.9,true);
        robot.extenderToPos(0, 0.4,false);
    }

    // The code that runs in Auto
    @Override
    public void runOpMode() throws InterruptedException {
        // Way for the RobotInitialize class to be used inside of this class
        robot = new RobotInitialize_RunToPos(this, true);
        // Waits for a person to press start on the control hub
        // then it runs the rest of the program
        waitForStart();

        robot.moveLiftPitch(800, 0.8,false);
        robot.executeMoveAlt(60, 345, 0, 2500, false); //First position

        //Process of scoring preloaded sample
        score();

        //TEMPORARY COMMENT: Might need to backup slightly to get back to initial position before next move
        robot.executeMoveAlt(950, -400, 0, 2500, false);
        robot.pitch.setPosition(0.0339);
        robot.moveLiftPitch(3090, 0.8,false);
        robot.intake();
        robot.executeMoveAlt(0, 200, 0, 3500, false);
        robot.intake.setPower(0);
        robot.moveLiftPitch(800, 0.8,true);
        robot.executeMoveAlt(-950, 200, 0, 2500, false); //First position again

        score();

        robot.executeMoveAlt(964, -200, 0, 2500, false);
        robot.pitch.setPosition(0.0339);
        robot.moveLiftPitch(3090, 0.8,false);
        robot.intake();
        robot.executeMoveAlt(0, 150, 0, 3500, false);
        robot.intake.setPower(0);
        robot.moveLiftPitch(800, 0.8,true);
        robot.executeMoveAlt(-921, 20, 0, 2500, false); //First position again

        score();



    }
}