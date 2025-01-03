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
        robot.moveLiftPitch(840, 0.8,false);
        robot.pitch.setPosition(0.0339);
        robot.extenderToPos(904, 0.8,true);
        robot.moveLiftPitch(1060, 0.5,true);
        sleep(800);
        robot.extake(500);
        sleep(700);
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

        robot.moveLiftPitch(840, 0.8,false);
        robot.executeMoveAlt(30, 300, 0, 2500, false); //ydist was 345
        //First position


        //Process of scoring preloaded sample
        score();

        robot.executeMoveAlt(1000, -345, 0, 3000, false);
        robot.pitch.setPosition(0.0339);
        robot.moveLiftPitch(3100, 0.8,false);
        robot.intake();
        robot.executeMoveAlt(0, 189, 0, 3500, false);
        robot.intake.setPower(0);
        robot.moveLiftPitch(840, 0.8,false);//maybe chnage to true
        robot.executeMoveAlt(-1000, 145, 0, 2500, false); //First position again

        score(); //Second sample in the high basket

        robot.executeMoveAlt(994, -102, 0, 3000, false);
        robot.pitch.setPosition(0.0339);
        robot.moveLiftPitch(3100, 0.8,false);
        robot.intake();
        robot.executeMoveAlt(0, 230, 0, 3500, false);
        robot.intake.setPower(0);
        robot.moveLiftPitch(840, 0.8,true);
        robot.executeMoveAlt(-981, -115, 0, 3000, false); //First position again

        score(); //Third sample in the high basket

        robot.executeMoveAlt(400, 0, 0, 2000, false);





    }
}