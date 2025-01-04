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
        robot.extenderToPos(860, 0.8,true);
        robot.moveLiftPitch(1030, 0.5,true);
        robotSleep(800);
        robot.extake(500);
        robotSleep(300);
        robot.moveLiftPitch(800, 0.9,true);
        robot.extenderToPos(0, 0.4,false);
    }

    final double GROUND_PITCH_POS = 0.0339;
    final double BASKET_PITCH_POS = 0.0309;

    // The code that runs in Auto
    @Override
    public void runOpMode() throws InterruptedException {
        // Way for the RobotInitialize class to be used inside of this class
        robot = new RobotInitialize_RunToPos(this, true);
        // Waits for a person to press start on the control hub
        // then it runs the rest of the program
        waitForStart();

        robot.pitch.setPosition(BASKET_PITCH_POS);

        robot.moveLiftPitch(840, 0.8,false);
        robot.executeMoveAlt(20, 315, 0, 3000, false); //ydist was 345
        //First position


        //Process of scoring preloaded sample
        score();

        //Pushes first spike mark sample out of the way
        robot.executeMoveAlt(1110, -360, 0, 3000, false);

        robot.pitch.setPosition(GROUND_PITCH_POS);

        //Moves back left to get second spike mark sample
        robot.executeMoveAlt(-125, -0, 0, 5000, false);

        robot.moveLiftPitch(3100, 0.9,false);
        robot.intake();
        robot.executeMoveAlt(0, 189, 0, 4500, false);
        robot.intake.setPower(0);
        robot.moveLiftPitch(840, 0.9,false);//maybe chnage to true
       // robot.extenderToPos(890, 0.8,false);

        robot.pitch.setPosition(BASKET_PITCH_POS);

        robot.executeMoveAlt(-1015, 145, 0, 2500, false, 860, 600); //First position again

        score(); //Second sample in the high basket

        robot.pitch.setPosition(GROUND_PITCH_POS);

        robot.executeMoveAlt(1025, -70, 0, 3000, false);

        robot.moveLiftPitch(3100, 0.9,false);
        robot.intake();
        robot.executeMoveAlt(0, 168, 0, 3500, false);
        robot.intake.setPower(0);
        robot.moveLiftPitch(840, 0.9,true);
        robot.pitch.setPosition(BASKET_PITCH_POS);
        robot.executeMoveAlt(-1011, -85, 0, 3000, false, 860, 600); //First position again

        score(); //Third sample in the high basket


        robot.executeMoveAlt(400, 0, 0, 2000, false);

    }

    public void robotSleep(int delay){
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis()-startTime < delay) {

        }
    }

}