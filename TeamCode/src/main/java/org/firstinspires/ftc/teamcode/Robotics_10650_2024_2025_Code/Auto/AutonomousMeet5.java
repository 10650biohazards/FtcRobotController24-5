// Program created by: Danny and William
// Purpose: FTC Robot Software

// The file path of the class
package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.Auto;

// Import all of the necessary FTC libraries and code

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize_RunToPos;

// Create an Autonomous program (Auto) that preselects a TeleOp (controller operated)
@Autonomous(name = "AutoMeet5", preselectTeleOp = "OLD_TeleOpCode_RobotCentric")
public class AutonomousMeet5 extends LinearOpMode {

    // Execute the function from the RobotInitialize class
    RobotInitialize_RunToPos robot;

    public void score() {
        robot.moveLiftPitch(840, 0.8,false);
        robot.extenderToPos(856, 0.8,true);//830 today apparently
        robot.moveLiftPitch(1149, 0.3,true);
        robotSleep(1400);
        robot.extake(500);
        robotSleep(300);
        robot.moveLiftPitch(800, 0.9,true);
        robot.extenderToPos(0, 0.4,false);
    }

    final double GROUND_PITCH_POS = 0.2206;
    final double BASKET_PITCH_POS = 0.220;
    //0.0309

    // The code that runs in Auto
    @Override
    public void runOpMode() throws InterruptedException {
        // Way for the RobotInitialize class to be used inside of this class
        robot = new RobotInitialize_RunToPos(this, true);
        // Waits for a person to press start on the control hub
        // then it runs the rest of the program
        waitForStart();

        //robot.executeMoveTouchSensor(-300, 40, 0, 3000,false  ,Integer.MIN_VALUE, Integer.MIN_VALUE, 1, 1, 800);

        //sleep(400000);
        robot.pitch.setPosition(BASKET_PITCH_POS);

        //robot.executeMoveDistanceSensors(20, 300, 0, 3000, false, Integer.MIN_VALUE, Integer.MIN_VALUE, 1,3, false, 9.6); //ydist was 345

        //sleep(1000000000);
        //robot.moveLiftPitch(840, 0.8,false);hgufwghjsghjgwy

        //First position (move forward and right from original location)
        robot.executeMoveDistanceSensors(0, 270, 0, 4000, false, Integer.MIN_VALUE, Integer.MIN_VALUE, 1,2, false, 350, true); //ydist was 345

       //robot.extenderToPos(100, 0.8,true);//830 today apparently
        score();



        //Process of scoring preloaded sample
        //score();

        //Pushes first spike mark sample out of the way (going right)
        robot.executeMoveAlt(1110, -360, 0, 3000, false);



        robot.pitch.setPosition(GROUND_PITCH_POS);

        //Moves back left to get second spike mark sample
        robot.moveLiftPitch(3200, 0.9,false);

        robot.executeMoveDistanceSensors(-125, 0, 0, 5000, false,Integer.MIN_VALUE,Integer.MIN_VALUE,1,1, false, 620, false);

        sleep(1000000000);

        //Pick up sample off of second spike mark
        robot.intake();
        robot.executeMoveAlt(0, 189, 0, 4500, false,Integer.MIN_VALUE,Integer.MIN_VALUE,1,1.3, false);
        robot.intake.setPower(0);
        robot.moveLiftPitch(840, 0.9,false);//maybe chnage to true
       // robot.extenderToPos(890, 0.8,false);

        robot.pitch.setPosition(BASKET_PITCH_POS);

        robot.executeMoveTouchSensor(-1015, 169, 0, 2500, false, 860, 600, 1, 1, 400); //First position again

        score(); //Second sample in the high basket

        robot.pitch.setPosition(GROUND_PITCH_POS);

        //Go to position to get sample from third spike mark
        robot.executeMoveAlt(1025, -70, 0, 3000, false);

        robot.moveLiftPitch(3100, 1,true);

        //Pick up sample from third spike mark
        robot.intake();
        robot.executeMoveAlt(0, 153, 0, 3500, false,Integer.MIN_VALUE,Integer.MIN_VALUE,1,1, true);
        robot.intake.setPower(0);
        robot.moveLiftPitch(840, 0.9,true);
        robot.pitch.setPosition(BASKET_PITCH_POS);
        robot.executeMoveTouchSensor(-1011, -61, 0, 3000, false, 860, 600,1,2, 400); //First position again

        robot.moveLiftPitch(840, 0.8,false);
        robot.extenderToPos(860, 0.8,true);//830 today apparently
        robot.moveLiftPitch(1030, 0.5,true);
        robotSleep(1000);
        robot.extake(500);
        robotSleep(300);
        robot.moveLiftPitch(780, 0.9,true);
        robot.extenderToPos(0, 0.4,false); //Third sample in the high basket


        robot.executeMoveLastStrafe(0, -40, 0, 4000);
//        robot.parkingServo.setPosition(0.9617);
//        robot.executeMoveLastStraight(0, -650, 0, 4000);
//        robot.parkingServo.setPosition(0.9617);


    }

    public void robotSleep(int delay){
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis()-startTime < delay) {

        }
    }

}