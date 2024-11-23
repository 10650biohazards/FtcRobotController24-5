package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "correct")

public class teleCorrectLift extends LinearOpMode{
    RobotInitialize robot;
    int liftPitchPosition = 0;
    double p = 1;
    double i = 1;
    double d = -2.5;
    double f = 3;



    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotInitialize(this, false);
        //robot.clawRoll.setPosition(0.1867);
        waitForStart();
        // int i = 0;
        while (opModeIsActive()) {
            if (Math.abs(robot.liftPitch.getCurrentPosition() - liftPitchPosition) > 50) {
                if (robot.liftPitch.getCurrentPosition() < liftPitchPosition) {
                    robot.liftPitch.setVelocity(2250);
                } else if (robot.liftPitch.getCurrentPosition() >= liftPitchPosition) {
                    robot.liftPitch.setVelocity(-2250);
                    if (liftPitchPosition > 1500) {
                        robot.liftPitch.setVelocity(-2450);
                    }
                }
            } else {
                robot.liftPitch.setVelocity(0);
            }

            if (liftPitchPosition <= 3000 && liftPitchPosition >= 0 ||
                    (liftPitchPosition >= 3000 && gamepad2.left_stick_y > 0) || // 3200 goes to the
                    // maximum horizontal position and further (try something less than this)
                    (liftPitchPosition <= 0 && gamepad2.left_stick_y < 0)) {

                //determines where the lift pitch goes
                if (gamepad2.left_stick_y > 0.2) {//going down

                    liftPitchPosition = liftPitchPosition - 25;
                    if (liftPitchPosition > 1500) {
                        liftPitchPosition = liftPitchPosition - 15;

                    }


                } else if (gamepad2.left_stick_y < -0.2) {//going up
                    liftPitchPosition = liftPitchPosition + 25;

                }

                if (liftPitchPosition < 0) {
                    liftPitchPosition = 0;
                } else if (liftPitchPosition > 3000) {
                    liftPitchPosition = 3000;  //change to max lift xtension
                }
                if (gamepad1.left_bumper){
                    p = p+.01;
                }
                if (gamepad1.left_trigger>0){
                    p = p-.01;
                }
                if (gamepad1.right_bumper){
                    i = i+.01;
                }
                if (gamepad1.right_trigger>0){
                    i = i-.01;
                }
                if (gamepad2.left_bumper){
                    d = d+.01;
                }
                if (gamepad2.left_trigger>0){
                    d = d-.01;
                }
                if (gamepad2.right_bumper){
                    f = f+.01;
                }
                if (gamepad2.right_trigger>0){
                    f = f-.01;
                }

                robot.liftPitch.setVelocityPIDFCoefficients(1,1,-2.5, 3);

                telemetry.addData("p",p);
                telemetry.addData("i",i);

                telemetry.addData("d",d);

                telemetry.addData("f",f);


//                telemetry.addData("extenderhpos", robot.liftExtender.getCurrentPosition());
                telemetry.addData("lift pitchpos", robot.liftPitch.getCurrentPosition());

                //robot.clawRoll.setPosition(robot.clawRoll.getPosition()+ gamepad1.left_stick_x*0.0001);
                //robot.pitch.setPosition(robot.pitch.getPosition()+ gamepad1.right_stick_x*0.0001);

                // telemetry.addData("clawRoll Pos", robot.clawRoll.getPosition());
                //telemetry.addData("clawPitch Pos",robot.pitch.getPosition());

//            robot.liftPitch.setVelocit(500*gamepad2.right_stick_y);
//                robot.hangR.setPosition(robot.hangR.getPosition() + (gamepad1.right_stick_x * 0.01));
//
//                robot.hangL.setPosition(robot.hangL.getPosition() + (gamepad1.left_stick_x * 0.001));
//
//
//                telemetry.addData("hang r pos", robot.clawRoll.getPosition());
//                telemetry.addData("hang l pos", robot.pitch.getPosition());

                telemetry.update();
            }
        }
    }
}

