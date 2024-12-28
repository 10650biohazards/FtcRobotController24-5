// Program created by: Danny and William
// Purpose: FTC Robot Software

package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

@TeleOp (name = "TeleOp_RobotCentric_Cleaned")
public class TeleOpCode_RobotCentric_Cleaned extends LinearOpMode {

    // Run the initialize function
    RobotInitialize robot;

    int liftExtenderPosition = 0;
    double maxLifEtxtension = 0;
    int speed = 2700;



    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize(this, false);
        robot.liftPitchPosition = 885;





        // initialization of the control of the robot when start is pressed
        waitForStart();


        robot.liftPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initial position
        robot.parkingServo.setPosition(1);


        robot.liftExtender.setTargetPosition(0);
        robot.liftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftExtender.setTargetPositionTolerance(100);


        // loop while the program is running
        // waits for controller input then runs the associated code
        while (opModeIsActive()) {
            // controller inputs that is inputted by the drive team
            controllerInput();
        }
    }

    public void controllerInput() {
        //robot.intake.setPower(0);

        // Gamepad usages (two gamepads in use, one for driving and one for mechanisms):

        // Gamepad1 is used for driving (motor controls)
        // Gamepad2 is used for mechanism manipulation (moving servos and the lift motors)


        // Variables that store the different game pad movements for ease of reference later
        // Gamepad1 configuration




            if (gamepad1.circle) {
                speed = 270;
            } else if(gamepad1.cross){
                speed = 2800; //DcMotorEx max speed (encoder ticks per second): 2,787.625
            } else{
                speed = 1400;
            }


            double strafeVelocity; // (left stick x-axis movement)
            strafeVelocity = Math.pow(gamepad1.right_stick_x, 3) * speed; // Min: -10000, Max: 10000
            //telemetry.addData("gamepad1.left_stick_x (strafing)", strafePower);
            double turnVelocity; // (right stick x-axis movement)
            turnVelocity = Math.pow(gamepad1.left_stick_x, 3) * speed; // Min: -10000, Max: 10000
            //telemetry.addData("gamepad1.right_stick_x (turning)", turnPower);

            double straightMovementVelocity;

            if (gamepad1.left_trigger > 0) {
                //slow
                straightMovementVelocity = Math.pow(gamepad1.left_trigger, 3) * speed;
                //strafeVelocity = 0*(gamepad1.left_trigger);

            } else if (gamepad1.right_trigger > 0) {
                //slow
                straightMovementVelocity = -Math.pow(gamepad1.right_trigger, 3) * speed;
                //strafeVelocity = 0*(gamepad1.right_trigger);
                //turnVelocity = 0 *(gamepad1.right_trigger);


            } else {
                straightMovementVelocity = 0;
            }


            // Set velocity of the motors (drivetrain)
            // Forward and backward movement (left stick y-axis movement)
            // Left and right turning (right stick x-axis movement)
            // Strafing left and right (left stick x-axis movement)
            {
                robot.fLeft.setVelocity(strafeVelocity - straightMovementVelocity + turnVelocity); // Overall
                // negative value
                robot.fRight.setVelocity(-strafeVelocity - straightMovementVelocity - turnVelocity); // Overall
                // positive value
                robot.bLeft.setVelocity(strafeVelocity + straightMovementVelocity - turnVelocity); // Overall
                // positive value
                robot.bRight.setVelocity(-strafeVelocity + straightMovementVelocity + turnVelocity); // Overall
                // negative value
            }
        }


        {

//            if(gamepad2.left_bumper){//emergency button stops all movement (we can change that actual button later)
//                liftPitchPosition =robot.liftPitch.getCurrentPosition();
//                liftExtenderPosition =robot.liftPitch.getCurrentPosition();
//
//            }

        //determines the speed
        if (Math.abs(robot.liftPitch.getCurrentPosition() - robot.liftPitchPosition) > 20) {
            if (robot.liftPitch.getCurrentPosition() < robot.liftPitchPosition) {
                robot.liftPitch.setVelocity(2000);
            } else if (robot.liftPitch.getCurrentPosition() >= robot.liftPitchPosition) {
                robot.liftPitch.setVelocity(-2000);
                if (robot.liftPitchPosition > 1500) {
                    robot.liftPitch.setVelocity(-2000);
                }
            }
        }else {
            robot.liftPitch.setVelocity(0);
        }


        if (robot.liftPitchPosition <= 3210 && robot.liftPitchPosition >= 0 ||
                (robot.liftPitchPosition >= 3210 && gamepad2.left_stick_y > 0) || // 3200 goes to the
                // maximum horizontal position and further (try something less than this)
                (robot.liftPitchPosition <= 0 && gamepad2.left_stick_y < 0)) {
            if (liftExtenderPosition > maxLifEtxtension) { //TODO: Move
                liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
            }
            //determines where the lift pitch goes
            if (gamepad2.left_stick_y < -0.2) {//going up
                if (robot.liftPitchPosition > 2800&&!gamepad2.left_bumper) {
                    robot.liftPitchPosition = robot.liftPitchPosition + 10;
                } else {
                    robot.liftPitchPosition = robot.liftPitchPosition + 40;
                }


            } else if (gamepad2.left_stick_y > 0.2) {//going down
                if (robot.liftPitchPosition > 2800&&!gamepad2.left_bumper) {
                    robot.liftPitchPosition = robot.liftPitchPosition - 10;
                } else {
                    robot.liftPitchPosition = robot.liftPitchPosition - 35;
                }

            }

            if (robot.liftPitchPosition < 300) {
                robot.liftPitchPosition = 300;
            } else if (robot.liftPitchPosition > 3185) {
                robot.liftPitchPosition = 3185;  //change to max lift xtension
            }


            if(robot.liftPitchPosition>=1793&&robot.liftExtender.getCurrentPosition()>=1808&&liftExtenderPosition>=483){
                liftExtenderPosition = 483;
            }

            // TODO: Prevent liftPitch from going above a certain amount IF the liftExtender currentPos is not retracted



        }


        //lift pitch horizontal bounds

        //if pitch degree is less than 31.25

        //find positon for extension
        double pitchAngle = (double)(robot.liftPitch.getCurrentPosition()-885) * (90) / 2595;





        if (pitchAngle >= 31.25) {
        //maxLifEtxtension = 121 / (Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
            maxLifEtxtension = 482;
        } else {
            maxLifEtxtension = 880;
        }


        if (gamepad2.options) {
            robot.liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("yep options is pressed");
            //press this button to set the zero point after adjusting with right stick button
        } else {
            robot.liftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }


        //if in bounnds, set new target pos
        if ((Math.abs(gamepad2.right_stick_y) > 0.2) && (liftExtenderPosition <= maxLifEtxtension)
                && (liftExtenderPosition >= 0) || (robot.liftExtender.getCurrentPosition() < 0 &&
                gamepad2.right_stick_y < 0) || (robot.liftExtender.getCurrentPosition() >
                maxLifEtxtension && gamepad2.right_stick_y > 0)) {

            liftExtenderPosition = liftExtenderPosition - (int) (20 * gamepad2.right_stick_y);


            if (liftExtenderPosition < 0) {
                liftExtenderPosition = 0;
            } else if (liftExtenderPosition > maxLifEtxtension) {
                liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
            }

        }


        robot.liftExtender.setTargetPosition(liftExtenderPosition);
        robot.liftExtender.setPower(0.5);

        //intake and extake
        if (gamepad2.left_trigger != 0) {
            robot.intake.setPower(-1.0);
            telemetry.addData("intake power", robot.intake.getPower());
        } else if (gamepad2.right_trigger != 0) {
            robot.intake.setPower(1.0);
            telemetry.addData("intake power", robot.intake.getPower());

        } else {
            robot.intake.setPower(0.0);
            telemetry.addData("intake power", robot.intake.getPower());
        }

        if (gamepad2.dpad_down) {//pitch claw down
            robot.pitch.setPosition(0);
            //robot.pitch.setPosition(robot.pitch.getPosition()-0.001);
        }
        if (gamepad2.dpad_up) {//pitch claw up
            robot.pitch.setPosition(0.0339);
            //robot.pitch.setPosition(robot.pitch.getPosition()+0.001);
        }

        if (gamepad1.dpad_left) {
            robot.parkingServo.setPosition(robot.parkingServo.getPosition() + 0.002);
        }
        if (gamepad1.dpad_right) {
            robot.parkingServo.setPosition(robot.parkingServo.getPosition() - 0.002);
        }

        if (gamepad2.circle) {//reaches into submersible
            robot.liftPitchPosition = 3111;
            liftExtenderPosition = 0;
        }
        if (gamepad2.square) {//slaps it in
            liftExtenderPosition = 0;
            robot.liftPitchPosition = 1157;
        }
        if (gamepad2.triangle) {//to score high basket
            robot.liftPitchPosition = 1157;
            liftExtenderPosition = 850;
        }
        if(gamepad1.square){//raises to get out of submersible
            liftExtenderPosition = 0;
            robot.liftPitchPosition = 2235;
        }
        if (gamepad1.right_bumper) {
            robot.parkingServo.setPosition(0.946); //Where it can touch the bar
        } else {
            robot.parkingServo.setPosition(1); //all the way down
        }

        // accelerationAdditive is 1428.57
        // The intended result is that when the control sticks are not stationary the speed slowly
        // increases until it gets to the max value of 10000 or -10000

        /*
        if encoder value too diff from initial, then increase speed
        as change in position increases, speed increases

        diff int = currentpos-initpos
        speed

        */

//        ElapsedTime accelerationTime = new ElapsedTime();
//
//        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0|| gamepad1.right_stick_x != 0) {
//            for (int i = 0; i < 7; i++) {
//                strafePower = gamepad1.left_stick_x * 1428.57;
//                turnPower = gamepad1.right_stick_x * 1428.57;
//                straightMovementPower = gamepad1.left_stick_y * 1428.57;
//            }
//        }

        // Prints to the robot driver station screen
        telemetry.update();
    }
}
//}