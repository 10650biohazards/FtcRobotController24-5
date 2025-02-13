// Program created by: Danny and William
// Purpose: FTC Robot Software

package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

@TeleOp (name = "TeleOp_RobotCentric_statses")
public class TeleOpCode_RobotCentric_States extends LinearOpMode {

    // Run the initialize function
    RobotInitialize robot;

    int liftExtenderPosition = 0;
    double maxLifEtxtension = 0;
    int speed;

    Boolean circleOn =false;
    double groundCorrecter = 0;


    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize(this, false);
        robot.liftPitchPosition = 885;
        // initialization of the control of the robot when start is pressed
        waitForStart();
        //robot.pitch.setPosition(0.1856);
        robot.pitch.setPosition(0.2206);


        circleOn=false;
        //initial position
        robot.parkingServo.setPosition(1);

        robot.liftPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftExtender.setTargetPosition(0);
        robot.liftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftExtender.setTargetPositionTolerance(70);

        robot.liftPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.liftPitch.setTargetPosition(0);
        robot.liftPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftPitch.setTargetPositionTolerance(100);




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
            speed = 540;
        } else if (gamepad1.cross) {
            speed = 3600; //DcMotorEx max speed (encoder ticks per second): 2,787.625
        } else {
            speed = 2400;
        }
        telemetry.addData("drive speed", speed);

        telemetry.addData("angle", robot.gyroScope.getRobotYawPitchRollAngles().getYaw());


        // TESTING??????
//        speed = speed*2;


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
        telemetry.addData("straight velocity", straightMovementVelocity);



        // Set velocity of the motors (drivetrain)
        // Forward and backward movement (left stick y-axis movement)
        // Left and right turning (right stick x-axis movement)
        // Strafing left and right (left stick x-axis movement)
        {
            // Using this RobotInitialize, fleft and bright are reversed.
            robot.fLeft.setVelocity(strafeVelocity - straightMovementVelocity + turnVelocity); // Overall
            // negative value
            robot.fRight.setVelocity(-strafeVelocity - straightMovementVelocity - turnVelocity); // Overall
            // positive value
            robot.bLeft.setVelocity(strafeVelocity + straightMovementVelocity - turnVelocity); // Overall
            // positive value
            robot.bRight.setVelocity(-strafeVelocity + straightMovementVelocity + turnVelocity); // Overall
            // negative value
        }
        int maxPitch = 3430;



//            if(gamepad2.left_bumper){//emergency button stops all movement (we can change that actual button later)
//                liftPitchPosition =robot.liftPitch.getCurrentPosition();
//                liftExtenderPosition =robot.liftPitch.getCurrentPosition();
//
//            }
        if (robot.liftPitchPosition < 480) {
            robot.liftPitchPosition = 480;
        } else if (robot.liftPitchPosition > maxPitch) {
            robot.liftPitchPosition = maxPitch;


        }

        if (robot.liftPitchPosition <= maxPitch && robot.liftPitchPosition >= 0 ||
                (robot.liftPitchPosition >= maxPitch && gamepad2.left_stick_y > 0) || // 3200 goes to the
                (robot.liftPitchPosition <= 0 && gamepad2.left_stick_y < 0)) {

            robot.liftPitchPosition += (int)(-40*Math.pow(gamepad2.left_stick_y, 3));

            //cannot go outside these bounds
            if (robot.liftPitchPosition < 480) {
                robot.liftPitchPosition = 480;
            } else if (robot.liftPitchPosition > maxPitch) {
                robot.liftPitchPosition = maxPitch;
            }

            //horizontal bound
            if (robot.liftPitchPosition >= 1793 && robot.liftExtender.getCurrentPosition() >= 1808 && liftExtenderPosition >= 483){
                robot.liftPitchPosition = 483;
            }

            // TODO: Prevent liftPitch from going above a certain amount IF the liftExtender currentPos is not retracted
        }

        if (circleOn) {//raises to get out of submersible
            //robot.liftPitchPosition=(int)((-580*(Math.asin(998/(((22.0/246.0)* robot.liftExtender.getCurrentPosition())+1000))))+3944);
//            robot.liftPitchPosition=(int)((-500*(Math.asin(999/(((22.0/220.0)* robot.liftExtender.getCurrentPosition())+1000))))+3953);
            //robot.liftPitchPosition=(int)((-500*(Math.asin(999/(((22.0/220.0)* robot.liftExtender.getCurrentPosition())+1000))))+3953+groundCorrecter);

//            robot.liftPitch.setTargetPositionTolerance(30);
//            robot.liftPitch.setPower(1.0);


            robot.pitch.setPosition(((0.0288/482)* robot.liftExtender.getCurrentPosition())+0.2206);
        } else {
            robot.liftPitch.setTargetPositionTolerance(70);
            robot.liftPitch.setPower(0.7);
        }
        robot.liftPitch.setTargetPosition(robot.liftPitchPosition);


        //determines the speed
//        if (Math.abs(robot.liftPitch.getCurrentPosition() - robot.liftPitchPosition) > 20) {
//            if (robot.liftPitch.getCurrentPosition() < robot.liftPitchPosition) {
//                robot.liftPitch.setVelocity(2000);
//            } else if (robot.liftPitch.getCurrentPosition() >= robot.liftPitchPosition) {
//                robot.liftPitch.setVelocity(-2000);
//                if (robot.liftPitchPosition > 1500) {
//                    robot.liftPitch.setVelocity(-2000);
//                }
//            }
//        } else {
//            robot.liftPitch.setVelocity(0);
//        }


        //lift pitch horizontal bounds
        //if pitch degree is less than 31.25
        //find positon for extension
        double pitchAngle = (double) (robot.liftPitch.getCurrentPosition() - 885) * (90) / 2595;

        if (pitchAngle >= 29.9) {
            //maxLifEtxtension = 121 / (Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
            maxLifEtxtension = 363;
        } else {
            maxLifEtxtension = 900;
        }



        //if in bounds + buttons moved, set new target pos
        if ((Math.abs(gamepad2.right_stick_y) > 0.2) && (liftExtenderPosition <= maxLifEtxtension)
                && (liftExtenderPosition >= 0) || (robot.liftExtender.getCurrentPosition() < 0 &&
                gamepad2.right_stick_y < 0) || (robot.liftExtender.getCurrentPosition() >
                maxLifEtxtension && gamepad2.right_stick_y > 0)) {

            liftExtenderPosition = liftExtenderPosition - (int) (20 * gamepad2.right_stick_y);

        }

        if (liftExtenderPosition < 0) {
            liftExtenderPosition = 0;
        } else if (liftExtenderPosition > maxLifEtxtension) {
            liftExtenderPosition = (int) maxLifEtxtension;
            telemetry.addData("recognizes new maxluft", "jjjj");

        }
        telemetry.addData("liftPos", robot.liftExtender.getCurrentPosition());
        telemetry.addData("liftextenderPos", liftExtenderPosition);

        telemetry.addData("pitch anlge", pitchAngle);
        telemetry.addData("maxLiftEc=xtensio", maxLifEtxtension);



        robot.liftExtender.setTargetPosition(liftExtenderPosition);
        if (liftExtenderPosition==0&&robot.liftExtender.getCurrentPosition()<10){
            robot.liftExtender.setPower(0);
        } else {
            robot.liftExtender.setPower(1.0);
        }


//        if ((Math.abs(gamepad2.right_stick_y) > 0.2) && (liftExtenderPosition <= maxLifEtxtension)
//                && (liftExtenderPosition >= 0) || (robot.liftExtender.getCurrentPosition() < 0 &&
//                gamepad2.right_stick_y < 0) || (robot.liftExtender.getCurrentPosition() >
//                maxLifEtxtension && gamepad2.right_stick_y > 0)) {
//
//            liftExtenderPosition = liftExtenderPosition - (int) (20 * gamepad2.right_stick_y);
//
//            if (liftExtenderPosition < 0) {
//                liftExtenderPosition = 0;
//            } else if (liftExtenderPosition > maxLifEtxtension) {
//                liftExtenderPosition = (int) maxLifEtxtension;
//            }
//        }





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

            //groundCorrecter = groundCorrecter+1;
           // robot.pitch.setPosition(0);
            robot.pitch.setPosition(robot.pitch.getPosition()-0.001);
        } else if (gamepad2.dpad_up) {//pitch claw up

            //groundCorrecter = groundCorrecter-1;

            // robot.pitch.setPosition(0.1856);
            robot.pitch.setPosition(robot.pitch.getPosition()+0.001);
        }

//        if (gamepad1.dpad_left) {
//            robot.pitch.setPosition(0);
//
//            //robot.pitch.setPosition(robot.pitch.getPosition()+0.001);
//            //robot.lActuator.setTargetPosition(robot.lActuator.getCurrentPosition() + 2);
//        }
//        if (gamepad1.dpad_right) {
//            robot.pitch.setPosition(robot.pitch.getPosition()+0.001);
//
//            //robot.lActuator.setTargetPosition(robot.lActuator.getCurrentPosition() - 2);
//        }

        if(gamepad2.circle) {
            robot.pitch.setPosition(0.2206);
            robot.liftPitchPosition = 3201;
        }

        if (gamepad2.right_bumper) {//reaches into submersible
            circleOn=true;
        } else if (gamepad2.square) {//slaps it in
            circleOn=false;
            liftExtenderPosition = 0;
            robot.liftPitchPosition = 1051;
            robot.pitch.setPosition(0.2206);
        } else if (gamepad2.triangle) {//to score high basket
            circleOn=false;
//            robot.liftPitchPosition = 1051;
            robot.liftPitchPosition = 579;
            robot.pitch.setPosition(0.5828);
            liftExtenderPosition = 788;
        } else if (gamepad2.cross){
            robot.liftPitchPosition = 1240;
            robot.pitch.setPosition(0.22);
            liftExtenderPosition = 883;
        }

        if (gamepad2.options) {//sets the zero point after adjusting with right stick button
            robot.liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("yep options is pressed");
        } else {//normal mode
            robot.liftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad1.square) {//raises to get out of submersible
            circleOn=false;
            liftExtenderPosition = 0;
            robot.liftPitchPosition = 2235;
        }


        if (gamepad2.left_bumper){
            circleOn=false;
//            robot.liftPitchPosition = 3201;
//            liftExtenderPosition = 0;
        }


        if (gamepad1.dpad_down) {//pitch claw down

            //sweep = groundCorrecter+1;
            // robot.pitch.setPosition(0);
            //robot.sweep.setPosition(robot.sweep.getPosition()-0.001);
        } else if (gamepad1.dpad_up) {//pitch claw up

            //groundCorrecter = groundCorrecter-1;

            // robot.pitch.setPosition(0.1856);
            //robot.pitch.setPosition(robot.pitch.getPosition()+0.001);
            //robot.sweep.setPosition(robot.sweep.getPosition()+0.001);
        }
        /*


        if(hangOn button){
           lActuator.setTargetPosition();
           rActuator.setTargetPosition();

            lActuator.setPower();
            rActuator.setPower();
        }

        if(liftOff button){
           lActuator.setTargetPosition();
           rActuator.setTargetPosition();

            lActuator.setPower();
            rActuator.setPower();
        }

        if(180 tringle buttn){
            robot.liftPitchPosition= 885;
            robot.pitch.setPosition(xxx);
            liftExtenderPosition= 980;

        }




         */




        if (gamepad1.right_bumper) {
            robot.parkingServo.setPosition(0.9617); //Where it can touch the bar
        } else if(gamepad1.left_bumper){
            robot.parkingServo.setPosition(1); //all the way down
        }

        double gorundModePos = -190*(Math.asin(1000/(((22/24)* robot.liftExtender.getCurrentPosition())+1000))+3490);
        telemetry.addData("parking pos", robot.parkingServo.getPosition());

        telemetry.addData("pitch servo pos", robot.pitch.getPosition());
        telemetry.addData(" pitch pos", robot.liftPitch.getCurrentPosition());
        telemetry.addData(" ext pos", robot.liftExtender.getCurrentPosition());
        telemetry.addData("ground mode pos", (-620*(Math.asin(998/(((62.0/220.0)* robot.liftExtender.getCurrentPosition())+1000))))+4136);
        //telemetry.addData("ground mode servo pos", ((1.0/9040.0)* robot.liftExtender.getCurrentPosition())+0.1856);

        telemetry.addData("circle mode?", circleOn);
        telemetry.addData("powa", robot.liftPitch.getPower());
        telemetry.addData("tolerance", robot.liftPitch.getTargetPositionTolerance());
        telemetry.addData("ideal position", robot.liftPitchPosition);
        telemetry.addData("ext Powa", robot.liftExtender.getPower());


//        telemetry.addData(" lactuator pos", robot.lActuator.getCurrentPosition());
//        telemetry.addData(" ractuator pos", robot.rActuator.getCurrentPosition());
//        telemetry.addData(" sweep pos", robot.sweep.getPosition());









        // Prints to the robot driver station screen
        telemetry.update();
    }
}
//}