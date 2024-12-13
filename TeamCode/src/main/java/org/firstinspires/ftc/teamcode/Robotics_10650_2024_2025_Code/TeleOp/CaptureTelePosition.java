package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

import java.io.FileWriter;
import java.util.ArrayList;

@TeleOp(name = "RecordDriverPosition", group = "Linear Opmode")
public class CaptureTelePosition extends LinearOpMode {
    RobotInitialize robot;
    int liftPitchPosition = 0;
    int liftExtenderPosition = 0;
    double maxLifEtxtension =0;
    int speed = 2700;





    private ArrayList<String> recordedInputs;

        // Variable for speed control
        private double speedMultiplier = 1.0;

        @Override
        public void runOpMode() {

//                robot = new ProgBotInitialize(this, false);
            robot = new RobotInitialize(this, false);


            // InitileftDrivealize hardware


                recordedInputs = new ArrayList<>();


                telemetry.addData("Status", "Ready to record driver inputs");
                telemetry.update();

                waitForStart();

                long startTime = System.currentTimeMillis();
                double strafeVelocity;
                double straightMovementVelocity;
                double turnVelocity;

                double fleftVel = 0;
                double bleftVel = 0;
                double frightVel= 0 ;
                double brightVel = 0;

                double pitchVel = 0;
                double extenderVel = 0;

                double intakeVel = 0;
                double clawPitchPos = 0;

                double hangRPos = 1;



                while (opModeIsActive()&&!gamepad1.share) {

                    // Adjust speed with Cross (X) or Circle




                    {





                        if (gamepad1.right_bumper) {
                            robot.parkingServo.setPosition(0.946); //Where it can touch the bar
                        } else {
                            robot.parkingServo.setPosition(1); //all the way down
                        }


                        if (gamepad1.circle) {
                            speed = 270;
                        } else if(gamepad1.cross){
                            speed = 2800; //DcMotorEx max speed (encoder ticks per second): 2,787.625
                        } else{
                            speed = 1400;
                        }


                        strafeVelocity = Math.pow(gamepad1.right_stick_x, 3) * speed; // Min: -10000, Max: 10000
                        //telemetry.addData("gamepad1.left_stick_x (strafing)", strafePower);
                        turnVelocity = Math.pow(gamepad1.left_stick_x, 3) * speed; // Min: -10000, Max: 10000
                        //telemetry.addData("gamepad1.right_stick_x (turning)", turnPower);


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
//                        if (Math.abs(robot.liftPitch.getCurrentPosition() - robot.liftPitchPosition) > 25) {
//                            if (robot.liftPitch.getCurrentPosition() < robot.liftPitchPosition) {
//                                robot.liftPitch.setVelocity(2150);
//                            } else if (robot.liftPitch.getCurrentPosition() >= robot.liftPitchPosition) {
//                                robot.liftPitch.setVelocity(-2150);
//                                if (robot.liftPitchPosition > 1500) {
//                                    robot.liftPitch.setVelocity(-3000);
//                                }
//                            }
//                        }else {
//                            robot.liftPitch.setVelocity(0);
//                        }
                        telemetry.addData(" extender curent pos", robot.liftExtender.getCurrentPosition());
                        telemetry.addData("extender target pos", liftExtenderPosition);
                        telemetry.addData(" pitch curent pos", robot.liftPitch.getCurrentPosition());
                        telemetry.addData("pitch target pos", robot.liftPitchPosition);




                        telemetry.addData("is right stick pressed?", gamepad2.right_stick_button);
                        telemetry.addData("is left stick pressed?", gamepad2.left_stick_button);


//
                        telemetry.addData("ground mode pitch pos", Math.asin(53.78 / robot.liftExtender.getCurrentPosition()));

//            if (gamepad2.circle){
//                telemetry.addData("ground mode pitch pos", Math.asin(53.78/robot.liftExtender.getCurrentPosition()));
//
//
//            }
                        if (robot.liftPitchPosition <= 2325 && robot.liftPitchPosition >= -600 ||
                                (robot.liftPitchPosition >= 2325 && gamepad2.left_stick_y > 0) || // 3200 goes to the
                                // maximum horizontal position and further (try something less than this)
                                (robot.liftPitchPosition <= -600 && gamepad2.left_stick_y < 0)) {
                            if (liftExtenderPosition > maxLifEtxtension) {
                                liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
                            }
                            //determines where the lift pitch goes
                            if (gamepad2.left_stick_y < -0.2) {//going up

                                robot.liftPitchPosition = robot.liftPitchPosition + 40;
                                if (robot.liftPitchPosition > 2090) {
                                    robot.liftPitchPosition = robot.liftPitchPosition + 10;
                                }

                            } else if (gamepad2.left_stick_y > 0.2) {//going down

                                robot.liftPitchPosition = robot.liftPitchPosition - 35;
                                if (robot.liftPitchPosition > 2090) {
                                    robot.liftPitchPosition = robot.liftPitchPosition - 10;
                                }


//                    if (robot.liftPitch.getCurrentPosition()<400) {
//                        robot.liftPitch.setVelocity(470 * gamepad2.left_stick_y);
//                    }else if(robot.liftPitch.getCurrentPosition()>600){
//                        robot.liftPitch.setVelocity(1550 * gamepad2.left_stick_y);
//
//
//                    } else{
//                        robot.liftPitch.setVelocity(800 * gamepad2.left_stick_y);
//                        telemetry.addData("left pitch pos", robot.liftPitch.getCurrentPosition());
//
//                    }

                            }

                            if (robot.liftPitchPosition < -600) {
                                robot.liftPitchPosition = -600;
                            } else if (robot.liftPitchPosition > 2300) {
                                robot.liftPitchPosition = 2300;  //change to max lift xtension
                            }
                            // double targetAngle = liftPitchPosition * (90) / 2595;


                            if(robot.liftPitchPosition>=908&&robot.liftExtender.getCurrentPosition()>=470){
                                robot.liftPitchPosition=908;
                            }

                            // TODO: Prevent liftPitch from going above a certain amount IF the liftExtender currentPos is not retracted


                            //1300
                        }//2700


                        //lift pitch horizontal bounds

                        //if pitch degree is less than 31.25

                        //find positon for extension
                        telemetry.addData("lift extender pos", robot.liftExtender.getCurrentPosition());
                        telemetry.addData("lift extender target pos", liftExtenderPosition);

                        telemetry.addData("lift pitch pos", robot.liftPitch.getCurrentPosition());


                        //Bounds on the liftExtender motor
                        //Lift PIDF might be able to be tuned more to improve but it does reasonably well currently
                        double pitchAngle = robot.liftPitch.getCurrentPosition() * (90) / 2595;


                        if (pitchAngle >= 31.25) {
                            //maxLifEtxtension = 121 / (Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
                            maxLifEtxtension = 470;

                        } else {
                            maxLifEtxtension = 901;
                        }




                        if (gamepad2.left_stick_button) { //when this button is pressed
                            robot.liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            //press this button to set the zero point after adjusting with right stick button
                        } else {
                            robot.liftExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        }
                        telemetry.addData("max lift etension", 1567 / (Math.sin(Math.toRadians(pitchAngle))));
                        telemetry.addData("cos", (Math.sin(Math.toRadians(pitchAngle))));
                        telemetry.addData("pitch angle", (robot.liftPitch.getCurrentPosition() * 90) / 2595);
                        telemetry.addData("pitch angle", pitchAngle);


                        //if in bounnds, set new target pos

                        //used to be ((Math.abs(gamepad2.right_stick_y)>0.2)&&(liftExtenderPosition<=maxLifEtxtension)
                        //                    &&(liftExtenderPosition>=0)||(robot.liftExtender.getCurrentPosition()<0&&
                        //                    gamepad2.right_stick_y<0)||(robot.liftExtender.getCurrentPosition()>
                        //                    maxLifEtxtension&&gamepad2.right_stick_y>0))
                        if ((Math.abs(gamepad2.right_stick_y) > 0.2) && (liftExtenderPosition <= maxLifEtxtension)
                                && (liftExtenderPosition >= 0) || (robot.liftExtender.getCurrentPosition() < 0 &&
                                gamepad2.right_stick_y < 0) || (robot.liftExtender.getCurrentPosition() >
                                maxLifEtxtension && gamepad2.right_stick_y > 0)) {

                            liftExtenderPosition = liftExtenderPosition - (int) (23 * gamepad2.right_stick_y);


                            if (liftExtenderPosition < 0) {
                                liftExtenderPosition = 0;
                            } else if (liftExtenderPosition > maxLifEtxtension) {
                                liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
                            }

                        }


                        //Determines if the liftExtender should go up or down based on the controller inputs
                        if (liftExtenderPosition <= (5) && robot.liftExtender.getCurrentPosition() <= (5) && !gamepad2.right_bumper) {
                            //when down, save power
                            robot.liftExtender.setPower(0);
                        } else if (Math.abs(robot.liftExtender.getCurrentPosition() - liftExtenderPosition) > 25) { //Is the error value range
                            //if far from target position

                            //next if own or up
                            if (robot.liftExtender.getCurrentPosition() < liftExtenderPosition) {
                                robot.liftExtender.setPower(.6);
                            } else if (robot.liftExtender.getCurrentPosition() >= liftExtenderPosition) {
                                robot.liftExtender.setPower(-.6);
                            }
                            //If no input, make sure the liftExtender motor does not move
                        } else {
                            robot.liftExtender.setPower(0.01);
                        }
//
                        //ejaculates the block
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

                        if (gamepad2.dpad_down) {

                            robot.pitch.setPosition(0);

                        }
                        if (gamepad2.dpad_up) {

                            robot.pitch.setPosition(0.1606/5);
                        }

                        if (gamepad1.dpad_left) {
                            robot.parkingServo.setPosition(robot.parkingServo.getPosition() + 0.002);

//                robot.hangR.setPosition(0.9611);
//                robot.hangL.setPosition(0.0439);
                        }
                        if (gamepad1.dpad_right) {
                            robot.parkingServo.setPosition(robot.parkingServo.getPosition() - 0.002);

//                robot.hangR.setPosition(0.9611);
//                robot.hangL.setPosition(0.0439);
                        }
                        telemetry.addData("hang r pos", robot.parkingServo.getPosition());

                        if (gamepad2.circle) {
//                before 2167
                            robot.liftPitchPosition = 2207;
                            liftExtenderPosition = 0;

                            //edit this to be valid for the dual mode servo
                        }

                        //edit this to be valid for the dual mode servo
                    }

                    if (gamepad2.square) {//slaps it in
                        liftExtenderPosition = 0;
                        robot.liftPitchPosition = 272;
                    }

                    if (gamepad2.triangle) {//to score hifh basket
//                robot.liftPitch(0, 0.2);
//                telemetry.addData("Pitchpos", robot.liftPitch.getCurrentPosition());
                        robot.liftPitchPosition = 272;
                        liftExtenderPosition = 901;
                    }

                    // Record inputs with timestamp
                    if(gamepad2.options) {
                        recordedInputs.add((System.currentTimeMillis() - startTime) + "," + (robot.fLeft.getCurrentPosition()) + "," + (robot.fRight.getCurrentPosition()) + "," + (robot.bLeft.getCurrentPosition()) + "," + (robot.bRight.getCurrentPosition()) );
                    }
//                    telemetry.addData("Recording", "fleft Velocity: %.2f, fright Velocity: %.2f, bleft Velocity: %.2f, bright Velocity %.2f,", (startTime),(fleftVel),(frightVel),(bleftVel), (brightVel));
                    telemetry.update();
                }

                // Save recorded inputs after the session

                saveInputsToFile("/sdcard/FIRST/recordedInputsOnce.txt");

        }

        private void saveInputsToFile(String filename) {
            try (FileWriter writer = new FileWriter(filename)) {
                for (String input : recordedInputs) {
                    writer.write(input + "\n");
                }
                telemetry.addData("Status", "Inputs saved for future use");
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }
            telemetry.update();
        }
    }



