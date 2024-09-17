package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "NEW Basic TeleOp")
public class BasicTeleOp extends LinearOpMode {
// Create servo variables
    Servo pitch;
    Servo Lclaw;
    Servo Rclaw;

    // Create the empty motor variables
    DcMotor fleft;
    DcMotor bright;
    DcMotor fright;
    DcMotor bleft;

    public void controllerInput() {
        // Variables that store the different game pad movements for ease of reference later

        float strafePower; // (left stick x-axis movement)
        strafePower = gamepad1.left_stick_x;
        float turnPower; // (right stick x-axis movement)
        turnPower = gamepad1.right_stick_x;
        float straightMovementPower; // (left stick y-axis movement)
        straightMovementPower = gamepad1.left_stick_y;

        // Set power of the motors
        // Forward and backward movement (left stick y-axis movement)
        // Left and right turning (right stick x-axis movement)
        // Strafing left and right (left stick x-axis movement)
        fleft.setPower(-strafePower + straightMovementPower - turnPower);
        bright.setPower(-strafePower + straightMovementPower + turnPower);
        fright.setPower(strafePower + straightMovementPower + turnPower);
        bleft.setPower(strafePower + straightMovementPower - turnPower);

        // Servo setting up

        // Makes the pitch servo go all the way up
        // Make sure claw is fully closed before lifting up (set up conditional for this)
        if (gamepad1.triangle && (Lclaw.getPosition() <= 0.2 && Lclaw.getPosition() >= 0.1) &&
                (Rclaw.getPosition() >= 0.8 && Rclaw.getPosition() <= 0.9)) {
            pitch.setPosition(1);
        }

        // Is supposed to make the pitch servo touch the ground (it keeps going too far down right now)
        if (gamepad1.cross) {
            pitch.setPosition(0.6);
        }

        // Makes the Lclaw and Rclaw servos open (fix right claw)
        if (gamepad1.circle) {
            // 0.6 is maximum value of Lclaw (opening all the way to the left)
            // Lclaw.setPosition(0.6); (Temporarily disabled but still working)
            // The range value works as intended for this servo

            Rclaw.setPosition(0.4);

            // For some reason the Rclaw needs a very specific setPosition
            // between 0.4-0.5 [0.4?]? As Rclaw setPosition value approaches 1 the claw closes
            // inwards; As Rclaw setPosition value goes away from 1 (0.9 or below)
            // the claw opens outwards (negative values are not necessary)
        }

        // Makes the Lclaw and Rclaw servos close (fix the right claw)
        if (gamepad1.square) {
            // Lclaw goes right as setPosition decreases
            // Lclaw goes left as setPosition increases
            // Lclaw.setPosition(0.5); (Temporarily disabled but still working)
            Rclaw.setPosition(0.9);
            // Fix the right claw
        }

        // Prints to the robot driver station screen
        telemetry.addData("Test", "This is a test"); // Adds to the list of statements to
        // print
        // Prints on the driver station screen
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
// map the motors to the hardware map
        fleft = hardwareMap.get(DcMotor.class, "fleft");
        bright = hardwareMap.get(DcMotor.class, "bright");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

// map the servos to the hardware map
        pitch = hardwareMap.get(Servo.class, "pitch");
        Lclaw = hardwareMap.get(Servo.class, "lClaw");
        Rclaw = hardwareMap.get(Servo.class, "rClaw");

        // initialization of the control of the robot

        waitForStart();

        // loop
        while(opModeIsActive()) {
            // controller code

            controllerInput();
        }

    }
}