package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

// Imports all of the necessary FTC libraries and code
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// How to connect to robot procedure

public class RobotInitialize {

    // Initialization Phase

    // Create servo variables (currently not available)
    // Servo pitch;
    // Servo lClaw;
    // Servo rClaw;

    // Create the empty normal motor variables
    DcMotorEx fLeft;
    DcMotorEx bRight;
    DcMotorEx fRight;
    DcMotorEx bLeft;
    // DcMotorEx liftMotor; (Placeholder for fifth motor)

    // Add servos once they are on the main robot

    // Create empty gyroscope variable
    BHI260IMU gyroScope;
    BHI260IMU.Parameters settings;

    // A constructor that makes a new instance of
    // The orientation class called lastAngles
    Orientation lastAngles = new Orientation();
    double globalAngle;

    // Makes an instance of the class LinearOpMode called opMode
    LinearOpMode opMode;

    // Enables the class to be referenced in other classes such as the Autonomous Code
    // and the TeleOpCode
    public RobotInitialize(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    // The main function that sets all of the hardware to different variables
    // The motors and the gyroscope are initialized here
    public void initialize() {
        // map the motors to the hardware map
        fLeft = opMode.hardwareMap.get(DcMotorEx.class, "fleft");
        bRight = opMode.hardwareMap.get(DcMotorEx.class, "bright");
        fRight = opMode.hardwareMap.get(DcMotorEx.class, "fright");
        bLeft = opMode.hardwareMap.get(DcMotorEx.class, "bleft");

        // The front left and back right motors are reversed so all wheels go in the same direction
        // When a positive or negative value is used
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);


// map the servos to the hardware map (not currently in use but will be used later)
//        pitch = opMode.hardwareMap.get(Servo.class, "pitch");
        //lClaw = opMode.hardwareMap.get(Servo.class, "lClaw");
        //rClaw = opMode.hardwareMap.get(Servo.class, "rClaw");

        // Resetting the encoders (distance measurement sensors)
        // and then start them again on program start
        // Repeat for all motors
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //without odom: bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Initialize Gyroscope
        gyroScope = opMode.hardwareMap.get(BHI260IMU.class, "gyroScope");

        RevHubOrientationOnRobot ori = new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 90, 0, 0,0));
        settings = new BHI260IMU.Parameters(ori);
        AngularVelocity angularVelocity = gyroScope.getRobotAngularVelocity(AngleUnit.DEGREES);
        YawPitchRollAngles orientation = gyroScope.getRobotYawPitchRollAngles();
        gyroScope.initialize(settings);

    }


    // Gets the angle that the robot is currently facing in from the gyroscope
    private double getAngle() {
        Orientation angles = gyroScope.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        Orientation angles = gyroScope.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360; // Changes the deltaAngle to be within a range of 0 to 360 degrees
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Sets the variable globalAngle to itself plus deltaAngle
        globalAngle += deltaAngle; // gA = gA + deltaAngle
        lastAngles = angles; // Sets lastAngles equal to
        return globalAngle;
    }

    // This function makes the robot travel a relativeDistance specified by the parameter relativeDistance
    // This parameter is measured in encoder ticks; the other parameter, velocity, is
    // a decimal value that determines how fast the robot will go
    public void goStraight (int distance, double velocity) {
        // Travel a certain relativeDistance based on the absolute value
        // of the average robot encoder reading
        // The relativeDistance is current position plus or minus the value that is being moved
        // RELATIVE DISTANCE MEASUREMENT IN USE
        int relativeDistance = distance + getPosStrafe();
        // Go forwards or backwards
        // It only moves if the distance to the final location is greater than or equal to 10 encoder
        // ticks
        while (opMode.opModeIsActive() && Math.abs(getPosStrafe() - relativeDistance) >= 10) {
            //if the current position is before final position
            if (getPosStrafe() < relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                setMotorVelocity(Math.abs(velocity));
                opMode.telemetry.addData("Encoder straight", getPosStrafe());
                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
                //if current position is less than needed
            } else if (getPosStrafe() > relativeDistance) {
                // Backwards (- negative relativeDistance value)
                setMotorVelocity(-Math.abs(velocity));
                opMode.telemetry.addData("Encoder straight", getPosStrafe());
                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        stopMotors();
    }

    // Makes the robot strafe right by determining where the robot is currently
    // located and where it is trying to go it does not return anything and
    // has parameters of the distance it needs to travel (measured in encoder ticks)
    // and the velocity that it moves (measured in encoder ticks per second)
    public void strafeR(int distance, int velocity){
        int relativeDistance = distance + getPosStrafe();
        // Go forwards or backwards
        //if difference <10 then stop
        // 10 is the accuracy tolerance in 10 encoder ticks
        while (opMode.opModeIsActive() && Math.abs(getPosStrafe() - relativeDistance) >= 10) {
            //if
            if (getPosStrafe() < relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                //setMotorVelocity(Math.abs(velocity));
                fLeft.setVelocity(velocity);
                fRight.setVelocity(-velocity);
                bLeft.setVelocity(velocity);
                bRight.setVelocity(-velocity);

//                while (){
//
//                }
                opMode.telemetry.addData("position", getPosStrafe());
                opMode.telemetry.addData("relative distance", relativeDistance);
                opMode.telemetry.addData("old pos", getAverageEncoderValue());
                opMode.telemetry.addData("must be greater than 10", Math.abs(getPosStrafe() - relativeDistance));
                opMode.telemetry.update();
            } else{
                setMotorVelocity(0);
            }
        }
        stopMotors();
    }

    // Makes the robot strafe left by determining where the robot is currently
    // located and where it is trying to go it does not return anything and
    // has parameters of the distance it needs to travel (measured in encoder ticks)
    // and the velocity that it moves (measured in encoder ticks per second)
    public void strafeL(int distance, int velocity){
        //relative distance is input value + pos
        int relativeDistance = distance + getPosStrafe();
        // Go forwards or backwards
        while (opMode.opModeIsActive() && Math.abs(getPosStrafe() - relativeDistance) >= 10) {
            if (getPosStrafe() < relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                //setMotorVelocity(Math.abs(velocity));
//
                Double[] motorvVelocity = {bLeft.getVelocity(), fLeft.getVelocity(), bRight.getVelocity(), fRight.getVelocity()};
                fLeft.setVelocity(-velocity);
                fRight.setVelocity(velocity);
                bLeft.setVelocity(-velocity);
                bRight.setVelocity(velocity);
                opMode.telemetry.addData("get vel", fLeft.getVelocity());
                opMode.telemetry.addData("get vel", bLeft.getVelocity());
                opMode.telemetry.addData("getposStrafeR", getPosStrafe());
                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        stopMotors();
    }

    // This is the new turn function that includes setVelocity instead of setPower
    public void newTurnFunction(int degrees) {
        // When turning left, counterclockwise is a positive gyro value
        // When turning right, clockwise is a negative gyro value
        // ABSOLUTE POSITIONING IN USE (will go to exact values)

        while (opMode.opModeIsActive() && Math.abs(degrees - getAngle()) >= 0.2) {
            opMode.telemetry.addData("Encoder turn:", gyroScope.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            opMode.telemetry.addData("Gyroscope", getAngle());
            opMode.telemetry.update();

            if (degrees > getAngle()) {
                // Turning left (positive gyro value)
                fLeft.setVelocity(-500);
                bLeft.setVelocity(500);
                fRight.setVelocity(500);
                bRight.setVelocity(-500);
            }
            else if (degrees < getAngle()) {
                // Turning right (negative gyro value)
                fLeft.setVelocity(500);
                bLeft.setVelocity(-500);
                fRight.setVelocity(-500);
                bRight.setVelocity(500);
            }
        }
        stopMotors();
    }
    //gets average magnitudes because motors are going in different directions
    public int getPosStrafe() {
        return ((Math.abs(fRight.getCurrentPosition())+Math.abs(bRight.getCurrentPosition())+Math.abs(fLeft.getCurrentPosition())+ Math.abs(bLeft.getCurrentPosition()))/4);
    }

    //
//    public int getPosStrafeL() {
//        return ((-fright.getCurrentPosition()+ bright.getCurrentPosition()+fleft.getCurrentPosition()- bleft.getCurrentPosition())/4);
//    }
    // Calculates the average encoder value
    // It takes the left and right motor encoder locations, then averages them
    public int getAverageEncoderValue() {
        return ((getLeftSideEncoderValues() + getRightSideEncoderValues()) / 2);
    }

    // Calculates the average left side encoder values
    // It takes the left side encoder location of the left side motors and averages them
    public int getLeftSideEncoderValues() {
        return ((fLeft.getCurrentPosition() + bLeft.getCurrentPosition())/2);
    }

    // Calculates the average right side encoder values
    // It takes the right side encoder location of the right side motors and averages them
    public int getRightSideEncoderValues() {
        return ((fRight.getCurrentPosition() + bRight.getCurrentPosition())/2);
    }

    // This sets the movement of the motors to be constant
    // The back wheels are set to negative velocity so the
    // robot goes forward when the velocity value is positive and
    // vice versa for going backwards
    public void setMotorVelocity(double velocity) {
        fLeft.setVelocity(velocity);
        fRight.setVelocity(velocity);
        bLeft.setVelocity(-velocity);
        bRight.setVelocity(-velocity);
    }

    // Stops the motors by setting the velocity to 0
    public void stopMotors() {
        setMotorVelocity(0);
    }
}


