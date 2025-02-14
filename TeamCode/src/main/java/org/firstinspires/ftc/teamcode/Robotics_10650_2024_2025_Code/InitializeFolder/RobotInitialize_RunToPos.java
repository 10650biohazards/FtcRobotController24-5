// Program created by: Danny and William
// Purpose: FTC Robot Software

package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder;

// Imports all of the necessary FTC libraries and code

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.LinkedList;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

// How to connect to robot procedure (see file called
// controlHubConnectionInstructions.md)

public class RobotInitialize_RunToPos {

    // Initialization Phase
    public int liftPitchPosition = 783;

    // Create servo variables
    public CRServo intake; // This is a special continuous rotation servo which allows it to act
    // like a motor
    public Servo parkingServo;

    public Servo pitch;
    public RevTouchSensor touch1;
    public RevTouchSensor touch2;
    public AnalogInput distanceSensor1;
    public AnalogInput distanceSensor2;

    public ModernRoboticsI2cRangeSensor sonicDistanceSensor;


    // Create the empty normal motor variables
    public DcMotorEx fLeft;
    public DcMotorEx bRight;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;

    // Create the empty lift control variables
    public DcMotorEx liftExtender; //Extends the lift outwards and pulls it inwards
    public DcMotorEx liftPitch; //Makes the lift go down to the floor and back up to perpendicular
    // with the drivetrain (uses worm gear)

    public GoBildaPinpointDriver odom;


    // Create empty gyroscope variable and its settings
    public BHI260IMU gyroScope;
    BHI260IMU.Parameters settings;

    // A constructor that makes a new instance of
    // The orientation class called lastAngles
    Orientation lastAngles = new Orientation();
    double globalAngle;

    // Makes an instance of the class LinearOpMode called opMode
    LinearOpMode opMode;

    // Enables the class to be referenced in other classes such as the Autonomous Code
    // and the OLD_TeleOpCode_RobotCentric
    public RobotInitialize_RunToPos(LinearOpMode opMode, boolean isAuto) {
        this.opMode = opMode; //Sets the argument for opMode to be the current opMode
        initialize(isAuto);
    }

    // The main function that sets all of the hardware to different variables
    // The motors and the gyroscope are initialized here
    public void initialize(boolean isAuto) {
        // map the devices to the hardware map

        //Drivetrain motors
        fLeft = opMode.hardwareMap.get(DcMotorEx.class, "fleft");
        bRight = opMode.hardwareMap.get(DcMotorEx.class, "bright");
        fRight = opMode.hardwareMap.get(DcMotorEx.class, "fright");
        bLeft = opMode.hardwareMap.get(DcMotorEx.class, "bleft");


        // The odometry pods are wired so that the odometry pods reference that +Y is forwards and
        // +X is to the right
        odom = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odom.resetPosAndIMU();
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        touch1 = opMode.hardwareMap.get(RevTouchSensor.class, "touchSensor1");
        touch2 = opMode.hardwareMap.get(RevTouchSensor.class, "touchSensor2");


        distanceSensor1 = opMode.hardwareMap.get(AnalogInput.class, "distanceSensor1");
        distanceSensor2 = opMode.hardwareMap.get(AnalogInput.class, "distanceSensor2");

        sonicDistanceSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sonicDistanceSensor");




        // The front left and back right motors are reversed so all wheels go in the same direction
        // When a positive or negative value is used
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        // Resetting the encoders (distance measurement sensors)
        // and then start them again on program start
        // Repeat for all motors
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setTargetPosition(0);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fLeft.setTargetPositionTolerance(10);

        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setTargetPosition(0);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setTargetPositionTolerance(10);

        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setTargetPosition(0);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setTargetPositionTolerance(10);

        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setTargetPosition(0);
        //without odom: bright.setMode(DcMotor.RunMode.tUSING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setTargetPositionTolerance(10);

        opMode.telemetry.addData("PID", fLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

        PIDFCoefficients posiitonCoeff = new PIDFCoefficients(10, 0.049988, 0, 0);

        bLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, posiitonCoeff);
        bRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, posiitonCoeff);
        fLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, posiitonCoeff);
        fRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, posiitonCoeff);

        //Manipulator mechanisms

            //Lift motors

        liftExtender = opMode.hardwareMap.get(DcMotorEx.class, "liftExtender");
        liftExtender.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isAuto) {
            liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftExtender.setTargetPosition(0);// Needs to not reset once teleop begins
            liftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftExtender.setTargetPositionTolerance(20);// Needs to not reset once teleop begins
        }
        liftExtender.setZeroPowerBehavior(BRAKE);



        liftPitch = opMode.hardwareMap.get(DcMotorEx.class, "liftPitch");

            //Initial conditions of the liftPitch MOTOR
            //PIDF Coefficients for the liftPitch MOTOR
            // important liftPitch.setVelocityPIDFCoefficients(1,1,-2.5, 3);
            liftPitch.setDirection(DcMotorSimple.Direction.REVERSE);
            liftPitch.setTargetPosition(0);
            liftPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftPitch.setTargetPositionTolerance(100);


        if (isAuto) {
            liftPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Needs to not reset once teleop begins
        }
            liftPitch.setZeroPowerBehavior(BRAKE);


        //Manipulator Servos

            // Hang on submersible servos





        parkingServo = opMode.hardwareMap.get(Servo.class, "parkingServo");
        //hangR.setDirection(Servo.Direction.REVERSE);
        parkingServo.setPosition(1);

        //Continuous rotation Servo
        intake = opMode.hardwareMap.get(CRServo.class, "intake");

        //Initial conditions of the intake SERVO
        intake.setPower(0); // Off by default
        intake.setDirection(CRServo.Direction.REVERSE);

        //Regular Servos
        pitch = opMode.hardwareMap.get(Servo.class, "pitch");
        pitch.setPosition(0);

//        pitch.setDirection(Servo.)
        { //This was causing problems
            //pitch.setPosition(0);
        }



        // Initialize Gyroscope
        gyroScope = opMode.hardwareMap.get(BHI260IMU.class, "gyroScope");
        // Possibly change this to the following code comment
        RevHubOrientationOnRobot ori = new RevHubOrientationOnRobot(new Orientation(AxesReference
                .INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 90, 0, 0,
                0));

        // Way to set up the gyroscope
        // new REVHubOrientationOnRobot(REVHubOrientationOnRobot.LogoFacingDirection.<direction
        // here>, REVHubOrientationOnRobot.UsbFacingDirection.<direction here>));

        // Setting up the gyroscope settings
        settings = new BHI260IMU.Parameters(ori);
        gyroScope.initialize(settings);
        AngularVelocity angularVelocity = gyroScope.getRobotAngularVelocity(AngleUnit.DEGREES);
        YawPitchRollAngles orientation = gyroScope.getRobotYawPitchRollAngles();
        gyroScope.resetYaw();




        //odom sect
        final int wheelDiam = 48;

        //final int x = ;

        opMode.telemetry.update();

    }

    public LinkedList <Double> pastDist1 = new LinkedList<>();
    final int MAXLIST = 2;

    public double getDistance1(){
        double newDist = ((distanceSensor1.getVoltage()*48.7)-4.9);

        pastDist1.add(newDist);

        if (pastDist1.size()>MAXLIST){
            pastDist1.remove(0);
        }
        double avgDist =0;

        for (int i = 0; i< pastDist1.size(); i++){
            avgDist = avgDist+ pastDist1.get(i);
        }

        avgDist = avgDist/ (pastDist1.size());


        return avgDist;
    }

    LinkedList <Double> pastDist2 = new LinkedList<>();

    public double getDistance2(){
        double newDist = ((distanceSensor2.getVoltage()*48.7)-4.9);

        pastDist2.add(newDist);

        if (pastDist2.size()>MAXLIST){
            pastDist2.remove(0);
        }
        double avgDist =0;

        for (int i = 0; i< pastDist2.size(); i++){
            avgDist = avgDist+ pastDist2.get(i);
        }

        avgDist = avgDist/ pastDist2.size();


        return avgDist;
    }


    public double getDistanceBoth(){

        double avgDist =0;


        avgDist = (getDistance1()+getDistance2())/2;


        return avgDist;
    }

    public double getValidDistance(boolean nearBasket){
        double validDistance = getDistanceBoth();
        if(nearBasket){
            validDistance = getDistance1();
        } else{
            validDistance = getDistanceBoth();
        }
        return validDistance;
    }

    public double getNonErroDist(){
        //double ultraDist =
        return 4.4;
    }


    public double getSonicDistance(){
        double newDist = (sonicDistanceSensor.getDistance(DistanceUnit.MM));

        // If the new value is NaN, don't add it
        if(!Double.isNaN(newDist)) {
            pastDist1.add(newDist);
            if (pastDist1.size()>MAXLIST){
                pastDist1.remove(0);
            }
        }


        double avgDist = 0;

        for (int i = 0; i< pastDist1.size(); i++){
            avgDist = avgDist+ pastDist1.get(i);
        }

        try {
            avgDist = avgDist/ (pastDist1.size());
        } catch (Exception e) {
            opMode.telemetry.addData("CRASH!!",e);
        }


        return avgDist;
    }


    //z represents rotation not z axis movement
    public void setVel(int x, int y, int z){
        x = (int)(x*1.7);
        // Using this RobotInitialize, fright and bleft are reversed
        int fleftVel = -x-y+z;
        int frightVel = x-y-z;
        int bleftVel = -x+y-z;
        int brightVel = x+y+z;

        fLeft.setVelocity(fleftVel);
        fRight.setVelocity(frightVel);
        bLeft.setVelocity(bleftVel);
        bRight.setVelocity(brightVel);
    }

    public void executeMove(int xDist, int yDist, int heading, int maxSpeed){
        // Reset position
        odom.update();
        odom.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        opMode.sleep(500);

        int xVel = Math.round(xDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);
        int yVel = Math.round(yDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Odometry references to x and y Dist need to be made
        while(opMode.opModeIsActive()){
            odom.update();

            if (Math.abs(xDist) - Math.abs(odom.getPosX()) <= 10 && Math.abs(yDist) - Math.abs(odom.getPosY()) <= 10 && Math.abs(heading-gyroScope.getRobotYawPitchRollAngles().getYaw())<2) {
                break;
            }
            double yPercent = odom.getPosY()/yDist;
            double xPercent= odom.getPosX()/xDist;

            int xCorrecter;

            if (yDist == 0) {
                xCorrecter = 0;
            } else {
                xCorrecter = (int) ((yPercent - xPercent) * 1000);
            }

            int xCorrectSign = xVel == 0 ? 1 : (int) Math.signum(xVel);



            int zErr = clamp((int) ((heading-(gyroScope.getRobotYawPitchRollAngles().getYaw())) * 150), 1000);
            opMode.telemetry.addData("zerr", zErr);
            opMode.telemetry.addData("xvel", xVel);
            opMode.telemetry.addData("yvel", yVel);
            opMode.telemetry.addData("xpercent", xPercent);
            opMode.telemetry.addData("ypercent",yPercent);
            opMode.telemetry.addData("gyro", (gyroScope.getRobotYawPitchRollAngles().getYaw()));
            opMode.telemetry.addData("gyro", odom.getPosX());

            opMode.telemetry.addData("xvel corrected", xVel + xCorrecter * (int) Math.signum(xVel));
            opMode.telemetry.update();
            setVel(xVel + xCorrecter * xCorrectSign, yVel, zErr);
        }

        setVel(0, 0, 0);
    }

    public void executeMoveAlt(int xDist, int yDist, int heading, int maxSpeed, boolean turnOnly) {
        executeMoveAlt(xDist,yDist,heading,maxSpeed,turnOnly,Integer.MIN_VALUE,Integer.MIN_VALUE);
    }

    public void executeMoveAlt(int xDist, int yDist, int heading, int maxSpeed, boolean turnOnly, int extenderPos, int delay) {
        executeMoveAlt(xDist, yDist, heading, maxSpeed, turnOnly,extenderPos,delay,1,1, false);
    }

    public void executeMoveAlt(int xDist, int yDist, int heading, int maxSpeed, boolean turnOnly, int extenderPos, int delay, double xMult, double yMult, boolean forceQuit) {
        final int kP = 4;
        final double kI = 0.07;

        // Reset position
        odom.update();
        odom.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        long startTime = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis()-startTime < 500) {}

        int xVel = Math.round(xDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);
        int yVel = Math.round(yDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double xI = 0;
        double yI = 0;

        boolean extenderMoved = false;
        startTime = System.currentTimeMillis();

        //Odometry references to x and y Dist need to be made
        while(opMode.opModeIsActive()){
            odom.update();
            double xerr = Math.abs(xDist) - Math.abs(odom.getPosX());
            double yerr = Math.abs(yDist) - Math.abs(odom.getPosY());
            double zerr = Math.abs(heading-gyroScope.getRobotYawPitchRollAngles().getYaw());

            xI += xerr;
            yI += yerr;

            xI  = clamp(xI, 200 / kI);
            yI  = clamp(yI, 200 / kI);

            if (turnOnly && zerr < 2) {
                break;
            }
            opMode.telemetry.addData("time left", System.currentTimeMillis()-startTime);

            if (System.currentTimeMillis()-startTime>=2250&&forceQuit==true){
                opMode.telemetry.addData("FORCESTOPPED", "HEHE");
                break;
            }

            if (Math.abs(xerr) <= 20 && Math.abs(yerr) <= 20 && zerr <1) {
                break;
            }

            double yPercent = odom.getPosY()/yDist;
            double xPercent= odom.getPosX()/xDist;

            int xCorrecter;

            if (yDist == 0) {
                xCorrecter = 0;
            } else {
                xCorrecter = (int) ((yPercent - xPercent) * 1000);
            }

            int xCorrectSign = xVel == 0 ? 1 : (int) Math.signum(xVel);



            int zErr = clamp((int) ((heading-(gyroScope.getRobotYawPitchRollAngles().getYaw())) * 150), 1000);
            opMode.telemetry.addData("zerr", zErr);
            opMode.telemetry.addData("xerr", xerr);
            opMode.telemetry.addData("yerr", yerr);

            opMode.telemetry.addData("xpercent", xPercent);
            opMode.telemetry.addData("ypercent",yPercent);
            opMode.telemetry.addData("gyro", (gyroScope.getRobotYawPitchRollAngles().getYaw()));

            opMode.telemetry.addData("xvel corrected", xVel + xCorrecter * (int) Math.signum(xVel));


            // The values from which the final velocity will inherit its sign
            double xSignVal = xerr*xVel;
            double ySignVal = yerr*yVel;
            // Calculated from:
            //       +xVel    -xVel
            //       ----------------
            // +xErr | +v   |  -v   |
            // -xErr | -v   |  +v   |
            //       ----------------

            opMode.telemetry.addData("xSignVal",xSignVal);
            opMode.telemetry.addData("ySignVal",ySignVal);

            // Calculate the velocity with PID constants
            int calcXVelocityBeforeClamp = (int) (xerr * kP + xI * kI);

            int calcYVelocityBeforeClamp = (int) (yerr * kP + yI * kI);

            // Clamp to max speed found in xVel and yVel
            int calcXVelocity = clamp(calcXVelocityBeforeClamp, xVel);
            int calcYVelocity = clamp(calcYVelocityBeforeClamp, yVel);

            // Copy the sign of the sign variables
            int finalXVelocity = (int)(Math.copySign(calcXVelocity, xSignVal)*xMult);
            int finalYVelocity = (int)(Math.copySign(calcYVelocity, ySignVal)*yMult);

            opMode.telemetry.addData("finalXVelocity",finalXVelocity);
            opMode.telemetry.addData("finalYVelocity",finalYVelocity);

            setVel(finalXVelocity, finalYVelocity, zErr);

            opMode.telemetry.update();


            // EXTENDER MOVE CODE
            if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
                if(!extenderMoved) {
                    if((System.currentTimeMillis() - startTime) >= delay) {
                        extenderMoved = true;
                        extenderToPos(extenderPos, 0.9, false);
                    }
                }
            }

        }

        setVel(0, 0, 0);

        // Check to see if the extender still hasn't moved. If it hasn't, move it.
        if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
            if(!extenderMoved) {
                while(opMode.opModeIsActive() && (System.currentTimeMillis() - startTime) <= delay) {}
                extenderMoved = true;
                extenderToPos(extenderPos, 0.8, false);
            }
        }
    }

    public int clamp(int val, int maxScalar) {
        return Math.round(Math.copySign(Math.min(Math.abs(val), Math.abs(maxScalar)), val));
    }



    public double clamp(double val, double maxScalar) {
        return Math.copySign(Math.min(Math.abs(val), Math.abs(maxScalar)), val);
    }

    public void executeMoveDistanceSensors(int xDist, int yDist, int heading, int maxSpeed, boolean turnOnly, int extenderPos, int delay, double xMult, double yMult, boolean forceQuit, double distanceSensorDist, boolean touch, int touchDelay){
        executeMoveDistanceSensors(xDist,  yDist,  heading,  maxSpeed,  turnOnly,  extenderPos,  delay,  xMult,  yMult,  forceQuit,  distanceSensorDist,  touch,  touchDelay, 50, 1);
    }
    public void executeMoveDistanceSensors(int xDist, int yDist, int heading, int maxSpeed, boolean turnOnly, int extenderPos, int delay, double xMult, double yMult, boolean forceQuit, double distanceSensorDist, boolean touch, int touchDelay, int zMulti, double distanceChange) {


        final int kP = 4;
        final double kI = 0.07;

        // Reset position
        odom.update();
        odom.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        distanceSensorDist=distanceSensorDist*distanceChange;

        long startTime = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis()-startTime < 500) {}

        // As opposed to referencing yDist, which is ALWAYS unused in this function, we
        // calculate a pseudo yDist with the distance we need to travel as given by the sensor.
        double yDistanceWeNeedToGoAtTheStart =  Math.abs(getSonicDistance())-Math.abs(distanceSensorDist);

        // Extracted variable from the xVel, yVel calculations
        double max = Math.max(Math.abs(xDist), Math.abs(yDistanceWeNeedToGoAtTheStart));

        // The MAX SPEED values as used later in clamps to ensure that the motors do not go crazy fast.
        double xVel = Math.round(xDist/(float) max * maxSpeed);
        double yVel = Math.round(yDistanceWeNeedToGoAtTheStart/(float) max * maxSpeed);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double xI = 0;
        double yI = 0;

        boolean extenderMoved = false;
        startTime = System.currentTimeMillis();
        long startTime2 = System.currentTimeMillis();


        //Odometry references to x and y Dist need to be made
        while(opMode.opModeIsActive()){
            odom.update();
            double xerr = xDist - odom.getPosX();
            double yerr = Math.abs(getSonicDistance())-Math.abs(distanceSensorDist);


            //double yerr = Math.abs(yDist) - Math.abs(odom.getPosY());
            double zerr = heading-gyroScope.getRobotYawPitchRollAngles().getYaw();

            xI += xerr;
            yI += yerr;

            xI  = clamp(xI, 200 / kI);
            yI  = clamp(yI, 200 / kI);

            if (turnOnly && zerr < 2) {
                break;
            }
            opMode.telemetry.addData("time left", System.currentTimeMillis()-startTime);

            if (System.currentTimeMillis()-startTime>=2250&&forceQuit==true){
                break;
            }

            if (Math.abs(xerr) <= 20 &&Math.abs(yerr)<5 && Math.abs(zerr) <=1.2) {
                break;
            }
            double xPercent= odom.getPosX()/xDist;

            int xCorrecter;


            opMode.telemetry.addData("yerr", yerr);
            opMode.telemetry.addData("xerr", xerr);
            opMode.telemetry.addData("zerr", zerr);

            opMode.telemetry.addData("sensor dist mm", getSonicDistance());


            opMode.telemetry.addData("odom dist", odom.getPosX());



//            opMode.telemetry.addData("xpercent", xPercent);
//            opMode.telemetry.addData("ypercent",yPercent);
            opMode.telemetry.addData("gyro dist", (gyroScope.getRobotYawPitchRollAngles().getYaw()));

           // opMode.telemetry.addData("valid distance");




            // The values from which the final velocity will inherit its sign
            double xSignVal = xerr*xVel;
            double ySignVal = yerr*yVel;
            // Calculated from:
            //       +xVel    -xVel
            //       ----------------
            // +xErr | +v   |  -v   |
            // -xErr | -v   |  +v   |
            //       ----------------

            opMode.telemetry.addData("xSignVal",xSignVal);
            opMode.telemetry.addData("ySignVal",ySignVal);


            // Calculate the velocity with PID constants
            int calcXVelocityBeforeClamp = (int) (xerr * kP + xI * kI);

            int calcYVelocityBeforeClamp = (int) (yerr * kP + yI * kI);

            // Clamp to max speed found in xVel and yVel
            double calcXVelocity = clamp(calcXVelocityBeforeClamp, xVel);
            double calcYVelocity = clamp(calcYVelocityBeforeClamp, yVel);

            // Copy the sign of the sign variables
            int finalXVelocity = (int)(Math.copySign(calcXVelocity, xerr)*xMult);
            int finalYVelocity = (int)(Math.copySign(calcYVelocity, yerr)*yMult);
            int finalZVelocity =  (int)(zerr*zMulti);

            opMode.telemetry.addData("yerr kp", yerr * kP);
            opMode.telemetry.addData("yerr ki",  yI * kI);
            opMode.telemetry.addData("xvel pre clamp",calcXVelocityBeforeClamp);

            opMode.telemetry.addData("yvel pre clamp",calcXVelocityBeforeClamp);

            opMode.telemetry.addData("xvel  clamp",calcXVelocity);
            opMode.telemetry.addData("yvel  clamp",calcYVelocity);

            opMode.telemetry.addData("yI",yI);



            opMode.telemetry.addData("finalXVelocity",finalXVelocity);
            opMode.telemetry.addData("finalYVelocity",finalYVelocity);
            opMode.telemetry.addData("finalZVelocity",finalZVelocity);
            opMode.telemetry.addData("xVel",xVel);
            opMode.telemetry.addData("yVel",yVel);



            //opMode.telemetry.addData("zVel",zVel);



            setVel(finalXVelocity, finalYVelocity, finalZVelocity);

            opMode.telemetry.update();


            // EXTENDER MOVE CODE
            if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
                if(!extenderMoved) {
                    if((System.currentTimeMillis() - startTime) >= delay) {
                        extenderMoved = true;
                        extenderToPos(extenderPos, 0.9, false);
                    }
                }
            }

        }

        setVel(0, 0, 0);

        // Check to see if the extender still hasn't moved. If it hasn't, move it.
        if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
            if(!extenderMoved) {
                while(opMode.opModeIsActive() && (System.currentTimeMillis() - startTime) <= delay) {}
                extenderMoved = true;
                extenderToPos(extenderPos, 0.8, false);
            }
        }
    }







    public void executeMoveTouchAndDistance(int xDist, int yDist, int heading, int maxSpeed, boolean turnOnly, int extenderPos, int delay, double xMult, double yMult, boolean forceQuit, double distanceSensorDist, boolean touch, int touchDelay, double distanceChange) {


        final int kP = 4;
        final double kI = 0.07;

        // Reset position
        odom.update();
        odom.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        distanceSensorDist=distanceSensorDist*distanceChange;

        long startTime = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis()-startTime < 500) {}

        // As opposed to referencing yDist, which is ALWAYS unused in this function, we
        // calculate a pseudo yDist with the distance we need to travel as given by the sensor.
        double yDistanceWeNeedToGoAtTheStart =  Math.abs(getSonicDistance())-Math.abs(distanceSensorDist);

        // Extracted variable from the xVel, yVel calculations
        double max = Math.max(Math.abs(xDist), Math.abs(yDistanceWeNeedToGoAtTheStart));

        // The MAX SPEED values as used later in clamps to ensure that the motors do not go crazy fast.
        double xVel = Math.round(xDist/(float) max * maxSpeed);
        double yVel = Math.round(yDistanceWeNeedToGoAtTheStart/(float) max * maxSpeed);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double xI = 0;
        double yI = 0;

        boolean extenderMoved = false;
        startTime = System.currentTimeMillis();
        long startTime2 = System.currentTimeMillis();


        //Odometry references to x and y Dist need to be made
        while(opMode.opModeIsActive()){
            odom.update();
            double xerr = xDist - odom.getPosX();
            double yerr = Math.abs(getSonicDistance())-Math.abs(distanceSensorDist);


            //double yerr = Math.abs(yDist) - Math.abs(odom.getPosY());
            double zerr = heading-gyroScope.getRobotYawPitchRollAngles().getYaw();

            xI += xerr;
            yI += yerr;

            xI  = clamp(xI, 200 / kI);
            yI  = clamp(yI, 200 / kI);

            if (turnOnly && zerr < 2) {
                break;
            }
            opMode.telemetry.addData("time left", System.currentTimeMillis()-startTime);

            if (System.currentTimeMillis()-startTime>=2250&&forceQuit==true){
                break;
            }

            if (Math.abs(xerr) <= 20 &&Math.abs(yerr)<5 && Math.abs(zerr) <=1.2) {
                break;
            }
            double xPercent= odom.getPosX()/xDist;

            int xCorrecter;


            opMode.telemetry.addData("yerr", yerr);
            opMode.telemetry.addData("xerr", xerr);
            opMode.telemetry.addData("zerr", zerr);

            opMode.telemetry.addData("sensor dist mm", getSonicDistance());
            opMode.telemetry.addData("odom dist", odom.getPosX());



//            opMode.telemetry.addData("xpercent", xPercent);
//            opMode.telemetry.addData("ypercent",yPercent);
            opMode.telemetry.addData("gyro dist", (gyroScope.getRobotYawPitchRollAngles().getYaw()));

            // opMode.telemetry.addData("valid distance");




            // The values from which the final velocity will inherit its sign
            double xSignVal = xerr*xVel;
            double ySignVal = yerr*yVel;
            // Calculated from:
            //       +xVel    -xVel
            //       ----------------
            // +xErr | +v   |  -v   |
            // -xErr | -v   |  +v   |
            //       ----------------

            opMode.telemetry.addData("xSignVal",xSignVal);
            opMode.telemetry.addData("ySignVal",ySignVal);


            // Calculate the velocity with PID constants
            int calcXVelocityBeforeClamp = (int) (xerr * kP + xI * kI);

            int calcYVelocityBeforeClamp = (int) (yerr * kP + yI * kI);

            // Clamp to max speed found in xVel and yVel
            double calcXVelocity = clamp(calcXVelocityBeforeClamp, xVel);
            double calcYVelocity = clamp(calcYVelocityBeforeClamp, yVel);

            // Copy the sign of the sign variables
            int finalXVelocity = (int)(Math.copySign(calcXVelocity, xerr)*xMult);
            int finalYVelocity = (int)(Math.copySign(calcYVelocity, yerr)*yMult);
            int finalZVelocity =  (int)(zerr*90);

            opMode.telemetry.addData("xvel pre clamp",calcXVelocityBeforeClamp);

            opMode.telemetry.addData("yvel pre clamp",calcXVelocityBeforeClamp);

            opMode.telemetry.addData("xvel  clamp",calcXVelocity);
            opMode.telemetry.addData("yvel  clamp",calcYVelocity);

            opMode.telemetry.addData("yI",yI);



            opMode.telemetry.addData("finalXVelocity",finalXVelocity);
            opMode.telemetry.addData("finalYVelocity",finalYVelocity);
            opMode.telemetry.addData("finalZVelocity",finalZVelocity);
            opMode.telemetry.addData("xVel",xVel);
            opMode.telemetry.addData("yVel",yVel);

            opMode.telemetry.addData("yerr kp", yerr * kP);
            opMode.telemetry.addData("yerr ki",  yI * kI);





            //opMode.telemetry.addData("zVel",zVel);
         if(touch){
             opMode.telemetry.addData("touching you", "gg");

             while(Math.abs(xerr)<20){
                 opMode.telemetry.addData("stuckkk", "gg");

                 finalXVelocity = (int) Math.copySign(700, xDist);
                 if ((!touch1.isPressed())){//need to turn left
//                     bLeft.setVelocity(1050);
//                     bRight.setVelocity(-1050);
                     bLeft.setVelocity(850);
                     bRight.setVelocity(-850);
                     opMode.telemetry.addData("1", "gg");
                 }

                 if(!touch2.isPressed()){//need to turn right
//                     fLeft.setVelocity(1050);
//                     fRight.setVelocity(-1050);
                     fLeft.setVelocity(850);
                     fRight.setVelocity(-850);
                     opMode.telemetry.addData("2", "gg");
                 }



                 if (touch1.isPressed() && touch2.isPressed()) {
                     opMode.telemetry.addData("stuck here", "gg");
                     startTime2 = System.currentTimeMillis();
                     while ((System.currentTimeMillis() - startTime2) <= touchDelay) {
                         opMode.telemetry.addData("waiting for delay: time left", (System.currentTimeMillis() - startTime2));
                     }
                     setVel(0, 0, 0);


                     opMode.telemetry.addData("shis pressed", "fhhf");


                     return;
                 }
             }
         }


            setVel(finalXVelocity, finalYVelocity, finalZVelocity);

            opMode.telemetry.update();


            // EXTENDER MOVE CODE
            if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
                if(!extenderMoved) {
                    if((System.currentTimeMillis() - startTime) >= delay) {
                        extenderMoved = true;
                        extenderToPos(extenderPos, 0.9, false);
                    }
                }
            }

        }

        setVel(0, 0, 0);

        // Check to see if the extender still hasn't moved. If it hasn't, move it.
        if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
            if(!extenderMoved) {
                while(opMode.opModeIsActive() && (System.currentTimeMillis() - startTime) <= delay) {}
                extenderMoved = true;
                extenderToPos(extenderPos, 0.8, false);
            }
        }
    }


//    public void executeMoveDistanceSensorsBreak(int xDist, int yDist, int heading, int maxSpeed, boolean turnOnly, int extenderPos, int delay, double xMult, double yMult, boolean forceQuit, double distanceSensorDist) {
//        final int kP = 4;
//        final double kI = 0.07;
//
//        // Reset position
//        odom.update();
//        odom.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));
//
//        long startTime = System.currentTimeMillis();
//        while (opMode.opModeIsActive() && System.currentTimeMillis()-startTime < 500) {}
//
//        int xVel = Math.round(xDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);
//        int yVel = Math.round(yDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);
//
//        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        double xI = 0;
//        double yI = 0;
//
//        boolean extenderMoved = false;
//        startTime = System.currentTimeMillis();
//
//        //Odometry references to x and y Dist need to be made
//        while(opMode.opModeIsActive()){
//            odom.update();
//            double xerr = Math.abs(xDist) - Math.abs(odom.getPosX());
//            double yerr = Math.abs(yDist) - Math.abs(odom.getPosY());
//            double sErr = Math.abs(distanceSensorDist) - Math.abs(getDistance1());
//
//            //double yerr = Math.abs(yDist) - Math.abs(odom.getPosY());
//            double zerr = Math.abs(heading-gyroScope.getRobotYawPitchRollAngles().getYaw());
//
//            xI += xerr;
//            yI += yerr;
//
//            xI  = clamp(xI, 200 / kI);
//            yI  = clamp(yI, 200 / kI);
//
//            if (turnOnly && zerr < 2) {
//                break;
//            }
//            opMode.telemetry.addData("time left", System.currentTimeMillis()-startTime);
//
//            if (System.currentTimeMillis()-startTime>=2250&&forceQuit==true){
//                break;
//            }
//
//            if (Math.abs(xerr) <= 20 && Math.abs(yerr) <= 20 && zerr <1&&Math.abs(sErr)<2) {
//                break;
//            }  else if (Math.abs(xerr) <= 20 && Math.abs(yerr) <= 20 && zerr <1&&Math.abs(sErr)>=2) {
//                yerr=(sErr)/((32*Math.PI)/(2000*25.4));
//            }
//
//            double yPercent = odom.getPosY()/yDist;
//            double xPercent= odom.getPosX()/xDist;
//
//            int xCorrecter;
//
//            if (yDist == 0) {
//                xCorrecter = 0;
//            } else {
//                xCorrecter = (int) ((yPercent - xPercent) * 1000);
//            }
//
//            int xCorrectSign = xVel == 0 ? 1 : (int) Math.signum(xVel);
//
//
//
//            int zErr = clamp((int) ((heading-(gyroScope.getRobotYawPitchRollAngles().getYaw())) * 150), 1000);
//            opMode.telemetry.addData("zerr", zErr);
//            opMode.telemetry.addData("xerr", xerr);
//            opMode.telemetry.addData("yerr", yerr);
//            opMode.telemetry.addData("serr", sErr);
//
//
//            opMode.telemetry.addData("xpercent", xPercent);
//            opMode.telemetry.addData("ypercent",yPercent);
//            opMode.telemetry.addData("gyro", (gyroScope.getRobotYawPitchRollAngles().getYaw()));
//
//            opMode.telemetry.addData("xvel corrected", xVel + xCorrecter * (int) Math.signum(xVel));
//            opMode.telemetry.addData("distance1", getDistance1());
//            opMode.telemetry.addData("distance2", getDistance2());
//            opMode.telemetry.addData("distanceBoth", getDistanceBoth());
//
//
//
//            // The values from which the final velocity will inherit its sign
//            double xSignVal = xerr*xVel;
//            double ySignVal = yerr*yVel;
//            // Calculated from:
//            //       +xVel    -xVel
//            //       ----------------
//            // +xErr | +v   |  -v   |
//            // -xErr | -v   |  +v   |
//            //       ----------------
//
//            opMode.telemetry.addData("xSignVal",xSignVal);
//            opMode.telemetry.addData("ySignVal",ySignVal);
//
//            // Calculate the velocity with PID constants
//            int calcXVelocityBeforeClamp = (int) (xerr * kP + xI * kI);
//
//            int calcYVelocityBeforeClamp = (int) (yerr * kP + yI * kI);
//
//            // Clamp to max speed found in xVel and yVel
//            int calcXVelocity = clamp(calcXVelocityBeforeClamp, xVel);
//            int calcYVelocity = clamp(calcYVelocityBeforeClamp, yVel);
//
//            // Copy the sign of the sign variables
//            int finalXVelocity = (int)(Math.copySign(calcXVelocity, xSignVal)*xMult);
//            int finalYVelocity = (int)(Math.copySign(calcYVelocity, ySignVal)*yMult);
//
//            opMode.telemetry.addData("finalXVelocity",finalXVelocity);
//            opMode.telemetry.addData("finalYVelocity",finalYVelocity);
//            opMode.telemetry.addData("finalZVelocity",zErr);
//
//            setVel(finalXVelocity, finalYVelocity, zErr);
//
//            opMode.telemetry.update();
//
//
//            // EXTENDER MOVE CODE
//            if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
//                if(!extenderMoved) {
//                    if((System.currentTimeMillis() - startTime) >= delay) {
//                        extenderMoved = true;
//                        extenderToPos(extenderPos, 0.9, false);
//                    }
//                }
//            }
//
//        }
//
//        setVel(0, 0, 0);
//
//        // Check to see if the extender still hasn't moved. If it hasn't, move it.
//        if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
//            if(!extenderMoved) {
//                while(opMode.opModeIsActive() && (System.currentTimeMillis() - startTime) <= delay) {}
//                extenderMoved = true;
//                extenderToPos(extenderPos, 0.8, false);
//            }
//        }
//    }






    public void executeMoveTouchSensor(int xDist, int yDist, int heading, int maxSpeed, boolean turnOnly, int extenderPos, int delay, double xMult, double yMult, int touchDelay) {
        final int kP = 4;
        final double kI = 0.07;

        // Reset position
        odom.update();
        odom.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        long startTime = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis()-startTime < 500) {}

        int xVel = Math.round(xDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);
        int yVel = Math.round(yDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double xI = 0;
        double yI = 0;

        boolean extenderMoved = false;
        startTime = System.currentTimeMillis();
        long startTime2 = System.currentTimeMillis();



        //Odometry references to x and y Dist need to be made
        while(opMode.opModeIsActive()) {
            odom.update();
            double xerr = Math.abs(xDist) - Math.abs(odom.getPosX());
            double yerr = Math.abs(yDist) - Math.abs(odom.getPosY());
            double zerror = heading - gyroScope.getRobotYawPitchRollAngles().getYaw();

            xI += xerr;
            yI += yerr;

            xI = clamp(xI, 200 / kI);
            yI = clamp(yI, 200 / kI);







            int zErr = clamp((int) ((heading - (gyroScope.getRobotYawPitchRollAngles().getYaw())) * 150), 1000);
            opMode.telemetry.addData("zerr", zerror);
            opMode.telemetry.addData("xerr", xerr);
            opMode.telemetry.addData("yerr", yerr);


            opMode.telemetry.addData("gyro", (gyroScope.getRobotYawPitchRollAngles().getYaw()));


            // The values from which the final velocity will inherit its sign
            double xSignVal = xerr * xVel;
            double ySignVal = yerr * yVel;
            // Calculated from:
            //       +xVel    -xVel
            //       ----------------
            // +xErr | +v   |  -v   |
            // -xErr | -v   |  +v   |
            //       ----------------

            opMode.telemetry.addData("xSignVal", xSignVal);
            opMode.telemetry.addData("ySignVal", ySignVal);

            // Calculate the velocity with PID constants
            int calcXVelocityBeforeClamp = (int) (xerr * kP + xI * kI);

            int calcYVelocityBeforeClamp = (int) (yerr * kP + yI * kI);

            // Clamp to max speed found in xVel and yVel
            int calcXVelocity = clamp(calcXVelocityBeforeClamp, xVel);
            int calcYVelocity = clamp(calcYVelocityBeforeClamp, yVel);

            // Copy the sign of the sign variables
            int finalXVelocity = (int) (Math.copySign(calcXVelocity, xSignVal) * xMult);
            int finalYVelocity = (int) (Math.copySign(calcYVelocity, ySignVal) * yMult);


            while(xerr<50){
                opMode.telemetry.addData("stuckkk", "gg");

                finalXVelocity = (int) Math.copySign(700, xDist);
                if ((!touch1.isPressed())){//need to turn left
                    bLeft.setVelocity(1250);
                    bRight.setVelocity(-1250);
                }

                if(!touch2.isPressed()){//need to turn right
                    fLeft.setVelocity(1250);
                    fRight.setVelocity(-1250);
                }



                if (touch1.isPressed() && touch2.isPressed()) {
                    opMode.telemetry.addData("stuck here", "gg");
                    startTime2 = System.currentTimeMillis();
                    while ((System.currentTimeMillis() - startTime2) <= touchDelay) {
                        opMode.telemetry.addData("waiting for delay: time left", (System.currentTimeMillis() - startTime2));
                    }
                    setVel(0, 0, 0);


                    opMode.telemetry.addData("shis pressed", "fhhf");


                    return;
                }
            }
            opMode.telemetry.addData("stuck somwehere else", "gg");






            opMode.telemetry.addData("finalXVelocity", finalXVelocity);
            opMode.telemetry.addData("finalYVelocity", finalYVelocity);

            setVel(finalXVelocity, finalYVelocity, zErr);

            opMode.telemetry.update();


            // EXTENDER MOVE CODE
            if (extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
                opMode.telemetry.addData("worst place to be stuck", "hhee");

                if (!extenderMoved) {
                    if ((System.currentTimeMillis() - startTime) >= delay) {
                        extenderMoved = true;
                        extenderToPos(extenderPos, 0.9, false);
                    }
                }
            }

        }

        setVel(0, 0, 0);

        // Check to see if the extender still hasn't moved. If it hasn't, move it.
        if(extenderPos != Integer.MIN_VALUE && delay != Integer.MIN_VALUE) {
            if(!extenderMoved) {
                while(opMode.opModeIsActive() && (System.currentTimeMillis() - startTime) <= delay) {}
                extenderMoved = true;
                extenderToPos(extenderPos, 0.8, false);
            }
        }
    }







    public void executeMoveLastStrafe(int xDist, int yDist, int heading, int maxSpeed) {
        final int kP = 4;
        final double kI = 0.07;

        // Reset position
        odom.update();
        odom.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        long startTime = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis()-startTime < 500) {}

        int xVel = Math.round(xDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);
        int yVel = Math.round(yDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double xI = 0;
        double yI = 0;

        startTime = System.currentTimeMillis();

        //Odometry references to x and y Dist need to be made
        while(opMode.opModeIsActive()){
            odom.update();
            double xerr = Math.abs(xDist) - Math.abs(odom.getPosX());
            double yerr = Math.abs(yDist) - Math.abs(odom.getPosY());
            double zerr = Math.abs(heading-gyroScope.getRobotYawPitchRollAngles().getYaw());

            xI += xerr;
            yI += yerr;

            xI  = clamp(xI, 200 / kI);
            yI  = clamp(yI, 200 / kI);


            if (xerr <= 20 && zerr <3) {
                break;
            }

            double yPercent = odom.getPosY()/yDist;
            double xPercent= odom.getPosX()/xDist;

            int xCorrecter;

            if (yDist == 0) {
                xCorrecter = 0;
            } else {
                xCorrecter = (int) ((yPercent - xPercent) * 1000);
            }

            int xCorrectSign = xVel == 0 ? 1 : (int) Math.signum(xVel);



            int zErr = clamp((int) ((heading-(gyroScope.getRobotYawPitchRollAngles().getYaw())) * 150), 1000);
            opMode.telemetry.addData("zerr", zErr);
            opMode.telemetry.addData("xerr", xerr);
            opMode.telemetry.addData("yerr", yerr);

            opMode.telemetry.addData("xpercent", xPercent);
            opMode.telemetry.addData("ypercent",yPercent);
            opMode.telemetry.addData("gyro", (gyroScope.getRobotYawPitchRollAngles().getYaw()));

            opMode.telemetry.addData("xvel corrected", xVel + xCorrecter * (int) Math.signum(xVel));


            // The values from which the final velocity will inherit its sign from
            double xSignVal = xerr*xVel;
            double ySignVal = yerr*yVel;
            // Calculated from:
            //       +xVel    -xVel
            //       ----------------
            // +xErr | +v   |  -v   |
            // -xErr | -v   |  +v   |
            //       ----------------

            opMode.telemetry.addData("xSignVal",xSignVal);
            opMode.telemetry.addData("ySignVal",ySignVal);

            // Calculate the velocity with PID constants
            int calcXVelocityBeforeClamp = (int) ((xerr * kP + xI * kI)*2);

            int calcYVelocityBeforeClamp = (int) ((yerr * kP + yI * kI)*2);

            // Clamp to max speed found in xVel and yVel
            int calcXVelocity = clamp(calcXVelocityBeforeClamp, xVel);
            int calcYVelocity = clamp(calcYVelocityBeforeClamp, yVel);

            // Copy the sign of the sign variables
            int finalXVelocity = (int)(Math.copySign(calcXVelocity, xSignVal));
            int finalYVelocity = (int)(Math.copySign(calcYVelocity, ySignVal));

            opMode.telemetry.addData("finalXVelocity",finalXVelocity);
            opMode.telemetry.addData("finalYVelocity",finalYVelocity);

            setVel(finalXVelocity, finalYVelocity, zErr);

            opMode.telemetry.update();


            // EXTENDER MOVE CODE


        }

        setVel(0, 0, 0);

        // Check to see if the extender still hasn't moved. If it hasn't, move it.
    }

    public void executeMoveLastStraight(int xDist, int yDist, int heading, int maxSpeed) {
        final int kP = 4;
        final double kI = 0.07;

        // Reset position
        odom.update();
        odom.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        long startTime = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis()-startTime < 500) {}

        int xVel = Math.round(xDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);
        int yVel = Math.round(yDist/(float) (Math.max(Math.abs(xDist), Math.abs(yDist))) * maxSpeed);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double xI = 0;
        double yI = 0;

        startTime = System.currentTimeMillis();

        //Odometry references to x and y Dist need to be made
        while(opMode.opModeIsActive()){
            odom.update();
            double xerr = Math.abs(xDist) - Math.abs(odom.getPosX());
            double yerr = Math.abs(yDist) - Math.abs(odom.getPosY());
            double zerr = Math.abs(heading-gyroScope.getRobotYawPitchRollAngles().getYaw());

            xI += xerr;
            yI += yerr;

            xI  = clamp(xI, 200 / kI);
            yI  = clamp(yI, 200 / kI);


            if (yerr <= 20 && zerr <3) {
                break;
            }

            double yPercent = odom.getPosY()/yDist;
            double xPercent= odom.getPosX()/xDist;

            int xCorrecter;

            if (yDist == 0) {
                xCorrecter = 0;
            } else {
                xCorrecter = (int) ((yPercent - xPercent) * 1000);
            }

            int xCorrectSign = xVel == 0 ? 1 : (int) Math.signum(xVel);



            int zErr = clamp((int) ((heading-(gyroScope.getRobotYawPitchRollAngles().getYaw())) * 150), 1000);
            opMode.telemetry.addData("zerr", zErr);
            opMode.telemetry.addData("xerr", xerr);
            opMode.telemetry.addData("yerr", yerr);

            opMode.telemetry.addData("xpercent", xPercent);
            opMode.telemetry.addData("ypercent",yPercent);
            opMode.telemetry.addData("gyro", (gyroScope.getRobotYawPitchRollAngles().getYaw()));

            opMode.telemetry.addData("xvel corrected", xVel + xCorrecter * (int) Math.signum(xVel));


            // The values from which the final velocity will inherit its sign from
            double xSignVal = xerr*xVel;
            double ySignVal = yerr*yVel;
            // Calculated from:
            //       +xVel    -xVel
            //       ----------------
            // +xErr | +v   |  -v   |
            // -xErr | -v   |  +v   |
            //       ----------------

            opMode.telemetry.addData("xSignVal",xSignVal);
            opMode.telemetry.addData("ySignVal",ySignVal);

            // Calculate the velocity with PID constants
            int calcXVelocityBeforeClamp = (int) ((xerr * kP + xI * kI)*2);

            int calcYVelocityBeforeClamp = (int) ((yerr * kP + yI * kI)*2);

            // Clamp to max speed found in xVel and yVel
            int calcXVelocity = clamp(calcXVelocityBeforeClamp, xVel);
            int calcYVelocity = clamp(calcYVelocityBeforeClamp, yVel);

            // Copy the sign of the sign variables
            int finalXVelocity = (int)(Math.copySign(calcXVelocity, xSignVal));
            int finalYVelocity = (int)(Math.copySign(calcYVelocity, ySignVal));

            opMode.telemetry.addData("finalXVelocity",finalXVelocity);
            opMode.telemetry.addData("finalYVelocity",finalYVelocity);

            setVel(finalXVelocity, finalYVelocity, zErr);

            opMode.telemetry.update();


            // EXTENDER MOVE CODE


        }

        setVel(0, 0, 0);

        // Check to see if the extender still hasn't moved. If it hasn't, move it.
    }





    public void moveLiftPitch(int position, double power, boolean block){
        liftPitch.setTargetPosition(position);
        liftPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftPitch.setPower(power);
        if(block) {
            while(Math.abs(liftPitch.getTargetPosition() - liftPitch.getCurrentPosition()) >= liftPitch.getTargetPositionTolerance());
        }
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

    double inchesToEncoderTicks;

    // This function makes the robot travel a relativeDistance specified by the parameter relativeDistance
    // This parameter is measured in encoder ticks; the other parameter, velocity, is
    // a decimal value that determines how fast the robot will go
    public void goStraight (int distance, double velocity) {
        // Travel a certain relativeDistance based on the absolute value
        // of the average robot encoder reading
        // The relativeDistance is current position plus or minus the value that is being moved
        // RELATIVE DISTANCE MEASUREMENT IN USE

        //Is supposed to be in inches
        //inchesToEncoderTicks = (distance*5267.65)/(12.56637061*13.7);
        //opMode.telemetry.addData("Receives a value in IN and converts to encoder ticks", inchesToEncoderTicks);

        int relativeDistance = distance + getPosStrafe();
        // Go forwards or backwards
        // It only moves if the distance to the final location is greater than or equal to 10 encoder
        // ticks
        while (opMode.opModeIsActive() && Math.abs(getPosStrafe() - relativeDistance) >= 10) {

            //if the current position is before final position
            if (getPosStrafe() < relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                setDrivetrainMotorVelocity(Math.abs(velocity));

                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
                //if current position is less than needed
            } else if (getPosStrafe() > relativeDistance) {
                // Backwards (- negative relativeDistance value)
                setDrivetrainMotorVelocity(-Math.abs(velocity));
                opMode.telemetry.addData("Encoder straight", getPosStrafe());
                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        stopMechanisms();
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
        while (opMode.opModeIsActive() && getPosStrafe() - relativeDistance <= 14) {
            //if
            opMode.telemetry.addData("velocity", fLeft.getVelocity());

            if (getPosStrafe() <= relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                //setMotorVelocity(Math.abs(velocity));
                fLeft.setVelocity(velocity);
                fRight.setVelocity(-velocity);
                bLeft.setVelocity(velocity);
                bRight.setVelocity(-velocity);

                opMode.telemetry.addData("position", getPosStrafe());
                opMode.telemetry.addData("relative distance", relativeDistance);
                opMode.telemetry.addData("old pos", getAverageEncoderValue());
                opMode.telemetry.addData("must be greater than 14", Math.abs(getPosStrafe() - relativeDistance));
                opMode.telemetry.update();
            }
        }
        setDrivetrainMotorVelocity(0);
    }
    public void intake(){
        intake.setPower(1);
    }
    // Parameter of time that it is run in milliseconds
    public void extake(double time){
        double intakeRunTime = time;
        long x = System.currentTimeMillis();

        while (System.currentTimeMillis()-x <time && opMode.opModeIsActive()) { //Check this
            intake.setPower(-1);

            // opMode.telemetry.addData(System.currentTimeMillis());
//            if (intakeRunTime == 0){
//                break;
//            }
//            intakeRunTime = intakeRunTime + 1000;
        }
        intake.setPower(0);
    }
    public void intake(double time){
        double intakeRunTime = time;
        long x = System.currentTimeMillis();

        while (System.currentTimeMillis()-x <time && opMode.opModeIsActive()) { //Check this
            intake.setPower(1);

            // opMode.telemetry.addData(System.currentTimeMillis());
//            if (intakeRunTime == 0){
//                break;
//            }
//            intakeRunTime = intakeRunTime + 1000;
        }
        intake.setPower(0);
    }

    //needs to move counterclockwise to move up
    public void liftPitch(int position, double velocity){
        liftPitch.setTargetPosition(position);
        liftPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(liftPitch.getCurrentPosition()-position)>10 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("needs to be >10", Math.abs(liftPitch.getCurrentPosition()-position));
            opMode.telemetry.addData("position", liftPitch.getCurrentPosition());
            opMode.telemetry.update();
            if (liftPitch.getCurrentPosition()<position) {
                liftPitch.setPower(velocity);

            } else if (liftPitch.getCurrentPosition()>= position){
                liftPitch.setPower(-velocity);
            }
        }
    }
    public void extenderToPos(int targetPos, double power, boolean block){
        liftExtender.setTargetPosition(targetPos);
        liftExtender.setPower(power);
        if (block) {
            while(opMode.opModeIsActive()&&Math.abs(liftExtender.getTargetPosition() - liftExtender.getCurrentPosition()) >= liftExtender.getTargetPositionTolerance()){
                opMode.telemetry.addData("extender pos", liftExtender.getCurrentPosition());
                opMode.telemetry.update();
            }
        }


    }
    // Extends the lift outwards and retracts it inwards
//    public void liftExtender(int position, double velocity){
//        liftExtender.setTargetPosition(position);
//        liftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (Math.abs(liftExtender.getCurrentPosition()-position)>10 && opMode.opModeIsActive()) {
//            opMode.telemetry.addData("needs to be >10", Math.abs(liftExtender.getCurrentPosition()-position));
//            opMode.telemetry.addData("position", liftExtender.getCurrentPosition());
//            opMode.telemetry.update();
//            if (liftExtender.getCurrentPosition()<position) {
//                liftExtender.setPower(velocity);
//
//            } else if (liftExtender.getCurrentPosition()>=position){
//                liftExtender.setPower(-velocity);
//            }
//        }
//    }



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
                Double[] motorVelocity = {bLeft.getVelocity(), fLeft.getVelocity(),
                        bRight.getVelocity(), fRight.getVelocity()};
                fLeft.setVelocity(-velocity);
                fRight.setVelocity(velocity);
                bLeft.setVelocity(-velocity);
                bRight.setVelocity(velocity);

                opMode.telemetry.addData("must be greated than 10", Math.abs(getPosStrafe() - (distance + getPosStrafe())));
                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        stopMechanisms();
    }

    //This makes the robot turn to the specified position from it's starting position in degrees
    public void turnFunction(int degrees) {
        // When turning left, counterclockwise is a positive gyro value
        // When turning right, clockwise is a negative gyro value
        // ABSOLUTE POSITIONING IN USE (will go to exact values)

        while (opMode.opModeIsActive() && Math.abs(degrees - getAngle()) >= 0.2) {
            opMode.telemetry.addData("Encoder turn:", gyroScope.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            opMode.telemetry.addData("Gyroscope", getAngle());
            opMode.telemetry.update();

            if (degrees > getAngle()) {
                // Turning left (positive gyro value)
                fLeft.setVelocity(-300);
                bLeft.setVelocity(300);
                fRight.setVelocity(300);
                bRight.setVelocity(-300);
            }
            else if (degrees < getAngle()) {
                // Turning right (negative gyro value)
                fLeft.setVelocity(300);
                bLeft.setVelocity(-300);
                fRight.setVelocity(-300);
                bRight.setVelocity(300);
            }
        }
        stopMechanisms();
    }

    public void movePitch(double position, double velocity) {
        //pitch.setPosition(position);
    }

    //gets average magnitudes because motors are going in different directions
    public int getPosStrafe() {
        return ((Math.abs(fRight.getCurrentPosition())+Math.abs(bRight.getCurrentPosition())+Math.abs(fLeft.getCurrentPosition())+ Math.abs(bLeft.getCurrentPosition()))/4);
    }

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
    public void setDrivetrainMotorVelocity(double velocity) {
        // Drivetrain motors
        fLeft.setVelocity(velocity);
        fRight.setVelocity(velocity);
        bLeft.setVelocity(-velocity);
        bRight.setVelocity(-velocity);
    }

    public void setMechanismVelocity(double velocity){
        // Drivetrain motors
        fLeft.setVelocity(velocity);
        fRight.setVelocity(velocity);
        bLeft.setVelocity(-velocity);
        bRight.setVelocity(-velocity);

        // Lift motors
        //liftExtender.setVelocity(velocity);
        liftPitch.setVelocity(velocity);
    }

    // Stops the motors by setting the velocity to 0
    public void stopMechanisms() {
        setMechanismVelocity(0);
    }
}


