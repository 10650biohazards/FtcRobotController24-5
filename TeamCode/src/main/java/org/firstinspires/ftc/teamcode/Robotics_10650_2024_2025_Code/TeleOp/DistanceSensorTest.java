package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize_RunToPos;

@TeleOp(name = "Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {

    // Run the initialize function
    RobotInitialize_RunToPos robot;

    int liftExtenderPosition = 0;
    double maxLifEtxtension = 0;
    int speed;

    Boolean circleOn =false;
    double groundCorrecter = 0;


    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize_RunToPos(this, false);

        // initialization of the control of the robot when start is pressed
        waitForStart();


        // loop while the program is running
        // waits for controller input then runs the associated code
        while (opModeIsActive()) {
            // controller inputs that is inputted by the drive team
            controllerInput();
        }
    }

    public void controllerInput() {
        //robot.intake.setPower(0);
        robot.pitch.setPosition(0.1856);
        telemetry.addData("raw voltage", robot.distanceSensor1.getVoltage());
        telemetry.addData("voltage to distance 15", (robot.distanceSensor1.getVoltage()*32.5)-2.6);
        telemetry.addData("voltage to distance 27", (robot.distanceSensor1.getVoltage()*78.1)-10.2);

        telemetry.addData("voltage to distance 20", (robot.distanceSensor1.getVoltage()*48.7)-4.9);
        telemetry.addData("avg dist", robot.getDistance1());
        telemetry.addData("size", robot.pastDist1.size());
        telemetry.addData("olest value", robot.pastDist1.get(0));




        telemetry.update();
    }
}