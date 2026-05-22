package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


@Configurable
@TeleOp
public class PeregrinesTeleOp extends OpMode {
    //GoBildaPinpointDriver pinpoint;
    TelemetryManager telemetryManager;

    boolean isOpen = false;

    public static double flywheelVelocity = 2900;
    public static double flywheelF = 0.35;
    public static double flywheelP = 70;


    // Unit conversion constants
    final double TICKS_PER_ROTATION = 28;
    final double TPS_PER_RPM = TICKS_PER_ROTATION / 60;

    //GoBildaPinpointDriver pinpoint;



    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, intake, rhinoL, rhinoR;
    Servo eat;
    // TODO: Uncomment the following line if you are using servos

    // The following code will run as soon as "INIT" is pressed on the Driver Station




    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set up drive motors
        // The names for each motor are taken from the driveConstants in the Constants file
        // TODO: Update "Constants" with the names of your drive motors from the driver station configuration file
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get( Constants.driveConstants.leftFrontMotorName );
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get( Constants.driveConstants.leftRearMotorName );
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get( Constants.driveConstants.rightFrontMotorName );
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get( Constants.driveConstants.rightRearMotorName );

        // Use the following line as a template for defining new servos
        //claw = (Servo) hardwareMap.servo.get("claw");

        // Reverse certain drive motors so that positive power to all motors makes the robot move forwards
        // TODO: Update "Constants" with the proper directions of your drive motors
        motorLF.setDirection( Constants.driveConstants.leftFrontMotorDirection );
        motorLB.setDirection( Constants.driveConstants.leftRearMotorDirection );
        motorRF.setDirection( Constants.driveConstants.rightFrontMotorDirection );
        motorRB.setDirection( Constants.driveConstants.rightRearMotorDirection );

        // This resets the encoder values when the code is initialized
        motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // This lets you look at encoder values while the OpMode is active
        // If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        rhinoL = (DcMotorEx)  hardwareMap.dcMotor.get("rhinoL");
        rhinoR = (DcMotorEx) hardwareMap.dcMotor.get("rhinoR");
        eat = (Servo) hardwareMap.servo.get("eat");

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rhinoL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhinoR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eat.setPosition(0);

        rhinoL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rhinoR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rhinoL.setDirection(DcMotorSimple.Direction.REVERSE);
        rhinoR.setDirection(DcMotorSimple.Direction.FORWARD);

        //voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

    }

    boolean reverseDrive = false;
    boolean lastLeftBumper = false;

    // This code runs repeatedly until the Stop button is pressed on the Driver Station
    // Replaces the old  while(OpModeIsActive())  loop
    @Override
    public void loop() {


        //follower.update();

        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Mecanum drive code
        double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
        double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
        double powerAng = 0.0;  // Desired power for turning          (-1 to 1)


        // Set the desired powers based on joystick inputs (-1 to 1)
        powerX = gamepad1.left_stick_x;
        powerY = -gamepad1.left_stick_y;
        powerAng = -gamepad1.right_stick_x;


        // Perform vector math to determine the desired powers for each wheel
        double powerLF = powerX + powerY - powerAng;
        double powerLB = -powerX + powerY - powerAng;
        double powerRF = -powerX + powerY + powerAng;
        double powerRB = powerX + powerY + powerAng;

        // Determine the greatest wheel power and set it to max
        double max = Math.max(1.0, Math.abs(powerLF));
        max = Math.max(max, Math.abs(powerRF));
        max = Math.max(max, Math.abs(powerLB));
        max = Math.max(max, Math.abs(powerRB));



        // Scale all power variables down to a number between 0 and 1 (so that setPower will accept them)
        powerLF /= max;
        powerLB /= max;
        powerRF /= max;
        powerRB /= max;


        motorLF.setPower(powerLF);
        motorLB.setPower(powerLB);
        motorRF.setPower(powerRF);
        motorRB.setPower(powerRB);

        double velDifference = Math.abs(rhinoL.getVelocity() - rhinoR.getVelocity());

        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intake.setPower(-1);
        }
        else if ((gamepad1.left_bumper || gamepad2.left_bumper) && (((gamepad1.right_trigger >= 0.2 || gamepad2.right_trigger >= 0.2) && velDifference <= 20) || (gamepad1.right_trigger <= 0.2 || gamepad2.right_trigger <= 0.2))) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }


        // FLYWHEEL CODE
        PIDFCoefficients flywheelCoefficients = new PIDFCoefficients(flywheelP, 0, 0, flywheelF);
        rhinoL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelCoefficients);
        rhinoR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelCoefficients);


        if (gamepad1.right_trigger >= 0.2 || gamepad2.right_trigger >= 0.2) {
            rhinoL.setVelocity(flywheelVelocity);
            rhinoR.setVelocity(flywheelVelocity);
            eat.setPosition(0.4);
        }
        else if (gamepad1.left_trigger >= 0.2 || gamepad2.left_trigger >= 0.2) {
            rhinoL.setVelocity(-flywheelVelocity);
            rhinoR.setVelocity(-flywheelVelocity);
        }
        else {
            rhinoL.setPower(0);
            rhinoR.setPower(0);
            eat.setPosition(0);
        }
        /*
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            eat.setPosition(0.5);
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            eat.setPosition(0);
        }*/

        // If you want to print information to the Driver Station, use telemetry
        // addData() lets you give a string which is automatically followed by a ":" when printed
        //     the variable that you list after the comma will be displayed next to the label
        // update() only needs to be run once and will "push" all of the added data+

        telemetryManager.addData("launchVel L", rhinoL.getVelocity());
        telemetryManager.addData("launchVel R", rhinoR.getVelocity());

        telemetryManager.addData("launchError L", flywheelVelocity - rhinoL.getVelocity());
        telemetryManager.addData("launchError R", flywheelVelocity - rhinoR.getVelocity());

        telemetryManager.addData("targetVel", flywheelVelocity);

        telemetryManager.update();

        telemetry.addData("Open", isOpen);
        telemetry.addData("Servo Pos", eat.getPosition());
        telemetry.addData("velocity R", rhinoR.getVelocity());
        telemetry.addData("velocity L", rhinoL.getVelocity());
        telemetry.addData("velocity difference", velDifference);
        //telemetry.addData("in close launch", launchDetection());
        telemetry.update();

    }

    // Any additional methods go here
/*
    public boolean launchDetection()
    {
        double pinX = pinpoint.getPosX(DistanceUnit.INCH);
        double pinY = pinpoint.getPosY(DistanceUnit.INCH);

        if (pinX >= 70) {
            double minY = pinX;
            if (pinY >= minY) {
                return true;
            } else {
                return false;
            }
        } else {
            double minY = -pinX + 140;
            if (pinY >= minY) {
                return true;
            } else {
                return false;
            }
        }
    }*/

}