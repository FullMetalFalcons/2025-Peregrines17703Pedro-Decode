package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class FalconsTeleOp extends OpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, belt, ball, rhino;
    Servo pusher;
    // TODO: Uncomment the following line if you are using servos
    //Servo claw;

    // The following code will run as soon as "INIT" is pressed on the Driver Station
    @Override
    public void init() {

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

        belt = (DcMotorEx) hardwareMap.dcMotor.get("belt");
        ball = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        rhino = (DcMotorEx)  hardwareMap.dcMotor.get("launch_ball");
        //imu = lazyImu.get();
        // Use the following line as a template for defining new servos
        //Claw = (Servo) hardwareMap.servo.get("claw");
        pusher = (Servo) hardwareMap.servo.get("pusher");

        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ball.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhino.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rhino.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rhino.setVelocityPIDFCoefficients(/*15*/ 800, 0, /*1*/0, 50  /*20*/);

        //voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

    }

    boolean reverseDrive = false;
    boolean lastLeftBumper = false;

    // This code runs repeatedly until the Stop button is pressed on the Driver Station
    // Replaces the old  while(OpModeIsActive())  loop
    @Override
    public void loop() {

        boolean slowMode = gamepad1.right_bumper;
        // Mecanum drive code
        double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
        double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
        double powerAng = 0.0;  // Desired power for turning          (-1 to 1)

        if (gamepad1.left_bumper && !lastLeftBumper)
        {
            reverseDrive = !reverseDrive;
        }
        lastLeftBumper = gamepad1.left_bumper;

        if (!reverseDrive)
        {
            powerAng = -gamepad1.right_stick_x;
        }
        else
        {
            powerAng = gamepad1.right_stick_x;
        }

        // Set the desired powers based on joystick inputs (-1 to 1)
        powerX = gamepad1.left_stick_x;
        powerY = -gamepad1.left_stick_y;


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
        //gooofy lefty

        if (slowMode)
        {
            powerLF /= 10;
            powerLB /= 10;
            powerRF /= 10;
            powerRB /= 10;
        }

        if (!reverseDrive)
        {
            motorLF.setPower(powerLF);
            motorLB.setPower(powerLB);
            motorRF.setPower(powerRF);
            motorRB.setPower(powerRB);
        }
        else
        {
            motorLF.setPower(-powerLF);
            motorLB.setPower(-powerLB);
            motorRF.setPower(-powerRF);
            motorRB.setPower(-powerRB);
        }


        // goofy belt
        boolean beltUp = gamepad2.dpad_up;
        boolean beltBown = gamepad2.dpad_down;

        if(beltUp)
        {
            belt.setPower(1);
        }
        else if(beltBown)
        {
            belt.setPower(-1);
        }
        else {
            belt.setPower(0);
        }

        boolean intake = gamepad2.x;
        boolean outake = gamepad2.a;

        if(intake)
        {
            ball.setPower(1);
        }
        else if(outake)
        {
            ball.setPower(-1);
        }
        else
        {
            ball.setPower(0);
        }

        //double multiplier = 13/voltageSensor.getVoltage();

        boolean launch_ball = gamepad2.right_bumper;
        boolean reverse_launcher = gamepad2.left_bumper;
        if(launch_ball)
        {
                /*multiplier = 13/voltageSensor.getVoltage();
                rhino.setPower(.75 * multiplier);*/
            rhino.setVelocity(1800);
        }
        else if (reverse_launcher)
        {
                /*multiplier = 13/voltageSensor.getVoltage();
                rhino.setPower(.57 * multiplier);*/
            //rhino.setVelocity(20520, AngleUnit.DEGREES);
            rhino.setVelocity(1400);

        }
        else
        {
            rhino.setPower(0);
        }

        boolean pusherIn = gamepad2.dpad_right;
        boolean pusherOut = gamepad2.dpad_left;

        if (pusherOut)
        {
            pusher.setPosition(.25);
        }
        else
        {
            pusher.setPosition(.75);
        }
        // If you want to print information to the Driver Station, use telemetry
        // addData() lets you give a string which is automatically followed by a ":" when printed
        //     the variable that you list after the comma will be displayed next to the label
        // update() only needs to be run once and will "push" all of the added data

        telemetry.addData("PerpEncoderTicks",belt.getCurrentPosition());
        telemetry.addData("Launcher Encoder Ticks", rhino.getCurrentPosition());
        telemetry.addData("LF power", powerLF);
        telemetry.addData("LB power", powerLB);
        telemetry.addData("RF power", powerRF);
        telemetry.addData("RB power", powerRB);
        //telemetry.addData("Launcher power", (.65 * multiplier));
        telemetry.update();

    }

    // Any additional methods go here

}