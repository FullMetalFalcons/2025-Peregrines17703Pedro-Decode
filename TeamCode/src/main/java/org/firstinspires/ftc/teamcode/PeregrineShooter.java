package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Line;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable

public class PeregrineShooter {
    DcMotorEx intake, belt, launcher;
    Servo pusher;
    ElapsedTime stateTimer = new ElapsedTime();
    LauncherState currentState = LauncherState.IDLE;
    public double prepTime;
    boolean launcherPrepared = false;
    int ballsToFire = 0;

    public enum LauncherState {
        IDLE,
        LOAD,
        PREPARE,
        LAUNCH,
        PREPARESHOOT,
        FOLLOW_THROUGH
    }
    //public boolean isBusy = false;

    public void init(HardwareMap hardwareMap)
    {
        currentState = LauncherState.IDLE;
        intake = (DcMotorEx) hardwareMap.get("intake");
        belt = (DcMotorEx) hardwareMap.get("belt");
        launcher = (DcMotorEx) hardwareMap.get("launch_ball");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setVelocityPIDFCoefficients(/*15*/ 800, 0, /*1*/0, 50  /*20*/);

        pusher = (Servo) hardwareMap.servo.get("pusher");
    }

    public void update()
    {
        switch(currentState) {
            case IDLE:

                    launcher.setVelocity(0);
                    pusher.setPosition(.75);
                    belt.setPower(0);
                    intake.setPower(0);

                break;
            case PREPARESHOOT:
                    //stateTimer.reset();
                    if (stateTimer.seconds() < 2.5)
                        launcher.setVelocity(1300);
                    else if (stateTimer.seconds() >= 2.5) {
                        changeState(LauncherState.LAUNCH);

                }
                break;
            case LAUNCH:

                    //stateTimer.reset();
                    launcher.setVelocity(1250);
                    intake.setPower(-.5);
                    belt.setPower(-.25);
                    if (stateTimer.seconds() <= .5)
                    {
                        pusher.setPosition(0);
                    }
                    //pusher.setPosition(0);
                    if (stateTimer.seconds() > .5) {
                        pusher.setPosition(.75);
                        launcher.setVelocity(1350);
                    }
                    if (stateTimer.seconds() >= .75) {
                        belt.setPower(1);
                    }
                    if (stateTimer.seconds() > 2) {
                        changeState(LauncherState.IDLE);
                    }

                break;
            case PREPARE:
                launcher.setVelocity(1250);
                break;
            case LOAD:
                intake.setPower(-1);
                belt.setPower(1);
                launcher.setVelocity(-1000);
        }

    }

    public void changeState(LauncherState newState)
    {
        stateTimer.reset();
        currentState = newState;
    }

    public boolean isBusy()
    {
        return (currentState != LauncherState.IDLE && currentState != LauncherState.PREPARE && currentState != LauncherState.LOAD);
    }
    public void launchBallsUnprepared()
    {
        if (!isBusy())
            changeState(LauncherState.PREPARESHOOT);
    }

    public void PrepareBalls()
    {
        if (!isBusy())
            changeState(LauncherState.PREPARE);
    }

    public void Intake()
    {
        if (!isBusy())
        {
            changeState(LauncherState.LOAD);
        }
    }

    public void launchBallsPrepared()
    {
        changeState(LauncherState.LAUNCH);
    /*public void launchBallsPrepared(double delay)
    {
        launcher(1400, 3);
        pusher(0);
        belt(1, 3);

    }

    public void belt(double power, double time)
    {
        ElapsedTime stateTimer = new ElapsedTime();
        beltPowered(power, time, stateTimer);
    }
    public boolean beltPowered(double power, double time, ElapsedTime timer)
    {
        if (timer.seconds() < time)
        {
            belt.setPower(power);
            return false;
        }
        else
        {
            belt.setPower(0);
            return true;
        }
    };

    public boolean intake(double power, double time)
    {
        ElapsedTime stateTimer = new ElapsedTime();
        if (stateTimer.seconds() < time)
        {
            intake.setPower(power);
            return false;
        }
        else
        {
            intake.setPower(0);
            return true;
        }
    };

    public boolean launcher(double velocity, double time)
    {
        ElapsedTime stateTimer = new ElapsedTime();
        if (stateTimer.seconds() < time)
        {
            launcher.setVelocity(velocity);
            return false;
        }
        else
        {
            launcher.setVelocity(0);
            return true;
        }
    }

    public boolean pusher(double position)
    {
        ElapsedTime stateTimer = new ElapsedTime();
        if (stateTimer.seconds() < .3)
        {
            pusher.setPosition(position);
            return false;
        }
        else
        {
            pusher.setPosition(0.75);
            return true;
        }
    };*/

        }
    }
