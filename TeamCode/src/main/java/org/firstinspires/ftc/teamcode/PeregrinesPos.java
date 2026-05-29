package org.firstinspires.ftc.teamcode; //67

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Configuration")
@Configurable
public class PeregrinesPos extends OpMode {

    TelemetryManager telemetryManager;

    public static Pose closeStart = new Pose(26, 129, 143);
    public static Pose farStart = new Pose(56, 8, 90);
    public static Pose startPos = new Pose();
    public static int pos = 0;
    public static String[] positions = {"Blue Close", "Red Close", "Blue Far", "Red Far"};
    public static boolean isSolo = false;

    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        telemetryManager.addData("Position", positions[pos]);
        telemetryManager.addData("Solo? ", isSolo);
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            pos += 1;
            if (pos > 3) {
                pos = 0;
            }
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            isSolo = !isSolo;
        }


    }

    @Override
    public void start() {
        if (pos == 0) {
            startPos = closeStart;
        }
        if (pos == 1) {
            startPos = closeStart.mirror();
        }
        if (pos == 2) {
            startPos = farStart;
        }
        if (pos == 3) {
            startPos = farStart.mirror();
        }
    }


    @Override
    public void loop() {
        telemetryManager.addData("Selected start position: ", positions[pos]);
    }

    }
