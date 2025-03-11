package org.firstinspires.ftc.teamcode.common.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V2;

@Disabled
@TeleOp(name = "DistanceSensorTeste")
public class DistanceSensorTeste extends OpMode {
    private V2 robot;
    double drive;
    double turn;
    double strafe;
    double fLeftPow, fRightPow, bLeftPow, bRightPow;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        drive = gamepad1.left_stick_y * -1;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        fLeftPow = Range.clip(drive + turn + strafe, -1, 1);
        bLeftPow = Range.clip(drive + turn - strafe, -1, 1);
        fRightPow = Range.clip(drive - turn - strafe, -1, 1);
        bRightPow = Range.clip(drive - turn + strafe, -1, 1);



        telemetry.update();

    }
}
