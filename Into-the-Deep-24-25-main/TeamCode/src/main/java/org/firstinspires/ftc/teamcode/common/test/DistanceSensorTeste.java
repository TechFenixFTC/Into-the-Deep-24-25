package org.firstinspires.ftc.teamcode.common.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.agregadoras.V2;

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
        robot.intakeOutake.distanceSensor.sensorDistanceRight.addTarget(new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, 2));
        robot.intakeOutake.distanceSensor.sensorDistanceRight.addTarget(new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, 4));
        robot.intakeOutake.distanceSensor.sensorDistanceRight.addTarget(new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, 6));
        robot.intakeOutake.distanceSensor.sensorDistanceRight.addTarget(new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, 8));
        robot.intakeOutake.distanceSensor.sensorDistanceRight.addTarget(new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, 10));
        robot.intakeOutake.distanceSensor.sensorDistanceRight.addTarget(new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, 12));
        robot.intakeOutake.distanceSensor.sensorDistanceRight.addTarget(new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, 14));


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

        robot.md.leftFront.setPower(fLeftPow);
        robot.md.leftBack.setPower(bLeftPow);
        robot.md.rightFront.setPower(fRightPow);
        robot.md.rightBack.setPower(bRightPow);

        telemetry.addData("target ",robot.intakeOutake.distanceSensor.sensorDistanceRight.checkAllTargets());
        telemetry.update();

    }
}
