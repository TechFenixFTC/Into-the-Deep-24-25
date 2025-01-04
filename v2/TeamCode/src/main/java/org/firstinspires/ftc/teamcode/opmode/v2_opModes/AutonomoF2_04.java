package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.V2;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Autonomo F2 0 + 4 (BlueBasket)")

public class AutonomoF2_04  extends LinearOpMode {

    V2 robot;
    Action trajectoryaction1, t2;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = createRobot(hardwareMap, 0, 0, 0);

        robot.linearHorizontal.targetPositionBracoDaGarra = 0.50;
        robot.garra.fecharGarra();
        sleep(500);
        robot.linearHorizontal.servoBracoDaGarra.setPosition(0.50);


        trajectoryaction1 = robot.md.actionBuilder(robot.md.pose).splineTo(new Vector2d(5, 8), Math.toRadians(90)).build();
        t2 = robot.md.actionBuilder(robot.md.pose).strafeToLinearHeading(new Vector2d(38 , 19), Math.toRadians(60)).build();







        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                robot.linearVertical.ElevadorGoTo(3500),
                                trajectoryaction1,
                                robot.linearHorizontal.goToOutake()
                        ),
                        robot.md.actionBuilder(robot.md.pose).splineTo(new Vector2d(5, 17), Math.toRadians(120)).build(),
                        robot.garra.abrirGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),
                        robot.linearHorizontal.goToStored(),
                        robot.md.actionBuilder(robot.md.pose).splineTo(new Vector2d(5, -10), Math.toRadians(60)).build(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(0.1).build(),
                        new ParallelAction(
                                robot.linearVertical.ElevadorGoTo(200),
                                t2
                        )


                )
        );


    }

    private V2 createRobot(HardwareMap hardwareMap, double x, double y, double heading) {
        V2 robot = new V2(hardwareMap, telemetry);
        robot.md.pose = new Pose2d(x, y, Math.toRadians(heading));
        return  robot;

    }

}