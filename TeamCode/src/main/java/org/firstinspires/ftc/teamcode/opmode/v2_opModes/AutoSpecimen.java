package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V2;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@Config
@Autonomous(name = "AutonomoSpecimen", group = "Autonomous")
public class AutoSpecimen extends LinearOpMode {
    V5 robot;

    @Override
    public void runOpMode()  {

        Pose2d initialPose = new Pose2d(8.5, -61, Math.toRadians(-90));

        robot = new V5(hardwareMap,telemetry);
        robot.md.pose = initialPose;
        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeCHAMBER()
                )
        );
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        sample1(),
                        goToSample2(),
                        intake()


                        //sample2(),
                        //sample3(),
                        //sample4(),
                        //irParaCasa()
                )

        );

    }


    public Action goToDeposit(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        //todo: colocar primeiro specimen
                        .setTangent(Math.toRadians(135))
                        .splineToConstantHeading(new Vector2d(0, -29.3), Math.toRadians(-90))
                        //todo: Go to empurrar sample 1

                        .build()
        );
    }
    public Action deposit() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(900),

                    robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeCHAMBER()
                )

        );

    }
    public Action intake(){
        return new SequentialAction(

                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToIntakeCHAMBER(),
                        robot.outtakeIntakeSuperior.garraSuperior.goToIntakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(0),
                        robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()

                )
        );
    }

    public Action goToSample2() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .splineToConstantHeading(new Vector2d(54, -30),Math.toRadians(90))

                        .splineToConstantHeading(new Vector2d(47, -8), Math.toRadians(-90))
                        .lineToX(61)
                        .lineToY(-50)
                        //.splineToConstantHeading(new Vector2d(61, -8), Math.toRadians(-90))
                        //.splineToConstantHeading(new Vector2d(61, -50), Math.toRadians(-90))
                        .build()
        );
    }
    public Action goToSample3() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(58, 50, Math.toRadians(-90)), Math.toRadians(-45))
                        .build()
        );
    }
    public Action goToSample4(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(46, 48, Math.toRadians(-45)), Math.toRadians(180))
                        .build()
        );
    }

    public Action sample1() {
        return new SequentialAction(
                new ParallelAction(
                        deposit(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit()
                        )

                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );
    }
    public Action sample2() {
        return new SequentialAction(
                goToSample2(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),
                intake(),
                deposit()
        );
    }
    public Action sample3() {
        return new SequentialAction(
                goToSample3(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                intake(),
                deposit()
        );
    }
    public Action sample4(){
        return new SequentialAction(
                goToSample4(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                intake(),
                deposit()

        );
    }

    public Action irParaCasa(){
        return new SequentialAction(
                new ParallelAction(
                        //mover lineares para estacionar
                        goToHouse()
                )


        );
    }
    public Action goToHouse() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-135))
                        .splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)), Math.toRadians(-135))
                        .build()
        );

    }
}