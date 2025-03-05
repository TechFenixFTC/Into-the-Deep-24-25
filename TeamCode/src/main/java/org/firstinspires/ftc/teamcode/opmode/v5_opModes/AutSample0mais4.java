package org.firstinspires.ftc.teamcode.opmode.v5_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperiorRotetionStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior.UpperSubsystemStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraAngulationStates;


@Config
@Autonomous(name = "TesteIntake", group = "Autonomous")
public class AutSample0mais4 extends LinearOpMode {
    V5 robot;
    Action push;
    public static double sample1y = -25,
            sample2Ax = 37, sample2Ay = -42, sample2Ah = 85,
            sample2Bh = -55,
            sample3Ax = 39, sample3Ay = -40, sample3Ah = 55,
            sample3Bh = -35,
            sample4Ax = 50, sample4Ay = -23, sample4Ah = 0,
            sample4Bx = 45, sample4By = -42, sample4Bh = -35
            ;


    ;


    public static double[] sample4 ={10,10,10};


    @Override
    public void runOpMode()  {

        Pose2d initialPose = new Pose2d(-35,  -62, Math.toRadians(0));

        robot = new V5(hardwareMap,telemetry);
        robot.md.pose = initialPose;
        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.fecharGarra();}),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.servoAngulacaoGarra.setPosition(0.625);}),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.garraChamber();}),


                        new InstantAction(() -> {robot.intakeInferior.intakeSuccao.angulacao.setPosition(0.033);})
                )
        );


        waitForStart();

        Actions.runBlocking(
                new ParallelAction(

                        AutomationSample(),
                        new SequentialAction(
                                goToDeposit(),
                            new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL),
                            new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.OUTTAKE)
                        )

                )
        );

    }
    public Action goToDeposit(){
        return robot.md.actionBuilder(robot.md.pose)

                // basket
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-51, -54, Math.toRadians(45)), Math.toRadians(-135))

                .build();

    }
    public Action goToEjectSample(double posex, double posey, double poseh, double sampleh) {
        return robot.md.actionBuilder(new Pose2d(posex, posey, Math.toRadians(poseh)))
                .turnTo(Math.toRadians(sampleh))
                .build();
    }
    public Action goToEjetingSample3(double posex, double posey, double poseh) {
        return robot.md.actionBuilder(new Pose2d(posex, posey, Math.toRadians(poseh)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(sample4Bx, sample4By, Math.toRadians(sample4Bh)), Math.toRadians(-90))
                .build();
    }

    public Action AutomationSample(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.runStatesSampleAutonomo(robot.carteiro, getRuntime(),robot);
                return true;
            }
        };
    }

    public  Action positionIntake() {
        return new ParallelAction(
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()                                                                                                                                                                                                           ,
                robot.outtakeIntakeSuperior.braco.goToIntakeCHAMBER(),
                robot.outtakeIntakeSuperior.garraSuperior.goToIntakeSpecimen(),
                robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(0)
        );
    }

    // todo colocar a rota certa e o comando certo também
    public Action collect() {
        return new SequentialAction(
                robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.8).build()
        );
    }

    public Action GoCollectSample2() {
        return robot.md.actionBuilder(robot.md.pose)

                // collect Sample 2
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-47.5, -50, Math.toRadians(90)), Math.toRadians(90))
                .build();

    }

    public Action GoCollectSample3() {//todo okey
        return robot.md.actionBuilder(robot.md.pose)

                // collect Sample 2
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(90)), Math.toRadians(135))
                .build();

    }
    public Action GoCollectSample4() {//todo okey
        return robot.md.actionBuilder(robot.md.pose)

                // collect Sample 2
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-46, -48, Math.toRadians(135)), Math.toRadians(0))
                .build();
    }
}