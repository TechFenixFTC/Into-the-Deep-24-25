package org.firstinspires.ftc.teamcode.opmode.v5_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior.UpperSubsystemStates;


@Config
@Autonomous(name = "AutonomoSpecimen4+0 beta ", group = "Autonomous")
public class AutoSpecimen5mais0Sugando extends LinearOpMode {
    V5 robot;
    Action push;
    public static int target = 0;
    private boolean encerrar;
    private Action proximaAction;


    @Override
    public void runOpMode()  {

        Pose2d initialPose = new Pose2d(8.5, -61, Math.toRadians(-90));

        robot = new V5(hardwareMap,telemetry);
        MecanumDrive.PARAMS.maxProfileAccel = 80;
        MecanumDrive.PARAMS.minProfileAccel = -80;
        MecanumDrive.PARAMS.maxWheelVel  = 80;

        robot.md.pose = initialPose;
        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.garraSuperior.goToInitialSpecimen()
                        //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
                        //robot.intakeInferior.horizontalInferior.goToRetracted()

                )
        );
        //push = pushSamples();
        waitForStart();
        while (robot.controladora.sampleID <= 4) {
            encerrar = false;
            proximaAction = robot.controladora.decideProximaAcao(robot, getRuntime());
            if(proximaAction != null){
                Actions.runBlocking(
                        new ParallelAction(
                                //todo Roda a próxima ação retornada pela controladora em paralelo com nossas automatizações
                                AutomationSpecimen(),
                                new SequentialAction(
                                        proximaAction,
                                        new InstantAction(() -> encerrar = true)
                                )

                        )
                );
            }

        }

        // todo: reseta no final
        encerrar = false;
        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
        robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL;
        Actions.runBlocking(
                AutomationSpecimen()
        );

    }
    public Action AutomationSpecimen(){
        return new Action() {
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    started = true;
                }

                robot.outtakeIntakeSuperior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.monitorAutonomo(telemetryPacket);
                robot.runStatesSpecimenAutonomo(robot.carteiro, getRuntime(),robot);
                telemetryPacket.addLine("Power Sugador: "+ robot.intakeInferior.intakeSuccao.sugador.getPower());
                telemetryPacket.addLine("tempo de execução: "+ getRuntime());
                telemetryPacket.addLine("sampleID: "+ robot.controladora.sampleID);
                telemetryPacket.addLine("estadoAutonomo: "+ robot.controladora.estadoSample);
                telemetryPacket.addLine("quantas vezes mandou a próxima ação: " + robot.controladora.quantasVezesFoiExecutado);
                if(encerrar){
                    return false;
                }
                return true;

            }
        };
    }

    public Action goToDeposit(){
        return robot.md.actionBuilder(robot.md.pose)
                //todo: colocar primeiro specimen
                .strafeTo(new Vector2d(-6,-22))
                //.splineToConstantHeading(new Vector2d(-6, -26), Math.toRadians(90))
                //todo: Go to empurrar sample 1
                .build();

    }
    public Action pushSamples() {
        return
                robot.md.actionBuilder(robot.md.pose)

                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(-85)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(44, -5, Math.toRadians(-90)), Math.toRadians(0))

                        .setTangent(Math.toRadians(-90))
                        .lineToY(-58)

                        //todo empurrar sample 2

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(56, -5, Math.toRadians(-95)), Math.toRadians(-90))



                        .lineToY(-57)
                        .setTangent(Math.toRadians(90))
                        //.strafeToConstantHeading(new Vector2d(47, -63))
                        .splineToLinearHeading(new Pose2d(43, -65, Math.toRadians(-90)), Math.toRadians(-90))
                        //.splineToLinearHeading(new Pose2d(55, -45, Math.toRadians(-90)), Math.toRadians(-85))
                        .waitSeconds(0.5)
                        //.lineToY(-66)

                        .build();


    }
    public Action goToDeposit2() {
        return robot.md.actionBuilder(new Pose2d(51, -63, Math.toRadians(-90)))
                //todo: colocar primeiro specimen
                //.strafeTo(new Vector2d(-6,-22))
                .strafeTo(new Vector2d(-12,-17))
                //.splineToSplineHeading(new Pose2d(EIXOX,-27,Math.toRadians(-90)
                .build();
    }
    public Action goToDeposit3() {
        return robot.md.actionBuilder(robot.md.pose)
                //todo: colocar primeiro specimen
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(0, -30.5), Math.toRadians(90))
                .build();
    }
    public Action firstIntake(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        //.strafeTo(new Vector2d(38 ,-35))
                        .lineToY(-63,  new TranslationalVelConstraint(30.0))
                        .build(),

                collect()

        );
    }
    public Action goToPark() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .strafeTo(new Vector2d(-5,-40))
                        .build()
        );

    }
    public Action deposit() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(1150),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeSpecimen(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit()
                        )
                ),

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );

    }
    public Action deposit2() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(1150),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeSpecimen(),
                        new SequentialAction(
                                //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit2()
                        )
                ),

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );

    }


    public Action intake(double x , double y){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                positionIntake(),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(2.60).build(),
                                collect()
                        ),
                        robot.md.actionBuilder(new Pose2d(x, y, Math.toRadians(-90)))
                                //.strafeTo(new Vector2d(38 ,-35))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(51, -55, Math.toRadians(-90)), Math.toRadians(-90))
                                .waitSeconds(0.4)
                                .lineToY(-66)
                                .build()
                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.30).build()


        );
    }

    public  Action positionIntake() {
        return new ParallelAction(
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()                                                                                                                                                                                                           ,
                robot.outtakeIntakeSuperior.braco.goToIntakeCHAMBER(),
                robot.outtakeIntakeSuperior.garraSuperior.goToIntakeSpecimen(),
                robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(-100)


        );
    }
    public Action collect() {
        return new SequentialAction(
                robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.45).build()
        );
    }
    public Action collect2() {
        return new SequentialAction(
                //new InstantAction(()->{robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.setPosition(2);}),
                robot.md.actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.8).build()
                //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build()
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

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),
                //intake(),
                deposit()
        );
    }
    public Action sample3() {
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                //intake(),
                deposit()
        );
    }
    public Action sample4(){
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                //intake(),
                deposit()

        );
    }

    public Action irParaCasa(){
        return new SequentialAction(
                new ParallelAction(
                        goToPark(),
                        new SequentialAction(
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(0.45).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(0)
                        )

                )


        );
    }

}