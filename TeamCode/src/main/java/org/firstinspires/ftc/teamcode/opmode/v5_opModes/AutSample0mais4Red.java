package org.firstinspires.ftc.teamcode.opmode.v5_opModes;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller.Controladora;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior.UpperSubsystemStates;


@Config
@Autonomous(name = "0+4 🔴🔴", group = "Autonomous")
public class AutSample0mais4Red extends LinearOpMode {
    V5 robot;
    Action push;
    public boolean encerrar = false;
    private double limeligtCorrection = 0;
    public static boolean girarAoContrario = false;
    Action proximaAction;
    ElapsedTime tempoDecorridoAutonomo = new ElapsedTime();

    public static double
            Xdeposit1 = -49 ,    Ydeposit1 = -57 ,  Hdeposit1 = 35,
            Xdeposit2 = -55 ,    Ydeposit2 = -53.3,  Hdeposit2 = 45,

            Xdeposit4 = -60 ,    Ydeposit4 = -55.3,  Hdeposit4 = 45,


            Xdeposit5 = -120 ,    Ydeposit5 = -124 ,  Hdeposit5 = 60,

            XCollect2 = -43.7 , YCollect2 = -43, HCollect2 = 89,
            LimelightXsample2 = 0.81,// distância entre o robo e a apriltag
            XCollect3 = -53.2 ,   YCollect3 = -43 , HCollect3 = 87,
            XCollect4 = -51 ,   YCollect4 = -39 , HCollect4 = 140,
            XCollect5 = -18,   YCollect5 = -5 , HCollect5 = 0, TangentCollect5 = 130, TangentCollect5part2 = 0,
            XCollect5part2 = -3,
            delaySoltarSample1 = 0.1, delaySoltarSample2 = 0.310, delaySoltarSample4 = 0.450, tangentCollect4 = 140, tangentDeposit4 = 145,
            LimelightDelayToRead = 0.400, currMax = 3.60, LimeLightKp = 15
            ;





    public static double[] sample4 ={10,10,10};


    @Override
    public void runOpMode()  {

        robot = Controladora.create0mai5Robot(hardwareMap, telemetry);

        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.fecharGarra();}),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.servoAngulacaoGarra.setPosition(0.625);}),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.garraChamber();}),
                        new InstantAction(() -> {robot.intakeInferior.intakeSuccao.angulacao.setPosition(0.466);}),
                        //todo : mudar estados do robo para outtake
                        new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL),
                        new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.OUTTAKE)
                )
        );


        waitForStart();
        resetRuntime();
        //tempoDecorridoAutonomo.reset();

        while (robot.controladora.sampleID <= 4) {
            encerrar = false;
            proximaAction = robot.controladora.decideProximaAcao(robot, getRuntime());
            if(proximaAction != null){
                Actions.runBlocking(
                        new ParallelAction(
                                //todo Roda a próxima ação retornada pela controladora em paralelo com nossas automatizações
                                AutomationSample(),
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
                AutomationSample()
        );

    }


    public Action AutomationSample(){
        return new Action() {
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    started =true;
                }

                robot.outtakeIntakeSuperior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.monitorAutonomo(telemetryPacket);
                robot.runStatesSampleAutonomo(robot.carteiro, getRuntime(),robot);
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

}