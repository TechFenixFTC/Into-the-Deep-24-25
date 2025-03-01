package org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5Modes;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Controller.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalMotor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.AlcapaoStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.SugarAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.common.controls.MatchColor;

public class SubsistemasInferiores {
    public MatchColor matchColor;
    double cooldown = 0;
    public Telemetry telemetry;
    public IntakeSuccao intakeSuccao;
    public UnderGrounSubystemStates underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;


    public GarraInferior garraInferior;
    public LinearHorizontalMotor linearHorizontalMotor;
    HardwareMap hardwaremap;
    public SubsistemasInferiores(HardwareMap hardwareMap, Telemetry telemetry) {
        this.matchColor = new MatchColor();
        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.linearHorizontalMotor = new LinearHorizontalMotor(hardwareMap);
        this.intakeSuccao = new IntakeSuccao(hardwareMap);
    }
    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }

    public void runStates(OrdersManager carteiro, double runtime, V5Modes v5Mode, GamepadEx gamepad) {

        switch (underGrounSubystemStates) {
            case INITIAL:
                Initial_ReadyToTransferSample(carteiro, runtime, v5Mode, gamepad);
            case TRASNFER:

        }

    }
    public void Initial_ReadyToTransferSample(OrdersManager carteiro, double runtime, V5Modes v5Mode, GamepadEx gamepad) {
        boolean linearEstaRetraido         = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED;
        boolean intakeSuccaoTaNoModoIntake = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean alcapaoPosicaoProTransfer  = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntake    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE;
        boolean temUmaSampleNoIntake       = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer   = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        boolean usandoControleManual       = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0;
        double delay = 0;
        // AÇÕES DE TRANSIÇÃO Automática
            /* todo: ir para Transfer quando> 1- tiver uma sample na posição 2- alçapão pra transfer 3- horizontal retraido*/
            if(samplePosicaoPraTransfer && alcapaoPosicaoProTransfer && linearEstaRetraido) {
                carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.TRASNFER), 0, "Mudar modo subsistemas inferiores pra transfer", runtime);
            }
        // AÇÕES DO ESTADO
            /* todo: levanto o intake antes de retrair */
            if(!linearEstaRetraido && intakeSuccaoTaNoModoIntake && !linearHorizontalMotor.isBusy) {
                    delay = 0.2;
                if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                    carteiro.addOrder(
                            intakeSuccao.GotoReadyToIntakeSample(),
                            0.07,
                            "Intake Sucção Pra ReadyToIntake",
                            runtime
                    );
                }

            }

            /* todo: retrai o horizontal*/
            if(!linearEstaRetraido && !linearHorizontalMotor.isBusy && !intakeSuccaoTaNoModoIntake) {
                carteiro.addOrder(linearHorizontalMotor.goToRetracted(), delay, "retrairLinear", runtime);
            }

            /* todo: Manter pressionando o horiontal enquanto tiver uma sample no intake e o horizontal estiver retraido */
            if(linearEstaRetraido) {
                if(temUmaSampleNoIntake && LinearHorizontalMotor.targetPosition != -10){
                    LinearHorizontalMotor.targetPosition = -10;
                }else if(!temUmaSampleNoIntake && LinearHorizontalMotor.targetPosition != 25) {
                    LinearHorizontalMotor.targetPosition = 25;
                }
            }

            /* todo: Ações do intake de sucção quando tem uma sampleNoIntake */
            if(temUmaSampleNoIntake){
                if(samplePosicaoPraTransfer){
                    /* todo: alçapão na posição de transfer */
                    if(linearEstaRetraido && !alcapaoPosicaoProTransfer) {
                        carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao transfer", runtime);
                    }
                }
            }

            /* todo: Ações do intake de sucção quando NÃO tem uma sampleNoIntake */
            if(!samplePosicaoPraTransfer) {
                /* todo: alçapão pra posição onde a sample consiga entrar e encaiaxr */
                if(linearEstaRetraido && !alcapaoPosicaoPraIntake) {
                    carteiro.addOrder(intakeSuccao.IntakePositionAlcapao(), 0, "alcapao transfer", runtime);
                }
                /* todo: enquanto a sample não entra, ficar sugando ali */
                if(!usandoControleManual) {
                    intakeSuccao.IntakeAjeitarSample();
                }
            }





    }

    public void TransferSample(OrdersManager carteiro, double runtime, V5Modes v5Mode, GamepadEx gamepad) {
        boolean linearEstaRetraido         = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED;
        boolean intakeSuccaoTaNoModoIntake = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean alcapaoPosicaoProTransfer  = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntake    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE;
        boolean temUmaSampleNoIntake       = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer   = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        boolean usandoControleManual       = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0;
        double delay = 0;

    }

    public void ejectingSampleWrong(OrdersManager carteiro, double runtime, V5Modes v5Mode, String ladoAliança) {
        if(underGrounSubystemStates == UnderGrounSubystemStates.INTAKE || underGrounSubystemStates == UnderGrounSubystemStates.READY_TOINTAKE ) {
                carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSample(), 0, "ready intake", runtime);
                carteiro.addOrder(intakeSuccao.TotalOpenPositionAlcapao(), 0.6, "alcapao aberto", runtime);
                carteiro.addOrder(intakeSuccao.GotoIntakeSample(), 3, "subsistemas superiores", runtime);
        }
    }

    public void goToInitial_goToReadyTransfer(OrdersManager carteiro, double runtime){
        underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
        carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao transfer", runtime);
        carteiro.addOrder(actionIntakeIrPraInitial(),0,"horizonte",runtime);
    }
    public Action actionIntakeIrPraInitial() {

        Action returnedAction;

            returnedAction = new SequentialAction(
                    // muda o estado para going to retracted
                    actionLevantarAntesDeRetrair(),
                    linearHorizontalMotor.goToRetracted(),
                    intakeSuccao.GoToTransfer()
            );

        return returnedAction;
    }
    public Action actionLevantarAntesDeRetrair() {
        if (linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){ // se o horizontal ja estiver retraido não rola nada
            return new InstantAction(() -> {});
        }
        else { // se não ele levanta o intake antes de retrair
            return new SequentialAction(
                    intakeSuccao.GotoReadyToIntakeSample(),
                    new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.1).build()
            );
        }
    }
    public void goToTransfer(OrdersManager carteiro, double runtime)    {
        carteiro.addOrder(goToTransferAction(),0,"garra inferior", runtime);
    }
    public Action goToTransferAction()    {
        return new SequentialAction(
                linearHorizontalMotor.goToRetracted(),
                intakeSuccao.GoToTransfer()
        );
    }


    public void goToReadyToIntake(OrdersManager carteiro, double runtime){
            this.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
            carteiro.addOrder(intakeSuccao.ReadytoIntakePositionAlcapao(),0.0,"alcapao aberto3",runtime);
            carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSpecimen(),0.0,"angulation",runtime);
            carteiro.addOrder(linearHorizontalMotor.goToExtended(),0.4,"horizonte",runtime);
    }
    public void goToReadyToIntakeSample(OrdersManager carteiro, double runtime){
        this.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
        carteiro.addOrder(intakeSuccao.ReadytoIntakePositionAlcapao(),0.0,"alcapao aberto4",runtime);
        carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSpecimen(),0.0,"angulation",runtime);
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0.4,"horizonte",runtime);
    }
    public void goToIntakeSample(OrdersManager carteiro, double runtime){
        underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0,"horizonte",runtime);
        carteiro.addOrder(intakeSuccao.IntakePositionAlcapao(),0.2,"alcapao aberto2",runtime);
        carteiro.addOrder(intakeSuccao.GotoIntakeSample(),0,"angulation",runtime);
    }
    public void goToIntakeSpecimen(OrdersManager carteiro, double runtime){
        this.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
        carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao intake", runtime);
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0,"horizonte",runtime);
        carteiro.addOrder(intakeSuccao.GotoIntakeSpecimen(),0,"angulation",runtime);
    }

    public void gerenciadorIntakSpecimen(OrdersManager carteiro, double runTime){
        if(runTime < cooldown){
            return;
        }
        cooldown = runTime + 0.35;

        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){
            goToReadyToIntake(carteiro,runTime);
            intakeSuccao.IntakeParar();
            return;
        }
        else if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE ){
            goToIntakeSpecimen(carteiro, runTime);
            intakeSuccao.sugador.setPower(0.7);
            return;
        }
        goToReadyToIntake(carteiro, runTime);
    }
    public void gerenciadorIntakeSample(OrdersManager carteiro, double runTime){
        if(runTime < cooldown){
            return;
        }
        cooldown = runTime + 0.15; // 0.35

        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){
            goToReadyToIntakeSample(carteiro,runTime);
            return;
        }
        else if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE && !linearHorizontalMotor.isBusy){//
            goToIntakeSample(carteiro, runTime);
            return;
        }
        goToReadyToIntakeSample(carteiro, runTime);
    }

}
