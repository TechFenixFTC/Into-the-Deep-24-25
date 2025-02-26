package org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5Modes;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Controller.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalMotor;
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


    public void DiseablePSEinferior(OrdersManager carteiro, double runtime){
        carteiro.addOrder(intakeSuccao.pwmDiseable(), 0, "PWM diseable", runtime);
    }

    public void ejectingSampleWrong(OrdersManager carteiro, double runtime, V5Modes v5Mode, String ladoAliança) {
        if(underGrounSubystemStates == UnderGrounSubystemStates.INTAKE || underGrounSubystemStates == UnderGrounSubystemStates.READY_TOINTAKE ) {
                carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSample(), 0, "ready intake", runtime);
                carteiro.addOrder(intakeSuccao.TotalOpenPositionAlcapao(), 0.6, "alcapao aberto", runtime);
                carteiro.addOrder(intakeSuccao.GotoIntakeSample(), 3, "subsistemas superiores", runtime);
        }
    }

    public void goToInitial_goToReadyTransfer(OrdersManager carteiro, double runtime){
        carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao transfer", runtime);
        carteiro.addOrder(actionIntakeIrPraInitial(),0,"horizonte",runtime);
    }
    public Action actionIntakeIrPraInitial() {

        Action returnedAction;

            returnedAction = new SequentialAction(
                    // muda o estado para going to retracted
                    actionLevantarAntesDeRetrair(),
                    linearHorizontalMotor.goToRetracted(),
                    new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.1).build(),
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
                    intakeSuccao.GotoReadyToIntakeSpecimen(),
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
        carteiro.addOrder(intakeSuccao.IntakePositionAlcapao(),0.0,"alcapao aberto2",runtime);
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0,"horizonte",runtime);
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
        cooldown = runTime + 0.35;

        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){
            goToReadyToIntakeSample(carteiro,runTime);
            return;
        }
        else if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE){
            goToIntakeSample(carteiro, runTime);
            return;
        }
        goToReadyToIntakeSample(carteiro, runTime);
    }

}
