package org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.SensorCor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.BracoGarra.BracoGarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalMotor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.SugarAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalStates;

public class SubsistemasInferiores {
    double cooldown = 0;
    public Telemetry telemetry;
    public IntakeSuccao intakeSuccao;
    public UnderGrounSubystemStates underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
    public GarraInferior garraInferior;
    public BracoGarraInferior bracoGarraInferior;
    public LinearHorizontalMotor linearHorizontalMotor;
    public SensorCor colorSensor;
    HardwareMap hardwaremap;
    public SubsistemasInferiores(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.linearHorizontalMotor = new LinearHorizontalMotor(hardwareMap);
        this.intakeSuccao = new IntakeSuccao(hardwareMap);

    }
    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }


    public void DiseablePSEinferior(OrdersManager carteiro, double runtime){
        carteiro.addOrder(intakeSuccao.pwmDiseable(), 0, "PWM diseable", runtime);
    }
    public void goToIntake(OrdersManager carteiro, double runtime){
            underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
            carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao intake", runtime);
            carteiro.addOrder(linearHorizontalMotor.goToExtended(),0,"horizonte",runtime);
            carteiro.addOrder(intakeSuccao.GotoIntakeSpecimen(),0,"angulation",runtime);


    }
    public void goToInitial(OrdersManager carteiro, double runtime){

        carteiro.addOrder(intakeSuccao.GoToInitial(),0,"succorIn",runtime);
        carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao transfer", runtime);
        carteiro.addOrder(linearHorizontalMotor.goToRetracted(),0,"horizonte",runtime);
        carteiro.addOrder(intakeSuccao.pwmDiseable(), 0.9, "alcapao transfer", runtime);
    }
    public void goToReadyToIntake(OrdersManager carteiro, double runtime){
            this.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
            carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSpecimen(),0.0,"angulation",runtime);
            carteiro.addOrder(linearHorizontalMotor.goToExtended(),0.4,"horizonte",runtime);
    }
    public void goToReadyToIntakeSample(OrdersManager carteiro, double runtime){
        this.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
        carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSpecimen(),0.0,"angulation",runtime);
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0.9,"horizonte",runtime);
    }
    public void goToIntakeSample(OrdersManager carteiro, double runtime){
        underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0,"horizonte",runtime);
        carteiro.addOrder(intakeSuccao.GotoIntakeSample(),0,"angulation",runtime);


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
    public void gerenciadorIntakSpecimen(OrdersManager carteiro, double runTime){
        double delay = 0.35;
        if(runTime < cooldown){
            return;
        }
        cooldown = runTime + delay;

        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){
            goToReadyToIntake(carteiro,runTime);
            return;
        }
        else if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE){
            goToIntake(carteiro, runTime);
            return;
        }
        goToReadyToIntake(carteiro, runTime);
    }
    public void gerenciadorIntakeSample(OrdersManager carteiro, double runTime){
        double delay = 0.35;
        if(runTime < cooldown){
            return;
        }
        cooldown = runTime + delay;

        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){
            goToReadyToIntakeSample(carteiro,runTime);
            return;
        }
        else if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE){
            goToIntakeSample(carteiro, runTime);
            return;
        }
        goToReadyToIntake(carteiro, runTime);
    }
    public Action goToReadytoIntakeNoHorizontal(OrdersManager carteiro, double runtime){return new InstantAction(() -> {carteiro.addOrder(garraInferior.goToReadytoIntake(), 0, "garra inferior", runtime);});}

}
