package org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.SensorCor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.BracoGarra.BracoGarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.SugarAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalStates;

public class SubsistemasInferiores {
    double cooldown = 0;
    public Telemetry telemetry;
    public IntakeSuccao intakeSuccao;
    public UnderGrounSubystemStates underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
    public GarraInferior garraInferior;
    public BracoGarraInferior bracoGarraInferior;
    public LinearHorizontalInferior horizontalInferior;
    public SensorCor colorSensor;
    HardwareMap hardwaremap;
    public SubsistemasInferiores(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.horizontalInferior = new LinearHorizontalInferior(hardwareMap);
        this.intakeSuccao = new IntakeSuccao(hardwareMap);

        // todo : deprecated
        //this.bracoGarraInferior = new BracoGarraInferior(hardwareMap);
        //this.garraInferior = new GarraInferior(hardwareMap);

    }
    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }



    public void goToIntake(OrdersManager carteiro, double runtime){
            underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
            carteiro.addOrder(horizontalInferior.goToExtended(),0, "horizontal inferior", runtime);
            carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao intake", runtime);
            //carteiro.addOrder(intakeSuccao.IntakeSugar(),0, "intake sugador", runtime);
            carteiro.addOrder(intakeSuccao.GotoIntake(),0.5,"succor",runtime);

            // todo: deprecated
            /*carteiro.addOrder(garraInferior.goToIntake(),0.1,"garra inferior", runtime);
            carteiro.addOrder(bracoGarraInferior.goToIntake(), 0.2, "braco garra inferior", runtime);
            carteiro.addOrder(garraInferior.fecharGarra(),0.4,"fecharGarra", runtime);*/


    }
    public void goToInitial(OrdersManager carteiro, double runtime){

        carteiro.addOrder(intakeSuccao.GoToInitial(),0,"succorIn",runtime);
        //carteiro.addOrder(intakeSuccao.IntakeParar(), 0.0, "intake parar", runtime);
        //carteiro.addOrder(intakeSuccao.IntakeSugarMedio(),0.8,"intake sugador", runtime);
        //carteiro.addOrder(intakeSuccao.verifyColorSensor(),0.550, "verify color sensor", runtime);
        carteiro.addOrder(horizontalInferior.goToRetracted(),1.5, "horizontal inferior", runtime);
        carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 1, "alcapao transfer", runtime);
        //carteiro.addOrder(intakeSuccao.IntakeParar(),1,"intake parar depois", runtime);
        /*carteiro.addOrder(garraInferior.goToTransfer(), 0, "garra Inferior", runtime);
        carteiro.addOrder(bracoGarraInferior.goToTransfer(), 0.0, "braco garra inferior", runtime);*/
    }
    public void goToReadyToIntake(OrdersManager carteiro, double runtime){
            this.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
            carteiro.addOrder(horizontalInferior.goToExtended(),0,"horizontal inferior intake", runtime);
            carteiro.addOrder(intakeSuccao.GoToTransfer(),0.0,"angulation",runtime);
            carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(),0.0,"alcapao",runtime);
    }
    public void goToTransfer(OrdersManager carteiro, double runtime){
            carteiro.addOrder(intakeSuccao.GoToTransfer(),0,"garra inferior", runtime);
            /* carteiro.addOrder(garraInferior.goToReadyTransfer(), 0, "garra Inferior", runtime);
            carteiro.addOrder(garraInferior.fecharGarra(),0,"feeecharGarra", runtime);
            carteiro.addOrder(bracoGarraInferior.goToTransfer(), 1.0, "braco garra inferior", runtime);*/

    }
    public void gerenciadorIntake(OrdersManager carteiro, double runTime){
        double delay = 0.35;
        if(runTime < cooldown){
            return;
        }
        cooldown = runTime + delay;

        if(horizontalInferior.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){
            goToReadyToIntake(carteiro,runTime);
            return;
        }
        if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.INITIAL){
            goToIntake(carteiro, runTime);
            return;
        }
        goToReadyToIntake(carteiro, runTime);
    }
    public Action goToReadytoIntakeNoHorizontal(OrdersManager carteiro, double runtime){return new InstantAction(() -> {carteiro.addOrder(garraInferior.goToReadytoIntake(), 0, "garra inferior", runtime);});}

}
