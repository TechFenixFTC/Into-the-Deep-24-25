package org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.BracoGarra.BracoGarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;

import java.util.ArrayList;
import java.util.List;

public class SubsistemasInferiores {

    public Telemetry telemetry;
    public IntakeSuccao intakeSuccao;
    public UnderGrounSubystemStates underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
    public GarraInferior garraInferior;
    public BracoGarraInferior bracoGarraInferior;
    public LinearHorizontalInferior horizontalInferior;
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

            carteiro.addOrder(horizontalInferior.goToExtended(),0, "horizontal inferior", runtime);
            carteiro.addOrder(intakeSuccao.GotoIntake(),0.3,"sucção",runtime);

            // todo: deprecated
            /*carteiro.addOrder(garraInferior.goToIntake(),0.1,"garra inferior", runtime);
            carteiro.addOrder(bracoGarraInferior.goToIntake(), 0.2, "braco garra inferior", runtime);
            carteiro.addOrder(garraInferior.fecharGarra(),0.4,"fecharGarra", runtime);*/


    }

    public void goToTransfer(OrdersManager carteiro, double runtime){

        carteiro.addOrder(horizontalInferior.goToRetracted(),0.0, "horizontal inferior", runtime);
        carteiro.addOrder(intakeSuccao.GoToInitial(),0.3,"sucção",runtime);
        /*carteiro.addOrder(garraInferior.goToTransfer(), 0, "garra Inferior", runtime);
        carteiro.addOrder(bracoGarraInferior.goToTransfer(), 0.0, "braco garra inferior", runtime);*/


    }

    public void goToReadyToIntake(OrdersManager carteiro, double runtime){
            carteiro.addOrder(horizontalInferior.goToExtended(),0,"horizontal inferior", runtime);
            /*carteiro.addOrder(bracoGarraInferior.goToIntake(), 0.1, "braco garra inferior", runtime);
            carteiro.addOrder(garraInferior.goToReadytoIntake(), 0, "garra inferior", runtime);*/
    }
    public void goToReadyTransfer(OrdersManager carteiro, double runtime){
        carteiro.addOrder(horizontalInferior.goToRetracted(),0.12, "horizontal inferior", runtime);
            /* carteiro.addOrder(garraInferior.goToReadyTransfer(), 0, "garra Inferior", runtime);
            carteiro.addOrder(garraInferior.fecharGarra(),0,"feeecharGarra", runtime);
            carteiro.addOrder(bracoGarraInferior.goToTransfer(), 1.0, "braco garra inferior", runtime);*/

    }
    public Action goToReadytoIntakeNoHorizontal(OrdersManager carteiro, double runtime){return new InstantAction(() -> {carteiro.addOrder(garraInferior.goToReadytoIntake(), 0, "garra inferior", runtime);});}

}
