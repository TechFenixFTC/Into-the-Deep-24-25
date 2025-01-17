package org.firstinspires.ftc.teamcode.agregadoras;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarraMotor.BracoGarraStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarraMotor.BracoGarraV4;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal.LinearHorizontalSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

import java.util.ArrayList;
import java.util.List;

public class Subsistemas {
    public Telemetry telemetry;
    public  LinearVertical linearVertical;
    public LinearHorizontalInferior horizontalInferior;
    public LinearHorizontalSuperior horizontalSuperior;

    public DistanceSensor distanceSensor;
    public GarraInferior garraInferior;
    public GarraSuperior garraSuperior;
    public BracoGarraV4 braco;
    public List<Action> runningActions = new ArrayList<>();
    public OrdersManager carteiro =new OrdersManager(telemetry);
    HardwareMap hardwaremap;

    public enum vertexState{
        Initial,
        Outtake,
        Intermediate,
        ChamberOuttake,
        ChamberIntake,
        Intake,
    }

    public vertexState RobotState;

    public vertexState getRobotState(){

        return RobotState;
    }
    public void setRobotState(vertexState newState) {
        this.RobotState = newState;

    }



    public Subsistemas(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        //this.distanceSensor = new DistanceSensor(hardwareMap);
        this.linearVertical = new LinearVertical(hardwareMap);
        this.braco = new BracoGarraV4(hardwareMap,telemetry);
        this.horizontalInferior = new LinearHorizontalInferior(hardwareMap);
        this.horizontalSuperior = new LinearHorizontalSuperior(hardwareMap);
        this.garraInferior = new GarraInferior(hardwareMap);
        this.garraSuperior = new GarraSuperior(hardwareMap);
        this.runningActions = new ArrayList<>();
        this.RobotState = getRobotState();



    }


    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }



    /*
     preciso colocar aqui as ações do vertex, que funcionem tanto pro teleoperado tanto pro autônomo
     pra isso funcionar, preciso criar as ações individuais em cada um dos subsistemas que constituem o vertex.
     Por enquanto é possível testar sem implementar as ações assim, só vou tirar as ações macro do teleoperado por enquanto.
     *
     *
     * */
}