package org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controller.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalMotor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal.LinearHorizontalSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

public class Subsistemas {
    public Telemetry telemetry;
    public  LinearVertical linearVertical;
    public LinearHorizontalMotor horizontalIMotor;
    public LinearHorizontalSuperior horizontalSuperior;

    public DistanceSensor distanceSensor;
    public GarraInferior garraInferior;
    public GarraSuperior garraSuperior;
    public BracoGarraSuperior braco;
    //public List<Action> runningActions = new ArrayList<>();
    public OrdersManager carteiro =new OrdersManager(telemetry);
    HardwareMap hardwaremap;




    public Subsistemas(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        //this.distanceSensor = new DistanceSensor(hardwareMap);
        this.linearVertical = new LinearVertical(hardwareMap);
        this.horizontalIMotor = new LinearHorizontalMotor(hardwareMap);
        this.horizontalSuperior = new LinearHorizontalSuperior(hardwareMap);
        this.garraInferior = new GarraInferior(hardwareMap);
        this.garraSuperior = new GarraSuperior(hardwareMap);
        //this.runningActions = new ArrayList<>();



        // todo: Fazer uma agregadora separada para os subsistemas inferiores - By miguel - feito!
        // todo: Usar essa classe como classe pai para as outras agregadoras de subsitemas - By miguel
    }


    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }
}