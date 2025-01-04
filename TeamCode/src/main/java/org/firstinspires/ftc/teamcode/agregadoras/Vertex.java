package org.firstinspires.ftc.teamcode.agregadoras;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.BracoGarra;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.Garra;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearHorizontal;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearVertical;

import java.util.ArrayList;
import java.util.List;

public class Vertex {
    public Telemetry telemetry;
    public  LinearVertical linearVertical;
    public  LinearHorizontal linearHorizontal;

    public DistanceSensor distanceSensor;
    public  Garra garra;
    public  BracoGarra braco;
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



    public Vertex(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.distanceSensor = new DistanceSensor(hardwareMap);
        this.linearVertical = new LinearVertical(hardwareMap);
        this.linearHorizontal = new LinearHorizontal(hardwareMap);
        this.garra = new Garra(hardwareMap);
        this.braco = new BracoGarra(hardwareMap);
        this.runningActions = new ArrayList<>();
        this.RobotState = getRobotState();



    }


    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }


    public void IntermediatePosition(double runTime){
        switch (RobotState) {
            case Intermediate:
                // codigo de intake no submersivo
                //carteiro.addOrder(linearVertical.ElevadorGoTo(500),runTime, "elevador descer");
                carteiro.addOrder(garra.fecharGarra(),runTime,"garra fechar");
                carteiro.addOrder(braco.goToAnyPosition(runTime,0, Globals.IS_ARMED),runTime,"braco");


        }
    }

    public  void InitialPosition(double runTime){
        switch (RobotState) {
            case Initial:
                //carteiro.addOrder(linearVertical.ElevadorGoTo(100),runTime, "vertical subindo");
                carteiro.addOrder(linearHorizontal.recolher(runTime, runTime),0, "recolher horizontal");
                carteiro.addOrder(braco.goToAnyPosition(runTime,0, Globals.IS_STORED),runTime,"braco");

        }
    }

    public void OuttakePosition(double runTime){
        switch (RobotState) {
            case Outtake:
                //carteiro.addOrder(linearVertical.ElevadorGoTo(2700),runTime, "linear vertical subir");
                //carteiro.addOrder(linearHorizontal.recolher(runTime, 0), runTime, "linear horizontal recolher");
                carteiro.addOrder(braco.goToAnyPosition(runTime,0, Globals.IS_SCORING),runTime,"braco");


        }
    }

    public void IntakePosition(double runTime){
        switch (RobotState){
            case Intake:
                // bind do gamepad2.x
                // vertical subir
                // horizontal esticar
                carteiro.addOrder(braco.goToAnyPosition(runTime,0, Globals.IS_INTAKE),runTime,"braco");
        }

    }

    public  void ChamberOuttake(double runTime){
        switch (RobotState){
            case ChamberOuttake:

        }
    }
    public  void ChamberIntake(double runTime){
        switch (RobotState){
            case ChamberIntake:

        }

    }
    public void GarraPosition(double runTime){
        switch (RobotState){

        }
    }

    /*
     preciso colocar aqui as ações do vertex, que funcionem tanto pro teleoperado tanto pro autônomo
     pra isso funcionar, preciso criar as ações individuais em cada um dos subsistemas que constituem o vertex.
     Por enquanto é possível testar sem implementar as ações assim, só vou tirar as ações macro do teleoperado por enquanto.
     *
     *
     * */
}