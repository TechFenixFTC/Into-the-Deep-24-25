package org.firstinspires.ftc.teamcode.agregadoras;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
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
    public  Garra garra;
    public  BracoGarra braco;
    private List<Action> runningActions = new ArrayList<>();
    HardwareMap hardwaremap;


    public Vertex(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.linearVertical = new LinearVertical(hardwareMap);
        this.linearHorizontal = new LinearHorizontal(hardwareMap);
        this.garra = new Garra(hardwareMap);
        this.braco = new BracoGarra(hardwareMap);
        this.runningActions = new ArrayList<>();

    }


    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }

    public enum vertexState{
        Initial,
        Outtake,
        Intermediate,
        ChamberOuttake,
        ChamberIntake,
    }

    public static vertexState RobotState;

    public void IntermediatePosition(double runTime){
        switch (RobotState) {
            case Intermediate:
                // codigo de intake no submersivo
                linearVertical.targetPosition = 30;


                runningActions.add(garra.fecharGarra());
                // runningActions.add(robot.intakeOutake.braco.goToIntermediatePosition(getRuntime(),0.2));
                //robot.intakeOutake.linearVertical.changeTarget(getRuntime(), 0.5, 280);

                // robot.intakeOutake.linearHorizontal.recolher(getRuntime(), 0.7);
        }
    }

    public  void InitialPosition(double runTime){
        switch (RobotState) {
            case Initial:
                linearVertical.targetPosition = 0;
                linearHorizontal.recolher(runTime, 0);
                runningActions.add(braco.goToChamberOutake(runTime, 0));
                //runningActions.add(robot.intakeOutake.braco.goToStored(runTime, 1));

        }
    }

    public void OuttakePosition(double runTime){
        switch (RobotState) {
            case Outtake:
                linearVertical.targetPosition = 3000;
                // robot.intakeOutake.linearHorizontal.extender(getRuntime(), 0);
                linearHorizontal.recolher(runTime, 0.3);
                // runningActions.add(braco.goToBasketOutake(runTime, 0));

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

    /*
     preciso colocar aqui as ações do vertex, que funcionem tanto pro teleoperado tanto pro autônomo
     pra isso funcionar, preciso criar as ações individuais em cada um dos subsistemas que constituem o vertex.
     Por enquanto é possível testar sem implementar as ações assim, só vou tirar as ações macro do teleoperado por enquanto.
     *
     *
     * */
}
