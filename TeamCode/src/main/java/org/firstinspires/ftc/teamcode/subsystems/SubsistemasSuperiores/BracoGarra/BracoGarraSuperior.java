package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.HashMap;

@Config
public class BracoGarraSuperior {
    public static boolean monitor;
    public Servo bracoGarraSuperiorServo;
    BracoGarraSuperiorStates bracoGarraSuperiorState = BracoGarraSuperiorStates.TRANSFER;
    HashMap<BracoGarraSuperiorStates, Double> mapBracoSuperior = new HashMap<>();
    public static  double   maxAcc = 2700, maxVelocity  = 4400, distance = 0, tempo = 0;
    public BracoGarraSuperior(HardwareMap hardwareMap, Telemetry telemetry) {
        bracoGarraSuperiorServo = hardwareMap.get(Servo.class, HardwareNames.bracoGarraSuperiorServo);

        mapBracoSuperior.put(BracoGarraSuperiorStates.OUTTAKE, 0.665);
        mapBracoSuperior.put(BracoGarraSuperiorStates.INITIAL,0.283);
        mapBracoSuperior.put(BracoGarraSuperiorStates.TRANSFER, 0.202);
        mapBracoSuperior.put(BracoGarraSuperiorStates.INTAKE, 0.314);
        mapBracoSuperior.put(BracoGarraSuperiorStates.INTAKE_CHAMBER,0.65);
        mapBracoSuperior.put(BracoGarraSuperiorStates.OUTTAKE_CHAMBER,0.313);
    }
    public Action goToTransfer(){
        bracoGarraSuperiorState = BracoGarraSuperiorStates.TRANSFER;
        return new InstantAction(()->{
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    public Action goToOuttake(){
        bracoGarraSuperiorState = BracoGarraSuperiorStates.OUTTAKE_CHAMBER;
        return new InstantAction(()->{
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    public Action goToInital(){
        bracoGarraSuperiorState = BracoGarraSuperiorStates.INITIAL;
        return new InstantAction(() -> {
           bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    /*public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("==============================");
            telemetry.addLine("  TELEMETRIA DO BRAÇO DA GARRA");
            telemetry.addLine("===============================");
            telemetry.addData("-Angulo do braco: ",this.getAngle());
            telemetry.addData("-alvo: ",targetAngle);
            //telemetry.addData("",);

        }
    }*/

}
