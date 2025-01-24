package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.HashMap;

@Config
public class BracoGarraV4 {
    public static boolean monitor;
    public Servo bracoGarraSuperior;
    BracoGarraStates bracoGarraSuperiorState = BracoGarraStates.Initial;

    HashMap<BracoGarraStates, Double> mapBracoSuperior = new HashMap<>();
    public static  double   maxAcc = 2700, maxVelocity  = 4400, distance = 0, tempo = 0;
    public BracoGarraV4(HardwareMap hardwareMap, Telemetry telemetry) {
        bracoGarraSuperior = hardwareMap.get(Servo.class, HardwareNames.bracoGarraSuperiorServo);
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
