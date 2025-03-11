package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Disabled
@TeleOp(name="Teste Servo")
public class TesteServos extends OpMode {

    List<Servo> servos = new ArrayList<>(4);
    String[] nomesServosTestados  = {"porta0c", "porta1c", "porta2c", "porta3c","porta4c"};

    double[] angulosServosTestado = {0, 0, 0, 0,0};
    int portaServoSendoTestado = 0;
    double cooldownChangePortaServo = 0;

    @Override
    public void init() {

        for (int i = 0; i < 5; i++) {
            servos.add(hardwareMap.get(Servo.class, nomesServosTestados[i]));
        }
    }

    @Override
    public void loop() {

        // Aumenta o ângulo, com limite
        if (gamepad1.dpad_right) {
            if (angulosServosTestado[portaServoSendoTestado] < 10.0) {  // Limite máximo
                angulosServosTestado[portaServoSendoTestado] += 0.001;
                servos.get(portaServoSendoTestado).setPosition(angulosServosTestado[portaServoSendoTestado]);
            }
        }

// Diminui o ângulo, com limite
        if (gamepad1.dpad_left) {
            if (angulosServosTestado[portaServoSendoTestado] > 0.0) {  // Limite mínimo
                angulosServosTestado[portaServoSendoTestado] -= 0.001;
                servos.get(portaServoSendoTestado).setPosition(angulosServosTestado[portaServoSendoTestado]);
            }
        }

        /* Muda a porta que está sendo alterada do servo */
        if(gamepad1.start) {
            if ( getRuntime() >=  cooldownChangePortaServo) {
                cooldownChangePortaServo = getRuntime() + 0.3;
                if (portaServoSendoTestado < 4) {
                    portaServoSendoTestado++;
                }
                else {
                    portaServoSendoTestado = 0;
                }
            }





        }
        // todo: write your code here
        telemetry.addLine("==== TESTES DE SERVO ====");
        telemetry.addData("Porta do servo", nomesServosTestados[portaServoSendoTestado]);
        telemetry.addData("Angulo do servo", angulosServosTestado[portaServoSendoTestado]);
        telemetry.addLine("============================");
        telemetry.update();
    }}


