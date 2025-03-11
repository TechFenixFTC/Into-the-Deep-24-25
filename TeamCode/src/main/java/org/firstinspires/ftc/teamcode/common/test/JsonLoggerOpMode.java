package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.LogJsonManager;

@Disabled
@Autonomous(name = "JSON Logger Test")
public class JsonLoggerOpMode extends LinearOpMode {

    private LogJsonManager logger;

    @Override
    public void runOpMode() {
        logger = new LogJsonManager("log_ftc.json");
        telemetry.addData("Status", "PREPARANDO PRA INICIAR");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "INICIANDO LEITURA");
        telemetry.update();
        while (opModeIsActive()) {
            logger.addLog("INFO", "O robô está rodando...");
            sleep(1000); // Simula alguma operação
        }
        telemetry.addData("Status", "FINALIZANDO");
        telemetry.update();

        logger.addLog("INFO", "Finalizando o OpMode...");
        logger.saveLogs(); // Salva os logs ao final

        telemetry.addData("Status", "Logs salvos!");
        telemetry.update();
        sleep(3000); // Simula alguma operação

    }
}
