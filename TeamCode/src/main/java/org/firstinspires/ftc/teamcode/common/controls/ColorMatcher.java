package org.firstinspires.ftc.teamcode.common.controls;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.SensorCor;


public class ColorMatcher {
    public static double distanciaMinima = 0.9, distanciaMaxima = 3.5;
    private final SensorCor sensorCor;
    public ColorMatcher(SensorCor sensorCor) {
        this.sensorCor = sensorCor;
    }

    // Método que calcula o Hue atual
    public float getHueValue() {
        int red = sensorCor.getRed();
        int green = sensorCor.getGreen();
        int blue = sensorCor.getBlue();

        float cMax = Math.max(red, Math.max(green, blue));
        float cMin = Math.min(red, Math.min(green, blue));
        float delta = cMax - cMin;

        if (cMax == 0) {
            return 0; // Preto ou ausência de cor
        }

        float hue = 0;
        if (delta != 0) {
            if (cMax == red) {
                hue = 60 * ((green - blue) / delta);
                if (hue < 0) hue += 360;
            } else if (cMax == green) {
                hue = 60 * ((blue - red) / delta + 2);
            } else if (cMax == blue) {
                hue = 60 * ((red - green) / delta + 4);
            }
        }

        return hue;
    }

    // Método para determinar a cor do elemento baseado no Hue
    public String getSampleColor() {
        float hue = getHueValue();

        if( sensorCor.getDistance() > ColorMatcher.distanciaMaxima ){
            return "Não há samples no intake";
        }
        if (sensorCor.getDistance() > ColorMatcher.distanciaMinima) {
            return "Sample no intake em posição incorreta pra transfer";
        }
        if (hue >= 210 && hue <= 270) {
            return "Azul";  // Azul -> 210° a 270°
        } else if (hue >= 330 || hue <= 30) {
            return "Vermelho"; // Vermelho -> 330° a 30°
        } else if (hue >= 40 && hue <= 100) {
            return "Amarelo"; // Amarelo -> 40° a 100°
        } else {

            return "Indefinido"; // Caso o sensor pegue um valor inesperado
        }
    }

    public boolean temUmaSampleNoIntake() {
        double distancia = sensorCor.getDistance();

        // Se a distância for maior que a máxima, não há sample
        if (distancia > ColorMatcher.distanciaMaxima) {
            return false;
        }
        // Se a distância for menor que a máxima, há uma sample no intake
        return true;
    }

    public boolean sampleNaPosicaoCorreta() {
        double distancia = sensorCor.getDistance();

        // Se a distância for maior que a mínima, a sample não está na posição correta
        return distancia <= ColorMatcher.distanciaMinima;
    }

    // 🔍 Função para depuração e monitoramento com Telemetria
    public void monitor(Telemetry telemetry) {
        int red = sensorCor.getRed();
        int green = sensorCor.getGreen();
        int blue = sensorCor.getBlue();
        float hue = getHueValue();
        double distancia = sensorCor.getDistance();
        String corDetectada = getSampleColor();

        telemetry.addData("🔴 Red", red);
        telemetry.addData("🟢 Green", green);
        telemetry.addData("🔵 Blue", blue);
        telemetry.addData("🎨 Hue", hue);
        telemetry.addData("📏 Distância", "%.2f cm", distancia);
        telemetry.addData("🖍️ Cor da Sample Detectada", corDetectada);
    }
}
