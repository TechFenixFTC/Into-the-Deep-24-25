package org.firstinspires.ftc.teamcode.Controller;

import android.os.Environment;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class OrdersManager {

    Telemetry telemetry;
    public OrdersManager(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static int numberOfOrders = 0;

    private static class Order {
        Action action;
        double time;
        String name;
        int ID;

        public Order(Action action, double time, String name) {
            this.action = action;
            this.time = time;
            this.name = name;
            numberOfOrders++;
            this.ID = numberOfOrders;
        }

        public String getName() { return name; }
        public double getTime() { return time; }
        public Action getAction() { return action; }
        public int getID() { return ID; }
    }

    private static class ActionLogObject {
        String name;
        int ID;
        double tempoDeInicio, tempoDeTermino, tempoDeExecucao;

        public ActionLogObject(String name, int ID, double tempoDeInicio) {
            this.name = name;
            this.ID = ID;
            this.tempoDeInicio = tempoDeInicio;
        }

        public String getName() { return name; }
        public int getID() { return ID; }
        public double getTempoDeExecucao() { return tempoDeExecucao; }
        public double getTempoDeInicio() { return tempoDeInicio; }
        public double getTempoDeTermino() { return tempoDeTermino; }

        public void setTempoDeTermino(double tempoDeTermino) {
            this.tempoDeTermino = tempoDeTermino;
            this.tempoDeExecucao = tempoDeTermino - this.tempoDeInicio;
        }

        // 🔥 Converte um único ALO para JSON
        public JSONObject toJson() {
            JSONObject json = new JSONObject();
            try {
                json.put("ID", ID);
                json.put("name", name);
                json.put("tempoDeInicio", tempoDeInicio);
                json.put("tempoDeTermino", tempoDeTermino);
                json.put("tempoDeExecucao", tempoDeExecucao);
            } catch (Exception e) {
                e.printStackTrace();
            }
            return json;
        }
    }

    private List<Order> runningActions = new ArrayList<>();
    private List<Order> orders = new ArrayList<>();
    private List<ActionLogObject> ALOs = new ArrayList<>();

    public void addOrder(Action action, double delay, String name, double runtime) {
        double time = delay + runtime;
        removeOrderByName(name);
        orders.add(new Order(action, time, name));
        telemetry.addData("Ação adicionada", name);
    }

    public void removeOrderByName(String name) {
        orders.removeIf(order -> order.getName().equals(name));
    }

    public void checkIfCanRun(double runtime) {
        List<Order> ordersToRemove = new ArrayList<>();

        for (Order order : orders) {
            if (order.getTime() <= runtime) {
                runningActions.add(order);
                ordersToRemove.add(order);

                // 🔥 Criar ActionLogObject no momento da execução
                ActionLogObject log = new ActionLogObject(order.getName(), order.getID(), runtime);
                ALOs.add(log);
            }
        }
        orders.removeAll(ordersToRemove);
    }

    public void runTeleopActions(double runtime) {
        TelemetryPacket packet = new TelemetryPacket();
        List<Order> newActions = new ArrayList<>();

        for (Order order : runningActions) {
            order.action.preview(packet.fieldOverlay());
            if (order.action.run(packet)) {
                newActions.add(order);
            } else {
                // 🚀 Atualiza o tempo de término do ActionLogObject
                for (ActionLogObject log : ALOs) {
                    if (log.getID() == order.getID()) {
                        log.setTempoDeTermino(runtime);
                    }
                }
            }
        }
        runningActions = newActions;
    }

    // 🔥 Função para salvar os logs em JSON
    public void saveLogsToJson() {
        try {
            // Diretório onde os logs são salvos
            File directory = new File(Environment.getExternalStorageDirectory(), "FTCLogs");
            if (!directory.exists()) {
                directory.mkdirs();
            }

            // 🕵️‍♂️ Descobrir o maior número de round já salvo
            int lastRound = 0;
            File[] files = directory.listFiles();
            if (files != null) {
                for (File file : files) {
                    String name = file.getName();
                    if (name.startsWith("ROUND") && name.contains(": logs de tempo de ações")) {
                        try {
                            int roundNumber = Integer.parseInt(name.split(" ")[1]);
                            if (roundNumber > lastRound) {
                                lastRound = roundNumber;
                            }
                        } catch (NumberFormatException ignored) {}
                    }
                }
            }

            // 🆕 Define o novo número de round
            int newRound = lastRound + 1;
            String filename = "ROUND " + newRound + " : logs de tempo de ações.json";
            File logFile = new File(directory, filename);

            // Criar um JSONArray para armazenar os logs
            JSONArray jsonArray = new JSONArray();
            for (ActionLogObject log : ALOs) {
                jsonArray.put(log.toJson());
            }

            // Salvar no arquivo
            try (FileWriter writer = new FileWriter(logFile, false)) {
                writer.write(jsonArray.toString(4)); // 4 espaços para indentação
                writer.flush();
            }

            telemetry.addData("Logs salvos!", filename);
        } catch (IOException | JSONException e) {
            e.printStackTrace();
            telemetry.addData("Erro ao salvar JSON", e.getMessage());
        }
    }

}
