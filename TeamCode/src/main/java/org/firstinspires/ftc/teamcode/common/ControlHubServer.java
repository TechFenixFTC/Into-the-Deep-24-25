package org.firstinspires.ftc.teamcode.common;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public class ControlHubServer extends NanoHTTPD {
    public ControlHubServer(int port) throws IOException {
        super(port);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
        System.out.println("Servidor rodando em http://localhost:" + port);
    }
    @Override
    public Response serve(IHTTPSession session) {
        String uri = session.getUri();
        if (uri.equals("/")) {
            return newFixedLengthResponse(Response.Status.OK, "text/plain", "API do Control Hub funcionando!");
        } else if (uri.startsWith("/files")) {
            return serveFile(uri.replace("/files/storage/emulated/0/FTCLogs/", ""));
        }
        return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "Rota não encontrada!");
    }

    private Response serveFile(String filePath) {
        // file path = /storage/emulated/0/FTCLogs/
        try {
            File file = new File("/files/storage/emulated/0/FTCLogs/" + filePath); // Ajuste conforme necessário
            if (!file.exists()) {
                return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "Arquivo não encontrado!");
            }
            FileInputStream fis = new FileInputStream(file);
            return newChunkedResponse(Response.Status.OK, "application/octet-stream", fis);
        } catch (Exception e) {
            return newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "text/plain", "Erro ao ler arquivo!");
        }
    }
}

