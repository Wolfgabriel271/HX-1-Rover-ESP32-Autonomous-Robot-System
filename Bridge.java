

//  Architecture:
//    Phone (HTML) <─── WebSocket ───> Bridge.java
//                                        │
//                                   /dev/ttyUSBx
//                                     ESP32
//                              (motors + sensors)

import com.fazecast.jSerialComm.SerialPort;           
import org.java_websocket.WebSocket;                   
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.JSONObject;                            

import com.sun.net.httpserver.HttpServer;             
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpExchange;

import java.io.*;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class Bridge {

    // ── Port / speed config ────────────────────────────────────
    static final int    ESP32_BAUD  = 115200;
    static final int    WS_PORT     = 8765;
    static final int    HTTP_PORT   = 9090;
    static volatile String ESP32_PORT = null;

    // ── Mode tuning ────────────────────────────────────────────
    // SUMO
    static final int SUMO_SEARCH_SPD = 150;
    static final int SUMO_ATTACK_SPD = 255;
    static final int SUMO_CLOSE_CM   = 60;
    static final int SUMO_SIDE_CM    = 50;

    // FOLLOW
    static final int FOLLOW_LOCK_CM   = 50;
    static final int FOLLOW_CLOSE_CM  = 30;
    static final int FOLLOW_FAR_CM    = 80;
    static final int FOLLOW_CREEP_CM  = 60;
    static final int FOLLOW_SPD       = 240;
    static final int FOLLOW_CREEP_SPD = 190;
    static final int FOLLOW_BACK_SPD  = 220;

    // OBSTACLE
    static final int OBS_FRONT_CM    = 40;
    static final int OBS_SIDE_CM     = 25;
    static final int OBS_SIDE_WARN_CM = 50;
    static final int OBS_SPD         = 180;
    static final int OBS_TURN_SPD    = 180;
    static final int OBS_BACK_SPD    = 160;

    // MAZE
    static final int MAZE_FRONT_CM  = 30;
    static final int MAZE_OPEN_CM   = 60;
    static final int MAZE_WALL_CM   = 15;
    static final int MAZE_JUMP_CM   = 40;
    static final int MAZE_FWD_SPD   = 160;
    static final int MAZE_TURN_SPD  = 160;
    static final int MAZE_DRIFT_SPD = 80;

    // GENERAL
    static final int SPD_FULL = 240;
    static final int SPD_MED  = 180;
    static final int SPD_SLOW = 110;
    static final int SPD_TURN = 160;

    // ── Shared state ───────────────────────────────────────────
    static final Object stateLock = new Object();

    // Sensor readings (updated by esp32 reader thread)
    static int sensorF  = 999, sensorL = 999, sensorR = 999;
    static int sensorIL = 0,   sensorIR = 0;

    // Robot state (synced to WebSocket clients)
    static String  mode      = "MANUAL";
    static int[]   servos    = {90, 90, 90, 90, 90, 90};
    static int     lightMode = 0;
    static int     armSpeed  = 5;
    static int     motorSL   = 0, motorSR = 0;

    // ── Serial port handle ─────────────────────────────────────
    static volatile SerialPort esp = null;

    // ── Connected WebSocket clients ────────────────────────────
    static final Set<WebSocket> clients =
        Collections.newSetFromMap(new ConcurrentHashMap<>());

    // ── Autonomous thread control ──────────────────────────────
    static volatile boolean modeRunning = false;

    // ── Sumo mode state ────────────────────────────────────────
    static String sumoState     = "SEARCH";
    static long   sumoStateTime = 0L;
    static int    sumoIrCount   = 0;

    // ── Obstacle mode state ────────────────────────────────────
    static long obsBackupUntil = 0L;

    // ── Maze mode state ────────────────────────────────────────
    static int mazePrevL = 999, mazePrevR = 999;

    // ==========================================================
    //  Serial helpers
    // ==========================================================

    static void detectPorts() {
        System.out.println("[DETECT] Scanning USB ports...");
        SerialPort[] ports = SerialPort.getCommPorts();
        List<SerialPort> usbPorts = new ArrayList<>();
        for (SerialPort p : ports) {
            String name = p.getSystemPortName();
            if (name.contains("USB") || name.contains("ACM")) {
                usbPorts.add(p);
            }
        }
        if (usbPorts.isEmpty()) {
            System.out.println("[DETECT] No USB ports found!");
            return;
        }
        System.out.println("[DETECT] Found: " + usbPorts.size() + " port(s)");
        if (usbPorts.size() == 1) {
            ESP32_PORT = usbPorts.get(0).getSystemPortName();
            System.out.println("[DETECT] Using " + ESP32_PORT);
            return;
        }
        // Probe each port for ESP32 JSON (has SL key)
        for (SerialPort p : usbPorts) {
            try {
                p.setBaudRate(115200);
                p.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 3000, 0);
                if (!p.openPort()) continue;
                Thread.sleep(500);
                StringBuilder buf = new StringBuilder();
                long deadline = System.currentTimeMillis() + 3000;
                while (System.currentTimeMillis() < deadline) {
                    if (p.bytesAvailable() > 0) {
                        byte[] b = new byte[1];
                        p.readBytes(b, 1);
                        char ch = (char) b[0];
                        if (ch == '\n') {
                            String line = buf.toString().trim();
                            if (line.startsWith("{") && line.contains("\"SL\"")) {
                                System.out.println("[DETECT] ESP32 on " + p.getSystemPortName());
                                p.closePort();
                                ESP32_PORT = p.getSystemPortName();
                                return;
                            }
                            buf.setLength(0);
                        } else if (ch != '\r') {
                            buf.append(ch);
                        }
                    }
                }
                p.closePort();
            } catch (Exception e) {
                System.out.println("[DETECT] " + p.getSystemPortName() + ": " + e.getMessage());
            }
        }
        ESP32_PORT = usbPorts.get(0).getSystemPortName();
        System.out.println("[DETECT] Fallback: " + ESP32_PORT);
    }

    static SerialPort openPort(String portName, int baud) {
        System.out.println("[ESP32] Connecting on " + portName + " @ " + baud + "...");
        SerialPort port = SerialPort.getCommPort(portName);
        port.setBaudRate(baud);
        port.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 100, 0);
        if (port.openPort()) {
            System.out.println("[ESP32] Connected.");
            return port;
        }
        System.out.println("[ESP32] Failed to open port.");
        return null;
    }

    // ==========================================================
    //  ESP32 reader thread
    // ==========================================================

    static void startEspReader() {
        Thread t = new Thread(() -> {
            StringBuilder buf = new StringBuilder();
            while (true) {
                if (ESP32_PORT == null) {
                    sleep(2000); continue;
                }
                if (esp == null || !esp.isOpen()) {
                    esp = openPort(ESP32_PORT, ESP32_BAUD);
                    if (esp == null) { sleep(3000); continue; }
                }
                try {
                    byte[] b = new byte[1];
                    int n = esp.readBytes(b, 1);
                    if (n <= 0) continue;
                    char ch = (char) b[0];
                    if (ch == '\n') {
                        String line = buf.toString().trim();
                        if (line.startsWith("{")) {
                            try {
                                JSONObject data = new JSONObject(line);
                                synchronized (stateLock) {
                                    if (data.has("F"))  sensorF  = data.getInt("F");
                                    if (data.has("L"))  sensorL  = data.getInt("L");
                                    if (data.has("R"))  sensorR  = data.getInt("R");
                                    if (data.has("IL")) sensorIL = data.getInt("IL");
                                    if (data.has("IR")) sensorIR = data.getInt("IR");
                                    if (data.has("SL")) motorSL  = data.getInt("SL");
                                    if (data.has("SR")) motorSR  = data.getInt("SR");
                                }
                                broadcast(buildStateJson());
                            } catch (Exception ignored) {}
                        } else if (line.length() > 0) {
                            System.out.println("[ESP32] " + line);
                        }
                        buf.setLength(0);
                    } else if (ch != '\r') {
                        if (buf.length() < 256) buf.append(ch);
                        else buf.setLength(0);
                    }
                } catch (Exception e) {
                    System.out.println("[ESP32] Lost connection — retrying...");
                    if (esp != null) esp.closePort();
                    esp = null;
                    sleep(2000);
                }
            }
        }, "esp-reader");
        t.setDaemon(true);
        t.start();
    }

    // ==========================================================
    //  Broadcast helpers
    // ==========================================================

    static String buildStateJson() {
        synchronized (stateLock) {
            JSONObject j = new JSONObject();
            j.put("F",  sensorF);  j.put("L",  sensorL);  j.put("R", sensorR);
            j.put("IL", sensorIL); j.put("IR", sensorIR);
            j.put("mode", mode);
            j.put("lightMode", lightMode);
            j.put("armSpeed",  armSpeed);
            j.put("SL", motorSL); j.put("SR", motorSR);
            // servos array
            org.json.JSONArray sa = new org.json.JSONArray();
            for (int v : servos) sa.put(v);
            j.put("servos", sa);
            return j.toString();
        }
    }

    static void broadcast(String msg) {
        Set<WebSocket> dead = new HashSet<>();
        for (WebSocket ws : clients) {
            try { ws.send(msg); }
            catch (Exception e) { dead.add(ws); }
        }
        clients.removeAll(dead);
    }

    // ==========================================================
    //  ESP32 write
    // ==========================================================

    static void espWrite(String cmd) {
        if (esp != null && esp.isOpen()) {
            byte[] data = (cmd.trim() + "\n").getBytes(StandardCharsets.UTF_8);
            try { esp.writeBytes(data, data.length); }
            catch (Exception e) { esp = null; }
        }
    }

    // ==========================================================
    //  Motor helpers
    // ==========================================================

    static void setMotors(int left, int right) {
        left  = Math.max(-255, Math.min(255, left));
        right = Math.max(-255, Math.min(255, right));
        synchronized (stateLock) { motorSL = left; motorSR = right; }
        espWrite("DRIVE:" + left + "," + right);
    }

    static void stopMotors() {
        setMotors(0, 0);
        espWrite("STOP");
    }

    static int[] getSensors() {
        // returns [F, L, R, IL, IR]
        synchronized (stateLock) {
            return new int[]{sensorF, sensorL, sensorR, sensorIL, sensorIR};
        }
    }

    // ==========================================================
    //  AUTONOMOUS MODES
    // ==========================================================

    // ── SUMO ──────────────────────────────────────────────────
    static void doSumo(int f, int l, int r, boolean il, boolean ir) {
        long now = System.currentTimeMillis();

        if (sumoState.equals("EVADE")) {
            long elapsed = now - sumoStateTime;
            if (elapsed < 200) {
                setMotors(-SPD_MED, -SPD_MED);
            } else if (elapsed < 500) {
                if (il) setMotors(SPD_TURN, -SPD_TURN);
                else    setMotors(-SPD_TURN, SPD_TURN);
            } else {
                sumoState = "SEARCH";
            }
            return;
        }

        if (f < SUMO_CLOSE_CM) {
            sumoIrCount = 0;
            sumoState = "CHARGE";
            setMotors(SUMO_ATTACK_SPD, SUMO_ATTACK_SPD);
            return;
        }

        if (l < SUMO_SIDE_CM && (r >= SUMO_SIDE_CM || l <= r)) {
            sumoIrCount = 0;
            sumoState = "SPIN_LEFT";
            setMotors(-SUMO_SEARCH_SPD, SUMO_SEARCH_SPD);
            return;
        }

        if (r < SUMO_SIDE_CM && (l >= SUMO_SIDE_CM || r < l)) {
            sumoIrCount = 0;
            sumoState = "SPIN_RIGHT";
            setMotors(SUMO_SEARCH_SPD, -SUMO_SEARCH_SPD);
            return;
        }

        if (il || ir) {
            sumoIrCount++;
            if (sumoIrCount >= 2) {
                sumoState     = "EVADE";
                sumoStateTime = now;
                sumoIrCount   = 0;
                setMotors(-SPD_MED, -SPD_MED);
            }
            return;
        } else {
            sumoIrCount = 0;
        }

        sumoState = "SEARCH";
        setMotors(SUMO_SEARCH_SPD, -SUMO_SEARCH_SPD);
    }

    // ── FOLLOW ────────────────────────────────────────────────
    static void doFollow(int f, int l, int r, boolean il, boolean ir) {
        if (f >= 999) {
            setMotors(120, -120);
            return;
        }
        if (f < FOLLOW_CLOSE_CM) { setMotors(-FOLLOW_BACK_SPD,  -FOLLOW_BACK_SPD);  return; }
        if (f < FOLLOW_LOCK_CM)  { setMotors(-FOLLOW_CREEP_SPD, -FOLLOW_CREEP_SPD); return; }
        if (f <= FOLLOW_CREEP_CM){ stopMotors(); return; }
        if (f < FOLLOW_FAR_CM)   { setMotors(FOLLOW_CREEP_SPD,  FOLLOW_CREEP_SPD);  return; }
        setMotors(FOLLOW_SPD, FOLLOW_SPD);
    }

    // ── OBSTACLE ──────────────────────────────────────────────
    static void doObstacle(int f, int l, int r, boolean il, boolean ir) {
        long now = System.currentTimeMillis();

        boolean frontBlocked  = f < OBS_FRONT_CM;
        boolean leftBlocked   = l < OBS_SIDE_CM;
        boolean rightBlocked  = r < OBS_SIDE_CM;
        boolean leftWarning   = l < OBS_SIDE_WARN_CM;
        boolean rightWarning  = r < OBS_SIDE_WARN_CM;

        if (now < obsBackupUntil) {
            long remaining = obsBackupUntil - now;
            if (remaining > 250) {
                setMotors(-OBS_BACK_SPD, -OBS_BACK_SPD);
            } else {
                if (l >= r) setMotors(-OBS_TURN_SPD, OBS_TURN_SPD);
                else        setMotors(OBS_TURN_SPD, -OBS_TURN_SPD);
            }
            return;
        }

        if (frontBlocked && leftBlocked && rightBlocked) {
            obsBackupUntil = now + 550;
            setMotors(-OBS_BACK_SPD, -OBS_BACK_SPD);
            return;
        }

        if (frontBlocked) {
            if (l >= r) setMotors(-OBS_TURN_SPD, OBS_TURN_SPD);
            else        setMotors(OBS_TURN_SPD, -OBS_TURN_SPD);
            return;
        }

        if (leftBlocked && !rightBlocked)  { setMotors(OBS_SPD / 2, OBS_SPD); return; }
        if (rightBlocked && !leftBlocked)  { setMotors(OBS_SPD, OBS_SPD / 2); return; }

        if (leftWarning && !rightWarning) {
            int correction = (int)((OBS_SIDE_WARN_CM - l) * 2.5);
            setMotors(OBS_SPD - correction, OBS_SPD);
            return;
        }
        if (rightWarning && !leftWarning) {
            int correction = (int)((OBS_SIDE_WARN_CM - r) * 2.5);
            setMotors(OBS_SPD, OBS_SPD - correction);
            return;
        }

        setMotors(OBS_SPD, OBS_SPD);
    }

    // ── MAZE ──────────────────────────────────────────────────
    static void doMaze(int f, int l, int r, boolean il, boolean ir) {
        int deltaL = (mazePrevL < 998) ? Math.abs(l - mazePrevL) : 0;
        int deltaR = (mazePrevR < 998) ? Math.abs(r - mazePrevR) : 0;
        mazePrevL = l;
        mazePrevR = r;

        boolean frontBlocked = f < MAZE_FRONT_CM;
        boolean rightOpen    = r > MAZE_OPEN_CM;
        boolean leftOpen     = l > MAZE_OPEN_CM;
        boolean rightWall    = r < MAZE_WALL_CM;
        boolean leftWall     = l < MAZE_WALL_CM;

        if (deltaL > MAZE_JUMP_CM && deltaL > deltaR) {
            if (l > mazePrevL - MAZE_JUMP_CM) setMotors(-MAZE_TURN_SPD, MAZE_TURN_SPD);
            else                               setMotors(MAZE_FWD_SPD, MAZE_FWD_SPD - MAZE_DRIFT_SPD);
            return;
        }
        if (deltaR > MAZE_JUMP_CM && deltaR > deltaL) {
            if (r > mazePrevR - MAZE_JUMP_CM) setMotors(MAZE_TURN_SPD, -MAZE_TURN_SPD);
            else                               setMotors(MAZE_FWD_SPD - MAZE_DRIFT_SPD, MAZE_FWD_SPD);
            return;
        }

        if (frontBlocked) {
            if (l >= r) setMotors(-MAZE_TURN_SPD, MAZE_TURN_SPD);
            else        setMotors(MAZE_TURN_SPD, -MAZE_TURN_SPD);
            return;
        }
        if (rightOpen && !leftOpen)  { setMotors(MAZE_FWD_SPD, MAZE_FWD_SPD - MAZE_DRIFT_SPD); return; }
        if (leftOpen  && !rightOpen) { setMotors(MAZE_FWD_SPD - MAZE_DRIFT_SPD, MAZE_FWD_SPD); return; }
        if (leftOpen  && rightOpen)  { setMotors(MAZE_FWD_SPD, MAZE_FWD_SPD); return; }
        if (leftWall  && rightWall)  { setMotors(MAZE_FWD_SPD, MAZE_FWD_SPD); return; }
        if (rightWall)               { setMotors(MAZE_FWD_SPD - 30, MAZE_FWD_SPD); return; }
        if (leftWall)                { setMotors(MAZE_FWD_SPD, MAZE_FWD_SPD - 30); return; }
        setMotors(MAZE_FWD_SPD, MAZE_FWD_SPD);
    }

    // ==========================================================
    //  Autonomous loop
    // ==========================================================

    static void runAutonomous() {
        modeRunning = true;
        while (modeRunning) {
            int[] s = getSensors();
            int f = s[0], l = s[1], r = s[2];
            boolean il = s[3] == 1, ir = s[4] == 1;
            String currentMode;
            synchronized (stateLock) { currentMode = mode; }

            switch (currentMode) {
                case "SUMO":     doSumo(f, l, r, il, ir);     break;
                case "FOLLOW":   doFollow(f, l, r, il, ir);   break;
                case "OBSTACLE": doObstacle(f, l, r, il, ir); break;
                case "MAZE":     doMaze(f, l, r, il, ir);     break;
                default: modeRunning = false; return;  // MANUAL — exit
            }
            sleep(80);  // ~12 Hz
        }
        modeRunning = false;
    }

    static void setMode(String newMode) {
        modeRunning = false;  // stop old autonomous thread
        synchronized (stateLock) { mode = newMode; }
        stopMotors();
        if (!newMode.equals("MANUAL")) {
            Thread t = new Thread(() -> {
                sleep(150);  // wait for old thread to exit
                runAutonomous();
            }, "auto-" + newMode);
            t.setDaemon(true);
            t.start();
        }
        System.out.println("[MODE] -> " + newMode);
    }

    // ==========================================================
    //  Track command state
    // ==========================================================

    static void trackCommand(String cmd) {
        synchronized (stateLock) {
            if (cmd.startsWith("SERVO:")) {
                String[] parts = cmd.substring(6).split(",");
                if (parts.length == 2) {
                    try {
                        int ch  = Integer.parseInt(parts[0].trim());
                        int ang = (int) Double.parseDouble(parts[1].trim());
                        if (ch >= 0 && ch < 6) servos[ch] = ang;
                    } catch (NumberFormatException ignored) {}
                }
            } else if (cmd.startsWith("LIGHT:")) {
                try { lightMode = Integer.parseInt(cmd.substring(6).trim()); }
                catch (NumberFormatException ignored) {}
            } else if (cmd.startsWith("ARMSPEED:")) {
                try { armSpeed = (int) Double.parseDouble(cmd.substring(9).trim()); }
                catch (NumberFormatException ignored) {}
            }
        }
    }

    // ==========================================================
    //  WebSocket server  (Java-WebSocket library)
    // ==========================================================

    static class RobotWsServer extends WebSocketServer {

        RobotWsServer(int port) {
            super(new InetSocketAddress(port));
        }

        @Override
        public void onOpen(WebSocket ws, ClientHandshake hs) {
            clients.add(ws);
            System.out.println("[WS] Client connected: " + ws.getRemoteSocketAddress());
            try { ws.send(buildStateJson()); } catch (Exception ignored) {}
        }

        @Override
        public void onClose(WebSocket ws, int code, String reason, boolean remote) {
            clients.remove(ws);
            System.out.println("[WS] Client disconnected");
        }

        @Override
        public void onMessage(WebSocket ws, String message) {
            String cmd = message.trim();

            if (cmd.startsWith("MODE:")) {
                String newMode = cmd.substring(5).toUpperCase();
                setMode(newMode);
                espWrite("MODE:" + newMode);
                return;
            }

            if (cmd.startsWith("DRIVE:") || cmd.equals("STOP")) {
                String currentMode;
                synchronized (stateLock) { currentMode = mode; }
                if (!currentMode.equals("MANUAL")) return;
                if (cmd.equals("STOP")) {
                    stopMotors();
                } else {
                    String[] parts = cmd.substring(6).split(",");
                    if (parts.length == 2) {
                        try {
                            setMotors(Integer.parseInt(parts[0].trim()),
                                      Integer.parseInt(parts[1].trim()));
                        } catch (NumberFormatException ignored) {}
                    }
                }
                return;
            }

            trackCommand(cmd);
            espWrite(cmd);
        }

        @Override
        public void onError(WebSocket ws, Exception ex) {
            System.out.println("[WS] Error: " + ex.getMessage());
        }

        @Override
        public void onStart() {
            System.out.println("[WS] Server started on port " + WS_PORT);
        }
    }

    // ==========================================================
    //  HTTP server  (built-in com.sun.net.httpserver)
    // ==========================================================

    static void startHttpServer() throws IOException {
        HttpServer server = HttpServer.create(new InetSocketAddress(HTTP_PORT), 0);
        server.createContext("/", new HttpHandler() {
            @Override
            public void handle(HttpExchange exchange) throws IOException {
                String path = exchange.getRequestURI().getPath();
                if (path.equals("/") || path.equals("/index.html")) {
                    File htmlFile = new File("controller.html");
                    if (!htmlFile.exists()) {
                        byte[] err = "controller.html not found".getBytes(StandardCharsets.UTF_8);
                        exchange.sendResponseHeaders(404, err.length);
                        exchange.getResponseBody().write(err);
                        exchange.getResponseBody().close();
                        return;
                    }
                    byte[] data = Files.readAllBytes(htmlFile.toPath());
                    exchange.getResponseHeaders().set("Content-Type", "text/html; charset=utf-8");
                    exchange.sendResponseHeaders(200, data.length);
                    exchange.getResponseBody().write(data);
                    exchange.getResponseBody().close();
                } else {
                    exchange.sendResponseHeaders(404, -1);
                }
            }
        });
        server.setExecutor(Executors.newFixedThreadPool(4));
        server.start();
        System.out.println("[HTTP] Serving at http://0.0.0.0:" + HTTP_PORT);
    }

    // ==========================================================
    //  Utility
    // ==========================================================

    static void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    // ==========================================================
    //  Main entry point
    // ==========================================================

    public static void main(String[] args) throws Exception {
        System.out.println("==================================================");
        System.out.println("  Robot Bridge — Orange Pi (Java)");
        System.out.println("  Modes: MANUAL / SUMO / FOLLOW / OBSTACLE / MAZE");
        System.out.println("==================================================");

        detectPorts();
        startEspReader();
        startHttpServer();

        RobotWsServer wsServer = new RobotWsServer(WS_PORT);
        wsServer.start();

        System.out.println("\n  Open on phone: http://192.168.100.1:" + HTTP_PORT + "\n");

        // Keep main thread alive
        Runtime.getRuntime().addShutdownHook(new Thread(() ->
            System.out.println("\n[BRIDGE] Stopped.")));
        Thread.currentThread().join();
    }
}