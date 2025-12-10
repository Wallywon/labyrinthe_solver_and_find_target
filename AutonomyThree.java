import java.util.Random;
import java.util.ArrayList;
import java.util.List;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;

/**
 * Classe principale gérant le comportement  d'un agent cognitif 
 * Niveau d'autonomie 3 avec une méthode d'auto-organisation
 */
public class AutonomyThree extends Robot {

    private int timeStep;
    private DistanceSensor[] distanceSensor;
    private Motor leftMotor;
    private Motor rightMotor;
    private PositionSensor leftMotorSensor;
    private PositionSensor rightMotorSensor;
    private double encoder_unit = 159.23;
    private Odometry odometry;
    private Camera camera;
    private Emitter emitter;
    private Receiver receiver;
    private LED[] leds;

    // Définition des états de l'agent 
    private enum State {
        INITIALIZATION, // Découverte du nombre total de robots
        EXPLORATION, // Recherche active de la cible
        ATTENTE, // Pause et synchronisation après découverte
        CONVERGENCE // Mouvement final vers la cible
    }
    private State currentState = State.INITIALIZATION; // L'état courant de l'agent

    private int NbRobots = 0; // Nombre total de robots dans l'environnement (déterminé en INITIALIZATION)
    private int NbTrouves = 0; // Compteur des robots ayant trouvé de la cible
    private boolean cibleTrouveeLocalement = false; // Vrai si ce robot a vu la cible lui-même

    private double initStartTime = 0;
    private static final double INIT_DURATION = 5.0; // 5 secondes pour la phase INITIALIZATION

    private int explorationCounter = 0; // Compteur de cycles d'exploration
    private int nextRotationTime = 0; // Temps aléatoire avant la prochaine rotation
    private Random random = new Random();
    private boolean isRotating = false; // indique si on est en train de faire un 360°
    private double accumulatedTurnAngle = 0.0; // angle total accumulé pendant la rotation
    private double lastAngleDuringTurn = 0.0; // Angle d'orientation au step précédent
    private boolean hasBroadcastedExist = false; // S'assurer que le message "J'existe" n'est envoyé qu'une seul

    // --- CONSTANTES ---
    private static final double OBSTACLE_SEUIL = 120.0; // Seuil pour considérer un capteur IR comme détectant un mur
    private static final double ROBOT_COLLISION_SEUIL = 0.20; // Seuil de distance pour considérer un robot comme trop proche
    private static final double TARGET_STOP_DISTANCE = 0.18; // Distance finale d'arrêt par rapport à la cible
    private static final double SPEED_EXPLORE = 30.0; // Vitesse de déplacement par défaut
    private static final double SPEED_FOLLOW = 50.0; // Vitesse de suivi (vers un robot ou la cible)
    private static final double ROTATION_SPEED = 25.0; // Vitesse de rotation pour les scans et l'évitement

    public AutonomyThree() {
        timeStep = 64;
        odometry = new Odometry();

        // Sensors initialization - IR distance sensors
        distanceSensor = new DistanceSensor[8];
        String[] sensorNames = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
        for (int i = 0; i < 8; i++) {
            distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
            distanceSensor[i].enable(timeStep);
        }

        // Camera
        camera = this.getCamera("camera");
        camera.enable(timeStep);
        camera.recognitionEnable(timeStep);

        // WiFi communication
        emitter = getEmitter("emitter");
        receiver = getReceiver("receiver");
        receiver.enable(timeStep);

        // Motors
        leftMotor = this.getMotor("left wheel motor");
        rightMotor = this.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);

        // Motor sensors
        leftMotorSensor = this.getPositionSensor("left wheel sensor");
        rightMotorSensor = this.getPositionSensor("right wheel sensor");
        leftMotorSensor.enable(timeStep);
        rightMotorSensor.enable(timeStep);

        // LEDs
        leds = new LED[10];
        String[] ledsNames = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};
        for (int i = 0; i < 10; i++) {
            leds[i] = this.getLED(ledsNames[i]);
        }

        // Initialiser le prochain temps de rotation aléatoire (entre 50 et 100 cycles = 3-6 secondes)
        nextRotationTime = 50 + random.nextInt(50);
    }

    /**
     * The main method of the robot behaviour
     */
    public void run() {
        initLocalisation();
        initStartTime = this.getTime(); // Enregistre le temps de début de l'initialisation
        while (step(timeStep) != -1) {
            localise(false); // Mise à jour de la position relative (odométrie)

            // Lecture des capteurs
            double[] psValues = readDistanceSensorValues();
            List<CameraRecognitionObject> detected = cameraDetection();
            CameraRecognitionObject target = targetDetected(detected);
            List<CameraRecognitionObject> otherRobots = otherRobotsDetected(detected);

            // Gestion des messages
            String msg = checkMailBox();
            if (msg != null) handleMessage(msg); // Traite les messages entrants

            //  Évitement d'obstacle/collision
            if (isObstacle(psValues, otherRobots)) {
                avoidObstacle(psValues, otherRobots);
                continue;
            }

            //  Si cible atteinte par tous , s'arrêter
            if (currentState == State.CONVERGENCE && isTargetReached(target)) {
                stopRobot();
                continue;
            }

            //  Machine à états
            switch (currentState) {
                case INITIALIZATION:
                    handleInitialization(target);
                    break;
                case EXPLORATION:
                    handleExploration(target, otherRobots);
                    break;
                case ATTENTE:
                    handleAttente(target);
                    break;
                case CONVERGENCE:
                    handleConvergence(target);
                    break;
            }

            updateLEDs();
        }
    }

    /**
     *  Phase 1: Découverte des robots (compteur NbRobots)
     */
    private void handleInitialization(CameraRecognitionObject target) {
        // 1. Envoi uniquement au premier cycle
        if (!hasBroadcastedExist) {
            broadcastMessage("EXIST;");
            hasBroadcastedExist = true;
        }

        // Transition après 5 secondes
        if (this.getTime() > initStartTime + INIT_DURATION) {
            NbRobots = NbRobots + 1; // Le robot s'inclut lui-même
            currentState = State.EXPLORATION;
        }

        // Reste immobile pendant l'initialisation
        move(0, 0);
    }

    /**
     * Phase 2: Exploration et Coopération
     */
    private void handleExploration(CameraRecognitionObject target, List<CameraRecognitionObject> otherRobots) {
        // CAS 1 : Découverte de la cible
        if (target != null) {
            cibleTrouveeLocalement = true; // Cible trouvée
            broadcastMessage("FOUND;"); // Envoie le message "FOUND" à tous
            NbTrouves++; 
            currentState = State.ATTENTE; 
            move(0, 0); 
            return;
        }

        // CAS 2 : Détection d'un robot ayant trouvé la cible (LED allumée)
        CameraRecognitionObject foundRobot = detectFoundRobot(otherRobots);
        if (foundRobot != null) {
            // Se diriger vers le robot trouvé
            goTowardsObject(foundRobot, SPEED_FOLLOW);
            return;
        }

        // CAS 3 : Exploration avec rotation périodique
        doExplorationMovement();
    }

    /**
     * Phase 3: Attente et Synchronisation
     */
    private void handleAttente(CameraRecognitionObject target) {
        // Tous les robots ont trouvé la cible
        if (NbTrouves >= NbRobots) {
            currentState = State.CONVERGENCE; // on passe en mode convergence
            return;
        }

        // Reste immobile en attendant
        move(0, 0);
    }

    /**
     * Phase 4: Convergence vers la cible
     */
    private void handleConvergence(CameraRecognitionObject target) {
        if (target != null) {
            // Se diriger directement vers la cible
            goTowardsObject(target, SPEED_FOLLOW);
        } else {
            // Si la cible est perdue, faire un tour sur soi pour la retrouver
            move(ROTATION_SPEED, -ROTATION_SPEED);
        }
    }

    /**
     * Mouvement d'exploration avec rotation périodique basée sur l’angle (Algorithme de Scan 360°)
     */
    private void doExplorationMovement() {
        // 1) Si on est en train de faire une rotation de 360°
        if (isRotating) { 
            // Rotation sur place 
            move(ROTATION_SPEED, -ROTATION_SPEED);

            // Vérifier si on voit la cible pendant la rotation
            List<CameraRecognitionObject> detected = cameraDetection();
            CameraRecognitionObject target = targetDetected(detected);
            if (target != null) {
                cibleTrouveeLocalement = true;
                broadcastMessage("FOUND;");
                NbTrouves++;
                isRotating = false; // on stoppe la rotation
                currentState = State.ATTENTE;
                move(0, 0);
                return;
            }
            // Mise à jour de l'angle accumulé pour suivre le 360°// Mettre à jour l’angle accumulé grâce à l’odométrie
            double currentAngle = odometry.getTheta();
            double delta = smallestAngleDiff(currentAngle, lastAngleDuringTurn);
            accumulatedTurnAngle += Math.abs(delta);
            lastAngleDuringTurn = currentAngle;

            // Si on a tourné d’au moins 360° (2π radians), on arrête la rotation
            if (accumulatedTurnAngle >= 2.0 * Math.PI) {
                isRotating = false;
            }
            return;
        }

        // 2) Sinon, on est en mode exploration "normal"
        explorationCounter++;

        // Déclencher une rotation à intervalles aléatoires
        if (explorationCounter >= nextRotationTime) {
            // On démarre une rotation complète de 360°
            isRotating = true;
            accumulatedTurnAngle = 0.0;
            lastAngleDuringTurn = odometry.getTheta();
            explorationCounter = 0;
            // Définit un nouveau temps d'attente aléatoire (3-6 secondes)
            nextRotationTime = 50 + random.nextInt(50); 
            return;
        }

        // Mouvement par défaut : avancer
        move(SPEED_EXPLORE, SPEED_EXPLORE);
    }

    // Calcule la différence d’angle minimale entre deux orientations (en radians).
    private double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2.0 * Math.PI;
        while (diff < -Math.PI) diff += 2.0 * Math.PI;
        return diff;
    }

    /**
     * Dirige le robot vers un objet détecté (cible ou robot)
     */
    private void goTowardsObject(CameraRecognitionObject obj, double speed) {
        double[] p = obj.getPosition();
        double y_pos = p[1]; // Position latérale (gauche/droite)
        double forward = speed;
        // Correction de trajectoire basée sur la position latérale
        double turn = y_pos * 150.0;
        move(forward - turn, forward + turn);
    }

    /**
     * Détecte un robot ayant sa LED allumée (signal "cible trouvée")
     */
    private CameraRecognitionObject detectFoundRobot(List<CameraRecognitionObject> otherRobots) {
        for (CameraRecognitionObject robot : otherRobots) {
            if (isLightON(robot)) {
                return robot;
            }
        }
        return null;
    }

    /**
     * Gestion des messages reçus
     */
    protected void handleMessage(String msg) {
        if (currentState == State.INITIALIZATION && msg.startsWith("EXIST")) {
            NbRobots++;
        } else if (msg.startsWith("FOUND")) {
            NbTrouves++;
        }
    }

  
    private void updateLEDs() {
        setLED(0, currentState == State.INITIALIZATION);
        setLED(1, currentState == State.EXPLORATION);
        setLED(2, currentState == State.ATTENTE);
        setLED(3, currentState == State.CONVERGENCE);
        setLED(8, cibleTrouveeLocalement); // Signal pour les autres robots
    }

    /**
     * Gestion de la détection des obstacles
     */
    private boolean isObstacle(double[] psValues, List<CameraRecognitionObject> otherRobots) {
        // Obstacles détectés par LES capteurs IR autour du robot
        boolean frontObstacle = psValues[7] > OBSTACLE_SEUIL || psValues[0] > OBSTACLE_SEUIL || psValues[1] > OBSTACLE_SEUIL;
        boolean sideObstacle = psValues[2] > OBSTACLE_SEUIL || psValues[5] > OBSTACLE_SEUIL || psValues[6] > OBSTACLE_SEUIL;
        boolean backObstacle = psValues[3] > OBSTACLE_SEUIL || psValues[4] > OBSTACLE_SEUIL;
        boolean irObstacle = frontObstacle || sideObstacle || backObstacle;

        // Robots trop proches (devant)
        boolean robotTooClose = false;
        for (CameraRecognitionObject robot : otherRobots) {
            if (robot.getPosition()[0] < ROBOT_COLLISION_SEUIL) {
                robotTooClose = true;
                break;
            }
        }

        return irObstacle || robotTooClose;
    }

    /**
     *  Evitement les obstacles
     */
    private void avoidObstacle(double[] psValues, List<CameraRecognitionObject> otherRobots) {
        //  Si on était en train de faire une rotation d'exploration, on l'annule
        if (isRotating) {
            isRotating = false;
            accumulatedTurnAngle = 0.0;
            explorationCounter = 0;
            nextRotationTime = 50 + random.nextInt(50);
        }

        //  Évitement de robots 
        boolean robotLeftClose = false;
        boolean robotRightClose = false;
        for (CameraRecognitionObject robot : otherRobots) {
            double[] pos = robot.getPosition();
            double dist = pos[0]; // profondeur
            double y = pos[1]; // latéral (gauche /droite)
            if (dist < ROBOT_COLLISION_SEUIL) {
                if (y > 0) robotLeftClose = true;
                else robotRightClose = true;
            }
        }

        //  Coincé entre deux robots (un à gauche et un à droite) -> on recule
        if (robotLeftClose && robotRightClose) {
            move(-40, -40);
            return;
        }

        // Sinon, comportement normal : un robot d'un seul côté → on tourne
        if (robotLeftClose) {
            move(40, -40); // tourner à droite
            return;
        }
        if (robotRightClose) {
            move(-40, 40); // tourner à gauche
            return;
        }

        //  Lecture des capteurs murs (avant + arrière)
        boolean front = psValues[0] > OBSTACLE_SEUIL;
        boolean frontLeft = psValues[7] > OBSTACLE_SEUIL;
        boolean frontRight = psValues[1] > OBSTACLE_SEUIL;
        boolean backLeft = psValues[3] > OBSTACLE_SEUIL;
        boolean backRight = psValues[4] > OBSTACLE_SEUIL;

        // Cas "coin" : murs devant + à gauche + à droite
        if (front && frontLeft && frontRight) {
            //  Si quelque chose est aussi derrière -> on ne recule pas
            if (backLeft || backRight) {
                // Bloqué devant et derrière -> rotation sur place
                double leftBack = psValues[3];
                double rightBack = psValues[4];
                if (leftBack > rightBack) {
                    move(40, -40);
                } else {
                    move(-40, 40);
                }
            } else {
                // Derrière libre -> on recule en pivotant pour sortir du coin
                move(-30, 50); // reculer + tourner
            }
            return;
        }

        // Cass "classique" : mur plutôt à gauche / plutôt à droite
        double leftFront = psValues[7] + psValues[0];
        double rightFront = psValues[0] + psValues[1];

        if (leftFront > rightFront) {
            // mur plus proche à gauche -> tourner à droite
            move(40, -40);
        } else {
            // mur plus proche à droite -> tourner à gauche
            move(-40, 40);
        }
    }

    private boolean isTargetReached(CameraRecognitionObject target) {
        if (target == null) return false;
        double[] p = target.getPosition();
        double dx = p[0]; // profondeur
        double dy = p[1]; // latéral
        double dist = Math.sqrt(dx * dx + dy * dy);

        // le robot doit être  proche de la cible
        if (dist > TARGET_STOP_DISTANCE) {
            return false;
        }
        return true;
    }

    private void stopRobot() {
        move(0, 0);
        setLED(9, true); // LED 9 = arrivé à destination
    }

    
    private void initLocalisation() {
        step(timeStep);
        odometry.track_start_pos(encoder_unit * leftMotorSensor.getValue(), encoder_unit * rightMotorSensor.getValue());
        odometry.setX(0);
        odometry.setY(0);
        odometry.setTheta(Math.PI / 2);
    }

    protected void localise(boolean print) {
        odometry.track_step_pos(encoder_unit * leftMotorSensor.getValue(), encoder_unit * rightMotorSensor.getValue());
        if (print) {
            Double[] pos = getPosition();
            System.out.println("Position : " + pos[0] + "," + pos[1]);
        }
    }

    protected Double[] getPosition() {
        return new Double[]{odometry.getX(), odometry.getY()};
    }

    protected double[] readDistanceSensorValues() {
        double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 8; i++) psValues[i] = distanceSensor[i].getValue();
        return psValues;
    }

    protected void move(double left, double right) {
        double max = 6.2;
        getMotor("left wheel motor").setVelocity(left * max / 100);
        getMotor("right wheel motor").setVelocity(right * max / 100);
    }

    protected void setLED(int num, boolean on) {
        if (num < 10) {
            leds[num].set(on ? 1 : 0);
        }
    }

    protected List<CameraRecognitionObject> cameraDetection() {
        ArrayList<CameraRecognitionObject> detected = new ArrayList<>();
        int nb = camera.getRecognitionNumberOfObjects();
        if (nb > 0) {
            CameraRecognitionObject[] objects = camera.getRecognitionObjects();
            for (int i = 0; i < objects.length; i++) {
                detected.add(objects[i]);
            }
        }
        return detected;
    }

    protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
        for (CameraRecognitionObject ob : detected) {
            if (ob.getModel().compareTo("cible") == 0) return ob;
        }
        return null;
    }

    protected List<CameraRecognitionObject> otherRobotsDetected(List<CameraRecognitionObject> detected) {
        ArrayList<CameraRecognitionObject> robots = new ArrayList<>();
        for (CameraRecognitionObject ob : detected) {
            if (ob.getModel().compareTo("e-puck") == 0) robots.add(ob);
        }
        return robots;
    }

    private boolean isLightON(CameraRecognitionObject robot) {
        int[] image = camera.getImage();
        boolean detected = false;
        int[] position = robot.getPositionOnImage();
        int width = robot.getSizeOnImage()[0];
        int height = robot.getSizeOnImage()[1];
        int startx = position[0] - (width + 1) / 2;
        int starty = position[1] - (height + 1) / 2;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                int pixel = image[(startx + i) + (camera.getWidth() * (starty + j))];
                if (Camera.pixelGetRed(pixel) >= 254 && Camera.pixelGetGreen(pixel) >= 254 && Camera.pixelGetBlue(pixel) < 200) {
                    if (detected) return true;
                    else detected = true;
                }
            }
        }
        return false;
    }

    protected void broadcastMessage(String message) {
        emitter.send(message.getBytes());
    }

    protected String checkMailBox() {
        while (receiver.getQueueLength() > 0) {
            byte[] message = receiver.getData();
            receiver.nextPacket();
            if (message != null) {
                return new String(message);
            } else return null;
        }
        return null;
    }

    public static void main(String[] args) {
        AutonomyThree controller = new AutonomyThree();
        controller.run();
    }

    /**
     * Do NOT modify
     * Private class providing tools to compute a relative position for the robot
     */
    private class Odometry {
        private double wheel_distance;
        private double wheel_conversion_left;
        private double wheel_conversion_right;
        private double pos_left_prev;
        private double pos_right_prev;
        private double x;
        private double y;
        private double theta;
        private double increments_per_tour = 1000.0;
        private double axis_wheel_ratio = 1.4134;
        private double wheel_diameter_left = 0.0416;
        private double wheel_diameter_right = 0.0416;
        private double scaling_factor = 0.976;

        public Odometry() {}

        public int track_start_pos(double pos_left, double pos_right) {
            x = 0;
            y = 0;
            theta = 0;
            pos_left_prev = pos_left;
            pos_right_prev = pos_right;
            wheel_distance = axis_wheel_ratio * scaling_factor * (wheel_diameter_left + wheel_diameter_right) / 2;
            wheel_conversion_left = wheel_diameter_left * scaling_factor * Math.PI / increments_per_tour;
            wheel_conversion_right = wheel_diameter_right * scaling_factor * Math.PI / increments_per_tour;
            return 1;
        }

        public void track_step_pos(double pos_left, double pos_right) {
            double delta_pos_left, delta_pos_right;
            double delta_left, delta_right, delta_theta, theta2;
            double delta_x, delta_y;

            delta_pos_left = pos_left - pos_left_prev;
            delta_pos_right = pos_right - pos_right_prev;

            delta_left = delta_pos_left * wheel_conversion_left;
            delta_right = delta_pos_right * wheel_conversion_right;

            delta_theta = (delta_right - delta_left) / wheel_distance;
            theta2 = theta + delta_theta * 0.5;

            delta_x = (delta_left + delta_right) * 0.5 * Math.cos(theta2);
            delta_y = (delta_left + delta_right) * 0.5 * Math.sin(theta2);

            x += delta_x;
            y += delta_y;
            theta += delta_theta;

            if (theta < 0) theta += 2 * Math.PI;
            if (theta > 2 * Math.PI) theta -= 2 * Math.PI;

            pos_left_prev = pos_left;
            pos_right_prev = pos_right;
        }

        public double getX() {
            return x;
        }

        public void setX(double x) {
            this.x = x;
        }

        public double getY() {
            return y;
        }

        public void setY(double y) {
            this.y = y;
        }

        public double getTheta() {
            return theta;
        }

        public void setTheta(double theta) {
            this.theta = theta;
        }
    }
}
