using fairino;
using System;
using System.Threading;
using System.Data;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using RMLib.Logger;
using RMLib.DataAccess;
using RMLib.PLC;
using RMLib.Alarms;
using RM.src.RM250714.Forms.Plant;
using RM.Properties;
using RMLib.MessageBox;
using RM.src.RM250714.Forms.DragMode;
using RM.src.RM250714.Classes.PLC;
using RM.src.RM250714.Classes.FR20.Jog;
using RM.src.RM250714.Classes.FR20;
using RM.src.RM250714.Classes.FR20.Properties;
using CookComputing.XmlRpc;
using System.IO;

namespace RM.src.RM250714
{
    /// <summary>
    /// Gestisce il robot in tutte le sue funzioni, la classe contiene solo riferimenti statici poichè il robot è unico 
    /// nell'impianto. Nel caso se ne dovessero aggiungere dei nuovi bisognerà rifare la classe in modo che ci sia un array
    /// di Robot e i metodi per accedere alle funzioni di un singolo robot alla volta. 
    /// <br>Il robot restituisce come feedback per ogni metodo interno alla sua libreria un codice di errore che può essere
    /// controllato al fine di gestire la pagina degli allarmi.</br>
    /// <br>Il robot apparentemente si muove di pochi mm perciò non sta mai del tutto fermo, per fare il controllo sul movimento
    /// è necessario aggiungere degli offset.</br>
    /// <br>La libreria fairino presenta problemi a gestire la sincronizzazione tra comando ed esecuzione, per questo motivo 
    /// è difficile sapere a quale posizione il robot si sta muovendo. Inoltre sembra che a volte il robot non si fermi subito 
    /// al comando Stop, proprio per via della coda di istruzioni inviate.</br>
    /// </summary>
    public class RobotManager
    {
        #region Campi Statici e Proprietà

        #region Componenti Principali e Connessioni

        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();
        /// <summary>
        /// Oggetto per l'accesso ai dati del robot nel database.
        /// </summary>
        private static readonly RobotDAOSqlite RobotDAO = new RobotDAOSqlite();
        /// <summary>
        /// Configurazione della connessione al database.
        /// </summary>
        private static readonly SqliteConnectionConfiguration DatabaseConnection = new SqliteConnectionConfiguration();
        /// <summary>
        /// Stringa di connessione al database.
        /// </summary>
        private static readonly string ConnectionString = DatabaseConnection.GetConnectionString();
        /// <summary>
        /// Oggetto di lock per garantire l'accesso thread-safe all'istanza del robot.
        /// </summary>
        private static readonly object _robotInstanceLock = new object();
        /// <summary>
        /// Campo privato contenente l'istanza del Robot dalla libreria fairino.
        /// </summary>
        private static Robot _robot;
        /// <summary>
        /// Proprietà pubblica e thread-safe per accedere all'istanza del Robot.
        /// </summary>
        public static Robot robot
        {
            get { lock (_robotInstanceLock) { return _robot; } }
            private set { lock (_robotInstanceLock) { _robot = value; } }
        }
        /// <summary>
        /// Gestisce i task in background.
        /// </summary>
        public readonly static TaskManager taskManager;
        /// <summary>
        /// IP statico assegnato al robot. Per modificarlo si deve usare il pannello dedicato.
        /// </summary>
        public static string RobotIpAddress = "192.168.2.70";

        #endregion

        #region Stato e Parametri del Robot

        /// <summary>
        /// Errore che restituisce il Robot.
        /// </summary>
        public static int err = 0;
        /// <summary>
        /// Codice principale errore Robot.
        /// </summary>
        public static int maincode = 0;
        /// <summary>
        /// Codice specifico errore Robot.
        /// </summary>
        public static int subcode = 0;
        /// <summary>
        /// Applicazione da far eseguire al Robot.
        /// </summary>
        public static string applicationName;
        /// <summary>
        /// Indica se la modalità del robot è al momento in automatica (true) o manuale (false).
        /// </summary>
        public static bool isAutomaticMode;
        /// <summary>
        /// Percentuale di velocità.
        /// </summary>
        public static int speed = 0;
        /// <summary>
        /// ID dello strumento in uso dal robot.
        /// </summary>
        public static int tool = 0;
        /// <summary>
        /// Utente che sta usando il robot.
        /// </summary>
        public static int user = 0;
        /// <summary>
        /// Carico massimo del robot in kg.
        /// </summary>
        public static int weight = 0;
        /// <summary>
        /// Percentuale di velocità.
        /// </summary>
        public static float vel = 0;
        /// <summary>
        /// Percentuale di accelerazione.
        /// </summary>
        public static float acc = 0;
        /// <summary>
        /// Fattore di scalatura di velocità.
        /// </summary>
        public static float ovl = 0;
        /// <summary>
        /// Valore che rappresenta la smoothness dei movimenti del robot (blend).
        /// </summary>
        public static float blendT = 0;
        /// <summary>
        /// Valore che rappresenta la smoothness dei movimenti lineari del robot (blend).
        /// </summary>
        public static float blendR = 0;
        /// <summary>
        /// Configurazione dello spazio giunto.
        /// </summary>
        public static int config = -1;
        /// <summary>
        /// Flag -> 0: blocking, 1: non_blocking.
        /// </summary>
        public static byte flag = 0;
        /// <summary>
        /// Estensione area di lavoro Robot.
        /// </summary>
        public static ExaxisPos ePos = new ExaxisPos(0, 0, 0, 0);
        /// <summary>
        /// Offset di posizione.
        /// </summary>
        public static DescPose offset = new DescPose();
        /// <summary>
        /// Frequenza registrazione punti in DragMode.
        /// </summary>
        public static int velRec = 500;
        /// <summary>
        /// Proprietà speed del Robot.
        /// </summary>
        public static int speedRobot = 30;
        /// <summary>
        /// Posizione TCP attuale del robot.
        /// </summary>
        public static DescPose TCPCurrentPosition = new DescPose(0, 0, 0, 0, 0, 0);
        /// <summary>
        /// Punto corrente precedente del Robot.
        /// </summary>
        public static DescPose previousTCPposition = new DescPose(0, 0, 0, 0, 0, 0);
        /// <summary>
        /// Raccoglie le proprietà del robot in un oggetto.
        /// </summary>
        public static RobotProperties robotProperties;
        /// <summary>
        /// Modalità operativa corrente.
        /// </summary>
        public static int mode = -1;
        /// <summary>
        /// Livello di collisione corrente.
        /// </summary>
        public static int currentCollisionLevel = 0;
        /// <summary>
        /// Tempo massimo in ms per controllare che il proxy stia comunicando
        /// </summary>
        private const int connectionCheckMaxTimeout = 500;

        #endregion

        #region Gestori di Componenti e Form

        /// <summary>
        /// Riferimento alla pagina degli allarmi.
        /// </summary>
        public static FormAlarmPage formAlarmPage;
        /// <summary>
        /// Istanza form di diagnostica.
        /// </summary>
        public static FormDiagnostics formDiagnostics;
        /// <summary>
        /// Gestore dei frame del robot.
        /// </summary>
        private static Frames frameManager;
        /// <summary>
        /// Gestore dei tool del robot.
        /// </summary>
        private static Tools toolManager;
        /// <summary>
        /// Gestore delle collisioni del robot.
        /// </summary>
        private static Collisions collisionManager;

        #endregion

        #region Variabili di Stato per la Logica di Controllo

        // --- Stato connessione e allarmi ---
        /// <summary>
        /// Flag che indica un errore di connessione con il robot.
        /// </summary>
        private static readonly bool Connection_Robot_Error = false;
        /// <summary>
        /// Dizionario di allarmi per evitare segnalazioni duplicate.
        /// </summary>
        private static readonly Dictionary<string, bool> allarmiSegnalati = new Dictionary<string, bool>();
        /// <summary>
        /// Rappresenta lo stato precedente della connessione al PLC.
        /// </summary>
        private static bool prevIsPlcConnected = true;

        // --- Stato movimento e posizione ---
        /// <summary>
        /// A true quando il punto corrente del Robot si trova nel punto endingPoint passato come parametro.
        /// </summary>
        public static bool inPosition = false;
        /// <summary>
        /// Parametro da usare per eseguire inPosition.
        /// </summary>
        public static DescPose endingPoint = new DescPose(0, 0, 0, 0, 0, 0);
        /// <summary>
        /// Parametro da usare per eseguire in position sulla pose del tracker calcolata.
        /// </summary>
        public static DescPose trackerEndingPoint = new DescPose(0, 0, 0, 0, 0, 0);
        /// <summary>
        /// A true quando il robot si trova in safe zone.
        /// </summary>
        public static bool isInSafeZone = false;
        /// <summary>
        /// A true quando il robot si trova in home zone.
        /// </summary>
        public static bool isInHomePosition = false;
        /// <summary>
        /// Stato precedente di isInSafeZone per rilevare i cambiamenti.
        /// </summary>
        private static bool? prevIsInSafeZone = null;
        /// <summary>
        /// Variabile per memorizzare lo stato precedente di isInHomePosition.
        /// </summary>
        private static bool? previousIsInHomePosition = null;
        /// <summary>
        /// Timestamp di quando il robot ha iniziato a muoversi, per logica di debounce.
        /// </summary>
        static DateTime? robotMovingStartTime = null;

        // --- Stato ciclo applicazione ---
        /// <summary>
        /// A true quando viene terminata la routine del ciclo.
        /// </summary>
        public static bool stopCycleRoutine = false;
        /// <summary>
        /// A true quando si richiede lo stop del ciclo del Robot.
        /// </summary>
        public static bool stopCycleRequested = false;
        /// <summary>
        /// A true quando viene richiesta una pausa del ciclo.
        /// </summary>
        public static bool pauseCycleRequested = false;
        /// <summary>
        /// A true quando il ciclo deve riprendere da un punto precedente.
        /// </summary>
        public static bool riprendiCiclo = false;
        /// <summary>
        /// Indica l'indice del punto corrente nel ciclo.
        /// </summary>
        public static int currentIndex = -1;

        // --- Stato catena ---
        /// <summary>
        /// Contatore spostamento catena.
        /// </summary>
        public static int chainPos = 0;
        /// <summary>
        /// A true quando bisogna fermare l'updater del contatore catena.
        /// </summary>
        public static bool stopChainUpdaterThread = false;

        // --- Stato Abilitazione e Modalità ---
        /// <summary>
        /// Stato attuale isEnabled del robot.
        /// </summary>
        public static bool isEnabledNow = false;
        /// <summary>
        /// Stato precedente isEnable.
        /// </summary>
        private static bool prevIsEnable = false;
        /// <summary>
        /// Stato precedente isNotEnable.
        /// </summary>
        private static bool prevIsNotEnable = false;
        /// <summary>
        /// Stato precedente di robotReadyToStart.
        /// </summary>
        private static bool prevRobotReadyToStart = false;
        /// <summary>
        /// Stato precedente di robotHasProgramInMemory.
        /// </summary>
        private static bool prevRobotHasProgramInMemory = false;
        /// <summary>
        /// Rappresenta il valore della modalità automatica nello step precedente.
        /// </summary>
        private static bool prevIsAuto = false;
        /// <summary>
        /// Rappresenta il valore della modalità manuale nello step precedente.
        /// </summary>
        private static bool prevIsManual = false;
        /// <summary>
        /// Rappresenta il valore della modalità Off nello step precedente.
        /// </summary>
        private static bool prevIsOff = false;
        /// <summary>
        /// Modalità precedente letta dal PLC.
        /// </summary>
        private static int lastMode = -1;
        /// <summary>
        /// Timestamp dell'ultima modifica di modalità.
        /// </summary>
        private static DateTime lastModeChangeTime;
        /// <summary>
        /// Modalità stabile da impostare dopo il debounce.
        /// </summary>
        private static int stableMode = -1;
        /// <summary>
        /// Ultimo errore del robot
        /// </summary>
        public static int robotError = 0;
        /// <summary>
        /// Stato del robot
        /// </summary>
        public static int robotStatus = 0;
        /// <summary>
        /// Modalità corrente letta dal robot 0:auto, 1:manual
        /// </summary>
        public static int currentRobotMode = -1;
        /// <summary>
        /// Indica se il robot è al momento enabled: 1, disabled: 0
        /// </summary>
        public static int currentRobotEnableStatus = -1;
        /// <summary>
        /// Tool usato al momento
        /// </summary>
        public static int currentTool = 0;
        /// <summary>
        /// Frame impostato al momento
        /// </summary>
        public static int currentUser = 0;
        /// <summary>
        /// Stato enable del robot
        /// </summary>
        public static int robotEnableStatus = 0;
        /// <summary>
        /// Speed utilizzata in home routine
        /// </summary>
        private static readonly int homeRoutineSpeed = 2;
        /// <summary>
        /// Velocity utilizzata in home routine
        /// </summary>
        private static readonly int homeRoutineVel = 100;
        /// <summary>
        /// Acceleration utilizzata in home routine
        /// </summary>
        private static readonly int homeRoutineAcc = 100;

        // --- Drag & Teach e Varie ---
        /// <summary>
        /// Lista di punti da visualizzare.
        /// </summary>
        public static List<PointPosition> positionsToSend = new List<PointPosition>();
        /// <summary>
        /// Punto da aggiungere alla lista di posizioni.
        /// </summary>
        public static PointPosition positionToSend = new PointPosition();
        /// <summary>
        /// Lista di punti da salvare.
        /// </summary>
        public static List<PointPosition> positionsToSave = new List<PointPosition>();
        /// <summary>
        /// Flag per avviare/fermare il recupero della posizione del tracker.
        /// </summary>
        public static bool getTrackerPosition = false;
        /// <summary>
        /// A true quando non si vuole eseguire retrieve delle coordinate dal tracker.
        /// </summary>
        public static bool stopFetchTrackerData = false;
        /// <summary>
        /// Callback per il timer di teaching.
        /// </summary>
        private static readonly TimerCallback timerCallback;
        /// <summary>
        /// Timer usato per il teaching lineare.
        /// </summary>
        private static readonly Timer timer;

        // --- Position Checker ---
        /// <summary>
        /// Oggetto usato per eseguire inPosition dei punti.
        /// </summary>
        private static PositionChecker checker_pos;
        /// <summary>
        /// Oggetto usato per controllare che un punto sia nella safeZone.
        /// </summary>
        private static PositionChecker checker_safeZone;
        /// <summary>
        /// Oggetto usato per controllare che la pose del tracker calcolata sia conforme con quella del robot pura.
        /// </summary>
        private readonly static PositionChecker checker_tracker = new PositionChecker(50.0);
        /// <summary>
        /// Checker utilizzato per la colorazione delle righe all'interna di lw_positions.
        /// </summary>
        private static readonly PositionChecker checker_monitoringPos = new PositionChecker(100.0);
        /// <summary>
        /// Checker per zona di pick
        /// </summary>
        private static PositionChecker checker_ingombro_pick;
        /// <summary>
        /// Checker per zona di place
        /// </summary>
        private static PositionChecker checker_ingombro_place;
        /// <summary>
        /// Checker per zona ingombro home
        /// </summary>
        private static PositionChecker checker_ingombro_home;

        #endregion

        #region Tempi di Delay dei Task

        /// <summary>
        /// Periodo di refresh per il task ad alta priorità.
        /// </summary>
        private readonly static int highPriorityRefreshPeriod = 20;
        /// <summary>
        /// Periodo di refresh per il task degli ausiliari.
        /// </summary>
        private readonly static int auxiliaryThreadRefreshPeriod = 200;
        /// <summary>
        /// Periodo di refresh per il task a bassa priorità.
        /// </summary>
        private readonly static int lowPriorityRefreshPeriod = 200;
        /// <summary>
        /// Periodo di refresh per il task che comunica al plc
        /// </summary>
        private readonly static int plcComTaskRefreshPeriod = 600;
        /// <summary>
        /// Periodo di refresh per il task che verifica la connessione al robot
        /// </summary>
        private readonly static int robotComTaskRefreshPeriod = 500;
        /// <summary>
        /// Periodo di refresh all'interno del metodo ApplicationTaskManager
        /// </summary>
        private readonly static int applicationTaskManagerRefreshPeriod = 100;
        /// <summary>
        /// Periodo di refresh all'interno del metodo SafetyYaskManager
        /// </summary>
        private readonly static int safetyTaskManagerRefreshPeriod = 100;

        #endregion

        #region Parametri ciclo

        /// <summary>
        /// Riga della matrice di carico del pallet
        /// </summary>
        public static int riga = 0;
        /// <summary>
        /// Colonna della matrice di carico del pallet
        /// </summary>
        public static int colonna = 0;
        /// <summary>
        /// Strato della matrice di carico del pallet
        /// </summary>
        public static int strato = 0;

        /// <summary>
        /// Parametro larghezza della focaccia da HMI
        /// </summary>
        public static int larghezzaScatola = 300;
        /// <summary>
        /// Parametro profondità della focaccia da HMI
        /// </summary>
        public static int lunghezzaScatola = 300;
        /// <summary>
        /// Altezza del pallet da HMI
        /// </summary>
        public static int altezzaScatola = 100;

        /// <summary>
        /// Larghezza del pallet da HMI
        /// </summary>
        public static int larghezzaPallet = 800;
        /// <summary>
        ///  Lunghezza del pallet da HMI
        /// </summary>
        public static int lunghezzaPallet = 1200;
        /// <summary>
        /// Altezza del pallet da HMI
        /// </summary>
        public static int altezzaPallet = 100;

        #endregion

        #region Stati precedenti dei comandi

        /// <summary>
        /// Memorizza lo stato precedente della variabile open/close grippers dal PLC
        /// </summary>
        private static bool previousGripperStatus = false;
        /// <summary>
        /// Memorizza lo stato precedente della variabile on/off barrier status dal PLC
        /// </summary>
        private static bool previousBarrierStatus = false;
        /// <summary>
        /// Memorizza lo stato precedente della variabile start ciclo dal PLC
        /// </summary>
        private static bool previousStartCommandStatus = false;
        /// <summary>
        /// Memorizza lo stato precedente della variabile stop ciclo dal PLC
        /// </summary>
        private static int previousStopCommandStatus = 0;
        /// <summary>
        /// Memorizza lo stato precedente della variabile go to home position dal PLC
        /// </summary>
        private static bool previousHomeCommandStatus = false;
        /// <summary>
        /// Memorizza lo stato della variabile selected pallet dal PLC
        /// </summary>
        private static int previousPalletCommandNumber = 0;
        /// <summary>
        /// Memorizza lo stato della variabile selected pallet dal PLC
        /// </summary>
        private static int previousFormatCommandNumber = 0;
        /// <summary>
        /// Memorizza lo stato della variabile selected box format dal PLC
        /// </summary>
        private static int previousBoxFormatCommandNumber = 0;
        /// <summary>
        /// Memorizza lo stato precedente del formato selezionato
        /// </summary>
        private static int previousSelectedFormat = 0;
        /// <summary>
        /// Memorizza lo stato precedente della variabile on/off jog nastro dal PLC
        /// </summary>
        private static bool previousJogNastroCommandStatus = false;

        #endregion

        #region Variabili Versioni Robot

        /// <summary>
        /// Versione SDK del controllore.
        /// </summary>
        public static string RobotSdkVer = "##########";
        /// <summary>
        /// Modello del robot.
        /// </summary>
        public static string RobotModelVer = "##########";
        /// <summary>
        /// Versione web.
        /// </summary>
        public static string RobotWebVer = "##########";
        /// <summary>
        /// Versione controller.
        /// </summary>
        public static string RobotControllerVer = "##########";
        /// <summary>
        /// Versione del firmware della control box.
        /// </summary>
        public static string RobotFwBoxBoardVer = "##########";
        /// <summary>
        /// Versione firmware driver 1.
        /// </summary>
        public static string RobotFwDriver1Ver = "##########";
        /// <summary>
        /// Versione firmware driver 2.
        /// </summary>
        public static string RobotFwDriver2Ver = "##########";
        /// <summary>
        /// Versione firmware driver 3.
        /// </summary>
        public static string RobotFwDriver3Ver = "##########";
        /// <summary>
        /// Versione firmware driver 4.
        /// </summary>
        public static string RobotFwDriver4Ver = "##########";
        /// <summary>
        /// Versione firmware driver 5.
        /// </summary>
        public static string RobotFwDriver5Ver = "##########";
        /// <summary>
        /// Versione firmware driver 6.
        /// </summary>
        public static string RobotFwDriver6Ver = "##########";
        /// <summary>
        /// Versione firmware della scheda end-effector.
        /// </summary>
        public static string RobotFwEndBoardVer = "##########";
        /// <summary>
        /// Versione hardware della control box.
        /// </summary>
        public static string RobotHwBoxBoardVer = "##########";
        /// <summary>
        /// Versione hardware driver 1.
        /// </summary>
        public static string RobotHwDriver1Ver = "##########";
        /// <summary>
        /// Versione hardware driver 2.
        /// </summary>
        public static string RobotHwDriver2Ver = "##########";
        /// <summary>
        /// Versione hardware driver 3.
        /// </summary>
        public static string RobotHwDriver3Ver = "##########";
        /// <summary>
        /// Versione hardware driver 4.
        /// </summary>
        public static string RobotHwDriver4Ver = "##########";
        /// <summary>
        /// Versione hardware driver 5.
        /// </summary>
        public static string RobotHwDriver5Ver = "##########";
        /// <summary>
        /// Versione hardware driver 6.
        /// </summary>
        public static string RobotHwDriver6Ver = "##########";
        /// <summary>
        /// Versione hardware della scheda end-effector.
        /// </summary>
        public static string RobotHwEndBoardVer = "##########";
        /// <summary>
        /// IP corrente del controllore.
        /// </summary>
        public static string RobotCurrentIP = "##########";

        #endregion

        #region Eventi Pubblici

        /// <summary>
        /// Evento invocato quando viene generato un allarme.
        /// </summary>
        public static event EventHandler AllarmeGenerato;
        /// <summary>
        /// Evento invocato quando gli allarmi vengono resettati.
        /// </summary>
        public static event EventHandler AllarmeResettato;
        /// <summary>
        /// Evento invocato al termine della routine per riabilitare i tasti per riavvio della routine.
        /// </summary>
        public static event EventHandler CycleRoutineStarted;
        /// <summary>
        /// Evento invocato dalla rpoutine go to home position
        /// </summary>
        public static event EventHandler HomeRoutineStarted;
        /// <summary>
        /// Evento invocato per riabilitare i tasti della modalità Drag.
        /// </summary>
        public static event EventHandler EnableDragModeButtons;
        /// <summary>
        /// Viene invocato quando si modifica la velocità del Robot.
        /// </summary>
        public static event EventHandler RobotVelocityChanged;
        /// <summary>
        /// Viene invocato quando si modifica la modalità del Robot.
        /// </summary>
        public static event EventHandler RobotModeChanged;
        /// <summary>
        /// Viene invocato quando si rileva che il robot si sta muovendo.
        /// </summary>
        public static event EventHandler RobotIsMoving;
        /// <summary>
        /// Evento scatenato quando il robot cambia posizione (per la colorazione della UI).
        /// </summary>
        public static event EventHandler RobotPositionChanged;
        /// <summary>
        /// Evento scatenato quando riparte il ciclo.
        /// </summary>
        public static event EventHandler CycleRestarted;
        /// <summary>
        /// Evento scatenato quando viene aggiunto un punto in modalità Drag.
        /// </summary>
        public static event EventHandler PointPositionAdded;
        /// <summary>
        /// Evento scatenato quando viene richiesto lo start della teach mode
        /// </summary>
        public static event EventHandler RequestedStartTeach;
        /// <summary>
        /// Evento scatenato quando viene richiesto lo stop della teach mode
        /// </summary>
        public static event EventHandler RequestedStopTeach;
        /// <summary>
        /// Evento scatenato quando il robot arriva in home
        /// </summary>
        public static event EventHandler RobotInHomePosition;
        /// <summary>
        /// Evento scatenato quando il robot esce dalla home position
        /// </summary>
        public static event EventHandler RobotNotInHomePosition;
        /// <summary>
        /// Evento scatenato quando le pinze si chiudono
        /// </summary>
        public static event EventHandler GripperStatusON;
        /// <summary>
        /// Evento scatenato quando le pinze si aprono
        /// </summary>
        public static event EventHandler GripperStatusOFF;
        /// <summary>
        /// Evento invocato per disattivare/attivare il pulsante di go to home position in honme page
        /// </summary>
        public static event EventHandler EnableButtonHome;
        /// <summary>
        /// Evento invocato per disattivare/attivare i pulsanti di start e stop ciclo in home page
        /// </summary>
        public static event EventHandler EnableCycleButtons;
        /// <summary>
        /// Evento invocato al termine della routine per riabilitare i tasti per riavvio della routine
        /// </summary>
        public static event EventHandler EnableButtonCycleEvent;
        /// <summary>
        /// Evento invocato quanto dall'hmi viene rischiesto di registrare uno specifico punto
        /// </summary>
        public static event EventHandler<RobotPointRecordingEventArgs> RecordPoint;

        #endregion

        #region Metodi wrapper robot

        /// <summary>
        /// Metodo che mette in pausa il Robot
        /// </summary>
        public static int PauseMotion()
        {
            int err = robot.PauseMotion();
            if (err != 0)
            {
                log.Error("pause err: " + err);
            }
            else
            {
                log.Info("pause err: " + err);
            }
            return err;
        }

        /// <summary>
        /// Metodo che riprende movimento Robot
        /// </summary>
        public static int ResumeMotion()
        {
            int err = robot.ResumeMotion();
            if (err != 0)
            {
                log.Error("resume err: " + err);
            }
            else
            {
                log.Info("resume err: " + err);
            }
            return err;
        }

        /// <summary>
        /// Metodo che mette in stop il Robot
        /// </summary>
        public static int StopMotion()
        {
            int err = robot.StopMotion();
            if (err != 0)
            {
                log.Error("stop err: " + err);
            }
            else
            {
                log.Info("stop err: " + err);
            }
            return err;
        }

        /// <summary>
        /// Abilita o disabilita il Robot
        /// </summary>
        /// <param name="enableFlag"></param>
        private static int EnableRobot(byte enableFlag)
        {
            int err = robot.RobotEnable(enableFlag);
            if (err != 0)
            {
                if (enableFlag == 1)
                    log.Error("enable robot err: " + err);
                else
                    log.Error("disable robot err: " + err);
            }
            else
            {
                if (enableFlag == 1)
                    log.Info("enable robot err: " + err);
                else
                    log.Info("disable robot err: " + err);
            }
            return err;
        }

        /// <summary>
        /// Imposta la modalità operativa del robot: 
        /// <para>0 = automatico</para>
        /// <para>1 = manuale</para>
        /// </summary>
        /// <param name="mode"></param>
        public static int SetRobotMode(int mode)
        {
            if (mode != 0 && mode != 1)
                return -1;

            int err = robot.Mode(mode);
            isAutomaticMode = mode == 0;

            if (err != 0)
            {
                log.Error("Errore durante cambio modalità robot a " + mode + " : Codice errore " + err);
            }
            else
            {
                log.Info("Cambio modalità robot a " + mode + " completato");
            }
            return err;
        }

        /// <summary>
        /// Imposta la velocità di movimento del robot in percentuale
        /// </summary>
        /// <param name="speedPerc"></param>
        /// <returns></returns>
        public static bool SetRobotSpeed(int speedPerc)
        {
            int errSpeed = robot.SetSpeed(speedPerc);
            if (errSpeed != 0)
            {
                log.Error("Errore durante update robot speed : " + errSpeed);
                GenerateAlarm(0, 4);
                return false;
            }
            log.Info("[Speed] velocita robot impostata a : " + speedPerc);
            return true;
        }

        /// <summary>
        /// Imposta il payload
        /// </summary>
        /// <param name="payload"></param>
        /// <returns></returns>
        public static bool SetRobotPayload(int payload)
        {
            int errPayload = robot.SetLoadWeight(0, payload);
            if (errPayload != 0)
            {
                log.Error("Errore durante update robot payload : " + errPayload);
                GenerateAlarm(0, 4);
                return false;
            }
            log.Info("[Payload] Peso robot impostato a : " + payload);
            return true;
        }

        /// <summary>
        /// Imposta le proprietà principali del robot: speed-frame-tool-collision levels
        /// </summary>
        /// <returns></returns>
        private static bool SetRobotProperties()
        {
            if (!SetRobotSpeed(robotProperties.Speed))
                return false;

            if (!toolManager.ChangeRobotTool(tool))
                return false;

            if (!frameManager.ChangeRobotFrame(user))
                return false;

            if (!collisionManager.ChangeRobotCollision(currentCollisionLevel))
                return false;

            if (!SetRobotPayload(robotProperties.Weight))
                return false;

            return true;
        }

        /// <summary>
        /// Metodo per reset errori Robot
        /// </summary>
        public static int ClearRobotAlarm()
        {
            int err = robot.ResetAllError();
            if (err != 0)
            {
                log.Error("reset robot alarms err: " + err);
            }
            else
            {
                log.Info("reset robot alarms err: " + err);
            }
            return err;
        }

        /// <summary>
        /// Invia comando al robot per calcolare la cinematica inversa e ottenere la posizione dei giunti a partire dalle coordinate.
        /// Lancia una eccezione se si verifica un errore.
        /// </summary>
        /// <param name="pose"></param>
        /// <param name="jPos"></param>
        /// <param name="type"></param>
        /// <param name="config"></param>
        /// <param name="pointName"></param>
        public static int GetInverseKin(DescPose pose, ref JointPos jPos, string pointName, int type = 0, int config = -1)
        {
            int err = robot.GetInverseKin(type, pose, config, ref jPos);

            if (err != 0)
            {
                log.Error("Errore durante calcolo cinematica inversa : " + err);
                if (!string.IsNullOrEmpty(pointName))
                {
                    throw new Exception("Errore durante calcolo cinematica inversa per il punto " + pointName + " : Codice errore " + err);
                }
                else
                {
                    throw new Exception("Errore durante calcolo cinematica inversa per il punto con codice errore " + err);
                }
            }

            return err;
        }
        /// <summary>
        /// Invia comando al robot per calcolare la cinematica inversa 
        /// </summary>
        /// <param name="jPos"></param>
        /// <param name="pose"></param>
        public static int GetForwardKin(JointPos jPos, ref DescPose pose)
        {
            int err = robot.GetForwardKin(jPos, ref pose);

            if (err != 0)
            {
                log.Error("Errore durante calcolo cinematica diretta : " + err);
            }

            return err;
        }

        /// <summary>
        /// Ottiene lo stato corrente del robot sotto forma di struct
        /// </summary>
        /// <param name="robot_state_pkg"></param>
        /// <returns></returns>
        public static int GetRobotRealTimeState(ref ROBOT_STATE_PKG robot_state_pkg)
        {
            int err = robot.GetRobotRealTimeState(ref robot_state_pkg);

            if (err != 0)
            {
                log.Error("Errore durante get status robot : " + err);
            }

            return err;
        }

        public static int MoveL(JointPos jPos, DescPose pose, int tool, int user, float vel, float acc, float ovl,
            float blendR, ExaxisPos epos, byte search, byte offsetFlag, DescPose offsetPos, int overSpeedStrategy = 0, int speedPercent = 10)
        {
            int err = robot.MoveL(jPos, pose, tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offsetPos, overSpeedStrategy, speedPercent);
            if (err != 0)
            {
                GetRobotMovementCode(err);
            }
            return err;
        }

        public static int MoveL(JointPos jPos, DescPose pose, int tool, int user, float vel, float acc, float ovl,
            float blendR, int blendMode, ExaxisPos epos, byte search, byte offsetFlag, DescPose offsetPos, int overSpeedStrategy = 0, int speedPercent = 10)
        {
            int err = robot.MoveL(jPos, pose, tool, user, vel, acc, ovl, blendR, blendMode, epos, search, offsetFlag, offsetPos, overSpeedStrategy, speedPercent);
            if (err != 0)
            {
                GetRobotMovementCode(err);
            }
            return err;
        }

        public static int MoveJ(JointPos jPos, DescPose pose, int tool, int user, float vel, float acc, float ovl,
           ExaxisPos epos, float blendT, byte offsetFlag, DescPose offsetPos)
        {
            int err = robot.MoveJ(jPos, pose, tool, user, vel, acc, ovl, epos, blendT, offsetFlag, offsetPos);
            if (err != 0)
            {
                GetRobotMovementCode(err);
            }
            return err;
        }

        public static int StartJOG(byte refType, byte nb, byte dir, float vel, float acc, float maxDis)
        {
            int err = robot.StartJOG(refType, nb, dir, vel, acc, maxDis);
            if (err != 0)
            {

            }
            return err;
        }

        public static int StopJOG(byte stopByte)
        {
            int err = robot.StopJOG(stopByte);
            if (err != 0)
            {

            }
            return err;
        }

        public static int ImmStopJOG()
        {
            int err = robot.ImmStopJOG();
            if (err != 0)
            {

            }
            return err;
        }

        #endregion

        #region Struttura checker position

        /// <summary>
        /// A true quando il thread/task che controlla la posizione per la UI deve essere concluso.
        /// </summary>
        private static bool stopPositionCheckerThread = false;
        /// <summary>
        /// A true quando il punto attuale del robot corrisponde con la posizione interrogata della lw_positions.
        /// </summary>
        private static bool inPositionGun = false;
        /// <summary>
        /// Lista di posizioni su cui eseguire l'inPosition per gestire colorazione di lw_positions.
        /// </summary>
        private static List<KeyValuePair<int, DescPose>> positionsToCheck = new List<KeyValuePair<int, DescPose>>();
        /// <summary>
        /// Numero di posizioni su cui eseguire l'inPosition e la relativa colorazione su lw_positions.
        /// </summary>
        private static int numPositionsToCheck = 5;
        /// <summary>
        /// A true quando l'aggiornamento della lista di posizioni su cui eseguire inPosition 
        /// per gestire colorazione su lw_positions viene terminato.
        /// </summary>
        private static bool aggiornamentoFinito = false;
        /// <summary>
        /// Variabile per tracciare l'ultima posizione aggiornata nella ListView.
        /// </summary>
        private static int? lastUpdatedKey = null;

        #endregion

        #region Step cicli

        /// <summary>
        /// True quando può partire la PlaceRoutine
        /// </summary>
        public static bool startPlaceRoutine = false;
        /// <summary>
        /// True quando può partire la HomeRoutine
        /// </summary>
        public static bool startHomeRoutine = false;
        /// <summary>
        /// True quando il ciclo puo essere avviato
        /// </summary>
        public static bool startCycle = false;
        /// <summary>
        /// Specifica se il robot è abilitato
        /// </summary>
        public static bool robotEnable = true;
        /// <summary>
        /// A true se robot in movimento
        /// </summary>
        private static bool isRobotMoving = false;
        /// <summary>
        /// A true quando robot pronto
        /// </summary>
        private static bool robotStarted = false;
        /// <summary>
        /// A true quando robot pronto
        /// </summary>
        private static bool robotOn = false;
        /// <summary>
        /// A true quando robot pronto
        /// </summary>
        private static bool robotReady = false;
        /// <summary>
        /// Indica la prenotazione di stop ciclo, prima di terminare il thread aspetta che sia stato fatto il place
        /// </summary>
        private static bool requestStopCycle = false;
        /// <summary>
        /// True quando le pinze del roboto sono state chiuse
        /// </summary>
        public static bool gripperClosed = false;
        /// <summary>
        /// True quando devo aprire le pinze
        /// </summary>
        public static bool closeGripper = false;
        /// <summary>
        /// Indica se il robot in è in pick position
        /// </summary>
        public static bool inPickPosition = false;
        /// <summary>
        /// True quando le pinze del robot sono aperte
        /// </summary>
        public static bool gripperOpened = false;
        /// <summary>
        /// True quando le devo aprire le pinze
        /// </summary>
        public static bool openGripper = false;
        /// <summary>
        /// Stato precedente ingombro nastro
        /// </summary>
        private static bool prevRobotOutPick = true;
        /// <summary>
        /// Stato precedente ingombro teglia 1
        /// </summary>
        private static bool prevRobotOutPlace = true;
        /// <summary>
        /// Stato precedente ingombro teglia 2
        /// </summary>
        private static bool prevRobotOutWelding = true;
        /// <summary>
        /// Stato precedente ingombro home position
        /// </summary>
        private static bool? prevInHomePos = null;
        /// <summary>
        /// Memorizza lo stato precedente della variabile on/off barrier status dal PLC
        /// </summary>
        private static bool previousBarrierPauseStatus = false;
        /// <summary>
        /// Memorizza lo stato precedente della variabile on/off barrier status dal PLC
        /// </summary>
        private static bool previousBarrierResumeStatus = false;
        /// <summary>
        /// Memorizza lo stato precedente della richiesta di registrazione punto
        /// </summary>
        private static int previousRecordPointRequest = -1;
        /// <summary>
        /// Memorizza lo stato precedente della richiesta di reset degli allarmi
        /// </summary>
        private static int previousAlarmResetRequested = -1;
        /// <summary>
        /// Stato precedente fuori ingombro
        /// </summary>
        private static bool prevFuoriIngombro = false;
        /// <summary>
        /// Offset sull'asse x per spostamento su teglie
        /// </summary>
        public static double xOffset = -150;
        /// <summary>
        /// Offset sull'asse y per spostamento su teglie
        /// </summary>
        public static double yOffset = 200;
        /// <summary>
        /// Numero di tentativi di ping al robot
        /// </summary>
        public static int numAttempsRobotPing = 0;
        /// <summary>
        /// Velocità di override del Robot
        /// </summary>
        public static int velocityOverride = 0;
        /// <summary>
        /// Indica se il robot è in pausa
        /// </summary>
        public static bool robotInPause = false;
        /// <summary>
        /// Riferimento allo step delle normal variables corrente
        /// </summary>
        public static int step = 0;
        /// <summary>
        /// A true quando viene richiesto lo stop del ciclo routine del robot
        /// </summary>
        public static bool robotCycleStopRequested = false;
        /// <summary>
        /// A true quando si trova in posizione di Pick
        /// </summary>
        public static bool isInPositionPick = false;
        /// <summary>
        /// A true quando si trova in posizione di Place
        /// </summary>
        public static bool isInPositionPlace = false;
        /// <summary>
        /// A true quando si trova in posizione di Welding
        /// </summary>
        public static bool isInPositionWelding = false;
        /// <summary>
        /// A true quando il robot si trova in home zone
        /// </summary>
        public static bool isInHomeZone = false;
        /// <summary>
        /// A true quando si trova in posizione di home
        /// </summary>
        public static bool isInPositionHome = false;
        /// <summary>
        /// A true quando il robot viene messo in pausa
        /// </summary>
        public static bool robotIsPaused = false;
        /// <summary>
        /// Salva stato di override velocità precedente
        /// </summary>
        private static int previousVel = 0;
        /// <summary>
        /// Richiesta stop ciclo home
        /// </summary>
        static bool stopHomeRoutine = false;
        /// <summary>
        /// Step ciclo home
        /// </summary>
        static int stepHomeRoutine = 0;
        /// <summary>
        /// A true quando robot in modalità automatica
        /// </summary>
        private static bool isAuto = false;
        /// <summary>
        /// A true quando robot in modalità manuale
        /// </summary>
        private static bool isManual = false;
        /// <summary>
        /// Valore di avvio ciclo main
        /// </summary>
        public static int CycleRun_Main = 0;
        /// <summary>
        /// Valore di avvio ciclo pick
        /// </summary>
        public static int CycleRun_Pick = 0;
        /// <summary>
        /// Valore di avvio ciclo place
        /// </summary>
        public static int CycleRun_Place = 0;
        /// <summary>
        /// Valore di avvio ciclo home
        /// </summary>
        public static int CycleRun_Home = 0;
        /// <summary>
        /// Segnale di stop della pick routine
        /// </summary>
        static bool stopPickRoutine = false;
        /// <summary>
        /// Step ciclo di pick
        /// </summary>
        static int stepPick = 0;
        /// <summary>
        /// Segnale di stop della place routine
        /// </summary>
        static bool stopPlaceRoutine = false;
        /// <summary>
        /// Step ciclo di place
        /// </summary>
        static int stepPlace = 0;
        /// <summary>
        /// Valore enable robot
        /// </summary>
        private static bool isEnable = false;
        /// <summary>
        /// Valore not enable robot
        /// </summary>
        private static bool isNotEnable = false;

        #endregion

        #region Nomi tasks

        /// <summary>
        /// Nome del task high priority
        /// </summary>
        public static string TaskHighPriorityName = nameof(CheckHighPriority);
        /// <summary>
        /// Nome del task low priority
        /// </summary>
        public static string TaskLowPriorityName = nameof(CheckLowPriority);
        /// <summary>
        /// Nome del task auxiliary worker
        /// </summary>
        public static string TaskAuxiliaryWorkerName = nameof(AuxiliaryWorker);
        /// <summary>
        /// Nome del task plc com handler
        /// </summary>
        public static string TaskPlcComHandlerName = nameof(PlcComHandler);
        /// <summary>
        /// Nome del task check robot com
        /// </summary>
        public static string TaskCheckRobotConneciton = nameof(CheckRobotConnection);
        /// <summary>
        /// Nome del task application manager
        /// </summary>
        public static string TaskApplicationManager = nameof(ApplicationTaskManager);
        /// <summary>
        /// Nome del task safety manager
        /// </summary>
        public static string TaskSafetyManager = nameof(SafetyTaskManager);
        /// <summary>
        /// Nome del task home routine
        /// </summary>
        public static string TaskHomeRoutine = nameof(HomeRoutine);
        /// <summary>
        /// Nome del task check robot com
        /// </summary>
        public static string TaskPickAndPlaceTegliaIperal = nameof(TaskPickAndPlaceTegliaIperal);

        #endregion

        #region Token per task interni

        /// <summary>
        /// Token per fermare il ciclo di pick
        /// </summary>
        private static CancellationTokenSource pickBox_cts;

        /// <summary>
        /// Token per fermare il ciclo di place
        /// </summary>
        private static CancellationTokenSource placeBox_cts;

        #endregion

        #endregion

        #region Metodi della classe RobotManager

        /// <summary>
        /// Costruttore statico, chiamato dal programma in automatico all'inizio
        /// </summary>
        static RobotManager()
        {
            taskManager = new TaskManager();
            taskManager.StartTaskChecker();
        }

        /// <summary>
        /// Metodo che inizializza Robot e lo accende
        /// </summary>
        /// <param name="robotIpAddress">Indirizzo IP Robot</param>
        /// <returns></returns>
        public static bool InitRobot(string robotIpAddress)
        {
            formAlarmPage = new FormAlarmPage();
            formAlarmPage.AlarmsCleared += RMLib_AlarmsCleared;

            formDiagnostics = new FormDiagnostics();

            // Istanzio il robot
            RobotIpAddress = robotIpAddress;
            robot = new Robot();
            string logDirectory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Logs");
            robot.LoggerInit(FrLogType.ASYNC, FrLogLevel.ERROR, logDirectory, 2, 2);
            robot.RPC(RobotIpAddress);
            AlarmManager.isRobotConnected = true;

            // Faccio partire i manager
            frameManager = new Frames(robot);
            toolManager = new Tools(robot);
            collisionManager = new Collisions(robot);

            // Inizializzazione mode
            ROBOT_STATE_PKG robot_state_pkg = new ROBOT_STATE_PKG();
            //robot.GetRobotRealTimeState(ref robot_state_pkg);
            GetRobotRealTimeState(ref robot_state_pkg);
            currentRobotMode = robot_state_pkg.robot_mode;
            isAutomaticMode = currentRobotMode == 0;

            // Faccio partire i task
            taskManager.AddTask(TaskCheckRobotConneciton, CheckRobotConnection, TaskType.LongRunning, true);
            taskManager.AddTask(TaskHighPriorityName, CheckHighPriority, TaskType.LongRunning, true);
            taskManager.AddTask(TaskAuxiliaryWorkerName, AuxiliaryWorker, TaskType.LongRunning, true);
            taskManager.AddTask(TaskLowPriorityName, CheckLowPriority, TaskType.LongRunning, true);
            //taskManager.AddTask(TaskApplicationManager, ApplicationTaskManager, TaskType.LongRunning, true);
            // taskManager.AddTask(TaskPlcComHandlerName, PlcComHandler, TaskType.LongRunning, true);
            //taskManager.AddTask(TaskSafetyManager, SafetyTaskManager, TaskType.LongRunning, true);

            taskManager.StartTask(TaskCheckRobotConneciton);
            taskManager.StartTask(TaskHighPriorityName);
            taskManager.StartTask(TaskAuxiliaryWorkerName);
            taskManager.StartTask(TaskLowPriorityName);
            //taskManager.StartTask(TaskApplicationManager);
            // taskManager.StartTask(TaskPlcComHandlerName);
            //taskManager.StartTask(TaskSafetyManager);

            log.Info("Task di background del robot avviati tramite TaskManager.");

            if (err == -4)
            {
                log.Error("RPC exception durante Init del Robot");
                return false;
            }

            // Se fallisce setting della proprietà del Robot
            if (!GetRobotProperties())
                return false;

            // Se fallisce setting della proprietà del Robot
            if (!SetRobotProperties())
                return false;

            log.Info("Parametri del robot assegnati");

            GetRobotInfo();

            ResetPLCVariables();

            return true;
        }

        #region Task di servizio

        /// <summary>
        /// Gestisce gli ausiliari del Robot
        /// </summary>
        private static async Task AuxiliaryWorker(CancellationToken token)
        {
            // Lista di aggiornamenti da inviare al PLC
            List<(string key, bool value, string type)> updates = new List<(string, bool, string)>();

            #region ingombro
            /*
            // Zone di ingombro
            var pickPose = ApplicationConfig.applicationsManager.GetPosition("101", "RM");
            var placePose = ApplicationConfig.applicationsManager.GetPosition("1101", "RM");
            var homePose = ApplicationConfig.applicationsManager.GetPosition("1", "RM");

            DescPose[] startPoints = new DescPose[]
            {
                new DescPose(pickPose.x, pickPose.y, pickPose.z, pickPose.rx, pickPose.ry, pickPose.rz),
                new DescPose(placePose.x, placePose.y, placePose.z, placePose.rx, placePose.ry, placePose.rz),
                new DescPose(homePose.x, homePose.y, homePose.z, homePose.rx, homePose.ry, homePose.rz),
            };

            // Oggetto che rileva ingombro pick
            double delta_ingombro_pick = 500.0;
            checker_ingombro_pick = new PositionChecker(delta_ingombro_pick);

            // Oggetto che rileva ingombro place
            double delta_ingombro_place = 500.0;
            checker_ingombro_place = new PositionChecker(delta_ingombro_place);

            // Oggetto che rileva ingombro home
            double delta_ingombro_home = 500.0;
            checker_ingombro_home = new PositionChecker(delta_ingombro_home);
            */
            #endregion

            #region Safe zone

            // Dichiarazione del punto di safe
            var pSafeZone = ApplicationConfig.applicationsManager.GetPosition("pSafeZone", "RM");

            DescPose pointSafeZone = new DescPose(pSafeZone.x, pSafeZone.y, pSafeZone.z, pSafeZone.rx, pSafeZone.ry, pSafeZone.rz);

            // Dichiarazione del punto di home
            var pHome = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");

            DescPose pointHome = new DescPose(pHome.x, pHome.y, pHome.z, pHome.rx, pHome.ry, pHome.rz);

            // Oggetto che rileva safe zone
            double delta_safeZone = 300.0; // soglia
            checker_safeZone = new PositionChecker(delta_safeZone);

            #endregion

            checker_pos = new PositionChecker(5.0);

            try
            {
                while (!token.IsCancellationRequested)
                {
                    if (AlarmManager.isRobotConnected)
                    {
                        await CheckIsRobotEnable();
                        CheckRobotMode();
                        CheckCurrentToolAndUser();
                        CheckGripperStatus();
                        //CheckIsRobotInObstructionArea(startPoints, updates);
                    }

                    await Task.Delay(auxiliaryThreadRefreshPeriod, token);
                }

                token.ThrowIfCancellationRequested(); //Solleva eccezione per far andare in stop e non in completed
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskAuxiliaryWorkerName}: {ex}");
                throw;
            }
            finally
            {

            }

        }

        /// <summary>
        /// Thread ad alta priorità che tiene monitorato movimento robot e zone di ingombro
        /// </summary>
        private async static Task CheckHighPriority(CancellationToken token)
        {
            // Lista di aggiornamenti da inviare al PLC
            List<(string key, bool value, string type)> updates = new List<(string, bool, string)>();

            #region Safe zone

            // Dichiarazione del punto di safe
            var pSafeZone = ApplicationConfig.applicationsManager.GetPosition("pSafeZone", "RM");

            DescPose pointSafeZone = new DescPose(pSafeZone.x, pSafeZone.y, pSafeZone.z, pSafeZone.rx, pSafeZone.ry, pSafeZone.rz);

            // Oggetto che rileva safe zone
            double delta_safeZone = 300.0; // soglia
            checker_safeZone = new PositionChecker(delta_safeZone);

            #endregion

            checker_pos = new PositionChecker(5.0);

            try
            {
                while (!token.IsCancellationRequested)
                {
                    if (robot != null && AlarmManager.isRobotConnected)
                    {
                        try
                        {
                            robot.GetActualTCPPose(flag, ref TCPCurrentPosition); // Leggo posizione robot TCP corrente
                            CheckIsRobotMoving(updates);
                            CheckIsRobotInSafeZone(pointSafeZone);
                            CheckIsRobotInPos();
                            CheckStatusRobot();
                        }
                        catch (Exception e)
                        {
                            log.Error("RobotManager: errore durante la valutazione delle variabili HIGH: " + e.Message);
                        }
                    }

                    updates.Clear(); //Svuoto la lista di aggiornamento

                    await Task.Delay(highPriorityRefreshPeriod, token);
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskHighPriorityName}: {ex}");
                throw;
            }
            finally
            {
                updates.Clear(); // Svuoto la lista di aggiornamento
            }
        }

        /// <summary>
        /// Thread che gestisce il controllo connessione plc e la scrittura degli aggiornamenti delle variabili
        /// </summary>
        /// <param name="token"></param>
        /// <returns></returns>
        private async static Task PlcComHandler(CancellationToken token)
        {
            DateTime now = DateTime.Now;
            long unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();
            DateTime dateTime = DateTime.Now;
            string formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

            Dictionary<string, object> alarmValues = new Dictionary<string, object>();
            Dictionary<string, string> alarmDescriptions = new Dictionary<string, string>
            {
                { "Safety NOK", "Ausiliari non pronti" },
                { "Modbus robot error", "Errore comunicazione modbus robot" },
                { "Robot Cycle Paused", "Ciclo robot in pausa" },
                { "Error plates full", "Teglie piene" },
                { "Check open gr.failed", "Controllo pinza aperta fallito" },
                { "Check pos_Dx gr. failed", "Controllo pinza chiusa fallito" },
                { "Robot fault present", "Errore robot" },
                { "US_Dx_Error", "Errore ultrasuono" },
                { "US_Dx_Enabled", "Ultrasuono abilitato" },
                { "US_Dx_Started", "Ultrasuono avviato" },
                { "US_Dx_Error_Disconnect", "Ultrasuono disconnesso" },
                { "Errore_Drive_Destro", "Mancata presa pinza robot" },
            };

            JointPos jPos = new JointPos(0, 0, 0, 0, 0, 0);

            try
            {
                while (!token.IsCancellationRequested)
                {
                    CheckRobotPosition(jPos);

                    GetPLCErrorCode(alarmValues, alarmDescriptions, now, unixTimestamp,
                        dateTime, formattedDate);

                    SendUpdatesToPLC();

                    await Task.Delay(plcComTaskRefreshPeriod, token);
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskPlcComHandlerName}: {ex}");
                throw;
            }
            finally
            {

            }
        }

        /// <summary>
        /// Thread a priorità bassa che gestisce allarmi robot e PLC
        /// </summary>
        private async static Task CheckLowPriority(CancellationToken token)
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                    CheckPLCConnection();
                    GetRobotErrorCode();

                    await Task.Delay(lowPriorityRefreshPeriod, token);
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskLowPriorityName}: {ex}");
                throw;
            }
            finally
            {

            }
        }

        /// <summary>
        /// Task asincrono che controlla se il robot è connesso invocando un metodo direttamente tramite il proxy (controllore).
        /// Non vengono usati metodi della libreria poichè gestiti male e bloccanti, una chiamata al diretto interessato funziona meglio
        /// e viene fatta senza bloccare il task.
        /// Nel caso fosse rilevata la disconnessione allora viene chiamato closeRPC e il task continua a cercare la riconnessione.
        /// Quando la riconnessione arriva allora prova a re istanziare il robot.
        /// </summary>
        /// <param name="token"></param>
        /// <returns></returns>
        private async static Task CheckRobotConnection(CancellationToken token)
        {
            const int MAX_FAILURE_ATTEMPTS = 2; // Tentativi consecutivi prima di dichiarare la disconnessione
            int consecutiveFailures = 0;

            try
            {
                //Istanzio il proxy che interfaccia il controllore in maniera diretta
                ICallSupervisor connectionProxy = XmlRpcProxyGen.Create<ICallSupervisor>();
                connectionProxy.Url = $"http://{RobotIpAddress}:20003/RPC2";
                connectionProxy.Timeout = connectionCheckMaxTimeout; // Timeout breve per non bloccare troppo a lungo.

                log.Warn("[Robot COM] Task di controllo connessione avviato.");

                while (!token.IsCancellationRequested)
                {
                    bool isProxyConnected = false;
                    try
                    {
                        // --- IL CONTROLLO DIRETTO SUL PROXY ---
                        // Eseguiamo la chiamata RPC su un thread del pool per non bloccare
                        // il nostro loop while nel caso in cui il timeout non funzioni bene.
                        await Task.Run(() => connectionProxy.GetRobotErrorCode(), token);
                        isProxyConnected = true;
                    }
                    catch (Exception)
                    {
                        // Qualsiasi eccezione (XmlRpcException, WebException) significa che non siamo connessi.
                        isProxyConnected = false;
                        //log.Error($"[Guardian] Rilevata disconnessione: {ex.GetType().Name} - {ex.Message}");
                    }

                    if (isProxyConnected) //Connessione verificata
                    {
                        if (!AlarmManager.isRobotConnected)
                        {
                            // Eravamo in stato "disconnesso", ma ora la rete è tornata.
                            // Forziamo comunque una riconnessione per essere sicuri che la libreria sia sana.
                            log.Warn("[Robot COM] Connessione al robot RISTABILITA.");

                            await AttemptReconnectToRobot(); // Tentiamo di ripristinare la libreria
                        }
                    }
                    else //Connessione assente
                    {
                        if (consecutiveFailures < MAX_FAILURE_ATTEMPTS)
                            consecutiveFailures++;

                        if (consecutiveFailures >= MAX_FAILURE_ATTEMPTS && AlarmManager.isRobotConnected)
                        {
                            // È la prima volta che rileviamo la disconnessione
                            log.Error("[Robot COM] Connessione al robot PERSA. Avvio tentativi di riconnessione...");
                            AlarmManager.isRobotConnected = false;
                            //RefresherTask.AddUpdate(PLCTagName.Emergency, 1, "INT16");

                            try { robot.CloseRPC(); } catch { } //Chiusura dei thread di libreria

                            // Generazione allarme bloccante
                            AlarmManager.blockingAlarm = true;

                            string id = "1";
                            string description = "Robot disconnesso.";

                            DateTime now = DateTime.Now;
                            long unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();
                            DateTime dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
                            string formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

                            string device = "Robot";
                            string state = "ON";

                            if (!IsAlarmAlreadySignaled(id))
                            {
                                CreateRobotAlarm(id, description, formattedDate, device, state);
                                MarkAlarmAsSignaled(id);
                                robotError = 1;
                            }
                        }
                    }

                    await Task.Delay(robotComTaskRefreshPeriod, token);
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskCheckRobotConneciton}: {ex}");
                throw;
            }
            finally
            {

            }
        }

        /// <summary>
        /// Esegue check apertura/chiusura pinza
        /// </summary>
        /// <returns></returns>
        private static async Task CheckGripperStatus()
        {

            // Get input digitale (pinza)
            byte gripperStatus = 0;
            RobotManager.robot.GetDI(0, 1, ref gripperStatus);

            if (Convert.ToBoolean(gripperStatus) != previousGripperStatus)
            {

                if (gripperStatus == 0) // Se la pinza è chiusa
                {
                    GripperStatusON?.Invoke(null, EventArgs.Empty);
                }
                else
                {
                    GripperStatusOFF?.Invoke(null, EventArgs.Empty);
                }

                previousGripperStatus = gripperStatus > 0;
            }

          
        }

        /// <summary>
        /// Esegue controlli su modalità robot, task in uso e sceglie quali task fermare/partire
        /// </summary>
        /// <param name="token"></param>
        /// <returns></returns>
        private async static Task ApplicationTaskManager(CancellationToken token)
        {
            try
            {
                await Task.Delay(2000);

                while (!token.IsCancellationRequested)
                {
                   // await CheckCommandStart();
                   // await CheckCommandGoToHome();

                  //  await CheckCommandRecordPoint();
                   // await CheckVelCommand();
                    //await CheckCloseGripper();
                   // CheckCommandResetAlarms();

                    //SetRobotMode();
                    //ManageTasks();

                    await Task.Delay(applicationTaskManagerRefreshPeriod);
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskApplicationManager}: {ex}");
                throw;
            }
            finally
            {

            }
        }

        /// <summary>
        /// Esegue controlli sui comandi safety: barriere->pause/resume e stop
        /// </summary>
        /// <param name="token"></param>
        /// <returns></returns>
        private async static Task SafetyTaskManager(CancellationToken token)
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                   // await CheckCommandStop();
                   // await CheckPauseStatus();
                    // CheckResumeStatus();

                    await Task.Delay(safetyTaskManagerRefreshPeriod);
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskSafetyManager}: {ex}");
                throw;
            }
            finally
            {

            }
        }

        #endregion

        #region Altri task

        /// <summary>
        /// Esegue ciclo teglie iperal
        /// </summary>
        public async static Task PickAndPlaceTegliaIperal(CancellationToken token)
        {
            #region Parametri movimento

            // Offset spostamento
            DescPose offset = new DescPose(0, 0, 0, 0, 0, 0);
            // Dichiarazione asse esterno
            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
            // Flag presenza offset
            byte offsetFlag = 0;
            // Flag per ricerca filo
            byte search = 0;
            // Parametri moveL
            int velAccParamMode = 0;
            int overSpeedStrategy = 0;
            int speedPercent = 0;
            // Codice risultante del movimento del Robot
            int movementResult = -1;
            // Reset condizione di stop ciclo
            stopCycleRoutine = false;
            // Reset richiesta di stop ciclo
            stopCycleRequested = false;
            // Reset step routine
            step = 0;
            byte ris = 0;

            #endregion

            #region Offset spostamenti

            int offsetAllontamento = 850;
            int offsetAvvicinamento = 400;
            int offsetPrePlace = 850;
            int offsetAllontamentoPostPlace = 300;
            int zOffsetPrePickTeglia = 40;
            int zOffsetPostPickTeglia = 40;
            int zOffsetAllontanamentoPostPickTeglia1 = 40;
            int zOffsetPrePlace = 20;

            #endregion

            #region Punti utili al ciclo

            #region Punto home

            // Oggetto jointPos
            JointPos jointPosHome = new JointPos(0, 0, 0, 0, 0, 0);

            // Get delle coordinate del punto dal database
            var home = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");

            // Creazione oggetto descPose
            DescPose descPosHome = new DescPose(
                home.x,
                home.y,
                home.z,
                home.rx,
                home.ry,
                home.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosHome, -1, ref jointPosHome);

            #endregion

            #region Pick teglia 1

            #region Punto di pick teglia 1

            // Oggetto jointPos
            JointPos jointPosPickTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Get delle coordinate del punto dal database
            var pickTeglia1 = ApplicationConfig.applicationsManager.GetPosition("pPickTegliaIperal", "RM");

            // Creazione oggetto descPose
            DescPose descPosPickTeglia1 = new DescPose(
                pickTeglia1.x,
                pickTeglia1.y,
                pickTeglia1.z,
                pickTeglia1.rx,
                pickTeglia1.ry,
                pickTeglia1.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosPickTeglia1, -1, ref jointPosPickTeglia1);

            #endregion

            #region Punto avvicinamento pick teglia 1

            // Oggetto jointPos
            JointPos jointPosApproachPickTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosApproachPickTeglia1 = new DescPose(
                pickTeglia1.x,
                pickTeglia1.y - offsetAvvicinamento,
                pickTeglia1.z - zOffsetPrePickTeglia,
                pickTeglia1.rx,
                pickTeglia1.ry,
                pickTeglia1.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosApproachPickTeglia1, -1, ref jointPosApproachPickTeglia1);

            #endregion

            #region Punto post pick teglia 1

            // Oggetto jointPos
            JointPos jointPosPostPickTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosPostPickTeglia1 = new DescPose(
                pickTeglia1.x,
                pickTeglia1.y,
                pickTeglia1.z + zOffsetPostPickTeglia,
                pickTeglia1.rx,
                pickTeglia1.ry,
                pickTeglia1.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosPostPickTeglia1, -1, ref jointPosPostPickTeglia1);

            #endregion

            #region Punto allontanamento post pick teglia 1

            // Oggetto jointPos
            JointPos jointPosAllontanamentoPickTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosAllontanamentoPickTeglia1 = new DescPose(
                descPosApproachPickTeglia1.tran.x,
                descPosApproachPickTeglia1.tran.y - offsetAllontamento,
                descPosApproachPickTeglia1.tran.z + zOffsetAllontanamentoPostPickTeglia1,
                descPosApproachPickTeglia1.rpy.rx,
                descPosApproachPickTeglia1.rpy.ry,
                descPosApproachPickTeglia1.rpy.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosAllontanamentoPickTeglia1, -1, ref jointPosAllontanamentoPickTeglia1);

            #endregion

            #endregion

            #region Place teglia 1

            #region Punto di place teglia 1

            // Oggetto jointPos
            JointPos jointPosPlaceTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Get delle coordinate del punto dal database
            var placeTeglia1 = ApplicationConfig.applicationsManager.GetPosition("pPlaceTegliaIperal", "RM");

            // Creazione oggetto descPose
            DescPose descPosPlaceTeglia1 = new DescPose(
                placeTeglia1.x,
                placeTeglia1.y,
                placeTeglia1.z,
                placeTeglia1.rx,
                placeTeglia1.ry,
                placeTeglia1.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosPlaceTeglia1, -1, ref jointPosPlaceTeglia1);

            #endregion

            #region Punto di rotazione da pick a place teglia 1

            // Oggetto jointPos
            JointPos jointPosRotationPrePlaceTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosRotationPrePlaceTeglia1 = new DescPose(
                descPosHome.tran.x,
                descPosHome.tran.y,
                descPosHome.tran.z,
                placeTeglia1.rx,
                placeTeglia1.ry,
                placeTeglia1.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosRotationPrePlaceTeglia1, -1, ref jointPosRotationPrePlaceTeglia1);

            #endregion

            #region Punto avvicinamento place teglia 1

            // Oggetto jointPos
            JointPos jointPosApproachPlaceTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosApproachPlaceTeglia1 = new DescPose(
                placeTeglia1.x - offsetPrePlace,
                placeTeglia1.y,
                placeTeglia1.z,
                placeTeglia1.rx,
                placeTeglia1.ry,
                placeTeglia1.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosApproachPlaceTeglia1, -1, ref jointPosApproachPlaceTeglia1);

            #endregion

            #region Punto post place teglia 1

            // Oggetto jointPos
            JointPos jointPosPostPlaceTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosPostPlaceTeglia1 = new DescPose(
                placeTeglia1.x,
                placeTeglia1.y,
                placeTeglia1.z,
                placeTeglia1.rx,
                placeTeglia1.ry,
                placeTeglia1.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosPostPlaceTeglia1, -1, ref jointPosPostPlaceTeglia1);

            #endregion

            #region Punto allontanamento place teglia 1

            // Oggetto jointPos
            JointPos jointPosAllontanamentoPlaceTeglia1 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosAllontanamentoPlaceTeglia1 = new DescPose(
                placeTeglia1.x - offsetAllontamento,
                placeTeglia1.y,
                placeTeglia1.z,
                placeTeglia1.rx,
                placeTeglia1.ry,
                placeTeglia1.rz);

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosAllontanamentoPlaceTeglia1, -1, ref jointPosAllontanamentoPlaceTeglia1);

            #endregion

            #endregion

            #region Place teglia 2

            #region Punto di place teglia 2

            // Oggetto jointPos
            JointPos jointPosPlaceTeglia2 = new JointPos(0, 0, 0, 0, 0, 0);

            // Get delle coordinate del punto dal database
            var placeTeglia2 = ApplicationConfig.applicationsManager.GetPosition("pPlaceTegliaIperal2", "RM");

            // Creazione oggetto descPose
            DescPose descPosPlaceTeglia2 = new DescPose(
                placeTeglia2.x,
                placeTeglia2.y,
                placeTeglia2.z,
                placeTeglia2.rx,
                placeTeglia2.ry,
                placeTeglia2.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosPlaceTeglia2, -1, ref jointPosPlaceTeglia2);

            #endregion

            #region Punto avvicinamento place teglia 2

            // Oggetto jointPos
            JointPos jointPosApproachPlaceTeglia2 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosApproachPlaceTeglia2 = new DescPose(
                placeTeglia2.x,
                placeTeglia2.y - offsetPrePlace,
                placeTeglia2.z + 20,
                placeTeglia2.rx,
                placeTeglia2.ry,
                placeTeglia2.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosApproachPlaceTeglia2, -1, ref jointPosApproachPlaceTeglia2);

            #endregion

            #region Punto di rotazione da place teglia 1 a place teglia 2

            // Oggetto jointPos
            JointPos jointPosRotationPrePlaceTeglia2 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosRotationPrePlaceTeglia2 = new DescPose(
                home.x,
                home.y,
                placeTeglia2.z + zOffsetPrePlace,
                placeTeglia2.rx,
                placeTeglia2.ry,
                placeTeglia2.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosRotationPrePlaceTeglia2, -1, ref jointPosRotationPrePlaceTeglia2);

            #endregion

            #region Punto allontanamento place teglia 2

            // Oggetto jointPos
            JointPos jointPosAllontanamentoPlaceTeglia2 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosAllontanamentoPlaceTeglia2 = new DescPose(
                placeTeglia2.x,
                placeTeglia2.y - offsetAllontamentoPostPlace,
                placeTeglia2.z,
                placeTeglia2.rx,
                placeTeglia2.ry,
                placeTeglia2.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosAllontanamentoPlaceTeglia2, -1, ref jointPosAllontanamentoPlaceTeglia2);

            #endregion


            #endregion

            #region Pick teglia1

            #region Punto di pick teglia 1

            // Oggetto jointPos
            JointPos jointPosPickTeglia2 = new JointPos(0, 0, 0, 0, 0, 0);

            // Get delle coordinate del punto dal database
            var pickTeglia2 = ApplicationConfig.applicationsManager.GetPosition("pPickTegliaIperal2", "RM");

            // Creazione oggetto descPose
            DescPose descPosPickTeglia2 = new DescPose(
                pickTeglia2.x,
                pickTeglia2.y,
                pickTeglia2.z,
                pickTeglia2.rx,
                pickTeglia2.ry,
                pickTeglia2.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosPickTeglia2, -1, ref jointPosPickTeglia2);

            #endregion

            #region Punto avvicinamento pick teglia 2

            // Oggetto jointPos
            JointPos jointPosApproachPickTeglia2 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosApproachPickTeglia2 = new DescPose(
                pickTeglia2.x,
                pickTeglia2.y - offsetAvvicinamento,
                pickTeglia2.z - zOffsetPrePickTeglia,
                pickTeglia2.rx,
                pickTeglia2.ry,
                pickTeglia2.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosApproachPickTeglia2, -1, ref jointPosApproachPickTeglia2);

            #endregion

            #region Punto post pick teglia 2

            // Oggetto jointPos
            JointPos jointPosPostPickTeglia2 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosPostPickTeglia2 = new DescPose(
                pickTeglia2.x,
                pickTeglia2.y,
                pickTeglia2.z + zOffsetPostPickTeglia,
                pickTeglia2.rx,
                pickTeglia2.ry,
                pickTeglia2.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosPostPickTeglia2, -1, ref jointPosPostPickTeglia2);

            #endregion

            #region Punto allontanamento post pick teglia 2

            // Oggetto jointPos
            JointPos jointPosAllontanamentoPickTeglia2 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosAllontanamentoPickTeglia2 = new DescPose(
                descPosApproachPickTeglia2.tran.x,
                descPosApproachPickTeglia2.tran.y - offsetAllontamento,
                descPosApproachPickTeglia2.tran.z + zOffsetAllontanamentoPostPickTeglia1,
                descPosApproachPickTeglia2.rpy.rx,
                descPosApproachPickTeglia2.rpy.ry,
                descPosApproachPickTeglia2.rpy.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosAllontanamentoPickTeglia2, -1, ref jointPosAllontanamentoPickTeglia2);

            #endregion

            #endregion

            #region Place teglia 3

            #region Punto di place teglia 3

            // Oggetto jointPos
            JointPos jointPosPlaceTeglia3 = new JointPos(0, 0, 0, 0, 0, 0);

            // Get delle coordinate del punto dal database
            var placeTeglia3 = ApplicationConfig.applicationsManager.GetPosition("pPlaceTegliaIperal3", "RM");

            // Creazione oggetto descPose
            DescPose descPosPlaceTeglia3 = new DescPose(
                placeTeglia3.x,
                placeTeglia3.y,
                placeTeglia3.z,
                placeTeglia3.rx,
                placeTeglia3.ry,
                placeTeglia3.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosPlaceTeglia3, -1, ref jointPosPlaceTeglia3);

            #endregion

            #region Punto avvicinamento place teglia 3

            // Oggetto jointPos
            JointPos jointPosApproachPlaceTeglia3 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosApproachPlaceTeglia3 = new DescPose(
                placeTeglia3.x,
                placeTeglia3.y - offsetPrePlace,
                placeTeglia3.z + 40,
                placeTeglia3.rx,
                placeTeglia3.ry,
                placeTeglia3.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosApproachPlaceTeglia3, -1, ref jointPosApproachPlaceTeglia3);

            #endregion

            #region Punto di rotazione da place teglia 1 a place teglia 3

            // Oggetto jointPos
            JointPos jointPosRotationPrePlaceTeglia3 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosRotationPrePlaceTeglia3 = new DescPose(
                home.x,
                home.y,
                placeTeglia3.z + 20,
                placeTeglia3.rx,
                placeTeglia3.ry,
                placeTeglia3.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosRotationPrePlaceTeglia3, -1, ref jointPosRotationPrePlaceTeglia3);

            #endregion

            #region Punto allontanamento place teglia 3

            // Oggetto jointPos
            JointPos jointPosAllontanamentoPlaceTeglia3 = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosAllontanamentoPlaceTeglia3 = new DescPose(
                placeTeglia3.x,
                placeTeglia3.y - offsetAllontamentoPostPlace,
                placeTeglia3.z,
                placeTeglia3.rx,
                placeTeglia3.ry,
                placeTeglia3.rz
                );

            // Calcolo del jointPos
            robot.GetInverseKin(0, descPosAllontanamentoPlaceTeglia3, -1, ref jointPosAllontanamentoPlaceTeglia3);

            #endregion

            #endregion

            #endregion

            #region Collisioni

            double[] levelCollision1 = new double[] { 1, 1, 1, 1, 1, 1 };
            double[] levelCollision2 = new double[] { 2, 2, 2, 2, 2, 2 };
            double[] levelCollision3 = new double[] { 3, 3, 3, 3, 3, 3 };
            double[] levelCollision4 = new double[] { 4, 4, 4, 4, 4, 4 };
            double[] levelCollision5 = new double[] { 5, 5, 5, 5, 5, 5 };
            double[] levelCollision6 = new double[] { 6, 6, 6, 6, 6, 6 };
            double[] levelCollision7 = new double[] { 7, 7, 7, 7, 7, 7 };
            double[] levelCollision8 = new double[] { 8, 8, 8, 8, 8, 8 };

            robot.SetAnticollision(0, levelCollision6, 1);

            #endregion

            // Aspetto che il metodo termini, ma senza bloccare il thread principale
            // La routine è incapsulata come 'async' per supportare futuri operatori 'await' nel caso ci fosse la necessità
            await Task.Run(async () =>
            {
                // Fino a quando la condizione di stop routine non è true e non sono presenti allarmi bloccanti
                while (!token.IsCancellationRequested && !AlarmManager.blockingAlarm && !stopCycleRoutine)
                {
                    switch (step)
                    {
                        case 0:
                            #region Check richiesta interruzione ciclo

                            if (!stopCycleRequested) // Se non è stata richiesta nessuna interruzione
                            {
                                step = 10;
                            }
                            else // Se è stata richiesta l'interruzione
                            {
                                // Ritorno del Robot a casa
                                GoToHomePosition();

                                // Reset inPosition
                                inPosition = false;

                                // Assegnazione del pHome come ending point
                                endingPoint = descPosHome;

                                step = 5;
                            }

                            log.Info("STEP 0 - Check richiesta interruzione ciclo");
                            break;

                        #endregion

                        case 5:
                            #region Termine routine

                            if (inPosition) // Se il Robot è arrivato in HomePosition
                            {
                                // Abilito il tasto Start per avviare nuovamente la routine
                                EnableButtonCycleEvent?.Invoke(1, EventArgs.Empty);

                                // Imposto a false il booleano che fa terminare il thread della routine
                                stopCycleRoutine = true;

                            }

                            log.Info("STEP 5 - Termine routine");
                            break;

                        #endregion

                        case 10:
                            #region Movimento a punto di Pick teglia 1

                            inPosition = false; // Reset inPosition

                            blendR = 50;
                            // Movimento a punto di avvicinamento pick teglia 1
                            movementResult = robot.MoveL(jointPosApproachPickTeglia1, descPosApproachPickTeglia1,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            blendR = 50;
                            // Movimento a pick teglia 1
                            movementResult = robot.MoveL(jointPosPickTeglia1, descPosPickTeglia1,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            endingPoint = descPosPickTeglia1; // Assegnazione endingPoint

                            step = 30;

                            log.Info("STEP 10 - Movimento a punto di Pick teglia 1");

                            break;

                        #endregion

                        case 30:
                            #region Attesa inPosition punto di Pick teglia 1 e chiusura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick 1
                            {
                                // Abilitazione pick
                                robot.SetDO(0, 1, 0, 0);

                                step = 40; // Passaggio a step 40
                            }

                            log.Info("STEP 30 - Attesa inPosition punto di Pick teglia 1 e chiusura pinza");

                            break;

                        #endregion

                        case 40:
                            #region Check chiusura pinza

                            robot.GetDI(0, 1, ref ris);

                            // Se pick done
                            if (ris == 1)
                            {

                                await Task.Delay(100); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza
                                step = 50;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 40 - Check chiusura pinza");

                            break;

                        #endregion

                        case 50:
                            #region Movimento di uscita dal carrello dopo pick teglia 1

                            blendR = 50;
                            // Movimento per uscire dal carrelo dopo pick 1
                            movementResult = robot.MoveL(jointPosPostPickTeglia1, descPosPostPickTeglia1,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            offset = new DescPose(0, 0, 0, 3, 0, 0);
                            // Movimento a punto di avvicinamento place teglia 1
                            movementResult = robot.MoveJ(jointPosPostPickTeglia1, descPosPostPickTeglia1,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);
                            offset = new DescPose(0, 0, 0, 0, 0, 0);

                            blendR = 50;
                            // Movimento per uscire dal carrelo dopo pick 1
                            movementResult = robot.MoveL(jointPosAllontanamentoPickTeglia1, descPosAllontanamentoPickTeglia1,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            log.Info("STEP 50 - Movimento di uscita dal carrello dopo pick teglia 1");

                            step = 60;

                            break;

                        #endregion

                        case 60:
                            #region Ritorno in home e movimento in place teglia 1

                            blendR = 50;
                            // Ritorno in posizione di home
                            movementResult = robot.MoveL(jointPosHome, descPosHome,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            blendR = 50;
                            // Movimento a punto di avvicinamento place teglia 1
                            movementResult = robot.MoveL(jointPosApproachPlaceTeglia1, descPosApproachPlaceTeglia1,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            blendR = 50;
                            // Movimento a  place teglia 1
                            movementResult = robot.MoveL(jointPosPlaceTeglia1, descPosPlaceTeglia1,
                               tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            endingPoint = descPosPlaceTeglia1; // Assegnazione ending point


                            log.Info("STEP 60 - Ritorno in home e movimento in place teglia 1");

                            step = 70;

                            break;

                        #endregion

                        case 70:
                            #region Check arrivo in place teglia 1

                            if (inPosition)
                            {
                                step = 80;
                            }

                            log.Info("STEP 70 - Check arrivo in place teglia 1");
                            break;

                        #endregion

                        case 80:
                            #region Ritorno in home e movimento in place teglia 2

                            log.Info("STEP 80 - Ritorno in home e movimento in place teglia 2");

                            blendR = 50;
                            // Movimento a punto di avvicinamento place teglia 1
                            movementResult = robot.MoveL(jointPosApproachPlaceTeglia1, descPosApproachPlaceTeglia1,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            blendR = 50;
                            // Movimento di rotazione pre place teglia 2
                            movementResult = robot.MoveL(jointPosRotationPrePlaceTeglia2, descPosRotationPrePlaceTeglia2,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            offset = new DescPose(0, 0, 0, 3, 0, 0);
                            // Movimento a punto di avvicinamento place teglia 2
                            movementResult = robot.MoveJ(jointPosApproachPlaceTeglia2, descPosApproachPlaceTeglia2,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);
                            offset = new DescPose(0, 0, 0, 0, 0, 0);

                            blendR = 50;
                            // Movimentoa punto di place teglia 2
                            movementResult = robot.MoveL(jointPosPlaceTeglia2, descPosPlaceTeglia2,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            endingPoint = descPosPlaceTeglia2;

                            step = 90;

                            break;
                        #endregion

                        case 90:
                            #region Attesa inPosition punto di place teglia 2 e apertura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Place teglia 2
                            {
                                // Abilitazione place
                                robot.SetDO(1, 1, 0, 0);

                                step = 100;
                            }

                            log.Info("STEP 90 - Attesa inPosition punto di place teglia 2 e apertura pinza");

                            break;

                        #endregion

                        case 100:
                            #region Check apertura pinza

                            robot.GetDI(1, 1, ref ris);

                            // se place done
                            if (ris == 1)
                            {

                                await Task.Delay(100); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza
                                step = 110;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 100 - Check apertura pinza");

                            break;

                        #endregion

                        case 110:
                            #region Ritorno in home e movimento in pick teglia 2

                            log.Info("STEP 110 - Ritorno in home e movimento in pick teglia 2");

                            blendR = 50;

                            offset = new DescPose(0, 0, 0, -3, 0, 0);
                            // Movimento a punto di post place teglia 2
                            movementResult = robot.MoveJ(jointPosPlaceTeglia2, descPosPlaceTeglia2,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);

                            blendR = 50;
                            // Movimento a punto di allontanamento place teglia 2
                            movementResult = robot.MoveL(jointPosAllontanamentoPlaceTeglia2, descPosAllontanamentoPlaceTeglia2,
                                tool, user, vel, acc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset

                            blendR = 50;
                            // Movimento a punto di avvicinamento pick teglia 2
                            movementResult = robot.MoveL(jointPosApproachPickTeglia2, descPosApproachPickTeglia2,
                                tool, user, vel, acc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            blendR = 50;
                            // Movimento a punto di pick teglia 2
                            movementResult = robot.MoveL(jointPosPickTeglia2, descPosPickTeglia2,
                                tool, user, vel, acc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            endingPoint = descPosPickTeglia2;

                            step = 120;

                            break;
                        #endregion

                        case 120:
                            #region Attesa inPosition punto di Pick teglia 1 e chiusura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick teglia 2
                            {
                                // Abilitazione pick
                                robot.SetDO(0, 1, 0, 0);

                                step = 130;
                            }

                            log.Info("STEP 120 - Attesa inPosition punto di Pick teglia 1 e chiusura pinza");

                            break;

                        #endregion

                        case 130:
                            #region Check chiusura pinza

                            robot.GetDI(0, 1, ref ris);

                            // Se pick done
                            if (ris == 1)
                            {

                                await Task.Delay(100); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza
                                step = 140;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 130 - Check chiusura pinza");

                            break;

                        #endregion

                        case 140:
                            #region Movimento di uscita dal carrello dopo pick teglia 2

                            blendR = 50;
                            // Movimento per uscire dal carrelo dopo pick 2
                            movementResult = robot.MoveL(jointPosPostPickTeglia2, descPosPostPickTeglia2,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            offset = new DescPose(0, 0, 0, 3, 0, 0);
                            // Movimento a portare la teglia inclinata verso l'alto
                            movementResult = robot.MoveJ(jointPosPostPickTeglia2, descPosPostPickTeglia2,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);
                            offset = new DescPose(0, 0, 0, 0, 0, 0);

                            blendR = 50;
                            // Movimento per uscire dal carrelo dopo pick teglia 2
                            movementResult = robot.MoveL(jointPosAllontanamentoPickTeglia2, descPosAllontanamentoPickTeglia2,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            log.Info("STEP 140 - Movimento di uscita dal carrello dopo pick teglia 2");

                            step = 150;

                            break;

                        #endregion

                        case 150:
                            #region Ritorno in home e movimento in place teglia 1

                            blendR = 50;
                            // Ritorno in posizione di home
                            movementResult = robot.MoveL(jointPosHome, descPosHome,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            blendR = 50;
                            // Movimento a punto di avvicinamento place teglia 1
                            movementResult = robot.MoveL(jointPosApproachPlaceTeglia1, descPosApproachPlaceTeglia1,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            blendR = 50;
                            // Movimento a  place teglia 1
                            movementResult = robot.MoveL(jointPosPlaceTeglia1, descPosPlaceTeglia1,
                               tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            endingPoint = descPosPlaceTeglia1; // Assegnazione ending point


                            log.Info("STEP 150 - Ritorno in home e movimento in place teglia 1");

                            step = 160;

                            break;

                        #endregion

                        case 160:
                            #region Check arrivo in place teglia 1

                            log.Info("STEP 160 - Check arrivo in place teglia 1");

                            if (inPosition)
                            {
                                step = 170;
                            }

                            break;

                        #endregion

                        case 170:
                            #region Ritorno in home e movimento in place teglia 3

                            log.Info("STEP 160 - Check arrivo in place teglia 3");

                            blendR = 50;
                            // Movimento a punto di avvicinamento place teglia 1
                            movementResult = robot.MoveL(jointPosApproachPlaceTeglia1, descPosApproachPlaceTeglia1,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);


                            blendR = 50;
                            // Movimento di rotazione pre place teglia 3
                            movementResult = robot.MoveL(jointPosRotationPrePlaceTeglia3, descPosRotationPrePlaceTeglia3,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            offset = new DescPose(0, 0, 0, 3, 0, 0);
                            // Movimento a punto di avvicinamento place teglia 3
                            movementResult = robot.MoveJ(jointPosApproachPlaceTeglia3, descPosApproachPlaceTeglia3,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);
                            offset = new DescPose(0, 0, 0, 0, 0, 0);

                            blendR = 50;
                            // Movimento di rotazione pre place teglia 3
                            movementResult = robot.MoveL(jointPosPlaceTeglia3, descPosPlaceTeglia3,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            endingPoint = descPosPlaceTeglia3;

                            step = 180;

                            break;
                        #endregion

                        case 180:
                            #region Attesa inPosition punto di place teglia 3 e apertura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick 1
                            {
                                // Abilitazione place
                                robot.SetDO(1, 1, 0, 0);

                                step = 190;
                            }

                            log.Info("STEP 180 - Attesa inPosition punto di place teglia 3 e apertura pinza");

                            break;

                        #endregion

                        case 190:
                            #region Check apertura pinza

                            robot.GetDI(1, 1, ref ris);

                            // Se place done
                            if (ris == 1)
                            {

                                await Task.Delay(100); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza
                                step = 200;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 190 - Check apertura pinza");

                            break;

                        #endregion

                        case 200:
                            #region Ritorno in home e riavvio ciclo

                            formDiagnostics.UpdateRobotStepDescription("STEP 200 - Ritorno in home e riavvio ciclo");

                            blendR = 50;

                            offset = new DescPose(0, 0, 0, -3, 0, 0);
                            // Movimento a punto di avvicinamento place teglia 3
                            movementResult = robot.MoveJ(jointPosPlaceTeglia3, descPosPlaceTeglia3,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);

                            blendR = 50;
                            // Movimento a punto di allontanamento place teglia 3
                            movementResult = robot.MoveL(jointPosAllontanamentoPlaceTeglia3, descPosAllontanamentoPlaceTeglia3,
                                tool, user, vel, acc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset

                            blendR = 50;
                            // Movimento a punto di home
                            movementResult = robot.MoveL(jointPosHome, descPosHome,
                                tool, user, vel, acc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            step = 0;

                            break;
                            #endregion

                    }
                    Thread.Sleep(50); // Delay routine
                }
            });

        }

        /// <summary>
        /// Gestisce routine di pick e di place, parte solo se si ha sia il pick che il place. In position non controllati
        /// </summary>
        public static async Task MainCycleFast(CancellationToken token)
        {
            #region Variabili necessarie per funzionamento ciclo

            // Reset condizione di stop ciclo
            stopCycleRoutine = false;

            // Reset richiesta di stop ciclo
            stopCycleRequested = false;

            // Reset step routine
            step = 0;

            stepPick = 0; // Step ciclo di pick
            stopPickRoutine = false; // Segnale di stop della pick routine

            stepPlace = 0; // Step ciclo di place
            stopPlaceRoutine = false; // Segnale di stop della place routine

            // Segnale di pick
            int execPick;

            // Segnale di place
            int execPlace;

            // Consensi di pick
            int enableToPick;

            // Consensi di place
            int enableToPlace;

            ApplicationPositions pick = new ApplicationPositions();
            ApplicationPositions place = new ApplicationPositions();

            byte ris = 0;

            #endregion

            #region Offset spostamenti

            int offsetAllontamento = 850;
            int offsetAvvicinamento = 400;
            int offsetPrePlace = 850;
            int offsetAllontamentoPostPlace = 300;
            int zOffsetPrePickTeglia = 40;
            int zOffsetPostPickTeglia = 40;
            int zOffsetAllontanamentoPostPickTeglia1 = 40;
            int zOffsetPrePlace = 20;

            #endregion

            #region Altre variabili

            int valve1_value = 0; // Valore comando valvola 1
            int valve2_value = 0; // Valore comando valvola 2
            int valve3_value = 0; // Valore comando valvola 3
            int vac1_value = 0; // Valore vacuostato 1
            int vac2_value = 0; // Valore vacuostato 2
            int vac3_value = 0; // Valore vacuostato 3
            bool anyValve = false; // A true se c'è almeno una valvola attiva

            int selectedFormat = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_SelectedFormat));

            int selectedProduct;

            int pointNumber = selectedFormat % 100; //Estrazione ultime 2 cifre
            int indexInGroup = (pointNumber - 1) % 6;
            bool isPuntoCritico = (indexInGroup >= 2 && indexInGroup <= 4); // se è il punto 3, 4 o 5 del piano corrente allora è un punto critico (solo per 3200)

            int xOffsetPick;
            int yOffsetPick;
            int zOffsetPick;

            int xOffsetPlace;
            int yOffsetPlace;
            int zOffsetPlace;

            #endregion

            #region Dichiarazione punti routine pick

            // home
            JointPos jHome = new JointPos();
            var home = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose descPosHome = new DescPose();

            // passaggio
            JointPos jPassaggio = new JointPos();
            var passaggio = ApplicationConfig.applicationsManager.GetPosition("pPassaggio", "RM");
            DescPose descPosePassaggio = new DescPose();

            // pick target
            JointPos jointPosPick = new JointPos();
            DescPose descPosPick = new DescPose();

            // pick avvicinamento
            JointPos jointPosApproachPick = new JointPos();
            DescPose descPosApproachPick = new DescPose();

            // pick allontanamento
            JointPos jointPosRialzoPick = new JointPos();
            DescPose descPoseRialzoPick = new DescPose();

            JointPos jointPosPostPick = new JointPos();
            DescPose descPosPostPick = new DescPose();

            // pick avvicinamento
            JointPos jointPosApproachPlace = new JointPos();
            DescPose descPosApproachPlace = new DescPose();

            // pick allontanamento
            JointPos jointPosAllontanamentoPick = new JointPos();
            DescPose descPosAllontanamentoPick = new DescPose();

            #endregion

            // beor
            JointPos jointPosBeor = new JointPos();
            var beor = ApplicationConfig.applicationsManager.GetPosition("pBeor", "RM");
            DescPose descPosBeor = new DescPose();
           
            #region Dichiarazione dei punti place

            // place target
            JointPos jointPosPlace = new JointPos();
            DescPose descPosPlace = new DescPose();

            JointPos jointPosRotationPrePlace = new JointPos();
            DescPose descPosRotationPrePlace = new DescPose();

            JointPos jointPosAllontanamentoPlace = new JointPos();
            DescPose descPosAllontanamentoPlace = new DescPose();

            #endregion

            #region Punto Beor

            #region Punto Beor

            // Oggetto jointPos
            jointPosBeor = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            descPosBeor = new DescPose(
                beor.x,
                beor.y,
                beor.z,
                beor.rx,
                beor.ry,
                beor.rz
                );

            GetInverseKin(descPosBeor, ref jointPosBeor, "Beor");

            #endregion

            #region Punto di rotazione da pick a beor

            // Oggetto jointPos
            JointPos jointPosRotationPreBeor = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosRotationPreBeor = new DescPose(
                descPosHome.tran.x,
                descPosHome.tran.y,
                descPosHome.tran.z,
                beor.rx,
                beor.ry,
                beor.rz
                );

            GetInverseKin(descPosRotationPreBeor, ref jointPosRotationPreBeor, "Rotazione pre Beor");

            #endregion

            #region Punto avvicinamento beor

            // Oggetto jointPos
            JointPos jointPosApproachBeor = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosApproachBeor = new DescPose(
                beor.x - offsetPrePlace,
                beor.y,
                beor.z,
                beor.rx,
                beor.ry,
                beor.rz
                );

            // Calcolo del jointPos
            GetInverseKin(descPosApproachBeor, ref jointPosApproachBeor, "Avvicinamento beor");

            #endregion

            #region Punto post beor

            // Oggetto jointPos
            JointPos jointPosPostBeor = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosPostBeor = new DescPose(
                beor.x,
                beor.y,
                beor.z,
                beor.rx,
                beor.ry,
                beor.rz
                );

            // Calcolo del jointPos
            GetInverseKin(descPosPostBeor, ref jointPosPostBeor, "Post beor");

            #endregion

            #region Punto allontanamento beor

            // Oggetto jointPos
            JointPos jointPosAllontanamentoBeor = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosAllontanamentoBeor = new DescPose(
                beor.x - offsetAllontamento,
                beor.y,
                beor.z,
                beor.rx,
                beor.ry,
                beor.rz);

            // Calcolo del jointPos
            GetInverseKin(descPosAllontanamentoBeor, ref jointPosAllontanamentoBeor, "Allontanamento beor");

            #endregion

            #endregion

            #region Parametri movimento

            DescPose offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset
            DescPose offsetRotatedPassaggio = new DescPose(0, 0, 0, 0, 0, 90);
            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)
            byte offsetFlagRobot = 1; // Flag per offset in base al robot

            int err1 = 0;
            int err2 = 0;
            int err3 = 0;
            int errKin = 0;

            byte search = 0;
            // Parametri moveL
            int velAccParamMode = 0;
            int overSpeedStrategy = 0;
            int speedPercent = 0;

            #endregion

            try
            {
                // home
                jHome = new JointPos(0, 0, 0, 0, 0, 0);
                descPosHome = new DescPose(home.x, home.y, home.z, home.rx, home.ry, home.rz);
                GetInverseKin(descPosHome, ref jHome, "Home");

               

                #endregion

                if (!collisionManager.ChangeRobotCollision(currentCollisionLevel))
                {
                    throw new Exception("Il comando per aggiornare il ivello di collisioni ha generato un errore");
                }

                // Fino a quando la condizione di stop routine non è true e non sono presenti allarmi bloccanti
                while (!stopCycleRoutine && !AlarmManager.blockingAlarm && !token.IsCancellationRequested)
                {
                    switch (step)
                    {
                        case 0:
                            #region Comunicazione avvio ciclo a PLC e calcolo punto di pick e place
                            // In questo step scrivo al plc che il ciclo di main è stato avviato e passo subito allo step successivo

                            // Aggiorno la variabile globale e statica che scrivo al PLC nel metodo SendUpdatesToPLC 
                            // per informare il plc che il ciclo main è in esecuzione
                            CycleRun_Main = 1;

                            // Controllo di avere sia pick che place da fare
                            // Se ho i consensi calcoli i punti di pick e place prima di partire col ciclo

                            // Get comando di place da plc
                            execPlace = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_Place));
                            // Get consensi di place da plc
                            enableToPlace = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Enable_To_Place));
                            // Get comando di pick da plc
                            execPick = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_Pick));
                            // Get consensi di pick da plc
                            enableToPick = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Enable_To_Pick));

                            if (execPick == 1) // Check richiesta di pick
                            {
                                if (enableToPick == 1 && enableToPlace == 1) // Check consensi
                                {
                                    #region Pick

                                    #region Punto di Pick

                                    // Get punto di pick da PLC
                                    selectedFormat = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_SelectedFormat));
                                    var pPick = ApplicationConfig.applicationsManager.GetPosition(selectedFormat.ToString(), "RM");

                                    if (pPick != null)
                                    {
                                        // Check validità del punto
                                        if (pPick.x != 0 && pPick.y != 0 && pPick.z != 0 && pPick.rx != 0 && pPick.ry != 0 && pPick.rz != 0) // Se il punto è valido
                                        {
                                            pick = pPick;
                                        }
                                        else
                                            throw new Exception("Punto di place non presente nel dizionario");
                                    }
                                    else
                                        throw new Exception("Punto di place non presente nel dizionario");

                                    // place target
                                    jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
                                    descPosPick = new DescPose(
                                        pick.x,
                                        pick.y,
                                        pick.z,
                                        pick.rx,
                                        pick.ry,
                                        pick.rz);

                                    GetInverseKin(descPosPick, ref jointPosPick, "Pick");

                                    #endregion

                                    #region Punto di avvicinamento Pick 

                                    // Oggetto jointPos
                                    jointPosApproachPick = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosApproachPick = new DescPose(
                                        pick.x,
                                        pick.y - offsetAvvicinamento,
                                        pick.z - zOffsetPrePickTeglia,
                                        pick.rx,
                                        pick.ry,
                                        pick.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosApproachPick, ref jointPosApproachPick, "Avvicinamento pick");

                                    #endregion

                                    #region Punto post Pick

                                    // Oggetto jointPos
                                    jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosPostPick = new DescPose(
                                        pick.x,
                                        pick.y,
                                        pick.z + zOffsetPostPickTeglia,
                                        pick.rx,
                                        pick.ry,
                                        pick.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosPostPick, ref jointPosPostPick, "Post pick");

                                    #endregion

                                    #region Punto allontanamento post Pick

                                    // Oggetto jointPos
                                    jointPosAllontanamentoPick = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosAllontanamentoPick = new DescPose(
                                        pick.x,
                                        pick.y - offsetAllontamento,
                                        pick.z + zOffsetAllontanamentoPostPickTeglia1,
                                        pick.rx,
                                        pick.ry,
                                        pick.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosAllontanamentoPick, ref jointPosAllontanamentoPick, "Allontanamento pick");

                                    #endregion

                                    #endregion

                                    #region Place

                                    #region Punto di place

                                    // Oggetto jointPos
                                    jointPosPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Get delle coordinate del punto dal database
                                    place = pick;

                                    // Creazione oggetto descPose
                                    descPosPlace = new DescPose(
                                        place.x,
                                        place.y,
                                        place.z,
                                        place.rx,
                                        place.ry,
                                        place.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosPlace, ref jointPosPlace, "Place");

                                    #endregion

                                    #region Punto avvicinamento place

                                    // Oggetto jointPos
                                    jointPosApproachPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosApproachPlace = new DescPose(
                                        place.x,
                                        place.y - offsetPrePlace,
                                        place.z + 20,
                                        place.rx,
                                        place.ry,
                                        place.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosApproachPlace, ref jointPosApproachPlace, "Avvicinamento place");

                                    #endregion

                                    #region Punto di rotazione pre place

                                    // Oggetto jointPos
                                    jointPosRotationPrePlace = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosRotationPrePlace = new DescPose(
                                        home.x,
                                        home.y,
                                        place.z + zOffsetPrePlace,
                                        place.rx,
                                        place.ry,
                                        place.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosRotationPrePlace, ref jointPosRotationPrePlace, "Rotazione pre place");

                                    #endregion

                                    #region Punto allontanamento place

                                    // Oggetto jointPos
                                    jointPosAllontanamentoPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosAllontanamentoPlace = new DescPose(
                                       place.x,
                                       place.y - offsetAllontamentoPostPlace,
                                       place.z,
                                       place.rx,
                                       place.ry,
                                       place.rz
                                       );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosAllontanamentoPlace, ref jointPosAllontanamentoPlace, "Allontanamento place");

                                    #endregion

                                    #endregion

                                    // Passaggio allo step 10
                                    step = 10;
                                }
                            }

                            break;

                        #endregion

                        case 10:
                            #region Check richiesta routine e consensi
                            // Controllo di avere sia pick che place da fare

                            // Get comando di place da plc
                            execPlace = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_Place));
                            // Get consensi di place da plc
                            enableToPlace = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Enable_To_Place));
                            // Get comando di pick da plc
                            execPick = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_Pick));
                            // Get consensi di pick da plc
                            enableToPick = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Enable_To_Pick));

                            if (execPick == 1) // Check richiesta di pick
                            {
                                if (enableToPick == 1 && enableToPlace == 1) // Check consensi
                                {
                                    step = 40; // Passaggio allo step dedicato alla preparazione dei punti
                                }
                            }

                            break;

                        #endregion

                        case 30:
                            #region Calcolo punto di Pick e di Place

                            if (robotStatus == 1) // Se robot fermo
                            {
                                #region Pick

                                #region Punto di Pick

                                // Get punto di pick da PLC
                                selectedFormat = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_SelectedFormat));
                                var pPick = ApplicationConfig.applicationsManager.GetPosition(selectedFormat.ToString(), "RM");

                                if (pPick != null)
                                {
                                    // Check validità del punto
                                    if (pPick.x != 0 && pPick.y != 0 && pPick.z != 0 && pPick.rx != 0 && pPick.ry != 0 && pPick.rz != 0) // Se il punto è valido
                                    {
                                        pick = pPick;
                                    }
                                    else
                                        throw new Exception("Punto di place non presente nel dizionario");
                                }
                                else
                                    throw new Exception("Punto di place non presente nel dizionario");

                                // place target
                                jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
                                descPosPick = new DescPose(
                                    pick.x, 
                                    pick.y, 
                                    pick.z, 
                                    pick.rx, 
                                    pick.ry, 
                                    pick.rz);

                                GetInverseKin(descPosPick, ref jointPosPick, "Pick");

                                #endregion

                                #region Punto di avvicinamento Pick 

                                // Oggetto jointPos
                                jointPosApproachPick = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosApproachPick = new DescPose(
                                    pick.x,
                                    pick.y - offsetAvvicinamento,
                                    pick.z - zOffsetPrePickTeglia,
                                    pick.rx,
                                    pick.ry,
                                    pick.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosApproachPick, ref jointPosApproachPick, "Avvicinamento pick");

                                #endregion

                                #region Punto post Pick

                                // Oggetto jointPos
                                jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosPostPick = new DescPose(
                                    pick.x,
                                    pick.y,
                                    pick.z + zOffsetPostPickTeglia,
                                    pick.rx,
                                    pick.ry,
                                    pick.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosPostPick, ref jointPosPostPick, "Post pick");

                                #endregion

                                #region Punto allontanamento post Pick

                                // Oggetto jointPos
                                jointPosAllontanamentoPick = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosAllontanamentoPick = new DescPose(
                                    pick.x,
                                    pick.y - offsetAllontamento,
                                    pick.z + zOffsetAllontanamentoPostPickTeglia1,
                                    pick.rx,
                                    pick.ry,
                                    pick.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosAllontanamentoPick, ref jointPosAllontanamentoPick, "Allontanamento pick");

                                #endregion

                                #endregion

                                #region Place

                                #region Punto di place

                                // Oggetto jointPos
                                jointPosPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                // Get delle coordinate del punto dal database
                               place = pick;

                                // Creazione oggetto descPose
                                descPosPlace = new DescPose(
                                    place.x,
                                    place.y,
                                    place.z,
                                    place.rx,
                                    place.ry,
                                    place.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosPlace, ref jointPosPlace, "Place");

                                #endregion

                                #region Punto avvicinamento place

                                // Oggetto jointPos
                                jointPosApproachPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosApproachPlace = new DescPose(
                                    place.x,
                                    place.y - offsetPrePlace,
                                    place.z + 20,
                                    place.rx,
                                    place.ry,
                                    place.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosApproachPlace, ref jointPosApproachPlace, "Avvicinamento place");

                                #endregion

                                #region Punto di rotazione pre place

                                // Oggetto jointPos
                                jointPosRotationPrePlace = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosRotationPrePlace = new DescPose(
                                    home.x,
                                    home.y,
                                    place.z + zOffsetPrePlace,
                                    place.rx,
                                    place.ry,
                                    place.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosRotationPrePlace, ref jointPosRotationPrePlace, "Rotazione pre place");

                                #endregion

                                #region Punto allontanamento place

                                // Oggetto jointPos
                                jointPosAllontanamentoPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                 descPosAllontanamentoPlace = new DescPose(
                                    place.x,
                                    place.y - offsetAllontamentoPostPlace,
                                    place.z,
                                    place.rx,
                                    place.ry,
                                    place.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosAllontanamentoPlace, ref jointPosAllontanamentoPlace, "Allontanamento place");

                                #endregion

                                #endregion

                                step = 40;
                            }

                            break;

                        #endregion

                        case 40:
                            #region Movimento a punto di Pick

                            log.Info("[PICK] invio punto di pick : " + pick.name);

                            CycleRun_Pick = 1;
                            CycleRun_Place = 1;

                            inPosition = false; // Reset inPosition

                            #region Movimento a punto di avvicinamento Pick

                            blendR = 50;
                            // Movimento a punto di avvicinamento pick teglia 1
                            err = robot.MoveL(jointPosApproachPick, descPosApproachPick,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);
                            GetRobotMovementCode(err); // Stampo risultato del movimento

                            #endregion

                            #region Movimento a punto di pick

                            blendR = 50;
                            float slowVel = vel * 0.9f;
                            float slowAcc = acc * 0.75f;

                            // Movimento a pick teglia 1
                            err = robot.MoveL(jointPosPick, descPosPick,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);
                            GetRobotMovementCode(err); // Stampo risultato del movimento

                            #endregion

                            if (err == 99)
                            {
                                await Task.Delay(500);
                            }
                            else
                            {
                                endingPoint = descPosPick; // Assegnazione endingPoint

                                step = 42; // Passaggio allo step successivo
                                stepPick = 10;
                            }

                            break;

                        #endregion

                        case 42:
                            #region Attesa inPosition punto di Pick
                            // In questo step attendo che il robot arrivi nella posizione di pick

                            if (inPosition && robotStatus == 1) // con robot fermo
                            {
                                // Abilitazione pick
                                robot.SetDO(0, 1, 0, 0);

                                step = 43; // Passaggio a step 40
                            }

                            break;

                        #endregion

                        case 43:
                            #region Check pick done

                            robot.GetDI(0, 1, ref ris);

                            // Se pick done
                            if (ris == 1)
                            {

                                await Task.Delay(100); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza
                                step = 50;
                            }

                            break;

                        #endregion

                        case 44:
                            #region Movimento di uscita dal carrello dopo pick e movimento verso beor

                            #region Movimento post Pick

                            blendR = 50;
                            // Movimento per uscire dal carrelo dopo pick 1
                            err = robot.MoveL(jointPosPostPick, descPosPostPick,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento post pick con offset

                            offset = new DescPose(0, 0, 0, 3, 0, 0); // Creazione offset
                            // Movimento a punto di avvicinamento place teglia 1
                            err = robot.MoveJ(jointPosPostPick, descPosPostPick,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);
                            offset = new DescPose(0, 0, 0, 0, 0, 0); // Reset offset

                            #endregion

                            #region Movimento allontanamento pick

                            blendR = 50;
                            // Movimento per uscire dal carrelo dopo pick 1
                            err = robot.MoveL(jointPosAllontanamentoPick, descPosAllontanamentoPick,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento a home

                            blendR = 50;
                            // Ritorno in posizione di home
                            err = robot.MoveL(jHome, descPosHome,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento avvicinamento beor

                            blendR = 50;
                            // Movimento a punto di avvicinamento beor
                            err = robot.MoveL(jointPosApproachBeor, descPosApproachBeor,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento a beor

                            blendR = 50;
                            // Movimento a  beor
                            err = robot.MoveL(jointPosBeor, descPosBeor,
                               tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            endingPoint = descPosBeor;

                            step = 60;

                            break;

                        #endregion

                        case 60:
                            #region Check arrivo in Beor

                            if (inPosition)
                            {
                                step = 65;
                            }

                            break;

                        #endregion

                        case 65:
                            #region Ritorno in home e movimento in place

                            #region Movimento avvicinamento beor

                            blendR = 50;
                            // Movimento a punto di avvicinamento beor
                            err = robot.MoveL(jointPosApproachBeor, descPosApproachBeor,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento rotazione pre place

                            blendR = 50;
                            // Movimento di rotazione pre place teglia 2
                            err = robot.MoveL(jointPosRotationPrePlace, descPosRotationPrePlace,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento a punto di avvicinamento place

                            offset = new DescPose(0, 0, 0, 3, 0, 0);
                            // Movimento a punto di avvicinamento place teglia 2
                            err = robot.MoveJ(jointPosApproachPlace, descPosApproachPlace,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);
                            offset = new DescPose(0, 0, 0, 0, 0, 0);

                            #endregion

                            #region Movimento a place

                            blendR = 50;
                            // Movimento a punto di place teglia 2
                            err = robot.MoveL(jointPosPlace, descPosPlace,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            endingPoint = descPosPlace;

                            step = 90;

                            break;
                        #endregion

                        case 90:
                            #region Attesa inPosition punto di place

                            if (inPosition & robotStatus == 1) // Se il Robot è arrivato in posizione di Place ed è fermo
                            {
                                // Abilitazione place
                                robot.SetDO(1, 1, 0, 0);

                                #region Calcolo nuovo punto di pick e place

                                #region Pick

                                #region Punto di Pick

                                // Get punto di pick da PLC
                                selectedFormat = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_SelectedFormat));
                                var pPick = ApplicationConfig.applicationsManager.GetPosition(selectedFormat.ToString(), "RM");

                                if (pPick != null)
                                {
                                    // Check validità del punto
                                    if (pPick.x != 0 && pPick.y != 0 && pPick.z != 0 && pPick.rx != 0 && pPick.ry != 0 && pPick.rz != 0) // Se il punto è valido
                                    {
                                        pick = pPick;
                                    }
                                    else
                                        throw new Exception("Punto di place non presente nel dizionario");
                                }
                                else
                                    throw new Exception("Punto di place non presente nel dizionario");

                                // place target
                                jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
                                descPosPick = new DescPose(
                                    pick.x,
                                    pick.y,
                                    pick.z,
                                    pick.rx,
                                    pick.ry,
                                    pick.rz);

                                GetInverseKin(descPosPick, ref jointPosPick, "Pick");

                                #endregion

                                #region Punto di avvicinamento Pick 

                                // Oggetto jointPos
                                jointPosApproachPick = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosApproachPick = new DescPose(
                                    pick.x,
                                    pick.y - offsetAvvicinamento,
                                    pick.z - zOffsetPrePickTeglia,
                                    pick.rx,
                                    pick.ry,
                                    pick.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosApproachPick, ref jointPosApproachPick, "Avvicinamento pick");

                                #endregion

                                #region Punto post Pick

                                // Oggetto jointPos
                                jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosPostPick = new DescPose(
                                    pick.x,
                                    pick.y,
                                    pick.z + zOffsetPostPickTeglia,
                                    pick.rx,
                                    pick.ry,
                                    pick.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosPostPick, ref jointPosPostPick, "Post pick");

                                #endregion

                                #region Punto allontanamento post Pick

                                // Oggetto jointPos
                                jointPosAllontanamentoPick = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosAllontanamentoPick = new DescPose(
                                    pick.x,
                                    pick.y - offsetAllontamento,
                                    pick.z + zOffsetAllontanamentoPostPickTeglia1,
                                    pick.rx,
                                    pick.ry,
                                    pick.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosAllontanamentoPick, ref jointPosAllontanamentoPick, "Allontanamento pick");

                                #endregion

                                #endregion

                                #region Place

                                #region Punto di place

                                // Oggetto jointPos
                                jointPosPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                // Get delle coordinate del punto dal database
                                place = pick;

                                // Creazione oggetto descPose
                                descPosPlace = new DescPose(
                                    place.x,
                                    place.y,
                                    place.z,
                                    place.rx,
                                    place.ry,
                                    place.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosPlace, ref jointPosPlace, "Place");

                                #endregion

                                #region Punto avvicinamento place

                                // Oggetto jointPos
                                jointPosApproachPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosApproachPlace = new DescPose(
                                    place.x,
                                    place.y - offsetPrePlace,
                                    place.z + 20,
                                    place.rx,
                                    place.ry,
                                    place.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosApproachPlace, ref jointPosApproachPlace, "Avvicinamento place");

                                #endregion

                                #region Punto di rotazione pre place

                                // Oggetto jointPos
                                jointPosRotationPrePlace = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosRotationPrePlace = new DescPose(
                                    home.x,
                                    home.y,
                                    place.z + zOffsetPrePlace,
                                    place.rx,
                                    place.ry,
                                    place.rz
                                    );

                                // Calcolo del jointPos
                                GetInverseKin(descPosRotationPrePlace, ref jointPosRotationPrePlace, "Rotazione pre place");

                                #endregion

                                #region Punto allontanamento place

                                // Oggetto jointPos
                                jointPosAllontanamentoPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                // Creazione oggetto descPose
                                descPosAllontanamentoPlace = new DescPose(
                                   place.x,
                                   place.y - offsetAllontamentoPostPlace,
                                   place.z,
                                   place.rx,
                                   place.ry,
                                   place.rz
                                   );

                                // Calcolo del jointPos
                                GetInverseKin(descPosAllontanamentoPlace, ref jointPosAllontanamentoPlace, "Allontanamento place");

                                #endregion

                                #endregion

                                #endregion

                                step = 100;
                            }

                            break;

                        #endregion

                        case 100:
                            #region Check place done

                            robot.GetDI(1, 1, ref ris);

                            // se place done
                            if (ris == 1)
                            {
                                await Task.Delay(100); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza
                                step = 110;
                            }

                            break;

                        #endregion

                        case 110:
                            #region Allontanamento place

                            #region Movimento post place

                            blendR = 50;

                            offset = new DescPose(0, 0, 0, -3, 0, 0);
                            // Movimento a punto di post place teglia 2
                            err = robot.MoveJ(jointPosPlace, descPosPlace,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);
                            offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset

                            #endregion

                            #region Movimento allontanamento place

                            blendR = 50;
                            // Movimento a punto di allontanamento place teglia 2
                            err = robot.MoveL(jointPosAllontanamentoPlace, descPosAllontanamentoPlace,
                                tool, user, vel, acc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            step = 10;

                            break;
                            #endregion

                    }

                    await Task.Delay(40); // Delay routine
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] : {ex}");
                throw;
            }
            finally
            {

            }
        }

        /// <summary>
        /// Routine di go to home position
        /// </summary>
        /// <returns></returns>
        public static async Task HomeRoutine(CancellationToken token)
        {

            // Get del punto di home
            var restPose = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose pHome = new DescPose(restPose.x, restPose.y, restPose.z, restPose.rx, restPose.ry, restPose.rz);

            // Get del punto di home
            var safeZone = ApplicationConfig.applicationsManager.GetPosition("pSafeZone", "RM");
            DescPose pSafeZone = new DescPose(safeZone.x, safeZone.y, safeZone.z, safeZone.rx, safeZone.ry, safeZone.rz);

            bool robotDangerousPose = false;

            if (TCPCurrentPosition.tran.y >= pSafeZone.tran.y)
            {
                robotDangerousPose = true;
            }

            stopHomeRoutine = false; // Reset segnale di stop ciclo home
            stepHomeRoutine = 0; // Reset degli step della HomeRoutine

            //robot.RobotEnable(1);
            await Task.Delay(1000);

            try
            {
                while (!stopHomeRoutine && !token.IsCancellationRequested) // Fino a quando non termino la home routine
                {
                    switch (stepHomeRoutine)
                    {
                        case 0:
                            #region Cancellazione coda Robot e disattivazione tasti applicazione
                            CycleRun_Home = 1;

                            SetHomeRoutineSpeed();
                            await Task.Delay(1000);

                            stepHomeRoutine = 5;

                            break;

                        #endregion

                        case 5:
                            #region Movimento a punto di approach home
                            try
                            {
                                if (robotDangerousPose)
                                {
                                    DescPose pApproach = new DescPose(
                                        TCPCurrentPosition.tran.x,
                                        pSafeZone.tran.y,
                                        TCPCurrentPosition.tran.z,
                                        TCPCurrentPosition.rpy.rx, 
                                        TCPCurrentPosition.rpy.ry, 
                                        TCPCurrentPosition.rpy.rz);

                                    GoToApproachHomePosition(pApproach);
                                    endingPoint = pApproach;
                                    stepHomeRoutine = 6;
                                }
                                else
                                {
                                    stepHomeRoutine = 10;
                                }
                                
                            }
                            catch (Exception e)
                            {
                                log.Error("Errore durante movimento robot : " + e.Message);
                                throw;
                            }
                            break;
                        #endregion

                        case 6:
                            #region Attesa in position approach home

                            if (inPosition)
                                stepHomeRoutine = 10;

                            break;
                        #endregion

                        case 10:

                            #region Movimento a punto di home
                            try
                            {
                                //MoveRobotToSafePosition();
                                GoToHomePosition();
                                endingPoint = pHome;

                                stepHomeRoutine = 20;
                            }
                            catch (Exception e)
                            {
                                log.Error("Errore durante movimento robot : " + e.Message);
                                throw;
                            }

                            break;

                        #endregion

                        case 20:
                            #region Attesa inPosition home

                            if (inPosition)
                            {
                                stepHomeRoutine = 99;
                                log.Info("[HOME] robot arrivato in home position");
                            }

                            break;

                        #endregion

                        case 99:
                            #region Termine ciclo e riattivazione tasti applicazione e tasto home 

                            ResetHomeRoutineSpeed();

                            CycleRun_Home = 0;
                            stepHomeRoutine = 0;
                            stopHomeRoutine = true;

                            await Task.Delay(1000);
                           // robot.RobotEnable(0);
                            

                            break;

                            #endregion
                    }

                    await Task.Delay(100); // Delay routine
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskHomeRoutine}: {ex}");
                throw;
            }
            finally
            {
                //previousHomeCommandStatus = false;   // viene resettato da plc il comando dopo 20 secondi
            }
        }

        #endregion

        #region Comandi interfaccia

        /// <summary>
        /// Esegue check su cambio velocità derivante dal plc
        /// </summary>
        private static async Task CheckVelCommand()
        {
            // Get valore variabile di stop ciclo robot
            int velocity = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_OverrideAuto));

            // Check su cambio di stato
            if (velocity != previousVel && velocity >= 1 && velocity <= 100)
            {
                log.Info("[Override speed] Richiesto comando cambio override speed da: " + previousVel + " a : " + velocity);

                await Task.Run(() => RobotDAO.SetRobotVelocity(ConnectionString, Convert.ToInt16(velocity)));
                await Task.Run(() => RobotDAO.SetRobotAcceleration(ConnectionString, Convert.ToInt16(velocity)));

                //Invoco metodo per cambiare etichetta velocità in homePage
                RobotVelocityChanged?.Invoke(velocity, EventArgs.Empty);

                // Aggiornamento della velocità precendete
                previousVel = velocity;

                log.Info("[Override speed] Comando cambio override speed completato");
            }
        }

        /// <summary>
        /// Check su comando di registrazione punto derivante da plc
        /// </summary>
        private static async Task CheckCommandRecordPoint()
        {
            // int recordPointCommand = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.SelectedPointRecordCommandIn));

            int recordPointCommand = 0;

            if (recordPointCommand > 0 && previousRecordPointRequest != recordPointCommand)
            {
                log.Warn("Richiesto comando RECORD su punto: " + recordPointCommand);

                // Registrazione punto 

                DescPose newPoint = await Task.Run(() => RecPoint());
                RecordPoint?.Invoke(null, new RobotPointRecordingEventArgs(recordPointCommand, newPoint));

                //Scrivo sul PLC i nuovi valori
                switch (recordPointCommand)
                {
                    // Punto 1
                    case 1:
                        break;
                    // Punto 2
                    case 2:
                        break;
                    // Punto 3
                    case 3:
                        break;
                    // Punto 4
                    case 4:
                        break;
                    // Altrimenti
                    default:
                        log.Warn($"Tentativo di sovrascrivere il punto: {recordPointCommand}, operazione annullata.");
                        break;
                }

                log.Warn("Comando record point completato");
                previousRecordPointRequest = recordPointCommand;
            }
            else if(recordPointCommand == 0)
            {
                previousRecordPointRequest = 0;
            }
        }

        #endregion

        #region Metodi interni

        private static void AbortTasks()
        {
            taskManager.StopTask(nameof(CheckHighPriority));
            taskManager.StopTask(nameof(CheckLowPriority));
            taskManager.StopTask(TaskHomeRoutine);
        }

        private static void ReStartTasks()
        {

        }

        /// <summary>
        /// Metodo helper che tenta di ricreare e riconnettere l'oggetto robot.
        /// Non è in un loop, viene chiamato dal Guardian quando necessario.
        /// </summary>
        private static async Task AttemptReconnectToRobot()
        {
            try
            {
                log.Warn("[Reconnect] Tentativo di ricreazione dell'oggetto Robot...");

                // 1. Ricrea l'oggetto da zero!
                robot = new Robot();

                // 2. Esegui di nuovo la procedura di connessione RPC e avvio dei thread di libreria
                int rpcResult = robot.RPC(RobotIpAddress);

                if (rpcResult == 0)
                {
                    // Diamo un secondo per stabilizzare
                    await Task.Delay(1000);

                    log.Warn("[Reconnect] Oggetto Robot ricreato.");

                    //Reset stati precedenti
                    lastMode = -1;
                    stableMode = -1;

                    // La prossima iterazione del Guardian lo confermerà.
                    AlarmManager.isRobotConnected = true;
                    //RefresherTask.AddUpdate(PLCTagName.Emergency, 0, "INT16");-----------------------

                    // Faccio ripartire i manager
                    frameManager = new Frames(robot);
                    toolManager = new Tools(robot);
                    collisionManager = new Collisions(robot);

                    SetRobotProperties();

                    // Inizializzazione mode
                    ROBOT_STATE_PKG robot_state_pkg = new ROBOT_STATE_PKG();
                    //robot.GetRobotRealTimeState(ref robot_state_pkg);
                    GetRobotRealTimeState(ref robot_state_pkg);
                    currentRobotMode = robot_state_pkg.robot_mode;
                    isAutomaticMode = currentRobotMode == 0;

                    GetRobotInfo();
                }
                else
                {
                    log.Warn("[Reconnect] Chiamata RPC fallita durante la riconnessione.");
                }
            }
            catch (Exception ex)
            {
                log.Error($"[Reconnect] Errore durante il tentativo di riconnessione: {ex.Message}");
            }
        }

        /// <summary>
        /// Esegue scrittua su plc
        /// </summary>
        private static void SendUpdatesToPLC()
        {
            RefresherTask.AddUpdate(PLCTagName.ApplicationComRobot_active, Convert.ToInt16(AlarmManager.isRobotConnected), "INT16"); // Scrittura comunicazione con robot attiva
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_Cycle_Home, stepHomeRoutine, "INT16"); // Scrittura fase ciclo home a PLC
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_MainCycle, step, "INT16"); // Scrittura fase ciclo main a PLC
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_Cycle_Pick, stepPick, "INT16"); // Scrittura fase ciclo pick a PLC
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_Cycle_Place, stepPlace, "INT16"); // Scrittura fase ciclo place a PLC
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Home, CycleRun_Home, "INT16"); // Scrittura valore avvio/stop ciclo home
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Main, CycleRun_Main, "INT16"); // Scrittura valore avvio/stop ciclo main
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Pick, CycleRun_Pick, "INT16"); // Scrittura valore avvio/stop ciclo pick
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Place, CycleRun_Place, "INT16"); // Scrittura valore avvio/stop ciclo place
            RefresherTask.AddUpdate(PLCTagName.Robot_error, robotError, "INT16"); // Scrittura stato errore del robot
            RefresherTask.AddUpdate(PLCTagName.Robot_enable, robotEnableStatus, "INT16"); // Scrittura stato enable del robot
            RefresherTask.AddUpdate(PLCTagName.Robot_status, robotStatus, "INT16"); // Scrittura stato del robot
            RefresherTask.AddUpdate(PLCTagName.ACT_N_Tool, currentTool, "INT16"); // Scrittura stato del robot
            RefresherTask.AddUpdate(PLCTagName.ACT_N_Frame, currentUser, "INT16"); // Scrittura stato del robot
            RefresherTask.AddUpdate(PLCTagName.ACT_CollisionLevel, currentCollisionLevel, "INT16"); // Scrittura stato del robot

            // RefresherTask.AddUpdate(PLCTagName.Move_InPause, robotMove_inPause, "INT16"); // Scrittura feedback pausa del robot
        }

        /// <summary>
        /// Invoca metodo relativo al cambio di velocità del robot
        /// </summary>
        /// <param name="vel">Velocità impostata al Robot</param>
        public static void TriggerRobotVelocityChangedEvent(int vel)
        {
            RobotVelocityChanged?.Invoke(vel, EventArgs.Empty);
        }

        /// <summary>
        /// Invoca metodo relativo al cambio di modalità del robot
        /// </summary>
        /// <param name="mode"></param>
        public static void TriggerRobotModeChangedEvent(int mode)
        {
            RobotModeChanged?.Invoke(mode, EventArgs.Empty);
        }

        /// <summary>
        /// Registra e restituisce punto posizione attuale del Robot
        /// </summary>
        /// <returns></returns>
        public static DescPose RecPoint()
        {
            DescPose pos = new DescPose();

            // Salvo le posizioni registrate
            robot.GetActualTCPPose(flag, ref pos);

            RoundPositionDecimals(ref pos, 3);

            return pos;
        }

        /// <summary>
        /// Imposta la velocità predefinita per eseguire la home routine
        /// </summary>
        public static void SetHomeRoutineSpeed()
        {
            robot.SetSpeed(homeRoutineSpeed);
        }

        /// <summary>
        /// Resetta la velocità utilizzata per la home routine
        /// </summary>
        public static void ResetHomeRoutineSpeed()
        {
            robot.SetSpeed(robotProperties.Speed);
        }

        /// <summary>
        /// Esegue check su Robot enable
        /// </summary>
        public static async Task CheckIsRobotEnable()
        {
            // Controllo se il robot è abilitato tramite PLC
            isEnabledNow = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.Enable));

            if (isEnabledNow && !prevIsEnable)
            {
                // Abilitazione del robot
                log.Warn("[ENABLE] Richiesta abilitazione robot");
                robot.RobotEnable(1);
                prevIsEnable = true;
                prevIsNotEnable = false; // Resetta lo stato "non abilitato"
                AlarmManager.blockingAlarm = false;
                robotEnableStatus = 1;
                currentIndex = -1;
                log.Warn("[ENABLE] Abilitazione robot completata");
            }
            else if (!isEnabledNow && !prevIsNotEnable)
            {
                // Disabilitazione del robot
                log.Warn("[ENABLE] Richiesta disabilitazione robot");
                robot.StopMotion(); // Cancellazione della coda di punti
                AlarmManager.blockingAlarm = true;
                JogMovement.StopJogRobotTask(); // Ferma il thread di Jog
                await Task.Delay(10);
                robot.RobotEnable(0);
                prevIsNotEnable = true;
                prevIsEnable = false; // Resetta lo stato "abilitato"
                prevIsManual = false;
                pauseCycleRequested = false;
                currentIndex = -1;
                robotEnableStatus = 0;
                UC_FullDragModePage.debugCurrentIndex = -1;
                log.Warn("[ENABLE] Disabilitazione robot completata");
            }
        }
        /// <summary>
        /// Metodo che ferma il robot e cancella la coda di punti
        /// </summary>
        public static void ClearRobotQueue()
        {
            AlarmManager.isRobotMoving = false;
            robot.PauseMotion();
            robot.StopMotion();
        }

        /// <summary>
        /// Creazione di un allarme quando il robot si ferma
        /// </summary>
        /// <param name="id">ID allarme</param>
        /// <param name="description">Descrizione allarme</param>
        /// <param name="timestamp">Timestamp allarme</param>
        /// <param name="device">Device da cui deriva l'allarme</param>
        /// <param name="state">ON-OFF</param>
        public static void CreateRobotAlarm(string id, string description, string timestamp, string device, string state)
        {
            // Solleva l'evento quando il robot si ferma
            OnRobotAlarm(new RobotAlarmsEventArgs(id, description, timestamp, device, state));
        }

        /// <summary>
        /// Metodo che aggiunge alla lista degli allarmi l'allarme
        /// </summary>
        /// <param name="e"></param>
        public static void OnRobotAlarm(RobotAlarmsEventArgs e)
        {
            // Calcola il timestamp Unix in millisecondi
            long unixTimestamp = ((DateTimeOffset)Convert.ToDateTime(e.Timestamp)).ToUnixTimeMilliseconds();

            RobotDAO.SaveRobotAlarm(ConnectionString, Convert.ToInt32(e.Id), e.Description,
                unixTimestamp.ToString(), e.Device, e.State);
            formAlarmPage.AddAlarmToList(e.Id, e.Description, e.Timestamp, e.Device, e.State);
            TriggerAllarmeGenerato();

        }

        /// <summary>
        /// Check su movimento del Robot
        /// </summary>
        /// <param name="updates"></param>
        public static void CheckIsRobotMoving(List<(string key, bool value, string type)> updates)
        {

            if (AlarmManager.isRobotConnected)
            {
                double[] coordNewTCPposition = {
                    Math.Round(TCPCurrentPosition.tran.x, 0),
                    Math.Round(TCPCurrentPosition.tran.y, 0),
                    Math.Round(TCPCurrentPosition.tran.z, 0),
                    Math.Round(TCPCurrentPosition.rpy.rx, 0),
                    Math.Round(TCPCurrentPosition.rpy.ry, 0),
                    Math.Round(TCPCurrentPosition.rpy.rz, 0)
                };

                double[] coordpreviousTCPposition = {
                    Math.Round(previousTCPposition.tran.x, 0),
                    Math.Round(previousTCPposition.tran.y, 0),
                    Math.Round(previousTCPposition.tran.z, 0),
                    Math.Round(previousTCPposition.rpy.rx, 0),
                    Math.Round(previousTCPposition.rpy.ry, 0),
                    Math.Round(previousTCPposition.rpy.rz, 0)
                };

                //TODO: è possibile aggiungere una tolleranza per ridurre ancora la quantità di allarmi generati

                // Confronta gli array arrotondati
                bool sonoUguali = coordNewTCPposition.SequenceEqual(coordpreviousTCPposition);

                if (sonoUguali)
                {
                    if (AlarmManager.isRobotMoving)
                    {
                        AlarmManager.isRobotMoving = false;
                        RobotIsMoving?.Invoke(false, EventArgs.Empty);
                        robotMovingStartTime = null; // Resetta il timer
                    }
                }
                else
                {
                    if (!AlarmManager.isRobotMoving)
                    {
                        // Quando il robot inizia a muoversi, avvia il timer
                        if (robotMovingStartTime == null)
                        {
                            robotMovingStartTime = DateTime.Now;
                        }
                        else if ((DateTime.Now - robotMovingStartTime.Value).TotalSeconds > 2)
                        {
                            // Invoca l'evento solo dopo 1 secondo
                            AlarmManager.isRobotMoving = true;
                            RobotIsMoving?.Invoke(true, EventArgs.Empty);
                            robotMovingStartTime = null; // Resetta il timer dopo l'invocazione
                        }
                    }
                    else
                    {
                        robotMovingStartTime = null; // Resetta il timer se torna falso
                    }
                }
            }

            // Aggiorna la posizione TCP precedente con la posizione TCP attuale
            previousTCPposition.tran.x = TCPCurrentPosition.tran.x;
            previousTCPposition.tran.y = TCPCurrentPosition.tran.y;
            previousTCPposition.tran.z = TCPCurrentPosition.tran.z;
            previousTCPposition.rpy.rx = TCPCurrentPosition.rpy.rx;
            previousTCPposition.rpy.ry = TCPCurrentPosition.rpy.ry;
            previousTCPposition.rpy.rz = TCPCurrentPosition.rpy.rz;

        }

        #endregion

        #region Metodi movimento

        /// <summary>
        /// Metodo che porta il Robot in HomePosition
        /// </summary>
        public static void GoToHomePosition()
        {
            var restPose = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose pHome = new DescPose(restPose.x, restPose.y, restPose.z, restPose.rx, restPose.ry, restPose.rz);

            int result = robot.MoveCart(pHome, tool, user, homeRoutineVel, homeRoutineAcc, ovl, blendT, config);

            GetRobotMovementCode(result);

            if (result != 0)
                throw new Exception("Err code: " + result);
        }

        /// <summary>
        /// Sposta il robot alla posizione definita
        /// </summary>
        /// <param name="target"></param>
        /// <exception cref="Exception"></exception>
        private static void GoToApproachHomePosition(DescPose target)
        {
            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)
            // Parametri moveL
            int velAccParamMode = 0;
            int overSpeedStrategy = 0;
            int speedPercent = 0;
            byte search = 0;

            JointPos jointTarget = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, target, -1, ref jointTarget);

            int result = robot.MoveL(jointTarget, target,
                tool, user, homeRoutineVel, homeRoutineAcc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

            GetRobotMovementCode(result);

            if (result != 0)
                throw new Exception("Err code: " + result);
        }

        #endregion

        #region Metodi helper

        /// <summary>
        /// Legge lo stato di una singola Uscita Digitale (DO) del controllore.
        /// </summary>
        /// <param name="doIndex">L'indice dell'uscita da leggere (da 0 a 15).</param>
        /// <param name="isOn">True se l'uscita è ON, False se è OFF.</param>
        /// <param name="do_state_h"></param>
        /// <param name="do_state_l"></param>
        /// <returns>True se la lettura ha avuto successo, False in caso di errore.</returns>
        public static bool TryGetSingleDOState(int doIndex, out bool isOn, int do_state_h, int do_state_l)
        {
            isOn = false;
            if (doIndex < 0 || doIndex > 15)
            {
                log.Error($"Indice DO non valido: {doIndex}. Deve essere tra 0 e 15.");
                return false;
            }

            if (doIndex <= 7)
            {
                // L'uscita è nel registro "basso" (do_state_l)
                // Usiamo l'operatore AND bitwise per isolare il bit che ci interessa.
                // (1 << doIndex) crea una maschera con un solo '1' nella posizione giusta.
                // Esempio per DO2: la maschera è 00000100 in binario.
                isOn = (do_state_l & (1 << doIndex)) != 0;
            }
            else
            {
                // L'uscita è nel registro "alto" (do_state_h)
                int bitIndexInHighRegister = doIndex - 8; // DO8 è il bit 0, DO9 è il bit 1, etc.
                isOn = (do_state_h & (1 << bitIndexInHighRegister)) != 0;
            }

            return true;
        }

        /// <summary>
        /// Gnerazione di un allarme
        /// </summary>
        /// <param name="maincode"></param>
        /// <param name="subcode"></param>
        public static void GenerateAlarm(int maincode, int subcode)
        {
            DataRow robotAlarm;
            DateTime now;
            long unixTimestamp;
            DateTime dateTime;
            string formattedDate;
            string id, description, timestamp, device, state;

            if (!IsAlarmAlreadySignaled(maincode.ToString() + subcode.ToString()))
            {
                robotAlarm = RobotDAO.GetRobotAlarm(ConnectionString, maincode, subcode);
                if (robotAlarm != null)
                {
                    // Ottieni la data e l'ora attuali
                    now = DateTime.Now;

                    // Calcola il timestamp Unix in millisecondi
                    unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();

                    dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
                    formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

                    if (robotAlarm["id"].ToString() == "")
                    {
                        id = "9999";
                        description = "Generic/Not found";
                        timestamp = formattedDate;
                        device = "Robot";
                        state = "ON";
                    }
                    else
                    {
                        id = robotAlarm["id"].ToString();
                        description = robotAlarm["descr_MainCode"].ToString() + ": " + robotAlarm["descr_SubCode"].ToString();
                        timestamp = formattedDate;
                        device = "Robot";
                        state = "ON";
                    }
                    CreateRobotAlarm(id, description, timestamp, device, state);
                    MarkAlarmAsSignaled(maincode.ToString() + subcode.ToString());
                    log.Warn(robotAlarm["descr_MainCode"].ToString() + ": " + robotAlarm["descr_SubCode"].ToString());
                }
                else
                {
                    // Ottieni la data e l'ora attuali
                    now = DateTime.Now;

                    // Calcola il timestamp Unix in millisecondi
                    unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();

                    dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
                    formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

                    id = "9999";
                    description = "Generic/Not found";
                    timestamp = formattedDate;
                    device = "Robot";
                    state = "ON";

                    CreateRobotAlarm(id, description, timestamp, device, state);
                }

                // Segnalo che è presente un allarme bloccante (allarme robot)
                AlarmManager.blockingAlarm = true;
                robotError = 1;
            }
        }

        /// <summary>
        /// Ottiene le informazioni del robot attraverso i metodi bloccanti della libreria
        /// </summary>
        private static void GetRobotInfo()
        {
            if (AlarmManager.isRobotConnected)
            {
                log.Info("Recupero informazioni del robot");
                robot.GetSDKVersion(ref RobotSdkVer);
                robot.GetControllerIP(ref RobotCurrentIP);
                robot.GetSoftwareVersion(ref RobotModelVer, ref RobotWebVer, ref RobotControllerVer);
                robot.GetFirmwareVersion(ref RobotFwBoxBoardVer, ref RobotFwDriver1Ver, ref RobotFwDriver2Ver, ref RobotFwDriver3Ver,
                    ref RobotFwDriver4Ver, ref RobotFwDriver5Ver, ref RobotFwDriver6Ver, ref RobotFwEndBoardVer);
                robot.GetHardwareVersion(ref RobotHwBoxBoardVer, ref RobotHwDriver1Ver, ref RobotHwDriver2Ver, ref RobotHwDriver3Ver,
                    ref RobotHwDriver4Ver, ref RobotHwDriver5Ver, ref RobotHwDriver6Ver, ref RobotHwEndBoardVer);
            }
        }

        /// <summary>
        /// Check su errori comunicati da PLC
        /// </summary>
        /// <param name="alarmValues"></param>
        /// <param name="alarmDescriptions"></param>
        /// <param name="now"></param>
        /// <param name="unixTimestamp"></param>
        /// <param name="dateTime"></param>
        /// <param name="formattedDate"></param>
        private static void GetPLCErrorCode(
            Dictionary<string, object> alarmValues,
            Dictionary<string, string> alarmDescriptions,
            DateTime now,
            long unixTimestamp,
            DateTime dateTime,
            string formattedDate
            )
        {
            /*
            object alarmsPresent;

            lock (PLCConfig.appVariables)
            {
                alarmsPresent = PLCConfig.appVariables.getValue("PLC1_" + "Alarm present");

                if (Convert.ToBoolean(alarmsPresent))
                {
                    foreach (var key in alarmDescriptions.Keys)
                    {
                        alarmValues[key] = PLCConfig.appVariables.getValue("PLC1_" + key);
                    }
                }
            }
            */
            /*
            try
            {

                foreach (var key in alarmDescriptions.Keys)
                {
                    alarmValues[key] = PLCConfig.appVariables.getValue("PLC1_" + key);
                }

                // if (Convert.ToBoolean(alarmsPresent))
                // {
                now = DateTime.Now;
                unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();
                dateTime = DateTimeOffset.FromUnixTimeMilliseconds(unixTimestamp).DateTime.ToLocalTime();
                formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

                foreach (var alarm in alarmValues)
                {
                    if (Convert.ToBoolean(alarm.Value) && !IsAlarmAlreadySignaled(alarm.Key))
                    {
                        string id = GenerateAlarmId(alarm.Key);
                        CreateRobotAlarm(id, alarmDescriptions[alarm.Key], formattedDate, "PLC", "ON");
                        MarkAlarmAsSignaled(alarm.Key);
                    }
                }
                // }
            }
            catch(Exception ex)
            {

            }
            */
        }

        /// <summary>
        /// Avvisa se un allarme è già stato segnalato
        /// </summary>
        /// <param name="alarmKey"></param>
        /// <returns></returns>
        private static bool IsAlarmAlreadySignaled(string alarmKey)
        {
            return allarmiSegnalati.ContainsKey(alarmKey) && allarmiSegnalati[alarmKey];
        }

        /// <summary>
        /// Imposta l'allarme come segnalato
        /// </summary>
        /// <param name="alarmKey"></param>
        private static void MarkAlarmAsSignaled(string alarmKey)
        {
            if (allarmiSegnalati.ContainsKey(alarmKey))
            {
                allarmiSegnalati[alarmKey] = true;
            }
            else
            {
                allarmiSegnalati.Add(alarmKey, true);
            }
        }

        /// <summary>
        /// Legge allarmi derivanti dal Robot
        /// </summary>
        private static void GetRobotErrorCode()
        {
            DataRow robotAlarm;
            DateTime now;
            string id;
            string description;
            string timestamp;
            string device;
            string state;
            long unixTimestamp;
            DateTime dateTime;
            string formattedDate;

            if (AlarmManager.isRobotConnected)
            {
                err = robot.GetRobotErrorCode(ref maincode, ref subcode);
                if (maincode != 0 && !IsAlarmAlreadySignaled(maincode.ToString() + subcode.ToString()))
                {
                    robotAlarm = RobotDAO.GetRobotAlarm(ConnectionString, maincode, subcode);
                    if (robotAlarm != null)
                    {
                        Console.WriteLine($"mainErrCode {maincode} subErrCode {subcode} ");

                        // Ottieni la data e l'ora attuali
                        now = DateTime.Now;

                        // Calcola il timestamp Unix in millisecondi
                        unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();

                        dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
                        formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

                        if (robotAlarm["id"].ToString() == "")
                        {
                            id = "9999";
                            description = "Generic/Not found";
                            timestamp = formattedDate;
                            device = "Robot";
                            state = "ON";
                        }
                        else
                        {
                            id = robotAlarm["id"].ToString();
                            description = robotAlarm["descr_MainCode"].ToString() + ": " + robotAlarm["descr_SubCode"].ToString();
                            timestamp = formattedDate;
                            device = "Robot";
                            state = "ON";
                        }
                        CreateRobotAlarm(id, description, timestamp, device, state);
                        MarkAlarmAsSignaled(maincode.ToString() + subcode.ToString());
                        log.Warn(robotAlarm["descr_MainCode"].ToString() + ": " + robotAlarm["descr_SubCode"].ToString());
                    }
                    else
                    {
                        // Ottieni la data e l'ora attuali
                        now = DateTime.Now;

                        // Calcola il timestamp Unix in millisecondi
                        unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();

                        dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
                        formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

                        id = "9999";
                        description = "Generic/Not found";
                        timestamp = formattedDate;
                        device = "Robot";
                        state = "ON";

                        CreateRobotAlarm(id, description, timestamp, device, state);
                        MarkAlarmAsSignaled(maincode.ToString() + subcode.ToString());
                        log.Warn($"Allarme generato: Generic/Not found MainCode: {maincode}, SubCode: {subcode}");
                    }

                    // Segnalo che è presente un allarme bloccante (allarme robot)
                    AlarmManager.blockingAlarm = true;
                    robotError = 1;
                }
                else if (maincode == 0)
                {
                    robotError = 0;
                }
            }
        }
        /// <summary>
        /// Reset iniziali delle variabili PLC
        /// </summary>
        private static void ResetPLCVariables()
        {
            /*
            var HomePoint = ApplicationConfig.applicationsManager.GetPosition("1", "RM"); // Get punto di home point
            RefresherTask.AddUpdate(PLCTagName.HomePoint_X, HomePoint.x, "FLOAT"); // Scrittura xCoord punto di home point
            RefresherTask.AddUpdate(PLCTagName.HomePoint_Y, HomePoint.y, "FLOAT"); // Scrittura yCoord punto di home point
            RefresherTask.AddUpdate(PLCTagName.HomePoint_Z, HomePoint.z, "FLOAT"); // Scrittura zCoord punto di home point
            // RefresherTask.AddUpdate(PLCTagName.HomePoint_RX, HomePoint.rx, "FLOAT"); // Scrittura rxCoord punto di home point
            // RefresherTask.AddUpdate(PLCTagName.HomePoint_RY, HomePoint.ry, "FLOAT"); // Scrittura ryCoord punto di home point
            // RefresherTask.AddUpdate(PLCTagName.HomePoint_RZ, HomePoint.rz, "FLOAT"); // Scrittura rzCoord punto di home point

            var pickPoint_Box1 = ApplicationConfig.applicationsManager.GetPosition("101", "RM"); // Get punto di pick box 1
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box1_X, pickPoint_Box1.x, "FLOAT"); // Scrittura xCoord punto di pick box 1
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box1_Y, pickPoint_Box1.y, "FLOAT"); // Scrittura yCoord punto di pick box 1
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box1_Z, pickPoint_Box1.z, "FLOAT"); // Scrittura zCoord punto di pick box 1
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box1_RX, pickPoint_Box1.rx, "FLOAT"); // Scrittura rxCoord punto di pick box 1
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box1_RY, pickPoint_Box1.ry, "FLOAT"); // Scrittura ryCoord punto di pick box 1
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box1_RZ, pickPoint_Box1.rz, "FLOAT"); // Scrittura rzCoord punto di pick box 1

            var pickPoint_Box2 = ApplicationConfig.applicationsManager.GetPosition("201", "RM"); // Get punto di pick box 2
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box2_X, pickPoint_Box2.x, "FLOAT"); // Scrittura xCoord punto di pick box 2
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box2_Y, pickPoint_Box2.y, "FLOAT"); // Scrittura yCoord punto di pick box 2
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box2_Z, pickPoint_Box2.z, "FLOAT"); // Scrittura zCoord punto di pick box 2
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box2_RX, pickPoint_Box2.rx, "FLOAT"); // Scrittura rxCoord punto di pick box 2
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box2_RY, pickPoint_Box2.ry, "FLOAT"); // Scrittura ryCoord punto di pick box 2
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box2_RZ, pickPoint_Box2.rz, "FLOAT"); // Scrittura rzCoord punto di pick box 2

            var pickPoint_Box3 = ApplicationConfig.applicationsManager.GetPosition("301", "RM"); // Get punto di pick box 3
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box3_X, pickPoint_Box3.x, "FLOAT"); // Scrittura xCoord punto di pick box 3
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box3_Y, pickPoint_Box3.y, "FLOAT"); // Scrittura yCoord punto di pick box 3
            RefresherTask.AddUpdate(PLCTagName.PickPoint_Box3_Z, pickPoint_Box3.z, "FLOAT"); // Scrittura zCoord punto di pick box 3
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box3_RX, pickPoint_Box3.rx, "FLOAT"); // Scrittura rxCoord punto di pick box 3
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box3_RY, pickPoint_Box3.ry, "FLOAT"); // Scrittura ryCoord punto di pick box 3
            // RefresherTask.AddUpdate(PLCTagName.PickPoint_Box3_RZ, pickPoint_Box3.rz, "FLOAT"); // Scrittura rzCoord punto di pick box 3

            var PlacePoint_Pallet1_Box1 = ApplicationConfig.applicationsManager.GetPosition("1101", "RM"); // Get punto di place pallet 1 box 1
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box1_X, PlacePoint_Pallet1_Box1.x, "FLOAT"); // Scrittura xCoord punto di place pallet 1 box 1
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box1_Y, PlacePoint_Pallet1_Box1.y, "FLOAT"); // Scrittura yCoord punto di place pallet 1 box 1
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box1_Z, PlacePoint_Pallet1_Box1.z, "FLOAT"); // Scrittura zCoord punto di place pallet 1 box 1
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box1_RX, PlacePoint_Pallet1_Box1.rx, "FLOAT"); // Scrittura rxCoord punto di place pallet 1 box 1
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box1_RY, PlacePoint_Pallet1_Box1.ry, "FLOAT"); // Scrittura ryCoord punto di place pallet 1 box 1
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box1_RZ, PlacePoint_Pallet1_Box1.rz, "FLOAT"); // Scrittura rzCoord punto di place pallet 1 box 1

            var PlacePoint_Pallet1_Box2 = ApplicationConfig.applicationsManager.GetPosition("1201", "RM"); // Get punto di place pallet 1 box 2
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box2_X, PlacePoint_Pallet1_Box2.x, "FLOAT"); // Scrittura xCoord punto di place pallet 1 box 2
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box2_Y, PlacePoint_Pallet1_Box2.y, "FLOAT"); // Scrittura yCoord punto di place pallet 1 box 2
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box2_Z, PlacePoint_Pallet1_Box2.z, "FLOAT"); // Scrittura zCoord punto di place pallet 1 box 2
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box2_RX, PlacePoint_Pallet1_Box2.rx, "FLOAT"); // Scrittura rxCoord punto di place pallet 1 box 2
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box2_RY, PlacePoint_Pallet1_Box2.ry, "FLOAT"); // Scrittura ryCoord punto di place pallet 1 box 2
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box2_RZ, PlacePoint_Pallet1_Box2.rz, "FLOAT"); // Scrittura rzCoord punto di place pallet 1 box 2

            var PlacePoint_Pallet1_Box3 = ApplicationConfig.applicationsManager.GetPosition("1301", "RM"); // Get punto di place pallet 1 box 3
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box3_X, PlacePoint_Pallet1_Box3.x, "FLOAT"); // Scrittura xCoord punto di place pallet 1 box 3
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box3_Y, PlacePoint_Pallet1_Box3.y, "FLOAT"); // Scrittura yCoord punto di place pallet 1 box 3
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box3_Z, PlacePoint_Pallet1_Box3.z, "FLOAT"); // Scrittura zCoord punto di place pallet 1 box 3
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box3_RX, PlacePoint_Pallet1_Box3.rx, "FLOAT"); // Scrittura rxCoord punto di place pallet 1 box 3
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box3_RY, PlacePoint_Pallet1_Box3.ry, "FLOAT"); // Scrittura ryCoord punto di place pallet 1 box 3
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet1_Box3_RZ, PlacePoint_Pallet1_Box3.rz, "FLOAT"); // Scrittura rzCoord punto di place pallet 1 box 3

            var PlacePoint_Pallet2_Box1 = ApplicationConfig.applicationsManager.GetPosition("2101", "RM"); // Get punto di place pallet 2 box 1
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box1_X, PlacePoint_Pallet2_Box1.x, "FLOAT"); // Scrittura xCoord punto di place pallet 2 box 1
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box1_Y, PlacePoint_Pallet2_Box1.y, "FLOAT"); // Scrittura yCoord punto di place pallet 2 box 1
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box1_Z, PlacePoint_Pallet2_Box1.z, "FLOAT"); // Scrittura zCoord punto di place pallet 2 box 1
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box1_RX, PlacePoint_Pallet2_Box1.rx, "FLOAT"); // Scrittura rxCoord punto di place pallet 2 box 1
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box1_RY, PlacePoint_Pallet2_Box1.ry, "FLOAT"); // Scrittura ryCoord punto di place pallet 2 box 1
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box1_RZ, PlacePoint_Pallet2_Box1.rz, "FLOAT"); // Scrittura rzCoord punto di place pallet 2 box 1

            var PlacePoint_Pallet2_Box2 = ApplicationConfig.applicationsManager.GetPosition("2201", "RM"); // Get punto di place pallet 2 box 2
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box2_X, PlacePoint_Pallet2_Box2.x, "FLOAT"); // Scrittura xCoord punto di place pallet 2 box 2
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box2_Y, PlacePoint_Pallet2_Box2.y, "FLOAT"); // Scrittura yCoord punto di place pallet 2 box 2
            RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box2_Z, PlacePoint_Pallet2_Box2.z, "FLOAT"); // Scrittura zCoord punto di place pallet 2 box 2
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box2_RX, PlacePoint_Pallet2_Box2.rx, "FLOAT"); // Scrittura rxCoord punto di place pallet 2 box 2
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box2_RY, PlacePoint_Pallet2_Box2.ry, "FLOAT"); // Scrittura ryCoord punto di place pallet 2 box 2
            // RefresherTask.AddUpdate(PLCTagName.PlacePoint_Pallet2_Box2_RZ, PlacePoint_Pallet2_Box2.rz, "FLOAT"); // Scrittura rzCoord punto di place pallet 2 box 2

            RefresherTask.AddUpdate(PLCTagName.ACT_Step_MainCycle, 0, "INT16"); // Reset fase ciclo a PLC
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_Cycle_Home, 0, "INT16"); // Reset fase ciclo a PLC
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_Cycle_Pick, 0, "INT16"); // Reset fase ciclo a PLC
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_Cycle_Place, 0, "INT16"); // Reset fase ciclo a PLC
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Main, 0, "INT16"); // Reset valore di avvio ciclo main 
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Home, 0, "INT16"); // Reset valore di avvio ciclo home 
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Pick, 0, "INT16"); // Reset valore di avvio ciclo pick 
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Place, 0, "INT16"); // Reset valore di avvio ciclo place 
            */
        }

        /// <summary>
        /// Verifica se il Robot si trova in posizione di Home
        /// </summary>
        private static void CheckIsRobotInHomePosition(DescPose homePose)
        {
            // Calcola lo stato corrente
            isInHomePosition = checker_pos.IsInPosition(homePose, TCPCurrentPosition);

            // Controlla se lo stato è cambiato rispetto al precedente
            if (previousIsInHomePosition == null || isInHomePosition != previousIsInHomePosition)
            {
                //if (stableMode == 2)
                {
                    if (isInHomePosition)
                    {
                        // Aggiorna l'icona della goto home pos in home page
                        RobotInHomePosition?.Invoke(null, EventArgs.Empty);
                    }
                    else
                    {
                        // Aggiorna l'icona della goto home pos in home page
                        RobotNotInHomePosition?.Invoke(null, EventArgs.Empty);
                    }
                }

                // Aggiorna lo stato precedente
                previousIsInHomePosition = isInHomePosition;
            }
        }

        /// <summary>
        /// Esegue check su modalità Robot
        /// </summary>
        /// <summary>
        /// Esegue check su modalità Robot
        /// </summary>
        private static void CheckRobotMode()
        {
            // Ottieni la modalità operativa dal PLC
            mode = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Operating_Mode));

            // Controlla se la modalità è cambiata rispetto all'ultima lettura
            if (mode != lastMode)
            {
                // Aggiorna l'ultima modalità letta e il timestamp
                lastMode = mode;
                lastModeChangeTime = DateTime.Now;
                return; // Aspettiamo che il valore si stabilizzi
            }
            /*
            // Verifica se la modalità è rimasta invariata per almeno 1 secondo
            if (DateTime.Now - lastModeChangeTime < TimeSpan.FromSeconds(1) && mode != stableMode)
            {
                // Modalità confermata stabile: aggiorniamo lo stato
                stableMode = mode;

                // Cambia la modalità del robot in base alla modalità stabile
                if (stableMode == 1 && !prevIsAuto) // Passaggio alla modalità automatica
                { 
                    log.Warn("[Mode] Cambio modalità in AUTO");
                    isAutomaticMode = true;
                    SetRobotMode(0); // Imposta il robot in modalità automatica
                    JogMovement.StopJogRobotTask(); // Ferma il thread di movimento manuale
                    prevIsAuto = true;
                    prevIsManual = false;
                    prevIsOff = false;
                    TriggerRobotModeChangedEvent(1);  // Evento: modalità automatica
                }
                else if (stableMode == 2 && !prevIsManual) // Passaggio alla modalità manuale
                {
                    log.Warn("[Mode] Cambio modalità in MANUAL");
                    isAutomaticMode = false;
                    SetRobotMode(1); // Imposta il robot in modalità manuale
                    prevIsManual = true;
                    prevIsAuto = false;
                    prevIsOff = false;
                    TriggerRobotModeChangedEvent(0);  // Evento: modalità manuale
                }
                else if (stableMode == 0 && !prevIsOff) // Passaggio alla modalità Off
                {
                    log.Warn("[Mode] Cambio modalità in OFF");
                    prevIsOff = true;
                    prevIsAuto = false;
                    prevIsManual = false;
                    TriggerRobotModeChangedEvent(3);  // Evento: modalità Off
                }
            }

            // Esegui logiche aggiuntive come il movimento manuale (Jog)
            if (isEnabledNow && stableMode == 2)
            {
                JogMovement.StartJogRobotTask(); // Avvia il thread di movimento manuale (Jog)
            }*/
            if (DateTime.Now - lastModeChangeTime < TimeSpan.FromSeconds(1))
            {
                return; // Aspetta che il valore del PLC sia stabile
            }
            // CASO A: Il PLC vuole la modalità AUTOMATICA
            if (mode == 1) // 1 = AUTO secondo la tua logica PLC
            {
                // Se il robot NON è GIA' in automatico...
                if (currentRobotMode != 0) // 0 = AUTOMATICO secondo la libreria robot
                {
                    log.Warn("[Mode] Cambio modalità in AUTO");
                    isAutomaticMode = true;
                    SetRobotMode(0); // Imposta il robot in modalità automatica
                    JogMovement.StopJogRobotTask(); // Ferma il thread di movimento manuale
                    prevIsAuto = true;
                    prevIsManual = false;
                    prevIsOff = false;
                    TriggerRobotModeChangedEvent(1);  // Evento: modalità automatica
                }
            }
            // CASO B: Il PLC vuole la modalità MANUALE
            else if (mode == 2) // 2 = MANUALE secondo la tua logica PLC
            {
                // Se il robot NON è GIA' in manuale...
                if (currentRobotMode != 1) // 1 = MANUALE secondo la libreria robot
                {
                    log.Warn("[Mode] Cambio modalità in MANUAL");
                    isAutomaticMode = false;
                    SetRobotMode(1); // Imposta il robot in modalità manuale
                    prevIsManual = true;
                    prevIsAuto = false;
                    prevIsOff = false;
                    TriggerRobotModeChangedEvent(0);  // Evento: modalità manuale
                }

                // La logica per avviare il JOG va qui.
                // Se siamo in manuale (lo siamo, altrimenti saremmo entrati nell'if sopra)
                // e il robot è abilitato, avvia il task di JOG.
                if (isEnabledNow)
                {
                    JogMovement.StartJogRobotTask(); // Questo ha già il controllo per non partire più volte
                }
            }
            // CASO C: Il PLC vuole la modalità OFF o un valore non valido
            else
            {
                if (!prevIsOff)
                {
                    log.Warn("[Mode] Cambio modalità in OFF");
                    isAutomaticMode = false;
                    prevIsOff = true;
                    prevIsAuto = false;
                    prevIsManual = false;
                    TriggerRobotModeChangedEvent(3);  // Evento: modalità Off
                }
            }
        }

        /// <summary>
        /// Legge lo stato del robot
        /// </summary>
        private static void CheckStatusRobot()
        {
            ROBOT_STATE_PKG robot_state_pkg = new ROBOT_STATE_PKG();
            byte mov_robot_state;

            //robot.GetRobotRealTimeState(ref robot_state_pkg);
            GetRobotRealTimeState(ref robot_state_pkg);
            mov_robot_state = robot_state_pkg.robot_state;
            robotStatus = mov_robot_state;
            currentRobotMode = robot_state_pkg.robot_mode;
            currentRobotEnableStatus = robot_state_pkg.rbtEnableState;
        }

        /// <summary>
        /// Esegue reset del contatore degli step delle routine
        /// </summary>
        public static void ResetRobotSteps()
        {
            step = 0;
        }

        /// <summary>
        /// Controlla il tool e user correnti
        /// </summary>
        private static void CheckCurrentToolAndUser()
        {
            robot.GetActualTCPNum(1, ref currentTool);
            robot.GetActualWObjNum(1, ref currentUser);
        }

        /// <summary>
        /// Check su connessione PLC
        /// </summary>
        private static void CheckPLCConnection()
        {
            if (!AlarmManager.isPlcConnected) // Se il PLC è disconnesso
            {
                log.Error("[PLC COM] Rilevata disconnessione PLC");
                string id = "0";
                string description = "PLC disconnesso. Il ciclo è stato terminato.";

                DateTime now = DateTime.Now;
                long unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();
                DateTime dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
                string formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

                string device = "PLC";
                string state = "ON";

                if (!IsAlarmAlreadySignaled(id))
                {
                    CreateRobotAlarm(id, description, formattedDate, device, state);
                    MarkAlarmAsSignaled(id);
                }

                prevIsPlcConnected = false;
            }
            else
            {
                if (!prevIsPlcConnected)
                {
                    log.Warn("[PLC COM] Connessione PLC riavviata");

                    //Reset stati precedenti
                    lastMode = -1;
                    stableMode = -1;

                    robotCycleStopRequested = false;

                    ClearRobotAlarm();
                    ClearRobotQueue();
                    ResetRobotSteps();

                    prevIsPlcConnected = true;
                }
            }
        }
        
        /// <summary>
        /// Verifica se il punto corrente è all'interno dell'area di ingombro rispetto a uno qualsiasi dei punti di partenza
        /// </summary>
        /// <param name="startPoints">Array con i punti di partenza per Pick, Place e Home</param>
        /// <param name="updates">Lista di aggiornamenti</param>
        private static void CheckIsRobotInObstructionArea(DescPose[] startPoints, List<(string key, bool value, string type)> updates)
        {/*
            isInPositionHome = checker_ingombro_home.IsInCubeObstruction(startPoints[0], TCPCurrentPosition);
            isInPositionPick = checker_ingombro_pick.IsInCubeObstruction(startPoints[1], TCPCurrentPosition);
            //isInPositionPlace1 = checker_ingombro_place_pallet_1.IsInCubeObstruction(startPoints[2], TCPCurrentPosition);
            //isInPositionPlace2 = checker_ingombro_place_pallet_2.IsInCubeObstruction(startPoints[3], TCPCurrentPosition);
            isInPositionPlace1 = checker_ingombro_place_pallet_1.IsInParallelepipedObstruction(startPoints[2], TCPCurrentPosition);
            isInPositionPlace2 = checker_ingombro_place_pallet_2.IsInParallelepipedObstruction(startPoints[3], TCPCurrentPosition);

            bool plcIsInPositionHome = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Home_inPos));
            bool plcIsInPositionPick = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Pick_inPos));
            bool plcIsInPositionPlacePallet1 = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Place_1_inPos));
            bool plcIsInPositionPlacePallet2 = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Place_2_inPos));
            */
            /*
            if (isInPositionPick)
            {
                if (/*prevRobotOutPick ||*/ /*!plcIsInPositionPick)
                {
                    prevRobotOutPick = false;
                    prevRobotOutPlace = true;
                    prevInHomePos = true;
                    prevFuoriIngombro = false;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 1, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet1, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet2, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                }
            }
            else if (isInPositionPlace1)
            {
                if (/*prevRobotOutPlace ||*/ /*!plcIsInPositionPlacePallet1)
                {
                    prevRobotOutPlace = false;
                    prevRobotOutPick = true;
                    prevInHomePos = true;
                    prevFuoriIngombro = false;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet1, 1, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet2, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                }
            }
            else if (isInPositionPlace2)
            {
                if (/*prevRobotOutPlace ||*/ /*!plcIsInPositionPlacePallet2)
                {
                    prevRobotOutPlace = false;
                    prevRobotOutPick = true;
                    prevInHomePos = true;
                    prevFuoriIngombro = false;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet1, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet2, 1, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                }
            }

            else if (isInPositionHome)
            {
                if (/*prevInHomePos != false ||*/ /*!plcIsInPositionHome)
                {
                    prevInHomePos = false;
                    prevRobotOutPick = true;
                    prevRobotOutPlace = true;
                    prevFuoriIngombro = false;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet1, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet2, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 1, "INT16");
                }
            }
            else
            {
                bool plcIsFuoriIngombro = !plcIsInPositionHome && !plcIsInPositionPick && !plcIsInPositionPlacePallet1 && !plcIsInPositionPlacePallet2;
                if (/*!prevFuoriIngombro ||*/ /*!plcIsFuoriIngombro)
                {
                    prevFuoriIngombro = true;
                    prevRobotOutPick = true;
                    prevRobotOutPlace = true;
                    prevInHomePos = true;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet1, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos_pallet2, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                }
            }*/
        }
            
        /// <summary>
        /// Verifica se il punto corrente è all'interno dell'area di safe zone
        /// </summary>
        private static void CheckIsRobotInSafeZone(DescPose pSafeZone)
        {
            isInSafeZone = checker_safeZone.IsYLessThan(pSafeZone, TCPCurrentPosition);

            if (!AlarmManager.isFormReady)
                return;

            if (!isInSafeZone && prevIsInSafeZone != false) // Se il robot non è nella safe zone
            {
                prevIsInSafeZone = false;
                FormHomePage.Instance.RobotSafeZone.BackgroundImage = Resources.safeZone_yellow32;

            }
            else if (isInSafeZone && prevIsInSafeZone != true) // Se il robot è nella safe zone
            {
                prevIsInSafeZone = true;
                FormHomePage.Instance.RobotSafeZone.BackgroundImage = Resources.safeZone_green32;

            }

        }

        /// <summary>
        /// Verifica se il punto corrente corrisponde ai punti di pick e/o place
        /// </summary>
        private static void CheckIsRobotInPos()
        {
            bool isInPosition = checker_pos.IsInPosition(endingPoint, TCPCurrentPosition);

            if (isInPosition)
            {
                inPosition = true;
            }
            else
            {
                inPosition = false;
            }

        }

        /// <summary>
        /// Gestore dell'evento allarmi cancellati presente nella libreria RMLib.Alarms
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private static void RMLib_AlarmsCleared(object sender, EventArgs e)
        {
            var criteria = new List<(string device, string description)>
            {
                ("Robot", ""),
                ("", "PLC disconnesso. Il ciclo è stato terminato.")
            };

            bool isBlocking = formAlarmPage.IsBlockingAlarmPresent(criteria);

            if (isBlocking)
            {
                ClearRobotAlarm();
                //ClearRobotQueue();

                // Segnalo che non ci sono più allarmi bloccanti
                AlarmManager.blockingAlarm = false;

                // Abilito il tasto Start per avviare nuovamente la routine
                EnableButtonCycleEvent?.Invoke(1, EventArgs.Empty);

                // Abilito i tasti relativi al monitoring
                EnableDragModeButtons?.Invoke(null, EventArgs.Empty);
            }

            TriggerAllarmeResettato();

            // Reset degli allarmi segnalati
            foreach (var key in allarmiSegnalati.Keys.ToList())
            {
                allarmiSegnalati[key] = false;
            }
        }

        /// <summary>
        /// Esegue get del codice di movimento del robot
        /// </summary>
        /// <param name="result">Codice risultato del movimento del robot</param>
        private static void GetRobotMovementCode(int result)
        {
            if (result != 0) // Se il codice passato come parametro è diverso da 0, significa che il movimento ha generato un errore
            {
                // Get del codice di errore dal database
                DataRow code = RobotDAO.GetRobotMovementCode(ConnectionString, result);

                if (code != null) // Se il codice è presente nel dizionario nel database eseguo la get dei dettagli
                {
                    // Stampo messaggio di errore
                    //CustomMessageBox.Show(
                    //    MessageBoxTypeEnum.ERROR,
                    //    "Errcode: " + code["Errcode"].ToString() + "\nDescribe: " + code["Describe"].ToString() + "\nProcessing method: " + code["Processing method"].ToString()
                    //    );

                    // Scrivo messaggio nel log
                    log.Error("Errcode: " + code["Errcode"].ToString() + "\nDescribe: " + code["Describe"].ToString() + "\nProcessing method: " + code["Processing method"].ToString());
                }
                else // Se il codice non è presente nel dizionario nel database stampo un errore generico
                {
                    //CustomMessageBox.Show(
                    //   MessageBoxTypeEnum.ERROR,
                    //   "Errore generico durante il movimento del robot"
                    //   );

                    log.Error("Errore generico durante il movimento del robot");

                }
            }
        }

        /// <summary>
        /// Imposta le proprietà del robot prelevandole dal database.
        /// </summary>
        /// <returns>True se l'operazione ha successo, altrimenti False.</returns>
        public static bool GetRobotProperties()
        {
            try
            {
                log.Info("Inizio impostazione delle proprietà del robot dal database.");

                // Ottieni le proprietà del robot dal database
                DataTable dt_robotProperties = RobotDAO.GetRobotProperties(ConnectionString);
                if (dt_robotProperties == null)
                {
                    log.Error("La tabella delle proprietà del robot è nulla.");
                    return false;
                }

                // Estrai e assegna le proprietà del robot
                int speed = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_SPEED_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float velocity = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_VELOCITY_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float blendT = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_BLENDT_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float acceleration = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_ACCELERATION_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float ovl = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_OVL_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int tool = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_TOOL_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int user = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_USER_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int weight = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_WEIGHT_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int velRec = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_VELREC_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int collLev = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_COLLISION_LEVELS_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float blendR = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_BLENDR_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());

                // Creazione dell'oggetto robotProperties
                robotProperties = new RobotProperties(speed, velocity, blendT, acceleration, ovl, tool, user, weight, velRec);

                log.Info($"SetRobotProperties completata: " +
                         $" Speed: {speed}" +
                         $" Velocity: {velocity}" + 
                         $" Blend T: {blendT}" +
                         $" Acceleration: {acceleration}" +
                         $" Ovl: {ovl}" +
                         $" Tool: {tool}" +
                         $" User: {user}" +
                         $" Weight: {weight}" +
                         $" VelRec: {velRec}" +
                         $" CollLev: {collLev}" +
                         $" Blend R: {blendR}");

                // Modifica delle variabili statiche e globali di RobotManager
                RobotManager.speed = robotProperties.Speed;
                RobotManager.vel = robotProperties.Velocity;
                RobotManager.acc = robotProperties.Acceleration;
                RobotManager.ovl = robotProperties.Ovl;
                RobotManager.blendT = robotProperties.Blend;
                RobotManager.tool = robotProperties.Tool;
                RobotManager.user = robotProperties.User;
                RobotManager.weight = robotProperties.Weight;
                RobotManager.velRec = robotProperties.VelRec;
                currentCollisionLevel = collLev;
                RobotManager.blendR = blendR;

                return true;
            }
            catch (Exception ex)
            {
                log.Error("Errore durante SetRobotProperties: " + ex.ToString());
                return false;
            }
        }

        /// <summary>
        /// Generazione evento da allarme ricevuto
        /// </summary>
        /// <param name="e"></param>
        protected static void OnAllarmeGenerato(EventArgs e)
        {
            AllarmeGenerato?.Invoke(null, e);
        }

        /// <summary>
        /// Generazione evento da allarmi resettati
        /// </summary>
        /// <param name="e"></param>
        protected static void OnAllarmeResettato(EventArgs e)
        {
            AllarmeResettato?.Invoke(null, e);
        }

        /// <summary>
        /// Generazione eventi
        /// </summary>
        public static void TriggerAllarmeGenerato()
        {
            OnAllarmeGenerato(EventArgs.Empty);
        }

        /// <summary>
        /// Trigger attivato quando vengono cancellati gli allarmi
        /// </summary>
        public static void TriggerAllarmeResettato()
        {
            OnAllarmeResettato(EventArgs.Empty);
        }

        /// <summary>
        /// Normalizza angolo robot
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        static float NormalizeAngle(float angle)
        {
            while (angle > 180f) angle -= 360f;
            while (angle <= -180f) angle += 360f;
            return angle;
        }

        /// <summary>
        /// Invia posizioni al PLC in formato cartesiano e joint
        /// </summary>
        /// <param name="jPos">Posizione in joint ottenuta dal calcolo di cinematica inversa partendo dalla posizione TCP</param>
        public static async Task CheckRobotPosition(JointPos jPos)
        {
            // Calcolo della posizione in joint eseguendo il calcolo di cinematica inversa
            await Task.Run(() => robot.GetInverseKin(0, TCPCurrentPosition, -1, ref jPos));

            #region TCP

            // Scrittura posizione su asse x
            RefresherTask.AddUpdate(PLCTagName.x_actual_pos, TCPCurrentPosition.tran.x, "FLOAT");

            // Scrittura posizione su asse y
            RefresherTask.AddUpdate(PLCTagName.y_actual_pos, TCPCurrentPosition.tran.y, "FLOAT");

            // Scrittura posizione su asse z
            RefresherTask.AddUpdate(PLCTagName.z_actual_pos, TCPCurrentPosition.tran.z, "FLOAT");

            // Scrittura posizione su asse rx
            RefresherTask.AddUpdate(PLCTagName.rx_actual_pos, TCPCurrentPosition.rpy.rx, "FLOAT");

            // Scrittura posizione su asse ry
            RefresherTask.AddUpdate(PLCTagName.ry_actual_pos, TCPCurrentPosition.rpy.ry, "FLOAT");

            // Scrittura posizione su asse rz
            RefresherTask.AddUpdate(PLCTagName.rz_actual_pos, TCPCurrentPosition.rpy.rz, "FLOAT");

            #endregion

            #region Joint

            // Scrittura posizione giunto 1
            RefresherTask.AddUpdate(PLCTagName.j1_actual_pos, jPos.jPos[0], "FLOAT");

            // Scrittura posizione giunto 2
            RefresherTask.AddUpdate(PLCTagName.j2_actual_pos, jPos.jPos[1], "FLOAT");

            // Scrittura posizione giunto 3
            RefresherTask.AddUpdate(PLCTagName.j3_actual_pos, jPos.jPos[2], "FLOAT");

            // Scrittura posizione giunto 4
            RefresherTask.AddUpdate(PLCTagName.j4_actual_pos, jPos.jPos[3], "FLOAT");

            // Scrittura posizione giunto 5
            RefresherTask.AddUpdate(PLCTagName.j5_actual_pos, jPos.jPos[4], "FLOAT");

            // Scrittura posizione giunto 6
            RefresherTask.AddUpdate(PLCTagName.j6_actual_pos, jPos.jPos[5], "FLOAT");

            #endregion
        }

        /// <summary>
        /// Approssima i valori delle posizioni a n cifre decimali
        /// </summary>
        /// <param name="dp">Contiene il riferimento allo struct che contiene i valori da approssimare</param>
        /// <param name="digits">Numero di cifre decimali desiderate</param>
        private static void RoundPositionDecimals(ref DescPose dp, int digits)
        {
            dp.tran.x = Math.Round(dp.tran.x, digits);
            dp.tran.y = Math.Round(dp.tran.y, digits);
            dp.tran.z = Math.Round(dp.tran.z, digits);
            dp.rpy.rx = Math.Round(dp.rpy.rx, digits);
            dp.rpy.ry = Math.Round(dp.rpy.ry, digits);
            dp.rpy.rz = Math.Round(dp.rpy.rz, digits);
        }

        #endregion


    }
}
