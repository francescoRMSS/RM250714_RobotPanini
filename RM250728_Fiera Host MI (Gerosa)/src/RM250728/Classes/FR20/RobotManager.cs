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
using RM.src.RM250728.Forms.Plant;
using RM.Properties;
using RMLib.MessageBox;
using RM.src.RM250728.Forms.DragMode;
using RM.src.RM250728.Classes.PLC;
using RM.src.RM250728.Classes.FR20.Jog;
using RM.src.RM250728.Classes.FR20;
using RM.src.RM250728.Classes.FR20.Properties;
using CookComputing.XmlRpc;

namespace RM.src.RM250728
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
        private readonly static int applicationTaskManagerRefreshPeriod = 200;
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
        public static string TaskPickAndPlaceTeglia3 = nameof(PickAndPlaceTegliaFiera);

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
            robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, 
                "C:\\Users\\WIN10\\source\\repos\\RM250728_Fiera Host MI (Gerosa)\\RM250728_Fiera Host MI (Gerosa)\\bin\\Debug\\Logs", 5, 5);

            robot.RPC(RobotIpAddress);
            AlarmManager.isRobotConnected = true;

            // Faccio partire i manager
            frameManager = new Frames(robot);
            toolManager = new Tools(robot);
            collisionManager = new Collisions(robot);

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

            GetRobotInfo();

            if (err == -4)
            {
                log.Error("RPC exception durante Init del Robot");
                return false;
            }

            // Se fallisce setting della proprietà del Robot
            if (!SetRobotProperties())
                return false;

            robot.SetSpeed(robotProperties.Speed);

            return true;
        }

        #region Task di servizio

        /// <summary>
        /// Gestisce gli ausiliari del Robot
        /// </summary>
        private static async Task AuxiliaryWorker(CancellationToken token)
        {
            // Get del punto di home dal dizionario delle posizioni
            var pHome = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            // Creazione della DescPose del punto di Home
            DescPose homePose = new DescPose(pHome.x, pHome.y, pHome.z, pHome.rx, pHome.ry, pHome.rz);

            try
            {
                while (!token.IsCancellationRequested)
                {
                    if (AlarmManager.isRobotConnected)
                    {
                        await CheckIsRobotEnable();
                        await CheckRobotMode();
                        await CheckCurrentToolAndUser();
                        await CheckGripperStatus();
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

            checker_pos = new PositionChecker(15.0);


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
                    if (robot != null && AlarmManager.isRobotConnected)
                    {
                        try
                        {
                            await Task.Run(() => robot.GetActualTCPPose(flag, ref TCPCurrentPosition)); // Leggo posizione robot TCP corrente
                            CheckIsRobotMoving(updates);
                           // CheckIsRobotInObstructionArea(startPoints, updates);
                            CheckIsRobotInSafeZone(pointSafeZone);
                            CheckIsRobotInHomePosition(pointHome);
                            CheckIsRobotInPos();
                            await CheckStatusRobot();
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
                    await CheckRobotPosition(jPos);

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
                        if (AlarmManager.isRobotConnected)
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

        public async static Task PickAndPlaceTegliaFiera(CancellationToken token)
        {
            // Parametro necessario al comando MoveL
            DescPose offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset

            int offsetAllontamento = 900;
            int offsetApproachPick = 800;
            int offsetSottoTegliaPick = 40;
            int offsetRotPlace = 3;
            int offsetSopraTegliaPlace = 5;
            int offsetZPostPick = 20;
            int offsetRotPostPick = 2;

            #region Punto home

            JointPos jointPosHome = new JointPos(0, 0, 0, 0, 0, 0);

            var home = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");

            DescPose descPosHome = new DescPose(
                home.x, 
                home.y, 
                home.z, 
                home.rx, 
                home.ry, 
                home.rz);

            robot.GetInverseKin(0, descPosHome, -1, ref jointPosHome);

            #endregion

            #region Pick 1

            #region Punto di pick 1

            JointPos jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);

            var pick = ApplicationConfig.applicationsManager.GetPosition("pPickTeglia1", "RM");

            DescPose descPosPick = new DescPose(
                pick.x, 
                pick.y, 
                pick.z, 
                pick.rx, 
                pick.ry, 
                pick.rz);

            robot.GetInverseKin(0, descPosPick, -1, ref jointPosPick);

            #endregion

            #region Punto avvicinamento pick

            JointPos jointPosApproachPick = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosApproachPick = new DescPose(
                pick.x, 
                pick.y - offsetApproachPick, 
                pick.z - offsetSottoTegliaPick, 
                pick.rx, 
                pick.ry, 
                pick.rz);

            robot.GetInverseKin(0, descPosApproachPick, -1, ref jointPosApproachPick);

            #endregion

            #region Punto pre avvicinamento pick

            JointPos jointPosPreApproachPick = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPreApproachPick = new DescPose(
                pick.x,
                pick.y - offsetAllontamento,
                pick.z - offsetSottoTegliaPick,
                pick.rx,
                pick.ry,
                pick.rz);

            robot.GetInverseKin(0, descPosPreApproachPick, -1, ref jointPosPreApproachPick);

            #endregion

            #region Punto post pick

            JointPos jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPostPick = new DescPose(
                 descPosHome.tran.x,
                 descPosHome.tran.y,
                 descPosPick.tran.z + offsetZPostPick, // Mi alzo di 2 cm per uscire dal carrello senza strisciare
                 descPosHome.rpy.rx,
                 NormalizeAngle((float)(descPosPick.rpy.ry - offsetRotPostPick)),
                 descPosHome.rpy.rz
                );

            robot.GetInverseKin(0, descPosPostPick, -1, ref jointPosPostPick);

            #endregion

            #endregion

            #region Place 1

            #region Punto di place 1

            JointPos jointPosPlace = new JointPos(0, 0, 0, 0, 0, 0);

            var place = ApplicationConfig.applicationsManager.GetPosition("pPlaceTeglia1", "RM");

            DescPose descPosPlace = new DescPose(
                place.x, 
                place.y, 
                place.z,
                place.rx,
                place.ry,
                place.rz
                );

            robot.GetInverseKin(0, descPosPlace, -1, ref jointPosPlace);

            #endregion

            #region Punto avvicinamento place

            JointPos jointPosApproachPlace = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosApproachPlace = new DescPose(
                place.x,
                descPosHome.tran.y, 
                place.z + offsetSopraTegliaPlace,
                place.rx, 
                NormalizeAngle(place.ry - offsetRotPlace), 
                place.rz);

            robot.GetInverseKin(0, descPosApproachPlace, -1, ref jointPosApproachPlace);

            #endregion

            #region Punto post place

            JointPos jointPosPostPlace = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPostPlace = new DescPose(
                place.x, 
                place.y, 
                place.z,
                place.rx ,
                NormalizeAngle((float)place.ry + offsetRotPlace),
                place.rz);

            robot.GetInverseKin(0, descPosPostPlace, -1, ref jointPosPostPlace);

            #endregion

            #region Punto allontanamento place

            JointPos jointPosAllontanamentoPlace = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosAllontanamentoPlace = new DescPose(
                place.x, 
                place.y - offsetApproachPick, 
                place.z,
                place.rx ,
                NormalizeAngle((float)place.ry - offsetRotPlace),
                place.rz);

            
            robot.GetInverseKin(0, descPosAllontanamentoPlace, -1, ref jointPosAllontanamentoPlace);

            #endregion

            #endregion

            #region Pick 2

            #region Punto di pick 2

            JointPos jointPosPick2 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPick2 = new DescPose(
                place.x,
                place.y,
                place.z,
                place.rx,
                place.ry,
                place.rz
                );

            robot.GetInverseKin(0, descPosPick2, -1, ref jointPosPick2);

            #endregion

            #region Punto avvicinamento pick 2

            JointPos jointPosApproachPick2 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosApproachPick2 = new DescPose(
                place.x,
                place.y - offsetApproachPick,
                place.z - offsetSottoTegliaPick,
                place.rx,
                place.ry,
                place.rz);

            robot.GetInverseKin(0, descPosApproachPick2, -1, ref jointPosApproachPick2);

            #endregion

            #region Punto pre avvicinamento pick 2

            JointPos jointPosPreApproachPick2 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPreApproachPick2 = new DescPose(
                place.x,
                place.y - offsetAllontamento,
                place.z - offsetSottoTegliaPick,
                place.rx,
                place.ry,
                place.rz);

            robot.GetInverseKin(0, descPosPreApproachPick2, -1, ref jointPosPreApproachPick2);

            #endregion

            #region Punto post pick 2

            JointPos jointPosPostPick2 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPostPick2 = new DescPose(
                 descPosHome.tran.x,
                 descPosHome.tran.y,
                 descPosPick2.tran.z + offsetZPostPick, // Mi alzo di 2 cm per uscire dal carrello senza strisciare
                 descPosHome.rpy.rx,
                 NormalizeAngle((float)(descPosPick2.rpy.ry - offsetRotPostPick)),
                 descPosHome.rpy.rz
                );

            robot.GetInverseKin(0, descPosPostPick2, -1, ref jointPosPostPick2);

            #endregion

            #endregion

            #region Place 2

            #region Punto di place 2

            JointPos jointPosPlace2 = new JointPos(0, 0, 0, 0, 0, 0);

            var place2 = ApplicationConfig.applicationsManager.GetPosition("pPlaceTeglia2", "RM");

            DescPose descPosPlace2 = new DescPose(
                place2.x,
                place2.y,
                place2.z,
                place2.rx,
                place2.ry,
                place2.rz
                );

            robot.GetInverseKin(0, descPosPlace2, -1, ref jointPosPlace2);

            #endregion

            #region Punto avvicinamento place 2

            JointPos jointPosApproachPlace2 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosApproachPlace2 = new DescPose(
                place2.x,
                descPosHome.tran.y,
                place2.z + offsetSopraTegliaPlace,
                place2.rx,
                NormalizeAngle(place2.ry - offsetRotPlace),
                place2.rz);

            robot.GetInverseKin(0, descPosApproachPlace2, -1, ref jointPosApproachPlace2);

            #endregion

            #region Punto post place 2

            JointPos jointPosPostPlace2 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPostPlace2 = new DescPose(
                place2.x,
                place2.y,
                place2.z,
                place2.rx,
                NormalizeAngle((float)place2.ry + offsetRotPlace),
                place2.rz
                );

            robot.GetInverseKin(0, descPosPostPlace2, -1, ref jointPosPostPlace2);

            #endregion

            #region Punto allontanamento place 2

            JointPos jointPosAllontanamentoPlace2 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosAllontanamentoPlace2 = new DescPose(
                place2.x,
                place2.y - offsetApproachPick,
                place2.z,
                place2.rx,
                NormalizeAngle((float)place2.ry - offsetRotPlace),
                place2.rz);


            robot.GetInverseKin(0, descPosAllontanamentoPlace2, -1, ref jointPosAllontanamentoPlace2);

            #endregion

            #endregion

            #region Pick 3

            #region Punto di pick 3

            JointPos jointPosPick3 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPick3 = new DescPose(
                place2.x,
                place2.y,
                place2.z,
                place2.rx,
                place2.ry,
                place2.rz
                );

            robot.GetInverseKin(0, descPosPick3, -1, ref jointPosPick3);

            #endregion

            #region Punto avvicinamento pick 3

            JointPos jointPosApproachPick3 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosApproachPick3 = new DescPose(
                place2.x,
                place2.y - offsetApproachPick,
                place2.z - offsetSottoTegliaPick,
                place2.rx,
                place2.ry,
                place2.rz);

            robot.GetInverseKin(0, descPosApproachPick3, -1, ref jointPosApproachPick3);

            #endregion

            #region Punto pre avvicinamento pick 3

            JointPos jointPosPreApproachPick3 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPreApproachPick3 = new DescPose(
                place2.x,
                place2.y - offsetAllontamento,
                place2.z - offsetSottoTegliaPick,
                place2.rx,
                place2.ry,
                place2.rz);

            robot.GetInverseKin(0, descPosPreApproachPick3, -1, ref jointPosPreApproachPick3);

            #endregion

            #region Punto post pick 3

            JointPos jointPosPostPick3 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPostPick3 = new DescPose(
                 descPosHome.tran.x,
                 descPosHome.tran.y,
                 descPosPick3.tran.z + offsetZPostPick, // Mi alzo di 2 cm per uscire dal carrello senza strisciare
                 descPosHome.rpy.rx,
                 NormalizeAngle((float)(descPosPick3.rpy.ry - offsetRotPostPick)),
                 descPosHome.rpy.rz
                );

            robot.GetInverseKin(0, descPosPostPick3, -1, ref jointPosPostPick3);

            #endregion

            #endregion

            #region Place 3

            #region Punto di place 3

            JointPos jointPosPlace3 = new JointPos(0, 0, 0, 0, 0, 0);

            var place3 = ApplicationConfig.applicationsManager.GetPosition("pPlaceTeglia3", "RM");

            DescPose descPosPlace3 = new DescPose(
                place3.x,
                place3.y,
                place3.z,
                place3.rx,
                place3.ry,
                place3.rz
                );

            robot.GetInverseKin(0, descPosPlace3, -1, ref jointPosPlace3);

            #endregion

            #region Punto avvicinamento place 3

            JointPos jointPosApproachPlace3 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosApproachPlace3 = new DescPose(
                place3.x,
                descPosHome.tran.y,
                place3.z + offsetSopraTegliaPlace,
                place3.rx,
                NormalizeAngle(place3.ry - offsetRotPlace),
                place3.rz);

            robot.GetInverseKin(0, descPosApproachPlace3, -1, ref jointPosApproachPlace3);

            #endregion

            #region Punto post place 3

            JointPos jointPosPostPlace3 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPostPlace3 = new DescPose(
                place3.x,
                place3.y,
                place3.z,
                place3.rx,
                NormalizeAngle((float)place3.ry + offsetRotPlace),
                place3.rz
                );

            robot.GetInverseKin(0, descPosPostPlace3, -1, ref jointPosPostPlace3);

            #endregion

            #region Punto allontanamento place 3

            JointPos jointPosAllontanamentoPlace3 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosAllontanamentoPlace3 = new DescPose(
                place3.x,
                place3.y - offsetApproachPick,
                place3.z,
                place3.rx,
                NormalizeAngle((float)place3.ry - offsetRotPlace),
                place3.rz);


            robot.GetInverseKin(0, descPosAllontanamentoPlace3, -1, ref jointPosAllontanamentoPlace3);

            #endregion

            #endregion

            #region Pick 4

            #region Punto di pick 4

            JointPos jointPosPick4 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPick4 = new DescPose(
                place3.x,
                place3.y,
                place3.z,
                place3.rx,
                place3.ry,
                place3.rz
                );

            robot.GetInverseKin(0, descPosPick4, -1, ref jointPosPick4);

            #endregion

            #region Punto avvicinamento pick 4

            JointPos jointPosApproachPick4 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosApproachPick4 = new DescPose(
                place3.x,
                place3.y - offsetApproachPick,
                place3.z - offsetSottoTegliaPick,
                place3.rx,
                place3.ry,
                place3.rz);

            robot.GetInverseKin(0, descPosApproachPick4, -1, ref jointPosApproachPick4);

            #endregion

            #region Punto pre avvicinamento pick 4

            JointPos jointPosPreApproachPick4 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPreApproachPick4 = new DescPose(
                place3.x,
                place3.y - offsetAllontamento,
                place3.z - offsetSottoTegliaPick,
                place3.rx,
                place3.ry,
                place3.rz);

            robot.GetInverseKin(0, descPosPreApproachPick4, -1, ref jointPosPreApproachPick4);

            #endregion

            #region Punto post pick 4

            JointPos jointPosPostPick4 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPostPick4 = new DescPose(
                 descPosHome.tran.x,
                 descPosHome.tran.y,
                 descPosPick4.tran.z + offsetZPostPick, // Mi alzo di 2 cm per uscire dal carrello senza strisciare
                 descPosHome.rpy.rx,
                 NormalizeAngle((float)(descPosPick4.rpy.ry - offsetRotPostPick)),
                 descPosHome.rpy.rz
                );

            robot.GetInverseKin(0, descPosPostPick4, -1, ref jointPosPostPick4);

            #endregion

            #endregion

            #region Place 4

            #region Punto di place 4

            JointPos jointPosPlace4 = new JointPos(0, 0, 0, 0, 0, 0);

            var place4 = ApplicationConfig.applicationsManager.GetPosition("pPlaceTeglia4", "RM");

            DescPose descPosPlace4 = new DescPose(
                place4.x,
                place4.y,
                place4.z,
                place4.rx,
                place4.ry,
                place4.rz
                );

            robot.GetInverseKin(0, descPosPlace4, -1, ref jointPosPlace4);

            #endregion

            #region Punto avvicinamento place 4

            JointPos jointPosApproachPlace4 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosApproachPlace4 = new DescPose(
                place4.x,
                descPosHome.tran.y,
                place4.z + offsetSopraTegliaPlace,
                place4.rx,
                NormalizeAngle(place4.ry - offsetRotPlace),
                place4.rz);

            robot.GetInverseKin(0, descPosApproachPlace4, -1, ref jointPosApproachPlace4);

            #endregion

            #region Punto post place 4

            JointPos jointPosPostPlace4 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosPostPlace4 = new DescPose(
                place4.x,
                place4.y,
                place4.z,
                place4.rx,
                NormalizeAngle((float)place4.ry + offsetRotPlace),
                place4.rz
                );

            robot.GetInverseKin(0, descPosPostPlace4, -1, ref jointPosPostPlace4);

            #endregion

            #region Punto allontanamento place 4

            JointPos jointPosAllontanamentoPlace4 = new JointPos(0, 0, 0, 0, 0, 0);

            DescPose descPosAllontanamentoPlace4 = new DescPose(
                place4.x,
                place4.y - offsetApproachPick,
                place4.z,
                place4.rx,
                NormalizeAngle((float)place4.ry - offsetRotPlace),
                place4.rz);

            robot.GetInverseKin(0, descPosAllontanamentoPlace4, -1, ref jointPosAllontanamentoPlace4);

            #endregion

            #endregion





            #region Parametri movimento

            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)

            // Indica il codice risultante del movimento del Robot
            int movementResult = -1;

            // Reset condizione di stop ciclo
            stopCycleRoutine = false;

            // Reset richiesta di stop ciclo
            stopCycleRequested = false;

            // Reset step routine
            step = 0;

            // Segnale di pick
            bool prendidaNastro = true;

            #endregion

            double[] levelCollision1 = new double[] { 1, 1, 1, 1, 1, 1 };
            double[] levelCollision2 = new double[] { 2, 2, 2, 2, 2, 2 };
            double[] levelCollision3 = new double[] { 3, 3, 3, 3, 3, 3 };
            double[] levelCollision4 = new double[] { 4, 4, 4, 4, 4, 4 };
            double[] levelCollision5 = new double[] { 5, 5, 5, 5, 5, 5 };
            double[] levelCollision6 = new double[] { 6, 6, 6, 6, 6, 6 };
            double[] levelCollision7 = new double[] { 7, 7, 7, 7, 7, 7 };
            double[] levelCollision8 = new double[] { 8, 8, 8, 8, 8, 8 };

            double[] workCollision = new double[] { 1, 3, 3, 8, 3, 3 };

            robot.SetAnticollision(0, workCollision, 1);

            byte ris = 0;
            try
            {
                while (!token.IsCancellationRequested && !AlarmManager.blockingAlarm)
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 0 - Check richiesta interruzione ciclo");
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 5 - Termine routine");
                            break;

                        #endregion

                        case 10:
                            #region Movimento a punto di Pick

                            inPosition = false; // Reset inPosition

                            // Movimento a punto di avvicinamento pick 1
                            movementResult = robot.MoveL(jointPosApproachPick, descPosApproachPick, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento a pick 1
                            movementResult = robot.MoveL(jointPosPick, descPosPick, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            endingPoint = descPosPick; // Assegnazione endingPoint

                            step = 30; // Passaggio a step 30

                            formDiagnostics.UpdateRobotStepDescription("STEP 10 - Movimento a punto di Pick 1");

                            break;

                        #endregion

                        case 30:
                            #region Attesa inPosition punto di Pick e chiusura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick 1
                            {
                                // Chiudo la pinza
                                await Task.Delay(200);
                                robot.SetDO(0, 1, 0, 0);

                                step = 40; // Passaggio a step 40
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 30 - Attesa inPosition punto di Pick 1 e chiusura pinza");

                            break;

                        #endregion

                        case 40:
                            #region Check chiusura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 0)
                            {
                                await Task.Delay(200);
                                step = 50;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 40 - Check chiusura pinza");

                            break;

                        #endregion

                        case 50:
                            #region Uscita dal carello dopo pick 1

                            // Movimento per uscire dal carrelo dopo pick 1
                            movementResult = robot.MoveL(jointPosPostPick, descPosPostPick, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            formDiagnostics.UpdateRobotStepDescription("STEP 50 - Uscita dal carello dopo pick 1");

                            step = 60;

                            break;

                        #endregion

                        case 60:
                            #region Movimento a punto di place 1

                            inPosition = false; // Reset inPosition

                            // Movimento a punto di avvicinamento place 1
                            movementResult = robot.MoveL(jointPosApproachPlace, descPosApproachPlace, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            // Movimento in place 1
                            movementResult = robot.MoveL(jointPosPlace, descPosPlace, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            endingPoint = descPosPlace; // Assegnazione endingPoint

                            step = 80; // Passaggio a step 80

                            formDiagnostics.UpdateRobotStepDescription("STEP 60 - Movimento a punto di place 1");

                            break;

                        #endregion

                        case 80:
                            #region Attesa inPosition punto di Place 1 e apertura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di place
                            {
                                await Task.Delay(200);
                                robot.SetDO(0, 0, 0, 0); // Apro la pinza
                                step = 85; // Passaggio a step 100
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 80 - Attesa inPosition punto di place 1 e apertura pinza");

                            break;

                        #endregion

                        case 85:
                            #region Check apertura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 1)
                            {
                                await Task.Delay(200);
                                step = 100;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 85 - Check apertura pinza");

                            break;
                        #endregion

                        case 100:
                            #region Movimento per uscire da place 1

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosPostPlace, descPosPostPlace, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosAllontanamentoPlace, descPosAllontanamentoPlace, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            endingPoint = descPosAllontanamentoPlace;

                            formDiagnostics.UpdateRobotStepDescription("STEP 100 - Movimento per uscire da place 1");

                            step = 105;

                            break;

                        #endregion

                        case 105:
                            #region Attesa inPosition uscita da place 1 e movimento a pick 2

                            if (inPosition) // Se il Robot è arrivato in posizione di place 1
                            {
                                inPosition = false; // Reset inPosition

                                // Movimento in posa di pre pick 2
                                movementResult = robot.MoveL(jointPosApproachPick2, descPosApproachPick2, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                                // Movimento in posa di pick 2
                                movementResult = robot.MoveL(jointPosPick2, descPosPick2, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                                endingPoint = descPosPick2; // Assegnazione endingPoint

                                step = 130; // Passaggio a step 130

                                formDiagnostics.UpdateRobotStepDescription("STEP 105 - Attesa inPosition uscita da place 1 e movimento a pick 2");
                            }

                       

                            break;

                        #endregion

                        case 130:
                            #region Attesa inPosition pick 2 e chiusura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di pick 2
                            {
                                await Task.Delay(200);
                                robot.SetDO(0, 1, 0, 0); // Chiudo la pinza

                                step = 140; // Passaggio a step 140
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 130 - Attesa inPosition punto di pick 2 e chiusura pinza");

                            break;

                        #endregion

                        case 140:
                            #region Check chiusura pinza pick 2

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 0)
                            {
                                await Task.Delay(200);
                                step = 150; // Passaggio a step 150
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 140 - Check chiusura pinza pick 2");

                            break;

                        #endregion

                        case 150:
                            #region Movimento per uscire dal carrello dopo il pick 2

                            // Movimento per uscire dal carrello dopo il place 2
                            movementResult = robot.MoveL(jointPosPostPick2, descPosPostPick2, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            formDiagnostics.UpdateRobotStepDescription("STEP 150 - Movimento per uscire dal carrello dopo il pick 2");

                         
                            step = 160;
                            break;
                        #endregion

                        case 160:
                            #region Movimento a punto di Place 2

                            inPosition = false; // Reset inPosition

                            // Movimento a punto di avvicinamento place 1
                            movementResult = robot.MoveL(jointPosApproachPlace2, descPosApproachPlace2, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            // Movimento in place 1
                            movementResult = robot.MoveL(jointPosPlace2, descPosPlace2, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            endingPoint = descPosPlace2; // Assegnazione endingPoint

                            step = 180; // Passaggio a step 80

                            formDiagnostics.UpdateRobotStepDescription("STEP 160 - Movimento a punto di Place 2");

                            break;
                        #endregion

                        case 180:
                            #region Attesa inPosition punto di Place 2 e apertura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick
                            {
                                await Task.Delay(200);
                                robot.SetDO(0, 0, 0, 0); // Apro la pinza

                                step = 185;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 180 - Attesa inPosition punto di place 2 e apertura pinza");
                            break;

                        #endregion

                        case 185:
                            #region Check apertura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 1)
                            {
                                await Task.Delay(200);
                                step = 190;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 85 - Check apertura pinza");

                            break;
                        #endregion

                        case 190:
                            #region Movimento per uscire da place 2

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosPostPlace2, descPosPostPlace2, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosAllontanamentoPlace2, descPosAllontanamentoPlace2, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            endingPoint = descPosAllontanamentoPlace2;

                            formDiagnostics.UpdateRobotStepDescription("STEP 190 - Movimento per uscire da place 2");

                            step = 200;

                            break;
                        #endregion

                        case 200:
                            #region Attesa inPosition uscita da place 2 e movimento a pick 3

                            if (inPosition) // Se il Robot è arrivato in posizione di place 1
                            {
                                inPosition = false; // Reset inPosition

                                // Movimento in posa di pre pick 2
                                movementResult = robot.MoveL(jointPosApproachPick3, descPosApproachPick3, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                                // Movimento in posa di pick 2
                                movementResult = robot.MoveL(jointPosPick3, descPosPick3, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                                endingPoint = descPosPick3; // Assegnazione endingPoint

                                step = 210;

                                formDiagnostics.UpdateRobotStepDescription("STEP 200 - Attesa inPosition uscita da place 2 e movimento a pick 3");
                            }

                            break;

                        #endregion

                        case 210:
                            #region Attesa inPosition pick 3 e chiusura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di pick 2
                            {
                                await Task.Delay(200);
                                robot.SetDO(0, 1, 0, 0); // Chiudo la pinza

                                step = 220; // Passaggio a step 140
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 210 - Attesa inPosition punto di pick 3 e chiusura pinza");

                            break;

                        #endregion

                        case 220:
                            #region Check chiusura pinza pick 3

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 0)
                            {
                                await Task.Delay(200);
                                step = 230; // Passaggio a step 150
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 220 - Check chiusura pinza pick 3");

                            break;

                        #endregion

                        case 230:
                            #region Movimento per uscire dal carrello dopo il pick 3

                            // Movimento per uscire dal carrello dopo il place 3
                            movementResult = robot.MoveL(jointPosPostPick3, descPosPostPick3, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            formDiagnostics.UpdateRobotStepDescription("STEP 230 - Movimento per uscire dal carrello dopo il pick 3");


                            step = 240;
                            break;
                        #endregion

                        case 240:
                            #region Movimento a punto di Place 3

                            inPosition = false; // Reset inPosition

                            // Movimento a punto di avvicinamento place 1
                            movementResult = robot.MoveL(jointPosApproachPlace3, descPosApproachPlace3, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            // Movimento in place 1
                            movementResult = robot.MoveL(jointPosPlace3, descPosPlace3, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            endingPoint = descPosPlace3; // Assegnazione endingPoint

                            step = 250; // Passaggio a step 80

                            formDiagnostics.UpdateRobotStepDescription("STEP 240 - Movimento a punto di Place 3");

                            break;
                        #endregion

                        case 250:
                            #region Attesa inPosition punto di Place 3 e apertura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick
                            {
                                await Task.Delay(200);
                                robot.SetDO(0, 0, 0, 0); // Apro la pinza

                                step = 255;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 250 - Attesa inPosition punto di place 3 e apertura pinza");
                            break;

                        #endregion

                        case 255:
                            #region Check apertura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 1)
                            {
                                await Task.Delay(200);
                                step = 260;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 260 - Check apertura pinza");

                            break;
                        #endregion

                        case 260:
                            #region Movimento per uscire da place 3

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosPostPlace3, descPosPostPlace3, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosAllontanamentoPlace3, descPosAllontanamentoPlace3, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            endingPoint = descPosAllontanamentoPlace3;

                            formDiagnostics.UpdateRobotStepDescription("STEP 260 - Movimento per uscire da place 3");

                            step = 270;

                            break;
                        #endregion

                        case 270:
                            #region Attesa inPosition uscita da place 3 e movimento a pick 4

                            if (inPosition) 
                            {
                                inPosition = false; // Reset inPosition

                                // Movimento in posa di pre pick 2
                                movementResult = robot.MoveL(jointPosApproachPick4, descPosApproachPick4, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                                // Movimento in posa di pick 2
                                movementResult = robot.MoveL(jointPosPick4, descPosPick4, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                                endingPoint = descPosPick4; // Assegnazione endingPoint

                                step = 280;

                                formDiagnostics.UpdateRobotStepDescription("STEP 270 - Attesa inPosition uscita da place 3 e movimento a pick 4");
                            }

                            break;

                        #endregion

                        case 280:
                            #region Attesa inPosition pick 3 e chiusura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di pick 2
                            {
                                await Task.Delay(200);
                                robot.SetDO(0, 1, 0, 0); // Chiudo la pinza

                                step = 290; // Passaggio a step 140
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 280 - Attesa inPosition punto di pick 4 e chiusura pinza");

                            break;

                        #endregion

                        case 290:
                            #region Check chiusura pinza pick 4

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 0)
                            {
                                await Task.Delay(200);
                                step = 300; // Passaggio a step 150
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 290 - Check chiusura pinza pick 4");

                            break;

                        #endregion

                        case 300:
                            #region Movimento per uscire dal carrello dopo il pick 3

                            // Movimento per uscire dal carrello dopo il place 3
                            movementResult = robot.MoveL(jointPosPostPick4, descPosPostPick4, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            formDiagnostics.UpdateRobotStepDescription("STEP 300 - Movimento per uscire dal carrello dopo il pick 4");


                            step = 310;
                            break;
                        #endregion

                        case 310:
                            #region Movimento a punto di Place 4

                            inPosition = false; // Reset inPosition

                            // Movimento a punto di avvicinamento place 1
                            movementResult = robot.MoveL(jointPosApproachPlace4, descPosApproachPlace4, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            // Movimento in place 1
                            movementResult = robot.MoveL(jointPosPlace4, descPosPlace4, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            endingPoint = descPosPlace4; // Assegnazione endingPoint

                            step = 320; // Passaggio a step 80

                            formDiagnostics.UpdateRobotStepDescription("STEP 310 - Movimento a punto di Place 4");

                            break;
                        #endregion

                        case 320:
                            #region Attesa inPosition punto di Place 3 e apertura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick
                            {
                                await Task.Delay(200);
                                robot.SetDO(0, 0, 0, 0); // Apro la pinza

                                step = 325;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 320 - Attesa inPosition punto di place 4 e apertura pinza");
                            break;

                        #endregion

                        case 325:
                            #region Check apertura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 1)
                            {
                                await Task.Delay(200);
                                step = 330;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 325 - Check apertura pinza");

                            break;
                        #endregion

                        case 330:
                            #region Movimento per uscire da place 4

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosPostPlace4, descPosPostPlace4, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosAllontanamentoPlace4, descPosAllontanamentoPlace4, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            endingPoint = descPosAllontanamentoPlace4;

                            formDiagnostics.UpdateRobotStepDescription("STEP 330 - Movimento per uscire da place 4");

                            step = 340;

                            break;
                        #endregion

                        case 340:
                            #region Ritorno in home

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPosHome, descPosHome, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento
                           
                            endingPoint = descPosHome;

                            formDiagnostics.UpdateRobotStepDescription("STEP 340 - Ritorno in home");

                            step = 350;

                            break;
                        #endregion

                        case 350:
                            #region Riavvio ciclo

                           if (inPosition)
                            {
                                step = 0;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 350 - Riavvio ciclo");

                           

                            break;
                            #endregion
                    }

                    await Task.Delay(50, token);
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskPickAndPlaceTeglia3}: {ex}");
                throw;
            }
            finally
            {

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
        /// Routine di go to home position
        /// </summary>
        /// <returns></returns>
        public static async Task HomeRoutine(CancellationToken token)
        {
            /*
            int frameErr = frameManager.ChangeRobotFrame("frNastro");
            if (frameManager.IsErrorBlocking(frameErr))
            {
                GenerateAlarm(0, 1);
                return;
            }
            int toolErr = toolManager.ChangeRobotTool("tVentosa");
            if (toolManager.IsErrorBlocking(toolErr))
            {
                GenerateAlarm(0, 2);
                return;
            }
            */

            // Get del punto di home
            var restPose = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose pHome = new DescPose(restPose.x, restPose.y, restPose.z, restPose.rx, restPose.ry, restPose.rz);

            // Get del punto di home
            var safeZone = ApplicationConfig.applicationsManager.GetPosition("pSafeZone", "RM");
            DescPose pSafeZone = new DescPose(safeZone.x, safeZone.y, safeZone.z, safeZone.rx, safeZone.ry, safeZone.rz);

            int offsetDangerousPose = 300;
            bool robotDangerousPose = false;

            if (TCPCurrentPosition.tran.y >= pSafeZone.tran.y)
            {
                robotDangerousPose = true;
            }

            stopHomeRoutine = false; // Reset segnale di stop ciclo home
            stepHomeRoutine = 0; // Reset degli step della HomeRoutine

            robot.RobotEnable(1);
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
                            robot.RobotEnable(0);
                            

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
            if (velocity != previousVel && velocity >= 0 && velocity <= 100)
            {
                log.Warn("Richiesto comando cambio override speed da: " + previousVel + " a : " + velocity);

                await Task.Run(() => RobotDAO.SetRobotVelocity(ConnectionString, Convert.ToInt16(velocity)));
                await Task.Run(() => RobotDAO.SetRobotAcceleration(ConnectionString, Convert.ToInt16(velocity)));

                //Invoco metodo per cambiare etichetta velocità in homePage
                RobotVelocityChanged?.Invoke(velocity, EventArgs.Empty);

                // Aggiornamento della velocità precendete
                previousVel = velocity;

                log.Warn("Comando cambio override speed completato");
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

                    await Task.Run(() => robot.SetSpeed(speedRobot));

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

                    await Task.Run(() => GetRobotInfo());
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
            vel = homeRoutineVel;
            acc = homeRoutineAcc;
        }

        /// <summary>
        /// Resetta la velocità utilizzata per la home routine
        /// </summary>
        public static void ResetHomeRoutineSpeed()
        {
            robot.SetSpeed(robotProperties.Speed);
            vel = robotProperties.Velocity;
            acc = robotProperties.Acceleration;
        }

        /// <summary>
        /// Metodo per reset errori Robot
        /// </summary>
        public static void ClearRobotAlarm()
        {
            int err = robot.ResetAllError();
        }

        /// <summary>
        /// Imposta la modalità operativa del robot: 
        /// <para>0 = automatico</para>
        /// <para>1 = manuale</para>
        /// </summary>
        /// <param name="mode"></param>
        public static void SetRobotMode(int mode)
        {
            if (mode != 0 && mode != 1)
                return;


            //if(isAutomaticMode && mode == 1) 
            //  RequestStopCycle();

            robot.Mode(mode);
            isAutomaticMode = mode == 0;
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
                await Task.Run(() => robot.RobotEnable(1));
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
                await Task.Run(() => robot.StopMotion()); // Cancellazione della coda di punti
                AlarmManager.blockingAlarm = true;
                JogMovement.StopJogRobotThread(); // Ferma il thread di Jog
                await Task.Delay(10);
                await Task.Run(() => robot.RobotEnable(0));
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
        /// Metodo che mette in pausa il Robot
        /// </summary>
        public static void PauseMotion()
        {
            robot.PauseMotion();
            log.Warn("Il robot è stato messo in pausa");
        }

        /// <summary>
        /// Metodo che riprende movimento Robot
        /// </summary>
        public static void ResumeMotion()
        {
            robot.ResumeMotion();
        }

        /// <summary>
        /// Metodo che porta il Robot in HomePosition
        /// </summary>
        public static void GoToHomePosition()
        {
            var restPose = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose pHome = new DescPose(restPose.x, restPose.y, restPose.z, restPose.rx, restPose.ry, restPose.rz);

            int result = robot.MoveCart(pHome, tool, user, vel, acc, ovl, blendT, config);

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

            JointPos jointTarget = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, target, -1, ref jointTarget);

            int result = robot.MoveL(jointTarget, target, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

            GetRobotMovementCode(result);

            if (result != 0)
                throw new Exception("Err code: " + result);
        }

        #endregion

        #region Metodi helper

        /// <summary>
        /// Calcola la  pose in cui mettere la scatola nel pallet. <br/>
        /// Il calcolo si basa sulla divisione del pallet (messo per il lato lungo in basso) in una matrice di place in questa maniera: <br/>
        /// /  (Larghezza) = Divisione in colonne <br/>
        /// -- (Lunghezza) = Divisione in righe <br/>
        /// I  (Altezza)   = Divisione in strati <br/>
        /// Inoltre sono necessiarie le misure Larg,Lung,Alt della scatola <br/>
        /// NB: le scatole devono essere necessariamente tutte grandi uguali nello stesso pallet <br/>
        /// </summary>
        /// <param name="riga">Riga che identifica il posto in cui mettere la nuova scatola</param>
        /// <param name="colonna">Colonna che identifica il posto in cui mettere la nuova scatola</param>
        /// <param name="strato">Strato in cui mettere la nuova scatola</param>
        /// <param name="larghezzaScatola">Larghezza della scatola(mm)</param>
        /// <param name="profonditaScatola">Lunghezza della scatola(mm)</param>
        /// <param name="altezzaScatola">Altezza della scatola(mm)</param>
        /// <param name="originePallet">Punto di origine del pallet, cioè il punto dell'angolo in alto a sinistra. Posizionare il TCP proprio sopra di esso.</param>
        /// <returns></returns>
        static DescPose CalcolaPosizioneScatola(
            int riga,
            int colonna,
            int strato,
            double larghezzaScatola,
            double profonditaScatola,
            double altezzaScatola,
            DescPose originePallet
            )
        {
            double x = originePallet.tran.x + (colonna * larghezzaScatola) + (larghezzaScatola / 2.0);
            double y = originePallet.tran.y + (riga * profonditaScatola) + (profonditaScatola / 2.0);
            double z = originePallet.tran.z + (strato * altezzaScatola);

            double rx = originePallet.rpy.rx;
            double ry = originePallet.rpy.ry;
            double rz = originePallet.rpy.rz;

            return new DescPose(x, y, z, rx, ry, rz);
        }

        /// <summary>
        /// Gnerazione di un allarme
        /// </summary>
        /// <param name="maincode"></param>
        /// <param name="subcode"></param>
        private static void GenerateAlarm(int maincode, int subcode)
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
        private static async Task CheckRobotMode()
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

            // Verifica se la modalità è rimasta invariata per almeno 1 secondo
            if (DateTime.Now - lastModeChangeTime >= TimeSpan.FromSeconds(2) && mode != stableMode)
            {
                // Modalità confermata stabile: aggiorniamo lo stato
                stableMode = mode;

                // Cambia la modalità del robot in base alla modalità stabile
                if (stableMode == 1 && !prevIsAuto) // Passaggio alla modalità automatica
                {
                    isAutomaticMode = true;
                    await Task.Run(() => SetRobotMode(0)); // Imposta il robot in modalità automatica
                    JogMovement.StopJogRobotThread(); // Ferma il thread di movimento manuale
                    prevIsAuto = true;
                    prevIsManual = false;
                    prevIsOff = false;
                    TriggerRobotModeChangedEvent(1);  // Evento: modalità automatica
                    log.Warn("[Mode] Cambio modalità in AUTO");
                }
                else if (stableMode == 2 && !prevIsManual) // Passaggio alla modalità manuale
                {
                    isAutomaticMode = false;
                    await Task.Run(() => SetRobotMode(1)); // Imposta il robot in modalità manuale
                    prevIsManual = true;
                    prevIsAuto = false;
                    prevIsOff = false;
                    TriggerRobotModeChangedEvent(0);  // Evento: modalità manuale
                    log.Warn("[Mode] Cambio modalità in MANUAL");
                }
                else if (stableMode == 0 && !prevIsOff) // Passaggio alla modalità Off
                {
                    prevIsOff = true;
                    prevIsAuto = false;
                    prevIsManual = false;
                    TriggerRobotModeChangedEvent(3);  // Evento: modalità Off
                    log.Warn("[Mode] Cambio modalità in OFF");
                }
            }

            // Esegui logiche aggiuntive come il movimento manuale (Jog)
            if (isEnabledNow && stableMode == 2)
            {
                JogMovement.StartJogRobotThread(); // Avvia il thread di movimento manuale (Jog)
            }
        }

        /// <summary>
        /// Legge lo stato del robot
        /// </summary>
        private static async Task CheckStatusRobot()
        {
            ROBOT_STATE_PKG robot_state_pkg = new ROBOT_STATE_PKG();
            byte mov_robot_state = 0;

            await Task.Run(() => robot.GetRobotRealTimeState(ref robot_state_pkg));
            mov_robot_state = robot_state_pkg.robot_state;
            robotStatus = mov_robot_state;
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
        private static async Task CheckCurrentToolAndUser()
        {
            await Task.Run(() => robot.GetActualTCPNum(1, ref currentTool));
            await Task.Run(() => robot.GetActualWObjNum(1, ref currentUser));
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
        {
            isInPositionPick = checker_ingombro_pick.IsInObstruction(startPoints[0], TCPCurrentPosition);
            isInPositionPlace = checker_ingombro_place.IsInObstruction(startPoints[1], TCPCurrentPosition);
            isInPositionHome = checker_ingombro_home.IsInObstruction(startPoints[2], TCPCurrentPosition);

            if (isInPositionPick)
            {
                if (prevRobotOutPick != false)
                {
                    prevRobotOutPick = false;
                    prevRobotOutPlace = true;
                    prevInHomePos = true;
                    prevFuoriIngombro = false;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 1, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                }
            }
            else if (isInPositionPlace)
            {
                if (prevRobotOutPlace != false)
                {
                    prevRobotOutPlace = false;
                    prevRobotOutPick = true;
                    prevInHomePos = true;
                    prevFuoriIngombro = false;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos, 1, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                }
            }
            else if (isInPositionHome)
            {
                if (prevInHomePos != false)
                {
                    prevInHomePos = false;
                    prevRobotOutPick = true;
                    prevRobotOutPlace = true;
                    prevFuoriIngombro = false;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 1, "INT16");
                }
            }
            else
            {
                if (prevFuoriIngombro != true)
                {
                    prevFuoriIngombro = true;
                    prevRobotOutPick = true;
                    prevRobotOutPlace = true;
                    prevInHomePos = true;

                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Pick_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Place_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                }
            }
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
                ClearRobotQueue();

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
        public static bool SetRobotProperties()
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
                float blend = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_BLEND_ROW_INDEX]
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

                // Creazione dell'oggetto robotProperties
                robotProperties = new RobotProperties(speed, velocity, blend, acceleration, ovl, tool, user, weight, velRec);

                /*
                 DescPose coord = new DescPose(
                    -1115.359,
                    212.482,
                    -541.592,
                    179.973,
                    0.010,
                    0.765
                    );

               robot.SetWObjCoord(1, coord);
                */

                log.Info($"SetRobotProperties completata: " +
                         $"\n Speed: {speed}" +
                         $"\n Velocity: {velocity}" +
                         $"\n Blend: {blend}" +
                         $"\n Acceleration: {acceleration}" +
                         $"\n Ovl: {ovl}" +
                         $"\n Tool: {tool}" +
                         $"\n User: {user}" +
                         $"\n Weight: {weight}" +
                         $"\n VelRec: {velRec}");

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

        #endregion

        #region Metodi progetto teglie

        /// <summary>
        /// Esegue ciclo di test per pick e place teglia
        /// </summary>
        public static async void PickAndPlaceTeglia()

        {
            #region Dichiarazione punti routine

            // Parametro necessario al comando MoveL
            DescPose offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset

            int offsetAllontamento = 900;

            #region Punto home

            var home = ApplicationConfig.applicationsManager.GetPosition("pHomeTeglia", "RM");
            DescPose descPosHome = new DescPose(home.x, home.y, home.z, home.rx, home.ry, home.rz);

            #endregion


            #region Prima fase

            #region Punto di pick

            JointPos jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
            var pick = ApplicationConfig.applicationsManager.GetPosition("pPickTeglia", "RM");
            DescPose descPosPick = new DescPose(pick.x, pick.y, pick.z, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPick, -1, ref jointPosPick);

            #endregion

            #region Punto avvicinamento pick

            // Prima fase del ciclo
            JointPos jointPosApproachPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPick = new DescPose(pick.x, pick.y - offsetAllontamento, pick.z - 40, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosApproachPick, -1, ref jointPosApproachPick);

            #endregion

            #region Punto post pick

            JointPos jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPick = new DescPose(pick.x, pick.y, pick.z + 20, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPostPick, -1, ref jointPosPostPick);

            #endregion

            #region Punto allontanamento post pick

            JointPos jointPosAllontanamentoPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPick = new DescPose(descPosApproachPick.tran.x, descPosApproachPick.tran.y,
                descPosApproachPick.tran.z + 80, descPosApproachPick.rpy.rx, descPosApproachPick.rpy.ry, descPosApproachPick.rpy.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPick, -1, ref jointPosAllontanamentoPick);

            #endregion


            #region Punto di place

            JointPos jointPosPlace = new JointPos(0, 0, 0, 0, 0, 0);
            var place = ApplicationConfig.applicationsManager.GetPosition("pPlaceTeglia", "RM");
            DescPose descPosPlace = new DescPose(place.x, place.y, place.z, place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosPlace, -1, ref jointPosPlace);

            #endregion

            #region Punto avvicinamento place

            JointPos jointPosApproachPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPlace = new DescPose(place.x + 10, place.y - offsetAllontamento, place.z + 50,
                place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosApproachPlace, -1, ref jointPosApproachPlace);

            #endregion

            #region Punto post place

            JointPos jointPosPostPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPlace = new DescPose(place.x, place.y, place.z,
                place.rx - 3, place.ry, place.rz);
            robot.GetInverseKin(0, descPosPostPlace, -1, ref jointPosPostPlace);

            #endregion

            #region Punto allontanamento place

            JointPos jointPosAllontanamentoPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPlace = new DescPose(place.x, place.y - offsetAllontamento, place.z,
                place.rx - 3, place.ry, place.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPlace, -1, ref jointPosAllontanamentoPlace);

            #endregion

            #endregion

            #region Seconda fase

            #region Punto avvicinamento pick

            // Prima fase del ciclo
            JointPos jointPosApproachPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPick2 = new DescPose(place.x, place.y - offsetAllontamento, place.z - 40, place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosApproachPick2, -1, ref jointPosApproachPick2);

            #endregion

            #region Punto di pick

            JointPos jointPosPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPick2 = new DescPose(place.x, place.y, place.z, place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosPick2, -1, ref jointPosPick2);

            #endregion

            #region Punto post pick

            JointPos jointPosPostPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPick2 = new DescPose(place.x, place.y, place.z + 20, place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosPostPick2, -1, ref jointPosPostPick2);

            #endregion

            #region Punto allontanamento post pick

            JointPos jointPosAllontanamentoPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPick2 = new DescPose(descPosApproachPick2.tran.x, descPosApproachPick2.tran.y,
                descPosApproachPick2.tran.z + 80, descPosApproachPick2.rpy.rx, descPosApproachPick2.rpy.ry, descPosApproachPick2.rpy.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPick2, -1, ref jointPosAllontanamentoPick2);

            #endregion


            #region Punto avvicinamento place

            JointPos jointPosApproachPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPlace2 = new DescPose(pick.x, pick.y - offsetAllontamento, pick.z + 50,
                pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosApproachPlace2, -1, ref jointPosApproachPlace2);

            #endregion

            #region Punto di place

            JointPos jointPosPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPlace2 = new DescPose(pick.x - 10, pick.y, pick.z, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPlace2, -1, ref jointPosPlace2);

            #endregion

            #region Punto post place

            JointPos jointPosPostPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPlace2 = new DescPose(pick.x, pick.y, pick.z,
                pick.rx - 3, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPostPlace2, -1, ref jointPosPostPlace2);

            #endregion

            #region Punto allontanamento place

            JointPos jointPosAllontanamentoPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPlace2 = new DescPose(pick.x, pick.y - offsetAllontamento, pick.z,
                pick.rx - 3, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPlace2, -1, ref jointPosAllontanamentoPlace2);

            #endregion

            #endregion

            #endregion

            #region Parametri movimento

            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)

            // Indica il codice risultante del movimento del Robot
            int movementResult = -1;

            // Reset condizione di stop ciclo
            stopCycleRoutine = false;

            // Reset richiesta di stop ciclo
            stopCycleRequested = false;

            // Reset step routine
            step = 0;

            // Segnale di pick
            bool prendidaNastro = true;

            // Segnale di place
            bool appoggiaSuScatola = true;

            #endregion


            // Aspetto che il metodo termini, ma senza bloccare il thread principale
            // La routine è incapsulata come 'async' per supportare futuri operatori 'await' nel caso ci fosse la necessità
            await Task.Run(async () =>
            {
                // Fino a quando la condizione di stop routine non è true e non sono presenti allarmi bloccanti
                while (!stopCycleRoutine && !AlarmManager.blockingAlarm)
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 0 - Check richiesta interruzione ciclo");
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 5 - Termine routine");
                            break;

                        #endregion

                        case 10:
                            #region Movimento a punto di Pick

                            inPosition = false; // Reset inPosition

                            // Invio punto di Home
                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config);
                            GetRobotMovementCode(movementResult);

                            // Invio punto di avvicinamento Pick
                            movementResult = robot.MoveCart(descPosApproachPick, tool, user, vel, acc,
                                ovl, blendT, config);
                            GetRobotMovementCode(movementResult);

                            // Movimento lineare finale in Pick
                            movementResult = robot.MoveL(jointPosPick, descPosPick, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            endingPoint = descPosPick; // Assegnazione endingPoint
                            step = 20;

                            formDiagnostics.UpdateRobotStepDescription("STEP 10 - Movimento a punto di Pick (prima fase)");

                            break;
                        #endregion


                        case 20:
                            #region Delay per calcolo in position punto di pick

                            await Task.Delay(500);
                            step = 30;
                            formDiagnostics.UpdateRobotStepDescription("STEP 20 -  Delay calcolo in position punto di pick (prima fase)");
                            break;

                        #endregion

                        case 30:
                            #region Attesa inPosition punto di Pick

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick
                            {
                                await Task.Delay(500);
                                // Chiudo la pinza

                                step = 40;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 30 - Attesa inPosition punto di Pick (prima fase)");
                            break;

                        #endregion

                        case 40:
                            #region Presa teglia

                            // if (gripperStatus == 0)
                            {
                                await Task.Delay(1000); // Per evitare "rimbalzo" del Robot
                                step = 50;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 40 - Presa teglia (prima fase)");
                            break;

                        #endregion

                        case 50:
                            #region Movimento a punto di Home

                            movementResult = robot.MoveL(jointPosPostPick, descPosPostPick, tool, user, vel, acc,
                                   ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosAllontanamentoPick, descPosAllontanamentoPick, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di Home
                            GetRobotMovementCode(movementResult);

                            formDiagnostics.UpdateRobotStepDescription("STEP 50 - Movimento a punto di Home");

                            step = 60;
                            break;

                        #endregion

                        case 60:
                            #region Movimento a punto di place

                            if (appoggiaSuScatola)
                            {
                                inPosition = false; // Reset inPosition

                                movementResult = robot.MoveCart(descPosApproachPlace, tool, user, vel, acc,
                                  ovl, blendT, config); // Invio punto di avvicinamento place
                                GetRobotMovementCode(movementResult);

                                // Movimento lineare
                                movementResult = robot.MoveL(jointPosPlace, descPosPlace, tool, user, vel, acc,
                                    ovl, blendT, epos, 0, offsetFlag, offset
                                    );

                                endingPoint = descPosPlace; // Assegnazione endingPoint

                                step = 70;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 60 - Movimento a punto di place (prima fase)");
                            break;

                        #endregion

                        case 70:
                            #region Delay per calcolo in position punto di place

                            await Task.Delay(500);
                            step = 80;
                            formDiagnostics.UpdateRobotStepDescription("STEP 70 -  Delay calcolo in position punto di place (prima fase)");
                            break;

                        #endregion

                        case 80:
                            #region Attesa inPosition punto di Place

                            if (inPosition) // Se il Robot è arrivato in posizione di place
                            {
                                await Task.Delay(500);
                                // Chiudo la pinza

                                step = 90;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 80 - Attesa inPosition punto di place (prima fase)");
                            break;
                        #endregion

                        case 90:
                            #region Rilascio teglia

                            // if (gripperStatus == 0)
                            {
                                await Task.Delay(1000); // Per evitare "rimbalzo" del Robot
                                step = 100;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 90 - Rilascio teglia (prima fase)");
                            break;

                        #endregion

                        case 100:
                            #region Movimento a punto di Home

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosPostPlace, descPosPostPlace, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosAllontanamentoPlace, descPosAllontanamentoPlace, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di Home
                            GetRobotMovementCode(movementResult);

                            formDiagnostics.UpdateRobotStepDescription("STEP 100 - Movimento a punto di Home");

                            step = 110;
                            break;
                        #endregion

                        case 110:
                            #region Movimento a punto di pick

                            inPosition = false; // Reset inPosition

                            movementResult = robot.MoveCart(descPosApproachPick2, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di avvicinamento place
                            GetRobotMovementCode(movementResult);

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosPick2, descPosPick2, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            endingPoint = descPosPick2; // Assegnazione endingPoint

                            step = 120;

                            formDiagnostics.UpdateRobotStepDescription("STEP 110 - Movimento a punto di pick (seconda fase)");
                            #endregion
                            break;

                        case 120:
                            #region Delay per calcolo in position punto di pick

                            await Task.Delay(500);
                            step = 130;
                            formDiagnostics.UpdateRobotStepDescription("STEP 120 -  Delay calcolo in position punto di pick (seconda fase)");
                            break;

                        #endregion

                        case 130:
                            #region Attesa inPosition punto di pick

                            if (inPosition) // Se il Robot è arrivato in posizione di place
                            {
                                await Task.Delay(500);
                                // Chiudo la pinza

                                step = 140;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 130 - Attesa inPosition punto di pick (seconda fase)");
                            break;
                        #endregion

                        case 140:
                            #region Presa teglia

                            // if (gripperStatus == 0)
                            {
                                await Task.Delay(1000); // Per evitare "rimbalzo" del Robot
                                step = 150;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 140 - Presa teglia (seconda fase)");
                            break;

                        #endregion

                        case 150:
                            #region Movimento a punto di Home e riavvio ciclo

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosPostPick2, descPosPostPick2, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosAllontanamentoPick2, descPosAllontanamentoPick2, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di Home
                            GetRobotMovementCode(movementResult);

                            formDiagnostics.UpdateRobotStepDescription("STEP 150 - Movimento a punto di Home");

                            step = 160;
                            break;
                        #endregion

                        case 160:
                            #region Movimento a punto di Place

                            if (prendidaNastro)
                            {
                                inPosition = false; // Reset inPosition

                                // STEP 1: Invio punto di Home
                                movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                    ovl, blendT, config);
                                GetRobotMovementCode(movementResult);

                                // STEP 3: Invio punto di avvicinamento Pick
                                movementResult = robot.MoveCart(descPosApproachPlace2, tool, user, vel, acc,
                                    ovl, blendT, config);
                                GetRobotMovementCode(movementResult);

                                // STEP 4: Movimento lineare finale
                                movementResult = robot.MoveL(jointPosPlace2, descPosPlace2, tool, user, vel, acc,
                                    ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult);

                                endingPoint = descPosPlace2; // Assegnazione endingPoint
                                step = 170;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 160 - Movimento a punto di Place (seconda fase)");
                            break;
                        #endregion


                        case 170:
                            #region Delay per calcolo in position punto di place

                            await Task.Delay(500);
                            step = 180;
                            formDiagnostics.UpdateRobotStepDescription("STEP 170 -  Delay calcolo in position punto di place (seconda fase)");
                            break;

                        #endregion

                        case 180:
                            #region Attesa inPosition punto di Place

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick
                            {
                                await Task.Delay(500);
                                // Chiudo la pinza

                                step = 190;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 180 - Attesa inPosition punto di place (seconda fase)");
                            break;

                        #endregion

                        case 190:
                            #region Rilascio teglia

                            // if (gripperStatus == 0)
                            {
                                await Task.Delay(1000); // Per evitare "rimbalzo" del Robot
                                step = 200;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 190 - Rilascio teglia (seconda fase)");
                            break;

                        #endregion

                        case 200:
                            #region Movimento a punto di Home e riavvio ciclo

                            movementResult = robot.MoveL(jointPosPostPlace2, descPosPostPlace2, tool, user, vel, acc,
                                   ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosAllontanamentoPlace2, descPosAllontanamentoPlace2, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di Home
                            GetRobotMovementCode(movementResult);

                            formDiagnostics.UpdateRobotStepDescription("STEP 200 - Movimento a punto di Home e riavvio ciclo");

                            step = 0;
                            break;

                            #endregion
                    }

                    await Task.Delay(10); // Delay routine
                }
            });
        }

        /// <summary>
        /// Esegue ciclo di pick di una scatola e place dentro al pallet in una determinata posizione
        /// </summary>
        public static async void PickAndPlaceScatola()
        {
            /*if(larghezzaScatola == 0 || larghezzaPallet == 0) // controllare che non ci siano valori = 0 prima di iniziare
            {
                return;
            }*/

            #region Dichiarazione punti routine

            #region Punto home

            var home = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose descPosHome = new DescPose(home.x, home.y, home.z, home.rx, home.ry, home.rz);

            #endregion

            #region Punto di pick

            JointPos jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
            var pick = ApplicationConfig.applicationsManager.GetPosition("pPick", "RM");
            DescPose descPosPick = new DescPose(pick.x, pick.y, pick.z, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPick, -1, ref jointPosPick);

            #endregion

            #region Punto avvicinamento pick

            JointPos jointPosApproachPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPick = new DescPose(pick.x, pick.y, pick.z + 200, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosApproachPick, -1, ref jointPosApproachPick);

            #endregion

            #region Punto di place

            JointPos jointPosPlace = new JointPos(0, 0, 0, 0, 0, 0);
            var place = ApplicationConfig.applicationsManager.GetPosition("pPlace", "RM");
            DescPose descPosPlace = new DescPose(place.x, place.y, place.z, place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosPlace, -1, ref jointPosPlace);

            #endregion

            #region Punto avvicinamento place

            JointPos jointPosApproachPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPlace = new DescPose(place.x, place.y, place.z + 200, place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosApproachPlace, -1, ref jointPosApproachPlace);

            #endregion

            #endregion

            #region Parametri movimento

            DescPose offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset
            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)

            #endregion

            // Indica il codice risultante del movimento del Robot
            int movementResult = -1;

            // Reset condizione di stop ciclo
            stopCycleRoutine = false;

            // Reset richiesta di stop ciclo
            stopCycleRequested = false;

            // Reset step routine
            step = 0;

            // Segnale di pick
            bool prendidaNastro = true;

            // Segnale di place
            bool appoggiaSuPallet = true;


            int numeroRighe = (int)(larghezzaPallet / larghezzaScatola);
            int numeroColonne = (int)(lunghezzaPallet / lunghezzaScatola);

            // CalcolaPosizioneFocacce(larghezzaFocaccia,profonditaFocaccia,larghezzaPallet,profonditaPallet);

            DescPose originePallet = descPosPlace;
            JointPos jointPosPlaceCalculated = new JointPos(0, 0, 0, 0, 0, 0);

            // Aspetto che il metodo termini, ma senza bloccare il thread principale
            // La routine è incapsulata come 'async' per supportare futuri operatori 'await' nel caso ci fosse la necessità
            await Task.Run(async () =>
            {
                // Fino a quando la condizione di stop routine non è true e non sono presenti allarmi bloccanti
                while (!stopCycleRoutine && !AlarmManager.blockingAlarm)
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 0 - Check richiesta interruzione ciclo");
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 5 - Termine routine");
                            break;

                        #endregion

                        case 10:
                            #region Movimento a punto di Pick

                            if (prendidaNastro)
                            {
                                inPosition = false; // Reset inPosition

                                // STEP 1: Invio punto di Home
                                movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                    ovl, blendT, config);
                                GetRobotMovementCode(movementResult);

                                // STEP 3: Invio punto di avvicinamento Pick
                                movementResult = robot.MoveCart(descPosApproachPick, tool, user, vel, acc,
                                    ovl, blendT, config);
                                GetRobotMovementCode(movementResult);

                                // STEP 4: Movimento lineare finale
                                movementResult = robot.MoveL(jointPosPick, descPosPick, tool, user, vel, acc,
                                    ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult);

                                endingPoint = descPosPick; // Assegnazione endingPoint
                                step = 20;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 10 - Movimento a punto di Pick");
                            break;
                        #endregion

                        case 20:
                            #region Delay per calcolo in position punto di pick

                            await Task.Delay(500);
                            step = 30;
                            formDiagnostics.UpdateRobotStepDescription("STEP 20 -  Delay calcolo in position punto di pick");
                            break;

                        #endregion

                        case 30:
                            #region Attesa inPosition punto di Pick

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick
                            {
                                // Chiudo la pinza

                                step = 40;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 30 - Attesa inPosition punto di Pick");
                            break;

                        #endregion

                        case 40:
                            #region Presa focaccia

                            // if (gripperStatus == 0)
                            {
                                await Task.Delay(500); // Per evitare "rimbalzo" del Robot
                                step = 50;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 40 - Presa focaccia");
                            break;

                        #endregion

                        case 50:
                            #region Movimento a punto di Home

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosApproachPick, descPosApproachPick, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            /*
                            // Movimento cartesiano
                            movementResult = robot.MoveCart(descPosApproachPick, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di avvicinamento Pick
                            GetRobotMovementCode(movementResult);
                            */
                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di Home
                            GetRobotMovementCode(movementResult);

                            formDiagnostics.UpdateRobotStepDescription("STEP 50 - Movimento a punto di Home");

                            step = 60;
                            break;

                        #endregion

                        case 60:
                            #region Movimento a punto di place

                            if (appoggiaSuPallet)
                            {
                                inPosition = false; // Reset inPosition

                                DescPose puntoPlaceScatola = CalcolaPosizioneScatola(
                                    riga,
                                colonna,
                                    strato,
                                    larghezzaScatola,
                                    lunghezzaScatola,
                                    altezzaScatola,
                                    originePallet
                                );

                                robot.GetInverseKin(0, puntoPlaceScatola, -1, ref jointPosPlaceCalculated);

                                descPosApproachPlace = new DescPose(
                                    puntoPlaceScatola.tran.x,
                                    puntoPlaceScatola.tran.y,
                                    puntoPlaceScatola.tran.z + 200,
                                    puntoPlaceScatola.rpy.rx,
                                    puntoPlaceScatola.rpy.ry,
                                    puntoPlaceScatola.rpy.rz);

                                robot.GetInverseKin(0, descPosApproachPlace, -1, ref jointPosApproachPlace);

                                movementResult = robot.MoveCart(descPosApproachPlace, tool, user, vel, acc,
                                  ovl, blendT, config); // Invio punto di avvicinamento place
                                GetRobotMovementCode(movementResult);

                                // Movimento lineare
                                movementResult = robot.MoveL(jointPosPlaceCalculated, puntoPlaceScatola, tool, user, vel, acc,
                                    ovl, blendT, epos, 0, offsetFlag, offset
                                    );

                                //TODO: questi calcoli non verranno più fatti qui ma direttamente del PLC, noi leggiamo e basta i valori
                                if (colonna < numeroColonne - 1)
                                {
                                    colonna++;
                                }
                                else
                                {
                                    if (riga < numeroRighe - 1)
                                    {
                                        riga++;
                                        colonna = 0;
                                    }
                                    else
                                    {
                                        riga = 0;
                                        colonna = 0;
                                        strato++;
                                    }
                                }
                                /*
                                // Movimento cartesiano
                                movementResult = robot.MoveCart(descPosPlace, tool, user, vel, acc,
                                    ovl, blendT, config); // Invio punto di place
                                GetRobotMovementCode(movementResult);
                                */

                                endingPoint = puntoPlaceScatola; // Assegnazione endingPoint

                                step = 70;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 60 - Movimento a punto di place");
                            break;

                        #endregion

                        case 70:
                            #region Delay per calcolo in position punto di place

                            await Task.Delay(500);
                            step = 80;
                            formDiagnostics.UpdateRobotStepDescription("STEP 70 -  Delay calcolo in position punto di place");
                            break;

                        #endregion

                        case 80:
                            #region Attesa inPosition punto di Place

                            if (inPosition) // Se il Robot è arrivato in posizione di place
                            {
                                // Chiudo la pinza

                                step = 90;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 80 - Attesa inPosition punto di place");
                            break;
                        #endregion

                        case 90:
                            #region Rilascio focaccia

                            // if (gripperStatus == 0)
                            {
                                await Task.Delay(500); // Per evitare "rimbalzo" del Robot
                                step = 100;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 90 - Rilascio focaccia");
                            break;

                        #endregion

                        case 100:
                            #region Movimento a punto di Home e riavvio ciclo

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosApproachPlace, descPosApproachPlace, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );
                            /*
                            // Movimento cartesiano
                            movementResult = robot.MoveCart(descPosApproachPlace, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di avvicinamento place
                            GetRobotMovementCode(movementResult);
                            */

                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di Home
                            GetRobotMovementCode(movementResult);

                            formDiagnostics.UpdateRobotStepDescription("STEP 100 - Movimento a punto di Home e riavvio ciclo");

                            step = 0;
                            break;
                            #endregion

                    }

                    await Task.Delay(10); // Delay routine
                }
            });
        }

        /// <summary>
        /// Esegue ciclo teglie
        /// </summary>
        public static async void PickAndPlaceTegliaGiroCompleto()
        {
            #region Dichiarazione punti routine

            // Parametro necessario al comando MoveL
            DescPose offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset

            int offsetAllontamento = 900;
            int offsetAvvicinamento = 200;

            #region Punto home

            JointPos jointPosHome = new JointPos(0, 0, 0, 0, 0, 0);
            var home = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose descPosHome = new DescPose(home.x, home.y, home.z, home.rx, home.ry, home.rz);
            robot.GetInverseKin(0, descPosHome, -1, ref jointPosHome);

            #endregion


            #region Prima fase

            #region Punto di pick

            JointPos jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
            var pick = ApplicationConfig.applicationsManager.GetPosition("pPickTeglia", "RM");
            DescPose descPosPick = new DescPose(pick.x, pick.y, pick.z, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPick, -1, ref jointPosPick);

            #endregion

            #region Punto avvicinamento pick

            // Prima fase del ciclo
            JointPos jointPosApproachPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPick = new DescPose(pick.x - offsetAvvicinamento, pick.y, pick.z - 40, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosApproachPick, -1, ref jointPosApproachPick);

            #endregion

            #region Punto post pick

            JointPos jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPick = new DescPose(pick.x, pick.y, pick.z + 40, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPostPick, -1, ref jointPosPostPick);

            #endregion

            #region Punto allontanamento post pick

            JointPos jointPosAllontanamentoPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPick = new DescPose(descPosApproachPick.tran.x, descPosApproachPick.tran.y,
                descPosApproachPick.tran.z + 80, descPosApproachPick.rpy.rx, descPosApproachPick.rpy.ry, descPosApproachPick.rpy.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPick, -1, ref jointPosAllontanamentoPick);

            #endregion


            #region Punto di place

            JointPos jointPosPlace = new JointPos(0, 0, 0, 0, 0, 0);
            var place = ApplicationConfig.applicationsManager.GetPosition("pPlaceTeglia", "RM");
            DescPose descPosPlace = new DescPose(place.x, place.y, place.z, NormalizeAngle((float)place.rx + 3), place.ry, place.rz);
            robot.GetInverseKin(0, descPosPlace, -1, ref jointPosPlace);

            #endregion

            #region Punto avvicinamento place

            JointPos jointPosApproachPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPlace = new DescPose(place.x, place.y - offsetAvvicinamento, place.z + 20,
                place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosApproachPlace, -1, ref jointPosApproachPlace);

            #endregion

            #region Punto post place

            JointPos jointPosPostPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPlace = new DescPose(place.x, place.y, place.z,
                place.rx - 3, place.ry, place.rz);
            robot.GetInverseKin(0, descPosPostPlace, -1, ref jointPosPostPlace);

            #endregion

            #region Punto allontanamento place

            JointPos jointPosAllontanamentoPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPlace = new DescPose(place.x, place.y - offsetAvvicinamento - 200, place.z,
                place.rx - 3, place.ry, place.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPlace, -1, ref jointPosAllontanamentoPlace);

            #endregion

            #endregion

            #region Seconda fase

            #region Punto di pick 2

            JointPos jointPosPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            var pick2 = place;
            DescPose descPosPick2 = new DescPose(pick2.x, pick2.y, pick2.z, pick2.rx, pick2.ry, pick2.rz);
            robot.GetInverseKin(0, descPosPick2, -1, ref jointPosPick2);

            #endregion

            #region Punto avvicinamento pick

            // Prima fase del ciclo
            JointPos jointPosApproachPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPick2 = new DescPose(pick2.x, pick2.y - offsetAvvicinamento, pick2.z - 40, pick2.rx, pick2.ry, pick2.rz);
            robot.GetInverseKin(0, descPosApproachPick2, -1, ref jointPosApproachPick2);

            #endregion

            #region Punto post pick

            JointPos jointPosPostPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPick2 = new DescPose(pick2.x, pick2.y, pick2.z + 40, pick2.rx, pick2.ry, pick2.rz);
            robot.GetInverseKin(0, descPosPostPick2, -1, ref jointPosPostPick2);

            #endregion

            #region Punto allontanamento post pick

            JointPos jointPosAllontanamentoPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPick2 = new DescPose(descPosPick2.tran.x, descPosPick2.tran.y - offsetAllontamento,
                descPosPick2.tran.z + 40, descPosPick2.rpy.rx, descPosPick2.rpy.ry, descPosPick2.rpy.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPick2, -1, ref jointPosAllontanamentoPick2);

            #endregion


            #region Punto avvicinamento place

            JointPos jointPosApproachPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPlace2 = new DescPose(pick.x - offsetAvvicinamento, pick.y, pick.z + 50,
                pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosApproachPlace2, -1, ref jointPosApproachPlace2);

            #endregion

            #region Punto di place

            JointPos jointPosPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPlace2 = new DescPose(pick.x - 10, pick.y, pick.z, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPlace2, -1, ref jointPosPlace2);

            #endregion

            #region Punto post place

            JointPos jointPosPostPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPlace2 = new DescPose(pick.x, pick.y, pick.z,
                pick.rx - 3, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPostPlace2, -1, ref jointPosPostPlace2);

            #endregion

            #region Punto allontanamento place

            JointPos jointPosAllontanamentoPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPlace2 = new DescPose(pick.x - offsetAvvicinamento, pick.y, pick.z,
                pick.rx - 3, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPlace2, -1, ref jointPosAllontanamentoPlace2);

            #endregion



            #endregion

            // Posa pre pick 1
            DescPose prePick1Pose = new DescPose(
                                descPosHome.tran.x,
                                descPosHome.tran.y,
                                descPosPick.tran.z - 40, // Mi abbasso di 4 cm prima di andare in pick
                                descPosHome.rpy.rx,
                                descPosHome.rpy.ry,
                                descPosHome.rpy.rz
                                );

            JointPos jointPrePick1 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, prePick1Pose, -1, ref jointPrePick1);

            // Posa post pick 1
            DescPose postPick1Pose = new DescPose(
                              descPosHome.tran.x,
                              descPosHome.tran.y,
                              descPosPick.tran.z + 20, // Mi alzo di 2 cm per uscire dal carrello senza strisciare
                              NormalizeAngle((float)(descPosPick.rpy.rx + 2)),
                              descPosHome.rpy.ry,
                              descPosHome.rpy.rz
                              );

            JointPos jointPostPick1 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, postPick1Pose, -1, ref jointPostPick1);

            // Posa per ruotare il robot dopo il pick 1 e prima di andare al place 1
            DescPose postPick1RotationPose = new DescPose(
                descPosHome.tran.x,
                descPosPlace.tran.y,
                descPosHome.tran.z,
                descPosPlace.rpy.rx,
                descPosPlace.rpy.ry,
                descPosPlace.rpy.rz
                );

            // Posa post place 1
            DescPose postPlace1Pose = new DescPose(
                descPosHome.tran.x,
                descPosPlace.tran.y,
                descPosPick.tran.z + 20, // Mi alzo di 2 cm prima di andare in home
                descPosHome.rpy.rx,
                descPosHome.rpy.ry,
                descPosPlace.rpy.rz
                );

            JointPos jointPostPlace1 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, postPlace1Pose, -1, ref jointPostPlace1);

            // Posa per ruotare il robot dopo il place 1 e prima di andare al place 2
            DescPose postPlace1RotationPose = new DescPose(
                descPosHome.tran.x,
                descPosHome.tran.y,
                descPosHome.tran.z + 20,
                descPosHome.rpy.rx,
                descPosHome.rpy.ry,
                descPosHome.rpy.rz);

            DescPose place2Pose = descPosPick;
            JointPos jointPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, place2Pose, -1, ref jointPlace2);

            // Posa per ruotare il robot dopo il place 1 e prima di andare al place 2
            DescPose postPlace2Pose = new DescPose(
                descPosHome.tran.x,
                descPosHome.tran.y,
                descPosPick.tran.z - 20, // Mi alzo di 4 cm prima di andare in home
                descPosPick.rpy.rx - 4,
                descPosHome.rpy.ry,
                descPosHome.rpy.rz
                );

            JointPos jointPostPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, postPlace2Pose, -1, ref jointPostPlace2);

            #endregion

            #region Parametri movimento

            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)

            // Indica il codice risultante del movimento del Robot
            int movementResult = -1;

            // Reset condizione di stop ciclo
            stopCycleRoutine = false;

            // Reset richiesta di stop ciclo
            stopCycleRequested = false;

            // Reset step routine
            step = 0;

            // Segnale di pick
            bool prendidaNastro = true;

            #endregion

            double[] levelCollision1 = new double[] { 1, 1, 1, 1, 1, 1 };
            double[] levelCollision2 = new double[] { 2, 2, 2, 2, 2, 2 };
            double[] levelCollision3 = new double[] { 3, 3, 3, 3, 3, 3 };
            double[] levelCollision4 = new double[] { 4, 4, 4, 4, 4, 4 };
            double[] levelCollision5 = new double[] { 5, 5, 5, 5, 5, 5 };
            double[] levelCollision6 = new double[] { 6, 6, 6, 6, 6, 6 };
            double[] levelCollision7 = new double[] { 7, 7, 7, 7, 7, 7 };
            double[] levelCollision8 = new double[] { 8, 8, 8, 8, 8, 8 };

            robot.SetAnticollision(0, levelCollision6, 1);

            byte ris = 0;

            // Aspetto che il metodo termini, ma senza bloccare il thread principale
            // La routine è incapsulata come 'async' per supportare futuri operatori 'await' nel caso ci fosse la necessità
            await Task.Run(async () =>
            {
                // Fino a quando la condizione di stop routine non è true e non sono presenti allarmi bloccanti
                while (!stopCycleRoutine && !AlarmManager.blockingAlarm)
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 0 - Check richiesta interruzione ciclo");
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 5 - Termine routine");
                            break;

                        #endregion

                        case 10:
                            #region Movimento a punto di Pick

                            inPosition = false; // Reset inPosition

                            // Movimento in posa di pre pick 1
                            movementResult = robot.MoveL(jointPrePick1, prePick1Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento in posa di pick 1
                            movementResult = robot.MoveL(jointPosPick, descPosPick, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            endingPoint = descPosPick; // Assegnazione endingPoint

                            step = 30; // Passaggio a step 30

                            formDiagnostics.UpdateRobotStepDescription("STEP 10 - Movimento a punto di Pick 1");

                            break;

                        #endregion

                        case 20:
                            #region Delay per calcolo in position punto di pick 1

                            await Task.Delay(500);

                            step = 30; // Passaggio a step 30

                            formDiagnostics.UpdateRobotStepDescription("STEP 20 -  Delay calcolo in position punto di pick 1");

                            break;

                        #endregion

                        case 30:
                            #region Attesa inPosition punto di Pick e chiusura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick 1
                            {
                                // Chiudo la pinza
                                robot.SetDO(0, 1, 0, 0);

                                step = 40; // Passaggio a step 40
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 30 - Attesa inPosition punto di Pick 1");

                            break;

                        #endregion

                        case 40:
                            #region Delay post chiusura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 0)
                            {

                                //await Task.Delay(800); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza
                                step = 50;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 40 - Delay post chiusura pinza in fase di pick 1");

                            break;

                        #endregion

                        case 50:
                            #region Movimento di uscita dal carrello dopo pick 1

                            // Movimento per uscire dal carrelo dopo pick 1
                            movementResult = robot.MoveL(jointPostPick1, postPick1Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            formDiagnostics.UpdateRobotStepDescription("STEP 50 - Movimento di uscita dal carrello dopo pick 1");

                            step = 60;

                            break;

                        #endregion

                        case 60:
                            #region Movimento a punto di place 1

                            inPosition = false; // Reset inPosition

                            // Movimento di rotazione prima di andare in place 1
                            movementResult = robot.MoveCart(postPick1RotationPose, tool, user, vel / 2, acc / 2, ovl, blendT, config);
                            GetRobotMovementCode(movementResult);

                            // Movimento in place 1
                            movementResult = robot.MoveL(jointPosPlace, descPosPlace, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            endingPoint = descPosPlace; // Assegnazione endingPoint

                            step = 80; // Passaggio a step 80

                            formDiagnostics.UpdateRobotStepDescription("STEP 60 - Movimento a punto di place 1");

                            break;

                        #endregion

                        case 70:
                            #region Delay per calcolo in position punto di place

                            await Task.Delay(500);
                            step = 80; // Passaggio a step 80

                            formDiagnostics.UpdateRobotStepDescription("STEP 70 -  Delay calcolo in position punto di place (prima fase)");

                            break;

                        #endregion

                        case 80:
                            #region Attesa inPosition punto di Place 1

                            if (inPosition) // Se il Robot è arrivato in posizione di place
                            {
                                await Task.Delay(1000);
                                step = 100; // Passaggio a step 100
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 80 - Attesa inPosition punto di place 1");

                            break;

                        #endregion

                        case 90:
                            #region Rilascio teglia

                            // if (gripperStatus == 0)
                            {
                                await Task.Delay(500);
                                robot.SetDO(0, 0, 0, 0); // Apro la pinza
                                step = 100;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 90 - Rilascio teglia (prima fase)");
                            break;

                        #endregion

                        case 100:
                            #region Movimento per uscire da place 1 ed entrare in place 2

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPostPlace1, postPlace1Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento di preparazione al place 2
                            movementResult = robot.MoveCart(postPlace1RotationPose, tool, user, vel / 2, acc / 2, ovl, blendT, config);
                            GetRobotMovementCode(movementResult);

                            endingPoint = postPlace1RotationPose;

                            formDiagnostics.UpdateRobotStepDescription("STEP 100 - Movimento a punto di place 2");

                            step = 105;

                            break;

                        #endregion

                        case 105:
                            #region Attesa inPosition punto pre place 2

                            if (inPosition) // Se il Robot è arrivato in posizione di place 2
                            {
                                //  await Task.Delay(300);
                                // robot.SetAnticollision(0, levelCollision5, 1);

                                // Movimento a posa di place 2
                                movementResult = robot.MoveL(jointPlace2, place2Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                                endingPoint = place2Pose; // Assegnazione ending point

                                formDiagnostics.UpdateRobotStepDescription("STEP 100 - Movimento a punto di place 2");

                                step = 130; // Passaggio a step 140
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 130 - Attesa inPosition punto pre place");

                            break;

                        #endregion

                        case 120:
                            #region Delay per calcolo in position punto di place 2

                            await Task.Delay(500);
                            step = 130;

                            formDiagnostics.UpdateRobotStepDescription("STEP 120 -  Delay calcolo in position punto di place 2");
                            break;

                        #endregion

                        case 130:
                            #region Attesa inPosition punto di place 2

                            if (inPosition) // Se il Robot è arrivato in posizione di place 2
                            {
                                robot.SetDO(0, 0, 0, 0); // Apro la pinza

                                step = 140; // Passaggio a step 140
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 130 - Attesa inPosition punto di place 2 e apertura pinza");

                            break;

                        #endregion

                        case 140:
                            #region Delay post apertura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 1)
                            {
                                // await Task.Delay(300); // // Ritardo per evitare che il robot riparta senza aver finito di aprire la pinza
                                //  robot.SetAnticollision(0, levelCollision5, 1);

                                step = 150; // Passaggio a step 150
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 140 - Delay post apertura pinza in place 2");

                            break;

                        #endregion

                        case 150:
                            #region Movimento per uscire dal carrello dopo il place 2

                            // Movimento per uscire dal carrello dopo il place 2
                            movementResult = robot.MoveL(jointPostPlace2, postPlace2Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            formDiagnostics.UpdateRobotStepDescription("STEP 150 - Movimento per uscire dal carrello dopo il place 2");

                            await Task.Delay(500); // Delay prima di riavviare il ciclo
                            step = 0;
                            break;
                        #endregion

                        case 160:
                            #region Movimento a punto di Place

                            if (prendidaNastro)
                            {
                                inPosition = false; // Reset inPosition

                                // STEP 3: Invio punto di avvicinamento Pick
                                /* movementResult = robot.MoveCart(descPosApproachPlace2, tool, user, vel, acc,
                                     ovl, blendT, config);
                                 GetRobotMovementCode(movementResult);*/

                                // STEP 4: Movimento lineare finale
                                movementResult = robot.MoveL(jointPosApproachPlace2, descPosApproachPlace2, tool, user, vel, acc,
                                    ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult);

                                // STEP 4: Movimento lineare finale
                                movementResult = robot.MoveL(jointPosPlace2, descPosPlace2, tool, user, vel, acc,
                                    ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult);

                                endingPoint = descPosPlace2; // Assegnazione endingPoint
                                step = 170;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 160 - Movimento a punto di Place (seconda fase)");
                            break;
                        #endregion


                        case 170:
                            #region Delay per calcolo in position punto di place

                            // Thread.Sleep(500);
                            step = 180;
                            formDiagnostics.UpdateRobotStepDescription("STEP 170 -  Delay calcolo in position punto di place (seconda fase)");
                            break;

                        #endregion

                        case 180:
                            #region Attesa inPosition punto di Place

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick
                            {
                                // Thread.Sleep(500);
                                // Chiudo la pinza

                                step = 190;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 180 - Attesa inPosition punto di place (seconda fase)");
                            break;

                        #endregion

                        case 190:
                            #region Rilascio teglia

                            // if (gripperStatus == 0)
                            {
                                Thread.Sleep(500); // Per evitare "rimbalzo" del Robot
                                step = 200;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 190 - Rilascio teglia (seconda fase)");
                            break;

                        #endregion

                        case 200:
                            #region Movimento a punto di Home e riavvio ciclo

                            movementResult = robot.MoveL(jointPosPostPlace2, descPosPostPlace2, tool, user, vel, acc,
                                   ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosAllontanamentoPlace2, descPosAllontanamentoPlace2, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            /*
                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di Home
                            GetRobotMovementCode(movementResult);
                            */

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosHome, descPosHome, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            formDiagnostics.UpdateRobotStepDescription("STEP 200 - Movimento a punto di Home e riavvio ciclo");

                            step = 0;
                            break;

                            #endregion
                    }

                    Thread.Sleep(50); // Delay routine
                }
            });


        }

        /// <summary>
        /// Esegue ciclo teglie
        /// </summary>
        public static async Task PickAndPlaceTeglia5()
        {
            #region Dichiarazione punti routine

            // Parametro necessario al comando MoveL
            DescPose offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset

            int offsetAllontamento = 900;
            int offsetAvvicinamento = 200;

            #region Punto home

            JointPos jointPosHome = new JointPos(0, 0, 0, 0, 0, 0);
            var home = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose descPosHome = new DescPose(home.x, home.y, home.z, home.rx, home.ry, home.rz);
            robot.GetInverseKin(0, descPosHome, -1, ref jointPosHome);

            #endregion


            #region Prima fase

            #region Punto di pick

            JointPos jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
            var pick = ApplicationConfig.applicationsManager.GetPosition("pPickTeglia", "RM");
            DescPose descPosPick = new DescPose(pick.x, pick.y, pick.z, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPick, -1, ref jointPosPick);

            #endregion

            #region Punto avvicinamento pick

            // Prima fase del ciclo
            JointPos jointPosApproachPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPick = new DescPose(pick.x - offsetAvvicinamento, pick.y, pick.z - 40, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosApproachPick, -1, ref jointPosApproachPick);

            #endregion

            #region Punto post pick

            JointPos jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPick = new DescPose(pick.x, pick.y, pick.z + 40, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPostPick, -1, ref jointPosPostPick);

            #endregion

            #region Punto allontanamento post pick

            JointPos jointPosAllontanamentoPick = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPick = new DescPose(descPosApproachPick.tran.x, descPosApproachPick.tran.y,
                descPosApproachPick.tran.z + 80, descPosApproachPick.rpy.rx, descPosApproachPick.rpy.ry, descPosApproachPick.rpy.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPick, -1, ref jointPosAllontanamentoPick);

            #endregion


            #region Punto di place

            JointPos jointPosPlace = new JointPos(0, 0, 0, 0, 0, 0);
            var place = ApplicationConfig.applicationsManager.GetPosition("pPlaceTeglia", "RM");
            DescPose descPosPlace = new DescPose(place.x, place.y, place.z, NormalizeAngle((float)place.rx + 3), place.ry, place.rz);
            robot.GetInverseKin(0, descPosPlace, -1, ref jointPosPlace);

            #endregion

            #region Punto avvicinamento place

            JointPos jointPosApproachPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPlace = new DescPose(place.x, place.y - offsetAvvicinamento, place.z + 20,
                place.rx, place.ry, place.rz);
            robot.GetInverseKin(0, descPosApproachPlace, -1, ref jointPosApproachPlace);

            #endregion

            #region Punto post place

            JointPos jointPosPostPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPlace = new DescPose(place.x, place.y, place.z,
                place.rx - 3, place.ry, place.rz);
            robot.GetInverseKin(0, descPosPostPlace, -1, ref jointPosPostPlace);

            #endregion

            #region Punto allontanamento place

            JointPos jointPosAllontanamentoPlace = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPlace = new DescPose(place.x, place.y - offsetAvvicinamento - 200, place.z,
                place.rx - 3, place.ry, place.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPlace, -1, ref jointPosAllontanamentoPlace);

            #endregion

            #endregion

            #region Seconda fase

            #region Punto di pick 2

            JointPos jointPosPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            var pick2 = place;
            DescPose descPosPick2 = new DescPose(pick2.x, pick2.y, pick2.z, pick2.rx, pick2.ry, pick2.rz);
            robot.GetInverseKin(0, descPosPick2, -1, ref jointPosPick2);

            #endregion

            #region Punto avvicinamento pick

            // Prima fase del ciclo
            JointPos jointPosApproachPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPick2 = new DescPose(pick2.x, pick2.y - offsetAvvicinamento, pick2.z - 40, pick2.rx, pick2.ry, pick2.rz);
            robot.GetInverseKin(0, descPosApproachPick2, -1, ref jointPosApproachPick2);

            #endregion

            #region Punto post pick

            JointPos jointPosPostPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPick2 = new DescPose(pick2.x, pick2.y, pick2.z + 40, pick2.rx, pick2.ry, pick2.rz);
            robot.GetInverseKin(0, descPosPostPick2, -1, ref jointPosPostPick2);

            #endregion

            #region Punto allontanamento post pick

            JointPos jointPosAllontanamentoPick2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPick2 = new DescPose(descPosPick2.tran.x, descPosPick2.tran.y - offsetAllontamento,
                descPosPick2.tran.z + 40, descPosPick2.rpy.rx, descPosPick2.rpy.ry, descPosPick2.rpy.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPick2, -1, ref jointPosAllontanamentoPick2);

            #endregion


            #region Punto avvicinamento place

            JointPos jointPosApproachPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosApproachPlace2 = new DescPose(pick.x - offsetAvvicinamento, pick.y, pick.z + 50,
                pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosApproachPlace2, -1, ref jointPosApproachPlace2);

            #endregion

            #region Punto di place

            JointPos jointPosPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPlace2 = new DescPose(pick.x - 10, pick.y, pick.z, pick.rx, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPlace2, -1, ref jointPosPlace2);

            #endregion

            #region Punto post place

            JointPos jointPosPostPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosPostPlace2 = new DescPose(pick.x, pick.y, pick.z,
                pick.rx - 3, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosPostPlace2, -1, ref jointPosPostPlace2);

            #endregion

            #region Punto allontanamento place

            JointPos jointPosAllontanamentoPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            DescPose descPosAllontanamentoPlace2 = new DescPose(pick.x - offsetAvvicinamento, pick.y, pick.z,
                pick.rx - 3, pick.ry, pick.rz);
            robot.GetInverseKin(0, descPosAllontanamentoPlace2, -1, ref jointPosAllontanamentoPlace2);

            #endregion



            #endregion

            // Posa pre pick 1
            DescPose prePick1Pose = new DescPose(
                                descPosHome.tran.x,
                                descPosHome.tran.y,
                                descPosPick.tran.z - 40, // Mi abbasso di 4 cm prima di andare in pick
                                descPosHome.rpy.rx,
                                descPosHome.rpy.ry,
                                descPosHome.rpy.rz
                                );

            JointPos jointPrePick1 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, prePick1Pose, -1, ref jointPrePick1);

            // Posa post pick 1
            DescPose postPick1Pose = new DescPose(
                              descPosHome.tran.x,
                              descPosHome.tran.y,
                              descPosPick.tran.z + 20, // Mi alzo di 2 cm per uscire dal carrello senza strisciare
                              NormalizeAngle((float)(descPosPick.rpy.rx + 2)),
                              descPosHome.rpy.ry,
                              descPosHome.rpy.rz
                              );

            JointPos jointPostPick1 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, postPick1Pose, -1, ref jointPostPick1);

            // Posa per ruotare il robot dopo il pick 1 e prima di andare al place 1
            DescPose postPick1RotationPose = new DescPose(
                descPosHome.tran.x,
                descPosPlace.tran.y,
                descPosHome.tran.z,
                descPosPlace.rpy.rx,
                descPosPlace.rpy.ry,
                descPosPlace.rpy.rz
                );

            // Posa post place 1
            DescPose postPlace1Pose = new DescPose(
                descPosHome.tran.x,
                descPosPlace.tran.y,
                descPosPick.tran.z + 20, // Mi alzo di 2 cm prima di andare in home
                descPosHome.rpy.rx,
                descPosHome.rpy.ry,
                descPosPlace.rpy.rz
                );

            JointPos jointPostPlace1 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, postPlace1Pose, -1, ref jointPostPlace1);

            // Posa per ruotare il robot dopo il place 1 e prima di andare al place 2
            DescPose postPlace1RotationPose = new DescPose(
                descPosHome.tran.x,
                descPosHome.tran.y,
                descPosHome.tran.z + 20,
                descPosHome.rpy.rx,
                descPosHome.rpy.ry,
                descPosHome.rpy.rz);

            DescPose place2Pose = descPosPick;
            JointPos jointPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, place2Pose, -1, ref jointPlace2);

            // Posa per ruotare il robot dopo il place 1 e prima di andare al place 2
            DescPose postPlace2Pose = new DescPose(
                descPosHome.tran.x,
                descPosHome.tran.y,
                descPosPick.tran.z - 20, // Mi alzo di 4 cm prima di andare in home
                descPosPick.rpy.rx - 4,
                descPosHome.rpy.ry,
                descPosHome.rpy.rz
                );

            JointPos jointPostPlace2 = new JointPos(0, 0, 0, 0, 0, 0);
            robot.GetInverseKin(0, postPlace2Pose, -1, ref jointPostPlace2);

            #endregion

            #region Parametri movimento

            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)

            // Indica il codice risultante del movimento del Robot
            int movementResult = -1;

            // Reset condizione di stop ciclo
            stopCycleRoutine = false;

            // Reset richiesta di stop ciclo
            stopCycleRequested = false;

            // Reset step routine
            step = 0;

            // Segnale di pick
            bool prendidaNastro = true;

            #endregion

            double[] levelCollision1 = new double[] { 1, 1, 1, 1, 1, 1 };
            double[] levelCollision2 = new double[] { 2, 2, 2, 2, 2, 2 };
            double[] levelCollision3 = new double[] { 3, 3, 3, 3, 3, 3 };
            double[] levelCollision4 = new double[] { 4, 4, 4, 4, 4, 4 };
            double[] levelCollision5 = new double[] { 5, 5, 5, 5, 5, 5 };
            double[] levelCollision6 = new double[] { 6, 6, 6, 6, 6, 6 };
            double[] levelCollision7 = new double[] { 7, 7, 7, 7, 7, 7 };
            double[] levelCollision8 = new double[] { 8, 8, 8, 8, 8, 8 };

            robot.SetAnticollision(0, levelCollision6, 1);

            byte ris = 0;

            // Aspetto che il metodo termini, ma senza bloccare il thread principale
            // La routine è incapsulata come 'async' per supportare futuri operatori 'await' nel caso ci fosse la necessità
            await Task.Run(async () =>
            {
                // Fino a quando la condizione di stop routine non è true e non sono presenti allarmi bloccanti
                while (!stopCycleRoutine && !AlarmManager.blockingAlarm)
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 0 - Check richiesta interruzione ciclo");
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

                            formDiagnostics.UpdateRobotStepDescription("STEP 5 - Termine routine");
                            break;

                        #endregion

                        case 10:
                            #region Movimento a punto di Pick

                            inPosition = false; // Reset inPosition

                            // Movimento in posa di pre pick 1
                            movementResult = robot.MoveL(jointPrePick1, prePick1Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento in posa di pick 1
                            movementResult = robot.MoveL(jointPosPick, descPosPick, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            endingPoint = descPosPick; // Assegnazione endingPoint

                            step = 30; // Passaggio a step 30

                            formDiagnostics.UpdateRobotStepDescription("STEP 10 - Movimento a punto di Pick 1");

                            break;

                        #endregion

                        case 20:
                            #region Delay per calcolo in position punto di pick 1

                            await Task.Delay(500);

                            step = 30; // Passaggio a step 30

                            formDiagnostics.UpdateRobotStepDescription("STEP 20 -  Delay calcolo in position punto di pick 1");

                            break;

                        #endregion

                        case 30:
                            #region Attesa inPosition punto di Pick e chiusura pinza

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick 1
                            {
                                // Chiudo la pinza
                                robot.SetDO(0, 1, 0, 0);

                                step = 40; // Passaggio a step 40
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 30 - Attesa inPosition punto di Pick 1");

                            break;

                        #endregion

                        case 40:
                            #region Delay post chiusura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 0)
                            {

                                //await Task.Delay(800); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza
                                step = 50;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 40 - Delay post chiusura pinza in fase di pick 1");

                            break;

                        #endregion

                        case 50:
                            #region Movimento di uscita dal carrello dopo pick 1

                            // Movimento per uscire dal carrelo dopo pick 1
                            movementResult = robot.MoveL(jointPostPick1, postPick1Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            formDiagnostics.UpdateRobotStepDescription("STEP 50 - Movimento di uscita dal carrello dopo pick 1");

                            step = 60;

                            break;

                        #endregion

                        case 60:
                            #region Movimento a punto di place 1

                            inPosition = false; // Reset inPosition

                            // Movimento di rotazione prima di andare in place 1
                            movementResult = robot.MoveCart(postPick1RotationPose, tool, user, vel / 2, acc / 2, ovl, blendT, config);
                            GetRobotMovementCode(movementResult);

                            // Movimento in place 1
                            movementResult = robot.MoveL(jointPosPlace, descPosPlace, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            endingPoint = descPosPlace; // Assegnazione endingPoint

                            step = 80; // Passaggio a step 80

                            formDiagnostics.UpdateRobotStepDescription("STEP 60 - Movimento a punto di place 1");

                            break;

                        #endregion

                        case 70:
                            #region Delay per calcolo in position punto di place

                            await Task.Delay(500);
                            step = 80; // Passaggio a step 80

                            formDiagnostics.UpdateRobotStepDescription("STEP 70 -  Delay calcolo in position punto di place (prima fase)");

                            break;

                        #endregion

                        case 80:
                            #region Attesa inPosition punto di Place 1

                            if (inPosition) // Se il Robot è arrivato in posizione di place
                            {
                                await Task.Delay(1000);
                                step = 100; // Passaggio a step 100
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 80 - Attesa inPosition punto di place 1");

                            break;

                        #endregion

                        case 90:
                            #region Rilascio teglia

                            // if (gripperStatus == 0)
                            {
                                await Task.Delay(500);
                                robot.SetDO(0, 0, 0, 0); // Apro la pinza
                                step = 100;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 90 - Rilascio teglia (prima fase)");
                            break;

                        #endregion

                        case 100:
                            #region Movimento per uscire da place 1 ed entrare in place 2

                            // Movimento per uscire da place 1
                            movementResult = robot.MoveL(jointPostPlace1, postPlace1Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult); // Stampo risultato del movimento

                            // Movimento di preparazione al place 2
                            movementResult = robot.MoveCart(postPlace1RotationPose, tool, user, vel / 2, acc / 2, ovl, blendT, config);
                            GetRobotMovementCode(movementResult);

                            endingPoint = postPlace1RotationPose;

                            formDiagnostics.UpdateRobotStepDescription("STEP 100 - Movimento a punto di place 2");

                            step = 105;

                            break;

                        #endregion

                        case 105:
                            #region Attesa inPosition punto pre place 2

                            if (inPosition) // Se il Robot è arrivato in posizione di place 2
                            {
                                //  await Task.Delay(300);
                                // robot.SetAnticollision(0, levelCollision5, 1);

                                // Movimento a posa di place 2
                                movementResult = robot.MoveL(jointPlace2, place2Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                                endingPoint = place2Pose; // Assegnazione ending point

                                formDiagnostics.UpdateRobotStepDescription("STEP 100 - Movimento a punto di place 2");

                                step = 130; // Passaggio a step 140
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 130 - Attesa inPosition punto pre place");

                            break;

                        #endregion

                        case 120:
                            #region Delay per calcolo in position punto di place 2

                            await Task.Delay(500);
                            step = 130;

                            formDiagnostics.UpdateRobotStepDescription("STEP 120 -  Delay calcolo in position punto di place 2");
                            break;

                        #endregion

                        case 130:
                            #region Attesa inPosition punto di place 2

                            if (inPosition) // Se il Robot è arrivato in posizione di place 2
                            {
                                robot.SetDO(0, 0, 0, 0); // Apro la pinza

                                step = 140; // Passaggio a step 140
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 130 - Attesa inPosition punto di place 2 e apertura pinza");

                            break;

                        #endregion

                        case 140:
                            #region Delay post apertura pinza

                            robot.GetDI(0, 1, ref ris);

                            if (ris == 1)
                            {
                                // await Task.Delay(300); // // Ritardo per evitare che il robot riparta senza aver finito di aprire la pinza
                                //  robot.SetAnticollision(0, levelCollision5, 1);

                                step = 150; // Passaggio a step 150
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 140 - Delay post apertura pinza in place 2");

                            break;

                        #endregion

                        case 150:
                            #region Movimento per uscire dal carrello dopo il place 2

                            // Movimento per uscire dal carrello dopo il place 2
                            movementResult = robot.MoveL(jointPostPlace2, postPlace2Pose, tool, user, vel, acc, ovl, blendT, epos, 0, offsetFlag, offset);

                            formDiagnostics.UpdateRobotStepDescription("STEP 150 - Movimento per uscire dal carrello dopo il place 2");

                            await Task.Delay(500); // Delay prima di riavviare il ciclo
                            step = 0;
                            break;
                        #endregion

                        case 160:
                            #region Movimento a punto di Place

                            if (prendidaNastro)
                            {
                                inPosition = false; // Reset inPosition

                                // STEP 3: Invio punto di avvicinamento Pick
                                /* movementResult = robot.MoveCart(descPosApproachPlace2, tool, user, vel, acc,
                                     ovl, blendT, config);
                                 GetRobotMovementCode(movementResult);*/

                                // STEP 4: Movimento lineare finale
                                movementResult = robot.MoveL(jointPosApproachPlace2, descPosApproachPlace2, tool, user, vel, acc,
                                    ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult);

                                // STEP 4: Movimento lineare finale
                                movementResult = robot.MoveL(jointPosPlace2, descPosPlace2, tool, user, vel, acc,
                                    ovl, blendT, epos, 0, offsetFlag, offset);
                                GetRobotMovementCode(movementResult);

                                endingPoint = descPosPlace2; // Assegnazione endingPoint
                                step = 170;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 160 - Movimento a punto di Place (seconda fase)");
                            break;
                        #endregion


                        case 170:
                            #region Delay per calcolo in position punto di place

                            // Thread.Sleep(500);
                            step = 180;
                            formDiagnostics.UpdateRobotStepDescription("STEP 170 -  Delay calcolo in position punto di place (seconda fase)");
                            break;

                        #endregion

                        case 180:
                            #region Attesa inPosition punto di Place

                            if (inPosition) // Se il Robot è arrivato in posizione di Pick
                            {
                                // Thread.Sleep(500);
                                // Chiudo la pinza

                                step = 190;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 180 - Attesa inPosition punto di place (seconda fase)");
                            break;

                        #endregion

                        case 190:
                            #region Rilascio teglia

                            // if (gripperStatus == 0)
                            {
                                Thread.Sleep(500); // Per evitare "rimbalzo" del Robot
                                step = 200;
                            }

                            formDiagnostics.UpdateRobotStepDescription("STEP 190 - Rilascio teglia (seconda fase)");
                            break;

                        #endregion

                        case 200:
                            #region Movimento a punto di Home e riavvio ciclo

                            movementResult = robot.MoveL(jointPosPostPlace2, descPosPostPlace2, tool, user, vel, acc,
                                   ovl, blendT, epos, 0, offsetFlag, offset);
                            GetRobotMovementCode(movementResult);

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosAllontanamentoPlace2, descPosAllontanamentoPlace2, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            /*
                            movementResult = robot.MoveCart(descPosHome, tool, user, vel, acc,
                                ovl, blendT, config); // Invio punto di Home
                            GetRobotMovementCode(movementResult);
                            */

                            // Movimento lineare
                            movementResult = robot.MoveL(jointPosHome, descPosHome, tool, user, vel, acc,
                                ovl, blendT, epos, 0, offsetFlag, offset
                                );

                            formDiagnostics.UpdateRobotStepDescription("STEP 200 - Movimento a punto di Home e riavvio ciclo");

                            step = 0;
                            break;

                            #endregion
                    }

                    Thread.Sleep(50); // Delay routine
                }
            });


        }

        #endregion
    }
}
