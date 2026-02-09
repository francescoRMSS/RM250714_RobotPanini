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
using RM.src.RM250714.Classes.PLC;
using RM.src.RM250714.Classes.FR20.Jog;
using RM.src.RM250714.Classes.FR20;
using RM.src.RM250714.Classes.FR20.Properties;
using CookComputing.XmlRpc;
using System.IO;
using RM.src.RM250714.Classes.FR20.Exceptions;
using RMLib.TaskManager;

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
        protected static Robot Robot
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
        /// Indica se è possibile avere più zone attive allo stesso tempo
        /// </summary>
        public static bool useOverlappingZones = false;
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
        /// Livello di collisioni da db
        /// </summary>
        public static int collisionLevel = 0;
        /// <summary>
        /// Livello di collisione corrente.
        /// </summary>
        public static int currentCollisionLevel = 0;
        /// <summary>
        /// Livello di collisione service
        /// </summary>
        public static int collisionLevelService = 0;
        /// <summary>
        /// Richiesta cambio livello di collisione
        /// </summary>
        public static int changeCollisionLevel = 0;
        /// <summary>
        /// Stato precedente della richiesta di cambio collisione
        /// </summary>
        public static int? prevChangeCollisionLevel;
        /// <summary>
        /// Tempo massimo in ms per controllare che il proxy stia comunicando
        /// </summary>
        private const int connectionCheckMaxTimeout = 500;
        /// <summary>
        /// Numero corrente di errori di connessione -2
        /// </summary>
        private static int currentConnectionErrorTries = 0;
        /// <summary>
        /// Numero massimo accettato di errori di connessione -2
        /// </summary>
        private const int connectionErrorMaxTries = 10;

        #endregion

        #region Allarmi pr il PLC

        #region Allarmi relativi al robot

        private static readonly object _robotErrorLock = new object();

        /// <summary>
        /// 1 in caso di allarme del robot
        /// </summary>
        private static bool RobotGeneralError
        {
            get { lock (_robotErrorLock) { return _robotGeneralError; } }
            set { lock (_robotErrorLock) { _robotGeneralError = value; } }
        }

        /// <summary>
        /// 1 quando il robot è in allarme 
        /// </summary>
        private static bool RobotError
        {
            get { lock (_robotErrorLock) { return _robotError; } }
            set { lock (_robotErrorLock) { _robotError = value; } }
        }

        /// <summary>
        /// 1 quando il robot non riesce a calcolare una cinematica inversa o diretta
        /// </summary>
        private static bool RobotKinError
        {
            get { lock (_robotErrorLock) { return _robotKinError; } }
            set { lock (_robotErrorLock) { _robotKinError = value; } }
        }

        /// <summary>
        /// 1 quando il robot restituisce un allarme da un movimento
        /// </summary>
        private static bool RobotMovementError
        {
            get { lock (_robotErrorLock) { return _robotMovementError; } }
            set { lock (_robotErrorLock) { _robotMovementError = value; } }
        }

        /// <summary>
        /// 1 quando il robot restituisce un allarme da un metodo che cambia alcune sue proprietà
        /// </summary>
        public static bool RobotPropertiesError
        {
            get { lock (_robotErrorLock) { return _robotPropertiesError; } }
            set { lock (_robotErrorLock) { _robotPropertiesError = value; } }
        }

        /// <summary>
        /// 0 quando il robot restituisce -2 n volte o il task segnala una disconnessione
        /// </summary>
        private static bool RobotConnected
        {
            get { lock (_robotErrorLock) { return _robotConnected; } }
            set { lock (_robotErrorLock) { _robotConnected = value; } }
        }

        /// <summary>
        /// 1 in caso di allarme del robot
        /// </summary>
        private static bool _robotGeneralError = false; //bit 0
        /// <summary>
        /// 1 quando il robot è in allarme 
        /// </summary>
        private static bool _robotError = false; //bit 1
        /// <summary>
        /// 1 quando il robot non riesce a calcolare una cinematica inversa o diretta
        /// </summary>
        private static bool _robotKinError = false; //bit 2
        /// <summary>
        /// 1 quando il robot restituisce un allarme da un movimento
        /// </summary>
        private static bool _robotMovementError = false; //bit 3
        /// <summary>
        /// 1 quando il robot restituisce un allarme da un metodo che cambia alcune sue proprietà
        /// </summary>
        private static bool _robotPropertiesError = false; //bit 4
        /// <summary>
        /// 0 quando il robot restituisce -2 n volte o il task segnala una disconnessione
        /// </summary>
        private static bool _robotConnected = true; //bit 5

        #endregion

        #region Allarmi relativi all'applicazione

        private static readonly object _applicationErrorLock = new object();

        /// <summary>
        /// 1 quando un ciclo va in eccezione durante l'esecuzione
        /// </summary>
        private static bool RunTimeError
        {
            get { lock (_applicationErrorLock) { return _runTimeError; } }
            set { lock (_applicationErrorLock) { _runTimeError = value; } }
        }

        /// <summary>
        /// 1 quando alcuni dati nel ciclo sono mancanti o errati
        /// </summary>
        private static bool DataError
        {
            get { lock (_applicationErrorLock) { return _dataError; } }
            set { lock (_applicationErrorLock) { _dataError = value; } }
        }

        /// <summary>
        /// 1 quando mancano i consensi per la home routine o sono avvenuti degli errori nel ciclo
        /// </summary>
        private static bool HomeRoutineError
        {
            get { lock (_applicationErrorLock) { return _homeRoutineError; } }
            set { lock (_applicationErrorLock) { _homeRoutineError = value; } }
        }

        /// <summary>
        /// 1 quando dopo aver fatto il pick della teglia, la fotocellula non rileva la teglia
        /// </summary>
        private static bool TrayNotPresentError
        {
            get { lock (_applicationErrorLock) { return _trayNotPresentError; } }
            set { lock (_applicationErrorLock) { _trayNotPresentError = value; } }
        }

        /// <summary>
        /// 1 quando dopo aver fatto il pick della teglia la pinza non ha raggiunto il fine corsa
        /// </summary>
        private static bool GripperNotClosedError
        {
            get { lock (_applicationErrorLock) { return _gripperNotClosedError; } }
            set { lock (_applicationErrorLock) { _gripperNotClosedError = value; } }
        }

        /// <summary>
        /// 1 quando dopo aver tirato fuori la slitta, il sensore rileva che la slitta non è fuori
        /// </summary>
        private static bool SlideNotOutError
        {
            get { lock (_applicationErrorLock) { return _slideNotOutError; } }
            set { lock (_applicationErrorLock) { _slideNotOutError = value; } }
        }

        /// <summary>
        /// 1 quando dopo aver tirato dentro la slitta, il sensore rileva che la slitta non è dentro
        /// </summary>
        private static bool SlideNotInError
        {
            get { lock (_applicationErrorLock) { return _slideNotInError; } }
            set { lock (_applicationErrorLock) { _slideNotInError = value; } }
        }

        /// <summary>
        /// 1 quando un ciclo va in eccezione durante l'esecuzione
        /// </summary>
        private static bool _runTimeError = false; //bit 0
        /// <summary>
        /// 1 quando alcuni dati nel ciclo sono mancanti o errati
        /// </summary>
        private static bool _dataError = false; //bit 1
        /// <summary>
        /// 1 quando mancano i consensi per la home routine o sono avvenuti degli errori nel ciclo
        /// </summary>
        private static bool _homeRoutineError = false; //bit 2
        /// <summary>
        /// 1 quando dopo aver fatto il pick della teglia, la fotocellula non rileva la teglia
        /// </summary>
        private static bool _trayNotPresentError = false; //bit 3
        /// <summary>
        /// 1 quando dopo aver fatto il pick della teglia la pinza non ha raggiunto il fine corsa
        /// </summary>
        private static bool _gripperNotClosedError = false; //bit 4
        /// <summary>
        /// 1 quando dopo aver tirato fuori la slitta, il sensore rileva che la slitta non è fuori
        /// </summary>
        private static bool _slideNotOutError = false; //bit 5
        /// <summary>
        /// 1 quando dopo aver tirato dentro la slitta, il sensore rileva che la slitta non è dentro
        /// </summary>
        private static bool _slideNotInError = false; //bit 6

        #endregion

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
        /// A true quando il robot si trova in safe zone.
        /// </summary>
        public static bool isInSafeZone = false;
        /// <summary>
        /// A true quando il robot si trova in home zone.
        /// </summary>
        public static bool isInHomePosition = false;
        /// <summary>
        /// A true quando si trova in posizione di Pick
        /// </summary>
        public static bool isInPositionCarrello1 = false;
        /// <summary>
        /// A true quando si trova in posizione di Place
        /// </summary>
        public static bool isInPositionCarrello2 = false;
        /// <summary>
        /// A true quando si trova in posizione di beor
        /// </summary>
        public static bool isInPositionBeor = false;
        /// <summary>
        /// A true quando si trova in posizione di home
        /// </summary>
        public static bool isInPositionHome = false;
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

        // --- Stato Abilitazione e Modalità ---
        /// <summary>
        /// Stato attuale isEnabled del robot.
        /// </summary>
        public static bool isEnabledNow = false;
        /// <summary>
        /// Rappresenta il valore della modalità Off nello step precedente.
        /// </summary>
        private static bool prevIsOff = false;
        /// <summary>
        /// Modalità precedente letta dal PLC.
        /// </summary>
        private static int lastMode = -1;
        /// <summary>
        /// Modalità stabile da impostare dopo il debounce.
        /// </summary>
        private static int stableMode = -1;
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
        private static readonly int homeRoutineSpeed = 5;
        /// <summary>
        /// Velocity utilizzata in home routine
        /// </summary>
        private static readonly int homeRoutineVel = 100;
        /// <summary>
        /// Acceleration utilizzata in home routine
        /// </summary>
        private static readonly int homeRoutineAcc = 100;

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
        /// Checker per zona di pick
        /// </summary>
        private static PositionChecker checker_ingombro_carrello1;
        /// <summary>
        /// Checker per zona di place
        /// </summary>
        private static PositionChecker checker_ingombro_carrello2;
        /// <summary>
        /// Checker per zona ingombro home
        /// </summary>
        private static PositionChecker checker_ingombro_home;
        /// <summary>
        /// Checker per zona ingombro beor
        /// </summary>
        private static PositionChecker checker_ingombro_beor;

        #endregion

        #region Tempi di Delay dei Task

        /// <summary>
        /// Periodo di refresh per il task ad alta priorità.
        /// </summary>
        private readonly static int highPriorityRefreshPeriod = 50;
        /// <summary>
        /// Periodo di refresh per il task degli ausiliari.
        /// </summary>
        private readonly static int auxiliaryThreadRefreshPeriod = 150;
        /// <summary>
        /// Periodo di refresh per il task a bassa priorità.
        /// </summary>
        private readonly static int lowPriorityRefreshPeriod = 300;
        /// <summary>
        /// Periodo di refresh per il task che comunica al plc
        /// </summary>
        private readonly static int plcComTaskRefreshPeriod = 300;
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

        #region Stati precedenti dei comandi

        /// <summary>
        /// Memorizza lo stato precedente della variabile open/close grippers dal PLC
        /// </summary>
        private static bool previousGripperStatus = false;
        /// <summary>
        /// Memorizza lo stato precedente della variabile start ciclo dal PLC
        /// </summary>
        private static bool previousStartCommandStatus = false;
        /// <summary>
        /// Memorizza lo stato precedente della variabile stop ciclo dal PLC
        /// </summary>
        private static int previousStopCommandStatus = 0;
        /// <summary>
        /// Memorizza lo stato precedente della variabile richiesta stop ciclo dal PLC
        /// </summary>
        private static int previousRequestedStopCommandStatus = 0;
        /// <summary>
        /// Memorizza lo stato precedente della variabile go to home position dal PLC
        /// </summary>
        private static bool previousHomeCommandStatus = false;

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

        #endregion

        #region Step cicli

        /// <summary>
        /// Memorizza lo stato precedente della variabile on/off barrier status dal PLC
        /// </summary>
        private static int previousBarrierPauseStatus = -1;
        /// <summary>
        /// Memorizza lo stato precedente della variabile on/off barrier status dal PLC
        /// </summary>
        private static int previousBarrierResumeStatus = -1;
        /// <summary>
        /// Memorizza lo stato precedente della richiesta di registrazione punto
        /// </summary>
        private static int previousRecordPointRequest = -1;
        /// <summary>
        /// Memorizza lo stato precedente della richiesta di reset degli allarmi
        /// </summary>
        private static int previousAlarmResetRequested = -1;
        /// <summary>
        /// Riferimento allo step delle normal variables corrente
        /// </summary>
        public static int step = 0;
        /// <summary>
        /// A true quando viene richiesto lo stop del ciclo routine del robot
        /// </summary>
        public static bool robotCycleStopRequested = false;
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
        /// Nome del task che gestisce il ciclo teglie
        /// </summary>
        public static string TaskPickAndPlaceTegliaIperal = nameof(PickAndPlaceTegliaIperal);

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
        public static async Task<bool> InitRobot(string robotIpAddress)
        {
            formAlarmPage = new FormAlarmPage();
            formAlarmPage.AlarmsCleared += RMLib_AlarmsCleared;

            formDiagnostics = new FormDiagnostics();

            // Istanzio il robot
            RobotIpAddress = robotIpAddress;
            Robot = new Robot();
            string logDirectory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Logs");
            int errLogInit = Robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.ERROR, logDirectory, 2, 2);
            int errRPC = Robot.RPC(RobotIpAddress);
            AlarmManager.isRobotConnected = true;

            RefresherTask.AddUpdate(PLCTagName.Automatic_Start, 0, "INT16");
            RefresherTask.AddUpdate(PLCTagName.VersionYear, 2026, "INT16");
            RefresherTask.AddUpdate(PLCTagName.VersionMonth, 02, "INT16");
            RefresherTask.AddUpdate(PLCTagName.VersionDay, 06, "INT16");

            // Faccio partire i manager
            frameManager = new Frames();
            toolManager = new Tools();
            collisionManager = new Collisions();

            // Inizializzazione mode
            ROBOT_STATE_PKG robot_state_pkg = new ROBOT_STATE_PKG();
            GetRobotRealTimeState(ref robot_state_pkg);
            currentRobotMode = robot_state_pkg.robot_mode;
            currentRobotEnableStatus = robot_state_pkg.rbtEnableState;
            robotStatus = robot_state_pkg.robot_state;
            isAutomaticMode = currentRobotMode == 0;

            // Procedura di stop robot e lettura stato per poter impostare i parametri corretti
            StopMotion();
            while(robotStatus != 1)
            {
                GetRobotRealTimeState(ref robot_state_pkg);
                robotStatus = robot_state_pkg.robot_state;
                await Task.Delay(100);
            }

            // Se fallisce setting della proprietà del Robot
            if (!GetRobotProperties())
                return false;

            if (!SetRobotProperties())
            {
                log.Error("Errore durante set parametri del robot");
            }

            log.Info("Parametri del robot assegnati");

            // Faccio partire i task
            taskManager.AddTask(TaskCheckRobotConneciton, CheckRobotConnection, TaskType.LongRunning, true);
            taskManager.AddTask(TaskHighPriorityName, CheckHighPriority, TaskType.LongRunning, true);
            taskManager.AddTask(TaskAuxiliaryWorkerName, AuxiliaryWorker, TaskType.LongRunning, true);
            taskManager.AddTask(TaskLowPriorityName, CheckLowPriority, TaskType.LongRunning, true);
            taskManager.AddTask(TaskApplicationManager, ApplicationTaskManager, TaskType.LongRunning, true);
            taskManager.AddTask(TaskPlcComHandlerName, PlcComHandler, TaskType.LongRunning, true);
            taskManager.AddTask(TaskSafetyManager, SafetyTaskManager, TaskType.LongRunning, true);

            taskManager.StartTask(TaskCheckRobotConneciton);
            taskManager.StartTask(TaskHighPriorityName);
            taskManager.StartTask(TaskAuxiliaryWorkerName);
            taskManager.StartTask(TaskLowPriorityName);
            taskManager.StartTask(TaskApplicationManager);
            taskManager.StartTask(TaskPlcComHandlerName);
            taskManager.StartTask(TaskSafetyManager);

            log.Info("Task di background del robot avviati tramite TaskManager.");

            if (errRPC != 0)
            {
                log.Error("RPC exception durante Init del Robot");
                //return false;
            }

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
            #region ingombri
            
            // Zone di ingombro
            var carrello1 = ApplicationConfig.applicationsManager.GetPosition("pIngombroCarrello1", "RM");
            var carrello2 = ApplicationConfig.applicationsManager.GetPosition("pIngombroCarrello2", "RM");
            var homePose = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            var IngombroBeorPose = ApplicationConfig.applicationsManager.GetPosition("pIngombroBeor", "RM");

            DescPose[] startPoints = new DescPose[]
            {
                new DescPose(homePose.x, homePose.y, homePose.z, homePose.rx, homePose.ry, homePose.rz),
                new DescPose(carrello1.x, carrello1.y, carrello1.z, carrello1.rx, carrello1.ry, carrello1.rz),
                new DescPose(carrello2.x, carrello2.y, carrello2.z, carrello2.rx, carrello2.ry, carrello2.rz),
                new DescPose(IngombroBeorPose.x, IngombroBeorPose.y, IngombroBeorPose.z, IngombroBeorPose.rx, IngombroBeorPose.ry, IngombroBeorPose.rz),
            };

            // Oggetto che rileva ingombro carrello 1 [parallelepipedo]
            double lenght_carrello1 = 300.0;
            double width_carrello1 = 2000.0;
            double height_carrello1 = 2000.0;
            checker_ingombro_carrello1 = new PositionChecker(lenght_carrello1, width_carrello1, height_carrello1);

            // Oggetto che rileva ingombro carrello 2 [parallelepipedo]
            double lenght_carrello2 = 300.0;
            double width_carrello2 = 2000.0;
            double height_carrello2 = 2000.0;
            checker_ingombro_carrello2 = new PositionChecker(lenght_carrello2, width_carrello2, height_carrello2);

            // Oggetto che rileva ingombro home
            double delta_ingombro_home = 300.0;
            checker_ingombro_home = new PositionChecker(delta_ingombro_home);

            // Oggetto che rileva ingombro macchina beor [parallelepipedo]
            double lenght_beor = 1000.0;
            double width_beor = 200.0;
            double height_beor = 1500.0;
            checker_ingombro_beor = new PositionChecker(lenght_beor, width_beor, height_beor);

            #endregion

            #region Safe zone

            // Dichiarazione del punto di safe
            var pSafeZone = ApplicationConfig.applicationsManager.GetPosition("pSafeZone", "RM");

            DescPose pointSafeZone = new DescPose(pSafeZone.x, pSafeZone.y, pSafeZone.z, pSafeZone.rx, pSafeZone.ry, pSafeZone.rz);

            // Oggetto che rileva ingombro safe zone [parallelepipedo]
            double lenght_safeZone = 2000.0;
            double width_safeZone = 1200.0;
            double height_safeZone = 2000.0;
            checker_safeZone = new PositionChecker(lenght_safeZone, width_safeZone, height_safeZone);

            #endregion

            try
            {
                if (carrello1 == null)
                    throw new DataErrorException("Il punto di ingombro carrello 1 non esiste");

                if (carrello2 == null)
                    throw new DataErrorException("Il punto di ingombro carrello 2 non esiste");

                if (homePose == null)
                    throw new DataErrorException("Il punto di ingombro home non esiste");

                if (IngombroBeorPose == null)
                    throw new DataErrorException("Il punto di ingombro beor non esiste");

                if (pSafeZone == null)
                    throw new DataErrorException("Il punto di safe zone non esiste");

                while (!token.IsCancellationRequested)
                {
                    if (AlarmManager.isRobotConnected)
                    {
                        await CheckIsRobotEnable();
                        await CheckRobotMode();
                        CheckLevelCollision();
                        //CheckGripperStatus();
                        CheckIsRobotInObstructionArea(startPoints);
                        CheckIsRobotInSafeZone(pointSafeZone);
                    }

                    await Task.Delay(auxiliaryThreadRefreshPeriod, token);
                }

                token.ThrowIfCancellationRequested(); //Solleva eccezione per far andare in stop e non in completed
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (DataErrorException ex)
            {
                log.Error($"[TASK] {TaskAuxiliaryWorkerName} Data error: {ex}");
                DataError = true;
            }
            catch (Exception ex)
            {
                log.Error($"[TASK] {TaskAuxiliaryWorkerName}: {ex}");
                RunTimeError = true;
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
           checker_pos = new PositionChecker(5.0);

            try
            {
                while (!token.IsCancellationRequested)
                {
                    if (Robot != null && AlarmManager.isRobotConnected)
                    {
                        try
                        {
                            GetActualTcpPose(flag, ref TCPCurrentPosition); // Leggo posizione robot TCP corrente
                            CheckIsRobotMoving();
                            CheckIsRobotInPos();
                            CheckStatusRobot();
                        }
                        catch (Exception e)
                        {
                            log.Error("RobotManager: errore durante la valutazione delle variabili HIGH: " + e.Message);
                        }
                    }

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
                RunTimeError = true;
                throw;
            }
            finally
            {

            }
        }

        /// <summary>
        /// Thread che gestisce il controllo connessione plc e la scrittura degli aggiornamenti delle variabili
        /// </summary>
        /// <param name="token"></param>
        /// <returns></returns>
        private async static Task PlcComHandler(CancellationToken token)
        {
            JointPos jPos = new JointPos(0, 0, 0, 0, 0, 0);

            const int LOW_PRIORITY_DELAY = 2;
            int lowPriorityDelayNum = 0;

            try
            {
                while (!token.IsCancellationRequested)
                {
                    lowPriorityDelayNum++;
                    SendHighPriorityUpdatesToPLC();

                    if (lowPriorityDelayNum >= LOW_PRIORITY_DELAY)
                    {
                        CheckRobotPosition(jPos);
                        SendUpdatesToPLC();

                        lowPriorityDelayNum = 0;
                    }

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
                RunTimeError = true;
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
                    CheckCurrentToolAndUser();

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
                RunTimeError = true;
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
            const int MAX_FAILURE_ATTEMPTS = 3; // Tentativi consecutivi prima di dichiarare la disconnessione
            int consecutiveFailures = 0; // Numero di tentativi correnti per il proxy
            bool isReconnectionNeeded = false; // Indica se la riconnessione è necessaria

            try
            {
                //Istanzio il proxy che interfaccia il controllore in maniera diretta
                ICallSupervisor connectionProxy = XmlRpcProxyGen.Create<ICallSupervisor>();
                connectionProxy.Url = $"http://{RobotIpAddress}:20003/RPC2";
                connectionProxy.Timeout = connectionCheckMaxTimeout; // Timeout breve per non bloccare troppo a lungo.

                log.Warn("[Robot COM] Task di controllo connessione avviato.");

                while (!token.IsCancellationRequested)
                {
                    bool isProxyConnected;
                    try
                    {
                        // --- IL CONTROLLO DIRETTO SUL PROXY ---
                        // Eseguiamo la chiamata RPC su un thread del pool per non bloccare
                        // il nostro loop while nel caso in cui il timeout non funzioni bene.
                        connectionProxy.GetRobotErrorCode();
                        isProxyConnected = true;
                    }
                    catch (Exception) // Qualsiasi eccezione (XmlRpcException, WebException) significa che non siamo connessi.
                    {
                        isProxyConnected = false;
                    }

                    // Controllo aggiuntivo che guarda il numero di volte che un get status restituisce errore -2
                    if (currentConnectionErrorTries >= connectionErrorMaxTries * 2)
                    {
                        isProxyConnected = false;
                        isReconnectionNeeded = true; // Abilita la condizione di riconnessione
                    }

                    if (isProxyConnected) //Connessione verificata dal proxy
                    {
                        consecutiveFailures = 0;

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

                        if ((consecutiveFailures >= MAX_FAILURE_ATTEMPTS && AlarmManager.isRobotConnected) ||
                            (isReconnectionNeeded && AlarmManager.isRobotConnected))
                        {
                            // È la prima volta che rileviamo la disconnessione
                            log.Error("[Robot COM] Connessione al robot PERSA. Avvio tentativi di riconnessione...");
                            AlarmManager.isRobotConnected = false;
                            //RefresherTask.AddUpdate(PLCTagName.Emergency, 1, "INT16");

                            try
                            {
                                //Chiusura dei thread di libreria
                                Robot.CloseRPC();
                            }
                            catch (Exception ex)
                            {
                                log.Error("Errore durante chiusura RPC: " + ex.Message);
                            }

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
                                RobotConnected = false;
                            }

                            isReconnectionNeeded = false; // Reset del bypass
                            currentConnectionErrorTries = 0; // Reset del numero di tentativi con errore -2
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
                RunTimeError = true;
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
                await Task.Delay(2500);

                while (!token.IsCancellationRequested)
                {
                    CheckCommandStart();
                    CheckCommandGoToHome();
                    CheckVelCommand();
                    CheckCommandResetAlarms();

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
                RunTimeError = true;
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
                    CheckCommandStop();
                    CheckCommandRequestedStop();
                    await CheckPauseStatus();
                    CheckResumeStatus();

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
                RunTimeError = true;
                throw;
            }
            finally
            {

            }
        }

        #endregion

        #region Altri task

        /// <summary>
        /// Gestisce routine di pick e di place, parte solo se si ha sia il pick che il place. In position non controllati
        /// </summary>
        public static async Task PickAndPlaceTegliaIperal(CancellationToken token)
        {
            #region Variabili necessarie per funzionamento ciclo

            // #region contentente tutte le variabili utilizzate all'interno del ciclo e che permette di eseguire lo stesso

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

            // Piano del carrello selezionato
            int selectedFormat;

            // Apertura pinza
            byte isGripperOpen = 0;

            // Chiusura pinza
            byte isGripperClosed = 0;

            // Slitta avanti
            byte isGripperExtended = 0;

            // Slitta indietro
            byte isGripperRetracted = 0;

            // Slitta avanti/indietro
            byte isTrayPresent = 0;

            // Cod. errore movimento 1
            int err1;

            // Cod. errore movimento 2
            int err2;

            // Cod. errore movimento 3
            int err3;

            // Segnala quando il carrello è pieno
            bool carrelloTerminato = false;

            bool homeRequested = false;
            bool homeInProgress = false;
            #endregion

            #region Offset spostamenti

            // Questa #region contiene tutti gli offset applicati ai punti durante il ciclo.
            // Ogni sotto-#region è suddivisa per comportamento (pick, place, beor)

            #region Offset pick

            float offsetAvvicinamentoPick = 5; // Offset applicato al punto di avvicinamento pick
            float zOffsetAvvicinamentoPick = 10; // Offset su asse Z in cui mi abbasso leggermente prima di andare in pick
            float zOffsetPostPick = 20; // Offset applicato dopo chiusura pinza (mantengo questo offset anche durante movimento di allontanamento dal pick)
            float zOffsetAllontanamentoPick = 20; // Offset applicato per punto di allontanamento pick
            float offsetAllontanamentoPick = 750; // Offset  di allontamento post pick
            float zOffsetPick = 3; // Offset del punto di pick su asse z
            float offsetAllontamentoPreSlittaIndietro = 0; // Offset utilizzato per sapere quanto dopo aver eseguito il pick inviare il comando di slitta indietro
            float yOffsetPick = 10; // Offset del punto di pick su asse y
            float offsetPreAvvicinamentoPick = 130; // Offset di avvicinamento al punto di avvicinamento pick
            float rxRotationPick = 3; // Gradi di rotazione su asse rx dopo aver eseguito il pick

            #endregion

            #region Offset place

            float offsetAvvicinamentoPlace = offsetAllontanamentoPick; // Offset per eseguire punto di avvicinamento place
            float zOffsetAvvicinamentoPlace = 40; // Offset su asse Z in cui mi alzo leggermente prima di andare in place
            float zOffsetPostPlace = 5; // Offset su asse Z in cui mi abbasso leggermente dopo essere andato in place
            float zOffsetDistaccoPlace = 5; // Offse su asse Z in cui mi abbasso in linea retta
            float offsetAllontamentoPostPlace = 130; // Offset di allontanamento dal carrello dopo aver eseguito il place
            float offsetAllontamentoPreSlittaAvanti = 550; // Offset utilizzato per sapere quanto prima di raggiungere il place inviare il comando di slitta avanti
            float yoffsetAvvicinamentoPlace = 100; // Offset in y usato per sapere quanto prima di raggiungere il place di inviare il comando apri pinza
            float zOffsetPlace = 10; // Offset del punto di place su asse z
            float yOffsetPlace = 100; // Offset del punto di place su asse y
            float rxOffsetPrePlace = 2; // Offset di rotazione su asse x applicato al punto di avvicinamento place

            #endregion

            #region Offset beor

            float offsetAvvicinamentoBeor = 900; // Offset di avvicinamento al punto beor
            float offsetAllontamentoBeor = 700; // Offset di allontanamento dal punto beor

            #endregion

            #endregion

            #region Dichiarazione punti

            // In questa #region sono presenti tutti i punti necessari al funzionamento del ciclo.
            // Sono divisi in sotto-#region -> home, pick, place, beor.
            // Ognuna di queste (esclusa la home) comprende tutti i punti relativi alla gestione del proprio punto. (es.: avvicinamento, allontanamento, ecc.)

            #region Dichiarazione punto di home

            // home
            JointPos jHome = new JointPos(0, 0, 0, 0, 0, 0);
            var home = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");

            DescPose descPosHome = new DescPose(home.x, home.y, home.z, home.rx, home.ry, home.rz);

            #endregion

            #region Dichiarazione dei punti di pick

            #region Punto di pick

            // Punto finale di pick

            ApplicationPositions pick = new ApplicationPositions();
            JointPos jointPosPick = new JointPos();
            DescPose descPosPick = new DescPose();

            #endregion

            #region Punto di avvicinamento pick

            // Punto di avvicinamento prima di eseguire il pick

            JointPos jointPosApproachPick = new JointPos();
            DescPose descPosApproachPick = new DescPose();

            #endregion

            #region Punto pre avvicinamento pick

            // Punto di pre avvicinamento prima di eseguire il pick

            JointPos jointPosPreApproachPick = new JointPos();
            DescPose descPosPreApproachPick = new DescPose();

            #endregion

            #region Punto di post-pick

            // Punto post-pick in cui mi alzo leggermente sull'asse Z dopo aver chiuso la pinza

            JointPos jointPosPostPick = new JointPos();
            DescPose descPosPostPick = new DescPose();

            #endregion

            #region Punto di allontanamento pick

            // Punto di allontanamento pick dopo aver chiuso la pinza

            JointPos jointPosAllontanamentoPick = new JointPos();
            DescPose descPosAllontanamentoPick = new DescPose();

            #endregion

            #region Punto interemdio di allontanamento pick

            // Punto intermedio di allontanamento pick dopo aver chiuso la pinza

            JointPos jointPosIntAllontanamentoPick = new JointPos();
            DescPose descPosIntAllontanamentoPick = new DescPose();

            #endregion

            #endregion

            #region Dichiarazione dei punti place

            #region Punto di place

            // Punto finale di place

            ApplicationPositions place = new ApplicationPositions();
            JointPos jointPosPlace = new JointPos();
            DescPose descPosPlace = new DescPose();

            #endregion

            #region Punto di rotazione pre-place

            // Punto necessario alla rotazione del robot dalla posizione di pick per prepararsi ad andare in pre-place

            JointPos jointPosRotationPrePlace = new JointPos();
            DescPose descPosRotationPrePlace = new DescPose();

            #endregion

            #region Punto di avvicinamento place

            // Punto di avvicinamento prima di eseguire il place

            JointPos jointPosApproachPlace = new JointPos();
            DescPose descPosApproachPlace = new DescPose();

            #endregion

            #region Punto post place

            // Punto da eseguire dopo apertura pinza che mi permette di scendere leggermente sull'asse Z

            JointPos jointPosPostPlace = new JointPos();
            DescPose descPosPostPlace = new DescPose();

            #endregion

            #region Punto abbassamento post place

            // Punto da eseguire dopo post place, abbassamento in z prima di allontanamento place

            JointPos jointPosDistaccoTegliaPlace = new JointPos();
            DescPose descPosDistaccoTegliaPlace = new DescPose();

            #endregion

            #region Punto di allontanamento place

            // Punto di allontanamento da punto di place dopo che ho aperto le pinze e che mi sono abbassato sull'asse Z

            JointPos jointPosAllontanamentoPlace = new JointPos();
            DescPose descPosAllontanamentoPlace = new DescPose();

            #endregion

            #endregion

            #region Dichiarazione dei punto di Beor

            #region Punto Beor

            // Punto finale Beor

            JointPos jointPosBeor = new JointPos();
            var beor = ApplicationConfig.applicationsManager.GetPosition("pBeor", "RM");

            // Creazione oggetto descPose
            DescPose descPosBeor = new DescPose(beor.x, beor.y, beor.z, beor.rx, beor.ry, beor.rz);

            // Oggetto jointPos
            jointPosBeor = new JointPos(0, 0, 0, 0, 0, 0);

            #endregion

            #region Punto di rotazione da pick a beor

            // Punto che permettere la rotazione da pick in preparazione a punto Beor

            JointPos jointPosRotationPreBeor = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosRotationPreBeor = new DescPose(descPosHome.tran.x, descPosHome.tran.y, descPosHome.tran.z, beor.rx, beor.ry, beor.rz);

            #endregion

            #region Punto avvicinamento beor

            // Punto di avvicinamento beor

            // Oggetto jointPos
            JointPos jointPosApproachBeor = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosApproachBeor = new DescPose(beor.x - offsetAvvicinamentoBeor, beor.y, beor.z, beor.rx, beor.ry, beor.rz);

            #endregion

            #region Punto allontanamento beor

            // Punto di allontanamento da beor

            // Oggetto jointPos
            JointPos jointPosAllontanamentoBeor = new JointPos(0, 0, 0, 0, 0, 0);

            // Creazione oggetto descPose
            DescPose descPosAllontanamentoBeor = new DescPose(beor.x - offsetAllontamentoBeor, beor.y, beor.z, beor.rx, beor.ry, beor.rz);

            #endregion

            #endregion

            #endregion

            #region Parametri movimento

            // Questa #region è dedicata a tutti i parametri interni dei metodi di movimento del Robot

            DescPose offset = new DescPose(0, 0, 0, 0, 0, 0); // Nessun offset
            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)
            byte search = 0;

            // Parametri moveL
            int velAccParamMode = 0; // Velocity and acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s^2)
            int overSpeedStrategy = 0; // Overspeed handling strategy, 1-Standard; 2-Stop with error on overspeed; 3-Adaptive deceleration, default is 0
            int speedPercent = 0; // Allowed deceleration threshold percentage [0-100], default 10%

            float slowVel; // Velocità movimento parametrizzata
            float slowAcc; // Accelerazione movimento parametrizzata

            #endregion

            #region Ciclo

            try
            {
                if (home == null)
                    throw new DataErrorException("Il punto di home position non esiste");

                if (beor == null)
                    throw new DataErrorException("Il punto di beor non esiste");

                GetInverseKin(descPosHome, ref jHome, "Home");
                GetInverseKin(descPosBeor, ref jointPosBeor, "Beor");
                GetInverseKin(descPosRotationPreBeor, ref jointPosRotationPreBeor, "Rotazione pre Beor");
                GetInverseKin(descPosApproachBeor, ref jointPosApproachBeor, "Avvicinamento beor");
                GetInverseKin(descPosAllontanamentoBeor, ref jointPosAllontanamentoBeor, "Allontanamento beor");

                if (!frameManager.ChangeRobotFrame("frBeor"))
                    throw new RobotPropertiesChangeException("Cambio frame a frBeor ha generato un'eccezione");

                if (!toolManager.ChangeRobotTool("tPinza"))
                    throw new RobotPropertiesChangeException("Cambio tool a tPinza ha generato un'eccezione");

                // Fino a quando la condizione di stop routine non è true e non sono presenti allarmi bloccanti
                while (!stopCycleRoutine && !AlarmManager.blockingAlarm && !token.IsCancellationRequested)
                {
                    switch (step)
                    {
                        case 0:
                            #region Comunicazione avvio ciclo a PLC, check dei consensi e calcolo punto di pick e place
                            // In questo step scrivo al plc che il ciclo di main è stato avviato e passo subito allo step successivo
                            // Eseguo anche il calcolo dei punto di pick e place la prima volta, per poi non passare più da questo step.

                            // Aggiorno la variabile globale e statica che scrivo al PLC nel metodo SendUpdatesToPLC 
                            // per informare il plc che il ciclo main è in esecuzione
                            CycleRun_Main = 1;

                            // Get apertura pinza
                            GetDI(2, 1, ref isGripperOpen);
                            // Get slitta avanti
                            GetDI(4, 1, ref isGripperExtended);
                            // Get slitta indietro
                            GetDI(5, 1, ref isGripperRetracted);
                            // Get presenza teglia
                            GetDI(7, 1, ref isTrayPresent);

                            // Controllo che la pinza sia aperta e la slitta avanti oppure che la pinza sia aperta, la slitta indietro e la teglia assente
                            if ((isGripperOpen == 1 && isGripperExtended == 1) || (isGripperOpen == 1 && isGripperRetracted == 1 && isTrayPresent == 0))
                            {
                                // Apro la pinza
                                SetDO(0, 1, 0, 0);

                                // Slitta avanti
                                //robot.SetDO(1, 1, 0, 0);
                                RefresherTask.AddUpdate(PLCTagName.CMD_slittaAvanti, 1, "INT16");

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
                                        #region Calcolo dei punti di pick e di place

                                        #region Pick

                                        #region Punto di Pick

                                        // Get punto di pick da PLC
                                        selectedFormat = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_SelectedFormat));
                                        // selectedFormat = 1001;

                                        var pPick = ApplicationConfig.applicationsManager.GetPosition(selectedFormat.ToString(), "RM");

                                        if (pPick != null)
                                        {
                                            // Check validità del punto
                                            if (pPick.x != 0 && pPick.y != 0 && pPick.z != 0 && pPick.rx != 0 && pPick.ry != 0 && pPick.rz != 0) // Se il punto è valido
                                            {
                                                pick = pPick;
                                            }
                                            else
                                                throw new DataErrorException("Punto di pick non presente nel dizionario");
                                        }
                                        else
                                            throw new DataErrorException("Punto di pick non presente nel dizionario");

                                        // pick target
                                        jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
                                        descPosPick = new DescPose(
                                            pick.x,
                                            pick.y + yOffsetPick,
                                            pick.z + zOffsetPick,
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
                                            pick.y - offsetAvvicinamentoPick,
                                            pick.z - zOffsetAvvicinamentoPick + zOffsetPick,
                                            pick.rx,
                                            pick.ry,
                                            pick.rz
                                            );

                                        // Calcolo del jointPos
                                        GetInverseKin(descPosApproachPick, ref jointPosApproachPick, "Avvicinamento pick");

                                        #endregion

                                        #region Punto di pre avvicinamento Pick 

                                        // Oggetto jointPos
                                        jointPosPreApproachPick = new JointPos(0, 0, 0, 0, 0, 0);

                                        // Creazione oggetto descPose
                                        descPosPreApproachPick = new DescPose(
                                            pick.x,
                                            pick.y - offsetPreAvvicinamentoPick,
                                            pick.z - zOffsetAvvicinamentoPick + zOffsetPick,
                                            pick.rx,
                                            pick.ry,
                                            pick.rz
                                            );

                                        // Calcolo del jointPos
                                        GetInverseKin(descPosPreApproachPick, ref jointPosPreApproachPick, "Pre avvicinamento pick");

                                        #endregion

                                        #region Punto post Pick

                                        // Oggetto jointPos
                                        jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);

                                        // Creazione oggetto descPose
                                        descPosPostPick = new DescPose(
                                            pick.x,
                                            pick.y,
                                            pick.z + zOffsetPostPick + zOffsetPick,
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
                                            pick.y - offsetAllontanamentoPick,
                                            pick.z + zOffsetPostPick + zOffsetAllontanamentoPick + zOffsetPick,
                                            NormalizeAngle(pick.rx + rxRotationPick),
                                            pick.ry,
                                            pick.rz
                                            );

                                        // Calcolo del jointPos
                                        GetInverseKin(descPosAllontanamentoPick, ref jointPosAllontanamentoPick, "Allontanamento pick");

                                        #endregion

                                        #region Punto intermedio allontanamento post Pick

                                        // Oggetto jointPos
                                        jointPosIntAllontanamentoPick = new JointPos(0, 0, 0, 0, 0, 0);

                                        // Creazione oggetto descPose
                                        descPosIntAllontanamentoPick = new DescPose(
                                            descPosAllontanamentoPick.tran.x,
                                            pick.y - offsetAllontamentoPreSlittaIndietro,
                                            descPosAllontanamentoPick.tran.z + zOffsetPick,
                                            descPosAllontanamentoPick.rpy.rx,
                                            descPosAllontanamentoPick.rpy.ry,
                                            descPosAllontanamentoPick.rpy.rz
                                            );

                                        // Calcolo del jointPos
                                        GetInverseKin(descPosIntAllontanamentoPick, ref jointPosIntAllontanamentoPick, "Allontanamento intermedio pick");

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
                                            place.y - yOffsetPlace,
                                            place.z + zOffsetPlace,
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
                                            place.y - offsetAvvicinamentoPlace,
                                            place.z + zOffsetAvvicinamentoPlace,
                                            NormalizeAngle(place.rx + rxOffsetPrePlace),
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
                                            place.z + zOffsetAvvicinamentoPlace,
                                            place.rx,
                                            place.ry,
                                            place.rz
                                            );

                                        // Calcolo del jointPos
                                        GetInverseKin(descPosRotationPrePlace, ref jointPosRotationPrePlace, "Rotazione pre place");

                                        #endregion

                                        #region Punto post place

                                        // Oggetto jointPos
                                        jointPosPostPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                        // Creazione oggetto descPose
                                        descPosPostPlace = new DescPose(
                                           place.x,
                                           place.y,
                                           place.z - zOffsetPostPlace,
                                           place.rx,
                                           place.ry,
                                           place.rz
                                           );

                                        // Calcolo del jointPos
                                        GetInverseKin(descPosPostPlace, ref jointPosPostPlace, "Post place");

                                        #endregion

                                        #region Punto distacco teglia 

                                        // Oggetto jointPos
                                        jointPosDistaccoTegliaPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                        // Creazione oggetto descPose
                                        descPosDistaccoTegliaPlace = new DescPose(
                                           place.x,
                                           place.y,
                                           place.z - zOffsetPostPlace - zOffsetDistaccoPlace,
                                           place.rx,
                                           place.ry,
                                           place.rz
                                           );

                                        // Calcolo del jointPos
                                        GetInverseKin(descPosDistaccoTegliaPlace, ref jointPosDistaccoTegliaPlace, "Distacco teglia place");

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

                                        // Passaggio allo step 10
                                        step = 10;

                                    }
                                }
                            }

                            break;

                        #endregion

                        case 10:
                            #region Check richiesta routine e consensi

                            if (stopCycleRequested && !homeRequested) // Se è stata richiesto uno stop ciclo, vado allo step per mandare in home il robot
                            {
                                homeRequested = true;
                            }
                            else if (homeRequested && !homeInProgress)
                            {
                                step = 220;   // step dedicato alla home
                            }
                            else
                            {
                                // Get apertura pinza
                                GetDI(2, 1, ref isGripperOpen);
                                // Get slitta avanti
                                GetDI(4, 1, ref isGripperExtended);
                                // Get slitta indietro
                                GetDI(5, 1, ref isGripperRetracted);
                                // Get presenza teglia
                                GetDI(7, 1, ref isTrayPresent);

                                // Controllo che la pinza sia aperta e la slitta avanti oppure che la pinza sia aperta, la slitta indietro e la teglia assente
                                if ((isGripperOpen == 1 && isGripperExtended == 1) || (isGripperOpen == 1 && isGripperRetracted == 1 && isTrayPresent == 0))
                                {
                                    // Apro la pinza
                                    SetDO(0, 1, 0, 0);

                                    // Slitta avanti
                                    SetDO(1, 1, 0, 0);

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
                                            log.Info("STEP 10 - Check richiesta routine e consensi.");
                                            step = 40;
                                        }
                                    }
                                    else
                                    {

                                        step = 0;
                                    }
                                }
                            }
                            break;

                        #endregion

                        case 40:
                            #region Check consensi

                            // Get slitta avanti
                            GetDI(4, 1, ref isGripperExtended);

                            // Get apertura pinza
                            GetDI(2, 1, ref isGripperOpen);

                            // Get presenza teglia
                            GetDI(7, 1, ref isTrayPresent);

                            // Se slitta avanti, pinza aperta, teglia assente e robot in safe zone
                            if (isGripperExtended == 1 && isGripperOpen == 1 && isTrayPresent == 0 && isInSafeZone)
                            {
                                log.Info("STEP 40 - Slitta avanti, pinza aperta, teglia assente e robot in safe zone.");
                                step = 50;
                            }

                            break;

                        #endregion

                        case 50:
                            #region Movimento a punto di Pick

                            log.Info("STEP 50 - invio punto di pick : " + pick.name);

                            CycleRun_Pick = 1;
                            CycleRun_Place = 1;

                            inPosition = false; // Reset inPosition

                            #region Movimento a punto di pre avvicinamento Pick

                            slowVel = vel * 1f;
                            slowAcc = acc * 1f;

                            blendR = 20;
                            // Movimento a punto di avvicinamento pick teglia 1
                            err1 = MoveL(jointPosPreApproachPick, descPosPreApproachPick,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento a punto di avvicinamento Pick

                            slowVel = vel * 1f;
                            slowAcc = acc * 1f;

                            blendR = 20;
                            // Movimento a punto di avvicinamento pick teglia 1
                            err2 = MoveL(jointPosApproachPick, descPosApproachPick,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento a punto di pick

                            blendR = 20;

                            slowVel = vel * 1f;
                            slowAcc = acc * 1f;

                            // Movimento a pick teglia 1
                            err3 = MoveL(jointPosPick, descPosPick,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion
                          
                            // Se uno dei movimenti a generato un errore relativo alla pausa,
                            // riavvio lo step
                            if (err1 == 99 || err2 == 99 || err3 == 99)
                            {
                               await Task.Delay(500);
                            }
                            else
                            {
                                endingPoint = descPosPick; // Assegnazione endingPoint

                                step = 60; // Passaggio allo step successivo
                                stepPick = 10;
                            }

                            break;

                        #endregion

                        case 60:
                            #region Attesa inPosition punto di Pick e chiusura pinza
                            // In questo step attendo che il robot arrivi nella posizione di pick

                            if (inPosition && robotStatus == 1) // con robot fermo
                            {
                                log.Info("STEP 60 - Attesa inPosition punto di Pick e chiusura pinza");
                                // Chiusura pinza
                                SetDO(0, 0, 0, 0);
                                step = 80;
                            }

                            break;

                        #endregion
                          
                        case 70:      
                            #region Check slitta avanti e presenza teglia
                            /*
                            // Get slitta avanti
                            GetDI(4, 1, ref isGripperExtended);

                            // Get presenza teglia
                            GetDI(7, 1, ref isTrayPresent);

                            // Se slitta avanti e teglia presente chiudo la pinza
                            if (isGripperExtended == 1 && isTrayPresent == 1)
                            {
                                log.Info("STEP 70 - Invio comando chiusura pinza");
                                // Chiusura pinza
                                //SetDO(0, 0, 0, 0);
                                step = 80;
                            }

                            break;*/

                        #endregion
                            
                        case 80:
                            #region Check chiusura pinza

                            // Get chiusura pinza
                            GetDI(3, 1, ref isGripperClosed);

                            // Get apertura pinza
                            GetDI(2, 1, ref isGripperOpen);

                            // Se la pinza è chiusa e non è aperta
                            if (isGripperOpen == 0) // && isGripperClosed == 1 )
                            {
                                log.Info("STEP 80 - Pinza chiusa e non e' aperta");
                                step = 100;
                            }

                            break;

                        #endregion

                        case 90:
                            #region Check slitta indietro
                            /*
                            // Get slitta indietro
                            GetDI(5, 1, ref isGripperRetracted);

                            // Se la slitta è indietro
                            if (isGripperRetracted == 1)
                            {
                                log.Info("STEP 40 - Slitta indietro");
                                step = 100;
                            }

                            break;*/

                        #endregion
                            
                        case 100:
                            #region Movimento di allontanamento post chiusura pinza

                            log.Info("STEP 100 - Invio movimento di allontanamento post chiusura pinza");

                            inPosition = false;

                            #region Movimento post Pick
                            
                            blendR = 50;
                            // Movimento per uscire dal carrelo dopo pick 1
                            err1 = MoveL(jointPosPostPick, descPosPostPick,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);
                            
                            #endregion

                            #region Movimento post Pick con offset
                            /*
                            blendR = 50;
                            offset = new DescPose(0, 0, 0, 2, 0, 0);
                            // Movimento a punto di avvicinamento place teglia 2
                            err2 = MoveJ(jointPosPostPick, descPosPostPick,
                                tool, user, vel, acc, ovl, epos, blendT, 1, offset);
                            offset = new DescPose(0, 0, 0, 0, 0, 0);
                            */
                            #endregion

                            #region Movimento allontanamento pick

                            blendR = 50;

                            // Movimento per uscire dal carrelo dopo pick 1
                            err3 = MoveL(jointPosAllontanamentoPick, descPosAllontanamentoPick,
                                tool, user, vel, acc, ovl, blendR, epos, search, 0, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            // Se uno dei movimenti a generato un errore relativo alla pausa,
                            // riavvio lo step
                            if (err1 == 99 || err3 == 99)
                            {
                                await Task.Delay(500);
                            }
                            else
                            {
                                endingPoint = descPosAllontanamentoPick;
                                step = 110;
                            }

                            break;

                        #endregion

                        case 110:
                            #region Invio comando slitta indietro

                            if (TCPCurrentPosition.tran.y <= descPosPick.tran.y - offsetAllontamentoPreSlittaIndietro)
                            {
                                // Slitta indietro
                                RefresherTask.AddUpdate(PLCTagName.CMD_slittaAvanti, 0, "INT16");

                                log.Info("STEP 110 - Invio comando slitta indietro");

                                step = 120;
                            }


                            break;

                        #endregion

                        case 120:
                            #region Attesa inPosition punto di allontanamento pick                           

                            // Get apertura pinza
                            GetDI(2, 1, ref isGripperOpen);

                            // Get valore slitta indietro e presenza teglia e pinza non aperta
                            GetDI(5, 1, ref isGripperRetracted);

                            if (inPosition && isGripperRetracted == 1 && isGripperOpen == 0 ) // Se arrivato in posizione e la slitta è indietro
                            {
                                log.Info("STEP 120 - Robot arrivato in posizione di allontanamento pick");
                                step = 125;
                            }
                            
                            break;

                        #endregion

                        case 125:
                            #region Controllo presenza teglia

                            // Get presenza teglia
                            GetDI(7, 1, ref isTrayPresent);

                            if (isTrayPresent == 1)
                            {
                                step = 130;
                            }
                            else
                            {
                                TrayNotPresentError = true;
                            }
                            break;

                        #endregion

                        case 130:
                            #region Movimento a beor

                            log.Info("STEP 130 - Invio movimento a beor");

                            // Reset inPosition
                            inPosition = false;

                            #region Movimento a home

                            blendR = 50;
                            // Ritorno in posizione di home
                            //  err1 = robot.MoveL(jHome, descPosHome,
                            //     tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento avvicinamento beor

                            slowVel = vel * 1f;
                            slowAcc = acc * 1f;

                            blendR = 10;
                            // Movimento a punto di avvicinamento beor
                            err2 = MoveL(jointPosApproachBeor, descPosApproachBeor,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento a beor

                            slowVel = vel * 1f;
                            slowAcc = acc * 1f;


                            blendR = 50;
                            // Movimento a  beor
                            err3 = MoveL(jointPosBeor, descPosBeor,
                               tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            if (err2 == 99 || err3 == 99)
                            {
                                await Task.Delay(500);
                            }
                            else
                            {
                                endingPoint = descPosBeor;

                                step = 140;
                            }

                            break;

                        #endregion

                        case 140:
                            #region Check arrivo in Beor

                            if (inPosition && robotStatus == 1)
                            {
                                // await Task.Delay(2000);

                                log.Info("STEP 140 - Check arrivo in Beor");

                                step = 150;
                            }

                            break;

                        #endregion

                        case 150:
                            #region Ritorno in home e movimento in avvicinamento place

                            log.Info("STEP 150 - Invio ritorno in home e movimento in avvicinamento place");

                            // Reset inPosition
                            inPosition = false;

                            #region Movimento avvicinamento beor

                            slowVel = vel * 1f;
                            slowAcc = acc * 1f;


                            blendR = 50;
                            // Movimento a punto di avvicinamento beor
                            err1 = MoveL(jointPosApproachBeor, descPosApproachBeor,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento rotazione pre place
                            /*
                            blendR = 50;
                            // Movimento di rotazione pre place teglia 2
                            err2 = robot.MoveL(jointPosRotationPrePlace, descPosRotationPrePlace,
                                tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);
                            */
                            #endregion

                            #region Movimento a punto di avvicinamento place

                            // offset = new DescPose(0, 0, 0, 3, 0, 0);
                            // Movimento a punto di avvicinamento place teglia 2
                            if (vel > 70)
                            {
                                slowVel = vel * 0.7f;
                                slowAcc = acc * 0.7f;
                            }
                            else
                            {
                                slowVel = vel * 1f;
                                slowAcc = acc * 1f;
                            }

                            blendR = 10;
                            err3 = MoveL(jointPosApproachPlace, descPosApproachPlace,
                                   tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);
                            // offset = new DescPose(0, 0, 0, 0, 0, 0);

                            #endregion

                            if (err1 == 99 || err3 == 99)
                            {
                                await Task.Delay(500);
                            }
                            else
                            {
                                endingPoint = descPosApproachPlace;

                                step = 160;
                            }

                            break;

                        #endregion

                        case 160:
                            #region Movimento a punto di place

                           // if (inPosition)
                            {
                                log.Info("STEP 160 - Invio movimento a punto di place");

                                // Reset inPosition
                                inPosition = false;

                                #region Movimento a punto di  place

                                if (vel > 70)
                                {
                                    slowVel = vel * 0.7f;
                                    slowAcc = acc * 0.7f;
                                }
                                else
                                {
                                    slowVel = vel * 1f;
                                    slowAcc = acc * 1f;
                                }

                                blendR = 1;
                                err1 = MoveL(jointPosPlace, descPosPlace,
                                       tool, user, slowVel, slowAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                                #endregion

                                if (err1 == 99)
                                {
                                    await Task.Delay(500);
                                }
                                else
                                {
                                    endingPoint = descPosPlace;

                                    step = 170;
                                } 
                            }

                            break;

                        #endregion

                        case 170:
                            #region Movimento slitta in avanti

                            if (TCPCurrentPosition.tran.y >= descPosPlace.tran.y - offsetAllontamentoPreSlittaAvanti)
                            {
                                // Slitta avanti
                                RefresherTask.AddUpdate(PLCTagName.CMD_slittaAvanti, 1, "INT16");

                                log.Info("STEP 170 - Movimento slitta in avanti");

                                step = 180;
                            }

                            break;
                        #endregion

                        case 180:
                            #region Attesa inPosition punto di place e apertura pinza

                            // Get valore slitta avanti
                            GetDI(4,1,ref isGripperExtended);

                            if (inPosition && robotStatus == 1 && isGripperExtended == 1) // Se il Robot è arrivato in posizione di Place ed è fermo e la slitta è avanti
                            //if (TCPCurrentPosition.tran.y >= descPosPlace.tran.y - yoffsetAvvicinamentoPlace)
                            {
                                //await Task.Delay(200);
                                // Apro la pinza
                                SetDO(0, 1, 0, 0);

                                log.Info("STEP 180 - Attesa inPosition punto di place e apertura pinza");
                                //log.Info("STEP 180 -Attesa quota y e apertura pinza");

                                step = 190;
                            }

                            break;

                        #endregion

                        case 190:
                            #region Check apertura pinza

                            GetDI(2, 1, ref isGripperOpen);

                            // se la pinza è aperta
                            if (isGripperOpen == 1)
                            {
                                // await Task.Delay(100); // Ritardo per evitare che il robot riparta senza aver finito di chiudere la pinza

                                log.Info("STEP 190 - Check apertura pinza");

                                step = 200;
                            }

                            break;

                        #endregion

                        case 200:
                            #region Allontanamento place

                            log.Info("STEP 200 - Invio allontanamento place");

                            // Reset inPosition
                            inPosition = false;

                            #region Movimento post place

                            slowVel = vel * 0.5f;
                            slowAcc = acc * 0.5f;


                            blendR = 0; // era 50
                           
                            // Movimento a punto di allontanamento place teglia 2
                            err1 = MoveL(jointPosPostPlace, descPosPostPlace,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento distacco teglia

                            slowVel = vel * 1f;
                            slowAcc = acc * 0.75f;

                            blendR = 0;

                            // Movimento a punto di distacco place teglia 
                            err2 = MoveL(jointPosDistaccoTegliaPlace, descPosDistaccoTegliaPlace,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento allontanamento place

                            slowVel = vel * 1f;
                            slowAcc = acc * 1f;

                            blendR = 0; // era 50
                            // Movimento a punto di allontanamento place teglia 2
                            err3 = MoveL(jointPosAllontanamentoPlace, descPosAllontanamentoPlace,
                                tool, user, slowVel, slowAcc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            if (err1 == 99 || err2 == 99 || err3 == 99)
                            {
                                await Task.Delay(500);
                            }
                            else
                            {
                                endingPoint = descPosAllontanamentoPlace;

                                step = 210;
                            }

                            break;

                        #endregion

                        case 210:
                            #region Calcolo nuovi punti

                            if (inPosition && robotStatus == 1)
                            {
                                log.Info("STEP 210 - Calcolo nuovi punti");

                                if (carrelloTerminato)
                                {
                                    homeRequested = true;
                                    step = 220;
                                }
                                else
                                {

                                    #region Calcolo dei punti di pick e di place

                                    #region Pick

                                    #region Punto di Pick

                                    // Get punto di pick da PLC
                                    selectedFormat = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_SelectedFormat));
                                    // selectedFormat = 1001;

                                    var pPick = ApplicationConfig.applicationsManager.GetPosition(selectedFormat.ToString(), "RM");

                                    if (pPick != null)
                                    {
                                        // Check validità del punto
                                        if (pPick.x != 0 && pPick.y != 0 && pPick.z != 0 && pPick.rx != 0 && pPick.ry != 0 && pPick.rz != 0) // Se il punto è valido
                                        {
                                            pick = pPick;
                                        }
                                        else
                                            throw new DataErrorException("Punto di pick non presente nel dizionario");
                                    }
                                    else
                                        throw new DataErrorException("Punto di pick non presente nel dizionario");

                                    // pick target
                                    jointPosPick = new JointPos(0, 0, 0, 0, 0, 0);
                                    descPosPick = new DescPose(
                                        pick.x,
                                        pick.y + yOffsetPick,
                                        pick.z + zOffsetPick,
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
                                        pick.y - offsetAvvicinamentoPick,
                                        pick.z - zOffsetAvvicinamentoPick + zOffsetPick,
                                        pick.rx,
                                        pick.ry,
                                        pick.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosApproachPick, ref jointPosApproachPick, "Avvicinamento pick");

                                    #endregion

                                    #region Punto di pre avvicinamento Pick 

                                    // Oggetto jointPos
                                    jointPosPreApproachPick = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosPreApproachPick = new DescPose(
                                        pick.x,
                                        pick.y - offsetPreAvvicinamentoPick,
                                        pick.z - zOffsetAvvicinamentoPick + zOffsetPick,
                                        pick.rx,
                                        pick.ry,
                                        pick.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosPreApproachPick, ref jointPosPreApproachPick, "Pre avvicinamento pick");

                                    #endregion

                                    #region Punto post Pick

                                    // Oggetto jointPos
                                    jointPosPostPick = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosPostPick = new DescPose(
                                        pick.x,
                                        pick.y,
                                        pick.z + zOffsetPostPick + zOffsetPick,
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
                                        pick.y - offsetAllontanamentoPick,
                                        pick.z + zOffsetPostPick + zOffsetAllontanamentoPick + zOffsetPick,
                                        NormalizeAngle(pick.rx + rxRotationPick),
                                        pick.ry,
                                        pick.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosAllontanamentoPick, ref jointPosAllontanamentoPick, "Allontanamento pick");

                                    #endregion

                                    #region Punto intermedio allontanamento post Pick

                                    // Oggetto jointPos
                                    jointPosIntAllontanamentoPick = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosIntAllontanamentoPick = new DescPose(
                                        descPosAllontanamentoPick.tran.x,
                                        pick.y - offsetAllontamentoPreSlittaIndietro,
                                        descPosAllontanamentoPick.tran.z + zOffsetPick,
                                        descPosAllontanamentoPick.rpy.rx,
                                        descPosAllontanamentoPick.rpy.ry,
                                        descPosAllontanamentoPick.rpy.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosIntAllontanamentoPick, ref jointPosIntAllontanamentoPick, "Allontanamento intermedio pick");

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
                                        place.y - yOffsetPlace,
                                        place.z + zOffsetPlace,
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
                                        place.y - offsetAvvicinamentoPlace,
                                        place.z + zOffsetAvvicinamentoPlace,
                                        NormalizeAngle(place.rx + rxOffsetPrePlace),
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
                                        place.z + zOffsetAvvicinamentoPlace,
                                        place.rx,
                                        place.ry,
                                        place.rz
                                        );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosRotationPrePlace, ref jointPosRotationPrePlace, "Rotazione pre place");

                                    #endregion

                                    #region Punto post place

                                    // Oggetto jointPos
                                    jointPosPostPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosPostPlace = new DescPose(
                                       place.x,
                                       place.y,
                                       place.z - zOffsetPostPlace,
                                       place.rx,
                                       place.ry,
                                       place.rz
                                       );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosPostPlace, ref jointPosPostPlace, "Post place");

                                    #endregion

                                    #region Punto distacco teglia 

                                    // Oggetto jointPos
                                    jointPosDistaccoTegliaPlace = new JointPos(0, 0, 0, 0, 0, 0);

                                    // Creazione oggetto descPose
                                    descPosDistaccoTegliaPlace = new DescPose(
                                       place.x,
                                       place.y,
                                       place.z - zOffsetPostPlace - zOffsetDistaccoPlace,
                                       place.rx,
                                       place.ry,
                                       place.rz
                                       );

                                    // Calcolo del jointPos
                                    GetInverseKin(descPosDistaccoTegliaPlace, ref jointPosDistaccoTegliaPlace, "Distacco teglia place");

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

                                    if (selectedFormat == 1015 || selectedFormat == 2015)
                                        carrelloTerminato = true;

                                    step = 10;
                                }
                            }

                            break;

                        #endregion

                        case 220:
                            #region HomeRoutine

                            #region Movimento a punto avvicinamento home

                            homeInProgress = true;

                            DescPose descPoseApproachHome = new DescPose(
                                           TCPCurrentPosition.tran.x,
                                           descPosHome.tran.y,
                                           TCPCurrentPosition.tran.z,
                                           TCPCurrentPosition.rpy.rx,
                                           TCPCurrentPosition.rpy.ry,
                                           TCPCurrentPosition.rpy.rz);

                            JointPos JointPosApproachHome = new JointPos(0, 0, 0, 0, 0, 0);
                            GetInverseKin(descPoseApproachHome, ref JointPosApproachHome, "Approach home position");

                            int result = MoveL(JointPosApproachHome, descPoseApproachHome,
                                tool, user, vel, acc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            #region Movimento a punto di home

                            result = MoveL(jHome, descPosHome,
                                tool, user, vel, acc, ovl, blendR, epos, search, 1, offset, velAccParamMode, overSpeedStrategy, speedPercent);

                            #endregion

                            inPosition = false; // reset inPosition
                            endingPoint = descPosHome;

                            step = 230;

                            break;
                        #endregion

                        case 230:
                            #region Check arrivo in HomePosition

                            if (inPosition) // Se sono arrivato in posizione di home
                            {
                                homeRequested = false;
                                homeInProgress = false;
                                carrelloTerminato = false;

                                if (stopCycleRequested)
                                {
                                    step = 999;
                                }
                                else
                                {
                                    step = 0;
                                }
                            }

                            break;
                        #endregion

                        case 999:
                            #region Stop debug

                            break;

                            #endregion
                    }

                    await Task.Delay(40); // Delay routine
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException) 
            {
                // Azioni da eseguire quando il task viene fermato tramite cancellation token
                throw;
            }
            catch (RobotConnectionException ex)
            {
                log.Error($"[TASK] : {TaskPickAndPlaceTegliaIperal} - robot disconnesso: {ex}");
                RobotConnected = false;
            }
            catch (RobotPropertiesChangeException ex)
            {
                log.Error($"[TASK] : {TaskPickAndPlaceTegliaIperal} - eccezione durante il cambio delle proprietà del robot: {ex}");
                RobotPropertiesError = true;
            }
            catch (RobotKinException ex)
            {
                log.Error($"[TASK] : {TaskPickAndPlaceTegliaIperal} - eccezione durante il calcolo cinematica inversa/diretta: {ex}");
                RobotKinError = true;
            }
            catch (RobotMovementException ex)
            {
                log.Error($"[TASK] : {TaskPickAndPlaceTegliaIperal} - eccezione durante il movimento del robot: {ex}");
                RobotMovementError = true;
            }
            catch (RunTimeException ex)
            {
                log.Error($"[TASK] : {TaskPickAndPlaceTegliaIperal} - eccezione durante l'esecuzione del ciclo: {ex}");
                RunTimeError = true;
            }
            catch (DataErrorException ex)
            {
                log.Error($"[TASK] : {TaskPickAndPlaceTegliaIperal} - eccezione generata da dei dati mancanti o errati: {ex}");
                DataError = true;
            }
            catch (Exception ex)
            {
                // Azioni da eseguire quando il task va in una eccezione generica
                log.Error($"[TASK] : {TaskPickAndPlaceTegliaIperal} - generic exception: {ex}");
                RunTimeError = true;
                throw;
            }
            finally
            {
                // Azioni da eseguire al termine del task
            }

            #endregion
        }

        /// <summary>
        /// Routine di go to home position
        /// </summary>
        /// <returns></returns>
        public static async Task HomeRoutine(CancellationToken token)
        {
            #region Get da db dei punti utili alla routine

            // Get del punto di home
            var restPose = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose pHome = new DescPose(restPose.x, restPose.y, restPose.z, restPose.rx, restPose.ry, restPose.rz);

            // Get del punto di safeZone
            var safeZone = ApplicationConfig.applicationsManager.GetPosition("pSafeZone", "RM");
            DescPose pSafeZone = new DescPose(safeZone.x, safeZone.y, safeZone.z, safeZone.rx, safeZone.ry, safeZone.rz);

            // Get del punto di beor
            var beor = ApplicationConfig.applicationsManager.GetPosition("pBeor", "RM");
            DescPose pBeor = new DescPose(beor.x, beor.y, beor.z, beor.rx, beor.ry, beor.rz);

            #endregion

            #region offset

            int offsetBeor = 300; // Offset col quale verifico distanza da macchina beor prima di lanciare la routine di home

            #endregion

            bool robotDangerousPoseCarrello = false; // A true se il robot si trova in zona di ingombro carrello 1 o carrello 2
            bool robotDangerousPoseBeor = false; // A true se il robot si trova in zona di ingombro macchina beor

            // Se il robot si trova in zona di ingombro carrello 1 o carrello 2, lo segnalo con il relativo bit
            if (isInPositionCarrello1 || isInPositionCarrello2)
            {
                robotDangerousPoseCarrello = true;
            }
            else
            // Se il robot si trova in zona di ingombro macchina beor, lo segnalo con il relativo bit
            if (isInPositionBeor)
            {
                robotDangerousPoseBeor = true;
            }

            stopHomeRoutine = false; // Reset segnale di stop ciclo home
            stepHomeRoutine = 0; // Reset degli step della HomeRoutine

            int homeRoutineTaskRefreshPeriod = 100; // Tempo di refresh della HomeRoutine

            // Delay per stabilizzare il sistema
            await Task.Delay(1000);

            #region Check dei consensi per iniziare la routine

            // Get apertura pinza
            byte isGripperOpen = 0;
            GetDI(2, 1, ref isGripperOpen);
            // Get slitta avanti
            byte isGripperExtended = 0;
            GetDI(4, 1, ref isGripperExtended);
            // Get slitta indietro
            byte isGripperRetracted = 0;
            GetDI(5, 1, ref isGripperRetracted);
            // Get presenza teglia
            byte isTrayPresent = 0;
            GetDI(7, 1, ref isTrayPresent);

            #endregion

            try
            {
                // Controllo che la pinza sia aperta e la slitta avanti oppure che la pinza sia aperta, la slitta indietro e la teglia assente
                if ((isGripperOpen == 0 && isGripperExtended == 0) || (isGripperOpen == 0 && isGripperRetracted == 0 && isTrayPresent == 1))
                    throw new HomeRoutineException("Mancanza condizione di avvio routine");

                if (restPose == null)
                    throw new DataErrorException("Il punto di home position non esiste");

                if (beor == null)
                    throw new DataErrorException("Il punto di beor non esiste");

                if (safeZone == null)
                    throw new DataErrorException("Il punto di safeZone non esiste");

                if (!frameManager.ChangeRobotFrame("frBeor"))
                    throw new RobotPropertiesChangeException("Cambio frame a frBeor ha generato un'eccezione");

                if (!toolManager.ChangeRobotTool("tPinza"))
                    throw new RobotPropertiesChangeException("Cambio tool a tPinza ha generato un'eccezione");

                // Apro la pinza
                SetDO(0, 1, 0, 0);

                // Slitta avanti
                // SetDO(1, 1, 0, 0);
                // RefresherTask.AddUpdate(PLCTagName.CMD_slittaAvanti, 1, "INT16");

                while (!stopHomeRoutine && !token.IsCancellationRequested) // Fino a quando non termino la home routine
                {
                    switch (stepHomeRoutine)
                    {
                        case 0:
                            #region Comunicazione a PLC avvio della HomeRoutine e setting HomeRoutineSpeed

                            if (!isInSafeZone)
                            {
                                step = 999;
                                break;
                            }

                            CycleRun_Home = 1;

                            SetHomeRoutineSpeed();
                            await Task.Delay(1000);

                            stepHomeRoutine = 5;

                            break;

                        #endregion

                        case 5:
                            #region Check zone di ingmboro e movimento a punto di approach home

                            if (robotDangerousPoseCarrello)
                            {
                                DescPose pApproach = new DescPose(
                                    TCPCurrentPosition.tran.x,
                                    pHome.tran.y,
                                    TCPCurrentPosition.tran.z,
                                    TCPCurrentPosition.rpy.rx,
                                    TCPCurrentPosition.rpy.ry,
                                    TCPCurrentPosition.rpy.rz);

                                GoToApproachHomePosition(pApproach);
                                endingPoint = pApproach;
                                stepHomeRoutine = 6;
                            }
                            else
                                if (robotDangerousPoseBeor)
                            {
                                DescPose pApproach = new DescPose(
                                    pBeor.tran.x - offsetBeor,
                                    TCPCurrentPosition.tran.y,
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

                            //MoveRobotToSafePosition();
                            GoToHomePosition();
                            endingPoint = pHome;

                            stepHomeRoutine = 20;

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
                            #region Termine ciclo, comunicazione a PLC e reset robot speed

                            ResetHomeRoutineSpeed();

                            CycleRun_Home = 0;
                            stepHomeRoutine = 0;
                            stopHomeRoutine = true;

                            await Task.Delay(1000);

                            break;

                        #endregion

                        case 999:
                            #region Step di errore

                            // Home routine partita con il robot fuori dalla safe zone
                            // Rimango in attesa 

                            break;

                        #endregion
                    }

                    await Task.Delay(homeRoutineTaskRefreshPeriod); // Delay routine
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                // Azioni da eseguire quando il task viene fermato tramite cancellation token
                throw;
            }
            catch (RobotConnectionException ex)
            {
                log.Error($"[TASK] : {TaskHomeRoutine} - robot disconnesso: {ex}");
                RobotConnected = false;
            }
            catch (RobotPropertiesChangeException ex)
            {
                log.Error($"[TASK] : {TaskHomeRoutine} - eccezione durante il cambio delle proprietà del robot: {ex}");
                RobotPropertiesError = true;
            }
            catch (RobotKinException ex)
            {
                log.Error($"[TASK] : {TaskHomeRoutine} - eccezione durante il calcolo cinematica inversa/diretta: {ex}");
                RobotKinError = true;
            }
            catch (RobotMovementException ex)
            {
                log.Error($"[TASK] : {TaskHomeRoutine} - eccezione durante il movimento del robot: {ex}");
                RobotMovementError = true;
            }
            catch (RunTimeException ex)
            {
                log.Error($"[TASK] : {TaskHomeRoutine} - eccezione durante l'esecuzione del ciclo: {ex}");
                RunTimeError = true;
            }
            catch (DataErrorException ex)
            {
                log.Error($"[TASK] : {TaskHomeRoutine} - eccezione generata da dei dati mancanti o errati: {ex}");
                DataError = true;
            }
            catch (HomeRoutineException ex)
            {
                log.Error($"[TASK] : {TaskHomeRoutine} - mancanza di consensi o errori durante la home routine: {ex}");
                HomeRoutineError = true;
            }
            catch (Exception ex)
            {
                // Azioni da eseguire quando il task va in una eccezione generica
                log.Error($"[TASK] : {TaskHomeRoutine} - generic exception: {ex}");
                RunTimeError = true;
                throw;
            }
            finally
            {
                // Azioni da eseguire al termine del task
            }
        }

        #endregion

        #region Comandi interfaccia

        /// <summary>
        /// Gestione comando di stop derivante da plc
        /// </summary>
        private static void CheckCommandStop()
        {
            // Get valore variabile di stop ciclo robot
            int stopStatus = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_StopCicloAuto));

            if (stopStatus == 1 && previousStopCommandStatus != 1)
            {
                log.Warn("Richiesto comando STOP");

                stopCycleRoutine = true; // Alzo segnale di stop ciclo main
                CycleRun_Main = 0; // Segnalo interruzione ciclo main
                step = 0; // reset step ciclo main

                stopHomeRoutine = true; // Alzo segnale di stop ciclo home
                CycleRun_Home = 0; // Segnalo interruzione ciclo home
                stepHomeRoutine = 0; // reset step ciclo home

                stopPickRoutine = true; // Alzo segnale di stop ciclo pick
                CycleRun_Pick = 0; // Segnalo interruzione ciclo pick
                stepPick = 0; // reset step ciclo pick

                stopPlaceRoutine = true; // Alzo segnale di stop ciclo place
                CycleRun_Place = 0; // Segnalo interruzione ciclo place
                stepPlace = 0; // reset step ciclo place

                //robot.PauseMotion(); 
                //PauseMotion(); // Invio comando di pausa al robot
                //await Task.Delay(200); // Leggero ritardo per stabilizzare il robot
                //robot.StopMotion(); 
                StopMotion(); // Stop Robot con conseguente cancellazione di coda di punti

                previousStopCommandStatus = 1;

                log.Warn("Comando STOP eseguito");
            }
            else if (stopStatus == 0)
            {
                previousStopCommandStatus = 0;
            }
        }

        /// <summary>
        /// Gestione comando di richiesta stop derivante da plc
        /// </summary>
        private static void CheckCommandRequestedStop()
        {
            // Get valore variabile di stop ciclo robot
            int requestedStopStatus = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Stop_Cycle_Requested));

            if (requestedStopStatus == 1 && previousRequestedStopCommandStatus != 1)
            {
                log.Warn("Richiesto comando di richiesta STOP");

                stopCycleRequested = true; // Alzo segnale di richiesta stop ciclo main
                previousRequestedStopCommandStatus = 1;

                log.Warn("Comando di richiesta STOP eseguito");
            }
            else if (requestedStopStatus == 0)
            {
                previousRequestedStopCommandStatus = 0;
            }
        }

        /// <summary>
        /// Esegue check su cambio velocità derivante dal plc
        /// </summary>
        private static void CheckVelCommand()
        {
            // Get valore variabile di stop ciclo robot
            int velocity = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_OverrideAuto));

            // Check su cambio di stato
            if (velocity != previousVel && velocity >= 1 && velocity <= 100)
            {
                log.Info("[Override speed] Richiesto comando cambio override speed da: " + previousVel + " a : " + velocity);

                RobotDAO.SetRobotVelocity(ConnectionString, Convert.ToInt16(velocity));
                RobotDAO.SetRobotAcceleration(ConnectionString, Convert.ToInt16(velocity));
                //ovl = velocity;

                //Invoco metodo per cambiare etichetta velocità in homePage
                RobotVelocityChanged?.Invoke(velocity, EventArgs.Empty);

                // Aggiornamento della velocità precendete
                previousVel = velocity;

                log.Info("[Override speed] Comando cambio override speed completato");
            }
        }

        /// <summary>
        /// Check su comando di start derivante da plc
        /// </summary>
        private static void CheckCommandStart()
        {
            // Get valore variabile di avvio ciclo robot
            int startStatus = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_StartCicloAuto));

            // Get valore variabile di stop ciclo robot
            int stopStatus = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_StopCicloAuto));

            // Check su cambio di stato
            if (Convert.ToBoolean(startStatus) != previousStartCommandStatus)
            {
                log.Warn("Richiesto comando START");

                if (startStatus == 1 && stopStatus != 1) // Start
                {
                    // Controllo che il robot sia in automatico
                    if (!isAutomaticMode)
                    {
                        log.Warn("Tentativo di avvio ciclo con robot non in modalità automatica");
                        return;
                    }
                    // Setto della velocità del Robot dalle sue proprietà memorizzate sul database
                    if (robotProperties.Speed > 1)
                    {
                        int speed = robotProperties.Speed;
                        //robot.SetSpeed(speed);
                        SetRobotSpeed(speed);
                        log.Info($"Velocità Robot: {speed}");
                    }
                    // Se il Robot non è in movimento 
                    if (!AlarmManager.isRobotMoving)
                    {
                        taskManager.AddAndStartTask(TaskPickAndPlaceTegliaIperal, PickAndPlaceTegliaIperal, TaskType.LongRunning, false);
                        EnableButtonCycleEvent?.Invoke(0, EventArgs.Empty);
                    }
                    else // Se il Robot è in movimento
                    {
                        log.Error("Impossibile inviare nuovi punti al Robot. Robot in movimento");
                    }
                }
                else // Stop
                {
                    // stopCycleRequested = true;  // Valutare se alzare un bit o fermare subito il robot
                    EnableButtonCycleEvent?.Invoke(1, EventArgs.Empty);
                }

                previousStartCommandStatus = startStatus > 0;
            }
            else if (!Convert.ToBoolean(startStatus))
            {
                previousStartCommandStatus = false;
            }
        }

        /// <summary>
        /// Check su comando di stop derivante da plc
        /// </summary>
        private static void CheckCommandGoToHome()
        {
            int homeStatus = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_GoHome));

            if (homeStatus == 1 && !previousHomeCommandStatus) // Go to home
            {
                log.Warn("[HOME] Richiesto comando GO TO HOME");
                previousHomeCommandStatus = true;
                taskManager.AddAndStartTask(TaskHomeRoutine, HomeRoutine, TaskType.Default, false);
            }
            else if (homeStatus == 0)
            {
                previousHomeCommandStatus = false; // reset status
            }
        }

        /// <summary>
        /// Check su uscita barriere
        /// </summary>
        private static void CheckResumeStatus()
        {
            // Get valore richiesta di pausa
            int barrierStatus = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.MovePause));

            // Get valore richiesta di resume
            int resumeMov = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_Resume));

            // Controllo se c'è stato un cambio di valore della richiesta di resume
            if (resumeMov == 1 && previousBarrierResumeStatus == 0 && barrierStatus == 0)
            {
                log.Warn("Richiesto comando RESUME");

                // Ripresa
                ResumeMotion();

                log.Warn("Comando RESUME completato");

                previousBarrierResumeStatus = 1;
            }
            else if (resumeMov == 0)
            {
                previousBarrierResumeStatus = 0;
            }
        }

        /// <summary>
        /// Check su accesso barriere
        /// </summary>
        private static async Task CheckPauseStatus()
        {
            int barrierStatus = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.MovePause));

            if (barrierStatus == 1 && previousBarrierPauseStatus == 0)
            {
                log.Warn("Richiesto comando PAUSA");

                // Richiesta di pausa
                PauseMotion();

                // 🔁 Aspetta che lo stato robot diventi 3 (pausa)
                const int maxAttempts = 3;
                int attempt = 0;

                do
                {
                    if (robotStatus == 3 || robotStatus == 1)
                    {
                        // robotMove_inPause = 1;
                        break;
                    }
                    await Task.Delay(100); // Attendi un po' prima di riprovare
                    attempt++;

                } while (attempt < maxAttempts);

                if (robotStatus != 3 && robotStatus != 1)
                {
                    log.Error("ERRORE: Il robot non si è messo in pausa correttamente.");
                }
                log.Warn("Comando PAUSA completato");

                previousBarrierPauseStatus = 1;
            }
            else if (barrierStatus == 0)
            {
                previousBarrierPauseStatus = 0;
            }
        }

        /// <summary>
        /// Esegue check apertura/chiusura pinza
        /// </summary>
        /// <returns></returns>
        private static void CheckGripperStatus()
        {

            // Get input digitale (pinza)
            byte gripperStatus = 0;
            GetDI(0, 1, ref gripperStatus);

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
        /// Check su comando di reset allarmi derivante da plc
        /// </summary>
        private static void CheckCommandResetAlarms()
        {
            int resetAlarmsCommand = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.CMD_ResetAlarms));

            if (resetAlarmsCommand > 0 && resetAlarmsCommand != previousAlarmResetRequested)
            {
                log.Warn("Richiesto comando RESET allarmi");
                try
                {
                    // Reset allarme
                    //RMLib_AlarmsCleared(null, EventArgs.Empty);
                    formAlarmPage.ClearActiveAlarms();
                    // Reset valore
                    RefresherTask.AddUpdate(PLCTagName.CMD_ResetAlarms, 0, "INT16");

                    //Reset allarmi robot
                    RobotGeneralError = false;
                    RobotError = false;
                    RobotKinError = false;
                    RobotMovementError = false;
                    RobotConnected = true;
                    RobotPropertiesError = false;

                    //Reset allarmi applicazione
                    RunTimeError = false; //TODO: forse non bisogna resettare l'allarme ma riavviare i task
                    DataError = false;
                    HomeRoutineError = false;
                    TrayNotPresentError = false;
                    GripperNotClosedError = false;
                    SlideNotOutError = false;
                    SlideNotInError = false;

                    log.Warn("Comando RESET completato");
                }
                catch (Exception)
                {
                    log.Error("Eccezione generata durante reset allarmi");
                }
                previousAlarmResetRequested = resetAlarmsCommand;
            }
            else if (resetAlarmsCommand == 0)
            {
                previousAlarmResetRequested = 0;
            }
        }

        #endregion

        #region Metodi interni

        private static void AbortTasks()
        {

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
                Robot = new Robot();

                // 2. Esegui di nuovo la procedura di connessione RPC e avvio dei thread di libreria
                int rpcResult = Robot.RPC(RobotIpAddress);

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
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_Cycle_Home, stepHomeRoutine, "INT16"); // Scrittura fase ciclo home a PLC
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Home, CycleRun_Home, "INT16"); // Scrittura valore avvio/stop ciclo home
            RefresherTask.AddUpdate(PLCTagName.CycleRun_Main, CycleRun_Main, "INT16"); // Scrittura valore avvio/stop ciclo main
            RefresherTask.AddUpdate(PLCTagName.ACT_N_Tool, currentTool, "INT16"); // Scrittura stato del robot
            RefresherTask.AddUpdate(PLCTagName.ACT_N_Frame, currentUser, "INT16"); // Scrittura stato del robot
            RefresherTask.AddUpdate(PLCTagName.ACT_CollisionLevel, currentCollisionLevel, "INT16"); // Scrittura stato del robot
            // RefresherTask.AddUpdate(PLCTagName.Move_InPause, robotMove_inPause, "INT16"); // Scrittura feedback pausa del robot
        }

        /// <summary>
        /// Esegue scrittura delle variabili ad alta priorità sul plc
        /// </summary>
        private static void SendHighPriorityUpdatesToPLC()
        {
            //RefresherTask.AddUpdate(PLCTagName.ApplicationComRobot_active, Convert.ToInt16(AlarmManager.isRobotConnected), "INT16"); // Scrittura comunicazione con robot attiva
            RefresherTask.AddUpdate(PLCTagName.Robot_error_alarms, BuildRobotAlarms(), "INT16"); // Scrittura stato errori del robot
            RefresherTask.AddUpdate(PLCTagName.Application_errror_alarms, BuildApplicationAlarms(), "INT16"); // Scrittura stato errori dell'applicazione
            RefresherTask.AddUpdate(PLCTagName.Robot_enable, robotEnableStatus, "INT16"); // Scrittura stato enable del robot
            RefresherTask.AddUpdate(PLCTagName.Robot_status, robotStatus, "INT16"); // Scrittura stato del robot
            RefresherTask.AddUpdate(PLCTagName.ACT_Step_MainCycle, step, "INT16"); // Scrittura fase ciclo main a PLC
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
            GetActualTcpPose(flag, ref pos);

            RoundPositionDecimals(ref pos, 3);

            return pos;
        }

        /// <summary>
        /// Imposta la velocità predefinita per eseguire la home routine
        /// </summary>
        public static void SetHomeRoutineSpeed()
        {
            SetRobotSpeed(homeRoutineSpeed);
        }

        /// <summary>
        /// Resetta la velocità utilizzata per la home routine
        /// </summary>
        public static void ResetHomeRoutineSpeed()
        {
            SetRobotSpeed(robotProperties.Speed);
        }

        /// <summary>
        /// Esegue check su Robot enable
        /// </summary>
        private static async Task CheckIsRobotEnable()
        {
            // Controllo se il robot è abilitato tramite PLC
            isEnabledNow = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.Enable));

            if (isEnabledNow && currentRobotEnableStatus != 1)
            {
                // Abilitazione del robot
                log.Warn("[ENABLE] Richiesta abilitazione robot");
                EnableRobot(1);
                //AlarmManager.blockingAlarm = false;
                
            }
            else if (!isEnabledNow && currentRobotEnableStatus != 0)
            {
                // Disabilitazione del robot
                log.Warn("[ENABLE] Richiesta disabilitazione robot");
                //robot.StopMotion(); // Cancellazione della coda di punti
                // AlarmManager.blockingAlarm = true;
                JogMovement.StopJogRobotTask(); // Ferma il task di Jog
                await Task.Delay(10);
                EnableRobot(0);
            }

            if (isEnabledNow)
                robotEnableStatus = 1;
            else
                robotEnableStatus = 0;
        }

        /// <summary>
        /// Esegue check su modalità Robot
        /// </summary>
        /// <summary>
        /// Esegue check su modalità Robot
        /// </summary>
        private static async Task CheckRobotMode()
        {
            // Ottieni la modalità operativa dal PLC
            mode = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Operating_Mode));

            // CASO A: Il PLC vuole la modalità AUTOMATICA
            if (mode == 1) // 1 = AUTO secondo la tua logica PLC
            {
                // Se il robot NON è GIA' in automatico...
                if (currentRobotMode != 0) // 0 = AUTOMATICO secondo la libreria robot
                {
                    await Task.Delay(1000);
                    log.Warn("[Mode] Cambio modalita in AUTO");
                    isAutomaticMode = true;
                    prevIsOff = false;
                    SetRobotMode(0); // Imposta il robot in modalità automatica
                    JogMovement.StopJogRobotTask(); // Ferma il thread di movimento manuale
                    TriggerRobotModeChangedEvent(1);  // Evento: modalità automatica
                }
            }
            // CASO B: Il PLC vuole la modalità MANUALE
            else if (mode == 2) // 2 = MANUALE secondo la tua logica PLC
            {
                // Se il robot NON è GIA' in manuale...
                if (currentRobotMode != 1) // 1 = MANUALE secondo la libreria robot
                {
                    await Task.Delay(1000);
                    log.Warn("[Mode] Cambio modalita in MANUAL");
                    isAutomaticMode = false;
                    prevIsOff = false;
                    SetRobotMode(1); // Imposta il robot in modalità manuale
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
                    //await Task.Delay(100);
                    log.Warn("[Mode] Cambio modalita in OFF");
                    isAutomaticMode = false;
                    prevIsOff = true;
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

            int err = GetRobotRealTimeState(ref robot_state_pkg);
            if (err == 0)
            {
                robotStatus = robot_state_pkg.robot_state;
                currentRobotMode = robot_state_pkg.robot_mode;
                currentRobotEnableStatus = robot_state_pkg.rbtEnableState;

                currentConnectionErrorTries = 0;
            }
            else if (err == -2)
            {
                if (currentConnectionErrorTries < connectionErrorMaxTries * 2)
                    currentConnectionErrorTries++;
            }
        }

        /// <summary>
        /// Creazione di un allarme quando il robot si ferma
        /// </summary>
        /// <param name="id">ID allarme</param>
        /// <param name="description">Descrizione allarme</param>
        /// <param name="timestamp">Timestamp allarme</param>
        /// <param name="device">Device da cui deriva l'allarme</param>
        /// <param name="state">ON-OFF</param>
        private static void CreateRobotAlarm(string id, string description, string timestamp, string device, string state)
        {
            // Solleva l'evento quando il robot si ferma
            OnRobotAlarm(new RobotAlarmsEventArgs(id, description, timestamp, device, state));
        }

        /// <summary>
        /// Metodo che aggiunge alla lista degli allarmi l'allarme
        /// </summary>
        /// <param name="e"></param>
        private static void OnRobotAlarm(RobotAlarmsEventArgs e)
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
        private static void CheckIsRobotMoving()
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

        #region Cicli di movimento

        /// <summary>
        /// Metodo che porta il Robot in HomePosition
        /// </summary>
        private static void GoToHomePosition()
        {
            var restPose = ApplicationConfig.applicationsManager.GetPosition("pHome", "RM");
            DescPose pHome = new DescPose(restPose.x, restPose.y, restPose.z, restPose.rx, restPose.ry, restPose.rz);

            //int result = robot.MoveCart(pHome, tool, user, homeRoutineVel, homeRoutineAcc, ovl, blendT, config);

            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0); // Nessun asse esterno
            byte offsetFlag = 0; // Flag per offset (0 = disabilitato)
            // Parametri moveL
            int velAccParamMode = 0;
            int overSpeedStrategy = 0;
            int speedPercent = 0;
            byte search = 0;

            JointPos jointTarget = new JointPos(0, 0, 0, 0, 0, 0);
            GetInverseKin(pHome, ref jointTarget, "Go To Home position");

            int result = MoveL(jointTarget, pHome,
                tool, user, homeRoutineVel, homeRoutineAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

            if (result != 0)
                throw new RobotMovementException("Err code: " + result);
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
            GetInverseKin(target, ref jointTarget, "Go to approach home position");

            int result = MoveL(jointTarget, target,
                tool, user, homeRoutineVel, homeRoutineAcc, ovl, blendR, epos, search, offsetFlag, offset, velAccParamMode, overSpeedStrategy, speedPercent);

            if (result != 0)
                throw new RobotMovementException("Err code: " + result);
        }

        #endregion

        #endregion

        #region Metodi wrapper robot

        #region Safety

        /// <summary>
        /// Metodo che mette in pausa il Robot
        /// </summary>
        public static int PauseMotion()
        {
            int err = Robot.PauseMotion();
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
            int err = Robot.ResumeMotion();
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
            int err = Robot.StopMotion();
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

        #endregion

        #region Controlli

        /// <summary>
        /// Abilita o disabilita il Robot
        /// </summary>
        /// <param name="enableFlag"></param>
        public static int EnableRobot(byte enableFlag)
        {
            int err = Robot.RobotEnable(enableFlag);
            if (err != 0)
            {
                if (enableFlag == 1)
                    log.Error("enable robot err: " + err);
                else
                    log.Error("disable robot err: " + err);
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

            int err = Robot.Mode(mode);
            isAutomaticMode = mode == 0;

            if (err != 0)
            {
                log.Error("Errore durante cambio modalita robot a " + mode + " : Codice errore " + err);
            }
            return err;
        }

        /// <summary>
        /// Metodo per reset errori Robot
        /// </summary>
        public static int ClearRobotAlarm()
        {
            int err = Robot.ResetAllError();
            if (err != 0)
            {
                log.Error("reset robot alarms err: " + err);
            }
            return err;
        }

        /// <summary>
        /// TODO
        /// </summary>
        /// <param name="state"></param>
        /// <returns></returns>
        public static int GetRobotMotionDone(ref byte state)
        {
            int err = Robot.GetRobotMotionDone(ref state);
            if(err != 0)
            {
                log.Error("Errore durante get robot motion done: " + err);
            }
            return err;
        }

        /// <summary>
        /// Get delle coordinate del tool nello spazio del frame
        /// </summary>
        /// <param name="flag"></param>
        /// <param name="TcpCurrentPos"></param>
        /// <returns></returns>
        public static int GetActualTcpPose(byte flag, ref DescPose TcpCurrentPos)
        {
            int err = Robot.GetActualTCPPose(flag, ref TcpCurrentPos);
            if(err != 0)
            {
                log.Error("Errore durante get posizione attuale delle coordinate del tool: " + err);
            }
            return err;
        }

        /// <summary>
        /// Get degli errori del robot sotto forma di main code + sub code
        /// </summary>
        /// <param name="mainCode"></param>
        /// <param name="subCode"></param>
        /// <returns></returns>
        public static int GetRobotErrorCode(ref int mainCode, ref int subCode)
        {
            int err = Robot.GetRobotErrorCode(ref mainCode, ref subCode);
            if(err != 0)
            {
                log.Error("Errore durante il get degli errori del robot: " + err);
            }
            return err;
        }

        #endregion

        #region Proprietà

        /// <summary>
        /// Imposta la velocità di movimento del robot in percentuale
        /// </summary>
        /// <param name="speedPerc"></param>
        /// <returns></returns>
        public static bool SetRobotSpeed(int speedPerc)
        {
            int errSpeed = Robot.SetSpeed(speedPerc);
            if (errSpeed != 0)
            {
                log.Error("Errore durante update robot speed : " + errSpeed);
                GenerateAlarm(0, 4);
                RobotPropertiesError = true;
                return false;
            }
            log.Info("[Speed] velocita robot impostata a : " + speedPerc);
            return true;
        }

        /// <summary>
        /// Imposta il payload
        /// </summary>
        /// <param name="payload">Peso in kg</param>
        /// <param name="loadNum">Load number</param>
        /// <returns></returns>
        public static bool SetRobotPayload(float payload, int loadNum = 0)
        {
            int errPayload = Robot.SetLoadWeight(loadNum, payload);
            if (errPayload != 0)
            {
                log.Error("Errore durante update robot payload : " + errPayload);
                GenerateAlarm(0, 4);
                RobotPropertiesError = true;
                return false;
            }
            log.Info("[Payload] Peso robot impostato a : " + payload);
            return true;
        }

        /// <summary>
        /// Cambio frame al robot
        /// NB: il cambio funziona solo se il parametro coord contiene esattamente le stesse coordinate che possiede il frame 
        /// nel controllore.
        /// </summary>
        /// <param name="id">id numerico del frame [0-14]</param>
        /// <param name="coord">struttura contenente le coordinate per il calcolo del frame</param>
        /// <param name="refFrame">Reference coordinate system</param>
        /// <returns></returns>
        public static int SetWObjCoord(int id, DescPose coord, int refFrame)
        {
            int err = Robot.SetWObjCoord(id, coord, refFrame);
            if (err != 0)
            {
                log.Error("Errore durante cambio frame a " + id + " : Codice errore " + err);
                RobotPropertiesError = true;
            }
            return err;
        }

        /// <summary>
        /// Cambio tool al robot
        /// NB: il cambio funziona solo se il parametro coord contiene esattamente le stesse coordinate che possiede il tool 
        /// nel controllore.
        /// </summary>
        /// <param name="id">id numerico del tool [0-14]</param>
        /// <param name="coord">coordinate del tool (a partire dalla flangia)</param>
        /// <param name="type">0 : coordinate system, 1: sensor coordinate system</param>
        /// <param name="install">posizione di installazione: 0: robot end, 1: external to robot</param>
        /// <param name="toolId">tool id</param>
        /// <param name="loadNum">load number</param>
        /// <returns></returns>
        public static int SetToolCoord(int id, DescPose coord, int type, int install, int toolId = 0, int loadNum = 0)
        {
            int err = Robot.SetToolCoord(id, coord, type, install, toolId, loadNum);
            if (err != 0)
            {
                log.Error("Errore durante cambio tool a " + id + " : Codice errore " + err);
                RobotPropertiesError = true;
            }
            return err;
        }

        /// <summary>
        /// Imposta il livello di collisioni
        /// </summary>
        /// <param name="mode">0-level, 1-percentage</param>
        /// <param name="levels">level Collision threshold. Level range: [1-10 corresponds to levels 1-10, 100-disabled]. Percentage range: [0~10 corresponds to 0% - 100%]</param>
        /// <param name="config">config 0-Do not update config file, 1-Update config file</param>
        /// <returns></returns>
        public static int SetAntiCollision(int mode, double[] levels, int config)
        {
            int err = Robot.SetAnticollision(mode, levels, config);
            if (err != 0)
            {
                log.Error("Errore durante calcolo cinematica inversa : " + err);
                RobotPropertiesError = true;
            }

            return err;
        }

        /// <summary>
        /// Ottiene il frame corrente sotto forma di id
        /// </summary>
        /// <param name="flag"></param>
        /// <param name="checkNewFrame">id del frame corrente</param>
        /// <returns></returns>
        public static int GetActualWobjCoord(ref int checkNewFrame, byte flag = 0)
        {
            int err = Robot.GetActualWObjNum(flag, ref checkNewFrame);
            if (err != 0)
            {
                log.Error("Errore durante get frame corrente : " + err);
            }

            return err;
        }

        /// <summary>
        /// Ottiene il tool corrente sotto forma di id
        /// </summary>
        /// <param name="checkNewTool"> id del tool corrente</param>
        /// <param name="flag"></param>
        /// <returns></returns>
        public static int GetActualTCPNum(ref int checkNewTool, byte flag = 0)
        {
            int err = Robot.GetActualTCPNum(flag, ref checkNewTool);
            if (err != 0)
            {
                log.Error("Errore durante get tool corrente : " + err);
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
            int err = Robot.GetRobotRealTimeState(ref robot_state_pkg);

            if (err != 0)
            {
                log.Error("Errore durante get status robot : " + err);
            }

            return err;
        }

        #endregion

        #region Movimento

        /// <summary>
        /// Invia comando al robot per calcolare la cinematica inversa e ottenere la posizione dei giunti a partire dalle coordinate
        /// </summary>
        /// <param name="pose">Cartesian pose</param>
        /// <param name="jPos">Joint position</param>
        /// <param name="type">0-Absolute pose (base frame), 1-Incremental pose (base frame), 2-Incremental pose (tool frame)</param>
        /// <param name="config">Joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve according to specific joint space configuration
        /// 0	Spalla DX + Gomito ALTO + Polso NoFlip	
        /// 1	Spalla SX + Gomito ALTO + Polso NoFlip	
        /// 2	Spalla DX + Gomito BASSO + Polso NoFlip	
        /// 3	Spalla SX + Gomito BASSO + Polso NoFlip	
        /// 4	Spalla DX + Gomito ALTO + Polso Flip	
        /// 5	Spalla SX + Gomito ALTO + Polso Flip	
        /// 6	Spalla DX + Gomito BASSO + Polso Flip	
        /// 7	Spalla SX + Gomito BASSO + Polso Flip</param>
        public static int GetInverseKin(DescPose pose, ref JointPos jPos, int type = 0, int config = -1)
        {
            int err = Robot.GetInverseKin(type, pose, config, ref jPos);

            if (err != 0)
            {
                log.Error("Errore durante calcolo cinematica inversa : " + err);
                RobotKinError = true;
            }

            return err;
        }

        /// <summary>
        /// Invia comando al robot per calcolare la cinematica inversa e ottenere la posizione dei giunti a partire dalle coordinate
        /// Genera una eccezione se viene generato un errore dal controllore
        /// </summary>
        /// <param name="pose">Cartesian pose</param>
        /// <param name="jPos">Joint position</param>
        /// <param name="pointName">Nome del punto da scrivere nel log</param>
        /// <param name="type">0-Absolute pose (base frame), 1-Incremental pose (base frame), 2-Incremental pose (tool frame)</param>
        /// <param name="config">Joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve according to specific joint space configuration
        /// 0	Spalla DX + Gomito ALTO + Polso NoFlip	
        /// 1	Spalla SX + Gomito ALTO + Polso NoFlip	
        /// 2	Spalla DX + Gomito BASSO + Polso NoFlip	
        /// 3	Spalla SX + Gomito BASSO + Polso NoFlip	
        /// 4	Spalla DX + Gomito ALTO + Polso Flip	
        /// 5	Spalla SX + Gomito ALTO + Polso Flip	
        /// 6	Spalla DX + Gomito BASSO + Polso Flip	
        /// 7	Spalla SX + Gomito BASSO + Polso Flip</param>
        public static int GetInverseKin(DescPose pose, ref JointPos jPos, string pointName, int type = 0, int config = -1)
        {
            int err = Robot.GetInverseKin(type, pose, config, ref jPos);

            if (err != 0)
            {
                log.Error("Errore durante calcolo cinematica inversa : " + err);
                RobotKinError = true;
                if (!string.IsNullOrEmpty(pointName))
                {
                    throw new RobotKinException("Errore durante calcolo cinematica inversa per il punto " + pointName + " : Codice errore " + err);
                }
                else
                {
                    throw new RobotKinException("Errore durante calcolo cinematica inversa per il punto con codice errore " + err);
                }
            }

            return err;
        }

        /// <summary>
        /// Invia comando al robot per calcolare la cinematica diretta per ottenere le coordinate a partire dai giunti. 
        /// </summary>
        /// <param name="jPos"></param>
        /// <param name="pose"></param>
        public static int GetForwardKin(JointPos jPos, ref DescPose pose)
        {
            int err = Robot.GetForwardKin(jPos, ref pose);

            if (err != 0)
            {
                RobotKinError = true;
                log.Error("Errore durante calcolo cinematica diretta : " + err);
            }

            return err;
        }

        /// <summary>
        /// Invia comando al robot per calcolare la cinematica diretta per ottenere le coordinate a partire dai giunti.
        /// Genera una eccezione se il robot restituisce un errore.
        /// </summary>
        /// <param name="jPos"></param>
        /// <param name="pose"></param>
        /// <param name="pointName"></param>
        /// <returns></returns>
        public static int GetForwardKin(JointPos jPos, ref DescPose pose, string pointName)
        {
            int err = Robot.GetForwardKin(jPos, ref pose);

            if (err != 0)
            {
                log.Error("Errore durante calcolo cinematica diretta : " + err);
                RobotKinError = true;
                if (!string.IsNullOrEmpty(pointName))
                {
                    throw new Exception("Errore durante calcolo cinematica diretta per il punto " + pointName + " : Codice errore " + err);
                }
                else
                {
                    throw new Exception("Errore durante calcolo cinematica diretta per il punto con codice errore " + err);
                }
            }

            return err;
        }

        /// <summary>
        /// Movimento in assi cartesiani, simile al moveJ
        /// </summary>
        /// <param name="pose"></param>
        /// <param name="tool"></param>
        /// <param name="user"></param>
        /// <param name="vel"></param>
        /// <param name="acc"></param>
        /// <param name="ovl"></param>
        /// <param name="blendT"></param>
        /// <param name="config"></param>
        /// <returns></returns>
        public static int MoveCart(DescPose pose, int tool, int user, float vel, float acc, float ovl, float blendT, int config)
        {
#if DEBUG
            log.Info($"MoveL(x:{pose.tran.x}, y:{pose.tran.y}, z:{pose.tran.z}, rx:{pose.rpy.rx}, ry:{pose.rpy.ry}, rz:{pose.rpy.rz}");
#endif
            int err = Robot.MoveCart(pose, tool, user, vel, acc, ovl, blendT, config);
            if(err != 0)
            {
                log.Error("Errore durante moveCart: " + err);
                RobotMovementError = true;
            }
            return err;
        }

        /// <summary>
        /// Movimento lineare al punto target.
        /// </summary>
        /// <param name="jPos">Configurazione dei giunti</param>
        /// <param name="pose">Coordinate del punto target</param>
        /// <param name="tool">tool da utilizzare</param>
        /// <param name="user">Workspace da utilizzare</param>
        /// <param name="vel">Override della speed [0-100]</param>
        /// <param name="acc">Accelerazione da usare [0-100]</param>
        /// <param name="ovl">Scalatura della velocità [0-100]</param>
        /// <param name="blendR">Raggio di curvatura per smoothness in mm. -1.0 = movimento completo, [0 - 1000.0]</param>
        /// <param name="epos">Configurazione degli assi esterni in mm</param>
        /// <param name="search">0 = no wire search, 1 = wire search</param>
        /// <param name="offsetFlag">0 = No offset, 1 = Offset in base/workpiece coordinate system, 2 = Offset in tool coordinate system</param>
        /// <param name="offsetPos">Coordinate di offset</param>
        /// <param name="velAccParamMode">Parametro per velocità e accelerazione: 0 = percentuale, 1 = velocità fisica (mm/s) e accelerazione fisica (mm/s^2) </param>
        /// <param name="overSpeedStrategy">Overspeed handling strategy, 1 = Standard; 2 = Stop with error on overspeed; 3 = Adaptive deceleration, default is 0</param>
        /// <param name="speedPercent">Allowed deceleration threshold percentage [0-100], default 10%</param>
        /// <returns></returns>
        public static int MoveL(JointPos jPos, DescPose pose, int tool, int user, float vel, float acc, float ovl,
            float blendR, ExaxisPos epos, byte search, byte offsetFlag, DescPose offsetPos, int velAccParamMode = 0, int overSpeedStrategy = 0, int speedPercent = 5)
        {
#if DEBUG
            string joints = $"{string.Join(", ", jPos.jPos)}";
            log.Info($"MoveL(x:{pose.tran.x}, y:{pose.tran.y}, z:{pose.tran.z}, rx:{pose.rpy.rx}, ry:{pose.rpy.ry}, rz:{pose.rpy.rz}, giunti:{joints}");
#endif

            int err = Robot.MoveL(jPos, pose, tool, user, vel, acc, ovl, blendR, epos, search, offsetFlag, offsetPos, velAccParamMode, overSpeedStrategy, speedPercent);

            if (err != 0)
            {
                GetRobotMovementCode(err);
                RobotMovementError = true;
            }

            return err;
        }

        /// <summary>
        /// Movimento lineare al punto target con blendMode.
        /// </summary>
        /// <param name="jPos">Configurazione dei giunti</param>
        /// <param name="pose">Coordinate del punto target</param>
        /// <param name="tool">tool da utilizzare</param>
        /// <param name="user">Workspace da utilizzare</param>
        /// <param name="vel">Override della speed [0-100]</param>
        /// <param name="acc">Accelerazione da usare [0-100]</param>
        /// <param name="ovl">Scalatura della velocità [0-100]</param>
        /// <param name="blendR">Raggio di curvatura per smoothness in mm. -1.0 = movimento completo, [0 - 1000.0]</param>
        /// <param name="blendMode">Transition mode; 0 = Tangent transition; 1 = Corner transition</param>
        /// <param name="epos">Configurazione degli assi esterni in mm</param>
        /// <param name="search"> 0 = no wire search, 1 = wire search</param>
        /// <param name="offsetFlag">0 = No offset, 1 = Offset in base/workpiece coordinate system, 2 = Offset in tool coordinate system</param>
        /// <param name="offsetPos">Coordinate di offset</param>
        /// <param name="overSpeedStrategy">Overspeed handling strategy, 1 = Standard; 2 = Stop with error on overspeed; 3 = Adaptive deceleration, default is 0</param>
        /// <param name="speedPercent">Allowed deceleration threshold percentage [0-100], default 10%</param>
        /// <returns></returns>
        public static int MoveL(JointPos jPos, DescPose pose, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode,
             ExaxisPos epos, byte search, byte offsetFlag, DescPose offsetPos, int overSpeedStrategy = 0, int speedPercent = 5)
        {
#if DEBUG
            string joints = $"{string.Join(", ", jPos.jPos)}";
            log.Info($"MoveL(x:{pose.tran.x}, y:{pose.tran.y}, z:{pose.tran.z}, rx:{pose.rpy.rx}, ry:{pose.rpy.ry}, rz:{pose.rpy.rz}, giunti:{joints}");
#endif
            int err = Robot.MoveL(jPos, pose, tool, user, vel, acc, ovl, blendR, blendMode, epos, search, offsetFlag, offsetPos, overSpeedStrategy, speedPercent);
            if (err != 0)
            {
                GetRobotMovementCode(err);
                RobotMovementError = true;
            }
            return err;
        }

        /// <summary>
        /// Movimento lineare al punto target con blendMode e velAccParamMode.
        /// </summary>
        /// <param name="jPos">Configurazione dei giunti</param>
        /// <param name="pose">Coordinate del punto target</param>
        /// <param name="tool">tool da utilizzare</param>
        /// <param name="user">Workspace da utilizzare</param>
        /// <param name="vel">Override della speed [0-100]</param>
        /// <param name="acc">Accelerazione da usare [0-100]</param>
        /// <param name="ovl">Scalatura della velocità [0-100]</param>
        /// <param name="blendR">Raggio di curvatura per smoothness in mm. -1.0 = movimento completo, [0 - 1000.0]</param>
        /// <param name="blendMode">Transition mode; 0 = Tangent transition; 1 = Corner transition</param>
        /// <param name="epos">Configurazione degli assi esterni in mm</param>
        /// <param name="search"> 0 = no wire search, 1 = wire search</param>
        /// <param name="offsetFlag">0 = No offset, 1 = Offset in base/workpiece coordinate system, 2 = Offset in tool coordinate system</param>
        /// <param name="offsetPos">Coordinate di offset</param>
        /// <param name="velAccParamMode">Parametro per velocità e accelerazione: 0 = percentuale, 1 = velocità fisica (mm/s) e accelerazione fisica (mm/s^2) </param>
        /// <param name="overSpeedStrategy">Overspeed handling strategy, 1 = Standard; 2 = Stop with error on overspeed; 3 = Adaptive deceleration, default is 0</param>
        /// <param name="speedPercent">Allowed deceleration threshold percentage [0-100], default 10%</param>
        /// <returns></returns>
        public static int MoveL(JointPos jPos, DescPose pose, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode,
             ExaxisPos epos, byte search, byte offsetFlag, DescPose offsetPos, int velAccParamMode = 0, int overSpeedStrategy = 0, int speedPercent = 5)
        {
#if DEBUG
            string joints = $"{string.Join(", ", jPos.jPos)}";
            log.Info($"MoveL(x:{pose.tran.x}, y:{pose.tran.y}, z:{pose.tran.z}, rx:{pose.rpy.rx}, ry:{pose.rpy.ry}, rz:{pose.rpy.rz}, giunti:{joints}");
#endif
            int err = Robot.MoveL(jPos, pose, tool, user, vel, acc, ovl, blendR, blendMode, epos, search, offsetFlag, offsetPos, velAccParamMode, overSpeedStrategy, speedPercent);
            if (err != 0)
            {
                GetRobotMovementCode(err);
                RobotMovementError = true;
            }
            return err;
        }

        /// <summary>
        /// Movimento lineare al punto target con calcolo automatico della cinematica inversa.
        /// </summary>
        /// <param name="pose">Coordinate del punto target</param>
        /// <param name="tool">tool da utilizzare</param>
        /// <param name="user">Workspace da utilizzare</param>
        /// <param name="vel">Override della speed [0-100]</param>
        /// <param name="acc">Accelerazione da usare [0-100]</param>
        /// <param name="ovl">Scalatura della velocità [0-100]</param>
        /// <param name="blendR">Raggio di curvatura per smoothness in mm. -1.0 = movimento completo, [0 - 1000.0]</param>
        /// <param name="blendMode">Transition mode; 0 = Tangent transition; 1 = Corner transition</param>
        /// <param name="epos">Configurazione degli assi esterni in mm</param>
        /// <param name="search"> 0 = no wire search, 1 = wire search</param>
        /// <param name="offsetFlag">0 = No offset, 1 = Offset in base/workpiece coordinate system, 2 = Offset in tool coordinate system</param>
        /// <param name="offsetPos">Coordinate di offset</param>
        /// <param name="config">Inverse kinematic joint space configuration, [-1] = Reference current joint position for calculation, [0~7] = Solve based on specific joint space configuration</param>
        /// <param name="velAccParamMode">Parametro per velocità e accelerazione: 0 = percentuale, 1 = velocità fisica (mm/s) e accelerazione fisica (mm/s^2) </param>
        /// <param name="overSpeedStrategy">Overspeed handling strategy, 1 = Standard; 2 = Stop with error on overspeed; 3 = Adaptive deceleration, default is 0</param>
        /// <param name="speedPercent">Allowed deceleration threshold percentage [0-100], default 10%</param>
        /// <returns></returns>
        public static int MoveL(DescPose pose, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode,
             ExaxisPos epos, byte search, byte offsetFlag, DescPose offsetPos, int config, int velAccParamMode = 0, int overSpeedStrategy = 0, int speedPercent = 5)
        {
#if DEBUG
            log.Info($"MoveL(x:{pose.tran.x}, y:{pose.tran.y}, z:{pose.tran.z}, rx:{pose.rpy.rx}, ry:{pose.rpy.ry}, rz:{pose.rpy.rz}");
#endif
            int err = Robot.MoveL(pose, tool, user, vel, acc, ovl, blendR, blendMode, epos, search, offsetFlag, offsetPos, config, velAccParamMode, overSpeedStrategy, speedPercent);
            if (err != 0)
            {
                GetRobotMovementCode(err);
                RobotMovementError = true;
            }
            return err;
        }

        /// <summary>
        /// Movimento in joint al punto target.
        /// </summary>
        /// <param name="jPos">Configurazione dei giunti</param>
        /// <param name="pose">Coordinate del punto target</param>
        /// <param name="tool">Tool da utilizzare</param>
        /// <param name="user">Workspace da utilizzare</param>
        /// <param name="vel">Override della speed [0-100]</param>
        /// <param name="acc">Accelerazione da usare [0-100]</param>
        /// <param name="ovl">Scalatura della velocità [0-100]</param>
        /// <param name="epos">Configurazione degli assi esterni in mm</param>
        /// <param name="blendT">[-1.0] = move to position (blocking), [0~500.0] = smoothing time (non-blocking), unit ms</param>
        /// <param name="offsetFlag">0 = No offset, 1 = Offset in base/workpiece coordinate system, 2 = Offset in tool coordinate system</param>
        /// <param name="offsetPos">Coordinate di offset</param>
        /// <returns></returns>
        public static int MoveJ(JointPos jPos, DescPose pose, int tool, int user, float vel, float acc, float ovl,
           ExaxisPos epos, float blendT, byte offsetFlag, DescPose offsetPos)
        {
#if DEBUG
            string joints = $"{string.Join(", ", jPos.jPos)}";
            log.Info($"MoveL(x:{pose.tran.x}, y:{pose.tran.y}, z:{pose.tran.z}, rx:{pose.rpy.rx}, ry:{pose.rpy.ry}, rz:{pose.rpy.rz}, giunti:{joints}");
#endif
            int err = Robot.MoveJ(jPos, pose, tool, user, vel, acc, ovl, epos, blendT, offsetFlag, offsetPos);
            if (err != 0)
            {
                GetRobotMovementCode(err);
                RobotMovementError = true;
            }
            return err;
        }

        /// <summary>
        /// Movimento ni joint al punto target, calcola automaticamente la cinematica diretta per ottenere il desc pose
        /// </summary>
        /// <param name="jPos">Configurazione dei giunti</param>
        /// <param name="tool">Tool da utilizzare</param>
        /// <param name="user">Workspace da utilizzare</param>
        /// <param name="vel">Override della speed [0-100]</param>
        /// <param name="acc">Accelerazione da usare [0-100]</param>
        /// <param name="ovl">Scalatura della velocità [0-100]</param>
        /// <param name="epos">Configurazione degli assi esterni in mm</param>
        /// <param name="blendT">[-1.0] = move to position (blocking), [0~500.0] = smoothing time (non-blocking), unit ms</param>
        /// <param name="offsetFlag">0 = No offset, 1 = Offset in base/workpiece coordinate system, 2 = Offset in tool coordinate system</param>
        /// <param name="offsetPos">Coordinate di offset</param>
        /// <returns></returns>
        public static int MoveJ(JointPos jPos, int tool, int user, float vel, float acc, float ovl,
           ExaxisPos epos, float blendT, byte offsetFlag, DescPose offsetPos)
        {
#if DEBUG
            string joints = $"{string.Join(", ", jPos.jPos)}";
            log.Info($"MoveL(giunti:{joints}");
#endif
            int err = Robot.MoveJ(jPos, tool, user, vel, acc, ovl, epos, blendT, offsetFlag, offsetPos);
            if (err != 0)
            {
                GetRobotMovementCode(err);
                RobotMovementError = true;
            }
            return err;
        }

        /// <summary>
        /// Movimento in JOG
        /// </summary>
        /// <param name="refType">0-joint pointing, 2-pointing in base coordinate system, 4-pointing in tool coordinate system, 8-pointing in artifact coordinate system.</param>
        /// <param name="nb">1-joint 1 (or x-axis), 2-joint 2 (or y-axis), 3-joint 3 (or z-axis), 4-joint 4 (or rotate around x-axis), 5-joint 5 (or rotate around y-axis), 6-joint 6 (or rotate around z-axis)</param>
        /// <param name="dir">0-negative direction, 1-positive direction</param>
        /// <param name="vel">Override della speed [0-100]</param>
        /// <param name="acc">Accelerazione da usare [0-100]</param>
        /// <param name="maxDis">Maximum angle of a single tap, in [°] or distance, in [mm]</param>
        /// <returns></returns>
        public static int StartJOG(byte refType, byte nb, byte dir, float vel, float acc, float maxDis)
        {
            int err = Robot.StartJOG(refType, nb, dir, vel, acc, maxDis);
            if (err != 0)
            {
                GetRobotMovementCode(err);
                RobotMovementError = true;
            }
            return err;
        }

        /// <summary>
        /// jog pointwise deceleration stops.
        /// </summary>
        /// <param name="stopByte">1-joint-point deceleration stop, 3-point deceleration stop in base coordinate system, 5-point deceleration stop in tool coordinate system, 9-point deceleration stop in workpiece coordinate system</param>
        /// <returns></returns>
        public static int StopJOG(byte stopByte)
        {
            int err = Robot.StopJOG(stopByte);
            if (err != 0)
            {
                GetRobotMovementCode(err);
            }
            return err;
        }

        /// <summary>
        /// jog tapping stops immediately.
        /// </summary>
        /// <returns></returns>
        public static int ImmStopJOG()
        {
            int err = Robot.ImmStopJOG();
            if (err != 0)
            {
                GetRobotMovementCode(err);
            }
            return err;
        }

        #endregion

        #region DI/O

        /// <summary>
        /// Imposta l'uscita digitale a on o a off del controllore.
        /// NB: Può capitare che il relè sia collegato in modo che funzioni in modo invertito (1=OFF, 0=ON)
        /// </summary>
        /// <param name="index">id dell'uscita da usare</param>
        /// <param name="status">0: OFF, 1: ON</param>
        /// <param name="smooth">0-non-smooth, 1-smooth</param>
        /// <param name="block">0-blocking, 1-non-blocking</param>
        /// <returns></returns>
        public static int SetDO(int index, byte status, byte smooth = 0, byte block = 0)
        {
            int err = Robot.SetDO(index, status, smooth, block);
            if (err != 0)
            {
                log.Error("Errore durante set DO " + index + " a " + status + " : Codice errore " + err);
            }
            return err;
        }

        /// <summary>
        /// Lettura dello stato delle uscite del controllore.
        /// </summary>
        /// <param name="state_high"></param>
        /// <param name="state_low"></param>
        /// <returns></returns>
        public static int GetDO(ref int state_high, ref int state_low)
        {
            int err = Robot.GetDO(ref state_high, ref state_low);
            if (err != 0)
            {
                log.Error("Errore durante get DO: " + err);
            }
            return err;
        }

        /// <summary>
        /// Get dello stato dell'uscita id
        /// </summary>
        /// <param name="id"></param>
        /// <param name="block"></param>
        /// <param name="level"></param>
        /// <returns></returns>
        public static int GetDI(int id, byte block, ref byte level)
        {
            int err = Robot.GetDI(id, block, ref level);
            if(err != 0)
            {
                log.Error("Errore durante get DI: " + err);
            }
            return err;
        }

        /// <summary>
        /// Imposta l'uscita digitale a on o a off del tool.
        /// </summary>
        /// <param name="index">id dell'uscita da usare</param>
        /// <param name="status">0: OFF, 1: ON</param>
        /// <param name="smooth">0-non-smooth, 1-smooth</param>
        /// <param name="block">0-blocking, 1-non-blocking</param>
        /// <returns></returns>
        public static int SetToolDO(int index, byte status, byte smooth = 0, byte block = 0)
        {
            int err = Robot.SetToolDO(index, status, smooth, block);
            if (err != 0)
            {
                log.Error("Errore durante get DO: " + err);
            }
            return err;
        }

        /// <summary>
        /// Lettura dello stato delle uscite del tool.
        /// </summary>
        /// <param name="do_state">output state, do0~do1 corresponds to bit1~bit2, start from bit0</param>
        /// <returns></returns>
        public static int GetToolDO(ref byte do_state)
        {
            int err = Robot.GetToolDO(ref do_state);
            if (err != 0)
            {
                log.Error("Errore durante get tool DO: " + err);
            }
            return err;
        }

        #endregion

        #region Informazioni

        /// <summary>
        /// Restituisce la versione sdk c# di farino
        /// </summary>
        /// <param name="sdkVer"></param>
        /// <returns></returns>
        public static int GetRobotSDKVersion(ref string sdkVer)
        {
            int err = Robot.GetSDKVersion(ref sdkVer);
            if(err != 0)
            {
                log.Error("Errore durante get SDK version");
            }
            return err;
        }

        /// <summary>
        /// Restituisce l'ip impostato nel controllore
        /// </summary>
        /// <param name="controllerIp"></param>
        /// <returns></returns>
        public static int GetRobotControllerIP(ref string controllerIp)
        {
            int err = Robot.GetSDKVersion(ref controllerIp);
            if (err != 0)
            {
                log.Error("Errore durante get IP controllore");
            }
            return err;
        }

        /// <summary>
        /// Restituisce le versioni del modello, web e controller
        /// </summary>
        /// <param name="modelVer"></param>
        /// <param name="webVer"></param>
        /// <param name="controllerVer"></param>
        /// <returns></returns>
        public static int GetRobotSoftwareVersion(ref string modelVer, ref string webVer, ref string controllerVer)
        {
            int err = Robot.GetSoftwareVersion(ref modelVer, ref webVer, ref controllerVer);
            if (err != 0)
            {
                log.Error("Errore durante get versione software del controllore");
            }
            return err;
        }

        /// <summary>
        /// Get della versione firmware del controllore e driver
        /// </summary>
        /// <param name="fwBoardBoxBoardVer"></param>
        /// <param name="fwDriver1Ver"></param>
        /// <param name="fwDriver2Ver"></param>
        /// <param name="fwDriver3Ver"></param>
        /// <param name="fwDriver4Ver"></param>
        /// <param name="fwDriver5Ver"></param>
        /// <param name="fwDriver6Ver"></param>
        /// <param name="fwEndBoardVer"></param>
        /// <returns></returns>
        public static int GetRobotFirmwareVersion(ref string fwBoardBoxBoardVer, ref string fwDriver1Ver, ref string fwDriver2Ver, ref string fwDriver3Ver,
            ref string fwDriver4Ver, ref string fwDriver5Ver, ref string fwDriver6Ver, ref string fwEndBoardVer)
        {
            int err = Robot.GetFirmwareVersion(ref fwBoardBoxBoardVer, ref fwDriver1Ver, ref fwDriver2Ver, ref fwDriver3Ver,
                ref fwDriver4Ver, ref fwDriver5Ver, ref fwDriver6Ver, ref fwEndBoardVer);
            if(err != 0)
            {
                log.Error("Errore durante get firmware del robot: " + err);
            }
            return err;
        }

        /// <summary>
        /// Get della versione hardware del controllore e driver
        /// </summary>
        /// <param name="hwBoardBoxBoardVer"></param>
        /// <param name="hwDriver1Ver"></param>
        /// <param name="hwDriver2Ver"></param>
        /// <param name="hwDriver3Ver"></param>
        /// <param name="hwDriver4Ver"></param>
        /// <param name="hwDriver5Ver"></param>
        /// <param name="hwDriver6Ver"></param>
        /// <param name="hwEndBoardVer"></param>
        /// <returns></returns>
        public static int GetRobotHardwareVersion(ref string hwBoardBoxBoardVer, ref string hwDriver1Ver, ref string hwDriver2Ver, ref string hwDriver3Ver,
            ref string hwDriver4Ver, ref string hwDriver5Ver, ref string hwDriver6Ver, ref string hwEndBoardVer)
        {
            int err = Robot.GetHardwareVersion(ref hwBoardBoxBoardVer, ref hwDriver1Ver, ref hwDriver2Ver, ref hwDriver3Ver,
                ref hwDriver4Ver, ref hwDriver5Ver, ref hwDriver6Ver, ref hwEndBoardVer);
            if(err != 0)
            {
                log.Error("Errore durante get hardware versione del controllore");
            }
            return err;
        }

        #endregion

        #endregion

        #region Metodi helper

        /// <summary>
        /// Impacchetta gli allarmi del robot in un unico intero da scrivere al plc
        /// </summary>
        /// <returns></returns>
        private static int BuildRobotAlarms()
        {
            lock(_robotErrorLock)
            {
                _robotGeneralError = (_robotError |
                                _robotKinError |
                                _robotMovementError |
                                _robotPropertiesError |
                                !_robotConnected
                                ) ? true : false;

                int errorWord = Convert.ToInt16(_robotGeneralError);

                errorWord |= _robotError ? 1 << 1 : 0;
                errorWord |= _robotKinError ? 1 << 2 : 0;
                errorWord |= _robotMovementError ? 1 << 3 : 0;
                errorWord |= _robotPropertiesError ? 1 << 4 : 0;
                errorWord |= _robotConnected ? 1 << 5 : 0;

                return errorWord;
            }
        }

        /// <summary>
        /// Impacchetta gli allarmi dell'applicazione in un unico intero da scrivere al plc
        /// </summary>
        /// <returns></returns>
        private static int BuildApplicationAlarms()
        {
            lock(_applicationErrorLock)
            {
                int errorWord = 0;

                errorWord |= _runTimeError ? 1 << 0 : 0;
                errorWord |= _dataError ? 1 << 1 : 0;
                errorWord |= _homeRoutineError ? 1 << 2 : 0;
                errorWord |= _trayNotPresentError ? 1 << 3 : 0;
                errorWord |= _gripperNotClosedError ? 1 << 4 : 0;
                errorWord |= _slideNotOutError ? 1 << 5 : 0;
                errorWord |= _slideNotInError ? 1 << 6 : 0;

                return errorWord;
            }
        }

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
                RobotError = true;
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
                GetRobotSDKVersion(ref RobotSdkVer);
                GetRobotControllerIP(ref RobotCurrentIP);
                GetRobotSoftwareVersion(ref RobotModelVer, ref RobotWebVer, ref RobotControllerVer);
                GetRobotFirmwareVersion(ref RobotFwBoxBoardVer, ref RobotFwDriver1Ver, ref RobotFwDriver2Ver, ref RobotFwDriver3Ver,
                    ref RobotFwDriver4Ver, ref RobotFwDriver5Ver, ref RobotFwDriver6Ver, ref RobotFwEndBoardVer);
                GetRobotHardwareVersion(ref RobotHwBoxBoardVer, ref RobotHwDriver1Ver, ref RobotHwDriver2Ver, ref RobotHwDriver3Ver,
                    ref RobotHwDriver4Ver, ref RobotHwDriver5Ver, ref RobotHwDriver6Ver, ref RobotHwEndBoardVer);
            }
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
                err = GetRobotErrorCode(ref maincode, ref subcode);
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
                    RobotError = true;
                }
                //else if (maincode == 0)
                //{
                //    robotError = 0;
                //}
            }
        }

        /// <summary>
        /// Reset iniziali delle variabili PLC
        /// </summary>
        private static void ResetPLCVariables()
        {

        }

        /// <summary>
        /// Controlla il tool e user correnti
        /// </summary>
        private static void CheckCurrentToolAndUser()
        {
            GetActualTCPNum(ref currentTool, 1);
            GetActualWobjCoord(ref currentUser, 1);
        }

        /// <summary>
        /// Controlla e aggiorna il livello di collisione
        /// </summary>
        private static void CheckLevelCollision()
        {
            // Get del valore PLC
            changeCollisionLevel = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Sel_Service));

            // Se il valore è 0 ed è cambiato di stato
            if (changeCollisionLevel == 0 && (changeCollisionLevel != prevChangeCollisionLevel))
            {
                prevChangeCollisionLevel = 0;
                collisionManager.ChangeRobotCollision(collisionLevelService);
            }
            // Se il valore è 1 ed è cambiato di stato
            else if (changeCollisionLevel == 1 && (changeCollisionLevel != prevChangeCollisionLevel)) 
            {
                prevChangeCollisionLevel = 1;
                collisionManager.ChangeRobotCollision(collisionLevel);
            }
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

                    //ClearRobotAlarm();
                    //ClearRobotQueue();
                    //ResetRobotSteps();

                    prevIsPlcConnected = true;
                }
            }
        }

        /// <summary>
        /// Verifica se il punto corrente è all'interno dell'area di ingombro rispetto a uno qualsiasi dei punti di partenza
        /// </summary>
        /// <param name="startPoints">Array con i punti di partenza per Pick, Place e Home</param>
        private static void CheckIsRobotInObstructionArea(DescPose[] startPoints)
        {
            isInPositionHome = checker_ingombro_home.IsInCubeObstruction(startPoints[0], TCPCurrentPosition);
            isInPositionCarrello1 = checker_ingombro_carrello1.IsInParallelepipedObstruction(startPoints[1], TCPCurrentPosition);
            isInPositionCarrello2 = checker_ingombro_carrello2.IsInParallelepipedObstruction(startPoints[2], TCPCurrentPosition);
            isInPositionBeor = checker_ingombro_beor.IsInParallelepipedObstruction(startPoints[3], TCPCurrentPosition);

            bool plcIsInPositionHome = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Home_inPos));
            bool plcIsInPositionCarrello1 = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Carrello1));
            bool plcIsInPositionCarrello2 = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Carrello2));
            bool plcIsInPositionBeor = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Beor));

            if (isInPositionCarrello1) // Ora in zona carrello 1
            {
                if (!plcIsInPositionCarrello1) // Prima non ero in zona carrello 1 o sul plc c'è un valore diverso
                {
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello1, 1, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello2, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Beor, 0, "INT16");
                }
            }
            else if (isInPositionCarrello2) // Ora in zona carrello 2
            {
                if (!plcIsInPositionCarrello2) // Prima non ero in zona carrello 2 o sul plc c'è un valore diverso
                {
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello1, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello2, 1, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Beor, 0, "INT16");
                }
            }
            else if (isInPositionHome) // Ora in zona di home
            {
                if (!plcIsInPositionHome) // Prima non ero in zona di home o sul plc c'è un valore diverso
                {
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello1, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello2, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 1, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Beor, 0, "INT16");
                }
            }
            else if (isInPositionBeor) // Ora in zona beor
            {
                if (!plcIsInPositionBeor) // Prima non ero in zona beor o sul plc c'è un valore diverso
                {
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello1, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello2, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Beor, 1, "INT16");
                }
            }
            else // Altrimenti
            {
                bool plcIsFuoriIngombro = !plcIsInPositionHome && !plcIsInPositionCarrello1 && !plcIsInPositionCarrello2 && !plcIsInPositionBeor;
                if (!plcIsFuoriIngombro) // Prima non ero fuori ingombro o sul plc c'è un valore sbagliato
                {
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello1, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Carrello2, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Home_inPos, 0, "INT16");
                    RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Beor, 0, "INT16");
                }
            }
        }

        /// <summary>
        /// Verifica se il punto corrente è all'interno dell'area di safe zone
        /// </summary>
        private static void CheckIsRobotInSafeZone(DescPose pSafeZone)
        {
            if (!AlarmManager.isFormReady)
                return;

            isInSafeZone = checker_safeZone.IsInParallelepipedObstruction(pSafeZone, TCPCurrentPosition);

            bool plcIsInSafeZone = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.RET_Zone_Safe));

            if (isInSafeZone && !plcIsInSafeZone) // Se il robot è nella safe zone
            {
                FormHomePage.Instance.RobotSafeZone.BackgroundImage = Resources.safeZone_green32;
                RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Safe, 1, "INT16");
            }
            else if (!isInSafeZone && plcIsInSafeZone) // Se il robot non è nella safe zone
            {
                FormHomePage.Instance.RobotSafeZone.BackgroundImage = Resources.safeZone_yellow32;
                RefresherTask.AddUpdate(PLCTagName.ACT_Zone_Safe, 0, "INT16");
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
        private static bool GetRobotProperties()
        {
            try
            {
                log.Info("Inizio impostazione delle proprieta' del robot dal database.");

                // Ottieni le proprietà del robot dal database
                DataTable dt_robotProperties = RobotDAO.GetRobotProperties(ConnectionString);
                if (dt_robotProperties == null)
                {
                    log.Error("La tabella delle proprietà del robot è nulla.");
                    return false;
                }

                // Estrai e assegna le proprietà del robot
                int _speed = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_SPEED_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float _velocity = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_VELOCITY_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float _blendT = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_BLENDT_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float _acceleration = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_ACCELERATION_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float _ovl = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_OVL_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int _tool = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_TOOL_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int _user = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_USER_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int _weight = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_WEIGHT_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int _velRec = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_VELREC_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int _collLev = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_COLLISION_LEVELS_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                float _blendR = float.Parse(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_BLENDR_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());
                int _collLevService = Convert.ToInt16(dt_robotProperties.Rows[RobotDAOSqlite.ROBOT_PROPERTIES_COLLISION_LEVELS_SERVICE_ROW_INDEX]
                    [RobotDAOSqlite.ROBOT_PROPERTIES_VALUE_COLUMN_INDEX].ToString());

                // Creazione dell'oggetto robotProperties
                robotProperties = new RobotProperties(_speed, _velocity, _blendT, _acceleration, _ovl, _tool, _user, _weight, _velRec);

                log.Info($"GetRobotProperties completata: " +
                         $"Speed: {_speed} " +
                         $"Velocity: {_velocity} " +
                         $"Blend T: {_blendT} " +
                         $"Blend R: {_blendR} " +
                         $"Acceleration: {_acceleration} " +
                         $"Ovl: {_ovl} " +
                         $"Tool: {_tool} " +
                         $"User: {_user} " +
                         $"Weight: {_weight} " +
                         $"VelRec: {_velRec} " +
                         $"CollLev: {_collLev} " +
                         $"CollLevServ: {_collLevService} "
                         );

                // Modifica delle variabili statiche e globali di RobotManager
                speed = _speed;
                vel = _velocity;
                acc = _acceleration;
                ovl = _ovl;
                blendT = _blendT;
                blendR = _blendR;
                tool = _tool;
                user = _user;
                weight = _weight;
                velRec = _velRec;
                currentCollisionLevel = _collLev;
                collisionLevel = _collLev;
                collisionLevelService = _collLevService;
                

                return true;
            }
            catch (Exception ex)
            {
                log.Error("Errore durante GetRobotProperties: " + ex.ToString());
                return false;
            }
        }

        /// <summary>
        /// Imposta le proprietà principali del robot: speed-frame-tool-collision levels
        /// </summary>
        /// <returns></returns>
        private static bool SetRobotProperties()
        {
            if (!SetRobotSpeed(robotProperties.Speed))
                return false;

            if (!frameManager.ChangeRobotFrame(user))
                return false;

            if (!toolManager.ChangeRobotTool(tool))
                return false;

            if (!collisionManager.ChangeRobotCollision(currentCollisionLevel))
                return false;

            if (!SetRobotPayload(robotProperties.Weight))
                return false;

            return true;
        }

        /// <summary>
        /// Generazione evento da allarme ricevuto
        /// </summary>
        /// <param name="e"></param>
        private static void OnAllarmeGenerato(EventArgs e)
        {
            AllarmeGenerato?.Invoke(null, e);
        }

        /// <summary>
        /// Generazione evento da allarmi resettati
        /// </summary>
        /// <param name="e"></param>
        private static void OnAllarmeResettato(EventArgs e)
        {
            AllarmeResettato?.Invoke(null, e);
        }

        /// <summary>
        /// Generazione eventi
        /// </summary>
        private static void TriggerAllarmeGenerato()
        {
            OnAllarmeGenerato(EventArgs.Empty);
        }

        /// <summary>
        /// Trigger attivato quando vengono cancellati gli allarmi
        /// </summary>
        private static void TriggerAllarmeResettato()
        {
            OnAllarmeResettato(EventArgs.Empty);
        }

        /// <summary>
        /// Normalizza angolo robot
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        private static float NormalizeAngle(float angle)
        {
            while (angle > 180f) angle -= 360f;
            while (angle <= -180f) angle += 360f;
            return angle;
        }

        /// <summary>
        /// Invia posizioni al PLC in formato cartesiano e joint
        /// </summary>
        /// <param name="jPos">Posizione in joint ottenuta dal calcolo di cinematica inversa partendo dalla posizione TCP</param>
        private static void CheckRobotPosition(JointPos jPos)
        {
            if (!AlarmManager.isRobotConnected)
                return;

            // Calcolo della posizione in joint eseguendo il calcolo di cinematica inversa
            GetInverseKin(TCPCurrentPosition, ref jPos);

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
