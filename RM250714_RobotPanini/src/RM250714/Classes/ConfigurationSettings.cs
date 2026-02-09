using RM.src.RM250714.Classes.FR20.Jog;
using RM.src.RM250714.Classes.PLC;
using RMLib.Alarms;
using RMLib.DataAccess;
using RMLib.Logger;
using RMLib.MessageBox;
using RMLib.PLC;
using RMLib.Security;
using RMLib.Translations;
using RMLib.Utils;
using RMLib.View;
using System.Data;
using System.Linq;
using System.Reflection;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RM.src.RM250714
{
    /// <summary>
    /// Classe che gestisce la FormLoading
    /// e che configura e avvia comunicazione con PLC
    /// </summary>
    public class ConfigurationSettings
    {
        #region Proprietà della classe ConfigurationSettings

        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        /// <summary>
        /// Data table che conterrà la tabella omonima del database con le configurazioni di base del progetto
        /// </summary>
        public static DataTable configurationProperties;

        #endregion

        /// <summary>
        /// Costruttore vuoto
        /// </summary>
        public ConfigurationSettings()
        {
           
        }

        #region Metodi della classe ConfigurationSettings
        /// <summary>
        /// Metodo che gestisce avanzamento progressBar della FormLoading.
        /// Viene fatto il get del setting dal database.
        /// Avvio e gestione comunicazione con PLC
        /// </summary>
        /// <param name="lb_ProgressBar">Etichetta ProgressBar avanzameto</param>
        /// <param name="progressBar1">ProgressBar avanzamento</param>
        /// <param name="formLoading">Form di caricamento</param>
        public async void Setup(Label lb_ProgressBar, CustomProgressBar progressBar1, FormLoading formLoading, string[] args)
        {
            FormHomePage homePage = new FormHomePage();

            lb_ProgressBar.Visible = true;

            SqliteConnectionConfiguration databaseConnection = new SqliteConnectionConfiguration();
            log.Info("Il percorso del database è: " + databaseConnection.GetConnectionString());

            RMLib.Environment.Environment.SettingUpFromConsoleParam(args);

            log.Info("ENVIRONMENT SETTING: " + RMLib.Environment.Environment.GetEnvironment());
            log.Info("********** Application is starting... **********");

            if (!Configuration.BasicConfigurationFromFile())
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during get basic info from file");
                log.Error("Error during get basic info from file");
                Application.Exit();
            }

            // ProgressBar a 20%
            progressBar1.Increment(20);
            lb_ProgressBar.Text = "Connect to database...";
            await Task.Delay(ProjectVariables.ProgressBarDelay);

            if (!GetConfigureProperties(databaseConnection.GetConnectionString()))
            {
                log.Error("Error during configuration_properties configuration");
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during configuration_properties configuration");
            }

            // ProgressBar a 30%
            progressBar1.Increment(20);
            lb_ProgressBar.Text = "Check other applications running...";
            await Task.Delay(ProjectVariables.ProgressBarDelay);

            if (ExisistingInstanceProgram())
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during configuration_properties configuration");
                log.Error("Another instance is in execution!");
                Application.Exit();
            }

            // ProgressBar a 40%
            progressBar1.Increment(10);
            lb_ProgressBar.Text = "Create executor service...";

            // ProgressBar a 45%
            progressBar1.Increment(5);
            lb_ProgressBar.Text = "Configure PLC coms...";
            await Task.Delay(ProjectVariables.ProgressBarDelay);

            //ProgressBar a 50%
            progressBar1.Increment(10);
            lb_ProgressBar.Text = "Configure alarms...";
            AlarmManager.Init();

            if (!PLCConfig.configurePLCs(databaseConnection.GetConnectionString()))
            {
                log.Error("Error during plc configuration loading");
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during plc configuration loading");
                Application.Exit();
            }

            if (!PLCConfig.initPLCs())
            {
                log.Error("Error during plc configuration loading");
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during plc configuration loading");
                Application.Exit();
            }

            // Assegno le variabili da guardare per il plcClock
            RefresherTask.SetPlcClock(PLCTagName.LifeBit_in, PLCTagName.LifeBit_out, "INT16");

            // SyncManager.StartWriteProcessor();
            if (!PLCConfig.startPLCManagers(databaseConnection.GetConnectionString()))
            {
                log.Error("Error during plc configuration loading");
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during plc configuration loading");
                Application.Exit();
            }

            // ProgressBar a 55%
            progressBar1.Increment(10);
            lb_ProgressBar.Text = "Configure security...";
            await Task.Delay(ProjectVariables.ProgressBarDelay);

            if (!SecurityManager.InitSecurityManager())
            {
                log.Error("Error during security loading");
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during security loading");
            }

            //ProgressBar a 60%
            progressBar1.Increment(5);
            lb_ProgressBar.Text = "Configure Robot...";

            //ProgressBar a 80%
            progressBar1.Increment(5);
            lb_ProgressBar.Text = "Load plant configuration...";
            await Task.Delay(ProjectVariables.ProgressBarDelay);

            if (!ApplicationConfig.ConfigureApplications(databaseConnection.GetConnectionString()))
            {
                log.Info("Nessuna applicazione esistente");
            }

            if (!ApplicationConfig.InitApplications())
            {
                if (ApplicationConfig.applications.Rows.Count > 0)
                {
                    log.Error("Error during applications loading");
                    CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during applications loading");
                }
            }

            string robotIp = string.Empty;
            bool useOverlappingZones = false;
            foreach (DataRow row in configurationProperties.Rows)
            {
                // Controlla il valore di robot ip
                if (row["key"] != null && row["key"].ToString() == "ROBOT_IP")
                {
                    robotIp = row[RMLib.Environment.Environment.GetEnvironment().ToString().ToLower()].ToString();
                    break;
                }
                if (row["key"] != null && row["key"].ToString() == "USE_OVERLAPPING_ZONES")
                {
                    if(bool.TryParse(row[RMLib.Environment.Environment.GetEnvironment().ToString().ToLower()].ToString(), out useOverlappingZones))
                    {
                       
                    }
                    RobotManager.useOverlappingZones = useOverlappingZones;
                    break;
                }
                // Controlla il valore di useOverlappingZones
            }

            if ((string.IsNullOrEmpty(robotIp)))
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Errore durante configurazione del Robot");
                Application.Exit();
            }

            bool robotConfigurated = await RobotManager.InitRobot(robotIp);

            if (!robotConfigurated)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Errore durante configurazione del Robot");
                Application.Exit();
            }

            if (!JogMovement.InitJog())
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Errore durante configurazione del Jog del Robot");
                log.Error("Errore durante configurazione del Jog del Robot");
            }

            //ProgressBar a 85%
            progressBar1.Increment(5);
            lb_ProgressBar.Text = "Start Robot monitoring...";

            //ProgressBar a 90%
            progressBar1.Increment(10);
            lb_ProgressBar.Text = "Load translations...";
            await Task.Delay(ProjectVariables.ProgressBarDelay);

            if (!TranslationManager.Init(databaseConnection.GetConnectionString()))
            {
                log.Error("Error during load translations");
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Error during load translations");
            }

            //ProgressBar a 100%
            progressBar1.Increment(10);
            await Task.Delay(ProjectVariables.ProgressBarDelay);

#if DEBUG
            SecurityManager.SetActualUser(new User("debug", "debug", 10, "psw", "mai", "mai"));
#endif

            formLoading.OpenHomePage(homePage);
        }

        /// <summary>
        /// Metodo che esegue get della tabella database configuration_properties
        /// </summary>
        /// <param name="connectionString">Stringa di connessione</param>
        /// <returns></returns>
        private bool GetConfigureProperties(string connectionString)
        {
            log.Info("configuration_properties configuration");

            PLCDAOSqlite DAO = new PLCDAOSqlite();

            configurationProperties = DAO.GetConfigurationProperties(connectionString);

            if (configurationProperties.Rows.Count < 1)
            {
                return false;
            }

            log.Info("configuration_properties configured");

            return true;
        }

        /// <summary>
        /// Controlla se è già avviata un'altra istanza dell'applicativo
        /// </summary>
        /// <returns></returns>
        public bool ExisistingInstanceProgram()
        {
            if (System.Diagnostics.Process.GetProcessesByName(
                System.IO.Path.GetFileNameWithoutExtension(Assembly.GetEntryAssembly().Location)).Count() > 1
                )
            {
                return true;
            }
            return false;
        }
        #endregion
    }
}
