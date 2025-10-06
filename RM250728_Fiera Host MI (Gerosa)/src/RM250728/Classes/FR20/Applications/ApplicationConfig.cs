using RM.src.RM250728.Classes.FR20.Applications.Application;
using RMLib.DataAccess;
using RMLib.Logger;
using System;
using System.Collections.Generic;
using System.Data;

namespace RM.src.RM250728
{
    /// <summary>
    /// Gestisce la applicazioni del robot e anche gli eventi che sono ad esse correlati
    /// </summary>
    public class ApplicationConfig
    {
        #region Parametri di ApplicationConfig

        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        /// <summary>
        /// Dizionario Observable che conterrà tutte le applicazioni del Robot
        /// </summary>
        public static ObservableApplicationsDictionary applicationsManager;

        /// <summary>
        /// DataTable contenete tutte le applicazioni
        /// </summary>
        public static DataTable applications;

        /// <summary>
        /// Timer controllo ingombro
        /// </summary>
        private static System.Threading.Timer robotObstructionTimer;

        #endregion

        #region Metodi di ApplicationConfig

        /// <summary>
        /// Set della dataTable applications
        /// </summary>
        /// <param name="connectionString">Stringa di connessione</param>
        /// <returns></returns>
        public static bool ConfigureApplications(String connectionString)
        {
            log.Info("Start applications configuration");

            RobotDAOSqlite DAO = new RobotDAOSqlite();

            applications = DAO.GetRobotApplications(connectionString);

            if (applications.Rows.Count < 1)
            {
                return false;
            }

            log.Info("applications configured");

            return true;
        }

        /// <summary>
        /// Metodo che lancia procedura che riempie il dictionary delle applicazioni
        /// </summary>
        /// <returns></returns>
        public static bool InitApplications()
        {

            applicationsManager = new ObservableApplicationsDictionary();
            FillApplicationsManagerDictionary();
            if (applicationsManager.getDictionary().Count < 1)
                return false;

            //RobotManager.applicationName = ApplicationConfig.applicationsManager.GetApplicationName();
            RobotManager.applicationName = "Applicazione RM250728 Robot e nastri";

            log.Info("Applications loaded");

            return true;
        }

        /// <summary>
        /// Metodo che riempie il dictionary delle ricette
        /// </summary>
        private static void FillApplicationsManagerDictionary()
        {
            SqliteConnectionConfiguration databaseConnection = new SqliteConnectionConfiguration();
            string connectionString = databaseConnection.GetConnectionString();
            RobotDAOSqlite DAO = new RobotDAOSqlite();

            int id;
            String applicationName;

            foreach (DataRow app in applications.Rows)
            {
                id = Convert.ToInt16(app["id"]);
                applicationName = app["ApplicationName"].ToString();

                DataTable applicationPositions = DAO.GetPointsPosition(connectionString, applicationName);
                List<ApplicationPositions> positions = new List<ApplicationPositions>();

                // Variabili necessarie alla creazione dell'oggeto GunSettings
                string guid_gun_settings;
                string guid_pos;
                int? id_gun_settings;
                int? feed_air;
                int? dosage_air;
                int? gun_air;
                int? kV;
                int? microampere;
                int? status;
                string application;

                // Riempio ogni componente della ricetta che sto ciclando
                foreach (DataRow pos in applicationPositions.Rows)
                {
                    guid_gun_settings = pos["guid_gun_settings"].ToString();
                    guid_pos = pos["guid_pos"].ToString();
                    id_gun_settings = pos["id_gun_settings"].ToString() != "" ? Convert.ToInt32(pos["id_gun_settings"]) : (int?)null;
                    feed_air = pos["feed_air"].ToString() != "" ? Convert.ToInt32(pos["feed_air"]) : (int?)null;
                    dosage_air = pos["dosage_air"].ToString() != "" ? Convert.ToInt32(pos["dosage_air"]) : (int?)null;
                    gun_air = pos["gun_air"].ToString() != "" ? Convert.ToInt32(pos["gun_air"]) : (int?)null;
                    kV = pos["kV"].ToString() != "" ? Convert.ToInt32(pos["kV"]) : (int?)null;
                    microampere = pos["microampere"].ToString() != "" ? Convert.ToInt32(pos["microampere"]) : (int?)null;
                    status = pos["status"].ToString() != "" ? Convert.ToInt32(pos["status"]) : (int?)null;
                    application = pos["application"].ToString();

                    // Costruisco oggetto gunSettings da inserire nelle posizioni
                    GunSettings gunSettings = new GunSettings
                        (
                            guid_gun_settings,
                            guid_pos,
                            id_gun_settings,
                            feed_air,
                            dosage_air,
                            gun_air,
                            kV,
                            microampere,
                            status,
                            application
                        );

                    positions.Add(new ApplicationPositions(
                        pos["guid_pos"].ToString(),
                        Convert.ToInt16(pos["id_pos"]),
                        float.Parse(pos["x"].ToString()),
                        float.Parse(pos["y"].ToString()),
                        float.Parse(pos["z"].ToString()),
                        float.Parse(pos["rx"].ToString()),
                        float.Parse(pos["ry"].ToString()),
                        float.Parse(pos["rz"].ToString()),
                        pos["positionName"].ToString(),
                        gunSettings
                        ));
                }

                applicationsManager.getDictionary().Add(applicationName,
                    new RobotApplication(id, positions));
            }
        }

        #endregion
    }
}

