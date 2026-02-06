using fairino;
using RMLib.Logger;
using System;
using System.Data;
using System.Data.SQLite;

namespace RM.src.RM250714
{
    /// <summary>
    /// Contiene procedure relative al funzionamento del Robot
    /// </summary>
    public class RobotDAOSqlite
    {
        #region Parametri di RobotDAOSqlite
        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        #region Variabili tabella DB application_positions
        
        public const String APPLICATION_POSITIONS_TABLE_NAME = "application_positions";
        public const String APPLICATION_POSITIONS_GUID_COLUMN_NAME = "guid";
        public const String APPLICATION_POSITIONS_TABLE_APPLICATION_NAME = "Application";
        public const String APPLICATION_POSITIONS_ID_COLUMN_NAME = "id";
        public const String APPLICATION_POSITIONS_X_COLUMN_NAME = "x";
        public const String APPLICATION_POSITIONS_Y_COLUMN_NAME = "y";
        public const String APPLICATION_POSITIONS_Z_COLUMN_NAME = "z";
        public const String APPLICATION_POSITIONS_RX_COLUMN_NAME = "rx";
        public const String APPLICATION_POSITIONS_RY_COLUMN_NAME = "ry";
        public const String APPLICATION_POSITIONS_RZ_COLUMN_NAME = "rz";
        public const String APPLICATION_POSITIONS_SAMPLETIME_COLUMN_NAME = "SampleTime";
        public const String APPLICATION_POSITIONS_MODE_COLUMN_NAME = "Mode";
        public const String APPLICATION_POSITIONS_POSITIONNAME_COLUMN_NAME = "positionName";

        #endregion

        #region Variabili tabella DB applications
        public const String APPLICATIONS_TABLE_NAME = "applications";
        public const String APPLICATIONS_TABLE_APPLICATION_NAME = "ApplicationName";
        public const String APPLICATIONS_TABLE_CREATION_TIME = "creation_time";
        public const String APPLICATIONS_TABLE_LAST_UPDATE = "last_update_time";
        #endregion

        #region Variabili tabella DB error_codes
        public const String ERROR_CODES_TABLE_NAME = "error_codes";
        public const String ERROR_CODES_ID_MAINCODE_COLUMN_NAME = "id_MainCode";
        public const String ERROR_CODES_ID_SUBCODE_COLUMN_NAME = "id_SubCode";
        public const String ERROR_CODES_DESCR_MAINCODE_COLUMN_NAME = "descr_MainCode";
        public const String ERROR_CODES_DESCR_SUBCODE_COLUMN_NAME = "descr_SubCode";
        #endregion

        #region Variabili tabella DB alarm_history
        public const String ALARM_HISTORY_TABLE_NAME = "alarm_history";
        public const String ALARM_HISTORY_ID_COLUMN_NAME = "id";
        public const String ALARM_HISTORY_DESCRIPTION_COLUMN_NAME = "description";
        public const String ALARM_HISTORY_TIMESTAMP_COLUMN_NAME = "timestamp";
        public const String ALARM_HISTORY_DEVICE_COLUMN_NAME = "device";
        public const String ALARM_HISTORY_STATE_COLUMN_NAME = "state";
        #endregion

        #region Variabili tabella DB robot_properties
        public const String ROBOT_PROPERTIES_TABLE_NAME = "robot_properties";
        public const String ROBOT_PROPERTIES_VALUE_COLUMN_NAME = "value";
        #endregion

        #region Variabili tabella robot_properties
        public const int ROBOT_PROPERTIES_SPEED_ID = 1;
        public const int ROBOT_PROPERTIES_VELOCITY_ID = 2;
        public const int ROBOT_PROPERTIES_BLEND_ID = 3;
        public const int ROBOT_PROPERTIES_ACCELERATION_ID = 4;
        public const int ROBOT_PROPERTIES_OVL_ID = 5;
        public const int ROBOT_PROPERTIES_TOOL_ID = 6;
        public const int ROBOT_PROPERTIES_USER_ID = 7;
        public const int ROBOT_PROPERTIES_WEIGHT_ID = 8;
        public const int ROBOT_PROPERTIES_VELREC_ID = 9;
        #endregion

        #region Indici tabella robot_properties
        public const int ROBOT_PROPERTIES_SPEED_ROW_INDEX = 0;
        public const int ROBOT_PROPERTIES_VELOCITY_ROW_INDEX = 1;
        public const int ROBOT_PROPERTIES_BLENDT_ROW_INDEX = 2;
        public const int ROBOT_PROPERTIES_ACCELERATION_ROW_INDEX = 3;
        public const int ROBOT_PROPERTIES_OVL_ROW_INDEX = 4;
        public const int ROBOT_PROPERTIES_TOOL_ROW_INDEX = 5;
        public const int ROBOT_PROPERTIES_USER_ROW_INDEX = 6;
        public const int ROBOT_PROPERTIES_WEIGHT_ROW_INDEX = 7;
        public const int ROBOT_PROPERTIES_VELREC_ROW_INDEX = 8;
        public const int ROBOT_PROPERTIES_COLLISION_LEVELS_ROW_INDEX = 9;
        public const int ROBOT_PROPERTIES_BLENDR_ROW_INDEX = 10;
        public const int ROBOT_PROPERTIES_COLLISION_LEVELS_SERVICE_ROW_INDEX = 11;

        public const int ROBOT_PROPERTIES_VALUE_COLUMN_INDEX = 2;
        #endregion


        #region Variabili tabella robot_movements_codes

        public const String ROBOT_MOVEMENTS_CODES_TABLE_NAME = "robot_movements_codes";
        public const String ROBOT_MOVEMENTS_CODES_ID_COLUMN_NAME = "Errcode";

        #endregion

        #region Variabili tabella gun_settings

        public const String GUN_SETTINGS_TABLE_NAME = "gun_settings";
        public const String GUN_SETTINGS_GUID_COLUMN_NAME = "guid";
        public const String GUN_SETTINGS_GUID_ELE_COLUMN_NAME = "guid_ele";
        public const String GUN_SETTINGS_ID_COLUMN_NAME = "id";
        public const String GUN_SETTINGS_FEED_AIR_COLUMN_NAME = "feed_air";
        public const String GUN_SETTINGS_DOSAGE_AIR_COLUMN_NAME = "dosage_air";
        public const String GUN_SETTINGS_GUN_AIR_COLUMN_NAME = "gun_air";
        public const String GUN_SETTINGS_KV_COLUMN_NAME = "kV";
        public const String GUN_SETTINGS_MICROAMPERE_COLUMN_NAME = "microampere";
        public const String GUN_SETTINGS_STATUS_COLUMN_NAME = "status";
        public const String GUN_SETTINGS_APPLICATION_COLUMN_NAME = "application";

        #endregion

        #region Variabili tabella robot_frames

        public const string ROBOT_FRAMES_TABLE_NAME = "robot_frames";
        public const string ROBOT_FRAMES_ID_COLUMN_NAME = "id";
        public const string ROBOT_FRAMES_NAME_COLUMN_NAME = "name";
        public const string ROBOT_FRAMES_X_COLUMN_NAME = "x";
        public const string ROBOT_FRAMES_Y_COLUMN_NAME = "y";
        public const string ROBOT_FRAMES_Z_COLUMN_NAME = "z";
        public const string ROBOT_FRAMES_RX_COLUMN_NAME = "rx";
        public const string ROBOT_FRAMES_RY_COLUMN_NAME = "ry";
        public const string ROBOT_FRAMES_RZ_COLUMN_NAME = "rz";

        #endregion

        #region Variabili tabella robot_tools

        public const string ROBOT_TOOLS_TABLE_NAME = "robot_tools";
        public const string ROBOT_TOOLS_ID_COLUMN_NAME = "id";
        public const string ROBOT_TOOLS_NAME_COLUMN_NAME = "name";
        public const string ROBOT_TOOLS_X_COLUMN_NAME = "x";
        public const string ROBOT_TOOLS_Y_COLUMN_NAME = "y";
        public const string ROBOT_TOOLS_Z_COLUMN_NAME = "z";
        public const string ROBOT_TOOLS_RX_COLUMN_NAME = "rx";
        public const string ROBOT_TOOLS_RY_COLUMN_NAME = "ry";
        public const string ROBOT_TOOLS_RZ_COLUMN_NAME = "rz";
        public const string ROBOT_TOOLS_TYPE_COLUMN_NAME = "type";
        public const string ROBOT_TOOLS_INSTALL_COLUMN_NAME = "install";

        #endregion

        #region Variabili tabella robot_collision_levels

        public const string ROBOT_COLLISION_LEVELS_TABLE_NAME = "robot_collision_levels";
        public const string ROBOT_COLLISION_LEVELS_ID_COLUMN_NAME = "id";
        public const string ROBOT_COLLISION_LEVELS_MODE_COLUMN_NAME = "mode";
        public const string ROBOT_COLLISION_LEVELS_J1_COLUMN_NAME = "j1";
        public const string ROBOT_COLLISION_LEVELS_J2_COLUMN_NAME = "j2";
        public const string ROBOT_COLLISION_LEVELS_J3_COLUMN_NAME = "j3";
        public const string ROBOT_COLLISION_LEVELS_J4_COLUMN_NAME = "j4";
        public const string ROBOT_COLLISION_LEVELS_J5_COLUMN_NAME = "j5";
        public const string ROBOT_COLLISION_LEVELS_J6_COLUMN_NAME = "j6";
        public const string ROBOT_COLLISION_LEVELS_CONFIG_COLUMN_NAME = "config";

        #endregion

        #endregion

        #region Metodi di RobotDAOSqlite

        /// <summary>
        /// Metodo che restituisce le posizioni di una determinata applicazione
        /// </summary>
        /// <param name="connectionString">Stringa di connessione</param>
        /// <param name="application">Nome applicazione</param>
        /// <returns></returns>
        public DataTable GetPointsPosition(String connectionString, string application)
        {
            // Dichiaro DataTable da restituire
            var dt_points = new DataTable();

            try
            {
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    string query =
                                    "SELECT " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.guid as guid_pos, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_ID_COLUMN_NAME} as id_pos, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_POSITIONNAME_COLUMN_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_X_COLUMN_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_Y_COLUMN_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_Z_COLUMN_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_RX_COLUMN_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_RY_COLUMN_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_RZ_COLUMN_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_SAMPLETIME_COLUMN_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_TABLE_APPLICATION_NAME}, " +
                                        $"{APPLICATION_POSITIONS_TABLE_NAME}.{APPLICATION_POSITIONS_MODE_COLUMN_NAME}, " +
                                        $"{GUN_SETTINGS_TABLE_NAME}.{GUN_SETTINGS_GUID_COLUMN_NAME} as guid_gun_settings, " +
                                        $"{GUN_SETTINGS_TABLE_NAME}.{GUN_SETTINGS_ID_COLUMN_NAME} as id_gun_settings, " +
                                        $"{GUN_SETTINGS_TABLE_NAME}.{GUN_SETTINGS_FEED_AIR_COLUMN_NAME}, " +
                                        $"{GUN_SETTINGS_TABLE_NAME}.{GUN_SETTINGS_DOSAGE_AIR_COLUMN_NAME}, " +
                                        $"{GUN_SETTINGS_TABLE_NAME}.{GUN_SETTINGS_GUN_AIR_COLUMN_NAME}, " +
                                        $"{GUN_SETTINGS_TABLE_NAME}.{GUN_SETTINGS_KV_COLUMN_NAME}, " +
                                        $"{GUN_SETTINGS_TABLE_NAME}.{GUN_SETTINGS_MICROAMPERE_COLUMN_NAME}, " +
                                        $"{GUN_SETTINGS_TABLE_NAME}.{GUN_SETTINGS_STATUS_COLUMN_NAME} " +

                                    "FROM " + APPLICATION_POSITIONS_TABLE_NAME + " " +

                                    "LEFT JOIN " + GUN_SETTINGS_TABLE_NAME + " " +
                                    "ON " + GUN_SETTINGS_TABLE_NAME + "." + GUN_SETTINGS_GUID_ELE_COLUMN_NAME + " = " + APPLICATION_POSITIONS_TABLE_NAME + ".guid " +

                                    "WHERE " + APPLICATION_POSITIONS_TABLE_NAME + "." + APPLICATION_POSITIONS_TABLE_APPLICATION_NAME + " = '" + application + "'";

                    using (var cmd = new SQLiteCommand(query, con))
                    {
                        using (var reader = cmd.ExecuteReader())
                        {
                            dt_points.Load(reader);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante la procedura GetPointsPosition: " + ex.ToString());
            }

            return dt_points;
        }

        /// <summary>
        /// Restituisce codice movimento robot
        /// </summary>
        /// <param name="connectionString">Stringa di connessione al database</param>
        /// <param name="id">id del movimento robot</param>
        /// <returns></returns>
        public DataRow GetRobotMovementCode(string connectionString, int id)
        {
            DataTable robot_movement_code = new DataTable();
            DataRow code = null;

            try
            {
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    string query = "SELECT * " +
                        "FROM " + ROBOT_MOVEMENTS_CODES_TABLE_NAME +
                        " WHERE [" + ROBOT_MOVEMENTS_CODES_ID_COLUMN_NAME + "] = " + id;

                    using (var cmd = new SQLiteCommand(query, con))
                    {
                        using (var reader = cmd.ExecuteReader())
                        {
                            robot_movement_code.Load(reader);
                            if(robot_movement_code.Rows.Count > 0) 
                                code = robot_movement_code.Rows[0];
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante la procedura GetPointsPosition: " + ex.ToString());
            }

            return code;
        }

        /// <summary>
        /// Metodo che restituisce DataRow contenente allarme del Robot
        /// </summary>
        /// <param name="connectionString"></param>
        /// <param name="mainCode"></param>
        /// <param name="subCode"></param>
        /// <returns></returns>
        public DataRow GetRobotAlarm(String connectionString, int mainCode, int subCode)
        {
            using (SQLiteConnection connection = new SQLiteConnection(connectionString))
            {
                connection.Open();

                string query = "SELECT * FROM " + ERROR_CODES_TABLE_NAME +
                    " WHERE " + ERROR_CODES_ID_MAINCODE_COLUMN_NAME + " = " + mainCode +
                    " AND " + ERROR_CODES_ID_SUBCODE_COLUMN_NAME + " = " + subCode;
                ; // La tua query
                using (SQLiteCommand command = new SQLiteCommand(query, connection))
                {
                    using (SQLiteDataAdapter adapter = new SQLiteDataAdapter(command))
                    {
                        DataTable dataTable = new DataTable();
                        adapter.Fill(dataTable);

                        if (dataTable.Rows.Count > 0)
                        {
                            return dataTable.Rows[0];
                        }
                        else
                        {
                            return null;
                        }
                    }
                }


            }
        }

        /// <summary>
        /// Aggiorna le coordinate di un punto
        /// </summary>
        /// <param name="connectionString"></param>
        /// <param name="pos"></param>
        /// <param name="formattedTimeStamp"></param>
        /// <param name="id"></param>
        /// <param name="positionName"></param>
        public void UpdatePositionCoord(string connectionString, DescPose pos, string formattedTimeStamp, int id, string positionName, string application)
        {
            //log.Info($"Avvio del metodo SaveUpdatedPositionToDatabase per ID: {UC_DragMode_Debug.idPositionUpdated + 1}");

            using (var con = new SQLiteConnection(connectionString))
            {
                con.Open();
                log.Info("Connessione al database aperta.");

                string query = "UPDATE " + APPLICATION_POSITIONS_TABLE_NAME +
                               " SET x = @x, y = @y, z = @z, rx = @rx, ry = @ry, rz = @rz, " +
                               " sampleTime = @sampleTime  WHERE id = @id AND application = @application";

                using (var cmd = new SQLiteCommand(query, con))
                {
                    cmd.Parameters.AddWithValue("@id", id);
                    cmd.Parameters.AddWithValue("@sampleTime", formattedTimeStamp);
                    cmd.Parameters.AddWithValue("@x", pos.tran.x);
                    cmd.Parameters.AddWithValue("@y", pos.tran.y);
                    cmd.Parameters.AddWithValue("@z", pos.tran.z);
                    cmd.Parameters.AddWithValue("@rx", pos.rpy.rx);
                    cmd.Parameters.AddWithValue("@ry", pos.rpy.ry);
                    cmd.Parameters.AddWithValue("@rz", pos.rpy.rz);
                    cmd.Parameters.AddWithValue("@application", application);

                    cmd.ExecuteNonQuery();
                    log.Info("Query di aggiornamento eseguita.");
                }

                ApplicationConfig.applicationsManager.UpdatePosition(positionName, pos, application);
                con.Close();
                log.Info("Connessione al database chiusa.");
            }

            log.Info("Fine del metodo UpdatePositionCoord.");
        }

        /// <summary>
        /// Salva sul db l'allarme del robot
        /// </summary>
        /// <param name="connectionString"></param>
        /// <param name="id"></param>
        /// <param name="description"></param>
        /// <param name="timpestamp"></param>
        /// <param name="device"></param>
        /// <param name="state"></param>
        public void SaveRobotAlarm(String connectionString, int id, string description, string timpestamp, string device, string state)
        {
            try
            {
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    // Aggiungo il nuovo utente
                    string insertQuery = "INSERT INTO " + ALARM_HISTORY_TABLE_NAME + "" +
                        " (id, description, timestamp, device, state) " +
                        "VALUES (@id, @Description, @Timestamp, @Device, @State)";

                    using (var cmd = new SQLiteCommand(insertQuery, con))
                    {
                        cmd.Parameters.AddWithValue("@id", id);
                        cmd.Parameters.AddWithValue("@Description", description);
                        cmd.Parameters.AddWithValue("@Timestamp", timpestamp);
                        cmd.Parameters.AddWithValue("@Device", device);
                        cmd.Parameters.AddWithValue("@State", state);

                        cmd.ExecuteNonQuery();
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante l'aggiunta dell'azione: " + ex.ToString());
            }
        }

        /// <summary>
        /// Metodo che aggiunge routine su database e aggiorna dizionario delle posizioni
        /// </summary>
        /// <param name="connectionString">Stringa di connessione</param>
        /// <param name="position">Nome nuova posizione</param>
        /// <param name="date">data</param>
        /// <param name="pointIndex">indice del punto</param>
        /// <param name="applicationName">nome dell'applicazione scelta</param>
        public void AddNewRobotPosition(String connectionString, string position, string date, ref int pointIndex, string applicationName)
        {
            log.Info($"Avvio del metodo AddNewRobotRoutine");

            try
            {
                // Generazione guid della posizione
                string guid_pos = Guid.NewGuid().ToString();

                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    // Aggiungo il nuovo utente
                    string insertQuery = "INSERT INTO " + APPLICATION_POSITIONS_TABLE_NAME + " (guid, id, sampleTime, Application, Mode, x, y, z, rx, ry, rz, positionName) " +
                        "VALUES (@guid_pos, @id, @SampleTime, @Application, @Mode, @x, @y, @z, @rx, @ry, @rz, @positionName)";

                    using (var cmd = new SQLiteCommand(insertQuery, con))
                    {
                        cmd.Parameters.AddWithValue("@guid_pos", guid_pos);
                        cmd.Parameters.AddWithValue("@id", pointIndex);
                        cmd.Parameters.AddWithValue("@SampleTime", date);
                        cmd.Parameters.AddWithValue("@Application", applicationName);
                        cmd.Parameters.AddWithValue("@Mode", "");
                        cmd.Parameters.AddWithValue("@x", 0);
                        cmd.Parameters.AddWithValue("@y", 0);
                        cmd.Parameters.AddWithValue("@z", 0);
                        cmd.Parameters.AddWithValue("@rx", 0);
                        cmd.Parameters.AddWithValue("@ry", 0);
                        cmd.Parameters.AddWithValue("@rz", 0);
                        cmd.Parameters.AddWithValue("@positionName", position);
                        cmd.ExecuteNonQuery();
                    }

                    ApplicationConfig.applicationsManager.AddApplicationRobotPosition(guid_pos, new DescPose(), applicationName, pointIndex, position);
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante l'aggiunta dell'azione: " + ex.ToString());
            }
            log.Info("Fine del metodo AddNewRobotRoutine.");
        }

        /// <summary>
        /// Metodo che restituisce lista applicazioni
        /// </summary>
        /// <param name="connectionString"></param>
        /// <returns></returns>
        public DataTable GetRobotApplications(String connectionString)
        {
            // Dichiaro DataTable da restituire
            var dt_applications = new DataTable();

            try
            {
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    string query = "SELECT *" +
                        "FROM " + APPLICATIONS_TABLE_NAME;

                    using (var cmd = new SQLiteCommand(query, con))
                    {
                        using (var reader = cmd.ExecuteReader())
                        {
                            dt_applications.Load(reader);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante la query GetRobotApplications: " + ex.ToString());
            }

            return dt_applications;
        }

        /// <summary>
        /// Get della tabella robot_properties
        /// </summary>
        /// <param name="connectionString">Stringa di connessione</param>
        /// <returns></returns>
        public DataTable GetRobotProperties(String connectionString)
        {
            // Dichiaro DataTable da restituire
            var dt_robot_properties = new DataTable();

            try
            {
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    string query = "SELECT *" +
                        "FROM " + ROBOT_PROPERTIES_TABLE_NAME;

                    using (var cmd = new SQLiteCommand(query, con))
                    {
                        using (var reader = cmd.ExecuteReader())
                        {
                            dt_robot_properties.Load(reader);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante la query GetRobotProperties: " + ex.ToString());
            }

            return dt_robot_properties;
        }

        /// <summary>
        /// Aggiorna la velocità del robot nel database.
        /// </summary>
        /// <param name="connectionString">La stringa di connessione al database SQLite.</param>
        /// <param name="velRec">La velocità corrente del robot da impostare nel database.</param>
        public void SetRobotVelRec(string connectionString, int velRec)
        {
            try
            {
                log.Info("Inizio dell'aggiornamento della velocità di campionamento delle posizioni del robot nel database.");

                // Utilizza una connessione usando la stringa di connessione fornita
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();
                    log.Info("Connessione al database aperta con successo.");

                    // Query di aggiornamento per modificare il campo VelRec nella tabella properties
                    string sqlUpdate = $"UPDATE {ROBOT_PROPERTIES_TABLE_NAME} " +
                                       $"SET {ROBOT_PROPERTIES_VALUE_COLUMN_NAME} = @VelRec " +
                                       $"WHERE Id = @Id";

                    using (var cmd = new SQLiteCommand(sqlUpdate, con))
                    {
                        // Aggiungi i parametri alla query
                        cmd.Parameters.AddWithValue("@VelRec", velRec);
                        cmd.Parameters.AddWithValue("@Id", ROBOT_PROPERTIES_VELREC_ID); // Supponendo che l'Id sia noto e fisso

                        // Esegui la query di aggiornamento
                        int rowsAffected = cmd.ExecuteNonQuery();
                        log.Info($"{rowsAffected} riga/e aggiornata/e nel database con velocità: {velRec}");

                        // Modifico il valore dentro robot manager
                        RobotManager.velRec = Convert.ToInt32(velRec);
                        RobotManager.robotProperties.VelRec = velRec;
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante SetRobotVelRec: " + ex.ToString());
            }
        }

        /// <summary>
        /// Esegue rinomina della posizione sul database
        /// </summary>
        /// <param name="connectionString">Stringa di connessione</param>
        /// <param name="name">Nome della posizione</param>
        /// <param name="newName">Nuovo nome della posizione</param>
        /// <param name="date">Data modifica</param>
        public void UpdateRobotPositionName(String connectionString, string name, string newName, DateTime date)
        {
            try
            {
                log.Info("SQLite: query UpdateRobotPositionName");

                string formattedTimeToDB = date.ToString("yyyy-MM-dd HH:mm:ss.fffffff"); // Formattazione per DB
                string formattedTimeToLW = date.ToString("HH:mm:ss:ff"); // Formattazione per LW

                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    string sqlPLC = "UPDATE " + APPLICATION_POSITIONS_TABLE_NAME +
                                    " SET PositionName = '" + newName +
                                    "' WHERE " + APPLICATION_POSITIONS_POSITIONNAME_COLUMN_NAME + " = @PositionName";

                    using (var cmd = new SQLiteCommand(sqlPLC, con))
                    {
                        cmd.Parameters.AddWithValue("@PositionName", name);
                        cmd.ExecuteNonQuery();
                    }

                    sqlPLC = "UPDATE " + APPLICATION_POSITIONS_TABLE_NAME +
                                    " SET SampleTime = '" + formattedTimeToDB +
                                    "' WHERE " + APPLICATION_POSITIONS_POSITIONNAME_COLUMN_NAME + " = @PositionName";

                    using (var cmd = new SQLiteCommand(sqlPLC, con))
                    {
                        cmd.Parameters.AddWithValue("@PositionName", newName);
                        cmd.ExecuteNonQuery();
                    }

                }

                ApplicationConfig.applicationsManager.UpdateRobotPositionName(newName, formattedTimeToLW);
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante la modifica del nome della posizione: " + ex.ToString());
            }
        }

        /// <summary>
        /// Cancella una posizione del Robot
        /// </summary>
        /// <param name="connectionString">Stringa di connessione</param>
        /// <param name="positionName">Nome della posizione</param>
        /// <param name="idPosition">Id del punto</param>
        public void DeleteRobotPosition(String connectionString, string positionName, int idPosition)
        {
            try
            {
                log.Info("SQLite: query DeleteRobotPosition");
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    // Elimino le posizioni dell'applicazione selezionata dal database
                    string sqlPLC = "DELETE FROM " + APPLICATION_POSITIONS_TABLE_NAME +
                                    " WHERE " + APPLICATION_POSITIONS_POSITIONNAME_COLUMN_NAME + " = @PositionName";

                    // Assegnazione parametri per la query
                    using (var cmd = new SQLiteCommand(sqlPLC, con))
                    {
                        cmd.Parameters.AddWithValue("@PositionName", positionName);

                        // Esecuzione query
                        cmd.ExecuteNonQuery();
                    }

                    ApplicationConfig.applicationsManager.RemovePositionFromApplication(idPosition, "RM");
                }

            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante l'eliminazione della ricetta: " + ex.ToString());
            }
        }

        /// <summary>
        /// Aggiorna il blend del robot nel database.
        /// </summary>
        /// <param name="connectionString">La stringa di connessione al database SQLite.</param>
        /// <param name="blend">Il blend corrente del robot da impostare nel database.</param>
        public void SetRobotBlend(string connectionString, int blend)
        {
            try
            {
                log.Info("Inizio dell'aggiornamento del blend del robot nel database.");

                // Utilizza una connessione usando la stringa di connessione fornita
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();
                    log.Info("Connessione al database aperta con successo.");

                    // Query di aggiornamento per modificare il campo Blend nella tabella properties
                    string sqlUpdate = $"UPDATE {ROBOT_PROPERTIES_TABLE_NAME} " +
                                       $"SET {ROBOT_PROPERTIES_VALUE_COLUMN_NAME} = @Blend " +
                                       $"WHERE Id = @Id";

                    using (var cmd = new SQLiteCommand(sqlUpdate, con))
                    {
                        // Aggiungi i parametri alla query
                        cmd.Parameters.AddWithValue("@Blend", blend);
                        cmd.Parameters.AddWithValue("@Id", ROBOT_PROPERTIES_BLEND_ID); // Supponendo che l'Id sia noto e fisso

                        // Esegui la query di aggiornamento
                        int rowsAffected = cmd.ExecuteNonQuery();
                        log.Info($"{rowsAffected} riga/e aggiornata/e nel database con blend: {blend}");

                        // Modifico proprietà blend dell'oggetto robotProperties
                        RobotManager.robotProperties.Blend = blend;

                        // Modifico la variabile globale
                        RobotManager.blendT = blend;
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante SetRobotBlend: " + ex.ToString());
            }
        }

        /// <summary>
        /// Aggiorna la velocity del robot nel database.
        /// </summary>
        /// <param name="connectionString">La stringa di connessione al database SQLite.</param>
        /// <param name="velocity">La velocity corrente del robot da impostare nel database.</param>
        public void SetRobotVelocity(string connectionString, int velocity)
        {
            try
            {
                // Utilizza una connessione usando la stringa di connessione fornita
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();
                    log.Info("Connessione al database aperta con successo.");

                    // Query di aggiornamento per modificare il campo Velocity nella tabella properties
                    string sqlUpdate = $"UPDATE {ROBOT_PROPERTIES_TABLE_NAME} " +
                                       $"SET {ROBOT_PROPERTIES_VALUE_COLUMN_NAME} = @Velocity " +
                                       $"WHERE Id = @Id";

                    using (var cmd = new SQLiteCommand(sqlUpdate, con))
                    {
                        // Aggiungi i parametri alla query
                        cmd.Parameters.AddWithValue("@Velocity", velocity);
                        cmd.Parameters.AddWithValue("@Id", ROBOT_PROPERTIES_VELOCITY_ID); // Supponendo che l'Id sia noto e fisso

                        // Esegui la query di aggiornamento
                        int rowsAffected = cmd.ExecuteNonQuery();
                        log.Info($"{rowsAffected} riga/e aggiornata/e nel database con blend: {velocity}");

                        // Modifico proprietà velocity dell'oggetto robotProperties
                        RobotManager.robotProperties.Velocity = velocity;

                        // Modifico la variabile globale
                        RobotManager.vel = velocity;
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante SetRobotVelocity: " + ex.ToString());
            }
        }

        /// <summary>
        /// Aggiorna la acceleration del robot nel database.
        /// </summary>
        /// <param name="connectionString">La stringa di connessione al database SQLite.</param>
        /// <param name="acceleration">La acceleration corrente del robot da impostare nel database.</param>
        public void SetRobotAcceleration(string connectionString, int acceleration)
        {
            try
            {
                log.Info("Inizio dell'aggiornamento della acceleration del robot nel database.");

                // Utilizza una connessione usando la stringa di connessione fornita
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();
                    log.Info("Connessione al database aperta con successo.");

                    // Query di aggiornamento per modificare il campo acceleration nella tabella properties
                    string sqlUpdate = $"UPDATE {ROBOT_PROPERTIES_TABLE_NAME} " +
                                       $"SET {ROBOT_PROPERTIES_VALUE_COLUMN_NAME} = @Acceleration " +
                                       $"WHERE Id = @Id";

                    using (var cmd = new SQLiteCommand(sqlUpdate, con))
                    {
                        // Aggiungi i parametri alla query
                        cmd.Parameters.AddWithValue("@Acceleration", acceleration);
                        cmd.Parameters.AddWithValue("@Id", ROBOT_PROPERTIES_ACCELERATION_ID); // Supponendo che l'Id sia noto e fisso

                        // Esegui la query di aggiornamento
                        int rowsAffected = cmd.ExecuteNonQuery();
                        log.Info($"{rowsAffected} riga/e aggiornata/e nel database con acceleration: {acceleration}");

                        // Modifico proprietà acceleration dell'oggetto robotProperties
                        RobotManager.robotProperties.Acceleration = acceleration;

                        // Modifico la variabile globale
                        RobotManager.acc = acceleration;
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante SetRobotAcceleration: " + ex.ToString());
            }
        }

        /// <summary>
        /// Aggiorna la ovl del robot nel database.
        /// </summary>
        /// <param name="connectionString">La stringa di connessione al database SQLite.</param>
        /// <param name="ovl">La ovl corrente del robot da impostare nel database.</param>
        public void SetRobotOvl(string connectionString, int ovl)
        {
            try
            {
                log.Info("Inizio dell'aggiornamento della ovl del robot nel database.");

                // Utilizza una connessione usando la stringa di connessione fornita
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();
                    log.Info("Connessione al database aperta con successo.");

                    // Query di aggiornamento per modificare il campo acceleration nella tabella properties
                    string sqlUpdate = $"UPDATE {ROBOT_PROPERTIES_TABLE_NAME} " +
                                       $"SET {ROBOT_PROPERTIES_VALUE_COLUMN_NAME} = @Ovl " +
                                       $"WHERE Id = @Id";

                    using (var cmd = new SQLiteCommand(sqlUpdate, con))
                    {
                        // Aggiungi i parametri alla query
                        cmd.Parameters.AddWithValue("@Ovl", ovl);
                        cmd.Parameters.AddWithValue("@Id", ROBOT_PROPERTIES_OVL_ID); // Supponendo che l'Id sia noto e fisso

                        // Esegui la query di aggiornamento
                        int rowsAffected = cmd.ExecuteNonQuery();
                        log.Info($"{rowsAffected} riga/e aggiornata/e nel database con ovl: {ovl}");

                        // Modifico proprietà ovl dell'oggetto robotProperties
                        RobotManager.robotProperties.Ovl = ovl;

                        // Modifico la variabile globale
                        RobotManager.ovl = ovl;
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante SetRobotOvl: " + ex.ToString());
            }
        }

        /// <summary>
        /// Aggiorna il tool del robot nel database.
        /// </summary>
        /// <param name="connectionString">La stringa di connessione al database SQLite.</param>
        /// <param name="tool">Il tool corrente del robot da impostare nel database.</param>
        public void SetRobotTool(string connectionString, int tool)
        {
            try
            {
                log.Info("Inizio dell'aggiornamento del tool del robot nel database.");

                // Utilizza una connessione usando la stringa di connessione fornita
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();
                    log.Info("Connessione al database aperta con successo.");

                    // Query di aggiornamento per modificare il campo tool nella tabella properties
                    string sqlUpdate = $"UPDATE {ROBOT_PROPERTIES_TABLE_NAME} " +
                                       $"SET {ROBOT_PROPERTIES_VALUE_COLUMN_NAME} = @Tool " +
                                       $"WHERE Id = @Id";

                    using (var cmd = new SQLiteCommand(sqlUpdate, con))
                    {
                        // Aggiungi i parametri alla query
                        cmd.Parameters.AddWithValue("@Tool", tool);
                        cmd.Parameters.AddWithValue("@Id", ROBOT_PROPERTIES_TOOL_ID); // Supponendo che l'Id sia noto e fisso

                        // Esegui la query di aggiornamento
                        int rowsAffected = cmd.ExecuteNonQuery();
                        log.Info($"{rowsAffected} riga/e aggiornata/e nel database con tool: {tool}");

                        // Modifico proprietà tool dell'oggetto robotProperties
                        RobotManager.robotProperties.Tool = tool;

                        // Modifico la variabile globale
                        RobotManager.tool = tool;
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante SetRobotTool: " + ex.ToString());
            }
        }

        /// <summary>
        /// Aggiorna lo user del robot nel database.
        /// </summary>
        /// <param name="connectionString">La stringa di connessione al database SQLite.</param>
        /// <param name="user">User corrente del robot da impostare nel database.</param>
        public void SetRobotUser(string connectionString, int user)
        {
            try
            {
                log.Info("Inizio dell'aggiornamento dello user del robot nel database.");

                // Utilizza una connessione usando la stringa di connessione fornita
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();
                    log.Info("Connessione al database aperta con successo.");

                    // Query di aggiornamento per modificare il campo user nella tabella properties
                    string sqlUpdate = $"UPDATE {ROBOT_PROPERTIES_TABLE_NAME} " +
                                       $"SET {ROBOT_PROPERTIES_VALUE_COLUMN_NAME} = @User " +
                                       $"WHERE Id = @Id";

                    using (var cmd = new SQLiteCommand(sqlUpdate, con))
                    {
                        // Aggiungi i parametri alla query
                        cmd.Parameters.AddWithValue("@User", user);
                        cmd.Parameters.AddWithValue("@Id", ROBOT_PROPERTIES_USER_ID); // Supponendo che l'Id sia noto e fisso

                        // Esegui la query di aggiornamento
                        int rowsAffected = cmd.ExecuteNonQuery();
                        log.Info($"{rowsAffected} riga/e aggiornata/e nel database con user: {user}");

                        // Modifico proprietà user dell'oggetto robotProperties
                        RobotManager.robotProperties.User = user;

                        // Modifico la variabile globale
                        RobotManager.user = user;
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante SetRobotUser: " + ex.ToString());
            }
        }

        /// <summary>
        /// Aggiorna il carico del robot nel database.
        /// </summary>
        /// <param name="connectionString">La stringa di connessione al database SQLite.</param>
        /// <param name="weight">Carico corrente del robot da impostare nel database.</param>
        public void SetRobotWeight(string connectionString, int weight)
        {
            try
            {
                log.Info("Inizio dell'aggiornamento del carico del robot nel database.");

                // Utilizza una connessione usando la stringa di connessione fornita
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();
                    log.Info("Connessione al database aperta con successo.");

                    // Query di aggiornamento per modificare il campo user nella tabella properties
                    string sqlUpdate = $"UPDATE {ROBOT_PROPERTIES_TABLE_NAME} " +
                                       $"SET {ROBOT_PROPERTIES_VALUE_COLUMN_NAME} = @Weight " +
                                       $"WHERE Id = @Id";

                    using (var cmd = new SQLiteCommand(sqlUpdate, con))
                    {
                        // Aggiungi i parametri alla query
                        cmd.Parameters.AddWithValue("@Weight", weight);
                        cmd.Parameters.AddWithValue("@Id", ROBOT_PROPERTIES_WEIGHT_ID); // Supponendo che l'Id sia noto e fisso

                        // Esegui la query di aggiornamento
                        int rowsAffected = cmd.ExecuteNonQuery();
                        log.Info($"{rowsAffected} riga/e aggiornata/e nel database con user: {weight}");

                        // Modifico proprietà user dell'oggetto robotProperties
                        RobotManager.robotProperties.Weight = weight;

                        // Modifico la variabile globale
                        RobotManager.weight = weight;
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante SetRobotUser: " + ex.ToString());
            }
        }

        #region Metodi gestione robot frames

        /// <summary>
        /// Recupera dal database tutti i frame salvati
        /// </summary>
        /// <param name="connectionString"></param>
        /// <returns></returns>
        public DataTable GetRobotFrames(String connectionString)
        {
            // Dichiaro DataTable da restituire
            var dt_robot_frames = new DataTable();

            try
            {
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    string query = "SELECT *" +
                        "FROM " + ROBOT_FRAMES_TABLE_NAME;

                    using (var cmd = new SQLiteCommand(query, con))
                    {
                        using (var reader = cmd.ExecuteReader())
                        {
                            dt_robot_frames.Load(reader);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante la query GetRobotFrames: " + ex.ToString());
            }

            return dt_robot_frames;
        }

        #endregion

        #region Metodi gestione robot tools

        /// <summary>
        /// Recupera dal database tutti i tool salvati
        /// </summary>
        /// <param name="connectionString"></param>
        /// <returns></returns>
        public DataTable GetRobotTools(String connectionString)
        {
            // Dichiaro DataTable da restituire
            var dt_robot_tools = new DataTable();

            try
            {
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    string query = "SELECT *" +
                        "FROM " + ROBOT_TOOLS_TABLE_NAME;

                    using (var cmd = new SQLiteCommand(query, con))
                    {
                        using (var reader = cmd.ExecuteReader())
                        {
                            dt_robot_tools.Load(reader);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante la query GetRobotTools: " + ex.ToString());
            }

            return dt_robot_tools;
        }

        #endregion

        #region Metodi gestione robot collision levels

        /// <summary>
        /// Recupera dal database tutti i livelli di collisione salvati
        /// </summary>
        /// <param name="connectionString"></param>
        /// <returns></returns>
        public DataTable GetRobotCollisionLevels(String connectionString)
        {
            // Dichiaro DataTable da restituire
            var dt_robot_collision_levels = new DataTable();

            try
            {
                using (var con = new SQLiteConnection(connectionString))
                {
                    con.Open();

                    string query = "SELECT *" +
                        "FROM " + ROBOT_COLLISION_LEVELS_TABLE_NAME;

                    using (var cmd = new SQLiteCommand(query, con))
                    {
                        using (var reader = cmd.ExecuteReader())
                        {
                            dt_robot_collision_levels.Load(reader);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                log.Error("Si è verificata un'eccezione SQL durante la query GetRobotTools: " + ex.ToString());
            }

            return dt_robot_collision_levels;
        }

        #endregion

        #endregion
    }
}
