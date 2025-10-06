using RMLib.Logger;
using RMLib.MessageBox;
using System;
using System.IO;

namespace RM.src.RM250728
{
    /// <summary>
    /// Gestisce la configurazione di sistema per l'avvio corretto del programma in base alla riga di comando passata. 
    /// Serve quindi per aprire il programma in modalità LOCAL, DEBUG o PRODUCTION
    /// </summary>
    public class Configuration
    {
        #region Proprietà della classe Configuration
        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        /// <summary>
        /// Stringa di connessione per comunicare con il database
        /// </summary>
        public static String DB_CONN_STRING = "";

        /// <summary>
        /// Driver per la comunicazione
        /// </summary>
        public static String DB_DRIVER_CLASS = "org.sqlite.JDBC";

        /// <summary>
        /// Username del db
        /// </summary>
        public static String DB_USR = "";

        /// <summary>
        /// Password per il db
        /// </summary>
        public static String DB_PSW = "";

        /// <summary>
        /// IP del database
        /// </summary>
        public static String DB_HOST_IP = "";

        /// <summary>
        /// Nome identificativo del database
        /// </summary>
        public static String DB_DB_NAME = "";

        /// <summary>
        /// Porta del database
        /// </summary>
        public static int DB_HOST_PORT = 1433;
        #endregion

        #region Metodi della classe Configuration
        /// <summary>
        /// Funzione generica per get dei parametri dai file config
        /// </summary>
        /// <returns></returns>
        public static bool basicConfigurationFromFile()
        {
            String line;
            try
            {
                // Passaggio del percorso del file
                // e del nome del file al costruttore dello StreamReader
                StreamReader sr = new StreamReader(RMLib.Environment.Environment.EncodeFileName("config", "config", ".properties"));

                // Leggo la prima riga
                line = sr.ReadLine();

                // Continuo fino all'ultima riga del file
                while (line != null)
                {
                    if (!string.IsNullOrWhiteSpace(line) && !line.Contains("#"))
                    {
                        string[] words = line.Split('=');
                        switch (words[0].Trim())
                        {
                            case "DB_HOST_IP":
                                DB_HOST_IP = words[1].Trim();
                                break;
                            case "DB_HOST_PORT":
                                DB_HOST_PORT = Convert.ToInt32(words[1].Trim());
                                break;
                            case "DB_DB_NAME":
                                DB_DB_NAME = words[1].Trim();
                                break;
                            case "DB_USR":
                                DB_USR = words[1].Trim();
                                break;
                            case "DB_PSW":
                                DB_PSW = words[1].Trim();
                                break;
                        }
                    }

                    // Leggo la prossima riga
                    line = sr.ReadLine();
                }
                // Chiudo il file
                sr.Close();
                Console.ReadLine();
            }
            catch (FileNotFoundException ex)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Configuration file not found " + ex.Message);
                //MessageBox.Show("Configuration file not found " + ex.Message, "ERROR", MessageBoxButtons.OK, MessageBoxIcon.Error);

                log.Error("Configuration file not found " + ex.Message);
                return false;
            }
            catch (IOException e)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Configuration file error " + e.Message);
                //MessageBox.Show("Configuration file error " + e.Message, "ERROR", MessageBoxButtons.OK, MessageBoxIcon.Error);

                log.Error("Configuration file error " + e.Message);
                return false;
            }

            log.Info("Loaded basic configuration from file");
            return true;
        }
        #endregion
    }
}
