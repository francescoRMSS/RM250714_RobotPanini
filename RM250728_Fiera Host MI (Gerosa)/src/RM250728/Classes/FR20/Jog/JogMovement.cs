using RM.src.RM250728.Classes.PLC;
using RMLib.Alarms;
using RMLib.PLC;
using System;
using System.Threading;
using System.Web.UI;

namespace RM.src.RM250728.Classes.FR20.Jog
{
    /// <summary>
    /// Gestisce movimento del Robot in Jog
    /// </summary>
    public static class JogMovement
    {
        #region Parametri di JogMovement

        /// <summary>
        /// Dichiarazione oggetto static per gestione jog robot
        /// </summary>
        public static JogRobotProperties jog = new JogRobotProperties();

        #region Struttura thread

        /// <summary>
        /// Oggetto thread
        /// </summary>
        public static Thread jogRobotThread;

        /// <summary>
        /// Delay utilizzato nel thread
        /// </summary>
        private static int jogRobotThreadRefreshPeriod = 50;

        /// <summary>
        /// A 1 quando il thread è partito
        /// </summary>
        public static bool jogRobotThreadStarted = false;

        /// <summary>
        /// A 1 quando il thread deve essere fermato
        /// </summary>

        public static bool stopJogThread = false;

        #endregion

        #endregion

        #region Metodi di JogMovement

        /// <summary>
        /// Inizializza JogMovement collegano l'evento della modifica del dizionario delle variabili al metodo RefreshVariables
        /// </summary>
        /// <returns></returns>
        public static bool InitJog()
        {
            // Collegamento evento ValueChanged del dizionario al metodo RefreshVariables
            PLCConfig.appVariables.ValueChanged += RefreshVariables;

            return true;
        }

        /// <summary>
        /// Esegue aggiornamento delle soglie e della velocità di Jog
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public static void RefreshVariables(object sender, DictionaryChangedEventArgs e)
        {
            if (AlarmManager.isPlcConnected) // Se il PLC è connesso
            {
                switch (e.Key)
                {
                    case PLCTagName.x_threshold:
                        #region Get soglia asse X [mm]

                        jog.JogX_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.x_threshold));

                        break;

                    #endregion

                    case PLCTagName.y_threshold:
                        #region Get soglia asse Y [mm]

                        jog.JogY_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.y_threshold));

                        break;

                    #endregion

                    case PLCTagName.z_threshold:
                        #region Get soglia asse Z [mm]

                        jog.JogZ_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.z_threshold));

                        break;

                    #endregion

                    case PLCTagName.rx_threshold:
                        #region Get soglia rx [°]

                        jog.JogRX_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.rx_threshold));

                        break;

                    #endregion

                    case PLCTagName.ry_threshold:
                        #region Get soglia ry [°]

                        jog.JogRY_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.ry_threshold));

                        break;

                    #endregion

                    case PLCTagName.rz_threshold:
                        #region Get soglia rz [°]

                        jog.JogRZ_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.rz_threshold));

                        break;

                    #endregion

                    case PLCTagName.Jog_speed_set:
                        #region Get speed in Jog

                        jog.JogSpeed = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Jog_speed_set));

                        break;

                    #endregion

                    case PLCTagName.Jog_Ref_type:
                        #region Get sistema di riferimento

                        jog.JogRefType = Convert.ToByte(PLCConfig.appVariables.getValue(PLCTagName.Jog_Ref_type));

                        break;

                    #endregion

                    case PLCTagName.Axis_selection:
                        #region Get dell'asse selezionato

                        jog.axisSelection = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Axis_selection));

                        break;

                        #endregion
                }
            }
        }

        /// <summary>
        /// Avvia thread che gestisce jog del robot
        /// </summary>
        public static void StartJogRobotThread()
        {
            if (!jogRobotThreadStarted) // Se il thread non è già avviato
            {
                jogRobotThread = new Thread(new ThreadStart(CheckJogRequest));
                jogRobotThread.IsBackground = true;
                jogRobotThread.Priority = ThreadPriority.Normal;
                jogRobotThread.Start();
                jogRobotThreadStarted = true;
                stopJogThread = false;
            }
        }

        /// <summary>
        /// Stoppa thread che gestisce jog del robot
        /// </summary>
        public static void StopJogRobotThread()
        {
            if (jogRobotThreadStarted) // Se il thread è attivo
            {
                stopJogThread = true; // Imposta il flag per fermare il thread
                jogRobotThreadStarted = false;
                jogRobotThread.Join(); // Attende che il thread si fermi
            }
        }

        /// <summary>
        /// Gestisce Jog del robot
        /// </summary>
        private static void CheckJogRequest()
        {
            // Dichiarazione step da usare nei vari switch per gestione del jog robot
            int stepXpos = 0,
                stepXneg = 0,
                stepYpos = 0,
                stepYneg = 0,
                stepZpos = 0,
                stepZneg = 0,
                stepRXpos = 0,
                stepRXneg = 0,
                stepRYpos = 0,
                stepRYneg = 0,
                stepRZpos = 0,
                stepRZneg = 0;

            jog.JogAcceleration = 100; // Imposto accelerazione al massimo

            // Get delle soglie
            jog.JogX_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.x_threshold));
            jog.JogY_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.y_threshold));
            jog.JogZ_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.z_threshold));
            jog.JogRX_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.rx_threshold));
            jog.JogRY_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.ry_threshold));
            jog.JogRZ_treshold = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.rz_threshold));

            // Get del sistema di riferimento
            jog.JogRefType = Convert.ToByte(PLCConfig.appVariables.getValue(PLCTagName.Jog_Ref_type));

            // Get dell'asse selezionato
            jog.axisSelection = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Axis_selection));

            // Get della velocità attuale di jog
            jog.JogSpeed = Convert.ToInt16(PLCConfig.appVariables.getValue(PLCTagName.Jog_speed_set));

            // Dichiarazione byte che conterrà lo stato del movimento del robot (0 -> completato / 1-> in esecuzione)
            byte motionDone = 0;

            while (!stopJogThread)
            {
                if (jog.JogSpeed < 1) // Se la velocità è pari a zero il Robot non si deve muovere
                    continue;

                switch (jog.axisSelection) // Switch sull'asse selezionato
                {
                    case 1:
                        #region Asse x

                        // Get della direzione
                        jog.jogXpos = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogX_pos));
                        jog.jogXneg = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogX_neg));

                        #region Movimento (+) sull'asse x
                        switch (stepXpos)
                        {
                            case 0:
                                if (jog.jogXpos)
                                {
                                    jog.prevJogXpos = true;
                                    stepXpos = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 1, 1, jog.JogSpeed, jog.JogAcceleration, jog.JogX_treshold);
                                stepXpos = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepXpos = 30;
                                }
                                else if (!jog.jogXpos && jog.prevJogXpos)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogXpos = false;
                                    stepXpos = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogXpos)
                                    stepXpos = 0;
                                break;
                        }
                        #endregion

                        #region Movimento (-) sull'asse x
                        switch (stepXneg)
                        {
                            case 0:
                                if (jog.jogXneg)
                                {
                                    jog.prevJogXneg = true;
                                    stepXneg = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 1, 0, jog.JogSpeed, jog.JogAcceleration, jog.JogX_treshold);
                                stepXneg = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepXneg = 30;
                                }
                                else if (!jog.jogXneg && jog.prevJogXneg)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogXneg = false;
                                    stepXneg = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogXneg)
                                    stepXneg = 0;
                                break;
                        }
                        #endregion

                        break;

                    #endregion

                    case 2:
                        #region Asse y

                        // Get della direzione
                        jog.jogYpos = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogY_pos));
                        jog.jogYneg = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogY_neg));

                        #region Movimento (+) sull'asse y
                        switch (stepYpos)
                        {
                            case 0:
                                if (jog.jogYpos)
                                {
                                    jog.prevJogYpos = true;
                                    stepYpos = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 2, 1, jog.JogSpeed, jog.JogAcceleration, jog.JogY_treshold);
                                stepYpos = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepYpos = 30;
                                }
                                else if (!jog.jogYpos && jog.prevJogYpos)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogYpos = false;
                                    stepYpos = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogYpos)
                                    stepYpos = 0;
                                break;
                        }
                        #endregion

                        #region Movimento (-) sull'asse y
                        switch (stepYneg)
                        {
                            case 0:
                                if (jog.jogYneg)
                                {
                                    jog.prevJogYneg = true;
                                    stepYneg = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 2, 0, jog.JogSpeed, jog.JogAcceleration, jog.JogY_treshold);
                                stepYneg = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepYneg = 30;
                                }
                                else if (!jog.jogYneg && jog.prevJogYneg)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogYneg = false;
                                    stepYneg = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogYneg)
                                    stepYneg = 0;
                                break;
                        }
                        #endregion
                        break;

                    #endregion

                    case 3:
                        #region Asse z

                        // Get della direzione
                        jog.jogZpos = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogZ_pos));
                        jog.jogZneg = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogZ_neg));

                        #region Movimento (+) sull'asse z
                        switch (stepZpos)
                        {
                            case 0:
                                if (jog.jogZpos)
                                {
                                    jog.prevJogZpos = true;
                                    stepZpos = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 3, 1, jog.JogSpeed, jog.JogAcceleration, jog.JogZ_treshold);
                                stepZpos = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepZpos = 30;
                                }
                                else if (!jog.jogZpos && jog.prevJogZpos)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogZpos = false;
                                    stepZpos = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogZpos)
                                    stepZpos = 0;
                                break;
                        }
                        #endregion

                        #region Movimento (-) sull'asse z
                        switch (stepZneg)
                        {
                            case 0:
                                if (jog.jogZneg)
                                {
                                    jog.prevJogZneg = true;
                                    stepZneg = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 3, 0, jog.JogSpeed, jog.JogAcceleration, jog.JogZ_treshold);
                                stepZneg = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepZneg = 30;
                                }
                                else if (!jog.jogZneg && jog.prevJogZneg)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogZneg = false;
                                    stepZneg = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogZneg)
                                    stepZneg = 0;
                                break;
                        }
                        #endregion
                        break;

                    #endregion

                    case 4:
                        #region Asse rx

                        // Get della direzione
                        jog.jogRXpos = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogRX_pos));
                        jog.jogRXneg = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogRX_neg));

                        #region Movimento (+) sull'asse rx
                        switch (stepRXpos)
                        {
                            case 0:
                                if (jog.jogRXpos)
                                {
                                    jog.prevJogRXpos = true;
                                    stepRXpos = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 4, 1, jog.JogSpeed, jog.JogAcceleration, jog.JogRX_treshold);
                                stepRXpos = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepRXpos = 30;
                                }
                                else if (!jog.jogRXpos && jog.prevJogRXpos)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogRXpos = false;
                                    stepRXpos = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogRXpos)
                                    stepRXpos = 0;
                                break;
                        }
                        #endregion

                        #region Movimento (-) sull'asse rx
                        switch (stepRXneg)
                        {
                            case 0:
                                if (jog.jogRXneg)
                                {
                                    jog.prevJogRXneg = true;
                                    stepRXneg = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 4, 0, jog.JogSpeed, jog.JogAcceleration, jog.JogRX_treshold);
                                stepRXneg = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepRXneg = 30;
                                }
                                else if (!jog.jogRXneg && jog.prevJogRXneg)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogRXneg = false;
                                    stepRXneg = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogRXneg)
                                    stepRXneg = 0;
                                break;
                        }
                        #endregion
                        break;

                    #endregion

                    case 5:
                        #region Asse ry

                        // Get della direzione
                        jog.jogRYpos = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogRY_pos));
                        jog.jogRYneg = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogRY_neg));

                        #region Movimento (+) sull'asse ry
                        switch (stepRYpos)
                        {
                            case 0:
                                if (jog.jogRYpos)
                                {
                                    jog.prevJogRYpos = true;
                                    stepRYpos = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 5, 1, jog.JogSpeed, jog.JogAcceleration, jog.JogRY_treshold);
                                stepRYpos = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepRYpos = 30;
                                }
                                else if (!jog.jogRYpos && jog.prevJogRYpos)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogRYpos = false;
                                    stepRYpos = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogRYpos)
                                    stepRYpos = 0;
                                break;
                        }
                        #endregion

                        #region Movimento (-) sull'asse ry
                        switch (stepRYneg)
                        {
                            case 0:
                                if (jog.jogRYneg)
                                {
                                    jog.prevJogRYneg = true;
                                    stepRYneg = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 5, 0, jog.JogSpeed, jog.JogAcceleration, jog.JogRY_treshold);
                                stepRYneg = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepRYneg = 30;
                                }
                                else if (!jog.jogRYneg && jog.prevJogRYneg)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogRYneg = false;
                                    stepRYneg = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogRYneg)
                                    stepRYneg = 0;
                                break;
                        }
                        #endregion
                        break;

                    #endregion

                    case 6:
                        #region Asse rz

                        // Get della direzione
                        jog.jogRZpos = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogRZ_pos));
                        jog.jogRZneg = Convert.ToBoolean(PLCConfig.appVariables.getValue(PLCTagName.JogRZ_neg));

                        #region Movimento (+) sull'asse rz
                        switch (stepRZpos)
                        {
                            case 0:
                                if (jog.jogRZpos)
                                {
                                    jog.prevJogRZpos = true;
                                    stepRZpos = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 6, 1, jog.JogSpeed, jog.JogAcceleration, jog.JogRZ_treshold);
                                stepRZpos = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepRZpos = 30;
                                }
                                else if (!jog.jogRZpos && jog.prevJogRZpos)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogRZpos = false;
                                    stepRZpos = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogRZpos)
                                    stepRZpos = 0;
                                break;
                        }
                        #endregion

                        #region Movimento (-) sull'asse rz
                        switch (stepRZneg)
                        {
                            case 0:
                                if (jog.jogRZneg)
                                {
                                    jog.prevJogRZneg = true;
                                    stepRZneg = 10;
                                }
                                break;

                            case 10:
                                RobotManager.robot.StartJOG(jog.JogRefType, 6, 0, jog.JogSpeed, jog.JogAcceleration, jog.JogRZ_treshold);
                                stepRZneg = 20;
                                break;

                            case 20:
                                RobotManager.robot.GetRobotMotionDone(ref motionDone);
                                if (motionDone == 0)
                                {
                                    stepRZneg = 30;
                                }
                                else if (!jog.jogRZneg && jog.prevJogRZneg)
                                {
                                    RobotManager.robot.ImmStopJOG();
                                    jog.prevJogRZneg = false;
                                    stepRZneg = 0;
                                }
                                break;

                            case 30:
                                if (!jog.jogRZneg)
                                    stepRZneg = 0;
                                break;
                        }
                        #endregion
                        break;

                        #endregion
                }

                Thread.Sleep(jogRobotThreadRefreshPeriod);
            }

        }

        #endregion
    }
}
