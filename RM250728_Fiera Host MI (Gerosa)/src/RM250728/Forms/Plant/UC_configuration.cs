using RM.Properties;
using RMLib.PLC;
using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Collections.Generic;

namespace RM.src.RM250728
{
    /// <summary>
    /// Pagina per la configurazione delle proprietà del software
    /// </summary>
    public partial class UC_configuration : UserControl
    {
        #region Parametri di UC_configuration

        /// <summary>
        /// False quando interruttore tempo US è spento
        /// </summary>
        private bool USActivation = false;

        /// <summary>
        /// False quando interruttore pulizia US è spento
        /// </summary>
        private bool USCleaning = false;

        /// <summary>
        /// False quando tasto jog nastro non premuto 
        /// </summary>
        private bool jogTape = false;

        /// <summary>
        /// False quando tasto apri pinze robot non premuto
        /// </summary>
        private bool robotGripperOpened = false;

        /// <summary>
        /// False quando tasto chiudi pinze robot non premuto
        /// </summary>
        private bool robotGripperClosed = false;

        /// <summary>
        /// False quando tasto forzamento reset teglie non premuto
        /// </summary>
        private bool traysReseted = false;

        private static bool jogNastro_default = false, USActiveted_default = false, USCleaned_default = false, 
            gripperOpened_default = false, gripperClosed_default = false;

        private static bool isDemoEnabled = false;

        private static bool isEmptying = false;

        #endregion

        /// <summary>
        /// Costruttore vuoto per creare la pagina di configurazione
        /// </summary>
        public UC_configuration()
        {
            InitOnDemandVariables();
            InitializeComponent();
            InitComponents();

            // Collegamento evento ValueChanged del dizionario al metodo HandleDictionaryChange
            PLCConfig.appVariables.ValueChanged += RefreshVariables;

            /*
            if (Global.fieraScreenSaver)
            {
                foreach (Control control in Controls)
                {
                    control.MouseLeave += ScreenSaverManager.MouseMoveEvent_resetTimerScreenSaver;
                    //TODO: bisogna scorrere i controlli anche dentro i panel
                }
            }*/

            // Avvia il caricamento asincrono dei parametri
            Task.Run(() => InitParameters());
        }

        #region Metodi di UC_configuration

      

        private void InitOnDemandVariables()
        {/*
            RefresherTask.AddOnDemand("PLC1_" + "JogNastro_DX",
            "{\"register_type\":\"Holding Register\", \"address\":9, \"bit_position\":0}",
            AppVariableTypeEnum.BOOL,
            0);

            RefresherTask.AddOnDemand("PLC1_" + "CMD ON US_Dx",
            "{\"register_type\":\"Holding Register\", \"address\":1, \"bit_position\":1}",
            AppVariableTypeEnum.BOOL,
            0);

            RefresherTask.AddOnDemand("PLC1_" + "Lavaggio_DX",
            "{\"register_type\":\"Holding Register\", \"address\":9, \"bit_position\":4}",
            AppVariableTypeEnum.BOOL,
            0);*/
        }

        /// <summary>
        /// Inizializza i controlli
        /// </summary>
        private void InitComponents()
        {
            FormHomePage.Instance.LabelHeader = "CONFIGURAZIONE";
            FormHomePage.Instance.ChangeBackColor = BackColor;
            //TODO: lettura dei valori da assegnare ai pulsanti e label in base ai valori del plc nel dizionario
            //TODO: fare un timer che ogni tot tempo refresha i valori rileggendoli dal dizionario
        }

        /// <summary>
        /// Inizializza i valori dei controlli presenti nell'interfaccia
        /// </summary>
        private async Task InitParameters()
        {
           // await Task.Delay(2000);

            object jogNastro, USActiveted, USCleaned, gripperOpened, gripperClosed;          

            lock (PLCConfig.appVariables)
            {
                jogNastro = PLCConfig.appVariables.getValue("PLC1_" + "JogNastro_DX");
                USActiveted = PLCConfig.appVariables.getValue("PLC1_" + "CMD ON US_Dx");
                USCleaned = PLCConfig.appVariables.getValue("PLC1_" + "Lavaggio_DX");
                gripperOpened = PLCConfig.appVariables.getValue("PLC1_" + "FBK Gripper Open");
                gripperClosed = PLCConfig.appVariables.getValue("PLC1_" + "FBK Gripper Close 1");
            }
            
           
            // Aggiorna la UI nel thread della UI
            if (InvokeRequired)
            {
                Invoke(new Action(() =>
                {
                    UpdateUI(USActiveted, USCleaned, gripperOpened, gripperClosed);
                }));
            }
            else
            {
                UpdateUI(USActiveted, USCleaned, gripperOpened, gripperClosed);
            }
        }

        private void UpdateUI(object USActiveted, object USCleaned, object gripperOpened, object gripperClosed)
        {
            // Inizializzo l'interruttore 'Abilitazione US'
            if (USActiveted != null && USActiveted.ToString() == "True")
            {
                pnl_USActivation.BackgroundImage = Resources.on_button;
                USActivation = true;
            }
            else
            {
                pnl_USActivation.BackgroundImage = Resources.off_button;
                USActivation = false;
            }

            // Inizializzo l'interruttore 'Pulizia US'
            if (USCleaned != null && USCleaned.ToString() == "True")
            {
                pnl_USCleaning.BackgroundImage = Resources.on_button;
                USCleaning = true;
            }
            else
            {
                pnl_USCleaning.BackgroundImage = Resources.off_button;
                USCleaning = false;
            }

            // Inizializzo il button 'apri pinze ON'
            if (gripperOpened != null && gripperOpened.ToString() == "True")
            {
                btn_openRobotGripper.BackColor = Color.FromArgb(40, 175, 75);
                robotGripperOpened = true;
            }
            else
            {
                btn_openRobotGripper.BackColor = SystemColors.ActiveBorder;
                robotGripperOpened = false;
            }

            // Inizializzo il button 'chiudi pinze ON'
            if (gripperClosed != null && gripperClosed.ToString() == "True")
            {
                btn_closeRobotGripper.BackColor = Color.FromArgb(40, 175, 75);
                robotGripperClosed = true;
            }
            else
            {
                btn_closeRobotGripper.BackColor = SystemColors.ActiveBorder;
                robotGripperClosed = false;
            }

            if (USActiveted != null) USActiveted_default = USActiveted.ToString() == "True";
            if (USCleaned != null) USCleaned_default = USCleaned.ToString() == "True";
            if (gripperOpened != null) gripperOpened_default = gripperOpened.ToString() == "True";
            if (gripperClosed != null) gripperClosed_default = gripperClosed.ToString() == "True";
        }

    

    /// <summary>
    /// Metodo richiamato dall'evento ValueChanged del dizionario delle variabili PLC
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    public void RefreshVariables(object sender, DictionaryChangedEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action<object, DictionaryChangedEventArgs>(RefreshVariables), sender, e);
                return;
            }

            switch (e.Key)
            {
                case "PLC1_" + "FBK Gripper Open":
                    if (e.NewValue.ToString() == "True")
                        btn_openRobotGripper.BackColor = Color.FromArgb(40, 175, 75);
                    else
                        btn_openRobotGripper.BackColor = SystemColors.ActiveBorder;
                    break;

                case "PLC1_" + "FBK Gripper Close 1":
                    if (e.NewValue.ToString() == "True")
                        btn_closeRobotGripper.BackColor = Color.FromArgb(40, 175, 75);
                    else
                        btn_closeRobotGripper.BackColor = SystemColors.ActiveBorder;
                    break;

                case "PLC1_" + "CMD ON US_Dx":
                    if (e.NewValue.ToString() == "True")
                        pnl_USActivation.BackgroundImage = Resources.on_button;
                    else
                        pnl_USActivation.BackgroundImage = Resources.off_button;
                    break;

                case "PLC1_" + "Lavaggio_DX":
                    if (e.NewValue.ToString() == "True")
                        pnl_USCleaning.BackgroundImage = Resources.on_button;
                    else
                        pnl_USCleaning.BackgroundImage = Resources.off_button;
                    break;
            }
        }
        #endregion

        #region Eventi di UC_configuration
        /// <summary>
        /// Ritorno a pagina di home
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnHome_Click(object sender, EventArgs e)
        {
            FormHomePage.Instance.LabelHeader = "HOME PAGE";
            FormHomePage.Instance.ChangeBackColor = SystemColors.Control;
            FormHomePage.Instance.PnlContainer.Controls["UC_HomePage"].BringToFront();
            FormHomePage.Instance.PnlContainer.Controls.Remove(Controls["UC_permissions"]);

            List<string> keys = new List<string>();
            keys.Add("PLC1_" + "JogNastro_DX");
            keys.Add("PLC1_" + "CMD ON US_Dx");
            keys.Add("PLC1_" + "Lavaggio_DX");
            RefresherTask.RemoveOnDemand(keys);

            Dispose();
        }

        /// <summary>
        /// Gestione interruttore tempo US
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlUSActivation_Click(object sender, EventArgs e)
        {
            if (USActivation)
            {
                RefresherTask.AddUpdate("PLC1_" + "CMD ON US_Dx", false, "BOOL");
                pnl_USActivation.BackgroundImage = Resources.off_button;
            }
            else
            {
                RefresherTask.AddUpdate("PLC1_" + "CMD ON US_Dx", true, "BOOL");
                pnl_USActivation.BackgroundImage = Resources.on_button;
            }

            USActivation = !USActivation;

            USActiveted_default = USActivation;
        }

        /// <summary>
        /// Gestione interruttore pulizia US
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlUSCleaning_Click(object sender, EventArgs e)
        {
            if (USCleaning)
            {
                pnl_USCleaning.BackgroundImage = Resources.off_button;
                RefresherTask.AddUpdate("PLC1_" + "Lavaggio_DX", false, "BOOL");
            }
            else
            {
                pnl_USCleaning.BackgroundImage = Resources.on_button;
                RefresherTask.AddUpdate("PLC1_" + "Lavaggio_DX", true, "BOOL");
            }
            
            USCleaning = !USCleaning;

            USCleaned_default = USCleaning;
        }

        /// <summary>
        /// Gestisce colore tasto jog nastro
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnJogTape_Click(object sender, EventArgs e)
        {

        }

        /// <summary>
        /// Gestisce colore tasto apri pinze robot
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnOpenRobotGripper_Click(object sender, EventArgs e)
        {

        }

        /// <summary>
        /// Gestisce colore tasto chiudi pinze robot
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnCloseRobotGripper_Click(object sender, EventArgs e)
        {

        }

        /// <summary>
        /// Gestisce colore tasto forzamento reset teglie
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnResetTrays_Click(object sender, EventArgs e)
        {

        }
        #endregion

        private void btn_resetTrays_MouseDown(object sender, MouseEventArgs e)
        {
            btn_resetTrays.BackColor = Color.FromArgb(40, 175, 75);
            RefresherTask.AddUpdate("PLC1_" + "Reset_Teglie", true, "BOOL");
        }

        private void btn_resetTrays_MouseUp(object sender, MouseEventArgs e)
        {
            btn_resetTrays.BackColor = SystemColors.ActiveBorder;
            RefresherTask.AddUpdate("PLC1_" + "Reset_Teglie", false, "BOOL");
        }

        private void btn_jogTape_MouseDown(object sender, MouseEventArgs e)
        {
            btn_jogTape.BackColor = Color.FromArgb(40, 175, 75);
            RefresherTask.AddUpdate("PLC1_" + "JogNastro_DX", true, "BOOL");  
        }

        private void btn_jogTape_MouseUp(object sender, MouseEventArgs e)
        {
            btn_jogTape.BackColor = SystemColors.ActiveBorder;
            RefresherTask.AddUpdate("PLC1_" + "JogNastro_DX", false, "BOOL");   

            jogNastro_default = !jogNastro_default;
        }

   

        private void btn_openRobotGripper_MouseDown(object sender, MouseEventArgs e)
        {
            btn_openRobotGripper.BackColor = Color.FromArgb(40, 175, 75);
            RefresherTask.AddUpdate("PLC1_" + "Cmd_Apri_Pinze_Robot", true, "BOOL");
        }

        private void btn_openRobotGripper_MouseUp(object sender, MouseEventArgs e)
        {
            btn_openRobotGripper.BackColor = SystemColors.ActiveBorder;
            RefresherTask.AddUpdate("PLC1_" + "Cmd_Apri_Pinze_Robot", false, "BOOL");

            gripperOpened_default = !gripperOpened_default;
        }

        private void btn_closeRobotGripper_MouseDown(object sender, MouseEventArgs e)
        {
            btn_closeRobotGripper.BackColor = Color.FromArgb(40, 175, 75);
            RefresherTask.AddUpdate("PLC1_" + "Cmd_Chiudi_Pinze_Robot_Pos_1", true, "BOOL");     
        }

        private void btn_closeRobotGripper_MouseUp(object sender, MouseEventArgs e)
        {
            btn_closeRobotGripper.BackColor = SystemColors.ActiveBorder;
            RefresherTask.AddUpdate("PLC1_" + "Cmd_Chiudi_Pinze_Robot_Pos_1", false, "BOOL");

            gripperClosed_default = !gripperClosed_default;
        }
    }
}
