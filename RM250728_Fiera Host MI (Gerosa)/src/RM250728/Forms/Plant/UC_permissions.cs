using RM.Properties;
using RMLib.Alarms;
using RMLib.PLC;
using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Collections.Generic;

namespace RM.src.RM250728
{
    /// <summary>
    /// Pagina dedicata alla visualizzazione e gestione dei permessi necessari affinchè il robot possa 
    /// funzionare.
    /// </summary>
    public partial class UC_permissions : UserControl
    {
        #region Proprietà di UC_permissions
        // Memorie consensi robot
        private static bool 
            Hmi_Start_Condition_1_default = false, 
            Hmi_Start_Condition_2_default = false, 
            Hmi_Start_Condition_3_default = false, 
            Hmi_Start_Condition_4_default = false, 
            Hmi_Start_Condition_5_default = false, 
            Hmi_Start_Condition_6_default = false;
        #endregion

        /// <summary>
        /// Costruttore vuoto per inizializzare la pagina dei permessi
        /// </summary>
        public UC_permissions()
        {
            InitOnDemandVariables();
            InitializeComponent();
            InitComponents();

            // Avvia il caricamento asincrono dei parametri
            Task.Run(() => InitParameters());

            // Collegamento evento ValueChanged del dizionario al metodo HandleDictionaryChange
            PLCConfig.appVariables.ValueChanged += RefreshVariables;
        }

        #region Metodi di UC_permissions
        private void InitOnDemandVariables()
        {

        }

        /// <summary>
        /// Inizializza i valori dei controlli presenti nell'interfaccia
        /// </summary>
        private async Task InitParameters()
        {
           
        }

        /// <summary>
        /// Aggiornamento grafico dei consensi
        /// </summary>
        /// <param name="Hmi_Start_Condition_1"></param>
        /// <param name="Hmi_Start_Condition_2"></param>
        /// <param name="Hmi_Start_Condition_3"></param>
        /// <param name="Hmi_Start_Condition_4"></param>
        /// <param name="Hmi_Start_Condition_5"></param>
        /// <param name="Hmi_Start_Condition_6"></param>
        private void UpdateUI(object Hmi_Start_Condition_1, object Hmi_Start_Condition_2,object Hmi_Start_Condition_3,
                       object Hmi_Start_Condition_4,object Hmi_Start_Condition_5,object Hmi_Start_Condition_6)
        {
            // Aggiorna la UI nel thread della UI
            if (Hmi_Start_Condition_1 != null && Hmi_Start_Condition_1.ToString() == "True")
                pnl_ledStopCycleAlarm.BackgroundImage = Resources.greenCircle;
            else
                pnl_ledStopCycleAlarm.BackgroundImage = Resources.redCircle;

            if (Hmi_Start_Condition_2 != null && Hmi_Start_Condition_2.ToString() == "True")
                pnl_ledRobotReady.BackgroundImage = Resources.greenCircle;
            else
                pnl_ledRobotReady.BackgroundImage = Resources.redCircle;

            if (Hmi_Start_Condition_3 != null && Hmi_Start_Condition_3.ToString() == "True")
                pnl_ledTapeInit.BackgroundImage = Resources.greenCircle;
            else
                pnl_ledTapeInit.BackgroundImage = Resources.redCircle;

            if (Hmi_Start_Condition_4 != null && Hmi_Start_Condition_4.ToString() == "True")
                pnl_ledRobotGripperOpen.BackgroundImage = Resources.greenCircle;
            else
                pnl_ledRobotGripperOpen.BackgroundImage = Resources.redCircle;

            if (Hmi_Start_Condition_5 != null && Hmi_Start_Condition_5.ToString() == "True")
                pnl_ledBarrier.BackgroundImage = Resources.greenCircle;
            else
                pnl_ledBarrier.BackgroundImage = Resources.redCircle;

            if (Hmi_Start_Condition_6 != null && Hmi_Start_Condition_6.ToString() == "True")
                pnl_ledRobotInPosition.BackgroundImage = Resources.greenCircle;
            else
                pnl_ledRobotInPosition.BackgroundImage = Resources.redCircle;

            pnl_ledStopCycleAlarm.Visible = true;
            pnl_ledRobotReady.Visible = true;
            pnl_ledTapeInit.Visible = true;
            pnl_ledRobotGripperOpen.Visible = true;
            pnl_ledBarrier.Visible = true;
            pnl_ledRobotInPosition.Visible = true;

            if (Hmi_Start_Condition_1 != null) Hmi_Start_Condition_1_default = Hmi_Start_Condition_1.ToString() == "True";
            if (Hmi_Start_Condition_2 != null) Hmi_Start_Condition_2_default = Hmi_Start_Condition_2.ToString() == "True";
            if (Hmi_Start_Condition_3 != null) Hmi_Start_Condition_3_default = Hmi_Start_Condition_3.ToString() == "True";
            if (Hmi_Start_Condition_4 != null) Hmi_Start_Condition_4_default = Hmi_Start_Condition_4.ToString() == "True";
            if (Hmi_Start_Condition_5 != null) Hmi_Start_Condition_5_default = Hmi_Start_Condition_5.ToString() == "True";
            if (Hmi_Start_Condition_6 != null) Hmi_Start_Condition_6_default = Hmi_Start_Condition_6.ToString() == "True";

        }

        /// <summary>
        /// Inizializza i controlli di UC_permissions
        /// </summary>
        private void InitComponents()
        {
            FormHomePage.Instance.LabelHeader = "CONSENSI";
           // FormHomePage.Instance.ChangeBackColor = BackColor;
            //TODO: lettura dei valori da assegnare ai pulsanti e label in base ai valori del plc nel dizionario
            //TODO: fare un timer che ogni tot tempo refresha i valori rileggendoli dal dizionario
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
        }
        #endregion

        #region Eventi di UC_permissions
        /// <summary>
        /// Ritorno ad Home
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_back(object sender, EventArgs e)
        {
            FormHomePage.Instance.LabelHeader = "HOME PAGE";
           // FormHomePage.Instance.ChangeBackColor = SystemColors.Control;
            FormHomePage.Instance.PnlContainer.Controls["UC_HomePage"].BringToFront();
            FormHomePage.Instance.PnlContainer.Controls.Remove(Controls["UC_permissions"]);

            List<string> keys = new List<string>();

            RefresherTask.RemoveOnDemand(keys);

            Dispose();
        }

        /// <summary>
        /// info allarme stop ciclo
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlInfoStopCycleAlarm_Click(object sender, EventArgs e)
        {
            FormInfoRobot form = new FormInfoRobot("Allarme stop ciclo",
                "Non devono essere presenti allarmi bloccanti attivi");
            form.ShowDialog();
        }

        /// <summary>
        /// Info robot pronto
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlInfoRobotReady_Click(object sender, EventArgs e)
        {
            FormInfoRobot form = new FormInfoRobot("Robot pronto",
                "Il robot deve rispettare i parametri di avvio");
            form.ShowDialog();
        }

        /// <summary>
        /// Info init nastro
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlInfoTapeInit_Click(object sender, EventArgs e)
        {
            FormInfoRobot form = new FormInfoRobot("Init nastro",
                "Il nastro deve essere in posizione per lavorare");
            form.ShowDialog();
        }

        /// <summary>
        /// Info pinze aperte
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlInfoRobotGripperOpen_Click(object sender, EventArgs e)
        {
            FormInfoRobot form = new FormInfoRobot("Pinze robot aperte",
                "Le pinze del robot devono essere aperte");
            form.ShowDialog();
        }

        /// <summary>
        /// info barriere
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlInfoBarrier_Click(object sender, EventArgs e)
        {
            FormInfoRobot form = new FormInfoRobot("Barriere",
                "Le barriere non devono essere interrotte");
            form.ShowDialog();
        }

        /// <summary>
        /// Info robot in posizione
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlInfoRobotInPosition_Click(object sender, EventArgs e)
        {
            FormInfoRobot form = new FormInfoRobot("Robot in posizione",
                "Il robot si deve trovare in posizione di partenza");
            form.ShowDialog();
        }
        #endregion
    }
}
