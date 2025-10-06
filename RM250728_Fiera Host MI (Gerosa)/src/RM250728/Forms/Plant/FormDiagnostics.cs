using RMLib.Alarms;
using RMLib.PLC;
using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RM.src.RM250728.Forms.Plant
{
    public partial class FormDiagnostics : Form
    {
        #region Movimento form
        private bool dragging = false;
        private Point dragCursorPoint;
        private Point dragFormPoint;
        #endregion

        /// <summary>
        /// Comunica diagnostica del programma
        /// </summary>
        public FormDiagnostics()
        {
            InitializeComponent();

            // Avvia il caricamento asincrono dei parametri
            // Task.Run(() => InitParameters());

            // Collegamento evento ValueChanged del dizionario al metodo HandleDictionaryChange
            PLCConfig.appVariables.ValueChanged += RefreshVariables;
        }
        #region Metodi di FormDiagnostics

        /// <summary>
        /// Inizializzazione parametri
        /// </summary>
        /// <returns></returns>
        private async Task InitParameters()
        {
            object fasePLC;

            lock (PLCConfig.appVariables)
            {
                fasePLC = PLCConfig.appVariables.getValue("PLC1_" + "fasePLC");
            }

            // Aggiorna la UI nel thread della UI
            if (InvokeRequired)
            {
                Invoke(new Action(() =>
                {
                    UpdateUI(fasePLC);
                }));
            }
            else
            {
                UpdateUI(fasePLC);
            }

        }

        /// <summary>
        /// Aggiornamento dell'UI
        /// </summary>
        /// <param name="fasePLC"></param>
        private void UpdateUI(object fasePLC)
        {
            // Inizializzo la label 'Frequenza'
            if (fasePLC != null) lbl_fasePLC.Text = fasePLC.ToString();

        }

        /// <summary>
        /// Aggiornamento delle variabili
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void RefreshVariables(object sender, DictionaryChangedEventArgs e)
        {
            if (AlarmManager.isPlcConnected)
            {
                switch (e.Key)
                {
                    case "PLC1_" + "fasePLC":
                        lbl_fasePLC.Text = e.NewValue.ToString();
                        break;

                }
            }
        }

        /// <summary>
        /// Aggiorna label fase Robot
        /// </summary>
        /// <param name="description">Descrizione fase</param>
        public void UpdateRobotStepDescription(string description)
        {
            lbl_faseRobot.Text = description;
        }

        #endregion

        #region Eventi di FormDiagnostics

        /// <summary>
        /// Chiusura panel
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlClose_Click(object sender, EventArgs e)
        {
            Hide();
        }

        /// <summary>
        /// Movimento 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void FormDiagnostics_MouseDown(object sender, MouseEventArgs e)
        {
            dragging = true;
            dragCursorPoint = Cursor.Position;
            dragFormPoint = Location;
        }

        /// <summary>
        /// Movimento 
        /// </summary>
        private void FormDiagnostics_MouseUp(object sender, MouseEventArgs e)
        {
            dragging = false;
        }

        /// <summary>
        /// Movimento 
        /// </summary>
        private void FormDiagnostics_MouseMove(object sender, MouseEventArgs e)
        {
            if (dragging)
            {
                Point dif = Point.Subtract(Cursor.Position, new Size(dragCursorPoint));
                Location = Point.Add(dragFormPoint, new Size(dif));
            }
        }

        #endregion
    }
}
