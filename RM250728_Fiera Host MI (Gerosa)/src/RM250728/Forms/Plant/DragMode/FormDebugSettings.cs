using RMLib.DataAccess;
using System;
using System.Drawing;
using System.Windows.Forms;

namespace RM.src.RM250728.Forms.Plant.DragMode
{
    /// <summary>
    /// Form contente i controlli per modificare velocemente i parametri del robot senza cmabiare pagina tornando indietro
    /// </summary>
    public partial class FormDebugSettings : Form
    {
        #region Proprietà di FormDebugSettings

        #region Variabili per la connessione al DB
        // Variabili per la connessione al database
        private static readonly RobotDAOSqlite robotDAO = new RobotDAOSqlite();
        private static readonly SqliteConnectionConfiguration DatabaseConnection = new SqliteConnectionConfiguration();
        private static readonly string ConnectionString = DatabaseConnection.GetConnectionString();
        #endregion

        /// <summary>
        /// Evento per modificare la modalità di drag in Linear
        /// </summary>
        public event EventHandler LinearModeCheckedChanged;

        /// <summary>
        /// Evento per modificare la modalità di drag in PTP
        /// </summary>
        public event EventHandler PTPModeCheckedChanged;

        /// <summary>
        /// Indica se la form è già stata mostrata
        /// </summary>
        public bool isShown = false;

        #endregion

        /// <summary>
        /// Costruisce la form contenente i controlli per modificare i parametri del robot
        /// </summary>
        public FormDebugSettings()
        {
            InitializeComponent();
            InitLabels();
        }

        #region Metodi di FormDebugSettings

        /// <summary>
        /// Inizializza la form prendendo i valori da robot manager e quindi dal db
        /// </summary>
        private void InitLabels()
        {

        }

        #endregion

        #region Eventi della form

        /// <summary>
        /// Gestione scelta mode PTP
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CheckedChangeEvent_radioButtonPTP(object sender, EventArgs e)
        {
            if (rbPointToPoint.Checked)
            {
                PTPModeCheckedChanged?.Invoke(this, EventArgs.Empty);
            }
        }

        /// <summary>
        /// Gestione scelta mode LINEAR
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CheckedChangeEvent_radioButtonLinear(object sender, EventArgs e)
        {
            if (rbLinear.Checked)
            {
                LinearModeCheckedChanged?.Invoke(this, EventArgs.Empty);
            }
        }

        /// <summary>
        /// Info su PTP mode
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlInfoPTP_Click(object sender, EventArgs e)
        {
            FormInfoRobot form = new FormInfoRobot(
                "Point to point mode",
                "Sposta il robot e ferma la drag mode per ottenere il nuovo punto. Si avranno un numero minore di punti e una " +
                "curva più spigolosa."
                );
            form.ShowDialog();
        }


        /// <summary>
        /// Info su Linear mode
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlInfoLinear_Click(object sender, EventArgs e)
        {
            FormInfoRobot form = new FormInfoRobot(
                "Linear mode",
                "Prende i punti in automatico mentre sposti il robot. Si avranno più punti ravvicinati e una curva più lineare."
                );
            form.ShowDialog();
        }

        /// <summary>
        /// Seleziona la modalità ptp
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_pb_PTP(object sender, EventArgs e)
        {
            if (!rbPointToPoint.Checked)
            {
                rbPointToPoint.Checked = true;
            }
        }

        /// <summary>
        /// Seleziona la modalità linear
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_pb_linear(object sender, EventArgs e)
        {
            if (!rbLinear.Checked)
            {
                rbLinear.Checked = true;
            }
        }

        #endregion

        #region Eventi per movimento form

        // Variabili per il trascinamento
        private bool dragging = false;
        private Point dragCursorPoint;
        private Point dragFormPoint;

        /// <summary>
        /// Spostamento del mouse per il trascinamento della form
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MouseMoveEvent_moving(object sender, MouseEventArgs e)
        {
            if (dragging)
            {
                Point dif = Point.Subtract(Cursor.Position, new Size(dragCursorPoint));
                Location = Point.Add(dragFormPoint, new Size(dif));
            }
        }

        /// <summary>
        /// Click sulla form per il trascinamento in mouse up
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MouseUpEvent_stopMove(object sender, MouseEventArgs e)
        {
            dragging = false;
        }

        /// <summary>
        /// Click sulla form per il trascinamento in mouse down
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MouseDownEvent_startMove(object sender, MouseEventArgs e)
        {
            dragging = true;
            dragCursorPoint = Cursor.Position;
            dragFormPoint = Location;
        }

        #endregion
    }
}
