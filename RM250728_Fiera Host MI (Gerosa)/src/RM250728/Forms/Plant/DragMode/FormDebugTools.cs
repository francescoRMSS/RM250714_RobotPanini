using RM.Properties;
using System;
using System.Drawing;
using System.Windows.Forms;

namespace RM.src.RM250728.Forms.Plant.DragMode
{
    /// <summary>
    /// Form contenente dei controlli per spostarsi tra i punti presi durante la fase di drag mode
    /// </summary>
    public partial class FormDebugTools : Form
    {
        #region Proprietà di FormDebugTools

        /// <summary>
        /// Indica se la form è già stata mostrata prima. Se false significa che ancora non è mai stato fatto Show()
        /// </summary>
        public bool isShown = false;
        /// <summary>
        /// Evento al premere del pulsante previous
        /// </summary>
        public event EventHandler ButtonPreviousClicked;
        /// <summary>
        /// Evento al premere del pulsante next
        /// </summary>
        public event EventHandler ButtonNextClicked;
        /// <summary>
        /// Evento al premere del pulsante go to
        /// </summary>
        public event EventHandler ButtonGoToClicked;
        /// <summary>
        /// Evento al premere del pulsante start monitoring
        /// </summary>
        public event EventHandler ButtonStartMonitoringClicked;
        /// <summary>
        /// Evento al premere del pulsante pause/stop monitoring
        /// </summary>
        public event EventHandler ButtonPauseMonitoringClicked;
        //public event EventHandler ButtonResumeMonitoringClicked;
        /// <summary>
        /// Evento al premere del pulsante modifica
        /// </summary>
        public event EventHandler ButtonModifyClicked;
        /// <summary>
        /// Evento al premere del pulsante sovrascrivi
        /// </summary>
        public event EventHandler ButtonOverwriteClicked;
        /// <summary>
        /// Evento al premere del pulsante cancella
        /// </summary>
        public event EventHandler ButtonDeleteClicked;

        #endregion

        /// <summary>
        /// Costruisce la form dei tools e inizializza lo stato del pulsante di pause/stop monitoring
        /// </summary>
        public FormDebugTools()
        {
            InitializeComponent();

            EnableMonitoringButtons(false);
        }

        #region Metodi di FormDebugTools

        /// <summary>
        /// Abilita o disbilita dei controlli in base al parametro passato. 
        /// In pratica serve per disabilitare dei pulsanti quando la drag mode parte o viene fermata
        /// </summary>
        /// <param name="isDragStart"></param>
        public void EnableButtons(bool isDragStart)
        {
            btn_previousPoint.Enabled = !isDragStart;
            btn_nextPoint.Enabled = !isDragStart;
            btn_goToPoint.Enabled = !isDragStart;

            btn_startMonitoring.Enabled = !isDragStart;
            btn_pauseMonitoring.Enabled |= isDragStart;

            btn_deletePoint.Enabled = !isDragStart;
            btn_overwritePoint.Enabled = !isDragStart;
            btn_modifyPoint.Enabled = !isDragStart;
        }

        /// <summary>
        /// Riporta allo stato corretto l'abilitazione e la visualizzazione dei tasti di monitoring
        /// in base al parametro passato
        /// </summary>
        /// <param name="isMonitoringStart">Indica se il monitoring è stato attivato o disattivato</param>
        public void EnableMonitoringButtons(bool isMonitoringStart)
        {
            btn_pauseMonitoring.Enabled = isMonitoringStart;
            btn_pauseMonitoring.BackColor = isMonitoringStart ? SystemColors.Control : SystemColors.ControlDark;
            btn_pauseMonitoring.BackgroundImage = isMonitoringStart ? Resources.pausemonitoringRed_32 : null;

            btn_startMonitoring.Enabled = !isMonitoringStart;
            btn_startMonitoring.BackColor = !isMonitoringStart ? SystemColors.Control : SystemColors.ControlDark;
            btn_startMonitoring.BackgroundImage = !isMonitoringStart ? Resources.sartMonitoringGreen_32 : null;

            btn_previousPoint.Enabled = !isMonitoringStart;
            btn_previousPoint.BackColor = !isMonitoringStart ? SystemColors.Control : SystemColors.ControlDark;
            btn_previousPoint.BackgroundImage = !isMonitoringStart ? Resources.upArrowBlue_32 : null;

            btn_nextPoint.Enabled = !isMonitoringStart;
            btn_nextPoint.BackColor = !isMonitoringStart ? SystemColors.Control : SystemColors.ControlDark;
            btn_nextPoint.BackgroundImage = !isMonitoringStart ? Resources.downArrowBlue_32 : null;

            btn_goToPoint.Enabled = !isMonitoringStart;
            btn_goToPoint.BackColor = !isMonitoringStart ? SystemColors.Control : SystemColors.ControlDark;
            btn_goToPoint.BackgroundImage = !isMonitoringStart ? Resources.skipBlue_32 : null;

            btn_modifyPoint.Enabled = !isMonitoringStart;
            btn_modifyPoint.BackColor = !isMonitoringStart ? SystemColors.Control : SystemColors.ControlDark;
            btn_modifyPoint.BackgroundImage = !isMonitoringStart ? Resources.modifyBlue_32 : null;

            btn_overwritePoint.Enabled = !isMonitoringStart;
            btn_overwritePoint.BackColor = !isMonitoringStart ? SystemColors.Control : SystemColors.ControlDark;
            btn_overwritePoint.BackgroundImage = !isMonitoringStart ? Resources.overwriteBlue_32 : null;

            btn_deletePoint.Enabled = !isMonitoringStart;
            btn_deletePoint.BackColor = !isMonitoringStart ? SystemColors.Control : SystemColors.ControlDark;
            btn_deletePoint.BackgroundImage = !isMonitoringStart ? Resources.delete32 : null;
        }

        /// <summary>
        /// Riporta allo stato corretto l'abilitazione e la visualizzazione dei tasti di monitoring
        /// in base al parametro passato
        /// </summary>
        /// <param name="isInMovement">Indica se il monitoring è stato attivato o disattivato</param>
        public void EnableMovementButtons(bool isInMovement)
        {
            btn_previousPoint.Enabled = !isInMovement;
            btn_previousPoint.BackColor = isInMovement ? SystemColors.ControlDark : SystemColors.Control;
            btn_previousPoint.BackgroundImage = isInMovement ? null : Resources.upArrowBlue_32;

            btn_nextPoint.Enabled = !isInMovement;
            btn_nextPoint.BackColor = isInMovement ? SystemColors.ControlDark : SystemColors.Control;
            btn_nextPoint.BackgroundImage = isInMovement ? null : Resources.downArrowBlue_32;

            btn_goToPoint.Enabled = !isInMovement;
            btn_goToPoint.BackColor = isInMovement ? SystemColors.ControlDark : SystemColors.Control;
            btn_goToPoint.BackgroundImage = isInMovement ? null : Resources.skipBlue_32;

            btn_modifyPoint.Enabled = !isInMovement;
            btn_modifyPoint.BackColor = isInMovement ? SystemColors.ControlDark : SystemColors.Control;
            btn_modifyPoint.BackgroundImage = isInMovement ? null : Resources.modifyBlue_32;

            btn_overwritePoint.Enabled = !isInMovement;
            btn_overwritePoint.BackColor = isInMovement ? SystemColors.ControlDark : SystemColors.Control;
            btn_overwritePoint.BackgroundImage = isInMovement ? null : Resources.overwriteBlue_32;

            btn_deletePoint.Enabled = !isInMovement;
            btn_deletePoint.BackColor = isInMovement ? SystemColors.ControlDark : SystemColors.Control;
            btn_deletePoint.BackgroundImage = isInMovement ? null : Resources.delete32;

            btn_startMonitoring.Enabled = !isInMovement;
            btn_startMonitoring.BackColor = isInMovement ? SystemColors.ControlDark : SystemColors.Control;
            btn_startMonitoring.BackgroundImage = isInMovement ? null : Resources.sartMonitoringGreen_32;
        }

        #endregion

        #region Eventi di FormDebugTools

        /// <summary>
        /// Genera evento dopo aver cliccato indietro
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_previousPoint(object sender, EventArgs e)
        {
            ButtonPreviousClicked?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Genera evento dopo aver cliccato start monitoring
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_startMonitoring(object sender, EventArgs e)
        {
            ButtonStartMonitoringClicked?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Genera evento dopo aver clicclato modifica
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_modifyPoint(object sender, EventArgs e)
        {
            ButtonModifyClicked?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Genera evento dopo aver cliccato successivo
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_nextPoint(object sender, EventArgs e)
        {
            ButtonNextClicked?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Genera evento dopo aver cliccato pausa monitoring
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_pauseMonitoring(object sender, EventArgs e)
        {
            ButtonPauseMonitoringClicked?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Genera evento dopo aver cliccato sovrascrivi
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_overwritePoints(object sender, EventArgs e)
        {
            ButtonOverwriteClicked?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Genera evento dopo aver cliccato vai a
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_goToPoint(object sender, EventArgs e)
        {
            ButtonGoToClicked?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Genera evento dopo aver cliccato cancella
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_deletePoint(object sender, EventArgs e)
        {
            ButtonDeleteClicked?.Invoke(this, EventArgs.Empty);
        }

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

        #endregion
    }
}
