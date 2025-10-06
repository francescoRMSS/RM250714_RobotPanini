using RM.src.RM250728.Forms.ScreenSaver;
using RMLib.Utils;
using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RM.src.RM250728.Forms.Plant.DragMode
{
    /// <summary>
    /// Pagina contenente i controlli per modificare e visualizzare i parametri per la macchina saldatrice
    /// </summary>
    public partial class UC_weldingParameters : UserControl
    {
        #region Proprietà
        #endregion

        #region Metodi

        /// <summary>
        /// Costruisce la pagina UC_weldingParameters
        /// </summary>
        public UC_weldingParameters()
        {
            InitializeComponent();

            ScreenSaverManager.AutoAddClickEvents(this);
        }

        /// <summary>
        /// Chiude la pagina e torna alla home
        /// </summary>
        private void CloseForm()
        {
            FormHomePage.Instance.LabelHeader = "HOME PAGE";
            FormHomePage.Instance.PnlContainer.Controls["UC_HomePage"].BringToFront();
            FormHomePage.Instance.PnlContainer.Controls.Remove(Controls["UC_permissions"]);

            ScreenSaverManager.AutoRemoveClickEvents(this);

            Dispose();
        }

        /// <summary>
        /// Mostra una schermata di caricamento per tot tempo
        /// </summary>
        public async void ShowLoadingScreen()
        {
            pnl_loading.Location = new Point(0, 0);
            await Task.Delay(ProjectVariables.UserControlLoadingScreenTime);
            pnl_loading.Visible = false;
        }

        #endregion

        #region Eventi

        /// <summary>
        /// Torna alla home page chiudendo questa pagina
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_back(object sender, EventArgs e)
        {
            CloseForm();
        }

        #endregion

        /// <summary>
        /// Evento per simulare funzionamento dei tasti
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_colorButton(object sender, EventArgs e)
        {
            /*
            Label senderLbl = new Label();
            if (senderLbl.BackColor == Color.Red)
            {
                senderLbl.BackColor = Color.Green;
            }
            else
            {
                senderLbl.BackColor = Color.Red;
            }*/
        }
    }
}
