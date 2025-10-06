using System;
using System.Windows.Forms;

namespace RM.src.RM250728
{
    /// <summary>
    /// Viene utilizzata per le finestre di info relative a comandi e parametri del Robot
    /// </summary>
    public partial class FormInfoRobot : Form
    {
        /// <summary>
        /// Costruttore vuote
        /// </summary>
        public FormInfoRobot()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Costruttore parametrizzato
        /// </summary>
        /// <param name="title">Titolo dell'info</param>
        /// <param name="description">Descrizione dell'info</param>
        public FormInfoRobot(string title, string description)
        {
            InitializeComponent();
            LblTitle.Text = title;
            LblDescription.Text = description;
        }

        #region Eventi di FormInfoRobot
        /// <summary>
        /// Chiude la form
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PnlClose_Click(object sender, EventArgs e)
        {
            Close();
        }
        #endregion
    }
}
