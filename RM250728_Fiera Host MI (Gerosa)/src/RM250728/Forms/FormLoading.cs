using System;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using RMLib.MessageBox;
using RMLib.VATView;

namespace RM.src.RM250728
{
    /// <summary>
    /// Definisce la struttura ed il comportamento della Form di loading / Splash screen
    /// </summary>
    public partial class FormLoading : Form
    {
        /// <summary>   
        /// Specifica se la chiusura in corso è stata programmata oppure è a causa di un errore
        /// </summary>
        private bool closeToken = false;

        /// <summary>
        /// Campo per memorizzare gli argomenti
        /// </summary>
        private readonly string[] args;

        /// <summary>
        /// Costruisce la form di caricamento / splash screen
        /// </summary>
        public FormLoading(string[] args)
        {
            // Rimuovo bordi della form
            //FormBorderStyle = FormBorderStyle.None;
            InitializeComponent();
            SetLabelPosition();   
            this.args = args;
        }

        #region Metodi di FormLoading

        /// <summary>
        /// Imposta la progress bar label in mezzo alla progress bar
        /// </summary>
        private void SetLabelPosition()
        {
            int x = (PanelProgressBar.Size.Width - lb_ProgressBar.Size.Width) / 2;
            lb_ProgressBar.Location = new Point(x, lb_ProgressBar.Location.Y);
        }

        /// <summary>
        /// Chiude la form di caricamento ed apre la form di homepage
        /// </summary>
        public void OpenHomePage(FormHomePage homePage)
        {
            Hide();
            try
            {
                //FormHomePage homePage = new FormHomePage();
                homePage.ShowDialog();
                closeToken = true;
            }

            catch (NullReferenceException ex)
            {
                // Gestisci l'eccezione
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Errore: " + ex.Message);
                //MessageBox.Show("Errore: " + ex.Message);
                closeToken = false;
            }
            Close();
        }

        /// <summary>
        /// Apre form VATView
        /// </summary>
        public void OpenVATView()
        {
            FormVATView vatView = new FormVATView();
            vatView.Show();
        }

        /// <summary>
        /// Esegue il metodo per la configurazione iniziale del sistema
        /// </summary>
        private void LoadProgressBar(string[] args)
        { 
            ConfigurationSettings cs = new ConfigurationSettings();
            cs.Setup(lb_ProgressBar, progressBar1, this, args);
        }

        #endregion

        #region Eventi di FormLoading

        /// <summary>
        /// Evento di caricamento della Form.
        /// Lancia metodo setup della classe ConfigurationSettings
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void FormLoading_Load(object sender, EventArgs e)
        {
            LoadProgressBar(args);
        }

        /// <summary>
        /// Evento generato alla chiusura dell'app, termina tutti i thread in modo non-safe e distrugge tutti gli elementi subito.
        /// Metodo aggressivo per la chiusura che risolve il problema dei thread che rimangono in background impedendo la 
        /// riapertura del sw per via della doppia istanza.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClosingEvent_loadingPageClosing(object sender, FormClosingEventArgs e)
        {
            //Application.Exit(); // non basta
            //Environment.Exit(0); // metodo drastico, termina il processo e libera le risorse in questo momento
            if (!closeToken)
                Process.GetCurrentProcess().Kill(); //aspetta che i thread termino e libera le risorse 
        }

        #endregion
    }
}
