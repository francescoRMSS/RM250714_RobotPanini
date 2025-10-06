using RMLib.Logger;
using RMLib.MessageBox;
using RMLib.Translations;
using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading;
using System.Windows.Forms;

namespace RM.src.RM250728.Forms.Plant
{
    /// <summary>
    /// Rappresenta una form per la selezione delle lingue per cambiare la traduzione degli elementi della UI.
    /// <para>Cliccando al di fuori del panel principale o fuori dalla form, quest'ultima si deve chiudere.</para>
    /// </summary>
    public partial class FormLanguageSelection : Form
    {
        #region Proprietà della classe

        /// <summary>
        /// Definisce l'opacità e quindi la percentuale di trasparenza massima della form.
        /// </summary>
        private const double MAX_OPACITY = 0.95;
        /// <summary>
        /// Coordinata predefinita X di partenza della posizione
        /// </summary>
        private const int defalutPositionX = 0;
        /// <summary>
        /// Coordinata predefinita Y di partenza della posizione
        /// </summary>
        private const int defaultPositionY = 0;
        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();
        /// <summary>
        /// Il tag della lingua selezionata
        /// </summary>
        private string selectedLanuageTag = string.Empty;

        #endregion

        #region Metodi della form

        public FormLanguageSelection()
        {
            InitializeComponent();
            //posizione
            Location = new Point(defalutPositionX, defaultPositionY);
            //opacity di default 0
            Opacity = MAX_OPACITY;
            Thread.Sleep(10);
            //animazione in
            //AnimateFormOpacity(true); //animazione in entrata eliminata, parte direttamente con opacità giusta
            log.Info("LngsForm: aperta");
        }

        /// <summary>
        /// Animazione semplice che aumenta o decrementa progressivamente l'opacità della form
        /// </summary>
        /// <param name="opening"></param>
        private void AnimateFormOpacity(bool opening)
        {
            if (opening)
            {
                for(double i = 0.00; i < MAX_OPACITY; i += 0.01)
                {
                    Opacity = i;
                    Thread.Sleep(1);
                }
            } 
            else
            {
                for (double i = MAX_OPACITY; i > 0; i -= 0.01)
                {
                    Opacity = i;
                    Thread.Sleep(1);
                }
            }
        }

        /// <summary>
        /// Chiude la form
        /// </summary>
        private void CloseForm()
        {
            log.Info("LngsForm: richiesta chiusura pagina");
            //animazione out
            AnimateFormOpacity(false);
            Close(); 
        }

        /// <summary>
        /// Mostra quando necessario il panel per il message box fisico
        /// </summary>
        /// <param name="flagIcon"></param>
        private void ShowMsgBox(Image flagIcon)
        {
            if (!TranslationManager.actualLanguage.Equals(selectedLanuageTag))
            {
                lbl_msg.Text = TranslationManager.GetTranslation("MSG_CHANGE_LANGUAGE");
                pb_languageIcon.Image = flagIcon;
                pnl_confirmMsgBox.Visible = true;
            }
        }

        /// <summary>
        /// Metodo che applica i cambiamenti alla lingua selezionata se l'utente conferma la scelta, altrimenti chiude la form
        /// </summary>
        /// <param name="confirmation"></param>
        /// <param name="languageTag"></param>
        private void ChangeLanguage(bool confirmation)
        {
            if (string.IsNullOrEmpty(selectedLanuageTag)) 
                return;
            if (confirmation)
            {
                TranslationManager.UpdateDefaultLanguage(selectedLanuageTag);

                Global.shouldReset = true;

                // Ottieni il percorso dell'eseguibile dell'applicazione corrente
                string appPath = Application.ExecutablePath;

                // Chiudi l'applicazione corrente
                Application.Exit();

                // Riavvia l'applicazione
                Process.Start(appPath); // vedere perchè non va
                //Process.Start(appPath); // Avvia una nuova istanza

                Process.GetCurrentProcess().Kill(); //aspetta che i thread termino e libera le risorse 
            }
            else
            {
                CloseForm();
            }
        }

        #endregion

        #region Eventi della form

        /// <summary>
        /// Evento scatenato al premere dentro alla form ma al di fuori del panel
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_clickOnForm(object sender, EventArgs e)
        {
            CloseForm();
        }

        /// <summary>
        /// Evento scatenato al premere al di fuori della form
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void LeaveEvent_leaveForm(object sender, EventArgs e)
        {
            CloseForm();
        }

        /// <summary>
        /// Evento scatenato al premere al di fuori della form, deattivandola e quindi portandola in secondo piano
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void DeactivationEvent_deactivatedForm(object sender, EventArgs e)
        {
            string ciao = sender.ToString();
            string i = sender.GetType().ToString();
            Control control = sender as Control;
            string tag = control.Tag as string;
            CloseForm();
        }

        #endregion

        /// <summary>
        /// In base al pulsante cliccato verrà selezionata la lingua corrispondente e verrà fatta la query per il cambio sul db
        /// </summary>
        /// <param name="sender">pulsante che ha generato l'evento</param>
        /// <param name="e">evento generato</param>
        private void ClickEvent_languageSelection(object sender, EventArgs e)
        {
            Button button = sender as Button;
            string tag = button.Tag as string;

            switch (tag) 
            {
                case "ita":
                    log.Info("LngsForm: richiesta traduzione italiana");
                    selectedLanuageTag = "it_IT";
                    ShowMsgBox(button.BackgroundImage);
                    break;
                case "eng":
                    log.Info("LngsForm: richiesta traduzione inglese");
                    selectedLanuageTag = "en_GB";
                    ShowMsgBox(button.BackgroundImage);
                    break;
                case "deu":
                    log.Info("LngsForm: richiesta traduzione tedesca");
                    selectedLanuageTag = "de_DE";
                    ShowMsgBox(button.BackgroundImage);
                    break;
                case "fra":
                    log.Info("LngsForm: richiesta traduzione francese");
                    selectedLanuageTag = "fr_FR";
                    ShowMsgBox(button.BackgroundImage);
                    break;
                case "egy":
                    log.Info("LngsForm: richiesta traduzione araba");
                    selectedLanuageTag = "ar_AR";
                    ShowMsgBox(button.BackgroundImage);
                    break;
                case "chi":
                    log.Info("LngsForm: richiesta traduzione cinese");
                    selectedLanuageTag = "cn_CN";
                    ShowMsgBox(button.BackgroundImage);
                    break;
                default:
                    log.Error("LngsForm: richiesta traduzione inesistente");
                    selectedLanuageTag = string.Empty;
                    CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Errore durante la selezione della traduzione");
                    break;
            }
        }

        private void ClickEvent_confirmLanguageChange(object sender, EventArgs e)
        {
            ChangeLanguage(true);
        }

        private void ClickEvent_denyLanguageChange(object sender, EventArgs e)
        {
            ChangeLanguage(false);
        }
    }
}
