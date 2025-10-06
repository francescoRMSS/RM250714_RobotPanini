using System;
using System.Windows.Forms;

namespace RM.src.RM250728.Forms.ScreenSaver
{
    /// <summary>
    /// Form contenente il media player per mostrare lo screen saver
    /// </summary>
    public partial class VideoPlayer : Form
    {
        #region Proprietà

        /// <summary>
        /// Contiene il full path assoluto al video
        /// </summary>
        private readonly string videoPath;

        /// <summary>
        /// Evento per dire al manager che il video è stato chiuso
        /// </summary>
        public event EventHandler RestartTimer;

        #endregion

        /// <summary>
        /// Costruttore di default con scelta full path video
        /// </summary>
        /// <param name="path"></param>
        public VideoPlayer(string path)
        {
            InitializeComponent();
            videoPath = path;
            TopMost = true;
            InitVideo();
        }

        #region Metodi form

        /// <summary>
        /// Inizializza il media player in modo che sia senza controlli, in loop e parta automaticamente
        /// </summary>
        private void InitVideo()
        {
            //log.Info("Configurazione media player");

            // Nasconde i controlli UI di WMP
            mediaPlayer.uiMode = "none";
            // Volume a 0
            mediaPlayer.settings.volume = 0;
            // Imposta il video a schermo intero
            mediaPlayer.stretchToFit = true;
            // Imposta il video a schermo intero
            //mediaPlayer.fullScreen = true; // da errore
            // Avvia il video in loop infinito
            mediaPlayer.settings.setMode("loop", true);
            // Avvia automaticamente il video
            mediaPlayer.settings.autoStart = false;
            // Fa partire la riproduzione del video
            // mediaPlayer.Ctlcontrols.play();
            // Imposta il percorso del file video
            mediaPlayer.URL = videoPath;
        }

        /// <summary>
        /// Fa partire la riproduzione del video
        /// </summary>
        public void Play()
        {
            //WindowState = FormWindowState.Maximized;
            Show();
            mediaPlayer.Ctlcontrols.play();
        }

        /// <summary>
        /// Ferma la riproduzione in modo che la prossima volta riparta da capo
        /// </summary>
        public void Stop()
        {
            mediaPlayer.Ctlcontrols.stop();
            //WindowState = FormWindowState.Minimized;
            Hide();
        }

        /// <summary>
        /// Distrugge il media player e i suoi riferimenti
        /// </summary>
        public void DisposeElements()
        {
            mediaPlayer.Ctlcontrols.stop();
            mediaPlayer.URL = null;
            mediaPlayer.Dispose();
            mediaPlayer = null;
        }

        #endregion

        #region Eventi form

        private void mediaPlayer_MouseDownEvent(object sender, AxWMPLib._WMPOCXEvents_MouseDownEvent e)
        {
            Stop();
            RestartTimer?.Invoke(this, EventArgs.Empty);
        }

        #endregion
    }
}
