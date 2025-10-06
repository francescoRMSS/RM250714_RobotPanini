using RMLib.Keyboards;
using RMLib.Logger;
using RMLib.MessageBox;
using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;

namespace RM.src.RM250728.Forms.ScreenSaver
{
    /// <summary>
    /// Gestisce la visualizzazione di una form che controlla lo screen saver. Per usare lo screen saver è necessaria la libreria 
    /// wmp.dll, mettendo un media player nel designer di una form, le librerie associate verranno importate automaticamente. 
    /// Per usare la libreria wmp è necessario avere una form sempre aperta che al suo interno abbia il riferimento alla form dello
    /// screen saver. Questo perchè usando una classe statica che istanzia un thread STA e lo distrugge ogni volta porta al crash non
    /// prevedibile dell'applicazione con errore "tentativo di accesso alla memoria protetta" (che significa che la memoria si è corrotta)
    /// perciò l'uso di una form sempre aperta è necessario. 
    /// <br></br>
    /// Una buona idea è quella di istanziare il manager dentro alla form home page (che rimane sempre aperta), impostare file name e 
    /// delay, la creazione dell'oggetto va fatta dentro l'evento "shown" della form, mantenendo una variabile nella classe. 
    /// <br></br>
    /// Una volta fatto bisogna anche aggiungere gli eventi personalizzati che vanno a resettare il timer al click, per farlo
    /// è possibile usare il metodo <code>AutoAddClickEvents(Form form) o (UserControl uc)</code> che automaticamente aggiunge ai 
    /// controlli e i figli di questi un evento click. NB: è necessario anche rimuovere tali eventi con il metodo 
    /// <code>AutoRemoveClickEvents(Form form) o (UserControl uc)</code> quando le pagine vengono chiuse.
    /// <para></para>
    /// 20250422 - V1.0: Creazione per fiera lamiera CEMSA
    /// </summary>
    public partial class ScreenSaverManager : Form
    {
        #region Proprietà
        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();
        /// <summary>
        /// Versione della libreria
        /// </summary>
        public const string Version = "V1.0 - 22/04/2025";
        /// <summary>
        /// Valore numerico di default nel caso si scelga di usare il valore di default per il delay
        /// </summary>
        private const int defaultDelayValue = 10;
        /// <summary>
        /// Calcolo tempo in ms di delay utilizzando il valore di default
        /// </summary>
        private readonly int defaultDelayTime = (int)TimeSpan.FromMinutes(defaultDelayValue).TotalMilliseconds;
        /// <summary>
        /// Path di default usato per passare dal percorso dell'eseguibile al percorso del video
        /// </summary>
        private string videoDefaultDirectory = @"..\..\Resources\";
        /// <summary>
        /// Path calcolato a run time dell'eseguibile
        /// </summary>
        private string exeDefaultDirectory = AppDomain.CurrentDomain.BaseDirectory;
        /// <summary>
        /// Contiene il percorso del file video da calcolare a run time
        /// </summary>
        private string videoPath;
        /// <summary>
        /// Contiene il nome del file mp4 che si desidera mettere a schermo
        /// </summary>
        private string videoFileName = "screenSaver.mp4";
        /// <summary>
        /// Contiene il valore del delay in ms
        /// </summary>
        private int delay;
        /// <summary>
        /// Riferimento alla form dello screen saver
        /// </summary>
        private VideoPlayer videoPlayer = null;
        /// <summary>
        /// Riferimento al timer di system timers usato per mostrare lo screen saver ogni tot tempo
        /// </summary>
        private static Timer timer = null;
        /// <summary>
        /// Riferimento ad un thread in compartimento singolo STA che serve per eseguire e mostrare la form dello screen saver
        /// </summary>
        //private static Thread staThread = null;
        /// <summary>
        /// Indica se il timer è in esecuzione al momento
        ///</summary>
        private static bool currentTimerState = false;
        /// <summary>
        /// La coordinata x da sommare per nascondere la form del manager
        /// </summary>
        private const int hiddenControlsXCoordOffset = 3000;
        /// <summary>
        /// Specifica se usare o meno lo screen saver per l'app
        /// </summary>
        public static bool useScreenSaver = true;

        #endregion

        #region Costruttori di classe

        /// <summary>
        /// Costruttore vuoto che usa i valori di default
        /// </summary>
        public ScreenSaverManager()
        {
            delay = defaultDelayTime;

            InitializeComponent();
            InitView();
        }

        /// <summary>
        /// Costruttore per usare valori arbitrari
        /// </summary>
        /// <param name="delayMs"></param>
        /// <param name="fileName"></param>
        /// <param name="useScreenSaver"></param>
        public ScreenSaverManager(int delayMs, string fileName, bool useScreenSaver)
        {
            delay = delayMs;
            videoFileName = fileName;
            ScreenSaverManager.useScreenSaver = useScreenSaver;

            InitializeComponent();

            if(useScreenSaver)
            {
                InitView(); // inizializzo timer, eventi, form video player, file path
                Show(); // mostro per la prima volta la form
                StartTimer(); // faccio partire direttamente il timer
                HideControls(); // nascondo la form a destra
            }
        }

        #endregion

        #region Metodi form

        /// <summary>
        /// Inizializza la pagina e i controlli
        /// </summary>
        private void InitView()
        {
            SetupVideoFile();
            SetupTimer(delay);
            SetupVideoPlayer();

            lbl_delay.Text = (delay / 1000).ToString();
            lbl_filePath.Text = videoPath;
        }

        /// <summary>
        /// Inizializza gli eventi
        /// </summary>
        private void SetupEvents()
        {
            videoPlayer.RestartTimer += RestartTimer;
        }

        /// <summary>
        /// Inizializza la form media player
        /// </summary>
        private void SetupVideoPlayer()
        {
            if(videoPlayer == null) // prima creazione
            {
                videoPlayer = new VideoPlayer(videoPath);
                SetupEvents();
            } 
            else // cambio file name
            {
                videoPlayer.RestartTimer -= RestartTimer;
                videoPlayer.DisposeElements();
                videoPlayer.Dispose();
                videoPlayer = null;
                GC.Collect();
                GC.SuppressFinalize(this);
                GC.Collect();
                videoPlayer = new VideoPlayer(videoPath);
                SetupEvents();
            }
        }

        /// <summary>
        /// Inizializzazione del timer
        /// </summary>
        /// <param name="delayVal"></param>
        private void SetupTimer(int delayVal)
        {
            if (!currentTimerState)
            {
                timer = null;
                timer = new Timer();
                timer.Interval = delayVal;
                timer.Tick += ShowScreenSaver;
            }
            else
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.WARNING_OK, "Disattivare prima il timer");
            }
        }

        /// <summary>
        /// Inizializzazione e calcolo percorso completo del file video
        /// </summary>
        private void SetupVideoFile()
        { 
            string path;
            // Ottenimento percorso assoluto partendo dal percorso relativo del file .exe e del video
            path = Path.GetFullPath( // Calcolo percorso finale
                            Path.Combine(exeDefaultDirectory, videoDefaultDirectory + videoFileName) // Calcolo percorso file
                        );
            videoPath = path;
        }

        /// <summary>
        /// Controlla che il file video esista
        /// </summary>
        /// <returns></returns>
        private bool CheckFilePresence()
        {
            if (File.Exists(videoPath))
            {
                return true;
            }
            else
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "File non trovato");
                return false;
            }
        }

        /// <summary>
        /// Sposta la form del manager fuori dallo schermo in modo che rimanga comunque attiva
        /// </summary>
        public void HideControls()
        {
            Point currentPosition = Location;
            currentPosition.X += hiddenControlsXCoordOffset;
            Location = currentPosition;
        }

        /// <summary>
        /// Ripristina la posizione della form del manager in modo che torni visibile
        /// </summary>
        public void RestoreLocation()
        {
            Point currentPosition = Location;
            if (currentPosition.X > hiddenControlsXCoordOffset)
                currentPosition.X -= hiddenControlsXCoordOffset;
            Location = currentPosition;
        }

        #endregion

        #region Metodi timer

        /// <summary>
        /// Resetta il timer quando si clicca sul video per chiuderlo
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void RestartTimer(object sender, EventArgs e)
        {
            ResetTimer();
        }

        /// <summary>
        /// Fa partire il timer solo se il video esiste
        /// </summary>
        private void StartTimer()
        {
            if (!CheckFilePresence())
            {
                return;
            }
            timer.Start();
            currentTimerState = true;
            lbl_status.Text = "ON";
        }

        /// <summary>
        /// Ferma il timer e il video
        /// </summary>
        private void StopTimer()
        {
            timer.Stop();
            currentTimerState = false;
            lbl_status.Text = "OFF";

            videoPlayer.Stop();
        }

        /// <summary>
        /// Resetta il timer solo se si ha dato il via dal manager
        /// </summary>
        public static void ResetTimer()
        {
            if (currentTimerState)
            {
                timer.Stop();
                timer.Start();
                currentTimerState = true;
            }
        }

        /// <summary>
        /// Mostra la pagina dello screen saver
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="args"></param>
        private void ShowScreenSaver(object sender, EventArgs args)
        {
            log.Info("Timer scaduto, avvio screen saver");
            videoPlayer.Play();
            timer.Stop();
        }

        /// <summary>
        /// Aggiunge in modo automatico l'evento click ai controlli della form passata come parametro
        /// </summary>
        /// <param name="form"></param>
        public static void AutoAddClickEvents(Form form)
        {
            if(useScreenSaver)
            {
                //log.Info("Richiesta aggiunta automatica degli eventi click per screen saver da: " + form.Name);
                foreach (Control control in form.Controls)
                {
                    control.Click += (sender, e) => ResetTimer();
                    foreach (Control childControl in control.Controls)
                    {
                        childControl.Click += (sender, e) => ResetTimer();
                    }
                }
            }
            else
            {
                //log.Warn("Richiesta aggiunta automatica degli eventi click per screen saver da: " + form.Name + " annullata");
            }
        }

        /// <summary>
        /// Aggiunge in modo automatico l'evento click ai controlli dello user control passato come parametro
        /// </summary>
        /// <param name="UC"></param>
        public static void AutoAddClickEvents(UserControl UC)
        {
            if (useScreenSaver)
            {
                //log.Info("Richiesta aggiunta automatica degli eventi click per screen saver da: " + UC.Name);
                foreach (Control control in UC.Controls)
                {
                    control.Click += (sender, e) => ResetTimer();
                    foreach (Control childControl in control.Controls)
                    {
                        childControl.Click += (sender, e) => ResetTimer();
                    }
                }
            }
            else
            {
                //log.Warn("Richiesta aggiunta automatica degli eventi click per screen saver da: " + UC.Name + " annullata");
            }
        }

        /// <summary>
        /// Rimuove in modo automatico l'evento click dai controlli della form
        /// </summary>
        /// <param name="form"></param>
        public static void AutoRemoveClickEvents(Form form)
        {
            if(useScreenSaver)
            {
                //log.Info("Richiesta rimozione automatica degli eventi click per screen saver da: " + form.Name);
                foreach (Control control in form.Controls)
                {
                    control.Click -= (sender, e) => ResetTimer();
                    foreach (Control childControl in control.Controls)
                    {
                        childControl.Click -= (sender, e) => ResetTimer();
                    }
                }
            }
            else
            {
                //log.Info("Richiesta rimozione automatica degli eventi click per screen saver da: " + form.Name + " annullata");
            } 
        }

        /// <summary>
        /// Rimuove in modo automatico l'evento click dai controlli dello User control
        /// </summary>
        /// <param name="uc"></param>
        public static void AutoRemoveClickEvents(UserControl uc)
        {
            if(useScreenSaver)
            {
                //log.Info("Richiesta rimozione automatica degli eventi click per screen saver da: " + uc.Name);
                foreach (Control control in uc.Controls)
                {
                    control.Click -= (sender, e) => ResetTimer();
                    foreach (Control childControl in control.Controls)
                    {
                        childControl.Click -= (sender, e) => ResetTimer();
                    }
                }
            }
            else
            {
                //log.Info("Richiesta rimozione automatica degli eventi click per screen saver da: " + uc.Name + " annullata");
            }
        }

        #endregion

        #region Eventi form

        private void ClickEvent_startTimer(object sender, EventArgs e)
        {
            if(!currentTimerState)
                StartTimer();
        }

        private void ClickEvent_stopTimer(object sender, EventArgs e)
        {
            if(currentTimerState)
                StopTimer();
        }

        private void ClickEvent_openVersions(object sender, EventArgs e)
        {

        }

        private void ClickEvent_modifyDelay(object sender, EventArgs e)
        {
            if (!currentTimerState)
            {
                int newVal;
                string val = VK_Manager.OpenIntVK("1", 1, 30000);
                if (val != VK_Manager.CANCEL_STRING)
                {
                    newVal = Convert.ToInt32(val);
                    newVal *= 1000; // trasformo in ms
                    lbl_delay.Text = val;

                    SetupTimer(newVal);
                }
            }
            else
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.WARNING_OK, "Disattivare prima il timer");
            }
        }

        private void ClickEvent_modifyPath(object sender, EventArgs e)
        {
            if (!currentTimerState)
            {
                string newFileName = VK_Manager.OpenStringVK(videoFileName, false);
                //log.Warn("Richiesto cambio nome video, nuovo nome: " + newFileName);
                //string oldFileName = videoFileName; // Salvo il vecchio nome del file video
                videoFileName = newFileName;
                lbl_filePath.Text = newFileName;

                SetupVideoFile();
                SetupVideoPlayer();
            }
            else
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.WARNING_OK, "Disattivare prima il timer");
            }
        }

        private void ClickEvent_hideControls(object sender, EventArgs e)
        {
            HideControls();
        }

        #endregion
    }
}
