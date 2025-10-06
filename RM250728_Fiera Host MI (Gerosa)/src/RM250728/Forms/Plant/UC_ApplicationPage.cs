using RM.Properties;
using RM.src.RM250728.Forms.DragMode;
using RMLib.DataAccess;
using RMLib.Keyboards;
using RMLib.Logger;
using RMLib.MessageBox;
using RMLib.PLC;
using RMLib.Translations;
using RMLib.Utils;
using System;
using System.Collections.Generic;
using System.Data;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RM.src.RM250728
{
    /// <summary>
    /// Raccoglie le applicazione disponibili da caricare
    /// </summary>
    public partial class UC_ApplicationPage : UserControl
    {
        #region Proprietà di UC_ApplicationPage

        private static readonly log4net.ILog log = LogHelper.GetLogger(); // Dichiariazione Logger

        #region Variabili per la connessione al database
        private static readonly RobotDAOSqlite robotDAO = new RobotDAOSqlite();
        private static readonly SqliteConnectionConfiguration DatabaseConnection = new SqliteConnectionConfiguration();
        private static readonly string ConnectionString = DatabaseConnection.GetConnectionString();
        #endregion

        // DataTable utilizzata per filtrare la listview delle applicazioni disponibili
        readonly DataTable dt_filtered = new DataTable();

        // Booleano che gestisce attivazione e disattivazione filtri
        private bool IsFilterEnabled = false;

        /// <summary>
        /// Segnala il cambio di un'applicazione
        /// </summary>
        public static event EventHandler selectedApplicationChanged;

        #endregion

        /// <summary>
        /// Costruttore
        /// </summary>
        public UC_ApplicationPage()
        {
            InitializeComponent();

            FormHomePage.Instance.LabelHeader = "APPLICAZIONI";
            InitFont();
            SetHeight(lw_Applications, 30);
            BuildLw_Applications();
            FillLw_Applications();
            BuildAndFill_Dt_filtered();

            // Collegamento evento ApplicationAdded del dizionario al metodo HandleDictionaryDataAdded
            ApplicationConfig.applicationsManager.ApplicationAdded += HandleDictionaryDataAdded;

            // Collegamento evento ApplicationAdded del dizionario al metodo HandleDictionaryApplicationDeleted
            ApplicationConfig.applicationsManager.ApplicationDeleted += HandleDictionaryApplicationDeleted;

            // Collegamento evento ApplicationIdUpdated del dizionario al metodo HandleDictionaryApplicationIdUpdated
            ApplicationConfig.applicationsManager.ApplicationIdUpdated += HandleDictionaryApplicationIdUpdated;

            // Collegamento evento ApplicationNameUpdated del dizionario al metodo HandleDictionaryApplicationNameUpdated
            ApplicationConfig.applicationsManager.ApplicationNameUpdated += HandleDictionaryApplicationNameUpdated;

            // Collegamento evento ApplicationNoteUpdated del dizionario al metodo HandleDictionaryApplicationNoteUpdated
            ApplicationConfig.applicationsManager.ApplicationNoteUpdated += HandleDictionaryApplicationNoteUpdated;
        }

        #region Metodi di UC_ApplicationPage

        /// <summary>
        /// (TODO) Mostra caricamento pagina
        /// </summary>
        public void ShowLoadingScreen()
        {

        }

        /// <summary>
        /// Metodo che intercetta evento di aggiunta applicazione e refrsha la lw delle applicazioni
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryDataAdded(object sender, RobotDictionaryChangedEventArgs e)
        {
            DateTime dateTime;
            string formattedDate;

            dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(e.Application.creation)).DateTime.ToLocalTime();
            formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

            ListViewItem item = new ListViewItem(e.Application.id.ToString());

            item.SubItems.Add(e.Key);
            item.SubItems.Add("");
            item.SubItems.Add(formattedDate);
            item.SubItems.Add(formattedDate);

            lw_Applications.Items.Add(item);
        }

        /// <summary>
        /// Metodo che intercetta il comando UPDATE NOTA della ricetta e aggiorna campo nota e lastUpdateTime nella listView delle ricette
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryApplicationNoteUpdated(object sender, RobotDictionaryChangedEventArgs e)
        {
            lw_Applications.SelectedItems[0].SubItems[2].Text = e.StringValue;
            lw_Applications.SelectedItems[0].SubItems[4].Text = e.FormattedDate;
        }

        /// <summary>
        /// Metodo che intercetta il comando DELETE dell'applicazione e rimuove dalla
        /// listView delle applicazioni l'applicazione selezionata
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryApplicationDeleted(object sender, RobotDictionaryChangedEventArgs e)
        {
            lw_Applications.Items.Remove(lw_Applications.SelectedItems[0]);
        }

        /// <summary>
        /// Metodo che intercetta il comando UPDATE ID dell'applicazione e aggiorna campo id nella List view delle applicazioni
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryApplicationIdUpdated(object sender, RobotDictionaryChangedEventArgs e)
        {
            lw_Applications.SelectedItems[0].SubItems[0].Text = e.Id.ToString();
            lw_Applications.SelectedItems[0].SubItems[4].Text = e.FormattedDate;
        }

        /// <summary>
        /// Metodo che intercetta il comando UPDATE NAME dell'applicazione e aggiorna campo name nella listView delle applicazioni
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryApplicationNameUpdated(object sender, RobotDictionaryChangedEventArgs e)
        {
            lw_Applications.SelectedItems[0].SubItems[1].Text = e.StringValue;
            lw_Applications.SelectedItems[0].SubItems[4].Text = e.FormattedDate;
        }

        /// <summary>
        /// Metodo che costruisce ListView delle applicazioni
        /// </summary>
        private void BuildLw_Applications()
        {
            lw_Applications.Columns.Add("ID", 100);
            lw_Applications.Columns.Add("Nome applicazione", 187);
            lw_Applications.Columns.Add("Note applicazione", 210);
            lw_Applications.Columns.Add("Data creazione", 245);
            lw_Applications.Columns.Add("Ultima modifica", 245);

            // Imposta la proprietà OwnerDraw su true
            lw_Applications.OwnerDraw = true;

            // Gestisci l'evento DrawColumnHeader
            lw_Applications.DrawColumnHeader += new DrawListViewColumnHeaderEventHandler(lw_Applications_DrawColumnHeader);
            // Gestisci l'evento DrawItem
            lw_Applications.DrawItem += new DrawListViewItemEventHandler(lw_Applications_DrawItem);
            // Gestisci l'evento DrawSubItem
            lw_Applications.DrawSubItem += new DrawListViewSubItemEventHandler(lw_Applications_DrawSubItem);
        }

        /// <summary>
        /// Metodo che riempie ListView Applications
        /// </summary>
        private void FillLw_Applications()
        {
            DataTable applications = robotDAO.GetRobotApplications(ConnectionString);

            foreach (DataRow app in applications.Rows)
            {
                DateTime dateTime;
                string formattedDate;

                if (app["ApplicationName"].ToString() == "RM")
                    continue;

                ListViewItem item = new ListViewItem(app["id"].ToString());

                item.SubItems.Add(app["ApplicationName"].ToString());
                item.SubItems.Add(app["note"].ToString());

                // Formatto la data da secondi a data
                dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(app["creation_time"].ToString())).DateTime.ToLocalTime();
                formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");
                item.SubItems.Add(formattedDate);

                dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(app["last_update_time"].ToString())).DateTime.ToLocalTime();
                formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");
                item.SubItems.Add(formattedDate);

                lw_Applications.Items.Add(item);
            }
        }

        /// <summary>
        /// Metodo che costruisce e riempie DataTable di supporto per filtrare ListView
        /// </summary>
        private void BuildAndFill_Dt_filtered()
        {
            dt_filtered.Columns.Add("ID", Type.GetType("System.Int32"));
            dt_filtered.Columns.Add("ApplicationName", Type.GetType("System.String"));
            dt_filtered.Columns.Add("Note", Type.GetType("System.String"));
            dt_filtered.Columns.Add("Creation", Type.GetType("System.String"));
            dt_filtered.Columns.Add("Last update", Type.GetType("System.String"));

            Update_Dt_filtered();
        }

        /// <summary>
        /// Metodo usato per aggiornare la datatable per filtrare la listview, chiamato quando viene modificata la list view a
        /// seguito di un add, delete, import o qualsiasi modifica alle sue righe
        /// </summary>
        private void Update_Dt_filtered()
        {
            dt_filtered.Rows.Clear();// tolgo le precedenti righe della lista
            foreach (ListViewItem item in lw_Applications.Items) // aggiungo le nuove righe prendendole dalla list view originale
            {
                DataRow row = dt_filtered.NewRow();
                row["ID"] = Convert.ToInt32(item.SubItems[0].Text);
                row["ApplicationName"] = item.SubItems[1].Text;
                row["Note"] = item.SubItems[2].Text;
                row["Creation"] = item.SubItems[3].Text;
                row["Last update"] = item.SubItems[4].Text;

                dt_filtered.Rows.Add(row);
            }
        }

        /// <summary>
        /// Metodo che setta Font
        /// </summary>
        private void InitFont()
        {

        }

        /// <summary>
        /// Serve per aumentare la grandezza delle righe di una list view.
        /// NB: può solo aumentare la grandezza e bisogna stare attenti al SO perchè è in base a quello che
        /// cambia l'altezza delle righe.
        /// </summary>
        /// <param name="listView">riferimento alla lw</param>
        /// <param name="height">altezza supplementare specificata</param>
        private void SetHeight(ListView listView, int height)
        {
            ImageList imgList = new ImageList
            {
                ImageSize = new System.Drawing.Size(1, height)
            };
            listView.SmallImageList = imgList;
        }

        /// <summary>
        /// Toglie i filtri applicati in precedenza mettendone alcuni generici che mostrano tutti i risultati
        /// </summary>
        private void RemoveFilters()
        {
            string filterExpression = string.Empty;

            filterExpression += $"ID >= {0}";

            DataTable filteredDt; // rimozione filtri da dt filtered
            try
            {
                filteredDt = dt_filtered.Select(filterExpression).CopyToDataTable();
            }
            catch (Exception)
            {
                filteredDt = new DataTable();
            }
            if (lw_Applications.SelectedItems.Count > 0)
            {
                // aggiornamento list view con tutti gli elementi
                string selectedItemID = lw_Applications.SelectedItems[0].SubItems[0].Text; // vecchio elemento selezionato

                lw_Applications.Items.Clear();
                // Crea una lista di oggetti ListViewItem
                List<ListViewItem> itemsToAdd = new List<ListViewItem>();

                foreach (DataRow row in filteredDt.Rows) // ri aggiunta in lw degli elementi non filtrati da backup
                {
                    ListViewItem item = new ListViewItem(row[0].ToString()); // Valore della prima colonna
                    for (int i = 1; i < filteredDt.Columns.Count; i++)
                    {
                        item.SubItems.Add(row[i].ToString()); // Aggiungi i valori delle altre colonne come sotto-elementi
                    }
                    itemsToAdd.Add(item);
                }

                // Aggiungi tutti gli elementi alla ListView in un'unica operazione

                lw_Applications.Items.AddRange(itemsToAdd.ToArray());

                // vado a selezionare il vecchio elemento scorrendo la lw
                foreach (ListViewItem item in lw_Applications.Items)
                {
                    if (item.SubItems[0].Text.Equals(selectedItemID))
                    {
                        item.Selected = true;
                        break;
                    }
                }
            }
            else
            {
                lw_Applications.Items.Clear();
                // Crea una lista di oggetti ListViewItem
                List<ListViewItem> itemsToAdd = new List<ListViewItem>();

                foreach (DataRow row in filteredDt.Rows) // ri aggiunta in lw degli elementi non filtrati da backup
                {
                    ListViewItem item = new ListViewItem(row[0].ToString()); // Valore della prima colonna
                    for (int i = 1; i < filteredDt.Columns.Count; i++)
                    {
                        item.SubItems.Add(row[i].ToString()); // Aggiungi i valori delle altre colonne come sotto-elementi
                    }
                    itemsToAdd.Add(item);
                }

                // Aggiungi tutti gli elementi alla ListView in un'unica operazione

                lw_Applications.Items.AddRange(itemsToAdd.ToArray());
            }

            Tb_note_filter.Visible = false;
            Tb_name_filter.Visible = false;
            Ud_min_id_filter.Visible = false;
            Ud_max_id_filter.Visible = false;
            Btn_max_id_filter.Visible = false;
            Btn_min_id_filter.Visible = false;
            lb_Name.Visible = false;
            lb_Note.Visible = false;
            Bttn_AvailableChoiches_EnableFilter.BackgroundImage = Resources.filter_filled;
            IsFilterEnabled = false;
        }

        /// <summary>
        /// Applica i filtri derivati dalle textbox e dai numericUpDown alla list view sottostante
        /// </summary>
        private void ApplyFilters()
        {
            string filterExpression = string.Empty;

            decimal Min = Ud_min_id_filter.Value;
            decimal Max = Ud_max_id_filter.Value;

            if ((Min != 0 || Max != 0) && Max >= Min)  // Se gli id min e max sono a 0 allora non li filtro
            {
                // Verifica il campo min id ha un valore
                if (!string.IsNullOrWhiteSpace(Ud_min_id_filter.Text))
                {
                    int minId = Convert.ToInt32(Ud_min_id_filter.Value);
                    filterExpression += $"ID >= {minId}";
                    //filterExpression += $"ID >= {Ud_min_id_filter.Value}";
                }

                // Aggiungi l'operatore logico "AND" se è presente anche il filtro per max id
                if (!string.IsNullOrWhiteSpace(Ud_max_id_filter.Text))
                {
                    int maxId = Convert.ToInt32(Ud_max_id_filter.Value);
                    if (!string.IsNullOrWhiteSpace(filterExpression))
                        filterExpression += " AND ";
                    filterExpression += $"ID <= {maxId}";
                }
            }

            // Aggiungi l'operatore logico "AND" se è presente anche il filtro per Note
            if (!string.IsNullOrWhiteSpace(Tb_note_filter.Text))
            {
                if (!(string.IsNullOrWhiteSpace(filterExpression)))
                    filterExpression += " AND ";
                filterExpression += $"Note LIKE '%{Tb_note_filter.Text}%'";
            }

            // Aggiungi l'operatore logico "AND" se è presente anche il filtro per Name
            if (!(string.IsNullOrWhiteSpace(Tb_name_filter.Text)))
            {
                if (!(string.IsNullOrWhiteSpace(filterExpression)))
                    filterExpression += " AND ";
                filterExpression += $"ApplicationName LIKE '%{Tb_name_filter.Text}%'";
            }

            DataTable filteredDt;
            try
            {
                filteredDt = dt_filtered.Select(filterExpression).CopyToDataTable();
            }
            catch (Exception)
            {
                filteredDt = new DataTable();
            }
            lw_Applications.Items.Clear();
            // Crea una lista di oggetti ListViewItem
            List<ListViewItem> itemsToAdd = new List<ListViewItem>();

            foreach (DataRow row in filteredDt.Rows)
            {
                ListViewItem item = new ListViewItem(row[0].ToString()); // Valore della prima colonna
                for (int i = 1; i < filteredDt.Columns.Count; i++)
                {
                    item.SubItems.Add(row[i].ToString()); // Aggiungi i valori delle altre colonne come sotto-elementi
                }
                itemsToAdd.Add(item);
            }

            // Aggiungi tutti gli elementi alla ListView in un'unica operazione

            lw_Applications.Items.AddRange(itemsToAdd.ToArray());
        }

        #endregion

        #region Eventi di UC_ApplicationPage

        /// <summary>
        /// Disegno elemento
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void lw_Applications_DrawItem(object sender, DrawListViewItemEventArgs e)
        {
            e.DrawDefault = true; // Disegna l'elemento in modo predefinito
        }

        /// <summary>
        /// Disegno sub-elemento
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void lw_Applications_DrawSubItem(object sender, DrawListViewSubItemEventArgs e)
        {
            e.DrawDefault = true; // Disegna il sub-elemento in modo predefinito
        }

        /// <summary>
        /// Ritorno a casa
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_Home_Click(object sender, EventArgs e)
        {
            FormHomePage.Instance.LabelHeader = TranslationManager.GetTranslation("LBL_HOMEPAGE_HEADER");
            FormHomePage.Instance.PnlContainer.Controls["UC_HomePage"].BringToFront();
            FormHomePage.Instance.PnlContainer.Controls.Remove(Controls["UC_ApplicationPage"]);
        }

        /// <summary>
        /// Aggiunta applicazione
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_addApplication(object sender, EventArgs e)
        {
            // Nome nuova applicazione
            string newApplicationName;

            // Utilizzo della tastiera String
            using (VK_string virtualKeyboard_string = new VK_string("", false))
            {
                if (virtualKeyboard_string.ShowDialog() == DialogResult.OK)
                {
                    // Get del nome della nuova applicazione dal risultato della tastiera
                    newApplicationName = virtualKeyboard_string.GetResultString();

                    // Se il nome della applicazione è già presente nel dizionario delle applicazioni
                    // annullo l'operazione e stampo un messaggio di errore
                    if (ApplicationConfig.applicationsManager.getDictionary().ContainsKey(newApplicationName))
                    {
                        CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Applicazione già presente");
                        log.Error("Nome applicazione già presente nel database");
                    }
                    else
                    {
                        // Get applicationName
                        string applicationName = newApplicationName;

                        SqliteConnectionConfiguration databaseConnection = new SqliteConnectionConfiguration();
                        string connectionString = databaseConnection.GetConnectionString();

                        // Istanzio DAO per esecuzione della query GetRobotApplications
                        RobotDAOSqlite DAO = new RobotDAOSqlite();

                        // Ottieni la data e l'ora attuali
                        DateTime now = DateTime.Now;

                        // Calcola il timestamp Unix in millisecondi
                        long unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();

                        if (CustomMessageBox.Show(MessageBoxTypeEnum.WARNING, "Aggiungere " + applicationName + " ?") == DialogResult.OK)
                        {
                            // Aggiorno la tabella delle applicazioni
                            DAO.AddRobotApplication(connectionString, applicationName, unixTimestamp);

                            Update_Dt_filtered();

                            CustomMessageBox.Show(MessageBoxTypeEnum.INFO, "Caricamento completato");
                            log.Info("Caricamento completato");

                        }
                    }
                }


            }
        }

        /// <summary>
        /// Cancella applicazione
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_deleteApplication(object sender, EventArgs e)
        {

            // se non ci sono elementi selezionati
            if (lw_Applications.SelectedItems.Count < 1)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Nessuna applicazione da cancellare selezionata");
                log.Error("Nessuna applicazione da cancellare selezionata");
            }
            else
            {
                DialogResult result = CustomMessageBox.Show(MessageBoxTypeEnum.WARNING, "Cancellare l'applicazione selezionata?");

                if (result == DialogResult.OK)
                {
                    // Get applicationName
                    string applicationName = lw_Applications.SelectedItems[0].SubItems[1].Text;

                    if (string.Equals(applicationName, RobotManager.applicationName))
                    {
                        UC_HomePage uc = (UC_HomePage)FormHomePage.Instance.PnlContainer.Controls["UC_HomePage"];

                        RobotManager.applicationName = string.Empty;
                        uc.SetApplicationToExecute("Selezionare un'applicazione");
                    }

                    //RemoveFilters();

                    // Istanzio DAO per esecuzione della query DeleteRobotApplication
                    RobotDAOSqlite DAO = new RobotDAOSqlite();

                    DAO.DeleteRobotApplication(ConnectionString, applicationName);

                    log.Info("Operazione di cancellazione completata");
                }
                else if (result == DialogResult.No)
                {
                    log.Info("Operazione di cancellazione applicazion annullata");
                }
            }
            //Update_Dt_filtered();
        }

        /// <summary>
        /// Aggiornamento ID
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_EditID(object sender, EventArgs e)
        {
            // se non ci sono elementi selezionati
            if (lw_Applications.SelectedItems.Count < 1)
            {
                //"Nessun elemento selezionato per modificare Id applicazione"
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Nessun elemento selezionato per modificare Id applicazione");
                log.Error("Nessun elemento selezionato per modificare Id applicazione");
            }
            else
            {
                try
                {
                    // Get applicationName
                    string applicationName = lw_Applications.SelectedItems[0].SubItems[1].Text;

                    //apertura tastiera
                    string newApplicationId = VK_Manager.OpenIntVK("0");
                    if (newApplicationId.Equals(VK_Manager.CANCEL_STRING))
                    {
                        return;
                    }

                    bool isContained = false;

                    // Scorro il dizionario per controllare se il nuovo id inserito è già presente 
                    foreach (RobotApplication value in ApplicationConfig.applicationsManager.getDictionary().Values)
                    {
                        if (string.Equals(newApplicationId, value.id.ToString()))
                            isContained = true;
                    }

                    // Se non è presente
                    if (!isContained)
                    {
                        // Check su validità del valore
                        Int32 i = 0;
                        string s = newApplicationId;
                        bool result = Int32.TryParse(s, out i);

                        // Se è un valore valido
                        if (result)
                        {

                            SqliteConnectionConfiguration databaseConnection = new SqliteConnectionConfiguration();
                            string connectionString = databaseConnection.GetConnectionString();

                            // Istanzio DAO
                            RobotDAOSqlite DAO = new RobotDAOSqlite();

                            // Ottieni la data e l'ora attuali
                            DateTime now = DateTime.Now;

                            // Calcola il timestamp Unix in millisecondi
                            long unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();

                            // Get id
                            int id = Convert.ToInt32(newApplicationId);

                            if (MessageBox.Show("Modificare ID applicazione selezionata in " + id + " ?", "modify ID",
                                MessageBoxButtons.OKCancel, MessageBoxIcon.Warning) == DialogResult.OK)
                            {
                                DAO.UpdateRobotApplicationId(connectionString, applicationName, id, unixTimestamp);
                            }
                        }
                        else
                        {
                            CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Il valore Id inserito non è valido");

                            log.Error("Il valore Id inserito non è valido");
                        }
                    }
                    else
                    {
                        CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Id applicazione già presente nella lista delle applicazioni");
                        log.Error("Id applicazione già presente nella lista delle applicazioni");
                    }

                }
                catch (Exception ex)
                {
                    log.Error("Errore nella modifica Id dell'applicazione: " + ex.ToString());
                }
            }
        }

        /// <summary>
        /// Aggiornamento nome
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_rename(object sender, EventArgs e)
        {

            // se non ci sono elementi selezionati
            if (lw_Applications.SelectedItems.Count < 1)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Nessun elemento selezionato per modificare nome applicazione");
                log.Error("Nessun elemento selezionato per modificare nome applicazione");
            }
            else
            {
                try
                {
                    // Get applicationName
                    string applicationName = lw_Applications.SelectedItems[0].SubItems[1].Text;

                    string newApplicationName = VK_Manager.OpenStringVK("", false);
                    if (newApplicationName.Equals(VK_Manager.CANCEL_STRING))
                    {
                        return;
                    }

                    bool isContained = false;

                    // Scorro il dizionario per vedere se è già presente quell'applicazione
                    foreach (string key in ApplicationConfig.applicationsManager.getDictionary().Keys)
                    {
                        if (string.Equals(newApplicationName, key))
                            isContained = true;
                    }

                    // Se non è presente
                    if (!isContained)
                    {

                        SqliteConnectionConfiguration databaseConnection = new SqliteConnectionConfiguration();
                        string connectionString = databaseConnection.GetConnectionString();

                        // Istanzio DAO per esecuzione della query GetRecipes
                        RobotDAOSqlite DAO = new RobotDAOSqlite();

                        // Ottieni la data e l'ora attuali
                        DateTime now = DateTime.Now;

                        // Calcola il timestamp Unix in millisecondi
                        long unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();

                        if (CustomMessageBox.Show(MessageBoxTypeEnum.WARNING, "Modificare nome applicazione selezionata in " + newApplicationName + " ?") == DialogResult.OK)
                        {
                            DAO.UpdateRobotApplicationName(connectionString, applicationName, newApplicationName, unixTimestamp);
                        }
                    }
                    else
                    {
                        CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Nome applicazione già presente nella lista delle applicazioni");
                        log.Error("Nome applicazione già presente nella lista delle applicazioni");
                    }

                }
                catch (Exception ex)
                {
                    log.Error("Errore nella modifica del nome dell'applicazione: " + ex.ToString());
                }
            }
        }

        /// <summary>
        /// Aggiornamento note
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_EditNote(object sender, EventArgs e)
        {

            // se non ci sono elementi selezionati
            if (lw_Applications.SelectedItems.Count < 1)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Nessun elemento selezionato per modificare nota applicazione");
                log.Error("Nessun elemento selezionato per modificare nota applicazione");
            }
            else
            {
                try
                {
                    // Get applicationName
                    string applicationName = lw_Applications.SelectedItems[0].SubItems[1].Text;

                    string newNotes = VK_Manager.OpenStringVK("", false);
                    if (newNotes.Equals(VK_Manager.CANCEL_STRING))
                    {
                        return;
                    }

                    SqliteConnectionConfiguration databaseConnection = new SqliteConnectionConfiguration();
                    string connectionString = databaseConnection.GetConnectionString();

                    // Istanzio DAO per esecuzione della query GetRecipes
                    RobotDAOSqlite DAO = new RobotDAOSqlite();

                    // Ottieni la data e l'ora attuali
                    DateTime now = DateTime.Now;

                    // Calcola il timestamp Unix in millisecondi
                    long unixTimestamp = ((DateTimeOffset)now).ToUnixTimeMilliseconds();

                    // Get id
                    string note = newNotes;

                    if (CustomMessageBox.Show(MessageBoxTypeEnum.WARNING, "Modificare le note dell'applicazione selezionata in " + note + " ?") == DialogResult.OK)
                    {
                        // RemoveFilters();
                        DAO.UpdateRobotApplicationNote(connectionString, applicationName, note, unixTimestamp);
                        // Update_Dt_filtered();
                    }
                }
                catch (Exception ex)
                {
                    log.Error("Errore nella modifica nota della ricetta: " + ex.ToString());
                }
            }
        }

        /// <summary>
        /// Attivazione filtri
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Bttn_AvailableChoiches_EnableFilter_Click(object sender, EventArgs e)
        {
            IsFilterEnabled = !IsFilterEnabled;
            if (IsFilterEnabled)
            {
                Tb_note_filter.Visible = true;
                Tb_name_filter.Visible = true;
                Ud_min_id_filter.Visible = true;
                Ud_max_id_filter.Visible = true;
                Btn_max_id_filter.Visible = true;
                Btn_min_id_filter.Visible = true;
                lb_Name.Visible = true;
                lb_Note.Visible = true;
                Bttn_AvailableChoiches_EnableFilter.BackgroundImage = Resources.remove_filter_filled1;
                ApplyFilters();
            }
            else
            {
                RemoveFilters();
            }
        }

        /// <summary>
        /// Filtro su note
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Tb_note_filter_TextChanged(object sender, EventArgs e)
        {
            ApplyFilters();
        }

        /// <summary>
        /// Filtro su nome
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Tb_name_filter_TextChanged(object sender, EventArgs e)
        {
            ApplyFilters();
        }

        /// <summary>
        /// Filtro su ID
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Ud_min_id_filter_ValueChanged(object sender, EventArgs e)
        {
            ApplyFilters();
        }

        /// <summary>
        /// Filtro su ID
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Ud_max_id_filter_ValueChanged(object sender, EventArgs e)
        {
            ApplyFilters();
        }

        /// <summary>
        /// Inserimento note da filtrare
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Tb_note_filter_Click(object sender, EventArgs e)
        {
            string vkText = VK_Manager.OpenStringVK("", false);
            if (vkText.Equals(VK_Manager.CANCEL_STRING)) return;
            Tb_note_filter.Text = vkText;
        }

        /// <summary>
        /// Inserimento nome da filtrare
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Tb_name_filter_Click(object sender, EventArgs e)
        {
            string vkText = VK_Manager.OpenStringVK("", false);
            if (vkText.Equals(VK_Manager.CANCEL_STRING)) return;
            Tb_name_filter.Text = vkText;
        }

        /// <summary>
        /// Inserimento ID da filtrare
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Btn_min_id_filter_Click(object sender, EventArgs e)
        {
            string newID = VK_Manager.OpenIntVK("0");
            if (newID.Equals(VK_Manager.CANCEL_STRING)) return;
            Ud_min_id_filter.Text = newID;
        }

        /// <summary>
        /// Inserimento ID da filtrare
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Btn_max_id_filter_Click(object sender, EventArgs e)
        {
            string newID = VK_Manager.OpenIntVK("0");
            if (newID.Equals(VK_Manager.CANCEL_STRING)) return;
            Ud_max_id_filter.Text = newID;
        }

        /// <summary>
        /// Caricamento applicazione
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClickEvent_loadProgram(object sender, EventArgs e)
        {
            if (lw_Applications.Items.Count < 1)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Aggiungere un'applicazione per eseguire il caricamento");
                return;
            }

            if (lw_Applications.SelectedItems.Count < 1)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Selezionare un'applicazione per eseguire il caricamento");
                return;
            }

            UC_HomePage uc = (UC_HomePage)FormHomePage.Instance.PnlContainer.Controls["UC_HomePage"];
            string applicationName = lw_Applications.SelectedItems[0].SubItems[1].Text;
            RobotManager.applicationName = applicationName;
            uc.SetApplicationToExecute(applicationName);

            selectedApplicationChanged?.Invoke(null, EventArgs.Empty);

            UC_FullDragModePage.debugCurrentIndex = -1;
            UC_HomePage.index = 0;

            CustomMessageBox.Show(MessageBoxTypeEnum.INFO, "Applicazione inviata con successo");
            log.Info("Applicazione inviata con successo");


        }

        /// <summary>
        /// Disegno colonne listView
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void lw_Applications_DrawColumnHeader(object sender, DrawListViewColumnHeaderEventArgs e)
        {
            using (Font headerFont = new Font("Microsoft Sans Serif", 12, FontStyle.Bold | FontStyle.Italic))
            {
                e.Graphics.FillRectangle(SystemBrushes.ControlDarkDark, e.Bounds);
                e.Graphics.DrawRectangle(Pens.White, e.Bounds);

                StringFormat sf = new StringFormat
                {
                    Alignment = StringAlignment.Center,
                    LineAlignment = StringAlignment.Center
                };

                e.Graphics.DrawString(e.Header.Text, headerFont, Brushes.White, e.Bounds, sf);
            }
        }

        #endregion
    }
}
