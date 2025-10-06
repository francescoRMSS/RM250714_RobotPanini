using fairino;
using RM.src.RM250728.Classes.FR20;
using RM.src.RM250728.Forms.ScreenSaver;
using RMLib.DataAccess;
using RMLib.Keyboards;
using RMLib.Logger;
using RMLib.MessageBox;
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
    /// Pagina dedicata alla lista di posizioni RM
    /// </summary>
    public partial class UC_positions : UserControl
    {
        #region Variabili di connessione database
        private static readonly RobotDAOSqlite robotDAO = new RobotDAOSqlite();
        private static readonly SqliteConnectionConfiguration DatabaseConnection = new SqliteConnectionConfiguration();
        private static readonly string ConnectionString = DatabaseConnection.GetConnectionString();
        #endregion

        #region Parametri di UC_positions
        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        /// <summary>
        /// Tabella di supporto per filtri lw_positions
        /// </summary>
        readonly DataTable dt_filtered = new DataTable();

        /// <summary>
        /// Nome del programma con i punti di RM
        /// </summary>
        public static string RMProgramName = "RM";

        /// <summary>
        /// Indice della colonna ID
        /// </summary>
        public static readonly int ID_COLUMN_INDEX = 0;

        /// <summary>
        /// Indice della colonna name_position
        /// </summary>
        public static readonly int NAME_POSITION_COLUMN_INDEX = 9;
        #endregion

        /// <summary>
        /// Costruttore UC_FullDragModePage
        /// </summary>
        public UC_positions()
        {
            InitializeComponent();
            InitView();
            InitRobotParameters();
            Fill_LwPositions();
            //BuildAndFill_Dt_filtered();

            RobotManager.RecordPoint += RobotManager_UpdatePoint;

            ScreenSaverManager.AutoAddClickEvents(this);
        }

        #region Metodi di UC_positions
        /// <summary>
        /// Modifica nome posizione e timestamp
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryRobotPositionNameUpdated(object sender, RobotDictionaryChangedEventArgs e)
        {
            if (lw_positions.SelectedItems.Count > 0)
            {
                lw_positions.SelectedItems[0].SubItems[9].Text = e.StringValue;
                lw_positions.SelectedItems[0].SubItems[1].Text = e.FormattedDate;
                CustomMessageBox.Show(MessageBoxTypeEnum.INFO, "Posizione rinominata correttamente");
                log.Info("Posizione rinominata correttamente");
            }
        }

        /// <summary>
        /// Rimuove posizione selezionata dalla listview e aggiorna gli ID per mantenerli sequenziali
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryRobotPositionRemoved(object sender, RobotDictionaryChangedEventArgs e)
        {
            if (lw_positions.SelectedItems.Count > 0)
            {
                // Ottieni l'indice dell'elemento selezionato
                int selectedIndex = lw_positions.SelectedIndices[0];

                // Rimuovi l'elemento dalla ListView
                lw_positions.Items.RemoveAt(selectedIndex);

                // Aggiorna gli ID delle voci rimanenti
                for (int i = 0; i < lw_positions.Items.Count; i++)
                {
                    lw_positions.Items[i].SubItems[0].Text = (i + 1).ToString();
                }

                log.Info("Posizione eliminata correttamente");
            }
        }

        private void RobotManager_UpdatePoint(object sender, RobotPointRecordingEventArgs e)
        {
            ListViewItem selectedItem = lw_positions.Items[Convert.ToInt16(e)];
            string positionName = selectedItem.SubItems[NAME_POSITION_COLUMN_INDEX].Text;

            DateTime date = DateTime.Now;
            string formattedTimeToDB = date.ToString("yyyy-MM-dd HH:mm:ss.fffffff"); // Formattazione per DB
            string formattedTimeToLW = date.ToString("HH:mm:ss:ff"); // Formattazione per LW

            int id = Convert.ToInt32(selectedItem.SubItems[ID_COLUMN_INDEX].Text);

            // Aggiorna i campi della riga selezionata con i valori di newPoint
            selectedItem.SubItems[3].Text = e.pose.tran.x.ToString();
            selectedItem.SubItems[4].Text = e.pose.tran.y.ToString();
            selectedItem.SubItems[5].Text = e.pose.tran.z.ToString();
            selectedItem.SubItems[6].Text = e.pose.rpy.rx.ToString();
            selectedItem.SubItems[7].Text = e.pose.rpy.ry.ToString();
            selectedItem.SubItems[8].Text = e.pose.rpy.rz.ToString();

            // Aggiorno il timestamp
            selectedItem.SubItems[1].Text = formattedTimeToLW;

            robotDAO.UpdatePositionCoord(ConnectionString, e.pose, formattedTimeToDB, id, positionName, "RM");
        }

        /// <summary>
        /// Metodo per riempire la ListView lw_positions con i dati della tabella positions
        /// </summary>
        private void Fill_LwPositions()
        {
            // Ottieni i dati dalla tabella
            DataTable positions = robotDAO.GetPointsPosition(ConnectionString, RMProgramName);

            // Pulisci la ListView prima di riempirla
            lw_positions.Items.Clear();

            // Itera attraverso tutte le righe del DataTable
            foreach (DataRow row in positions.Rows)
            {
                // Crea un nuovo ListViewItem
                ListViewItem item = new ListViewItem(row["id_pos"].ToString());
                item.SubItems.Add(Convert.ToDateTime(row["sampleTime"]).ToString("HH:mm:ss:ff"));
                item.SubItems.Add(row["mode"].ToString());
                item.SubItems.Add(row["x"].ToString());
                item.SubItems.Add(row["y"].ToString());
                item.SubItems.Add(row["z"].ToString());
                item.SubItems.Add(row["rx"].ToString());
                item.SubItems.Add(row["ry"].ToString());
                item.SubItems.Add(row["rz"].ToString());
                item.SubItems.Add(row["positionName"].ToString());

                // Aggiungi l'elemento alla ListView
                lw_positions.Items.Add(item);
            }
        }

        /// <summary>
        /// Inizializza le textbox contenenti i parametri del Robot
        /// </summary>
        private void InitRobotParameters()
        {

        }

        /// <summary>
        /// Inizializza alcuni elementi grafici della schermata e iscrive form agli eventi
        /// </summary>
        private void InitView()
        {
            FormHomePage.Instance.LabelHeader = "POSIZIONI ROBOT";

            SetHeight(lw_positions, 30);

            InitFont();
            /*
            // Collegamento evento ApplicationAdded del dizionario al metodo HandleDictionaryPositionDeleted
            ApplicationConfig.applicationsManager.ApplicationPositionDeleted +=
                HandleDictionaryPositionDeleted;

            // Collegamento evento ApplicationAdded del dizionario al metodo HandleDictionaryPositionDeletetStartingFromId
            ApplicationConfig.applicationsManager.ApplicationPositionDeletedStartingFromId +=
                HandleDictionaryPositionDeletetStartingFromId;

            // Collegamento evento ApplicationAdded del dizionario al metodo HandleDictionaryPositionDeletetFromId
            ApplicationConfig.applicationsManager.ApplicationPositionDeletedFromId +=
                HandleDictionaryPositionDeletetFromId;
            */
            // Collegamento evento RobotPositionNameUpdated del dizionario al metodo HandleDictionaryRobotPositionNameUpdated
            ApplicationConfig.applicationsManager.RobotPositionNameUpdated +=
                HandleDictionaryRobotPositionNameUpdated;

            // Collegamento evento RobotPositionNameRemoved del dizionario al metodo HandleDictionaryRobotPositionRemoved
            ApplicationConfig.applicationsManager.RobotPositionRemoved +=
                HandleDictionaryRobotPositionRemoved;
        }

        /// <summary>
        /// Metodo che setta Font
        /// </summary>
        private void InitFont()
        {
            lw_positions.Font = ProjectVariables.FontListView;
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
                ImageSize = new Size(1, height)
            };
            listView.SmallImageList = imgList;
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

        /// <summary>
        /// Mostra o nasconde gli elementi relativi ai filtri, inoltre li applica o li toglie a seconda della variabile isFilterEnabled
        /// </summary>
        /// <param name="forceValue">Indica se si vuole forzare la chiusura dei filtri</param>
        private void FiltersManagement(bool forceValue)
        {
            /*  if (forceValue)
                  isFilterEnabled = false; // forzo a false per toglierli
              else
                  isFilterEnabled = !isFilterEnabled; // gestisco i filtri automaticamente

              if (isFilterEnabled)
              {
                  btn_filters.BackgroundImage = Properties.Resources.remove_filter_filled1;
                  //ApplyFilters();
              }
              else
              {
                  btn_filters.BackgroundImage = Properties.Resources.filter_filled;
                  //RemoveFilters();
              }

              ud_min_point.Visible = isFilterEnabled;
              ud_max_point.Visible = isFilterEnabled;
              btn_min_point.Visible = isFilterEnabled;
              btn_max_point.Visible = isFilterEnabled;
              ud_min_time.Visible = isFilterEnabled;
              ud_max_time.Visible = isFilterEnabled;
              btn_min_time.Visible = isFilterEnabled;
              btn_max_time.Visible = isFilterEnabled;*/
        }

        /// <summary>
        /// Applica i filtri derivati dalle textbox e dai numericUpDown alla list view sottostante
        /// </summary>
        private void ApplyFilters()
        {
            /*string filterExpression = string.Empty;

            decimal Min = ud_min_point.Value;
            decimal Max = ud_max_point.Value;

            if ((Min != 0 || Max != 0) && Max >= Min)  // Se gli id min e max sono a 0 allora non li filtro
            {
                // Verifica il campo min id ha un valore
                if (!string.IsNullOrWhiteSpace(ud_min_point.Text))
                {
                    int minId = Convert.ToInt32(ud_min_point.Value);
                    filterExpression += $"ID >= {minId}";
                    //filterExpression += $"ID >= {Ud_min_id_filter.Value}";
                }

                // Aggiungi l'operatore logico "AND" se è presente anche il filtro per max id
                if (!string.IsNullOrWhiteSpace(ud_max_point.Text))
                {
                    int maxId = Convert.ToInt32(ud_max_point.Value);
                    if (!string.IsNullOrWhiteSpace(filterExpression))
                        filterExpression += " AND ";
                    filterExpression += $"ID <= {maxId}";
                }
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
            lw_positions.Items.Clear();
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

            lw_positions.Items.AddRange(itemsToAdd.ToArray());*/
        }

        /// <summary>
        /// Metodo che costruisce e riempie DataTable di supporto per filtrare ListView
        /// </summary>
        private void BuildAndFill_Dt_filtered()
        {
            dt_filtered.Columns.Add("ID", Type.GetType("System.Int32"));
            dt_filtered.Columns.Add("TimeStamp", Type.GetType("System.String"));
            dt_filtered.Columns.Add("Mode", Type.GetType("System.String"));
            dt_filtered.Columns.Add("x", Type.GetType("System.String"));
            dt_filtered.Columns.Add("y", Type.GetType("System.String"));
            dt_filtered.Columns.Add("z", Type.GetType("System.String"));
            dt_filtered.Columns.Add("rx", Type.GetType("System.String"));
            dt_filtered.Columns.Add("ry", Type.GetType("System.String"));
            dt_filtered.Columns.Add("rz", Type.GetType("System.String"));
            dt_filtered.Columns.Add("name", Type.GetType("System.String"));

            // Imposta la proprietà OwnerDraw su true
            //lw_positions.OwnerDraw = true;

            // Gestisci l'evento DrawColumnHeader
            //lw_positions.DrawColumnHeader += new DrawListViewColumnHeaderEventHandler(lw_positions_DrawColumnHeader);
            // Gestisci l'evento DrawItem
            //lw_positions.DrawItem += new DrawListViewItemEventHandler(lw_positions_DrawItem);
            // Gestisci l'evento DrawSubItem
            //lw_positions.DrawSubItem += new DrawListViewSubItemEventHandler(lw_positions_DrawSubItem);

            //Update_Dt_filtered();
        }

        /// <summary>
        /// Metodo usato per aggiornare la datatable per filtrare la listview, chiamato quando viene modificata la list view a
        /// seguito di un add, delete, import o qualsiasi modifica alle sue righe
        /// </summary>
        private void Update_Dt_filtered()
        {
            /*
            dt_filtered.Rows.Clear();// tolgo le precedenti righe della lista
            foreach (ListViewItem item in lw_positions2.Items) // aggiungo le nuove righe prendendole dalla list view originale
            {

                DataRow row = dt_filtered.NewRow();
                row["ID"] = Convert.ToInt32(item.SubItems[0].Text);
                row["Timestamp"] = item.SubItems[1].Text;
                row["Mode"] = item.SubItems[2].Text;
                row["x"] = item.SubItems[3].Text;
                row["y"] = item.SubItems[4].Text;
                row["z"] = item.SubItems[5].Text;
                row["rx"] = item.SubItems[6].Text;
                row["ry"] = item.SubItems[7].Text;
                row["rz"] = item.SubItems[8].Text;
                row["name"] = item.SubItems[9].Text;

                dt_filtered.Rows.Add(row);
            }*/
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
            if (lw_positions.SelectedItems.Count > 0)
            {
                // aggiornamento list view con tutti gli elementi
                string selectedItemID = lw_positions.SelectedItems[0].SubItems[0].Text; // vecchio elemento selezionato

                lw_positions.Items.Clear();
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

                lw_positions.Items.AddRange(itemsToAdd.ToArray());
            }
            else
            {
                lw_positions.Items.Clear();
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

                lw_positions.Items.AddRange(itemsToAdd.ToArray());
            }
        }

        /*
        /// <summary>
        /// Metodo che intercetta evento di aggiunta applicazione e refresha la lw delle applicazioni
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryPositionDeleted(object sender, RobotDictionaryChangedEventArgs e)
        {
            // Rimuovi l'elemento selezionato
            lw_positions.Items.Remove(lw_positions.Items[e.Id]);
            savedPositions.RemoveAt(e.Id);

            for (int i = e.Id; i < lw_positions.Items.Count; i++)
            {
                lw_positions.Items[i].SubItems[0].Text = (Convert.ToInt32(lw_positions.Items[i].SubItems[0].Text) - 1).ToString();
            }

            if (debugCurrentIndex >= 0)
            {
                // Ottieni l'elemento precedente
                ListViewItem selectedItem = lw_positions.Items[debugCurrentIndex];

                // Deseleziona tutti gli elementi
                lw_positions.SelectedItems.Clear();

                // Seleziona l'elemento precedente
                selectedItem.Selected = true;
            }
        }

        /// <summary>
        /// Metodo che intercetta evento di aggiunta applicazione e refresha la lw delle applicazioni
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryPositionDeletetStartingFromId(object sender, RobotDictionaryChangedEventArgs e)
        {
            int startIndex = e.Id;
            // Rimuovi l'elemento selezionato
            int lwRowsCount = lw_positions.Items.Count;
            for (int i = lw_positions.Items.Count; i >= startIndex; i--)
            {
                lw_positions.Items.RemoveAt(i - 1);
                pointIndex--;
            }

            savedPositions.RemoveRange(e.Id - 1, savedPositions.Count - (e.Id - 1));

            // Deseleziona tutti gli elementi
            lw_positions.SelectedItems.Clear();

            if (CustomMessageBox.Show(MessageBoxTypeEnum.WARNING, "Iniziare la registrazione dei nuovi punti?") == DialogResult.OK)
            {
                isDragStart = true;
                EnableScreenElements();
                //FiltersManagement(true);
            }

            /*if (MessageBox.Show("Iniziare la registrazione dei nuovi punti?", "warning",
                    MessageBoxButtons.OKCancel, MessageBoxIcon.Warning) == DialogResult.OK)
            {
                isDragStart = true;
                EnableScreenElements();
                //FiltersManagement(true);
            }

        }

        /// <summary>
        /// Sostituisce l'elemento selezionato
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HandleDictionaryPositionDeletetFromId(object sender, RobotDictionaryChangedEventArgs e)
        {
            log.Info("Rimozione elemento dalla listView delle posizioni");

            lw_positions.Items.RemoveAt(e.Id);
            pointIndex--;
            log.Info("Rimozione elemento dalla lista delle posizioni salvate");
            savedPositions.RemoveAt(e.Id);

            // Deseleziona tutti gli elementi
            lw_positions.SelectedItems.Clear();

            if (CustomMessageBox.Show(MessageBoxTypeEnum.WARNING, "Iniziare la registrazione del punto da modificare?") == DialogResult.OK)
            {
                log.Info("Inizio registrazione punto da modificare");
                isDragStart = true;
                EnableScreenElements();
                //FiltersManagement(true);
            }

            
            if (MessageBox.Show("Iniziare la registrazione del punto da modificare?", "warning",
                    MessageBoxButtons.OKCancel, MessageBoxIcon.Warning) == DialogResult.OK)
            {
                log.Info("Inizio registrazione punto da modificare");
                isDragStart = true;
                EnableScreenElements();
                //FiltersManagement(true);
            }

        }*/

        /// <summary>
        /// Metodo che aggiunge una nuova routine alla ListView lw_positions
        /// </summary>
        /// <param name="pointIndex">L'indice del punto da aggiunger</param>
        /// <param name="formattedTime">Il tempo formattato da aggiungere</param>
        /// <param name="newPosition">La nuova posizione da aggiungere</param>
        private void AddNewPositionToListView(ref int pointIndex, string formattedTime, string newPosition)
        {
            ListViewItem item = new ListViewItem(pointIndex.ToString());
            item.SubItems.Add(formattedTime);
            item.SubItems.Add("");
            item.SubItems.Add("0");
            item.SubItems.Add("0");
            item.SubItems.Add("0");
            item.SubItems.Add("0");
            item.SubItems.Add("0");
            item.SubItems.Add("0");
            item.SubItems.Add(newPosition);

            lw_positions.Items.Add(item);

            // Stampo messaggio di conferma aggiunto del punto
            CustomMessageBox.Show(MessageBoxTypeEnum.INFO, $"Punto {newPosition} aggiunto con successo!");
        }
        #endregion

        #region Eventi di UC_positions

        /// <summary>
        /// Ritorno a Home
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnHome_Click(object sender, EventArgs e)
        {
            
            // Disiscrivi gli eventi prima di chiamare Dispose
           /* ApplicationConfig.applicationsManager.ApplicationPositionDeleted -=
                HandleDictionaryPositionDeleted;
            ApplicationConfig.applicationsManager.ApplicationPositionDeletedStartingFromId -=
                HandleDictionaryPositionDeletetStartingFromId;
            ApplicationConfig.applicationsManager.ApplicationPositionDeletedFromId -=
                HandleDictionaryPositionDeletetFromId;*/
            ApplicationConfig.applicationsManager.RobotPositionNameUpdated -=
                HandleDictionaryRobotPositionNameUpdated;
            ApplicationConfig.applicationsManager.RobotPositionRemoved -=
                HandleDictionaryRobotPositionRemoved;

            ScreenSaverManager.AutoRemoveClickEvents(this);
            
            FormHomePage.Instance.LabelHeader = TranslationManager.GetTranslation("LBL_HOMEPAGE_HEADER");
            FormHomePage.Instance.PnlContainer.Controls["UC_HomePage"].BringToFront();
            FormHomePage.Instance.PnlContainer.Controls.Remove(Controls["UC_applications"]);

            Dispose();
        }

        /// <summary>
        /// Cancellazione di un punto
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnDeletePoint_Click(object sender, EventArgs e)
        {
            // se non ci sono elementi selezionati
            if (lw_positions.SelectedItems.Count < 1)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Nessuna posizione da cancellare selezionata");
                log.Error("Nessuna posizione da cancellare selezionata");
            }
            else
            {
                // Se la cancellazione è confermata
                if (CustomMessageBox.Show(MessageBoxTypeEnum.WARNING, "Cancellare la posizione selezionata?") == DialogResult.OK)
                {
                    // Get applicationName
                    string positionName = lw_positions.SelectedItems[0].SubItems[NAME_POSITION_COLUMN_INDEX].Text;
                    // Get idPosition
                    int idPosition = Convert.ToInt32(lw_positions.SelectedItems[0].SubItems[ID_COLUMN_INDEX].Text);

                    // RemoveFilters();

                    robotDAO.DeleteRobotPosition(ConnectionString, positionName, idPosition);

                    log.Info("Operazione di cancellazione completata");
                }
                else
                {
                    log.Info("Operazione di cancellazione posizione annullata");
                }
            }
            //Update_Dt_filtered();
        }

        /// <summary>
        /// Button che registra posizione attuale del Robot
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnRecPoint_Click(object sender, EventArgs e)
        {
            // se non ci sono elementi selezionati
            if (lw_positions.SelectedItems.Count < 1)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Nessuna posizione da registrare selezionata");
                log.Error("Nessuna posizione da registrare selezionata");
            }
            else
            {
                ListViewItem selectedItem = lw_positions.SelectedItems[0];
                string positionName = selectedItem.SubItems[NAME_POSITION_COLUMN_INDEX].Text;


                if (CustomMessageBox.Show(MessageBoxTypeEnum.WARNING,
                    $"Sei sicuro di voler sovrascrivere la posizione {positionName}?") == DialogResult.OK)
                {

                    DateTime date = DateTime.Now;
                    string formattedTimeToDB = date.ToString("yyyy-MM-dd HH:mm:ss.fffffff"); // Formattazione per DB
                    string formattedTimeToLW = date.ToString("HH:mm:ss:ff"); // Formattazione per LW

                    DescPose newPoint = RobotManager.RecPoint();
                    // Ottieni la riga selezionata


                    int id = Convert.ToInt32(selectedItem.SubItems[ID_COLUMN_INDEX].Text);

                    // Aggiorna i campi della riga selezionata con i valori di newPoint
                    selectedItem.SubItems[3].Text = newPoint.tran.x.ToString();
                    selectedItem.SubItems[4].Text = newPoint.tran.y.ToString();
                    selectedItem.SubItems[5].Text = newPoint.tran.z.ToString();
                    selectedItem.SubItems[6].Text = newPoint.rpy.rx.ToString();
                    selectedItem.SubItems[7].Text = newPoint.rpy.ry.ToString();
                    selectedItem.SubItems[8].Text = newPoint.rpy.rz.ToString();

                    // Aggiorno il timestamp
                    selectedItem.SubItems[1].Text = formattedTimeToLW;

                    robotDAO.UpdatePositionCoord(ConnectionString, newPoint, formattedTimeToDB, id, positionName, "RM");
                }
            }
        }

        /// <summary>
        /// Esegue rename del punto
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnRenamePoint_Click(object sender, EventArgs e)
        {
            // se non ci sono elementi selezionati
            if (lw_positions.SelectedItems.Count < 1)
            {
                CustomMessageBox.Show(MessageBoxTypeEnum.ERROR, "Nessuna posizione da rinominare selezionata");
                log.Error("Nessuna posizione da rinominare selezionata");
            }
            else
            {
                try
                {
                    // Get positionName
                    string positionName = lw_positions.SelectedItems[0].SubItems[9].Text;

                    string newPositionName = VK_Manager.OpenStringVK("", false);
                    if (newPositionName.Equals(VK_Manager.CANCEL_STRING))
                        return;

                    bool isContained = false;

                    foreach (ListViewItem item in lw_positions.Items)
                    {
                        if (item.SubItems[9].Text == newPositionName)
                        {
                            isContained = true;
                        }
                    }

                    // Se non è presente
                    if (!isContained)
                    {

                        SqliteConnectionConfiguration databaseConnection = new SqliteConnectionConfiguration();
                        string connectionString = databaseConnection.GetConnectionString();


                        if (CustomMessageBox.Show(MessageBoxTypeEnum.WARNING,
                            "Modificare nome posizione selezionata in " + newPositionName + " ? ") == DialogResult.OK)
                        {
                            robotDAO.UpdateRobotPositionName(connectionString, positionName, newPositionName, DateTime.Now);
                        }
                    }
                    else
                    {
                        CustomMessageBox.Show(MessageBoxTypeEnum.ERROR,
                              "Nome posizione già presente nella lista delle posizioni");
                        log.Error("Nome posizione già presente nella lista delle posizioni");
                    }

                }
                catch (Exception ex)
                {
                    log.Error("Errore nella modifica del nome della posizione: " + ex.ToString());
                }
            }
        }

        /// <summary>
        /// Creazione del nuovo punto su database
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnAddPoint_Click(object sender, EventArgs e)
        {
            // Trova l'ID più alto nella ListView
            int maxId = 0;
            foreach (ListViewItem item in lw_positions.Items)
            {
                int itemId = int.Parse(item.Text);
                if (itemId > maxId)
                {
                    maxId = itemId;
                }
            }

            // Incrementa l'ID per il nuovo elemento
            int newId = maxId + 1;
            string newPosition = VK_Manager.OpenStringVK("", false);
            if (newPosition.Equals(VK_Manager.CANCEL_STRING)) return;
            // Ottieni la data e l'ora attuali
            DateTime now = DateTime.Now;
            string formattedTimeToDB = now.ToString("yyyy-MM-dd HH:mm:ss.fffffff");
            string formattedTimeToLW = now.ToString("HH:mm:ss:ff");

            //pointIndex++;
            AddNewPositionToListView(ref newId, formattedTimeToLW, newPosition);
            robotDAO.AddNewRobotPosition(ConnectionString, newPosition, formattedTimeToDB, ref newId, RMProgramName);
        }

        #endregion
    }
}