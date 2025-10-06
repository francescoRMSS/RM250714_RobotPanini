namespace RM.src.RM250728.Forms.DragMode
{
    partial class UC_FullDragModePage
    {
        /// <summary> 
        /// Variabile di progettazione necessaria.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Pulire le risorse in uso.
        /// </summary>
        /// <param name="disposing">ha valore true se le risorse gestite devono essere eliminate, false in caso contrario.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Codice generato da Progettazione componenti

        /// <summary> 
        /// Metodo necessario per il supporto della finestra di progettazione. Non modificare 
        /// il contenuto del metodo con l'editor di codice.
        /// </summary>
        private void InitializeComponent()
        {
            this.lw_positions = new RMLib.View.ScrollableListView();
            this.columnHeader1 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader2 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader3 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader4 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader5 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader6 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader7 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader8 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader9 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.lblDragMode = new System.Windows.Forms.Label();
            this.lbl_choosenApplication = new System.Windows.Forms.Label();
            this.lbl_choosenModeText = new System.Windows.Forms.Label();
            this.lbl_currentTime = new System.Windows.Forms.Label();
            this.lbl_currentTimeText = new System.Windows.Forms.Label();
            this.lbl_currentPoint = new System.Windows.Forms.Label();
            this.lbl_currentPointText = new System.Windows.Forms.Label();
            this.lbl_Monitor = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.btn_saveOperation = new RMLib.View.CustomButton();
            this.btn_PLAY = new RMLib.View.CustomButton();
            this.btn_cancelOperation = new RMLib.View.CustomButton();
            this.btn_STOP = new RMLib.View.CustomButton();
            this.lbl_buttonConfiguration = new System.Windows.Forms.Label();
            this.lbl_home = new System.Windows.Forms.Label();
            this.btn_add = new RMLib.View.CustomButton();
            this.btn_homePage = new RMLib.View.CustomButton();
            this.pb_LoadingGif = new System.Windows.Forms.PictureBox();
            this.btn_debugSettings = new RMLib.View.CustomButton();
            this.btn_debugTools = new RMLib.View.CustomButton();
            ((System.ComponentModel.ISupportInitialize)(this.pb_LoadingGif)).BeginInit();
            this.SuspendLayout();
            // 
            // lw_positions
            // 
            this.lw_positions.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.lw_positions.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] {
            this.columnHeader1,
            this.columnHeader2,
            this.columnHeader3,
            this.columnHeader4,
            this.columnHeader5,
            this.columnHeader6,
            this.columnHeader7,
            this.columnHeader8,
            this.columnHeader9});
            this.lw_positions.Cursor = System.Windows.Forms.Cursors.Hand;
            this.lw_positions.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F);
            this.lw_positions.FullRowSelect = true;
            this.lw_positions.HideSelection = false;
            this.lw_positions.LabelWrap = false;
            this.lw_positions.Location = new System.Drawing.Point(65, 57);
            this.lw_positions.MultiSelect = false;
            this.lw_positions.Name = "lw_positions";
            this.lw_positions.Size = new System.Drawing.Size(890, 386);
            this.lw_positions.TabIndex = 67;
            this.lw_positions.UseCompatibleStateImageBehavior = false;
            this.lw_positions.View = System.Windows.Forms.View.Details;
            this.lw_positions.DrawColumnHeader += new System.Windows.Forms.DrawListViewColumnHeaderEventHandler(this.lw_positions_DrawColumnHeader);
            this.lw_positions.DrawItem += new System.Windows.Forms.DrawListViewItemEventHandler(this.lw_positions_DrawItem);
            this.lw_positions.DrawSubItem += new System.Windows.Forms.DrawListViewSubItemEventHandler(this.lw_positions_DrawSubItem);
            // 
            // columnHeader1
            // 
            this.columnHeader1.Text = "ID";
            this.columnHeader1.Width = 40;
            // 
            // columnHeader2
            // 
            this.columnHeader2.Text = "Timestamp";
            this.columnHeader2.Width = 150;
            // 
            // columnHeader3
            // 
            this.columnHeader3.Text = "Mode";
            this.columnHeader3.Width = 90;
            // 
            // columnHeader4
            // 
            this.columnHeader4.Text = "x";
            this.columnHeader4.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader4.Width = 100;
            // 
            // columnHeader5
            // 
            this.columnHeader5.Text = "y";
            this.columnHeader5.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader5.Width = 100;
            // 
            // columnHeader6
            // 
            this.columnHeader6.Text = "z";
            this.columnHeader6.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader6.Width = 100;
            // 
            // columnHeader7
            // 
            this.columnHeader7.Text = "rx";
            this.columnHeader7.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader7.Width = 100;
            // 
            // columnHeader8
            // 
            this.columnHeader8.Text = "ry";
            this.columnHeader8.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader8.Width = 100;
            // 
            // columnHeader9
            // 
            this.columnHeader9.Text = "rz";
            this.columnHeader9.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader9.Width = 100;
            // 
            // lblDragMode
            // 
            this.lblDragMode.BackColor = System.Drawing.Color.Transparent;
            this.lblDragMode.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lblDragMode.ForeColor = System.Drawing.SystemColors.Control;
            this.lblDragMode.Location = new System.Drawing.Point(736, 454);
            this.lblDragMode.Name = "lblDragMode";
            this.lblDragMode.Size = new System.Drawing.Size(177, 34);
            this.lblDragMode.TabIndex = 124;
            this.lblDragMode.Text = "Point to point";
            this.lblDragMode.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_choosenApplication
            // 
            this.lbl_choosenApplication.BackColor = System.Drawing.Color.Transparent;
            this.lbl_choosenApplication.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_choosenApplication.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_choosenApplication.Location = new System.Drawing.Point(65, 4);
            this.lbl_choosenApplication.Name = "lbl_choosenApplication";
            this.lbl_choosenApplication.Size = new System.Drawing.Size(890, 34);
            this.lbl_choosenApplication.TabIndex = 120;
            this.lbl_choosenApplication.Text = "-";
            this.lbl_choosenApplication.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_choosenModeText
            // 
            this.lbl_choosenModeText.BackColor = System.Drawing.Color.Transparent;
            this.lbl_choosenModeText.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_choosenModeText.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_choosenModeText.Location = new System.Drawing.Point(572, 454);
            this.lbl_choosenModeText.Name = "lbl_choosenModeText";
            this.lbl_choosenModeText.Size = new System.Drawing.Size(158, 34);
            this.lbl_choosenModeText.TabIndex = 117;
            this.lbl_choosenModeText.Text = "Modalità scelta:";
            this.lbl_choosenModeText.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_currentTime
            // 
            this.lbl_currentTime.BackColor = System.Drawing.Color.Transparent;
            this.lbl_currentTime.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_currentTime.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_currentTime.Location = new System.Drawing.Point(275, 482);
            this.lbl_currentTime.Name = "lbl_currentTime";
            this.lbl_currentTime.Size = new System.Drawing.Size(141, 50);
            this.lbl_currentTime.TabIndex = 109;
            this.lbl_currentTime.Text = "-";
            this.lbl_currentTime.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_currentTimeText
            // 
            this.lbl_currentTimeText.BackColor = System.Drawing.Color.Transparent;
            this.lbl_currentTimeText.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_currentTimeText.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_currentTimeText.Location = new System.Drawing.Point(118, 482);
            this.lbl_currentTimeText.Name = "lbl_currentTimeText";
            this.lbl_currentTimeText.Size = new System.Drawing.Size(182, 50);
            this.lbl_currentTimeText.TabIndex = 108;
            this.lbl_currentTimeText.Text = "Tempo corrente:";
            this.lbl_currentTimeText.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_currentPoint
            // 
            this.lbl_currentPoint.BackColor = System.Drawing.Color.Transparent;
            this.lbl_currentPoint.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_currentPoint.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_currentPoint.Location = new System.Drawing.Point(275, 446);
            this.lbl_currentPoint.Name = "lbl_currentPoint";
            this.lbl_currentPoint.Size = new System.Drawing.Size(141, 50);
            this.lbl_currentPoint.TabIndex = 107;
            this.lbl_currentPoint.Text = "0";
            this.lbl_currentPoint.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_currentPointText
            // 
            this.lbl_currentPointText.BackColor = System.Drawing.Color.Transparent;
            this.lbl_currentPointText.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_currentPointText.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_currentPointText.Location = new System.Drawing.Point(118, 446);
            this.lbl_currentPointText.Name = "lbl_currentPointText";
            this.lbl_currentPointText.Size = new System.Drawing.Size(163, 50);
            this.lbl_currentPointText.TabIndex = 106;
            this.lbl_currentPointText.Text = "Punto corrente: ";
            this.lbl_currentPointText.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_Monitor
            // 
            this.lbl_Monitor.BackColor = System.Drawing.Color.Transparent;
            this.lbl_Monitor.Font = new System.Drawing.Font("Verdana", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_Monitor.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_Monitor.Location = new System.Drawing.Point(431, 565);
            this.lbl_Monitor.Name = "lbl_Monitor";
            this.lbl_Monitor.Size = new System.Drawing.Size(144, 40);
            this.lbl_Monitor.TabIndex = 114;
            this.lbl_Monitor.Text = "Drag mode: OFF";
            this.lbl_Monitor.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label9
            // 
            this.label9.BackColor = System.Drawing.Color.Transparent;
            this.label9.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label9.ForeColor = System.Drawing.SystemColors.Control;
            this.label9.Location = new System.Drawing.Point(934, 633);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(60, 20);
            this.label9.TabIndex = 1;
            this.label9.Text = "Salva";
            this.label9.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // label10
            // 
            this.label10.BackColor = System.Drawing.Color.Transparent;
            this.label10.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label10.ForeColor = System.Drawing.SystemColors.Control;
            this.label10.Location = new System.Drawing.Point(831, 633);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(68, 20);
            this.label10.TabIndex = 1;
            this.label10.Text = "Annulla";
            this.label10.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // btn_saveOperation
            // 
            this.btn_saveOperation.BackColor = System.Drawing.SystemColors.Control;
            this.btn_saveOperation.BackgroundColor = System.Drawing.SystemColors.Control;
            this.btn_saveOperation.BackgroundImage = global::RM.Properties.Resources.save32;
            this.btn_saveOperation.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_saveOperation.BorderColor = System.Drawing.SystemColors.ButtonShadow;
            this.btn_saveOperation.BorderRadius = 20;
            this.btn_saveOperation.BorderSize = 1;
            this.btn_saveOperation.FlatAppearance.BorderSize = 0;
            this.btn_saveOperation.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_saveOperation.Font = new System.Drawing.Font("Segoe UI Semibold", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_saveOperation.ForeColor = System.Drawing.Color.Black;
            this.btn_saveOperation.Location = new System.Drawing.Point(934, 570);
            this.btn_saveOperation.Name = "btn_saveOperation";
            this.btn_saveOperation.Size = new System.Drawing.Size(60, 60);
            this.btn_saveOperation.TabIndex = 38;
            this.btn_saveOperation.TextColor = System.Drawing.Color.Black;
            this.btn_saveOperation.UseVisualStyleBackColor = false;
            this.btn_saveOperation.Click += new System.EventHandler(this.ClickEvent_saveOperation);
            // 
            // btn_PLAY
            // 
            this.btn_PLAY.BackColor = System.Drawing.SystemColors.Control;
            this.btn_PLAY.BackgroundColor = System.Drawing.SystemColors.Control;
            this.btn_PLAY.BackgroundImage = global::RM.Properties.Resources.play32;
            this.btn_PLAY.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_PLAY.BorderColor = System.Drawing.SystemColors.ButtonShadow;
            this.btn_PLAY.BorderRadius = 20;
            this.btn_PLAY.BorderSize = 1;
            this.btn_PLAY.FlatAppearance.BorderSize = 0;
            this.btn_PLAY.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_PLAY.Font = new System.Drawing.Font("Segoe UI Semibold", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_PLAY.ForeColor = System.Drawing.Color.Black;
            this.btn_PLAY.Location = new System.Drawing.Point(581, 570);
            this.btn_PLAY.Name = "btn_PLAY";
            this.btn_PLAY.Size = new System.Drawing.Size(60, 60);
            this.btn_PLAY.TabIndex = 111;
            this.btn_PLAY.TextColor = System.Drawing.Color.Black;
            this.btn_PLAY.UseVisualStyleBackColor = false;
            this.btn_PLAY.Click += new System.EventHandler(this.ClickEvent_StartDrag);
            // 
            // btn_cancelOperation
            // 
            this.btn_cancelOperation.BackColor = System.Drawing.SystemColors.Control;
            this.btn_cancelOperation.BackgroundColor = System.Drawing.SystemColors.Control;
            this.btn_cancelOperation.BackgroundImage = global::RM.Properties.Resources.cancelOperation_32;
            this.btn_cancelOperation.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_cancelOperation.BorderColor = System.Drawing.SystemColors.ButtonShadow;
            this.btn_cancelOperation.BorderRadius = 20;
            this.btn_cancelOperation.BorderSize = 1;
            this.btn_cancelOperation.FlatAppearance.BorderSize = 0;
            this.btn_cancelOperation.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_cancelOperation.Font = new System.Drawing.Font("Segoe UI Semibold", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_cancelOperation.ForeColor = System.Drawing.Color.Black;
            this.btn_cancelOperation.Location = new System.Drawing.Point(834, 570);
            this.btn_cancelOperation.Name = "btn_cancelOperation";
            this.btn_cancelOperation.Size = new System.Drawing.Size(60, 60);
            this.btn_cancelOperation.TabIndex = 38;
            this.btn_cancelOperation.TextColor = System.Drawing.Color.Black;
            this.btn_cancelOperation.UseVisualStyleBackColor = false;
            this.btn_cancelOperation.Click += new System.EventHandler(this.ClickEvent_cancelOperation);
            // 
            // btn_STOP
            // 
            this.btn_STOP.BackColor = System.Drawing.SystemColors.Control;
            this.btn_STOP.BackgroundColor = System.Drawing.SystemColors.Control;
            this.btn_STOP.BackgroundImage = global::RM.Properties.Resources.stop;
            this.btn_STOP.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_STOP.BorderColor = System.Drawing.SystemColors.ButtonShadow;
            this.btn_STOP.BorderRadius = 20;
            this.btn_STOP.BorderSize = 1;
            this.btn_STOP.FlatAppearance.BorderSize = 0;
            this.btn_STOP.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_STOP.Font = new System.Drawing.Font("Segoe UI Semibold", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_STOP.ForeColor = System.Drawing.Color.Black;
            this.btn_STOP.Location = new System.Drawing.Point(356, 570);
            this.btn_STOP.Name = "btn_STOP";
            this.btn_STOP.Size = new System.Drawing.Size(60, 60);
            this.btn_STOP.TabIndex = 110;
            this.btn_STOP.TextColor = System.Drawing.Color.Black;
            this.btn_STOP.UseVisualStyleBackColor = false;
            this.btn_STOP.Click += new System.EventHandler(this.ClickEvent_StopDrag);
            // 
            // lbl_buttonConfiguration
            // 
            this.lbl_buttonConfiguration.BackColor = System.Drawing.Color.Transparent;
            this.lbl_buttonConfiguration.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_buttonConfiguration.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_buttonConfiguration.Location = new System.Drawing.Point(106, 633);
            this.lbl_buttonConfiguration.Name = "lbl_buttonConfiguration";
            this.lbl_buttonConfiguration.Size = new System.Drawing.Size(92, 20);
            this.lbl_buttonConfiguration.TabIndex = 1;
            this.lbl_buttonConfiguration.Text = "Applicazione";
            this.lbl_buttonConfiguration.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            this.lbl_buttonConfiguration.Visible = false;
            // 
            // lbl_home
            // 
            this.lbl_home.BackColor = System.Drawing.Color.Transparent;
            this.lbl_home.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_home.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_home.Location = new System.Drawing.Point(22, 633);
            this.lbl_home.Name = "lbl_home";
            this.lbl_home.Size = new System.Drawing.Size(60, 20);
            this.lbl_home.TabIndex = 1;
            this.lbl_home.Text = "Home";
            this.lbl_home.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // btn_add
            // 
            this.btn_add.BackColor = System.Drawing.SystemColors.Control;
            this.btn_add.BackgroundColor = System.Drawing.SystemColors.Control;
            this.btn_add.BackgroundImage = global::RM.Properties.Resources.addApp32;
            this.btn_add.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_add.BorderColor = System.Drawing.SystemColors.ButtonShadow;
            this.btn_add.BorderRadius = 20;
            this.btn_add.BorderSize = 1;
            this.btn_add.FlatAppearance.BorderSize = 0;
            this.btn_add.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_add.Font = new System.Drawing.Font("Segoe UI Semibold", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_add.ForeColor = System.Drawing.Color.Black;
            this.btn_add.Location = new System.Drawing.Point(120, 570);
            this.btn_add.Name = "btn_add";
            this.btn_add.Size = new System.Drawing.Size(60, 60);
            this.btn_add.TabIndex = 38;
            this.btn_add.TextColor = System.Drawing.Color.Black;
            this.btn_add.UseVisualStyleBackColor = false;
            this.btn_add.Visible = false;
            this.btn_add.Click += new System.EventHandler(this.ClickEvent_addApplication);
            // 
            // btn_homePage
            // 
            this.btn_homePage.BackColor = System.Drawing.SystemColors.Control;
            this.btn_homePage.BackgroundColor = System.Drawing.SystemColors.Control;
            this.btn_homePage.BackgroundImage = global::RM.Properties.Resources.back32;
            this.btn_homePage.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_homePage.BorderColor = System.Drawing.SystemColors.ButtonShadow;
            this.btn_homePage.BorderRadius = 20;
            this.btn_homePage.BorderSize = 1;
            this.btn_homePage.FlatAppearance.BorderSize = 0;
            this.btn_homePage.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_homePage.Font = new System.Drawing.Font("Segoe UI Semibold", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_homePage.ForeColor = System.Drawing.Color.Black;
            this.btn_homePage.Location = new System.Drawing.Point(20, 570);
            this.btn_homePage.Name = "btn_homePage";
            this.btn_homePage.Size = new System.Drawing.Size(60, 60);
            this.btn_homePage.TabIndex = 38;
            this.btn_homePage.TextColor = System.Drawing.Color.Black;
            this.btn_homePage.UseVisualStyleBackColor = false;
            this.btn_homePage.Click += new System.EventHandler(this.ClickEvent_HomePage);
            // 
            // pb_LoadingGif
            // 
            this.pb_LoadingGif.BackColor = System.Drawing.Color.Transparent;
            this.pb_LoadingGif.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.pb_LoadingGif.Image = global::RM.Properties.Resources.loading_gif;
            this.pb_LoadingGif.Location = new System.Drawing.Point(479, 608);
            this.pb_LoadingGif.Name = "pb_LoadingGif";
            this.pb_LoadingGif.Size = new System.Drawing.Size(45, 40);
            this.pb_LoadingGif.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.pb_LoadingGif.TabIndex = 115;
            this.pb_LoadingGif.TabStop = false;
            this.pb_LoadingGif.Visible = false;
            // 
            // btn_debugSettings
            // 
            this.btn_debugSettings.BackColor = System.Drawing.Color.Transparent;
            this.btn_debugSettings.BackgroundColor = System.Drawing.Color.Transparent;
            this.btn_debugSettings.BackgroundImage = global::RM.Properties.Resources.settings32;
            this.btn_debugSettings.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_debugSettings.BorderColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.btn_debugSettings.BorderRadius = 0;
            this.btn_debugSettings.BorderSize = 2;
            this.btn_debugSettings.FlatAppearance.BorderSize = 0;
            this.btn_debugSettings.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_debugSettings.ForeColor = System.Drawing.Color.White;
            this.btn_debugSettings.Location = new System.Drawing.Point(965, 272);
            this.btn_debugSettings.Name = "btn_debugSettings";
            this.btn_debugSettings.Size = new System.Drawing.Size(56, 80);
            this.btn_debugSettings.TabIndex = 2;
            this.btn_debugSettings.TextColor = System.Drawing.Color.White;
            this.btn_debugSettings.UseVisualStyleBackColor = false;
            this.btn_debugSettings.Click += new System.EventHandler(this.ClickEvent_debugSettings);
            // 
            // btn_debugTools
            // 
            this.btn_debugTools.BackColor = System.Drawing.Color.Transparent;
            this.btn_debugTools.BackgroundColor = System.Drawing.Color.Transparent;
            this.btn_debugTools.BackgroundImage = global::RM.Properties.Resources.tools32;
            this.btn_debugTools.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_debugTools.BorderColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.btn_debugTools.BorderRadius = 0;
            this.btn_debugTools.BorderSize = 2;
            this.btn_debugTools.FlatAppearance.BorderSize = 0;
            this.btn_debugTools.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_debugTools.ForeColor = System.Drawing.Color.White;
            this.btn_debugTools.Location = new System.Drawing.Point(965, 194);
            this.btn_debugTools.Name = "btn_debugTools";
            this.btn_debugTools.Size = new System.Drawing.Size(56, 80);
            this.btn_debugTools.TabIndex = 1;
            this.btn_debugTools.TextColor = System.Drawing.Color.White;
            this.btn_debugTools.UseVisualStyleBackColor = false;
            this.btn_debugTools.Click += new System.EventHandler(this.ClickEvent_debugTools);
            // 
            // UC_FullDragModePage
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Black;
            this.BackgroundImage = global::RM.Properties.Resources._20250324_UC_dragMode;
            this.Controls.Add(this.lw_positions);
            this.Controls.Add(this.lblDragMode);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.lbl_choosenApplication);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.btn_debugSettings);
            this.Controls.Add(this.lbl_choosenModeText);
            this.Controls.Add(this.btn_saveOperation);
            this.Controls.Add(this.btn_PLAY);
            this.Controls.Add(this.lbl_currentTime);
            this.Controls.Add(this.btn_cancelOperation);
            this.Controls.Add(this.lbl_currentPointText);
            this.Controls.Add(this.lbl_currentPoint);
            this.Controls.Add(this.btn_debugTools);
            this.Controls.Add(this.lbl_currentTimeText);
            this.Controls.Add(this.btn_STOP);
            this.Controls.Add(this.lbl_Monitor);
            this.Controls.Add(this.lbl_buttonConfiguration);
            this.Controls.Add(this.pb_LoadingGif);
            this.Controls.Add(this.lbl_home);
            this.Controls.Add(this.btn_homePage);
            this.Controls.Add(this.btn_add);
            this.Name = "UC_FullDragModePage";
            this.Size = new System.Drawing.Size(1024, 658);
            ((System.ComponentModel.ISupportInitialize)(this.pb_LoadingGif)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion
        private RMLib.View.CustomButton btn_debugTools;
        private RMLib.View.CustomButton btn_debugSettings;
        private RMLib.View.CustomButton btn_saveOperation;
        private RMLib.View.CustomButton btn_cancelOperation;
        private System.Windows.Forms.Label lbl_choosenApplication;
        private System.Windows.Forms.Label lbl_choosenModeText;
        private System.Windows.Forms.PictureBox pb_LoadingGif;
        private System.Windows.Forms.Label lbl_Monitor;
        private RMLib.View.CustomButton btn_PLAY;
        private System.Windows.Forms.Label lbl_currentTime;
        private RMLib.View.CustomButton btn_STOP;
        private System.Windows.Forms.Label lbl_currentTimeText;
        private System.Windows.Forms.Label lbl_currentPoint;
        private System.Windows.Forms.Label lbl_currentPointText;
        private RMLib.View.ScrollableListView lw_positions;
        private System.Windows.Forms.ColumnHeader columnHeader1;
        private System.Windows.Forms.ColumnHeader columnHeader2;
        private System.Windows.Forms.ColumnHeader columnHeader3;
        private System.Windows.Forms.ColumnHeader columnHeader4;
        private System.Windows.Forms.ColumnHeader columnHeader5;
        private System.Windows.Forms.ColumnHeader columnHeader6;
        private System.Windows.Forms.ColumnHeader columnHeader7;
        private System.Windows.Forms.ColumnHeader columnHeader8;
        private System.Windows.Forms.ColumnHeader columnHeader9;
        private RMLib.View.CustomButton btn_homePage;
        private RMLib.View.CustomButton btn_add;
        private System.Windows.Forms.Label lblDragMode;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label lbl_home;
        private System.Windows.Forms.Label lbl_buttonConfiguration;
    }
}
