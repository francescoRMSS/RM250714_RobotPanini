namespace RM.src.RM250728
{
    partial class UC_permissions
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
            this.pnl_stopCycleAlarm = new System.Windows.Forms.Panel();
            this.pnl_infoStopCycleAlarm = new System.Windows.Forms.Panel();
            this.pnl_ledStopCycleAlarm = new System.Windows.Forms.Panel();
            this.lbl_stopCycleAlarm = new System.Windows.Forms.Label();
            this.pnl_headerContainer = new RMLib.View.CustomPanel();
            this.pb_headerIcon = new System.Windows.Forms.PictureBox();
            this.lbl_header = new System.Windows.Forms.Label();
            this.pnl_container = new RMLib.View.CustomPanel();
            this.pnl_robotInPosition = new System.Windows.Forms.Panel();
            this.pnl_ledRobotInPosition = new System.Windows.Forms.Panel();
            this.pnl_infoRobotInPosition = new System.Windows.Forms.Panel();
            this.lbl_robotInPosition = new System.Windows.Forms.Label();
            this.pb_separator5 = new System.Windows.Forms.PictureBox();
            this.pnl_barrier = new System.Windows.Forms.Panel();
            this.pnl_ledBarrier = new System.Windows.Forms.Panel();
            this.pnl_infoBarrier = new System.Windows.Forms.Panel();
            this.lbl_barrier = new System.Windows.Forms.Label();
            this.pb_separator4 = new System.Windows.Forms.PictureBox();
            this.pnl_robotGripperOpen = new System.Windows.Forms.Panel();
            this.pnl_ledRobotGripperOpen = new System.Windows.Forms.Panel();
            this.pnl_infoRobotGripperOpen = new System.Windows.Forms.Panel();
            this.lbl_robotGripperOpen = new System.Windows.Forms.Label();
            this.pb_separator3 = new System.Windows.Forms.PictureBox();
            this.pnl_lbl_tapeInit = new System.Windows.Forms.Panel();
            this.pnl_ledTapeInit = new System.Windows.Forms.Panel();
            this.pnl_infoTapeInit = new System.Windows.Forms.Panel();
            this.lbl_tapeInit = new System.Windows.Forms.Label();
            this.pb_separator2 = new System.Windows.Forms.PictureBox();
            this.pnl_robotReady = new System.Windows.Forms.Panel();
            this.pnl_ledRobotReady = new System.Windows.Forms.Panel();
            this.pnl_infoRobotReady = new System.Windows.Forms.Panel();
            this.lbl_robotReady = new System.Windows.Forms.Label();
            this.pb_separator1 = new System.Windows.Forms.PictureBox();
            this.pnl_navigation = new System.Windows.Forms.Panel();
            this.pnl_buttonHome = new System.Windows.Forms.Panel();
            this.btn_home = new RMLib.View.CustomButton();
            this.lbl_buttonHome = new System.Windows.Forms.Label();
            this.pnl_stopCycleAlarm.SuspendLayout();
            this.pnl_headerContainer.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_headerIcon)).BeginInit();
            this.pnl_container.SuspendLayout();
            this.pnl_robotInPosition.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator5)).BeginInit();
            this.pnl_barrier.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator4)).BeginInit();
            this.pnl_robotGripperOpen.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator3)).BeginInit();
            this.pnl_lbl_tapeInit.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator2)).BeginInit();
            this.pnl_robotReady.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator1)).BeginInit();
            this.pnl_navigation.SuspendLayout();
            this.pnl_buttonHome.SuspendLayout();
            this.SuspendLayout();
            // 
            // pnl_stopCycleAlarm
            // 
            this.pnl_stopCycleAlarm.Controls.Add(this.pnl_infoStopCycleAlarm);
            this.pnl_stopCycleAlarm.Controls.Add(this.pnl_ledStopCycleAlarm);
            this.pnl_stopCycleAlarm.Controls.Add(this.lbl_stopCycleAlarm);
            this.pnl_stopCycleAlarm.Location = new System.Drawing.Point(5, 49);
            this.pnl_stopCycleAlarm.Name = "pnl_stopCycleAlarm";
            this.pnl_stopCycleAlarm.Size = new System.Drawing.Size(578, 56);
            this.pnl_stopCycleAlarm.TabIndex = 1;
            // 
            // pnl_infoStopCycleAlarm
            // 
            this.pnl_infoStopCycleAlarm.BackgroundImage = global::RM.Properties.Resources.info;
            this.pnl_infoStopCycleAlarm.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_infoStopCycleAlarm.Location = new System.Drawing.Point(515, 8);
            this.pnl_infoStopCycleAlarm.Name = "pnl_infoStopCycleAlarm";
            this.pnl_infoStopCycleAlarm.Size = new System.Drawing.Size(30, 30);
            this.pnl_infoStopCycleAlarm.TabIndex = 4;
            this.pnl_infoStopCycleAlarm.Click += new System.EventHandler(this.PnlInfoStopCycleAlarm_Click);
            // 
            // pnl_ledStopCycleAlarm
            // 
            this.pnl_ledStopCycleAlarm.BackgroundImage = global::RM.Properties.Resources.emptyCircle;
            this.pnl_ledStopCycleAlarm.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_ledStopCycleAlarm.Location = new System.Drawing.Point(65, 3);
            this.pnl_ledStopCycleAlarm.Name = "pnl_ledStopCycleAlarm";
            this.pnl_ledStopCycleAlarm.Size = new System.Drawing.Size(40, 40);
            this.pnl_ledStopCycleAlarm.TabIndex = 3;
            // 
            // lbl_stopCycleAlarm
            // 
            this.lbl_stopCycleAlarm.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_stopCycleAlarm.ForeColor = System.Drawing.Color.Black;
            this.lbl_stopCycleAlarm.Location = new System.Drawing.Point(159, 0);
            this.lbl_stopCycleAlarm.Name = "lbl_stopCycleAlarm";
            this.lbl_stopCycleAlarm.Size = new System.Drawing.Size(313, 50);
            this.lbl_stopCycleAlarm.TabIndex = 0;
            this.lbl_stopCycleAlarm.Text = "Allarme stop ciclo";
            this.lbl_stopCycleAlarm.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pnl_headerContainer
            // 
            this.pnl_headerContainer.BackColor = System.Drawing.Color.DarkBlue;
            this.pnl_headerContainer.BackgroundColor = System.Drawing.Color.DarkBlue;
            this.pnl_headerContainer.BorderColor = System.Drawing.SystemColors.ButtonShadow;
            this.pnl_headerContainer.BorderRadius = 5;
            this.pnl_headerContainer.BorderSize = 1;
            this.pnl_headerContainer.Controls.Add(this.pb_headerIcon);
            this.pnl_headerContainer.Controls.Add(this.lbl_header);
            this.pnl_headerContainer.ForeColor = System.Drawing.Color.White;
            this.pnl_headerContainer.Location = new System.Drawing.Point(0, 0);
            this.pnl_headerContainer.Name = "pnl_headerContainer";
            this.pnl_headerContainer.Size = new System.Drawing.Size(604, 42);
            this.pnl_headerContainer.TabIndex = 248;
            this.pnl_headerContainer.TextColor = System.Drawing.Color.White;
            // 
            // pb_headerIcon
            // 
            this.pb_headerIcon.BackgroundImage = global::RM.Properties.Resources.check_white;
            this.pb_headerIcon.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pb_headerIcon.Location = new System.Drawing.Point(15, 5);
            this.pb_headerIcon.Name = "pb_headerIcon";
            this.pb_headerIcon.Size = new System.Drawing.Size(32, 32);
            this.pb_headerIcon.TabIndex = 254;
            this.pb_headerIcon.TabStop = false;
            // 
            // lbl_header
            // 
            this.lbl_header.BackColor = System.Drawing.Color.Transparent;
            this.lbl_header.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_header.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_header.Location = new System.Drawing.Point(15, 0);
            this.lbl_header.Name = "lbl_header";
            this.lbl_header.Size = new System.Drawing.Size(572, 42);
            this.lbl_header.TabIndex = 3;
            this.lbl_header.Text = "Consensi Robot";
            this.lbl_header.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pnl_container
            // 
            this.pnl_container.BackColor = System.Drawing.Color.WhiteSmoke;
            this.pnl_container.BackgroundColor = System.Drawing.Color.WhiteSmoke;
            this.pnl_container.BorderColor = System.Drawing.Color.DarkBlue;
            this.pnl_container.BorderRadius = 10;
            this.pnl_container.BorderSize = 2;
            this.pnl_container.Controls.Add(this.pnl_robotInPosition);
            this.pnl_container.Controls.Add(this.pb_separator5);
            this.pnl_container.Controls.Add(this.pnl_barrier);
            this.pnl_container.Controls.Add(this.pb_separator4);
            this.pnl_container.Controls.Add(this.pnl_robotGripperOpen);
            this.pnl_container.Controls.Add(this.pb_separator3);
            this.pnl_container.Controls.Add(this.pnl_lbl_tapeInit);
            this.pnl_container.Controls.Add(this.pb_separator2);
            this.pnl_container.Controls.Add(this.pnl_robotReady);
            this.pnl_container.Controls.Add(this.pb_separator1);
            this.pnl_container.Controls.Add(this.pnl_headerContainer);
            this.pnl_container.Controls.Add(this.pnl_stopCycleAlarm);
            this.pnl_container.ForeColor = System.Drawing.Color.White;
            this.pnl_container.Location = new System.Drawing.Point(210, 40);
            this.pnl_container.Name = "pnl_container";
            this.pnl_container.Size = new System.Drawing.Size(604, 420);
            this.pnl_container.TabIndex = 245;
            this.pnl_container.TextColor = System.Drawing.Color.White;
            // 
            // pnl_robotInPosition
            // 
            this.pnl_robotInPosition.Controls.Add(this.pnl_ledRobotInPosition);
            this.pnl_robotInPosition.Controls.Add(this.pnl_infoRobotInPosition);
            this.pnl_robotInPosition.Controls.Add(this.lbl_robotInPosition);
            this.pnl_robotInPosition.Location = new System.Drawing.Point(5, 359);
            this.pnl_robotInPosition.Name = "pnl_robotInPosition";
            this.pnl_robotInPosition.Size = new System.Drawing.Size(578, 56);
            this.pnl_robotInPosition.TabIndex = 258;
            // 
            // pnl_ledRobotInPosition
            // 
            this.pnl_ledRobotInPosition.BackgroundImage = global::RM.Properties.Resources.emptyCircle;
            this.pnl_ledRobotInPosition.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_ledRobotInPosition.Location = new System.Drawing.Point(65, 3);
            this.pnl_ledRobotInPosition.Name = "pnl_ledRobotInPosition";
            this.pnl_ledRobotInPosition.Size = new System.Drawing.Size(40, 40);
            this.pnl_ledRobotInPosition.TabIndex = 261;
            // 
            // pnl_infoRobotInPosition
            // 
            this.pnl_infoRobotInPosition.BackgroundImage = global::RM.Properties.Resources.info;
            this.pnl_infoRobotInPosition.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_infoRobotInPosition.Location = new System.Drawing.Point(515, 8);
            this.pnl_infoRobotInPosition.Name = "pnl_infoRobotInPosition";
            this.pnl_infoRobotInPosition.Size = new System.Drawing.Size(30, 30);
            this.pnl_infoRobotInPosition.TabIndex = 4;
            this.pnl_infoRobotInPosition.Click += new System.EventHandler(this.PnlInfoRobotInPosition_Click);
            // 
            // lbl_robotInPosition
            // 
            this.lbl_robotInPosition.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_robotInPosition.ForeColor = System.Drawing.Color.Black;
            this.lbl_robotInPosition.Location = new System.Drawing.Point(159, 0);
            this.lbl_robotInPosition.Name = "lbl_robotInPosition";
            this.lbl_robotInPosition.Size = new System.Drawing.Size(313, 50);
            this.lbl_robotInPosition.TabIndex = 0;
            this.lbl_robotInPosition.Text = "Robot in posizione";
            this.lbl_robotInPosition.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pb_separator5
            // 
            this.pb_separator5.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_separator5.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_separator5.Location = new System.Drawing.Point(5, 343);
            this.pb_separator5.Name = "pb_separator5";
            this.pb_separator5.Size = new System.Drawing.Size(596, 18);
            this.pb_separator5.TabIndex = 257;
            this.pb_separator5.TabStop = false;
            // 
            // pnl_barrier
            // 
            this.pnl_barrier.Controls.Add(this.pnl_ledBarrier);
            this.pnl_barrier.Controls.Add(this.pnl_infoBarrier);
            this.pnl_barrier.Controls.Add(this.lbl_barrier);
            this.pnl_barrier.Location = new System.Drawing.Point(5, 297);
            this.pnl_barrier.Name = "pnl_barrier";
            this.pnl_barrier.Size = new System.Drawing.Size(578, 56);
            this.pnl_barrier.TabIndex = 256;
            // 
            // pnl_ledBarrier
            // 
            this.pnl_ledBarrier.BackgroundImage = global::RM.Properties.Resources.emptyCircle;
            this.pnl_ledBarrier.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_ledBarrier.Location = new System.Drawing.Point(65, 3);
            this.pnl_ledBarrier.Name = "pnl_ledBarrier";
            this.pnl_ledBarrier.Size = new System.Drawing.Size(40, 40);
            this.pnl_ledBarrier.TabIndex = 261;
            // 
            // pnl_infoBarrier
            // 
            this.pnl_infoBarrier.BackgroundImage = global::RM.Properties.Resources.info;
            this.pnl_infoBarrier.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_infoBarrier.Location = new System.Drawing.Point(515, 8);
            this.pnl_infoBarrier.Name = "pnl_infoBarrier";
            this.pnl_infoBarrier.Size = new System.Drawing.Size(30, 30);
            this.pnl_infoBarrier.TabIndex = 4;
            this.pnl_infoBarrier.Click += new System.EventHandler(this.PnlInfoBarrier_Click);
            // 
            // lbl_barrier
            // 
            this.lbl_barrier.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_barrier.ForeColor = System.Drawing.Color.Black;
            this.lbl_barrier.Location = new System.Drawing.Point(159, 0);
            this.lbl_barrier.Name = "lbl_barrier";
            this.lbl_barrier.Size = new System.Drawing.Size(313, 50);
            this.lbl_barrier.TabIndex = 0;
            this.lbl_barrier.Text = "Barriera";
            this.lbl_barrier.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pb_separator4
            // 
            this.pb_separator4.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_separator4.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_separator4.Location = new System.Drawing.Point(5, 281);
            this.pb_separator4.Name = "pb_separator4";
            this.pb_separator4.Size = new System.Drawing.Size(596, 18);
            this.pb_separator4.TabIndex = 255;
            this.pb_separator4.TabStop = false;
            // 
            // pnl_robotGripperOpen
            // 
            this.pnl_robotGripperOpen.Controls.Add(this.pnl_ledRobotGripperOpen);
            this.pnl_robotGripperOpen.Controls.Add(this.pnl_infoRobotGripperOpen);
            this.pnl_robotGripperOpen.Controls.Add(this.lbl_robotGripperOpen);
            this.pnl_robotGripperOpen.Location = new System.Drawing.Point(5, 235);
            this.pnl_robotGripperOpen.Name = "pnl_robotGripperOpen";
            this.pnl_robotGripperOpen.Size = new System.Drawing.Size(578, 56);
            this.pnl_robotGripperOpen.TabIndex = 254;
            // 
            // pnl_ledRobotGripperOpen
            // 
            this.pnl_ledRobotGripperOpen.BackgroundImage = global::RM.Properties.Resources.emptyCircle;
            this.pnl_ledRobotGripperOpen.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_ledRobotGripperOpen.Location = new System.Drawing.Point(65, 3);
            this.pnl_ledRobotGripperOpen.Name = "pnl_ledRobotGripperOpen";
            this.pnl_ledRobotGripperOpen.Size = new System.Drawing.Size(40, 40);
            this.pnl_ledRobotGripperOpen.TabIndex = 261;
            // 
            // pnl_infoRobotGripperOpen
            // 
            this.pnl_infoRobotGripperOpen.BackgroundImage = global::RM.Properties.Resources.info;
            this.pnl_infoRobotGripperOpen.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_infoRobotGripperOpen.Location = new System.Drawing.Point(515, 8);
            this.pnl_infoRobotGripperOpen.Name = "pnl_infoRobotGripperOpen";
            this.pnl_infoRobotGripperOpen.Size = new System.Drawing.Size(30, 30);
            this.pnl_infoRobotGripperOpen.TabIndex = 4;
            this.pnl_infoRobotGripperOpen.Click += new System.EventHandler(this.PnlInfoRobotGripperOpen_Click);
            // 
            // lbl_robotGripperOpen
            // 
            this.lbl_robotGripperOpen.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_robotGripperOpen.ForeColor = System.Drawing.Color.Black;
            this.lbl_robotGripperOpen.Location = new System.Drawing.Point(159, 0);
            this.lbl_robotGripperOpen.Name = "lbl_robotGripperOpen";
            this.lbl_robotGripperOpen.Size = new System.Drawing.Size(313, 50);
            this.lbl_robotGripperOpen.TabIndex = 0;
            this.lbl_robotGripperOpen.Text = "Pinze robot aperte";
            this.lbl_robotGripperOpen.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pb_separator3
            // 
            this.pb_separator3.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_separator3.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_separator3.Location = new System.Drawing.Point(5, 219);
            this.pb_separator3.Name = "pb_separator3";
            this.pb_separator3.Size = new System.Drawing.Size(596, 18);
            this.pb_separator3.TabIndex = 253;
            this.pb_separator3.TabStop = false;
            // 
            // pnl_lbl_tapeInit
            // 
            this.pnl_lbl_tapeInit.Controls.Add(this.pnl_ledTapeInit);
            this.pnl_lbl_tapeInit.Controls.Add(this.pnl_infoTapeInit);
            this.pnl_lbl_tapeInit.Controls.Add(this.lbl_tapeInit);
            this.pnl_lbl_tapeInit.Location = new System.Drawing.Point(5, 173);
            this.pnl_lbl_tapeInit.Name = "pnl_lbl_tapeInit";
            this.pnl_lbl_tapeInit.Size = new System.Drawing.Size(578, 56);
            this.pnl_lbl_tapeInit.TabIndex = 252;
            // 
            // pnl_ledTapeInit
            // 
            this.pnl_ledTapeInit.BackgroundImage = global::RM.Properties.Resources.emptyCircle;
            this.pnl_ledTapeInit.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_ledTapeInit.Location = new System.Drawing.Point(65, 3);
            this.pnl_ledTapeInit.Name = "pnl_ledTapeInit";
            this.pnl_ledTapeInit.Size = new System.Drawing.Size(40, 40);
            this.pnl_ledTapeInit.TabIndex = 261;
            // 
            // pnl_infoTapeInit
            // 
            this.pnl_infoTapeInit.BackgroundImage = global::RM.Properties.Resources.info;
            this.pnl_infoTapeInit.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_infoTapeInit.Location = new System.Drawing.Point(515, 8);
            this.pnl_infoTapeInit.Name = "pnl_infoTapeInit";
            this.pnl_infoTapeInit.Size = new System.Drawing.Size(30, 30);
            this.pnl_infoTapeInit.TabIndex = 4;
            this.pnl_infoTapeInit.Click += new System.EventHandler(this.PnlInfoTapeInit_Click);
            // 
            // lbl_tapeInit
            // 
            this.lbl_tapeInit.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_tapeInit.ForeColor = System.Drawing.Color.Black;
            this.lbl_tapeInit.Location = new System.Drawing.Point(159, 0);
            this.lbl_tapeInit.Name = "lbl_tapeInit";
            this.lbl_tapeInit.Size = new System.Drawing.Size(313, 50);
            this.lbl_tapeInit.TabIndex = 0;
            this.lbl_tapeInit.Text = "Init nastro";
            this.lbl_tapeInit.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pb_separator2
            // 
            this.pb_separator2.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_separator2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_separator2.Location = new System.Drawing.Point(5, 157);
            this.pb_separator2.Name = "pb_separator2";
            this.pb_separator2.Size = new System.Drawing.Size(596, 18);
            this.pb_separator2.TabIndex = 251;
            this.pb_separator2.TabStop = false;
            // 
            // pnl_robotReady
            // 
            this.pnl_robotReady.Controls.Add(this.pnl_ledRobotReady);
            this.pnl_robotReady.Controls.Add(this.pnl_infoRobotReady);
            this.pnl_robotReady.Controls.Add(this.lbl_robotReady);
            this.pnl_robotReady.Location = new System.Drawing.Point(5, 111);
            this.pnl_robotReady.Name = "pnl_robotReady";
            this.pnl_robotReady.Size = new System.Drawing.Size(578, 56);
            this.pnl_robotReady.TabIndex = 250;
            // 
            // pnl_ledRobotReady
            // 
            this.pnl_ledRobotReady.BackgroundImage = global::RM.Properties.Resources.emptyCircle;
            this.pnl_ledRobotReady.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_ledRobotReady.Location = new System.Drawing.Point(65, 3);
            this.pnl_ledRobotReady.Name = "pnl_ledRobotReady";
            this.pnl_ledRobotReady.Size = new System.Drawing.Size(40, 40);
            this.pnl_ledRobotReady.TabIndex = 4;
            // 
            // pnl_infoRobotReady
            // 
            this.pnl_infoRobotReady.BackgroundImage = global::RM.Properties.Resources.info;
            this.pnl_infoRobotReady.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_infoRobotReady.Location = new System.Drawing.Point(515, 8);
            this.pnl_infoRobotReady.Name = "pnl_infoRobotReady";
            this.pnl_infoRobotReady.Size = new System.Drawing.Size(30, 30);
            this.pnl_infoRobotReady.TabIndex = 4;
            this.pnl_infoRobotReady.Click += new System.EventHandler(this.PnlInfoRobotReady_Click);
            // 
            // lbl_robotReady
            // 
            this.lbl_robotReady.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_robotReady.ForeColor = System.Drawing.Color.Black;
            this.lbl_robotReady.Location = new System.Drawing.Point(159, 0);
            this.lbl_robotReady.Name = "lbl_robotReady";
            this.lbl_robotReady.Size = new System.Drawing.Size(313, 50);
            this.lbl_robotReady.TabIndex = 0;
            this.lbl_robotReady.Text = "Robot pronto";
            this.lbl_robotReady.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pb_separator1
            // 
            this.pb_separator1.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_separator1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_separator1.Location = new System.Drawing.Point(5, 95);
            this.pb_separator1.Name = "pb_separator1";
            this.pb_separator1.Size = new System.Drawing.Size(596, 18);
            this.pb_separator1.TabIndex = 249;
            this.pb_separator1.TabStop = false;
            // 
            // pnl_navigation
            // 
            this.pnl_navigation.BackColor = System.Drawing.Color.LightSlateGray;
            this.pnl_navigation.Controls.Add(this.pnl_buttonHome);
            this.pnl_navigation.Location = new System.Drawing.Point(0, 557);
            this.pnl_navigation.Name = "pnl_navigation";
            this.pnl_navigation.Size = new System.Drawing.Size(1024, 101);
            this.pnl_navigation.TabIndex = 246;
            // 
            // pnl_buttonHome
            // 
            this.pnl_buttonHome.Controls.Add(this.btn_home);
            this.pnl_buttonHome.Controls.Add(this.lbl_buttonHome);
            this.pnl_buttonHome.ForeColor = System.Drawing.SystemColors.Control;
            this.pnl_buttonHome.Location = new System.Drawing.Point(15, 8);
            this.pnl_buttonHome.Name = "pnl_buttonHome";
            this.pnl_buttonHome.Size = new System.Drawing.Size(70, 93);
            this.pnl_buttonHome.TabIndex = 35;
            // 
            // btn_home
            // 
            this.btn_home.BackColor = System.Drawing.Color.White;
            this.btn_home.BackgroundColor = System.Drawing.Color.White;
            this.btn_home.BackgroundImage = global::RM.Properties.Resources.go_back_arrow;
            this.btn_home.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_home.BorderColor = System.Drawing.Color.PaleVioletRed;
            this.btn_home.BorderRadius = 15;
            this.btn_home.BorderSize = 0;
            this.btn_home.FlatAppearance.BorderSize = 0;
            this.btn_home.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_home.ForeColor = System.Drawing.Color.White;
            this.btn_home.Location = new System.Drawing.Point(0, 0);
            this.btn_home.Name = "btn_home";
            this.btn_home.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_home.Size = new System.Drawing.Size(70, 70);
            this.btn_home.TabIndex = 2;
            this.btn_home.TextColor = System.Drawing.Color.White;
            this.btn_home.UseVisualStyleBackColor = false;
            this.btn_home.Click += new System.EventHandler(this.ClickEvent_back);
            // 
            // lbl_buttonHome
            // 
            this.lbl_buttonHome.BackColor = System.Drawing.Color.Transparent;
            this.lbl_buttonHome.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_buttonHome.Location = new System.Drawing.Point(0, 73);
            this.lbl_buttonHome.Name = "lbl_buttonHome";
            this.lbl_buttonHome.Size = new System.Drawing.Size(70, 27);
            this.lbl_buttonHome.TabIndex = 1;
            this.lbl_buttonHome.Text = "Home";
            this.lbl_buttonHome.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // UC_permissions
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.Control;
            this.Controls.Add(this.pnl_navigation);
            this.Controls.Add(this.pnl_container);
            this.Name = "UC_permissions";
            this.Size = new System.Drawing.Size(1024, 658);
            this.pnl_stopCycleAlarm.ResumeLayout(false);
            this.pnl_headerContainer.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_headerIcon)).EndInit();
            this.pnl_container.ResumeLayout(false);
            this.pnl_robotInPosition.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator5)).EndInit();
            this.pnl_barrier.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator4)).EndInit();
            this.pnl_robotGripperOpen.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator3)).EndInit();
            this.pnl_lbl_tapeInit.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator2)).EndInit();
            this.pnl_robotReady.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator1)).EndInit();
            this.pnl_navigation.ResumeLayout(false);
            this.pnl_buttonHome.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Panel pnl_stopCycleAlarm;
        private System.Windows.Forms.Label lbl_stopCycleAlarm;
        private RMLib.View.CustomPanel pnl_headerContainer;
        private System.Windows.Forms.Label lbl_header;
        private RMLib.View.CustomPanel pnl_container;
        private System.Windows.Forms.Panel pnl_navigation;
        private System.Windows.Forms.Panel pnl_buttonHome;
        private RMLib.View.CustomButton btn_home;
        private System.Windows.Forms.Label lbl_buttonHome;
        private System.Windows.Forms.PictureBox pb_headerIcon;
        private System.Windows.Forms.Panel pnl_ledStopCycleAlarm;
        private System.Windows.Forms.PictureBox pb_separator1;
        private System.Windows.Forms.Panel pnl_infoStopCycleAlarm;
        private System.Windows.Forms.PictureBox pb_separator2;
        private System.Windows.Forms.Panel pnl_robotReady;
        private System.Windows.Forms.Panel pnl_infoRobotReady;
        private System.Windows.Forms.Label lbl_robotReady;
        private System.Windows.Forms.PictureBox pb_separator3;
        private System.Windows.Forms.Panel pnl_lbl_tapeInit;
        private System.Windows.Forms.Panel pnl_infoTapeInit;
        private System.Windows.Forms.Label lbl_tapeInit;
        private System.Windows.Forms.Panel pnl_ledTapeInit;
        private System.Windows.Forms.Panel pnl_ledRobotReady;
        private System.Windows.Forms.Panel pnl_robotInPosition;
        private System.Windows.Forms.Panel pnl_ledRobotInPosition;
        private System.Windows.Forms.Panel pnl_infoRobotInPosition;
        private System.Windows.Forms.Label lbl_robotInPosition;
        private System.Windows.Forms.PictureBox pb_separator5;
        private System.Windows.Forms.Panel pnl_barrier;
        private System.Windows.Forms.Panel pnl_ledBarrier;
        private System.Windows.Forms.Panel pnl_infoBarrier;
        private System.Windows.Forms.Label lbl_barrier;
        private System.Windows.Forms.PictureBox pb_separator4;
        private System.Windows.Forms.Panel pnl_robotGripperOpen;
        private System.Windows.Forms.Panel pnl_ledRobotGripperOpen;
        private System.Windows.Forms.Panel pnl_infoRobotGripperOpen;
        private System.Windows.Forms.Label lbl_robotGripperOpen;
    }
}
