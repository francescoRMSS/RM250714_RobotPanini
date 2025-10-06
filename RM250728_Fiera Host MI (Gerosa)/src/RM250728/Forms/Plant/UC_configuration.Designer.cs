namespace RM.src.RM250728
{
    partial class UC_configuration
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
            this.pnl_navigation = new System.Windows.Forms.Panel();
            this.pnl_buttonHome = new System.Windows.Forms.Panel();
            this.btn_home = new RMLib.View.CustomButton();
            this.lbl_buttonHome = new System.Windows.Forms.Label();
            this.pnl_container = new RMLib.View.CustomPanel();
            this.pnl_USCleaning = new System.Windows.Forms.Panel();
            this.btn_resetTrays = new RMLib.View.CustomButton();
            this.lbl_trays = new System.Windows.Forms.Label();
            this.lbl_resetTrays = new System.Windows.Forms.Label();
            this.pb_line3 = new System.Windows.Forms.PictureBox();
            this.btn_openRobotGripper = new RMLib.View.CustomButton();
            this.btn_closeRobotGripper = new RMLib.View.CustomButton();
            this.lbl_closeGrippers = new System.Windows.Forms.Label();
            this.lbl_grippers = new System.Windows.Forms.Label();
            this.lbl_openGrippers = new System.Windows.Forms.Label();
            this.pb_line2 = new System.Windows.Forms.PictureBox();
            this.lbl_USCleaning = new System.Windows.Forms.Label();
            this.pnl_USActivation = new System.Windows.Forms.Panel();
            this.lbl_US3 = new System.Windows.Forms.Label();
            this.btn_jogTape = new RMLib.View.CustomButton();
            this.lbl_USActivation = new System.Windows.Forms.Label();
            this.pb_line1 = new System.Windows.Forms.PictureBox();
            this.pnl_headerContainer = new RMLib.View.CustomPanel();
            this.pb_config = new System.Windows.Forms.PictureBox();
            this.lbl_title = new System.Windows.Forms.Label();
            this.pnl_navigation.SuspendLayout();
            this.pnl_buttonHome.SuspendLayout();
            this.pnl_container.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_line3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_line2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_line1)).BeginInit();
            this.pnl_headerContainer.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_config)).BeginInit();
            this.SuspendLayout();
            // 
            // pnl_navigation
            // 
            this.pnl_navigation.BackColor = System.Drawing.Color.LightSlateGray;
            this.pnl_navigation.Controls.Add(this.pnl_buttonHome);
            this.pnl_navigation.Location = new System.Drawing.Point(0, 557);
            this.pnl_navigation.Name = "pnl_navigation";
            this.pnl_navigation.Size = new System.Drawing.Size(1024, 101);
            this.pnl_navigation.TabIndex = 247;
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
            this.btn_home.Click += new System.EventHandler(this.BtnHome_Click);
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
            // pnl_container
            // 
            this.pnl_container.BackColor = System.Drawing.Color.WhiteSmoke;
            this.pnl_container.BackgroundColor = System.Drawing.Color.WhiteSmoke;
            this.pnl_container.BorderColor = System.Drawing.Color.DarkBlue;
            this.pnl_container.BorderRadius = 10;
            this.pnl_container.BorderSize = 2;
            this.pnl_container.Controls.Add(this.pnl_USCleaning);
            this.pnl_container.Controls.Add(this.btn_resetTrays);
            this.pnl_container.Controls.Add(this.lbl_trays);
            this.pnl_container.Controls.Add(this.lbl_resetTrays);
            this.pnl_container.Controls.Add(this.pb_line3);
            this.pnl_container.Controls.Add(this.btn_openRobotGripper);
            this.pnl_container.Controls.Add(this.btn_closeRobotGripper);
            this.pnl_container.Controls.Add(this.lbl_closeGrippers);
            this.pnl_container.Controls.Add(this.lbl_grippers);
            this.pnl_container.Controls.Add(this.lbl_openGrippers);
            this.pnl_container.Controls.Add(this.pb_line2);
            this.pnl_container.Controls.Add(this.lbl_USCleaning);
            this.pnl_container.Controls.Add(this.pnl_USActivation);
            this.pnl_container.Controls.Add(this.lbl_US3);
            this.pnl_container.Controls.Add(this.btn_jogTape);
            this.pnl_container.Controls.Add(this.lbl_USActivation);
            this.pnl_container.Controls.Add(this.pb_line1);
            this.pnl_container.Controls.Add(this.pnl_headerContainer);
            this.pnl_container.ForeColor = System.Drawing.Color.White;
            this.pnl_container.Location = new System.Drawing.Point(210, 40);
            this.pnl_container.Name = "pnl_container";
            this.pnl_container.Size = new System.Drawing.Size(604, 478);
            this.pnl_container.TabIndex = 248;
            this.pnl_container.TextColor = System.Drawing.Color.White;
            // 
            // pnl_USCleaning
            // 
            this.pnl_USCleaning.BackgroundImage = global::RM.Properties.Resources.off_button;
            this.pnl_USCleaning.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_USCleaning.Location = new System.Drawing.Point(430, 180);
            this.pnl_USCleaning.Name = "pnl_USCleaning";
            this.pnl_USCleaning.Size = new System.Drawing.Size(62, 62);
            this.pnl_USCleaning.TabIndex = 263;
            this.pnl_USCleaning.Click += new System.EventHandler(this.PnlUSCleaning_Click);
            // 
            // btn_resetTrays
            // 
            this.btn_resetTrays.BackColor = System.Drawing.SystemColors.ActiveBorder;
            this.btn_resetTrays.BackgroundColor = System.Drawing.SystemColors.ActiveBorder;
            this.btn_resetTrays.BorderColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.btn_resetTrays.BorderRadius = 10;
            this.btn_resetTrays.BorderSize = 1;
            this.btn_resetTrays.FlatAppearance.BorderSize = 0;
            this.btn_resetTrays.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_resetTrays.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_resetTrays.ForeColor = System.Drawing.Color.Black;
            this.btn_resetTrays.Location = new System.Drawing.Point(384, 408);
            this.btn_resetTrays.Name = "btn_resetTrays";
            this.btn_resetTrays.Size = new System.Drawing.Size(150, 40);
            this.btn_resetTrays.TabIndex = 273;
            this.btn_resetTrays.Text = "RESET";
            this.btn_resetTrays.TextColor = System.Drawing.Color.Black;
            this.btn_resetTrays.UseVisualStyleBackColor = false;
            this.btn_resetTrays.MouseDown += new System.Windows.Forms.MouseEventHandler(this.btn_resetTrays_MouseDown);
            this.btn_resetTrays.MouseUp += new System.Windows.Forms.MouseEventHandler(this.btn_resetTrays_MouseUp);
            // 
            // lbl_trays
            // 
            this.lbl_trays.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_trays.ForeColor = System.Drawing.Color.DarkGray;
            this.lbl_trays.Location = new System.Drawing.Point(55, 370);
            this.lbl_trays.Name = "lbl_trays";
            this.lbl_trays.Size = new System.Drawing.Size(54, 18);
            this.lbl_trays.TabIndex = 271;
            this.lbl_trays.Text = "Vassoi";
            this.lbl_trays.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_resetTrays
            // 
            this.lbl_resetTrays.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_resetTrays.ForeColor = System.Drawing.Color.Black;
            this.lbl_resetTrays.Location = new System.Drawing.Point(78, 403);
            this.lbl_resetTrays.Name = "lbl_resetTrays";
            this.lbl_resetTrays.Size = new System.Drawing.Size(313, 50);
            this.lbl_resetTrays.TabIndex = 272;
            this.lbl_resetTrays.Text = "Forzamento teglie vuote";
            this.lbl_resetTrays.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pb_line3
            // 
            this.pb_line3.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_line3.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_line3.Location = new System.Drawing.Point(58, 370);
            this.pb_line3.Name = "pb_line3";
            this.pb_line3.Size = new System.Drawing.Size(542, 18);
            this.pb_line3.TabIndex = 270;
            this.pb_line3.TabStop = false;
            // 
            // btn_openRobotGripper
            // 
            this.btn_openRobotGripper.BackColor = System.Drawing.SystemColors.ActiveBorder;
            this.btn_openRobotGripper.BackgroundColor = System.Drawing.SystemColors.ActiveBorder;
            this.btn_openRobotGripper.BorderColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.btn_openRobotGripper.BorderRadius = 10;
            this.btn_openRobotGripper.BorderSize = 1;
            this.btn_openRobotGripper.FlatAppearance.BorderSize = 0;
            this.btn_openRobotGripper.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_openRobotGripper.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_openRobotGripper.ForeColor = System.Drawing.Color.Black;
            this.btn_openRobotGripper.Location = new System.Drawing.Point(384, 274);
            this.btn_openRobotGripper.Name = "btn_openRobotGripper";
            this.btn_openRobotGripper.Size = new System.Drawing.Size(150, 40);
            this.btn_openRobotGripper.TabIndex = 269;
            this.btn_openRobotGripper.Text = "ON";
            this.btn_openRobotGripper.TextColor = System.Drawing.Color.Black;
            this.btn_openRobotGripper.UseVisualStyleBackColor = false;
            this.btn_openRobotGripper.MouseDown += new System.Windows.Forms.MouseEventHandler(this.btn_openRobotGripper_MouseDown);
            this.btn_openRobotGripper.MouseUp += new System.Windows.Forms.MouseEventHandler(this.btn_openRobotGripper_MouseUp);
            // 
            // btn_closeRobotGripper
            // 
            this.btn_closeRobotGripper.BackColor = System.Drawing.SystemColors.ActiveBorder;
            this.btn_closeRobotGripper.BackgroundColor = System.Drawing.SystemColors.ActiveBorder;
            this.btn_closeRobotGripper.BorderColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.btn_closeRobotGripper.BorderRadius = 10;
            this.btn_closeRobotGripper.BorderSize = 1;
            this.btn_closeRobotGripper.FlatAppearance.BorderSize = 0;
            this.btn_closeRobotGripper.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_closeRobotGripper.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_closeRobotGripper.ForeColor = System.Drawing.Color.Black;
            this.btn_closeRobotGripper.Location = new System.Drawing.Point(384, 324);
            this.btn_closeRobotGripper.Name = "btn_closeRobotGripper";
            this.btn_closeRobotGripper.Size = new System.Drawing.Size(150, 40);
            this.btn_closeRobotGripper.TabIndex = 268;
            this.btn_closeRobotGripper.Text = "ON";
            this.btn_closeRobotGripper.TextColor = System.Drawing.Color.Black;
            this.btn_closeRobotGripper.UseVisualStyleBackColor = false;
            this.btn_closeRobotGripper.MouseDown += new System.Windows.Forms.MouseEventHandler(this.btn_closeRobotGripper_MouseDown);
            this.btn_closeRobotGripper.MouseUp += new System.Windows.Forms.MouseEventHandler(this.btn_closeRobotGripper_MouseUp);
            // 
            // lbl_closeGrippers
            // 
            this.lbl_closeGrippers.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_closeGrippers.ForeColor = System.Drawing.Color.Black;
            this.lbl_closeGrippers.Location = new System.Drawing.Point(78, 319);
            this.lbl_closeGrippers.Name = "lbl_closeGrippers";
            this.lbl_closeGrippers.Size = new System.Drawing.Size(313, 50);
            this.lbl_closeGrippers.TabIndex = 267;
            this.lbl_closeGrippers.Text = "Chiudi pinze robot";
            this.lbl_closeGrippers.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_grippers
            // 
            this.lbl_grippers.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_grippers.ForeColor = System.Drawing.Color.DarkGray;
            this.lbl_grippers.Location = new System.Drawing.Point(55, 236);
            this.lbl_grippers.Name = "lbl_grippers";
            this.lbl_grippers.Size = new System.Drawing.Size(54, 18);
            this.lbl_grippers.TabIndex = 265;
            this.lbl_grippers.Text = "Pinze";
            this.lbl_grippers.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_openGrippers
            // 
            this.lbl_openGrippers.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_openGrippers.ForeColor = System.Drawing.Color.Black;
            this.lbl_openGrippers.Location = new System.Drawing.Point(78, 269);
            this.lbl_openGrippers.Name = "lbl_openGrippers";
            this.lbl_openGrippers.Size = new System.Drawing.Size(313, 50);
            this.lbl_openGrippers.TabIndex = 266;
            this.lbl_openGrippers.Text = "Apri pinze robot";
            this.lbl_openGrippers.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pb_line2
            // 
            this.pb_line2.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_line2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_line2.Location = new System.Drawing.Point(58, 236);
            this.pb_line2.Name = "pb_line2";
            this.pb_line2.Size = new System.Drawing.Size(542, 18);
            this.pb_line2.TabIndex = 264;
            this.pb_line2.TabStop = false;
            // 
            // lbl_USCleaning
            // 
            this.lbl_USCleaning.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_USCleaning.ForeColor = System.Drawing.Color.Black;
            this.lbl_USCleaning.Location = new System.Drawing.Point(78, 183);
            this.lbl_USCleaning.Name = "lbl_USCleaning";
            this.lbl_USCleaning.Size = new System.Drawing.Size(313, 50);
            this.lbl_USCleaning.TabIndex = 262;
            this.lbl_USCleaning.Text = "Pulizia US";
            this.lbl_USCleaning.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pnl_USActivation
            // 
            this.pnl_USActivation.BackgroundImage = global::RM.Properties.Resources.off_button;
            this.pnl_USActivation.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_USActivation.Location = new System.Drawing.Point(430, 130);
            this.pnl_USActivation.Name = "pnl_USActivation";
            this.pnl_USActivation.Size = new System.Drawing.Size(62, 62);
            this.pnl_USActivation.TabIndex = 261;
            this.pnl_USActivation.Click += new System.EventHandler(this.PnlUSActivation_Click);
            // 
            // lbl_US3
            // 
            this.lbl_US3.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_US3.ForeColor = System.Drawing.Color.DarkGray;
            this.lbl_US3.Location = new System.Drawing.Point(55, 100);
            this.lbl_US3.Name = "lbl_US3";
            this.lbl_US3.Size = new System.Drawing.Size(54, 18);
            this.lbl_US3.TabIndex = 260;
            this.lbl_US3.Text = "US";
            this.lbl_US3.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // btn_jogTape
            // 
            this.btn_jogTape.BackColor = System.Drawing.SystemColors.ActiveBorder;
            this.btn_jogTape.BackgroundColor = System.Drawing.SystemColors.ActiveBorder;
            this.btn_jogTape.BorderColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.btn_jogTape.BorderRadius = 10;
            this.btn_jogTape.BorderSize = 1;
            this.btn_jogTape.FlatAppearance.BorderSize = 0;
            this.btn_jogTape.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_jogTape.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_jogTape.ForeColor = System.Drawing.Color.Black;
            this.btn_jogTape.Location = new System.Drawing.Point(58, 54);
            this.btn_jogTape.Name = "btn_jogTape";
            this.btn_jogTape.Size = new System.Drawing.Size(150, 40);
            this.btn_jogTape.TabIndex = 259;
            this.btn_jogTape.Text = "Jog nastro";
            this.btn_jogTape.TextColor = System.Drawing.Color.Black;
            this.btn_jogTape.UseVisualStyleBackColor = false;
            this.btn_jogTape.MouseDown += new System.Windows.Forms.MouseEventHandler(this.btn_jogTape_MouseDown);
            this.btn_jogTape.MouseUp += new System.Windows.Forms.MouseEventHandler(this.btn_jogTape_MouseUp);
            // 
            // lbl_USActivation
            // 
            this.lbl_USActivation.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_USActivation.ForeColor = System.Drawing.Color.Black;
            this.lbl_USActivation.Location = new System.Drawing.Point(78, 133);
            this.lbl_USActivation.Name = "lbl_USActivation";
            this.lbl_USActivation.Size = new System.Drawing.Size(313, 50);
            this.lbl_USActivation.TabIndex = 0;
            this.lbl_USActivation.Text = "Abilitazione US";
            this.lbl_USActivation.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // pb_line1
            // 
            this.pb_line1.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_line1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_line1.Location = new System.Drawing.Point(58, 100);
            this.pb_line1.Name = "pb_line1";
            this.pb_line1.Size = new System.Drawing.Size(542, 18);
            this.pb_line1.TabIndex = 257;
            this.pb_line1.TabStop = false;
            // 
            // pnl_headerContainer
            // 
            this.pnl_headerContainer.BackColor = System.Drawing.Color.DarkBlue;
            this.pnl_headerContainer.BackgroundColor = System.Drawing.Color.DarkBlue;
            this.pnl_headerContainer.BorderColor = System.Drawing.SystemColors.ButtonShadow;
            this.pnl_headerContainer.BorderRadius = 5;
            this.pnl_headerContainer.BorderSize = 1;
            this.pnl_headerContainer.Controls.Add(this.pb_config);
            this.pnl_headerContainer.Controls.Add(this.lbl_title);
            this.pnl_headerContainer.ForeColor = System.Drawing.Color.White;
            this.pnl_headerContainer.Location = new System.Drawing.Point(0, 0);
            this.pnl_headerContainer.Name = "pnl_headerContainer";
            this.pnl_headerContainer.Size = new System.Drawing.Size(604, 42);
            this.pnl_headerContainer.TabIndex = 248;
            this.pnl_headerContainer.TextColor = System.Drawing.Color.White;
            // 
            // pb_config
            // 
            this.pb_config.BackgroundImage = global::RM.Properties.Resources.settings_white;
            this.pb_config.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pb_config.Location = new System.Drawing.Point(15, 5);
            this.pb_config.Name = "pb_config";
            this.pb_config.Size = new System.Drawing.Size(32, 32);
            this.pb_config.TabIndex = 254;
            this.pb_config.TabStop = false;
            // 
            // lbl_title
            // 
            this.lbl_title.BackColor = System.Drawing.Color.Transparent;
            this.lbl_title.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_title.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_title.Location = new System.Drawing.Point(3, 0);
            this.lbl_title.Name = "lbl_title";
            this.lbl_title.Size = new System.Drawing.Size(597, 42);
            this.lbl_title.TabIndex = 3;
            this.lbl_title.Text = "Configurazione";
            this.lbl_title.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // UC_configuration
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.Control;
            this.Controls.Add(this.pnl_container);
            this.Controls.Add(this.pnl_navigation);
            this.Name = "UC_configuration";
            this.Size = new System.Drawing.Size(1024, 658);
            this.pnl_navigation.ResumeLayout(false);
            this.pnl_buttonHome.ResumeLayout(false);
            this.pnl_container.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_line3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_line2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_line1)).EndInit();
            this.pnl_headerContainer.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_config)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Panel pnl_navigation;
        private System.Windows.Forms.Panel pnl_buttonHome;
        private RMLib.View.CustomButton btn_home;
        private System.Windows.Forms.Label lbl_buttonHome;
        private RMLib.View.CustomPanel pnl_container;
        private RMLib.View.CustomButton btn_resetTrays;
        private System.Windows.Forms.Label lbl_trays;
        private System.Windows.Forms.Label lbl_resetTrays;
        private System.Windows.Forms.PictureBox pb_line3;
        private RMLib.View.CustomButton btn_openRobotGripper;
        private RMLib.View.CustomButton btn_closeRobotGripper;
        private System.Windows.Forms.Label lbl_closeGrippers;
        private System.Windows.Forms.Label lbl_grippers;
        private System.Windows.Forms.Label lbl_openGrippers;
        private System.Windows.Forms.PictureBox pb_line2;
        private System.Windows.Forms.Panel pnl_USCleaning;
        private System.Windows.Forms.Label lbl_USCleaning;
        private System.Windows.Forms.Panel pnl_USActivation;
        private System.Windows.Forms.Label lbl_US3;
        private RMLib.View.CustomButton btn_jogTape;
        private System.Windows.Forms.Label lbl_USActivation;
        private System.Windows.Forms.PictureBox pb_line1;
        private RMLib.View.CustomPanel pnl_headerContainer;
        private System.Windows.Forms.PictureBox pb_config;
        private System.Windows.Forms.Label lbl_title;
    }
}
