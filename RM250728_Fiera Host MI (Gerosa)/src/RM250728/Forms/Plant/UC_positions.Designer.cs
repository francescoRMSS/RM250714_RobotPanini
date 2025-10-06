namespace RM.src.RM250728
{
    partial class UC_positions
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
            this.columnHeader10 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.lbl_buttonRecPoint = new System.Windows.Forms.Label();
            this.lbl_buttonDeletePoint = new System.Windows.Forms.Label();
            this.lbl_buttonRenamePoint = new System.Windows.Forms.Label();
            this.lbl_buttonNewPoint = new System.Windows.Forms.Label();
            this.lbl_buttonHome = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.panel5 = new System.Windows.Forms.Panel();
            this.panel1 = new System.Windows.Forms.Panel();
            this.pnl_loading = new System.Windows.Forms.Panel();
            this.prgs_loading = new System.Windows.Forms.ProgressBar();
            this.BtnDeletePoint = new System.Windows.Forms.Button();
            this.BtnRecPoint = new System.Windows.Forms.Button();
            this.BtnRenamePoint = new System.Windows.Forms.Button();
            this.btn_addPoint_ = new System.Windows.Forms.Button();
            this.btn_home_ = new System.Windows.Forms.Button();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.panel5.SuspendLayout();
            this.panel1.SuspendLayout();
            this.pnl_loading.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.SuspendLayout();
            // 
            // lw_positions
            // 
            this.lw_positions.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] {
            this.columnHeader1,
            this.columnHeader2,
            this.columnHeader3,
            this.columnHeader4,
            this.columnHeader5,
            this.columnHeader6,
            this.columnHeader7,
            this.columnHeader8,
            this.columnHeader9,
            this.columnHeader10});
            this.lw_positions.Cursor = System.Windows.Forms.Cursors.Hand;
            this.lw_positions.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F);
            this.lw_positions.FullRowSelect = true;
            this.lw_positions.HideSelection = false;
            this.lw_positions.LabelWrap = false;
            this.lw_positions.Location = new System.Drawing.Point(3, 40);
            this.lw_positions.MultiSelect = false;
            this.lw_positions.Name = "lw_positions";
            this.lw_positions.Size = new System.Drawing.Size(988, 453);
            this.lw_positions.TabIndex = 67;
            this.lw_positions.UseCompatibleStateImageBehavior = false;
            this.lw_positions.View = System.Windows.Forms.View.Details;
            // 
            // columnHeader1
            // 
            this.columnHeader1.Text = "ID";
            this.columnHeader1.Width = 40;
            // 
            // columnHeader2
            // 
            this.columnHeader2.Text = "Timestamp";
            this.columnHeader2.Width = 120;
            // 
            // columnHeader3
            // 
            this.columnHeader3.Text = "Mode";
            this.columnHeader3.Width = 80;
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
            // columnHeader10
            // 
            this.columnHeader10.Text = "Name";
            this.columnHeader10.Width = 143;
            // 
            // lbl_buttonRecPoint
            // 
            this.lbl_buttonRecPoint.BackColor = System.Drawing.Color.Transparent;
            this.lbl_buttonRecPoint.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_buttonRecPoint.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_buttonRecPoint.Location = new System.Drawing.Point(319, 80);
            this.lbl_buttonRecPoint.Name = "lbl_buttonRecPoint";
            this.lbl_buttonRecPoint.Size = new System.Drawing.Size(60, 22);
            this.lbl_buttonRecPoint.TabIndex = 1;
            this.lbl_buttonRecPoint.Text = "Registra";
            this.lbl_buttonRecPoint.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // lbl_buttonDeletePoint
            // 
            this.lbl_buttonDeletePoint.BackColor = System.Drawing.Color.Transparent;
            this.lbl_buttonDeletePoint.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_buttonDeletePoint.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_buttonDeletePoint.Location = new System.Drawing.Point(410, 80);
            this.lbl_buttonDeletePoint.Name = "lbl_buttonDeletePoint";
            this.lbl_buttonDeletePoint.Size = new System.Drawing.Size(72, 22);
            this.lbl_buttonDeletePoint.TabIndex = 1;
            this.lbl_buttonDeletePoint.Text = "Cancella";
            this.lbl_buttonDeletePoint.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // lbl_buttonRenamePoint
            // 
            this.lbl_buttonRenamePoint.BackColor = System.Drawing.Color.Transparent;
            this.lbl_buttonRenamePoint.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_buttonRenamePoint.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_buttonRenamePoint.Location = new System.Drawing.Point(211, 80);
            this.lbl_buttonRenamePoint.Name = "lbl_buttonRenamePoint";
            this.lbl_buttonRenamePoint.Size = new System.Drawing.Size(78, 22);
            this.lbl_buttonRenamePoint.TabIndex = 1;
            this.lbl_buttonRenamePoint.Text = "Rinomina";
            this.lbl_buttonRenamePoint.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // lbl_buttonNewPoint
            // 
            this.lbl_buttonNewPoint.BackColor = System.Drawing.Color.Transparent;
            this.lbl_buttonNewPoint.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_buttonNewPoint.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_buttonNewPoint.Location = new System.Drawing.Point(101, 80);
            this.lbl_buttonNewPoint.Name = "lbl_buttonNewPoint";
            this.lbl_buttonNewPoint.Size = new System.Drawing.Size(94, 22);
            this.lbl_buttonNewPoint.TabIndex = 1;
            this.lbl_buttonNewPoint.Text = "Nuovo punto";
            this.lbl_buttonNewPoint.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // lbl_buttonHome
            // 
            this.lbl_buttonHome.BackColor = System.Drawing.Color.Transparent;
            this.lbl_buttonHome.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_buttonHome.ForeColor = System.Drawing.SystemColors.Control;
            this.lbl_buttonHome.Location = new System.Drawing.Point(22, 80);
            this.lbl_buttonHome.Name = "lbl_buttonHome";
            this.lbl_buttonHome.Size = new System.Drawing.Size(60, 22);
            this.lbl_buttonHome.TabIndex = 1;
            this.lbl_buttonHome.Text = "Home";
            this.lbl_buttonHome.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // label6
            // 
            this.label6.BackColor = System.Drawing.Color.Black;
            this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.label6.ForeColor = System.Drawing.Color.White;
            this.label6.Location = new System.Drawing.Point(1, 1);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(996, 40);
            this.label6.TabIndex = 266;
            this.label6.Text = "Posizioni";
            this.label6.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // panel5
            // 
            this.panel5.BackColor = System.Drawing.Color.Black;
            this.panel5.Controls.Add(this.BtnDeletePoint);
            this.panel5.Controls.Add(this.BtnRecPoint);
            this.panel5.Controls.Add(this.BtnRenamePoint);
            this.panel5.Controls.Add(this.btn_addPoint_);
            this.panel5.Controls.Add(this.btn_home_);
            this.panel5.Controls.Add(this.lbl_buttonRecPoint);
            this.panel5.Controls.Add(this.lbl_buttonDeletePoint);
            this.panel5.Controls.Add(this.lbl_buttonNewPoint);
            this.panel5.Controls.Add(this.lbl_buttonRenamePoint);
            this.panel5.Controls.Add(this.lbl_buttonHome);
            this.panel5.Location = new System.Drawing.Point(0, 556);
            this.panel5.Name = "panel5";
            this.panel5.Size = new System.Drawing.Size(1025, 102);
            this.panel5.TabIndex = 321;
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.Gainsboro;
            this.panel1.Controls.Add(this.pictureBox1);
            this.panel1.Controls.Add(this.label6);
            this.panel1.Controls.Add(this.lw_positions);
            this.panel1.Location = new System.Drawing.Point(18, 22);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(994, 496);
            this.panel1.TabIndex = 322;
            // 
            // pnl_loading
            // 
            this.pnl_loading.BackColor = System.Drawing.Color.Gainsboro;
            this.pnl_loading.Controls.Add(this.prgs_loading);
            this.pnl_loading.Location = new System.Drawing.Point(1100, 0);
            this.pnl_loading.Name = "pnl_loading";
            this.pnl_loading.Size = new System.Drawing.Size(1025, 654);
            this.pnl_loading.TabIndex = 324;
            // 
            // prgs_loading
            // 
            this.prgs_loading.BackColor = System.Drawing.Color.Black;
            this.prgs_loading.Location = new System.Drawing.Point(410, 600);
            this.prgs_loading.MarqueeAnimationSpeed = 1;
            this.prgs_loading.Name = "prgs_loading";
            this.prgs_loading.Size = new System.Drawing.Size(200, 10);
            this.prgs_loading.Step = 50;
            this.prgs_loading.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.prgs_loading.TabIndex = 2;
            this.prgs_loading.Value = 90;
            // 
            // BtnDeletePoint
            // 
            this.BtnDeletePoint.BackgroundImage = global::RM.Properties.Resources.delete32;
            this.BtnDeletePoint.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.BtnDeletePoint.Location = new System.Drawing.Point(420, 18);
            this.BtnDeletePoint.Name = "BtnDeletePoint";
            this.BtnDeletePoint.Size = new System.Drawing.Size(55, 55);
            this.BtnDeletePoint.TabIndex = 330;
            this.BtnDeletePoint.UseVisualStyleBackColor = true;
            this.BtnDeletePoint.Click += new System.EventHandler(this.BtnDeletePoint_Click);
            // 
            // BtnRecPoint
            // 
            this.BtnRecPoint.BackgroundImage = global::RM.Properties.Resources.rec32;
            this.BtnRecPoint.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.BtnRecPoint.Location = new System.Drawing.Point(320, 18);
            this.BtnRecPoint.Name = "BtnRecPoint";
            this.BtnRecPoint.Size = new System.Drawing.Size(55, 55);
            this.BtnRecPoint.TabIndex = 329;
            this.BtnRecPoint.UseVisualStyleBackColor = true;
            this.BtnRecPoint.Click += new System.EventHandler(this.BtnRecPoint_Click);
            // 
            // BtnRenamePoint
            // 
            this.BtnRenamePoint.BackgroundImage = global::RM.Properties.Resources.edit32;
            this.BtnRenamePoint.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.BtnRenamePoint.Location = new System.Drawing.Point(220, 18);
            this.BtnRenamePoint.Name = "BtnRenamePoint";
            this.BtnRenamePoint.Size = new System.Drawing.Size(55, 55);
            this.BtnRenamePoint.TabIndex = 328;
            this.BtnRenamePoint.UseVisualStyleBackColor = true;
            this.BtnRenamePoint.Click += new System.EventHandler(this.BtnRenamePoint_Click);
            // 
            // btn_addPoint_
            // 
            this.btn_addPoint_.BackgroundImage = global::RM.Properties.Resources.addApp32;
            this.btn_addPoint_.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_addPoint_.Location = new System.Drawing.Point(120, 18);
            this.btn_addPoint_.Name = "btn_addPoint_";
            this.btn_addPoint_.Size = new System.Drawing.Size(55, 55);
            this.btn_addPoint_.TabIndex = 327;
            this.btn_addPoint_.UseVisualStyleBackColor = true;
            this.btn_addPoint_.Click += new System.EventHandler(this.BtnAddPoint_Click);
            // 
            // btn_home_
            // 
            this.btn_home_.BackgroundImage = global::RM.Properties.Resources.back32;
            this.btn_home_.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.btn_home_.Location = new System.Drawing.Point(24, 18);
            this.btn_home_.Name = "btn_home_";
            this.btn_home_.Size = new System.Drawing.Size(55, 55);
            this.btn_home_.TabIndex = 326;
            this.btn_home_.UseVisualStyleBackColor = true;
            this.btn_home_.Click += new System.EventHandler(this.BtnHome_Click);
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackColor = System.Drawing.Color.Black;
            this.pictureBox1.Image = global::RM.Properties.Resources.positionsWhite32;
            this.pictureBox1.Location = new System.Drawing.Point(2, 2);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(35, 35);
            this.pictureBox1.TabIndex = 325;
            this.pictureBox1.TabStop = false;
            // 
            // UC_positions
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.CornflowerBlue;
            this.Controls.Add(this.pnl_loading);
            this.Controls.Add(this.panel5);
            this.Controls.Add(this.panel1);
            this.DoubleBuffered = true;
            this.Name = "UC_positions";
            this.Size = new System.Drawing.Size(1024, 658);
            this.panel5.ResumeLayout(false);
            this.panel1.ResumeLayout(false);
            this.pnl_loading.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.ColumnHeader columnHeader8;
        private System.Windows.Forms.ColumnHeader columnHeader9;
        private System.Windows.Forms.ColumnHeader columnHeader7;
        private System.Windows.Forms.ColumnHeader columnHeader6;
        private System.Windows.Forms.ColumnHeader columnHeader5;
        private System.Windows.Forms.ColumnHeader columnHeader4;
        private System.Windows.Forms.ColumnHeader columnHeader3;
        private System.Windows.Forms.ColumnHeader columnHeader2;
        private System.Windows.Forms.ColumnHeader columnHeader1;
        private System.Windows.Forms.ColumnHeader columnHeader10;
        private System.Windows.Forms.Label lbl_buttonHome;
        private System.Windows.Forms.Label lbl_buttonDeletePoint;
        private System.Windows.Forms.Label lbl_buttonRecPoint;
        private System.Windows.Forms.Label lbl_buttonRenamePoint;
        private System.Windows.Forms.Label lbl_buttonNewPoint;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Panel panel5;
        private System.Windows.Forms.Panel panel1;
        private RMLib.View.ScrollableListView lw_positions;
        private System.Windows.Forms.Panel pnl_loading;
        private System.Windows.Forms.ProgressBar prgs_loading;
        private System.Windows.Forms.Button BtnDeletePoint;
        private System.Windows.Forms.Button BtnRecPoint;
        private System.Windows.Forms.Button BtnRenamePoint;
        private System.Windows.Forms.Button btn_addPoint_;
        private System.Windows.Forms.Button btn_home_;
        private System.Windows.Forms.PictureBox pictureBox1;
    }
}
