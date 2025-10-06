namespace RM.src.RM250728.Forms.ScreenSaver
{
    partial class ScreenSaverManager
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

        #region Codice generato da Progettazione Windows Form

        /// <summary>
        /// Metodo necessario per il supporto della finestra di progettazione. Non modificare
        /// il contenuto del metodo con l'editor di codice.
        /// </summary>
        private void InitializeComponent()
        {
            this.button1 = new System.Windows.Forms.Button();
            this.btn_stopTimer = new System.Windows.Forms.Button();
            this.lbl_title = new System.Windows.Forms.Label();
            this.lbl_pathTitle = new System.Windows.Forms.Label();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.button2 = new System.Windows.Forms.Button();
            this.lbl_filePath = new System.Windows.Forms.Label();
            this.pnl_path = new System.Windows.Forms.Panel();
            this.pnl_controls = new System.Windows.Forms.Panel();
            this.lbl_status = new System.Windows.Forms.Label();
            this.lbl_delayTitle = new System.Windows.Forms.Label();
            this.pnl_delay = new System.Windows.Forms.Panel();
            this.button3 = new System.Windows.Forms.Button();
            this.lbl_delay = new System.Windows.Forms.Label();
            this.pb_logo = new System.Windows.Forms.PictureBox();
            this.pnl_hide = new System.Windows.Forms.Panel();
            this.button4 = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.pnl_path.SuspendLayout();
            this.pnl_controls.SuspendLayout();
            this.pnl_delay.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_logo)).BeginInit();
            this.pnl_hide.SuspendLayout();
            this.SuspendLayout();
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(25, 85);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(80, 80);
            this.button1.TabIndex = 0;
            this.button1.Text = "Start ";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.ClickEvent_startTimer);
            // 
            // btn_stopTimer
            // 
            this.btn_stopTimer.Location = new System.Drawing.Point(242, 85);
            this.btn_stopTimer.Name = "btn_stopTimer";
            this.btn_stopTimer.Size = new System.Drawing.Size(80, 80);
            this.btn_stopTimer.TabIndex = 1;
            this.btn_stopTimer.Text = "Stop";
            this.btn_stopTimer.UseVisualStyleBackColor = true;
            this.btn_stopTimer.Click += new System.EventHandler(this.ClickEvent_stopTimer);
            // 
            // lbl_title
            // 
            this.lbl_title.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_title.Location = new System.Drawing.Point(22, 28);
            this.lbl_title.Name = "lbl_title";
            this.lbl_title.Size = new System.Drawing.Size(307, 23);
            this.lbl_title.TabIndex = 2;
            this.lbl_title.Text = "Timer";
            this.lbl_title.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_pathTitle
            // 
            this.lbl_pathTitle.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_pathTitle.Location = new System.Drawing.Point(3, 9);
            this.lbl_pathTitle.Name = "lbl_pathTitle";
            this.lbl_pathTitle.Size = new System.Drawing.Size(174, 23);
            this.lbl_pathTitle.TabIndex = 3;
            this.lbl_pathTitle.Text = "Video selezionato:";
            this.lbl_pathTitle.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // button2
            // 
            this.button2.Enabled = false;
            this.button2.Location = new System.Drawing.Point(7, 43);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(95, 23);
            this.button2.TabIndex = 4;
            this.button2.Text = "Modifica";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.ClickEvent_modifyPath);
            // 
            // lbl_filePath
            // 
            this.lbl_filePath.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_filePath.Location = new System.Drawing.Point(183, 9);
            this.lbl_filePath.Name = "lbl_filePath";
            this.lbl_filePath.Size = new System.Drawing.Size(606, 57);
            this.lbl_filePath.TabIndex = 5;
            this.lbl_filePath.Text = "file path";
            // 
            // pnl_path
            // 
            this.pnl_path.BackColor = System.Drawing.Color.White;
            this.pnl_path.Controls.Add(this.lbl_pathTitle);
            this.pnl_path.Controls.Add(this.lbl_filePath);
            this.pnl_path.Controls.Add(this.button2);
            this.pnl_path.Location = new System.Drawing.Point(0, 375);
            this.pnl_path.Name = "pnl_path";
            this.pnl_path.Size = new System.Drawing.Size(804, 75);
            this.pnl_path.TabIndex = 6;
            // 
            // pnl_controls
            // 
            this.pnl_controls.BackColor = System.Drawing.Color.White;
            this.pnl_controls.Controls.Add(this.lbl_status);
            this.pnl_controls.Controls.Add(this.lbl_title);
            this.pnl_controls.Controls.Add(this.button1);
            this.pnl_controls.Controls.Add(this.btn_stopTimer);
            this.pnl_controls.Location = new System.Drawing.Point(218, 116);
            this.pnl_controls.Name = "pnl_controls";
            this.pnl_controls.Size = new System.Drawing.Size(351, 184);
            this.pnl_controls.TabIndex = 7;
            // 
            // lbl_status
            // 
            this.lbl_status.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_status.Location = new System.Drawing.Point(140, 112);
            this.lbl_status.Name = "lbl_status";
            this.lbl_status.Size = new System.Drawing.Size(62, 23);
            this.lbl_status.TabIndex = 4;
            this.lbl_status.Text = "OFF";
            this.lbl_status.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_delayTitle
            // 
            this.lbl_delayTitle.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_delayTitle.Location = new System.Drawing.Point(3, 7);
            this.lbl_delayTitle.Name = "lbl_delayTitle";
            this.lbl_delayTitle.Size = new System.Drawing.Size(158, 23);
            this.lbl_delayTitle.TabIndex = 3;
            this.lbl_delayTitle.Text = "Tempo delay [s]";
            this.lbl_delayTitle.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pnl_delay
            // 
            this.pnl_delay.BackColor = System.Drawing.Color.White;
            this.pnl_delay.Controls.Add(this.button3);
            this.pnl_delay.Controls.Add(this.lbl_delay);
            this.pnl_delay.Controls.Add(this.lbl_delayTitle);
            this.pnl_delay.Location = new System.Drawing.Point(640, 184);
            this.pnl_delay.Name = "pnl_delay";
            this.pnl_delay.Size = new System.Drawing.Size(164, 116);
            this.pnl_delay.TabIndex = 8;
            // 
            // button3
            // 
            this.button3.Location = new System.Drawing.Point(34, 84);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(95, 23);
            this.button3.TabIndex = 5;
            this.button3.Text = "Modifica";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.ClickEvent_modifyDelay);
            // 
            // lbl_delay
            // 
            this.lbl_delay.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_delay.Location = new System.Drawing.Point(3, 43);
            this.lbl_delay.Name = "lbl_delay";
            this.lbl_delay.Size = new System.Drawing.Size(158, 23);
            this.lbl_delay.TabIndex = 4;
            this.lbl_delay.Text = "0";
            this.lbl_delay.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pb_logo
            // 
            this.pb_logo.BackColor = System.Drawing.Color.White;
            this.pb_logo.Image = global::RM.Properties.Resources.RM_logo;
            this.pb_logo.Location = new System.Drawing.Point(0, 0);
            this.pb_logo.Name = "pb_logo";
            this.pb_logo.Size = new System.Drawing.Size(198, 59);
            this.pb_logo.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pb_logo.TabIndex = 9;
            this.pb_logo.TabStop = false;
            this.pb_logo.DoubleClick += new System.EventHandler(this.ClickEvent_openVersions);
            // 
            // pnl_hide
            // 
            this.pnl_hide.BackColor = System.Drawing.Color.White;
            this.pnl_hide.Controls.Add(this.button4);
            this.pnl_hide.Controls.Add(this.label2);
            this.pnl_hide.Location = new System.Drawing.Point(640, 73);
            this.pnl_hide.Name = "pnl_hide";
            this.pnl_hide.Size = new System.Drawing.Size(164, 110);
            this.pnl_hide.TabIndex = 10;
            // 
            // button4
            // 
            this.button4.Location = new System.Drawing.Point(34, 57);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(95, 35);
            this.button4.TabIndex = 5;
            this.button4.Text = "-->";
            this.button4.UseVisualStyleBackColor = true;
            this.button4.Click += new System.EventHandler(this.ClickEvent_hideControls);
            // 
            // label2
            // 
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(3, 25);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(158, 23);
            this.label2.TabIndex = 3;
            this.label2.Text = "Nascondi";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label1
            // 
            this.label1.BackColor = System.Drawing.Color.White;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(244, 29);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(296, 45);
            this.label1.TabIndex = 11;
            this.label1.Text = "Screen saver manager";
            this.label1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // ScreenSaverManager
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.CornflowerBlue;
            this.ClientSize = new System.Drawing.Size(800, 450);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.pnl_hide);
            this.Controls.Add(this.pb_logo);
            this.Controls.Add(this.pnl_delay);
            this.Controls.Add(this.pnl_controls);
            this.Controls.Add(this.pnl_path);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;
            this.Name = "ScreenSaverManager";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.TopMost = true;
            this.pnl_path.ResumeLayout(false);
            this.pnl_controls.ResumeLayout(false);
            this.pnl_delay.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_logo)).EndInit();
            this.pnl_hide.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button btn_stopTimer;
        private System.Windows.Forms.Label lbl_title;
        private System.Windows.Forms.Label lbl_pathTitle;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Label lbl_filePath;
        private System.Windows.Forms.Panel pnl_path;
        private System.Windows.Forms.Panel pnl_controls;
        private System.Windows.Forms.Label lbl_delayTitle;
        private System.Windows.Forms.Panel pnl_delay;
        private System.Windows.Forms.Label lbl_delay;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.PictureBox pb_logo;
        private System.Windows.Forms.Label lbl_status;
        private System.Windows.Forms.Panel pnl_hide;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
    }
}

