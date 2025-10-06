namespace RM.src.RM250728
{
    partial class FormInfoRobot
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.LblDescription = new System.Windows.Forms.Label();
            this.PbInfo = new System.Windows.Forms.PictureBox();
            this.LblTitle = new System.Windows.Forms.Label();
            this.PnlClose = new System.Windows.Forms.Panel();
            this.pb_separator1 = new System.Windows.Forms.PictureBox();
            ((System.ComponentModel.ISupportInitialize)(this.PbInfo)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator1)).BeginInit();
            this.SuspendLayout();
            // 
            // LblDescription
            // 
            this.LblDescription.BackColor = System.Drawing.Color.Transparent;
            this.LblDescription.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F);
            this.LblDescription.ForeColor = System.Drawing.SystemColors.Control;
            this.LblDescription.Location = new System.Drawing.Point(5, 47);
            this.LblDescription.Name = "LblDescription";
            this.LblDescription.Size = new System.Drawing.Size(301, 138);
            this.LblDescription.TabIndex = 4;
            this.LblDescription.Text = "Description";
            this.LblDescription.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // PbInfo
            // 
            this.PbInfo.BackgroundImage = global::RM.Properties.Resources.info_white;
            this.PbInfo.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.PbInfo.Location = new System.Drawing.Point(5, 5);
            this.PbInfo.Name = "PbInfo";
            this.PbInfo.Size = new System.Drawing.Size(32, 32);
            this.PbInfo.TabIndex = 254;
            this.PbInfo.TabStop = false;
            // 
            // LblTitle
            // 
            this.LblTitle.BackColor = System.Drawing.Color.Transparent;
            this.LblTitle.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.LblTitle.ForeColor = System.Drawing.SystemColors.Control;
            this.LblTitle.Location = new System.Drawing.Point(0, 5);
            this.LblTitle.Name = "LblTitle";
            this.LblTitle.Size = new System.Drawing.Size(306, 42);
            this.LblTitle.TabIndex = 3;
            this.LblTitle.Text = "Title";
            this.LblTitle.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // PnlClose
            // 
            this.PnlClose.BackColor = System.Drawing.Color.Red;
            this.PnlClose.BackgroundImage = global::RM.Properties.Resources.close_filled;
            this.PnlClose.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.PnlClose.Location = new System.Drawing.Point(287, 0);
            this.PnlClose.Name = "PnlClose";
            this.PnlClose.Size = new System.Drawing.Size(25, 25);
            this.PnlClose.TabIndex = 111;
            this.PnlClose.Click += new System.EventHandler(this.PnlClose_Click);
            // 
            // pb_separator1
            // 
            this.pb_separator1.BackColor = System.Drawing.Color.Transparent;
            this.pb_separator1.BackgroundImage = global::RM.Properties.Resources.line;
            this.pb_separator1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pb_separator1.Location = new System.Drawing.Point(-23, 35);
            this.pb_separator1.Name = "pb_separator1";
            this.pb_separator1.Size = new System.Drawing.Size(359, 18);
            this.pb_separator1.TabIndex = 255;
            this.pb_separator1.TabStop = false;
            // 
            // FormInfoRobot
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Black;
            this.ClientSize = new System.Drawing.Size(311, 194);
            this.Controls.Add(this.pb_separator1);
            this.Controls.Add(this.LblDescription);
            this.Controls.Add(this.PnlClose);
            this.Controls.Add(this.PbInfo);
            this.Controls.Add(this.LblTitle);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;
            this.Name = "FormInfoRobot";
            this.Opacity = 0.95D;
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "FormInfoRobotConsents";
            this.TopMost = true;
            ((System.ComponentModel.ISupportInitialize)(this.PbInfo)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_separator1)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Panel PnlClose;
        private System.Windows.Forms.Label LblTitle;
        private System.Windows.Forms.Label LblDescription;
        private System.Windows.Forms.PictureBox PbInfo;
        private System.Windows.Forms.PictureBox pb_separator1;
    }
}