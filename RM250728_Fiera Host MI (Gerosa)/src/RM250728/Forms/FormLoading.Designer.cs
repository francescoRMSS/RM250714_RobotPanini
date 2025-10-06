namespace RM.src.RM250728
{
    partial class FormLoading
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(FormLoading));
            this.lb_ProgressBar = new System.Windows.Forms.Label();
            this.PanelProgressBar = new System.Windows.Forms.Panel();
            this.progressBar1 = new RMLib.View.CustomProgressBar();
            this.picBox_Loading = new System.Windows.Forms.PictureBox();
            this.PanelProgressBar.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.picBox_Loading)).BeginInit();
            this.SuspendLayout();
            // 
            // lb_ProgressBar
            // 
            this.lb_ProgressBar.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right)));
            this.lb_ProgressBar.Font = new System.Drawing.Font("Nirmala UI", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_ProgressBar.Location = new System.Drawing.Point(142, 13);
            this.lb_ProgressBar.Name = "lb_ProgressBar";
            this.lb_ProgressBar.Size = new System.Drawing.Size(393, 22);
            this.lb_ProgressBar.TabIndex = 2;
            this.lb_ProgressBar.Text = "lb_ProgressBar";
            this.lb_ProgressBar.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.lb_ProgressBar.Visible = false;
            // 
            // PanelProgressBar
            // 
            this.PanelProgressBar.BackColor = System.Drawing.Color.White;
            this.PanelProgressBar.Controls.Add(this.progressBar1);
            this.PanelProgressBar.Controls.Add(this.lb_ProgressBar);
            this.PanelProgressBar.Location = new System.Drawing.Point(0, 377);
            this.PanelProgressBar.Name = "PanelProgressBar";
            this.PanelProgressBar.Size = new System.Drawing.Size(684, 35);
            this.PanelProgressBar.TabIndex = 3;
            // 
            // progressBar1
            // 
            this.progressBar1.ChannelColor = System.Drawing.Color.LightSteelBlue;
            this.progressBar1.ChannelHeight = 10;
            this.progressBar1.ForeBackColor = System.Drawing.Color.RoyalBlue;
            this.progressBar1.ForeColor = System.Drawing.Color.White;
            this.progressBar1.Location = new System.Drawing.Point(3, -23);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.ShowValue = RMLib.View.TextPosition.None;
            this.progressBar1.Size = new System.Drawing.Size(678, 34);
            this.progressBar1.SliderColor = System.Drawing.Color.RoyalBlue;
            this.progressBar1.SliderHeight = 10;
            this.progressBar1.TabIndex = 5;
            // 
            // picBox_Loading
            // 
            this.picBox_Loading.BackColor = System.Drawing.Color.White;
            this.picBox_Loading.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.picBox_Loading.Image = ((System.Drawing.Image)(resources.GetObject("picBox_Loading.Image")));
            this.picBox_Loading.Location = new System.Drawing.Point(0, -43);
            this.picBox_Loading.Name = "picBox_Loading";
            this.picBox_Loading.Size = new System.Drawing.Size(684, 455);
            this.picBox_Loading.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.picBox_Loading.TabIndex = 4;
            this.picBox_Loading.TabStop = false;
            // 
            // FormLoading
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(684, 411);
            this.Controls.Add(this.PanelProgressBar);
            this.Controls.Add(this.picBox_Loading);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "FormLoading";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.ClosingEvent_loadingPageClosing);
            this.Load += new System.EventHandler(this.FormLoading_Load);
            this.PanelProgressBar.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.picBox_Loading)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Label lb_ProgressBar;
        private System.Windows.Forms.Panel PanelProgressBar;
        private System.Windows.Forms.PictureBox picBox_Loading;
        private RMLib.View.CustomProgressBar progressBar1;
    }
}

