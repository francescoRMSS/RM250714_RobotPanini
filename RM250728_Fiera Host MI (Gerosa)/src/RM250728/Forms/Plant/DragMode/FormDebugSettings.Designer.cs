namespace RM.src.RM250728.Forms.Plant.DragMode
{
    partial class FormDebugSettings
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(FormDebugSettings));
            this.panel1 = new System.Windows.Forms.Panel();
            this.pnl_infoTool = new System.Windows.Forms.Panel();
            this.pictureBox3 = new System.Windows.Forms.PictureBox();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.rbPointToPoint = new RMLib.View.CustomRadioButton();
            this.rbLinear = new RMLib.View.CustomRadioButton();
            this.pb_iconTools = new System.Windows.Forms.PictureBox();
            this.lbl_header = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_iconTools)).BeginInit();
            this.SuspendLayout();
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.Transparent;
            this.panel1.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("panel1.BackgroundImage")));
            this.panel1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.panel1.Location = new System.Drawing.Point(340, 309);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(30, 30);
            this.panel1.TabIndex = 289;
            this.panel1.Click += new System.EventHandler(this.PnlInfoLinear_Click);
            // 
            // pnl_infoTool
            // 
            this.pnl_infoTool.BackColor = System.Drawing.Color.Transparent;
            this.pnl_infoTool.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("pnl_infoTool.BackgroundImage")));
            this.pnl_infoTool.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_infoTool.Location = new System.Drawing.Point(340, 117);
            this.pnl_infoTool.Name = "pnl_infoTool";
            this.pnl_infoTool.Size = new System.Drawing.Size(30, 30);
            this.pnl_infoTool.TabIndex = 288;
            this.pnl_infoTool.Click += new System.EventHandler(this.PnlInfoPTP_Click);
            // 
            // pictureBox3
            // 
            this.pictureBox3.Image = global::RM.Properties.Resources.linear_line;
            this.pictureBox3.Location = new System.Drawing.Point(122, 271);
            this.pictureBox3.Name = "pictureBox3";
            this.pictureBox3.Size = new System.Drawing.Size(140, 100);
            this.pictureBox3.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBox3.TabIndex = 287;
            this.pictureBox3.TabStop = false;
            this.pictureBox3.Click += new System.EventHandler(this.ClickEvent_pb_linear);
            // 
            // pictureBox1
            // 
            this.pictureBox1.Image = global::RM.Properties.Resources.ptp_line;
            this.pictureBox1.Location = new System.Drawing.Point(122, 89);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(140, 100);
            this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBox1.TabIndex = 286;
            this.pictureBox1.TabStop = false;
            this.pictureBox1.Click += new System.EventHandler(this.ClickEvent_pb_PTP);
            // 
            // pictureBox2
            // 
            this.pictureBox2.BackColor = System.Drawing.Color.Transparent;
            this.pictureBox2.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("pictureBox2.BackgroundImage")));
            this.pictureBox2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pictureBox2.Location = new System.Drawing.Point(5, 207);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(393, 18);
            this.pictureBox2.TabIndex = 285;
            this.pictureBox2.TabStop = false;
            // 
            // rbPointToPoint
            // 
            this.rbPointToPoint.Checked = true;
            this.rbPointToPoint.CheckedColor = System.Drawing.Color.White;
            this.rbPointToPoint.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F);
            this.rbPointToPoint.ForeColor = System.Drawing.SystemColors.Control;
            this.rbPointToPoint.Location = new System.Drawing.Point(12, 49);
            this.rbPointToPoint.Name = "rbPointToPoint";
            this.rbPointToPoint.Size = new System.Drawing.Size(157, 34);
            this.rbPointToPoint.TabIndex = 283;
            this.rbPointToPoint.Text = "Point to point";
            this.rbPointToPoint.UncheckedColor = System.Drawing.Color.White;
            this.rbPointToPoint.CheckedChanged += new System.EventHandler(this.CheckedChangeEvent_radioButtonPTP);
            // 
            // rbLinear
            // 
            this.rbLinear.Checked = false;
            this.rbLinear.CheckedColor = System.Drawing.Color.White;
            this.rbLinear.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F);
            this.rbLinear.ForeColor = System.Drawing.SystemColors.Control;
            this.rbLinear.Location = new System.Drawing.Point(12, 231);
            this.rbLinear.Name = "rbLinear";
            this.rbLinear.Size = new System.Drawing.Size(157, 34);
            this.rbLinear.TabIndex = 284;
            this.rbLinear.Text = "Linear";
            this.rbLinear.UncheckedColor = System.Drawing.Color.White;
            this.rbLinear.CheckedChanged += new System.EventHandler(this.CheckedChangeEvent_radioButtonLinear);
            // 
            // pb_iconTools
            // 
            this.pb_iconTools.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("pb_iconTools.BackgroundImage")));
            this.pb_iconTools.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pb_iconTools.Location = new System.Drawing.Point(5, 2);
            this.pb_iconTools.Name = "pb_iconTools";
            this.pb_iconTools.Size = new System.Drawing.Size(32, 32);
            this.pb_iconTools.TabIndex = 282;
            this.pb_iconTools.TabStop = false;
            this.pb_iconTools.MouseDown += new System.Windows.Forms.MouseEventHandler(this.MouseDownEvent_startMove);
            this.pb_iconTools.MouseMove += new System.Windows.Forms.MouseEventHandler(this.MouseMoveEvent_moving);
            this.pb_iconTools.MouseUp += new System.Windows.Forms.MouseEventHandler(this.MouseUpEvent_stopMove);
            // 
            // lbl_header
            // 
            this.lbl_header.BackColor = System.Drawing.Color.Transparent;
            this.lbl_header.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold);
            this.lbl_header.ForeColor = System.Drawing.Color.White;
            this.lbl_header.Location = new System.Drawing.Point(1, 2);
            this.lbl_header.Name = "lbl_header";
            this.lbl_header.Size = new System.Drawing.Size(397, 24);
            this.lbl_header.TabIndex = 281;
            this.lbl_header.Text = "Impostazioni modalità";
            this.lbl_header.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.lbl_header.MouseDown += new System.Windows.Forms.MouseEventHandler(this.MouseDownEvent_startMove);
            this.lbl_header.MouseMove += new System.Windows.Forms.MouseEventHandler(this.MouseMoveEvent_moving);
            this.lbl_header.MouseUp += new System.Windows.Forms.MouseEventHandler(this.MouseUpEvent_stopMove);
            // 
            // FormDebugSettings
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Black;
            this.ClientSize = new System.Drawing.Size(397, 417);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.pnl_infoTool);
            this.Controls.Add(this.pictureBox3);
            this.Controls.Add(this.pictureBox1);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.rbPointToPoint);
            this.Controls.Add(this.rbLinear);
            this.Controls.Add(this.pb_iconTools);
            this.Controls.Add(this.lbl_header);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;
            this.Name = "FormDebugSettings";
            this.Opacity = 0.9D;
            this.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
            this.TopMost = true;
            this.MouseDown += new System.Windows.Forms.MouseEventHandler(this.MouseDownEvent_startMove);
            this.MouseMove += new System.Windows.Forms.MouseEventHandler(this.MouseMoveEvent_moving);
            this.MouseUp += new System.Windows.Forms.MouseEventHandler(this.MouseUpEvent_stopMove);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_iconTools)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Panel pnl_infoTool;
        private System.Windows.Forms.PictureBox pictureBox3;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.PictureBox pictureBox2;
        private RMLib.View.CustomRadioButton rbPointToPoint;
        private RMLib.View.CustomRadioButton rbLinear;
        private System.Windows.Forms.PictureBox pb_iconTools;
        private System.Windows.Forms.Label lbl_header;
    }
}