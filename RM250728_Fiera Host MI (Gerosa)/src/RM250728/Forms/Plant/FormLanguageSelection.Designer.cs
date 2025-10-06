namespace RM.src.RM250728.Forms.Plant
{
    partial class FormLanguageSelection
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
            this.pnl_languagesSelection = new RMLib.View.CustomPanel();
            this.pnl_confirmMsgBox = new RMLib.View.CustomPanel();
            this.btn_confirm = new RMLib.View.CustomButton();
            this.btn_cancel = new RMLib.View.CustomButton();
            this.pnl_lblContainer = new RMLib.View.CustomPanel();
            this.lbl_msg = new System.Windows.Forms.Label();
            this.pb_icon = new System.Windows.Forms.PictureBox();
            this.btn_eng = new RMLib.View.CustomButton();
            this.btn_ita = new RMLib.View.CustomButton();
            this.btn_fra = new RMLib.View.CustomButton();
            this.btn_deu = new RMLib.View.CustomButton();
            this.btn_chi = new RMLib.View.CustomButton();
            this.btn_egy = new RMLib.View.CustomButton();
            this.pb_languageIcon = new System.Windows.Forms.PictureBox();
            this.pnl_languagesSelection.SuspendLayout();
            this.pnl_confirmMsgBox.SuspendLayout();
            this.pnl_lblContainer.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pb_icon)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_languageIcon)).BeginInit();
            this.SuspendLayout();
            // 
            // pnl_languagesSelection
            // 
            this.pnl_languagesSelection.BackColor = System.Drawing.Color.DarkBlue;
            this.pnl_languagesSelection.BackgroundColor = System.Drawing.Color.DarkBlue;
            this.pnl_languagesSelection.BorderColor = System.Drawing.Color.RoyalBlue;
            this.pnl_languagesSelection.BorderRadius = 25;
            this.pnl_languagesSelection.BorderSize = 0;
            this.pnl_languagesSelection.Controls.Add(this.btn_eng);
            this.pnl_languagesSelection.Controls.Add(this.btn_ita);
            this.pnl_languagesSelection.Controls.Add(this.btn_fra);
            this.pnl_languagesSelection.Controls.Add(this.btn_deu);
            this.pnl_languagesSelection.Controls.Add(this.btn_chi);
            this.pnl_languagesSelection.Controls.Add(this.btn_egy);
            this.pnl_languagesSelection.ForeColor = System.Drawing.Color.White;
            this.pnl_languagesSelection.Location = new System.Drawing.Point(750, 421);
            this.pnl_languagesSelection.Name = "pnl_languagesSelection";
            this.pnl_languagesSelection.Size = new System.Drawing.Size(197, 236);
            this.pnl_languagesSelection.TabIndex = 253;
            this.pnl_languagesSelection.TextColor = System.Drawing.Color.White;
            // 
            // pnl_confirmMsgBox
            // 
            this.pnl_confirmMsgBox.BackColor = System.Drawing.Color.DarkBlue;
            this.pnl_confirmMsgBox.BackgroundColor = System.Drawing.Color.DarkBlue;
            this.pnl_confirmMsgBox.BorderColor = System.Drawing.Color.DarkBlue;
            this.pnl_confirmMsgBox.BorderRadius = 20;
            this.pnl_confirmMsgBox.BorderSize = 5;
            this.pnl_confirmMsgBox.Controls.Add(this.pb_languageIcon);
            this.pnl_confirmMsgBox.Controls.Add(this.pb_icon);
            this.pnl_confirmMsgBox.Controls.Add(this.pnl_lblContainer);
            this.pnl_confirmMsgBox.Controls.Add(this.btn_cancel);
            this.pnl_confirmMsgBox.Controls.Add(this.btn_confirm);
            this.pnl_confirmMsgBox.ForeColor = System.Drawing.Color.White;
            this.pnl_confirmMsgBox.Location = new System.Drawing.Point(265, 171);
            this.pnl_confirmMsgBox.Name = "pnl_confirmMsgBox";
            this.pnl_confirmMsgBox.Size = new System.Drawing.Size(453, 278);
            this.pnl_confirmMsgBox.TabIndex = 254;
            this.pnl_confirmMsgBox.TextColor = System.Drawing.Color.White;
            this.pnl_confirmMsgBox.Visible = false;
            // 
            // btn_confirm
            // 
            this.btn_confirm.BackColor = System.Drawing.SystemColors.ControlLight;
            this.btn_confirm.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.btn_confirm.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_confirm.BorderColor = System.Drawing.Color.Black;
            this.btn_confirm.BorderRadius = 15;
            this.btn_confirm.BorderSize = 2;
            this.btn_confirm.FlatAppearance.BorderSize = 0;
            this.btn_confirm.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_confirm.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_confirm.ForeColor = System.Drawing.Color.Black;
            this.btn_confirm.Location = new System.Drawing.Point(93, 209);
            this.btn_confirm.Name = "btn_confirm";
            this.btn_confirm.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_confirm.Size = new System.Drawing.Size(88, 52);
            this.btn_confirm.TabIndex = 253;
            this.btn_confirm.Tag = "";
            this.btn_confirm.Text = "Ok";
            this.btn_confirm.TextColor = System.Drawing.Color.Black;
            this.btn_confirm.UseVisualStyleBackColor = false;
            this.btn_confirm.Click += new System.EventHandler(this.ClickEvent_confirmLanguageChange);
            // 
            // btn_cancel
            // 
            this.btn_cancel.BackColor = System.Drawing.SystemColors.ControlLight;
            this.btn_cancel.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.btn_cancel.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_cancel.BorderColor = System.Drawing.Color.Black;
            this.btn_cancel.BorderRadius = 15;
            this.btn_cancel.BorderSize = 2;
            this.btn_cancel.FlatAppearance.BorderSize = 0;
            this.btn_cancel.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_cancel.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_cancel.ForeColor = System.Drawing.Color.Black;
            this.btn_cancel.Location = new System.Drawing.Point(268, 209);
            this.btn_cancel.Name = "btn_cancel";
            this.btn_cancel.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_cancel.Size = new System.Drawing.Size(88, 52);
            this.btn_cancel.TabIndex = 254;
            this.btn_cancel.Tag = "";
            this.btn_cancel.Text = "Cancel";
            this.btn_cancel.TextColor = System.Drawing.Color.Black;
            this.btn_cancel.UseVisualStyleBackColor = false;
            this.btn_cancel.Click += new System.EventHandler(this.ClickEvent_denyLanguageChange);
            // 
            // pnl_lblContainer
            // 
            this.pnl_lblContainer.BackColor = System.Drawing.SystemColors.ControlLight;
            this.pnl_lblContainer.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.pnl_lblContainer.BorderColor = System.Drawing.Color.Gray;
            this.pnl_lblContainer.BorderRadius = 20;
            this.pnl_lblContainer.BorderSize = 2;
            this.pnl_lblContainer.Controls.Add(this.lbl_msg);
            this.pnl_lblContainer.ForeColor = System.Drawing.Color.White;
            this.pnl_lblContainer.Location = new System.Drawing.Point(93, 37);
            this.pnl_lblContainer.Name = "pnl_lblContainer";
            this.pnl_lblContainer.Size = new System.Drawing.Size(263, 153);
            this.pnl_lblContainer.TabIndex = 255;
            this.pnl_lblContainer.TextColor = System.Drawing.Color.White;
            // 
            // lbl_msg
            // 
            this.lbl_msg.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_msg.ForeColor = System.Drawing.Color.Black;
            this.lbl_msg.Location = new System.Drawing.Point(16, 13);
            this.lbl_msg.Name = "lbl_msg";
            this.lbl_msg.Size = new System.Drawing.Size(232, 125);
            this.lbl_msg.TabIndex = 255;
            this.lbl_msg.Text = "msg";
            this.lbl_msg.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pb_icon
            // 
            this.pb_icon.Image = global::RM.Properties.Resources.warning_message_icon;
            this.pb_icon.Location = new System.Drawing.Point(26, 17);
            this.pb_icon.Name = "pb_icon";
            this.pb_icon.Size = new System.Drawing.Size(45, 45);
            this.pb_icon.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pb_icon.TabIndex = 255;
            this.pb_icon.TabStop = false;
            // 
            // btn_eng
            // 
            this.btn_eng.BackColor = System.Drawing.SystemColors.ControlLight;
            this.btn_eng.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.btn_eng.BackgroundImage = global::RM.Properties.Resources.england_flag;
            this.btn_eng.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_eng.BorderColor = System.Drawing.Color.Black;
            this.btn_eng.BorderRadius = 15;
            this.btn_eng.BorderSize = 2;
            this.btn_eng.FlatAppearance.BorderSize = 0;
            this.btn_eng.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_eng.ForeColor = System.Drawing.Color.White;
            this.btn_eng.Location = new System.Drawing.Point(103, 12);
            this.btn_eng.Name = "btn_eng";
            this.btn_eng.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_eng.Size = new System.Drawing.Size(88, 65);
            this.btn_eng.TabIndex = 253;
            this.btn_eng.Tag = "eng";
            this.btn_eng.TextColor = System.Drawing.Color.White;
            this.btn_eng.UseVisualStyleBackColor = false;
            this.btn_eng.Click += new System.EventHandler(this.ClickEvent_languageSelection);
            // 
            // btn_ita
            // 
            this.btn_ita.BackColor = System.Drawing.SystemColors.ControlLight;
            this.btn_ita.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.btn_ita.BackgroundImage = global::RM.Properties.Resources.italy_flag;
            this.btn_ita.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_ita.BorderColor = System.Drawing.Color.Black;
            this.btn_ita.BorderRadius = 15;
            this.btn_ita.BorderSize = 2;
            this.btn_ita.FlatAppearance.BorderSize = 0;
            this.btn_ita.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_ita.ForeColor = System.Drawing.Color.Transparent;
            this.btn_ita.Location = new System.Drawing.Point(9, 12);
            this.btn_ita.Name = "btn_ita";
            this.btn_ita.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_ita.Size = new System.Drawing.Size(88, 65);
            this.btn_ita.TabIndex = 252;
            this.btn_ita.Tag = "ita";
            this.btn_ita.TextColor = System.Drawing.Color.Transparent;
            this.btn_ita.UseVisualStyleBackColor = false;
            this.btn_ita.Click += new System.EventHandler(this.ClickEvent_languageSelection);
            // 
            // btn_fra
            // 
            this.btn_fra.BackColor = System.Drawing.SystemColors.ControlLight;
            this.btn_fra.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.btn_fra.BackgroundImage = global::RM.Properties.Resources.france_flag;
            this.btn_fra.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_fra.BorderColor = System.Drawing.Color.Black;
            this.btn_fra.BorderRadius = 15;
            this.btn_fra.BorderSize = 2;
            this.btn_fra.FlatAppearance.BorderSize = 0;
            this.btn_fra.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_fra.ForeColor = System.Drawing.Color.White;
            this.btn_fra.Location = new System.Drawing.Point(101, 84);
            this.btn_fra.Name = "btn_fra";
            this.btn_fra.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_fra.Size = new System.Drawing.Size(88, 65);
            this.btn_fra.TabIndex = 251;
            this.btn_fra.Tag = "fra";
            this.btn_fra.TextColor = System.Drawing.Color.White;
            this.btn_fra.UseVisualStyleBackColor = false;
            this.btn_fra.Click += new System.EventHandler(this.ClickEvent_languageSelection);
            // 
            // btn_deu
            // 
            this.btn_deu.BackColor = System.Drawing.SystemColors.ControlLight;
            this.btn_deu.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.btn_deu.BackgroundImage = global::RM.Properties.Resources.germany_flag;
            this.btn_deu.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_deu.BorderColor = System.Drawing.Color.Black;
            this.btn_deu.BorderRadius = 15;
            this.btn_deu.BorderSize = 2;
            this.btn_deu.FlatAppearance.BorderSize = 0;
            this.btn_deu.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_deu.ForeColor = System.Drawing.Color.White;
            this.btn_deu.Location = new System.Drawing.Point(7, 84);
            this.btn_deu.Name = "btn_deu";
            this.btn_deu.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_deu.Size = new System.Drawing.Size(88, 65);
            this.btn_deu.TabIndex = 250;
            this.btn_deu.Tag = "deu";
            this.btn_deu.TextColor = System.Drawing.Color.White;
            this.btn_deu.UseVisualStyleBackColor = false;
            this.btn_deu.Click += new System.EventHandler(this.ClickEvent_languageSelection);
            // 
            // btn_chi
            // 
            this.btn_chi.BackColor = System.Drawing.SystemColors.ControlLight;
            this.btn_chi.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.btn_chi.BackgroundImage = global::RM.Properties.Resources.china_flag;
            this.btn_chi.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_chi.BorderColor = System.Drawing.Color.Black;
            this.btn_chi.BorderRadius = 15;
            this.btn_chi.BorderSize = 2;
            this.btn_chi.FlatAppearance.BorderSize = 0;
            this.btn_chi.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_chi.ForeColor = System.Drawing.Color.White;
            this.btn_chi.Location = new System.Drawing.Point(100, 157);
            this.btn_chi.Name = "btn_chi";
            this.btn_chi.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_chi.Size = new System.Drawing.Size(88, 65);
            this.btn_chi.TabIndex = 249;
            this.btn_chi.Tag = "chi";
            this.btn_chi.TextColor = System.Drawing.Color.White;
            this.btn_chi.UseVisualStyleBackColor = false;
            this.btn_chi.Click += new System.EventHandler(this.ClickEvent_languageSelection);
            // 
            // btn_egy
            // 
            this.btn_egy.BackColor = System.Drawing.SystemColors.ControlLight;
            this.btn_egy.BackgroundColor = System.Drawing.SystemColors.ControlLight;
            this.btn_egy.BackgroundImage = global::RM.Properties.Resources.egypt_flag;
            this.btn_egy.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_egy.BorderColor = System.Drawing.Color.Black;
            this.btn_egy.BorderRadius = 15;
            this.btn_egy.BorderSize = 2;
            this.btn_egy.FlatAppearance.BorderSize = 0;
            this.btn_egy.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btn_egy.ForeColor = System.Drawing.Color.White;
            this.btn_egy.Location = new System.Drawing.Point(6, 157);
            this.btn_egy.Name = "btn_egy";
            this.btn_egy.Padding = new System.Windows.Forms.Padding(0, 0, 0, 3);
            this.btn_egy.Size = new System.Drawing.Size(88, 65);
            this.btn_egy.TabIndex = 248;
            this.btn_egy.Tag = "egy";
            this.btn_egy.TextColor = System.Drawing.Color.White;
            this.btn_egy.UseVisualStyleBackColor = false;
            this.btn_egy.Click += new System.EventHandler(this.ClickEvent_languageSelection);
            // 
            // pb_languageIcon
            // 
            this.pb_languageIcon.Image = global::RM.Properties.Resources.italy_flag;
            this.pb_languageIcon.Location = new System.Drawing.Point(382, 17);
            this.pb_languageIcon.Name = "pb_languageIcon";
            this.pb_languageIcon.Size = new System.Drawing.Size(45, 45);
            this.pb_languageIcon.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pb_languageIcon.TabIndex = 256;
            this.pb_languageIcon.TabStop = false;
            // 
            // FormLanguageSelection
            // 
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.None;
            this.BackColor = System.Drawing.SystemColors.ControlLight;
            this.ClientSize = new System.Drawing.Size(1026, 668);
            this.Controls.Add(this.pnl_confirmMsgBox);
            this.Controls.Add(this.pnl_languagesSelection);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;
            this.Name = "FormLanguageSelection";
            this.Opacity = 0.95D;
            this.SizeGripStyle = System.Windows.Forms.SizeGripStyle.Hide;
            this.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
            this.Deactivate += new System.EventHandler(this.DeactivationEvent_deactivatedForm);
            this.Click += new System.EventHandler(this.ClickEvent_clickOnForm);
            this.Leave += new System.EventHandler(this.LeaveEvent_leaveForm);
            this.pnl_languagesSelection.ResumeLayout(false);
            this.pnl_confirmMsgBox.ResumeLayout(false);
            this.pnl_lblContainer.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pb_icon)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pb_languageIcon)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private RMLib.View.CustomPanel pnl_languagesSelection;
        private RMLib.View.CustomButton btn_eng;
        private RMLib.View.CustomButton btn_ita;
        private RMLib.View.CustomButton btn_fra;
        private RMLib.View.CustomButton btn_deu;
        private RMLib.View.CustomButton btn_chi;
        private RMLib.View.CustomButton btn_egy;
        private RMLib.View.CustomPanel pnl_confirmMsgBox;
        private RMLib.View.CustomButton btn_confirm;
        private RMLib.View.CustomPanel pnl_lblContainer;
        private System.Windows.Forms.Label lbl_msg;
        private RMLib.View.CustomButton btn_cancel;
        private System.Windows.Forms.PictureBox pb_icon;
        private System.Windows.Forms.PictureBox pb_languageIcon;
    }
}