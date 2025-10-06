namespace RM.src.RM250728
{
    partial class FormHomePage
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
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(FormHomePage));
            this.pnl_pageContainer = new System.Windows.Forms.Panel();
            this.timer_dateTime_clock = new System.Windows.Forms.Timer(this.components);
            this.pnl_header = new RMLib.View.CustomPanel();
            this.pnl_safetyTask = new System.Windows.Forms.Panel();
            this.pnl_safetyTaskStatus = new RMLib.View.CustomPanel();
            this.label7 = new System.Windows.Forms.Label();
            this.pnl_plcTask = new System.Windows.Forms.Panel();
            this.pnl_plcTaskStatus = new RMLib.View.CustomPanel();
            this.label3 = new System.Windows.Forms.Label();
            this.pnl_appTask = new System.Windows.Forms.Panel();
            this.pnl_appTaskStatus = new RMLib.View.CustomPanel();
            this.label6 = new System.Windows.Forms.Label();
            this.pnl_safeZone = new System.Windows.Forms.Panel();
            this.pnl_comRobotTask = new System.Windows.Forms.Panel();
            this.pnl_comRobotTaskStatus = new RMLib.View.CustomPanel();
            this.label5 = new System.Windows.Forms.Label();
            this.Pnl_ROBOT_alarm = new System.Windows.Forms.Panel();
            this.pnl_auxTask = new System.Windows.Forms.Panel();
            this.pnl_auxTaskStatus = new RMLib.View.CustomPanel();
            this.label2 = new System.Windows.Forms.Label();
            this.Pnl_PLC_alarm = new System.Windows.Forms.Panel();
            this.pnl_lowTask = new System.Windows.Forms.Panel();
            this.pnl_lowTaskStatus = new RMLib.View.CustomPanel();
            this.label1 = new System.Windows.Forms.Label();
            this.Pnl_diagnostics = new System.Windows.Forms.Panel();
            this.pnl_highTask = new System.Windows.Forms.Panel();
            this.pnl_highTaskStatus = new RMLib.View.CustomPanel();
            this.label4 = new System.Windows.Forms.Label();
            this.pnl_ActiveAlarms = new System.Windows.Forms.Panel();
            this.PnlLogoContainer = new RMLib.View.CustomPanel();
            this.Pnl_logo_rm = new RMLib.View.CustomPanel();
            this.Pnl_MOVROBOT_alarm = new System.Windows.Forms.Panel();
            this.lbl_pageTitle = new System.Windows.Forms.Label();
            this.lbl_dateTime = new System.Windows.Forms.Label();
            this.pnl_header.SuspendLayout();
            this.pnl_safetyTask.SuspendLayout();
            this.pnl_plcTask.SuspendLayout();
            this.pnl_appTask.SuspendLayout();
            this.pnl_comRobotTask.SuspendLayout();
            this.pnl_auxTask.SuspendLayout();
            this.pnl_lowTask.SuspendLayout();
            this.pnl_highTask.SuspendLayout();
            this.PnlLogoContainer.SuspendLayout();
            this.SuspendLayout();
            // 
            // pnl_pageContainer
            // 
            this.pnl_pageContainer.BackColor = System.Drawing.Color.Transparent;
            this.pnl_pageContainer.Location = new System.Drawing.Point(0, 110);
            this.pnl_pageContainer.Name = "pnl_pageContainer";
            this.pnl_pageContainer.Size = new System.Drawing.Size(1024, 658);
            this.pnl_pageContainer.TabIndex = 1;
            // 
            // pnl_header
            // 
            this.pnl_header.BackColor = System.Drawing.Color.RoyalBlue;
            this.pnl_header.BackgroundColor = System.Drawing.Color.RoyalBlue;
            this.pnl_header.BorderColor = System.Drawing.Color.Black;
            this.pnl_header.BorderRadius = 0;
            this.pnl_header.BorderSize = 0;
            this.pnl_header.Controls.Add(this.pnl_safetyTask);
            this.pnl_header.Controls.Add(this.pnl_plcTask);
            this.pnl_header.Controls.Add(this.pnl_appTask);
            this.pnl_header.Controls.Add(this.pnl_safeZone);
            this.pnl_header.Controls.Add(this.pnl_comRobotTask);
            this.pnl_header.Controls.Add(this.Pnl_ROBOT_alarm);
            this.pnl_header.Controls.Add(this.pnl_auxTask);
            this.pnl_header.Controls.Add(this.Pnl_PLC_alarm);
            this.pnl_header.Controls.Add(this.pnl_lowTask);
            this.pnl_header.Controls.Add(this.Pnl_diagnostics);
            this.pnl_header.Controls.Add(this.pnl_highTask);
            this.pnl_header.Controls.Add(this.pnl_ActiveAlarms);
            this.pnl_header.Controls.Add(this.PnlLogoContainer);
            this.pnl_header.Controls.Add(this.Pnl_MOVROBOT_alarm);
            this.pnl_header.Controls.Add(this.lbl_pageTitle);
            this.pnl_header.Controls.Add(this.lbl_dateTime);
            this.pnl_header.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.pnl_header.ForeColor = System.Drawing.Color.White;
            this.pnl_header.Location = new System.Drawing.Point(-4, -20);
            this.pnl_header.Margin = new System.Windows.Forms.Padding(0);
            this.pnl_header.Name = "pnl_header";
            this.pnl_header.Size = new System.Drawing.Size(1030, 120);
            this.pnl_header.TabIndex = 0;
            this.pnl_header.TextColor = System.Drawing.Color.White;
            // 
            // pnl_safetyTask
            // 
            this.pnl_safetyTask.Controls.Add(this.pnl_safetyTaskStatus);
            this.pnl_safetyTask.Controls.Add(this.label7);
            this.pnl_safetyTask.Location = new System.Drawing.Point(688, 74);
            this.pnl_safetyTask.Name = "pnl_safetyTask";
            this.pnl_safetyTask.Size = new System.Drawing.Size(41, 42);
            this.pnl_safetyTask.TabIndex = 41;
            // 
            // pnl_safetyTaskStatus
            // 
            this.pnl_safetyTaskStatus.BackColor = System.Drawing.Color.White;
            this.pnl_safetyTaskStatus.BackgroundColor = System.Drawing.Color.White;
            this.pnl_safetyTaskStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_safetyTaskStatus.BorderColor = System.Drawing.Color.Black;
            this.pnl_safetyTaskStatus.BorderRadius = 10;
            this.pnl_safetyTaskStatus.BorderSize = 0;
            this.pnl_safetyTaskStatus.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.pnl_safetyTaskStatus.ForeColor = System.Drawing.Color.White;
            this.pnl_safetyTaskStatus.Location = new System.Drawing.Point(10, 4);
            this.pnl_safetyTaskStatus.Margin = new System.Windows.Forms.Padding(0);
            this.pnl_safetyTaskStatus.Name = "pnl_safetyTaskStatus";
            this.pnl_safetyTaskStatus.Size = new System.Drawing.Size(20, 20);
            this.pnl_safetyTaskStatus.TabIndex = 30;
            this.pnl_safetyTaskStatus.TextColor = System.Drawing.Color.White;
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.BackColor = System.Drawing.Color.Transparent;
            this.label7.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.Location = new System.Drawing.Point(-1, 27);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(42, 16);
            this.label7.TabIndex = 32;
            this.label7.Text = "SAFE";
            this.label7.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pnl_plcTask
            // 
            this.pnl_plcTask.Controls.Add(this.pnl_plcTaskStatus);
            this.pnl_plcTask.Controls.Add(this.label3);
            this.pnl_plcTask.Location = new System.Drawing.Point(733, 74);
            this.pnl_plcTask.Name = "pnl_plcTask";
            this.pnl_plcTask.Size = new System.Drawing.Size(41, 42);
            this.pnl_plcTask.TabIndex = 40;
            // 
            // pnl_plcTaskStatus
            // 
            this.pnl_plcTaskStatus.BackColor = System.Drawing.Color.White;
            this.pnl_plcTaskStatus.BackgroundColor = System.Drawing.Color.White;
            this.pnl_plcTaskStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_plcTaskStatus.BorderColor = System.Drawing.Color.Black;
            this.pnl_plcTaskStatus.BorderRadius = 10;
            this.pnl_plcTaskStatus.BorderSize = 0;
            this.pnl_plcTaskStatus.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.pnl_plcTaskStatus.ForeColor = System.Drawing.Color.White;
            this.pnl_plcTaskStatus.Location = new System.Drawing.Point(10, 4);
            this.pnl_plcTaskStatus.Margin = new System.Windows.Forms.Padding(0);
            this.pnl_plcTaskStatus.Name = "pnl_plcTaskStatus";
            this.pnl_plcTaskStatus.Size = new System.Drawing.Size(20, 20);
            this.pnl_plcTaskStatus.TabIndex = 30;
            this.pnl_plcTaskStatus.TextColor = System.Drawing.Color.White;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.BackColor = System.Drawing.Color.Transparent;
            this.label3.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.Location = new System.Drawing.Point(5, 27);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(32, 16);
            this.label3.TabIndex = 32;
            this.label3.Text = "PLC";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pnl_appTask
            // 
            this.pnl_appTask.Controls.Add(this.pnl_appTaskStatus);
            this.pnl_appTask.Controls.Add(this.label6);
            this.pnl_appTask.Location = new System.Drawing.Point(778, 74);
            this.pnl_appTask.Name = "pnl_appTask";
            this.pnl_appTask.Size = new System.Drawing.Size(41, 42);
            this.pnl_appTask.TabIndex = 39;
            // 
            // pnl_appTaskStatus
            // 
            this.pnl_appTaskStatus.BackColor = System.Drawing.Color.White;
            this.pnl_appTaskStatus.BackgroundColor = System.Drawing.Color.White;
            this.pnl_appTaskStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_appTaskStatus.BorderColor = System.Drawing.Color.Black;
            this.pnl_appTaskStatus.BorderRadius = 10;
            this.pnl_appTaskStatus.BorderSize = 0;
            this.pnl_appTaskStatus.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.pnl_appTaskStatus.ForeColor = System.Drawing.Color.White;
            this.pnl_appTaskStatus.Location = new System.Drawing.Point(10, 4);
            this.pnl_appTaskStatus.Margin = new System.Windows.Forms.Padding(0);
            this.pnl_appTaskStatus.Name = "pnl_appTaskStatus";
            this.pnl_appTaskStatus.Size = new System.Drawing.Size(20, 20);
            this.pnl_appTaskStatus.TabIndex = 30;
            this.pnl_appTaskStatus.TextColor = System.Drawing.Color.White;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.BackColor = System.Drawing.Color.Transparent;
            this.label6.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.Location = new System.Drawing.Point(4, 27);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(34, 16);
            this.label6.TabIndex = 32;
            this.label6.Text = "APP";
            this.label6.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pnl_safeZone
            // 
            this.pnl_safeZone.BackColor = System.Drawing.Color.Transparent;
            this.pnl_safeZone.BackgroundImage = global::RM.Properties.Resources.safeZone_green32;
            this.pnl_safeZone.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_safeZone.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pnl_safeZone.Location = new System.Drawing.Point(731, 30);
            this.pnl_safeZone.Name = "pnl_safeZone";
            this.pnl_safeZone.Size = new System.Drawing.Size(40, 40);
            this.pnl_safeZone.TabIndex = 28;
            // 
            // pnl_comRobotTask
            // 
            this.pnl_comRobotTask.Controls.Add(this.pnl_comRobotTaskStatus);
            this.pnl_comRobotTask.Controls.Add(this.label5);
            this.pnl_comRobotTask.Location = new System.Drawing.Point(958, 74);
            this.pnl_comRobotTask.Name = "pnl_comRobotTask";
            this.pnl_comRobotTask.Size = new System.Drawing.Size(41, 42);
            this.pnl_comRobotTask.TabIndex = 40;
            // 
            // pnl_comRobotTaskStatus
            // 
            this.pnl_comRobotTaskStatus.BackColor = System.Drawing.Color.White;
            this.pnl_comRobotTaskStatus.BackgroundColor = System.Drawing.Color.White;
            this.pnl_comRobotTaskStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_comRobotTaskStatus.BorderColor = System.Drawing.Color.Black;
            this.pnl_comRobotTaskStatus.BorderRadius = 10;
            this.pnl_comRobotTaskStatus.BorderSize = 0;
            this.pnl_comRobotTaskStatus.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.pnl_comRobotTaskStatus.ForeColor = System.Drawing.Color.White;
            this.pnl_comRobotTaskStatus.Location = new System.Drawing.Point(10, 4);
            this.pnl_comRobotTaskStatus.Margin = new System.Windows.Forms.Padding(0);
            this.pnl_comRobotTaskStatus.Name = "pnl_comRobotTaskStatus";
            this.pnl_comRobotTaskStatus.Size = new System.Drawing.Size(20, 20);
            this.pnl_comRobotTaskStatus.TabIndex = 30;
            this.pnl_comRobotTaskStatus.TextColor = System.Drawing.Color.White;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.BackColor = System.Drawing.Color.Transparent;
            this.label5.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.Location = new System.Drawing.Point(3, 27);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(37, 16);
            this.label5.TabIndex = 32;
            this.label5.Text = "COM";
            this.label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // Pnl_ROBOT_alarm
            // 
            this.Pnl_ROBOT_alarm.BackColor = System.Drawing.Color.Transparent;
            this.Pnl_ROBOT_alarm.BackgroundImage = global::RM.Properties.Resources.robot_arm_connection;
            this.Pnl_ROBOT_alarm.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.Pnl_ROBOT_alarm.Location = new System.Drawing.Point(959, 30);
            this.Pnl_ROBOT_alarm.Name = "Pnl_ROBOT_alarm";
            this.Pnl_ROBOT_alarm.Size = new System.Drawing.Size(40, 40);
            this.Pnl_ROBOT_alarm.TabIndex = 25;
            // 
            // pnl_auxTask
            // 
            this.pnl_auxTask.Controls.Add(this.pnl_auxTaskStatus);
            this.pnl_auxTask.Controls.Add(this.label2);
            this.pnl_auxTask.Location = new System.Drawing.Point(823, 74);
            this.pnl_auxTask.Name = "pnl_auxTask";
            this.pnl_auxTask.Size = new System.Drawing.Size(41, 42);
            this.pnl_auxTask.TabIndex = 36;
            // 
            // pnl_auxTaskStatus
            // 
            this.pnl_auxTaskStatus.BackColor = System.Drawing.Color.White;
            this.pnl_auxTaskStatus.BackgroundColor = System.Drawing.Color.White;
            this.pnl_auxTaskStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_auxTaskStatus.BorderColor = System.Drawing.Color.Black;
            this.pnl_auxTaskStatus.BorderRadius = 10;
            this.pnl_auxTaskStatus.BorderSize = 0;
            this.pnl_auxTaskStatus.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.pnl_auxTaskStatus.ForeColor = System.Drawing.Color.White;
            this.pnl_auxTaskStatus.Location = new System.Drawing.Point(10, 4);
            this.pnl_auxTaskStatus.Margin = new System.Windows.Forms.Padding(0);
            this.pnl_auxTaskStatus.Name = "pnl_auxTaskStatus";
            this.pnl_auxTaskStatus.Size = new System.Drawing.Size(20, 20);
            this.pnl_auxTaskStatus.TabIndex = 30;
            this.pnl_auxTaskStatus.TextColor = System.Drawing.Color.White;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.BackColor = System.Drawing.Color.Transparent;
            this.label2.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(5, 27);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(32, 16);
            this.label2.TabIndex = 32;
            this.label2.Text = "AUX";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // Pnl_PLC_alarm
            // 
            this.Pnl_PLC_alarm.BackColor = System.Drawing.Color.Transparent;
            this.Pnl_PLC_alarm.BackgroundImage = global::RM.Properties.Resources.connection_icon;
            this.Pnl_PLC_alarm.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.Pnl_PLC_alarm.Location = new System.Drawing.Point(914, 30);
            this.Pnl_PLC_alarm.Name = "Pnl_PLC_alarm";
            this.Pnl_PLC_alarm.Size = new System.Drawing.Size(40, 40);
            this.Pnl_PLC_alarm.TabIndex = 24;
            // 
            // pnl_lowTask
            // 
            this.pnl_lowTask.Controls.Add(this.pnl_lowTaskStatus);
            this.pnl_lowTask.Controls.Add(this.label1);
            this.pnl_lowTask.Location = new System.Drawing.Point(868, 74);
            this.pnl_lowTask.Name = "pnl_lowTask";
            this.pnl_lowTask.Size = new System.Drawing.Size(41, 42);
            this.pnl_lowTask.TabIndex = 38;
            // 
            // pnl_lowTaskStatus
            // 
            this.pnl_lowTaskStatus.BackColor = System.Drawing.Color.White;
            this.pnl_lowTaskStatus.BackgroundColor = System.Drawing.Color.White;
            this.pnl_lowTaskStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_lowTaskStatus.BorderColor = System.Drawing.Color.Black;
            this.pnl_lowTaskStatus.BorderRadius = 10;
            this.pnl_lowTaskStatus.BorderSize = 0;
            this.pnl_lowTaskStatus.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.pnl_lowTaskStatus.ForeColor = System.Drawing.Color.White;
            this.pnl_lowTaskStatus.Location = new System.Drawing.Point(10, 4);
            this.pnl_lowTaskStatus.Margin = new System.Windows.Forms.Padding(0);
            this.pnl_lowTaskStatus.Name = "pnl_lowTaskStatus";
            this.pnl_lowTaskStatus.Size = new System.Drawing.Size(20, 20);
            this.pnl_lowTaskStatus.TabIndex = 30;
            this.pnl_lowTaskStatus.TextColor = System.Drawing.Color.White;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.BackColor = System.Drawing.Color.Transparent;
            this.label1.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(3, 27);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(37, 16);
            this.label1.TabIndex = 32;
            this.label1.Text = "LOW";
            this.label1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // Pnl_diagnostics
            // 
            this.Pnl_diagnostics.BackColor = System.Drawing.Color.Transparent;
            this.Pnl_diagnostics.BackgroundImage = global::RM.Properties.Resources.diagnostic;
            this.Pnl_diagnostics.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.Pnl_diagnostics.Cursor = System.Windows.Forms.Cursors.Hand;
            this.Pnl_diagnostics.Location = new System.Drawing.Point(778, 30);
            this.Pnl_diagnostics.Name = "Pnl_diagnostics";
            this.Pnl_diagnostics.Size = new System.Drawing.Size(40, 40);
            this.Pnl_diagnostics.TabIndex = 26;
            this.Pnl_diagnostics.Click += new System.EventHandler(this.Pnl_diagnostics_Click);
            // 
            // pnl_highTask
            // 
            this.pnl_highTask.Controls.Add(this.pnl_highTaskStatus);
            this.pnl_highTask.Controls.Add(this.label4);
            this.pnl_highTask.Location = new System.Drawing.Point(913, 74);
            this.pnl_highTask.Name = "pnl_highTask";
            this.pnl_highTask.Size = new System.Drawing.Size(41, 42);
            this.pnl_highTask.TabIndex = 35;
            // 
            // pnl_highTaskStatus
            // 
            this.pnl_highTaskStatus.BackColor = System.Drawing.Color.White;
            this.pnl_highTaskStatus.BackgroundColor = System.Drawing.Color.White;
            this.pnl_highTaskStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_highTaskStatus.BorderColor = System.Drawing.Color.Black;
            this.pnl_highTaskStatus.BorderRadius = 10;
            this.pnl_highTaskStatus.BorderSize = 0;
            this.pnl_highTaskStatus.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.pnl_highTaskStatus.ForeColor = System.Drawing.Color.White;
            this.pnl_highTaskStatus.Location = new System.Drawing.Point(10, 4);
            this.pnl_highTaskStatus.Margin = new System.Windows.Forms.Padding(0);
            this.pnl_highTaskStatus.Name = "pnl_highTaskStatus";
            this.pnl_highTaskStatus.Size = new System.Drawing.Size(20, 20);
            this.pnl_highTaskStatus.TabIndex = 30;
            this.pnl_highTaskStatus.TextColor = System.Drawing.Color.White;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.BackColor = System.Drawing.Color.Transparent;
            this.label4.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.Location = new System.Drawing.Point(2, 27);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(38, 16);
            this.label4.TabIndex = 32;
            this.label4.Text = "HIGH";
            this.label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pnl_ActiveAlarms
            // 
            this.pnl_ActiveAlarms.BackColor = System.Drawing.Color.Transparent;
            this.pnl_ActiveAlarms.BackgroundImage = global::RM.Properties.Resources.alarm_popup_grey;
            this.pnl_ActiveAlarms.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pnl_ActiveAlarms.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pnl_ActiveAlarms.Location = new System.Drawing.Point(869, 30);
            this.pnl_ActiveAlarms.Name = "pnl_ActiveAlarms";
            this.pnl_ActiveAlarms.Size = new System.Drawing.Size(40, 40);
            this.pnl_ActiveAlarms.TabIndex = 25;
            this.pnl_ActiveAlarms.Click += new System.EventHandler(this.ClickEvent_alarms);
            // 
            // PnlLogoContainer
            // 
            this.PnlLogoContainer.BackColor = System.Drawing.Color.White;
            this.PnlLogoContainer.BackgroundColor = System.Drawing.Color.White;
            this.PnlLogoContainer.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.PnlLogoContainer.BorderColor = System.Drawing.Color.Black;
            this.PnlLogoContainer.BorderRadius = 15;
            this.PnlLogoContainer.BorderSize = 0;
            this.PnlLogoContainer.Controls.Add(this.Pnl_logo_rm);
            this.PnlLogoContainer.Cursor = System.Windows.Forms.Cursors.Hand;
            this.PnlLogoContainer.ForeColor = System.Drawing.Color.White;
            this.PnlLogoContainer.Location = new System.Drawing.Point(20, 42);
            this.PnlLogoContainer.Margin = new System.Windows.Forms.Padding(0);
            this.PnlLogoContainer.Name = "PnlLogoContainer";
            this.PnlLogoContainer.Size = new System.Drawing.Size(208, 54);
            this.PnlLogoContainer.TabIndex = 28;
            this.PnlLogoContainer.TextColor = System.Drawing.Color.White;
            // 
            // Pnl_logo_rm
            // 
            this.Pnl_logo_rm.BackColor = System.Drawing.Color.White;
            this.Pnl_logo_rm.BackgroundColor = System.Drawing.Color.White;
            this.Pnl_logo_rm.BackgroundImage = global::RM.Properties.Resources.RM_logo;
            this.Pnl_logo_rm.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.Pnl_logo_rm.BorderColor = System.Drawing.Color.Black;
            this.Pnl_logo_rm.BorderRadius = 0;
            this.Pnl_logo_rm.BorderSize = 0;
            this.Pnl_logo_rm.ForeColor = System.Drawing.Color.White;
            this.Pnl_logo_rm.Location = new System.Drawing.Point(14, 3);
            this.Pnl_logo_rm.Name = "Pnl_logo_rm";
            this.Pnl_logo_rm.Size = new System.Drawing.Size(180, 49);
            this.Pnl_logo_rm.TabIndex = 29;
            this.Pnl_logo_rm.TextColor = System.Drawing.Color.White;
            this.Pnl_logo_rm.DoubleClick += new System.EventHandler(this.DoubleClickEvent_showSwVersion);
            // 
            // Pnl_MOVROBOT_alarm
            // 
            this.Pnl_MOVROBOT_alarm.BackColor = System.Drawing.Color.Transparent;
            this.Pnl_MOVROBOT_alarm.BackgroundImage = global::RM.Properties.Resources.noMov;
            this.Pnl_MOVROBOT_alarm.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.Pnl_MOVROBOT_alarm.Location = new System.Drawing.Point(824, 30);
            this.Pnl_MOVROBOT_alarm.Name = "Pnl_MOVROBOT_alarm";
            this.Pnl_MOVROBOT_alarm.Size = new System.Drawing.Size(40, 40);
            this.Pnl_MOVROBOT_alarm.TabIndex = 26;
            // 
            // lbl_pageTitle
            // 
            this.lbl_pageTitle.BackColor = System.Drawing.Color.Transparent;
            this.lbl_pageTitle.Font = new System.Drawing.Font("Arial", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_pageTitle.Location = new System.Drawing.Point(-2, 74);
            this.lbl_pageTitle.Name = "lbl_pageTitle";
            this.lbl_pageTitle.Size = new System.Drawing.Size(1030, 34);
            this.lbl_pageTitle.TabIndex = 25;
            this.lbl_pageTitle.Text = "HOMEPAGE";
            this.lbl_pageTitle.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_dateTime
            // 
            this.lbl_dateTime.BackColor = System.Drawing.Color.Transparent;
            this.lbl_dateTime.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_dateTime.Location = new System.Drawing.Point(298, 29);
            this.lbl_dateTime.Name = "lbl_dateTime";
            this.lbl_dateTime.Size = new System.Drawing.Size(427, 25);
            this.lbl_dateTime.TabIndex = 4;
            this.lbl_dateTime.Text = "12:34:56";
            this.lbl_dateTime.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // FormHomePage
            // 
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.None;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.BackColor = System.Drawing.Color.CornflowerBlue;
            this.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.ClientSize = new System.Drawing.Size(1024, 767);
            this.Controls.Add(this.pnl_pageContainer);
            this.Controls.Add(this.pnl_header);
            this.DoubleBuffered = true;
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "FormHomePage";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.ClosingEvent_homePageClosing);
            this.Load += new System.EventHandler(this.FormHomePage_Load);
            this.Shown += new System.EventHandler(this.FormHomePage_Shown);
            this.pnl_header.ResumeLayout(false);
            this.pnl_safetyTask.ResumeLayout(false);
            this.pnl_safetyTask.PerformLayout();
            this.pnl_plcTask.ResumeLayout(false);
            this.pnl_plcTask.PerformLayout();
            this.pnl_appTask.ResumeLayout(false);
            this.pnl_appTask.PerformLayout();
            this.pnl_comRobotTask.ResumeLayout(false);
            this.pnl_comRobotTask.PerformLayout();
            this.pnl_auxTask.ResumeLayout(false);
            this.pnl_auxTask.PerformLayout();
            this.pnl_lowTask.ResumeLayout(false);
            this.pnl_lowTask.PerformLayout();
            this.pnl_highTask.ResumeLayout(false);
            this.pnl_highTask.PerformLayout();
            this.PnlLogoContainer.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private global::RMLib.View.CustomPanel pnl_header;
        private System.Windows.Forms.Panel pnl_pageContainer;
        private System.Windows.Forms.Label lbl_dateTime;
        private System.Windows.Forms.Timer timer_dateTime_clock;
        private System.Windows.Forms.Panel Pnl_ROBOT_alarm;
        private System.Windows.Forms.Panel Pnl_PLC_alarm;
        private System.Windows.Forms.Label lbl_pageTitle;
        private System.Windows.Forms.Panel Pnl_MOVROBOT_alarm;
        private System.Windows.Forms.Panel pnl_ActiveAlarms;
        private RMLib.View.CustomPanel Pnl_logo_rm;
        private RMLib.View.CustomPanel PnlLogoContainer;
        private System.Windows.Forms.Panel Pnl_diagnostics;
        private System.Windows.Forms.Panel pnl_safeZone;
        private System.Windows.Forms.Panel pnl_appTask;
        private RMLib.View.CustomPanel pnl_appTaskStatus;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Panel pnl_comRobotTask;
        private RMLib.View.CustomPanel pnl_comRobotTaskStatus;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Panel pnl_auxTask;
        private RMLib.View.CustomPanel pnl_auxTaskStatus;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Panel pnl_lowTask;
        private RMLib.View.CustomPanel pnl_lowTaskStatus;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Panel pnl_highTask;
        private RMLib.View.CustomPanel pnl_highTaskStatus;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Panel pnl_plcTask;
        private RMLib.View.CustomPanel pnl_plcTaskStatus;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Panel pnl_safetyTask;
        private RMLib.View.CustomPanel pnl_safetyTaskStatus;
        private System.Windows.Forms.Label label7;
    }
}

