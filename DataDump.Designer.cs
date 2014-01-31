namespace WindowsFormsApplication1
{
    partial class DataDump
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
            this.comboBoxMode = new System.Windows.Forms.ComboBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.textBoxQuaternionW = new System.Windows.Forms.TextBox();
            this.textBoxQuaternionX = new System.Windows.Forms.TextBox();
            this.textBoxQuaternionY = new System.Windows.Forms.TextBox();
            this.textBoxQuaternionZ = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.textBoxYaw = new System.Windows.Forms.TextBox();
            this.textBoxPitch = new System.Windows.Forms.TextBox();
            this.textBoxRoll = new System.Windows.Forms.TextBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.label5 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.textBoxAccelerometerZ = new System.Windows.Forms.TextBox();
            this.textBoxAccelerometerY = new System.Windows.Forms.TextBox();
            this.textBoxAccelerometerX = new System.Windows.Forms.TextBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.textBoxMagnetometerZ = new System.Windows.Forms.TextBox();
            this.textBoxMagnetometerY = new System.Windows.Forms.TextBox();
            this.textBoxMagnetometerX = new System.Windows.Forms.TextBox();
            this.textBoxAccelerometerXFix = new System.Windows.Forms.TextBox();
            this.textBoxAccelerometerYFix = new System.Windows.Forms.TextBox();
            this.textBoxAccelerometerZFix = new System.Windows.Forms.TextBox();
            this.textBoxMagnetometerZFix = new System.Windows.Forms.TextBox();
            this.textBoxMagnetometerYFix = new System.Windows.Forms.TextBox();
            this.textBoxMagnetometerXFix = new System.Windows.Forms.TextBox();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.SuspendLayout();
            // 
            // comboBoxMode
            // 
            this.comboBoxMode.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.comboBoxMode.FormattingEnabled = true;
            this.comboBoxMode.Location = new System.Drawing.Point(12, 12);
            this.comboBoxMode.Name = "comboBoxMode";
            this.comboBoxMode.Size = new System.Drawing.Size(178, 21);
            this.comboBoxMode.TabIndex = 0;
            this.comboBoxMode.SelectedIndexChanged += new System.EventHandler(this.comboBoxMode_SelectedIndexChanged);
            this.comboBoxMode.SelectedValueChanged += new System.EventHandler(this.comboBoxMode_SelectedValueChanged);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.textBoxQuaternionZ);
            this.groupBox1.Controls.Add(this.textBoxQuaternionY);
            this.groupBox1.Controls.Add(this.textBoxQuaternionX);
            this.groupBox1.Controls.Add(this.textBoxQuaternionW);
            this.groupBox1.Location = new System.Drawing.Point(12, 39);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(178, 130);
            this.groupBox1.TabIndex = 1;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Quaternion";
            // 
            // textBoxQuaternionW
            // 
            this.textBoxQuaternionW.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxQuaternionW.ForeColor = System.Drawing.Color.White;
            this.textBoxQuaternionW.Location = new System.Drawing.Point(27, 19);
            this.textBoxQuaternionW.Name = "textBoxQuaternionW";
            this.textBoxQuaternionW.ReadOnly = true;
            this.textBoxQuaternionW.Size = new System.Drawing.Size(145, 20);
            this.textBoxQuaternionW.TabIndex = 2;
            this.textBoxQuaternionW.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxQuaternionX
            // 
            this.textBoxQuaternionX.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxQuaternionX.ForeColor = System.Drawing.Color.White;
            this.textBoxQuaternionX.Location = new System.Drawing.Point(27, 45);
            this.textBoxQuaternionX.Name = "textBoxQuaternionX";
            this.textBoxQuaternionX.ReadOnly = true;
            this.textBoxQuaternionX.Size = new System.Drawing.Size(145, 20);
            this.textBoxQuaternionX.TabIndex = 3;
            this.textBoxQuaternionX.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxQuaternionY
            // 
            this.textBoxQuaternionY.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxQuaternionY.ForeColor = System.Drawing.Color.White;
            this.textBoxQuaternionY.Location = new System.Drawing.Point(27, 71);
            this.textBoxQuaternionY.Name = "textBoxQuaternionY";
            this.textBoxQuaternionY.ReadOnly = true;
            this.textBoxQuaternionY.Size = new System.Drawing.Size(145, 20);
            this.textBoxQuaternionY.TabIndex = 4;
            this.textBoxQuaternionY.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxQuaternionZ
            // 
            this.textBoxQuaternionZ.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxQuaternionZ.ForeColor = System.Drawing.Color.White;
            this.textBoxQuaternionZ.Location = new System.Drawing.Point(27, 97);
            this.textBoxQuaternionZ.Name = "textBoxQuaternionZ";
            this.textBoxQuaternionZ.ReadOnly = true;
            this.textBoxQuaternionZ.Size = new System.Drawing.Size(145, 20);
            this.textBoxQuaternionZ.TabIndex = 5;
            this.textBoxQuaternionZ.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(6, 22);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(15, 13);
            this.label1.TabIndex = 6;
            this.label1.Text = "w";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(6, 48);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(14, 13);
            this.label2.TabIndex = 7;
            this.label2.Text = "X";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(6, 74);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(14, 13);
            this.label3.TabIndex = 8;
            this.label3.Text = "Y";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(6, 100);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(14, 13);
            this.label4.TabIndex = 9;
            this.label4.Text = "Z";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.label6);
            this.groupBox2.Controls.Add(this.label7);
            this.groupBox2.Controls.Add(this.label8);
            this.groupBox2.Controls.Add(this.textBoxYaw);
            this.groupBox2.Controls.Add(this.textBoxPitch);
            this.groupBox2.Controls.Add(this.textBoxRoll);
            this.groupBox2.Location = new System.Drawing.Point(196, 39);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(178, 130);
            this.groupBox2.TabIndex = 10;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Angles";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(6, 74);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(26, 13);
            this.label6.TabIndex = 8;
            this.label6.Text = "yaw";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(6, 48);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(30, 13);
            this.label7.TabIndex = 7;
            this.label7.Text = "pitch";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(6, 22);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(20, 13);
            this.label8.TabIndex = 6;
            this.label8.Text = "roll";
            // 
            // textBoxYaw
            // 
            this.textBoxYaw.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxYaw.ForeColor = System.Drawing.Color.White;
            this.textBoxYaw.Location = new System.Drawing.Point(49, 71);
            this.textBoxYaw.Name = "textBoxYaw";
            this.textBoxYaw.ReadOnly = true;
            this.textBoxYaw.Size = new System.Drawing.Size(123, 20);
            this.textBoxYaw.TabIndex = 4;
            this.textBoxYaw.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxPitch
            // 
            this.textBoxPitch.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxPitch.ForeColor = System.Drawing.Color.White;
            this.textBoxPitch.Location = new System.Drawing.Point(49, 45);
            this.textBoxPitch.Name = "textBoxPitch";
            this.textBoxPitch.ReadOnly = true;
            this.textBoxPitch.Size = new System.Drawing.Size(123, 20);
            this.textBoxPitch.TabIndex = 3;
            this.textBoxPitch.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxRoll
            // 
            this.textBoxRoll.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxRoll.ForeColor = System.Drawing.Color.White;
            this.textBoxRoll.Location = new System.Drawing.Point(49, 19);
            this.textBoxRoll.Name = "textBoxRoll";
            this.textBoxRoll.ReadOnly = true;
            this.textBoxRoll.Size = new System.Drawing.Size(123, 20);
            this.textBoxRoll.TabIndex = 2;
            this.textBoxRoll.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.textBoxAccelerometerZFix);
            this.groupBox3.Controls.Add(this.textBoxAccelerometerYFix);
            this.groupBox3.Controls.Add(this.textBoxAccelerometerXFix);
            this.groupBox3.Controls.Add(this.label5);
            this.groupBox3.Controls.Add(this.label9);
            this.groupBox3.Controls.Add(this.label10);
            this.groupBox3.Controls.Add(this.textBoxAccelerometerZ);
            this.groupBox3.Controls.Add(this.textBoxAccelerometerY);
            this.groupBox3.Controls.Add(this.textBoxAccelerometerX);
            this.groupBox3.Location = new System.Drawing.Point(380, 39);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(178, 130);
            this.groupBox3.TabIndex = 11;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Accelerometer";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(6, 74);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(14, 13);
            this.label5.TabIndex = 8;
            this.label5.Text = "Z";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(6, 48);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(14, 13);
            this.label9.TabIndex = 7;
            this.label9.Text = "Y";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(6, 22);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(14, 13);
            this.label10.TabIndex = 6;
            this.label10.Text = "X";
            // 
            // textBoxAccelerometerZ
            // 
            this.textBoxAccelerometerZ.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxAccelerometerZ.ForeColor = System.Drawing.Color.White;
            this.textBoxAccelerometerZ.Location = new System.Drawing.Point(26, 71);
            this.textBoxAccelerometerZ.Name = "textBoxAccelerometerZ";
            this.textBoxAccelerometerZ.ReadOnly = true;
            this.textBoxAccelerometerZ.Size = new System.Drawing.Size(67, 20);
            this.textBoxAccelerometerZ.TabIndex = 4;
            this.textBoxAccelerometerZ.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxAccelerometerY
            // 
            this.textBoxAccelerometerY.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxAccelerometerY.ForeColor = System.Drawing.Color.White;
            this.textBoxAccelerometerY.Location = new System.Drawing.Point(26, 45);
            this.textBoxAccelerometerY.Name = "textBoxAccelerometerY";
            this.textBoxAccelerometerY.ReadOnly = true;
            this.textBoxAccelerometerY.Size = new System.Drawing.Size(67, 20);
            this.textBoxAccelerometerY.TabIndex = 3;
            this.textBoxAccelerometerY.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxAccelerometerX
            // 
            this.textBoxAccelerometerX.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxAccelerometerX.ForeColor = System.Drawing.Color.White;
            this.textBoxAccelerometerX.Location = new System.Drawing.Point(26, 19);
            this.textBoxAccelerometerX.Name = "textBoxAccelerometerX";
            this.textBoxAccelerometerX.ReadOnly = true;
            this.textBoxAccelerometerX.Size = new System.Drawing.Size(67, 20);
            this.textBoxAccelerometerX.TabIndex = 2;
            this.textBoxAccelerometerX.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.textBoxMagnetometerZFix);
            this.groupBox4.Controls.Add(this.textBoxMagnetometerYFix);
            this.groupBox4.Controls.Add(this.textBoxMagnetometerXFix);
            this.groupBox4.Controls.Add(this.label11);
            this.groupBox4.Controls.Add(this.label12);
            this.groupBox4.Controls.Add(this.label13);
            this.groupBox4.Controls.Add(this.textBoxMagnetometerZ);
            this.groupBox4.Controls.Add(this.textBoxMagnetometerY);
            this.groupBox4.Controls.Add(this.textBoxMagnetometerX);
            this.groupBox4.Location = new System.Drawing.Point(564, 39);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(178, 130);
            this.groupBox4.TabIndex = 12;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Magnetometer";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(6, 74);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(14, 13);
            this.label11.TabIndex = 8;
            this.label11.Text = "Z";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(6, 48);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(14, 13);
            this.label12.TabIndex = 7;
            this.label12.Text = "Y";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(6, 22);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(14, 13);
            this.label13.TabIndex = 6;
            this.label13.Text = "X";
            // 
            // textBoxMagnetometerZ
            // 
            this.textBoxMagnetometerZ.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxMagnetometerZ.ForeColor = System.Drawing.Color.White;
            this.textBoxMagnetometerZ.Location = new System.Drawing.Point(26, 71);
            this.textBoxMagnetometerZ.Name = "textBoxMagnetometerZ";
            this.textBoxMagnetometerZ.ReadOnly = true;
            this.textBoxMagnetometerZ.Size = new System.Drawing.Size(67, 20);
            this.textBoxMagnetometerZ.TabIndex = 4;
            this.textBoxMagnetometerZ.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxMagnetometerY
            // 
            this.textBoxMagnetometerY.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxMagnetometerY.ForeColor = System.Drawing.Color.White;
            this.textBoxMagnetometerY.Location = new System.Drawing.Point(26, 45);
            this.textBoxMagnetometerY.Name = "textBoxMagnetometerY";
            this.textBoxMagnetometerY.ReadOnly = true;
            this.textBoxMagnetometerY.Size = new System.Drawing.Size(67, 20);
            this.textBoxMagnetometerY.TabIndex = 3;
            this.textBoxMagnetometerY.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxMagnetometerX
            // 
            this.textBoxMagnetometerX.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxMagnetometerX.ForeColor = System.Drawing.Color.White;
            this.textBoxMagnetometerX.Location = new System.Drawing.Point(26, 19);
            this.textBoxMagnetometerX.Name = "textBoxMagnetometerX";
            this.textBoxMagnetometerX.ReadOnly = true;
            this.textBoxMagnetometerX.Size = new System.Drawing.Size(67, 20);
            this.textBoxMagnetometerX.TabIndex = 2;
            this.textBoxMagnetometerX.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxAccelerometerXFix
            // 
            this.textBoxAccelerometerXFix.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxAccelerometerXFix.ForeColor = System.Drawing.Color.White;
            this.textBoxAccelerometerXFix.Location = new System.Drawing.Point(105, 19);
            this.textBoxAccelerometerXFix.Name = "textBoxAccelerometerXFix";
            this.textBoxAccelerometerXFix.ReadOnly = true;
            this.textBoxAccelerometerXFix.Size = new System.Drawing.Size(67, 20);
            this.textBoxAccelerometerXFix.TabIndex = 9;
            this.textBoxAccelerometerXFix.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxAccelerometerYFix
            // 
            this.textBoxAccelerometerYFix.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxAccelerometerYFix.ForeColor = System.Drawing.Color.White;
            this.textBoxAccelerometerYFix.Location = new System.Drawing.Point(105, 45);
            this.textBoxAccelerometerYFix.Name = "textBoxAccelerometerYFix";
            this.textBoxAccelerometerYFix.ReadOnly = true;
            this.textBoxAccelerometerYFix.Size = new System.Drawing.Size(67, 20);
            this.textBoxAccelerometerYFix.TabIndex = 10;
            this.textBoxAccelerometerYFix.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxAccelerometerZFix
            // 
            this.textBoxAccelerometerZFix.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxAccelerometerZFix.ForeColor = System.Drawing.Color.White;
            this.textBoxAccelerometerZFix.Location = new System.Drawing.Point(105, 71);
            this.textBoxAccelerometerZFix.Name = "textBoxAccelerometerZFix";
            this.textBoxAccelerometerZFix.ReadOnly = true;
            this.textBoxAccelerometerZFix.Size = new System.Drawing.Size(67, 20);
            this.textBoxAccelerometerZFix.TabIndex = 11;
            this.textBoxAccelerometerZFix.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxMagnetometerZFix
            // 
            this.textBoxMagnetometerZFix.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxMagnetometerZFix.ForeColor = System.Drawing.Color.White;
            this.textBoxMagnetometerZFix.Location = new System.Drawing.Point(105, 71);
            this.textBoxMagnetometerZFix.Name = "textBoxMagnetometerZFix";
            this.textBoxMagnetometerZFix.ReadOnly = true;
            this.textBoxMagnetometerZFix.Size = new System.Drawing.Size(67, 20);
            this.textBoxMagnetometerZFix.TabIndex = 11;
            this.textBoxMagnetometerZFix.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxMagnetometerYFix
            // 
            this.textBoxMagnetometerYFix.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxMagnetometerYFix.ForeColor = System.Drawing.Color.White;
            this.textBoxMagnetometerYFix.Location = new System.Drawing.Point(105, 45);
            this.textBoxMagnetometerYFix.Name = "textBoxMagnetometerYFix";
            this.textBoxMagnetometerYFix.ReadOnly = true;
            this.textBoxMagnetometerYFix.Size = new System.Drawing.Size(67, 20);
            this.textBoxMagnetometerYFix.TabIndex = 10;
            this.textBoxMagnetometerYFix.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxMagnetometerXFix
            // 
            this.textBoxMagnetometerXFix.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxMagnetometerXFix.ForeColor = System.Drawing.Color.White;
            this.textBoxMagnetometerXFix.Location = new System.Drawing.Point(105, 19);
            this.textBoxMagnetometerXFix.Name = "textBoxMagnetometerXFix";
            this.textBoxMagnetometerXFix.ReadOnly = true;
            this.textBoxMagnetometerXFix.Size = new System.Drawing.Size(67, 20);
            this.textBoxMagnetometerXFix.TabIndex = 9;
            this.textBoxMagnetometerXFix.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // DataDump
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(760, 182);
            this.ControlBox = false;
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.comboBoxMode);
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "DataDump";
            this.ShowIcon = false;
            this.ShowInTaskbar = false;
            this.Text = "Data Dump";
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ComboBox comboBoxMode;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox textBoxQuaternionZ;
        private System.Windows.Forms.TextBox textBoxQuaternionY;
        private System.Windows.Forms.TextBox textBoxQuaternionX;
        private System.Windows.Forms.TextBox textBoxQuaternionW;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox textBoxYaw;
        private System.Windows.Forms.TextBox textBoxPitch;
        private System.Windows.Forms.TextBox textBoxRoll;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox textBoxAccelerometerZ;
        private System.Windows.Forms.TextBox textBoxAccelerometerY;
        private System.Windows.Forms.TextBox textBoxAccelerometerX;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox textBoxMagnetometerZ;
        private System.Windows.Forms.TextBox textBoxMagnetometerY;
        private System.Windows.Forms.TextBox textBoxMagnetometerX;
        private System.Windows.Forms.TextBox textBoxAccelerometerZFix;
        private System.Windows.Forms.TextBox textBoxAccelerometerYFix;
        private System.Windows.Forms.TextBox textBoxAccelerometerXFix;
        private System.Windows.Forms.TextBox textBoxMagnetometerZFix;
        private System.Windows.Forms.TextBox textBoxMagnetometerYFix;
        private System.Windows.Forms.TextBox textBoxMagnetometerXFix;
    }
}