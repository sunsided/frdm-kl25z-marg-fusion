using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Linq;
using System.Windows.Forms;

namespace WindowsFormsApplication1
{
    public partial class DataDump : Form
    {
        /// <summary>
        /// Gets the selected mode.
        /// </summary>
        /// <value>The selected mode.</value>
        public byte SelectedMode { get; private set; }

        /// <summary>
        /// Occurs when [selected mode changed].
        /// </summary>
        public event EventHandler SelectedModeChanged;

        public DataDump()
        {
            InitializeComponent();
            DoubleBuffered = true;

            var dataSource = new List<Mode>();
            dataSource.Add(new Mode("Raw sensor data", 0));
            dataSource.Add(new Mode("Angles only", 42));
            dataSource.Add(new Mode("Orientation only", 43));
            dataSource.Add(new Mode("Orientation and angles", 44));

            comboBoxMode.DisplayMember = "Name";
            comboBoxMode.ValueMember = "Value";
            comboBoxMode.DataSource = dataSource;

            textBoxAccelerometerX.TextChanged += BoxOnTextChanged;
            textBoxAccelerometerY.TextChanged += BoxOnTextChanged;
            textBoxAccelerometerZ.TextChanged += BoxOnTextChanged;
            textBoxAccelerometerXFix.TextChanged += BoxOnTextChanged;
            textBoxAccelerometerYFix.TextChanged += BoxOnTextChanged;
            textBoxAccelerometerZFix.TextChanged += BoxOnTextChanged;

            textBoxMagnetometerX.TextChanged += BoxOnTextChanged;
            textBoxMagnetometerY.TextChanged += BoxOnTextChanged;
            textBoxMagnetometerZ.TextChanged += BoxOnTextChanged;
            textBoxMagnetometerXFix.TextChanged += BoxOnTextChanged;
            textBoxMagnetometerYFix.TextChanged += BoxOnTextChanged;
            textBoxMagnetometerZFix.TextChanged += BoxOnTextChanged;

            textBoxRoll.TextChanged += BoxOnTextChanged;
            textBoxPitch.TextChanged += BoxOnTextChanged;
            textBoxYaw.TextChanged += BoxOnTextChanged;

            textBoxQuaternionW.TextChanged += BoxOnTextChanged;
            textBoxQuaternionX.TextChanged += BoxOnTextChanged;
            textBoxQuaternionY.TextChanged += BoxOnTextChanged;
            textBoxQuaternionZ.TextChanged += BoxOnTextChanged;
        }

        private void BoxOnTextChanged(object sender, EventArgs eventArgs)
        {
            var box = (TextBox) sender;
            if (String.IsNullOrWhiteSpace(box.Text))
            {
                box.BackColor = DefaultBackColor;
            }
            else if (box.Text.StartsWith("-"))
            {
                box.BackColor = Color.DarkRed;
            }
            else
            {
                box.BackColor = Color.DarkBlue;
            }
        }

        /// <summary>
        /// Sets the quaternion.
        /// </summary>
        /// <param name="w">The w.</param>
        /// <param name="x">The x.</param>
        /// <param name="y">The y.</param>
        /// <param name="z">The z.</param>
        public void SetQuaternion(double w, double x, double y, double z)
        {
            Action set = delegate
                         {
                             textBoxQuaternionW.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", w);
                             textBoxQuaternionX.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", x);
                             textBoxQuaternionY.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", y);
                             textBoxQuaternionZ.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", z);
                           };
            Invoke(set);
        }

        /// <summary>
        /// Handles the SelectedIndexChanged event of the comboBoxMode control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="EventArgs"/> instance containing the event data.</param>
        private void comboBoxMode_SelectedIndexChanged(object sender, EventArgs e)
        {
            
        }

        /// <summary>
        /// Handles the SelectedValueChanged event of the comboBoxMode control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="EventArgs"/> instance containing the event data.</param>
        private void comboBoxMode_SelectedValueChanged(object sender, EventArgs e)
        {
            // fetch new mode
            var box = (ComboBox) sender;
            var value = box.SelectedValue;
            if (value == null) return;
            var mode = (byte)value;

            // set selected mode
            if (mode == SelectedMode) return;
            SelectedMode = mode;

            // raise event
            var handler = SelectedModeChanged;
            if (handler != null)
            {
                handler(this, EventArgs.Empty);
            }
        }

        /// <summary>
        /// Sets the mode.
        /// </summary>
        /// <param name="defaultMode">The default mode.</param>
        /// <exception cref="System.NotImplementedException"></exception>
        public void SetMode(byte defaultMode)
        {
            comboBoxMode.SelectedValue = defaultMode;
        }

        /// <summary>
        /// Sets the angles.
        /// </summary>
        /// <param name="roll">The roll.</param>
        /// <param name="pitch">The pitch.</param>
        /// <param name="yaw">The yaw.</param>
        /// <exception cref="System.NotImplementedException"></exception>
        public void SetAngles(double roll, double pitch, double yaw)
        {
            Action set = delegate
            {
                textBoxRoll.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", roll);
                textBoxPitch.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", pitch);
                textBoxYaw.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", yaw);
            };
            Invoke(set);
        }

        /// <summary>
        /// Sets the accelerometer.
        /// </summary>
        /// <param name="x">The x.</param>
        /// <param name="y">The y.</param>
        /// <param name="z">The z.</param>
        public void SetAccelerometer(double x, double y, double z)
        {
            Action set = delegate
            {
                textBoxAccelerometerX.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", x);
                textBoxAccelerometerY.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", y);
                textBoxAccelerometerZ.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", z);

                textBoxAccelerometerXFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", x * 65535);
                textBoxAccelerometerYFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", y * 65535);
                textBoxAccelerometerZFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", z * 65535);
            };
            Invoke(set);
        }

        /// <summary>
        /// Sets the magnetometer.
        /// </summary>
        /// <param name="x">The x.</param>
        /// <param name="y">The y.</param>
        /// <param name="z">The z.</param>
        public void SetMagnetometer(double x, double y, double z)
        {
            Action set = delegate
            {
                textBoxMagnetometerX.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", x);
                textBoxMagnetometerY.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", y);
                textBoxMagnetometerZ.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", z);

                textBoxMagnetometerXFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", x * 65535);
                textBoxMagnetometerYFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", y * 65535);
                textBoxMagnetometerZFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", z * 65535);
            };
            Invoke(set);
        }

        /// <summary>
        /// Clears the dumps.
        /// </summary>
        /// <exception cref="System.NotImplementedException"></exception>
        public void ClearDumps()
        {
            Action set = delegate
                         {
                             foreach (var group in Controls.OfType<GroupBox>())
                             {
                                 foreach (var textBox in group.Controls.OfType<TextBox>())
                                 {
                                     textBox.Text = String.Empty;
                                 }
                             }
                         };
            Invoke(set);
        }
    }
}
