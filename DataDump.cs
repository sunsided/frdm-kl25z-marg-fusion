using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.Linq;
using System.Text;
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

        /// <summary>
        /// The _lastValues
        /// </summary>
        private readonly Dictionary<TextBox, double> _lastValues = new Dictionary<TextBox, double>();

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

            foreach (var group in Controls.OfType<GroupBox>())
            {
                foreach (var textBox in group.Controls.OfType<TextBox>())
                {
                    textBox.Text = String.Empty;
                    textBox.TextChanged += BoxOnTextChanged;
                    _lastValues[textBox] = Double.NaN;
                }
            }
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
                textBoxRoll.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxRoll, roll));
                textBoxPitch.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxPitch, pitch));
                textBoxYaw.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxYaw, yaw));
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
                textBoxAccelerometerX.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxAccelerometerX, x));
                textBoxAccelerometerY.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxAccelerometerY, y));
                textBoxAccelerometerZ.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxAccelerometerZ, z));

                textBoxAccelerometerXFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", Filter(textBoxAccelerometerXFix, x * 65535));
                textBoxAccelerometerYFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", Filter(textBoxAccelerometerYFix, y * 65535));
                textBoxAccelerometerZFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", Filter(textBoxAccelerometerZFix, z * 65535));
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
                textBoxMagnetometerX.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxMagnetometerX, x));
                textBoxMagnetometerY.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxMagnetometerY, y));
                textBoxMagnetometerZ.Text = String.Format(CultureInfo.InvariantCulture, "{0:0.00000}", Filter(textBoxMagnetometerZ, z));

                textBoxMagnetometerXFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", Filter(textBoxMagnetometerXFix, x * 65535));
                textBoxMagnetometerYFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", Filter(textBoxMagnetometerYFix, y * 65535));
                textBoxMagnetometerZFix.Text = String.Format(CultureInfo.InvariantCulture, "{0:0}", Filter(textBoxMagnetometerZFix, z * 65535));
            };
            Invoke(set);
        }

        /// <summary>
        /// Low-pass filters the values
        /// </summary>
        /// <param name="key">The key.</param>
        /// <param name="value">The value.</param>
        /// <returns>System.Double.</returns>
        private double Filter(TextBox key, double value)
        {
            var oldValue = _lastValues[key];
            if (Double.IsNaN(oldValue) || !checkBoxLowPass.Checked)
            {
                _lastValues[key] = value;
                return value;
            }

            const double alpha = 0.1;
            value = alpha*value + (1 - alpha)*oldValue;
            _lastValues[key] = value;
            return value;
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
                                     _lastValues[textBox] = Double.NaN;
                                 }
                             }
                         };
            Invoke(set);
        }

        /// <summary>
        /// Handles the KeyUp event of the DataDump control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="KeyEventArgs"/> instance containing the event data.</param>
        private void DataDump_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Control && e.KeyCode == Keys.C)
            {
                e.Handled = true;

                var builder = new StringBuilder();

                builder.AppendLine("a = [");
                builder.Append("     ");
                builder.AppendLine(textBoxAccelerometerXFix.Text);
                builder.Append("     ");
                builder.AppendLine(textBoxAccelerometerYFix.Text);
                builder.Append("     ");
                builder.AppendLine(textBoxAccelerometerZFix.Text);
                builder.AppendLine("     ]/65535;");
                builder.AppendLine();

                builder.AppendLine("m = [");
                builder.Append("     ");
                builder.AppendLine(textBoxMagnetometerXFix.Text);
                builder.Append("     ");
                builder.AppendLine(textBoxMagnetometerYFix.Text);
                builder.Append("     ");
                builder.AppendLine(textBoxMagnetometerZFix.Text);
                builder.AppendLine("     ]/65535;");

                Clipboard.SetText(builder.ToString());
                Trace.WriteLine("Copied accelerometer and magnetometer to clipboard.");
            }
        }
    }
}
