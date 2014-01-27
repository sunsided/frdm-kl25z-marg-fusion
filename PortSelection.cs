using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace WindowsFormsApplication1
{
    /// <summary>
    /// Class PortSelection.
    /// </summary>
    public partial class PortSelection : Form
    {
        public string SelectedPort { get; private set; }

        /// <summary>
        /// Initializes a new instance of the <see cref="PortSelection"/> class.
        /// </summary>
        public PortSelection()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Handles the Load event of the PortSelection control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="EventArgs"/> instance containing the event data.</param>
        private void PortSelection_Load(object sender, EventArgs e)
        {
            comboBox1.SelectedValueChanged += (s, args) =>
            {
                SelectedPort = (string)comboBox1.SelectedItem;
            };

            string[] ports = SerialPort.GetPortNames();
            comboBox1.DataSource = ports;
        }

        /// <summary>
        /// Handles the Click event of the buttonReload control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="EventArgs"/> instance containing the event data.</param>
        private void buttonReload_Click(object sender, EventArgs e)
        {
            string[] ports = SerialPort.GetPortNames();
            comboBox1.DataSource = ports;
        }
    }
}
