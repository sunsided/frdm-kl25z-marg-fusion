using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WindowsFormsApplication1
{
    /// <summary>
    /// Class DataEvent.
    /// </summary>
    class DataEvent : EventArgs
    {
        /// <summary>
        /// Gets the data.
        /// </summary>
        /// <value>The data.</value>
        public byte[] Data { get; private set; }

        /// <summary>
        /// Initializes a new instance of the <see cref="DataEvent"/> class.
        /// </summary>
        /// <param name="data">The data.</param>
        public DataEvent(byte[] data)
        {
            Data = data;
        }
    }
}
