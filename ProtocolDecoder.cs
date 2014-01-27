using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WindowsFormsApplication1
{
    /// <summary>
    /// Class ProtocolDecoder.
    /// </summary>
    class ProtocolDecoder
    {
        private const int SOH = 1;
        private const int EOT = 4;
        private const int ESC = 27;
        private const int ESC_XOR = 66;
        private const int DA = 218;
        private const int TA = 122;
        
        private int state = 0;
        private int lastdatum = 0;

        private int dataLength = 0;
        private int dataBytesRead = 0;

        private bool escapeDetected = false;

        private byte[] data;

        public event EventHandler<DataEvent> DataReady;

        /// <summary>
        /// Decodifies the specified datum.
        /// </summary>
        /// <param name="datum">The datum.</param>
        public void Decodify(int datum)
        {
            switch (state)
            {
                case 0: // Await preamble
                {
                    if (datum == DA)
                    {
                        //state = 0;
                    }
                    else if ((datum == TA) && (lastdatum == DA))
                    {
                        // preamble detected
                        state = 1;
                        break;
                    }
                    else
                    {
                        //state = 0;

                        //disp('protocol error in state 0');
                        //disp(s.datumsAvailable);
                    }

                    // Remember datum for protocol decoding
                    lastdatum = datum;
                    break;
                }

                case 1: // Await SOH
                {
                    if (datum == SOH)
                    {
                        state = 2;
                    }
                    else
                    {
                        state = 0;
                        Console.WriteLine("protocol error in state 1");
                    }
                    break;
                }

                case 2: // Read length
                {
                    dataLength = datum;
                    dataBytesRead = 0;
                    data = new byte[dataLength];
                    state = 3;
                    break;
                }

                case 3: // Read data datums
                {
                    if (datum == ESC)
                    {
                        escapeDetected = true;
                    }
                    else
                    {
                        // decode escaped byte
                        if (escapeDetected)
                        {
                            datum = datum ^ ESC_XOR;
                            escapeDetected = false;
                        }

                        // store data byte and increment counter
                        data[dataBytesRead++] = (byte)datum;

                        // if all bytes are read, wait for EOT
                        if (dataBytesRead == dataLength)
                        {
                            state = 4;
                        }
                        else
                        {
                            //state = 3;
                        }
                    }
                    break;
                }
                case 4: // Await EOT
                {
                    if (datum == EOT)
                    {
                        state = 0;

                        // raise event
                        var handler = DataReady;
                        if (handler != null)
                        {
                            handler(this, new DataEvent(data));
                        }
                    }
                    else
                    {
                        state = 0;
                        Console.WriteLine("protocol error in state 6");
                    }
                    break;
                }
            }
        }
    }
}
